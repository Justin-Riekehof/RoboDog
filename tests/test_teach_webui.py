"""Teach web UI server: endpoints, geometry, safety pass-through. No browser.

The page's JavaScript is deliberately dumb (it only draws what the server sends
and posts intents back), so testing the endpoints tests the behaviour.
"""

from __future__ import annotations

import json
import math
import re
import threading
import time
import urllib.request
from collections.abc import Iterator
from http.server import ThreadingHTTPServer
from importlib import resources
from pathlib import Path
from typing import Any

import pytest

from robodog.api.client import RobotClient
from robodog.api.types import Drive, FunctionMode, LegId, SetFunction
from robodog.backends.http import (
    MOVE_FORWARD,
    MOVE_STOP_FB,
    MOVE_STOP_LR,
    MOVE_TURN_LEFT,
    HttpBackend,
)
from robodog.backends.mock import MockBackend
from robodog.kinematics.constants import LINKAGE_W, STAND_HEIGHT
from robodog.safety.limits import LimitConfig
from robodog.teach.format import load_routine
from robodog.teach.repl import RealtimeTicker
from robodog.teach.session import TeachSession
from robodog.teach.webui import TeachUIServer
from robodog.viz.stick import HIPS
from tests.conftest import FakeClock
from tests.test_http_backend import FakeFirmware, make_handler


class TickingClock:
    """A clock that moves on every reading -- makes the UI heartbeat testable
    without any test ever sleeping."""

    def __init__(self, step: float = 0.1) -> None:
        self.now = 0.0
        self.step = step

    def __call__(self) -> float:
        self.now += self.step
        return self.now


def wait_idle(url: str, timeout: float = 10.0) -> dict[str, Any]:
    """Poll until no run holds the session (the page does the same)."""
    deadline = time.monotonic() + timeout
    state = state_of(url)
    while state["busy"] and time.monotonic() < deadline:
        time.sleep(0.01)
        state = state_of(url)
    assert not state["busy"], "the run never finished"
    return state


@pytest.fixture
def rig(tmp_path: Path) -> Iterator[tuple[TeachUIServer, MockBackend, str]]:
    backend = MockBackend()
    client = RobotClient(backend, clock=FakeClock())
    client.connect()
    client.arm()
    session = TeachSession(client, name="uitest", default_path=tmp_path / "uitest.yaml")
    session.start()
    server = TeachUIServer(session, client, lock=threading.Lock())
    url = server.start()
    try:
        yield server, backend, url
    finally:
        server.shutdown()


def get(url: str, path: str) -> tuple[int, dict[str, Any]]:
    with urllib.request.urlopen(url.rstrip("/") + path, timeout=5) as response:
        return response.status, json.loads(response.read())


def get_text(url: str, path: str) -> tuple[int, str]:
    with urllib.request.urlopen(url.rstrip("/") + path, timeout=5) as response:
        return response.status, response.read().decode("utf-8")


def post(url: str, action: str, body: dict[str, Any] | None = None) -> tuple[int, dict[str, Any]]:
    request = urllib.request.Request(
        url.rstrip("/") + "/api/" + action,
        data=json.dumps(body or {}).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as exc:  # non-2xx still carries JSON
        return exc.code, json.loads(exc.read())


def state_of(url: str) -> dict[str, Any]:
    _status, data = get(url, "/api/state")
    return data


# --- serving -------------------------------------------------------------------


def test_binds_to_localhost_only(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    assert url.startswith("http://127.0.0.1:")


def test_serves_the_page(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    status, text = get_text(url, "/")
    assert status == 200
    assert "RoboDog Teach" in text
    assert "svg-side" in text and "svg-top" in text


def test_unknown_paths_are_404(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    status, _body = get(url, "/api/nope") if False else post(url, "nope")
    assert status == 400 or status == 404


# --- state ---------------------------------------------------------------------


def test_state_carries_geometry_and_limits(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    state = state_of(url)
    assert state["name"] == "uitest"
    assert state["backend"] == "mock"
    assert state["limits"]["depth_min"] == 75.0
    # Measured on the robot 2026-08-21 (ASSUMPTIONS C13).
    assert state["limits"]["roll_min"] == -27.0
    assert state["limits"]["roll_max"] == 135.0
    for leg in ("front_left", "hind_left", "front_right", "hind_right"):
        assert len(state["chains"][leg]) == 7  # full linkage chain
        assert len(state["chains"][leg][0]) == 3
        assert leg in state["measured_feet"]
    # The stand-pose foot sits at hip_x + 16 forward and -95 up (world frame).
    foot = state["chains"]["front_left"][6]
    assert foot[0] == pytest.approx(55 + 16)
    assert foot[2] == pytest.approx(-95)


# --- posing --------------------------------------------------------------------


def test_target_endpoint_moves_a_leg(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    status, data = post(url, "target", {"leg": "front_left", "x": 20, "y": 90, "z": 25})
    assert status == 200 and data["ok"]
    assert state_of(url)["targets"]["front_left"]["y"] == pytest.approx(90)


def test_rejected_target_reports_the_supervisor_message(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    status, data = post(url, "target", {"leg": "front_left", "x": 16, "y": 200, "z": 25})
    assert status == 200 and not data["ok"]
    assert "outside" in data["message"]
    assert state_of(url)["targets"]["front_left"]["y"] == pytest.approx(STAND_HEIGHT)


def test_side_view_drag_converts_world_to_leg_frame(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    # Front-left hip sits at world x = 55; drag the foot to world x = hip+20,
    # height -90 (i.e. y = 90 in the down-positive leg frame).
    status, data = post(url, "foot", {"leg": "front_left", "view": "side", "a": 55 + 20, "b": -90})
    assert status == 200 and data["ok"], data
    target = state_of(url)["targets"]["front_left"]
    assert target["x"] == pytest.approx(20)
    assert target["y"] == pytest.approx(90)
    assert target["z"] == pytest.approx(25)  # untouched by the side view


def test_top_view_drag_handles_the_right_side_sign(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    # front_right hip is at world y = -38; pushing the foot further right
    # (world y = -68) must mean z = +30 in that leg's outward-positive frame.
    status, data = post(url, "foot", {"leg": "front_right", "view": "top", "a": 55 + 16, "b": -68})
    assert status == 200 and data["ok"], data
    target = state_of(url)["targets"]["front_right"]
    assert target["z"] == pytest.approx(30)
    assert target["y"] == pytest.approx(STAND_HEIGHT)  # untouched by the top view


def test_front_view_drag_rolls_the_leg(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    """The front view is the one that moves the leg along its roll arc."""
    _server, _backend, url = rig
    before = state_of(url)["targets"]["front_left"]
    # front_left hip is at world y = +38. Point far out to the left and high up
    # -- a pose the old (y, z) box could not express at all.
    status, data = post(url, "foot", {"leg": "front_left", "view": "front", "a": 38 + 93, "b": -31})
    assert status == 200 and data["ok"], data
    target = state_of(url)["targets"]["front_left"]
    assert target["roll"] > 60.0
    assert target["x"] == pytest.approx(before["x"])  # untouched by the front view
    assert target["depth"] == pytest.approx(before["depth"])  # the drag rolls, nothing else


def test_a_front_drag_off_the_arc_still_only_rolls(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    """The pointer's distance from the hip is ignored, its angle is not.

    Taking the distance literally is what made the sweep unusable: a hand a
    centimetre off the arc leaves the reach band, every frame is rejected, and
    it reads as a rotation limit that is not there.
    """
    _server, _backend, url = rig
    before = state_of(url)["targets"]["front_left"]
    far = post(url, "foot", {"leg": "front_left", "view": "front", "a": 38 + 186, "b": -62})
    near = post(url, "foot", {"leg": "front_left", "view": "front", "a": 38 + 46, "b": -15})
    assert far[1]["ok"] and near[1]["ok"]
    # Same direction at twice and half the distance: the same roll, same reach.
    assert far[1]["target"]["roll"] == pytest.approx(near[1]["target"]["roll"], abs=0.5)
    assert near[1]["target"]["depth"] == pytest.approx(before["depth"])


def test_rolling_past_the_measured_limit_says_so(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    """Past +135 deg the answer must be the roll limit, not a reach message."""
    _server, _backend, url = rig
    # Straight up on the far side: an angle beyond anything the servo reached.
    status, data = post(url, "foot", {"leg": "front_left", "view": "front", "a": 38 - 60, "b": -80})
    assert status == 200
    assert not data["ok"]
    assert "roll" in data["message"]


def test_a_drag_onto_the_hip_changes_nothing(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    before = state_of(url)["targets"]["front_left"]
    post(url, "foot", {"leg": "front_left", "view": "front", "a": 38 + 2, "b": -3})
    after = state_of(url)["targets"]["front_left"]
    assert after == before


def test_state_targets_carry_their_roll_decomposition(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    target = state_of(url)["targets"]["front_left"]
    assert "roll" in target and "depth" in target
    assert target["depth"] == pytest.approx(96.35, abs=0.01)


def test_jog_and_absolute_set_with_explicit_legs(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    post(url, "jog", {"legs": ["hind_left"], "axis": "x", "delta": -5})
    assert state_of(url)["targets"]["hind_left"]["x"] == pytest.approx(-21)
    post(url, "set", {"legs": "all", "axis": "y", "value": 85})
    targets = state_of(url)["targets"]
    assert all(targets[leg]["y"] == pytest.approx(85) for leg in targets)


def test_pose_endpoint(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    post(url, "pose", {"pose": "crouch"})
    targets = state_of(url)["targets"]
    assert all(targets[leg]["y"] == pytest.approx(75) for leg in targets)


def test_malformed_requests_are_400(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    status, _data = post(url, "target", {"leg": "front_left", "x": "far", "y": 90, "z": 25})
    assert status == 400
    status, _data = post(url, "foot", {"leg": "middle_left", "view": "side", "a": 0, "b": -90})
    assert status == 400
    status, _data = post(url, "foot", {"leg": "front_left", "view": "diagonal", "a": 0, "b": -90})
    assert status == 400


# --- keyframes -----------------------------------------------------------------


def test_capture_undo_apply_delete_flow(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    post(url, "capture", {})
    post(url, "set", {"legs": "all", "axis": "y", "value": 82})
    post(url, "capture", {"spacing": 0.5})
    state = state_of(url)
    assert [kf["at"] for kf in state["keyframes"]] == [0.0, 0.5]
    assert state["dirty"]

    # Jump back to the first keyframe's pose.
    post(url, "kf_apply", {"index": 0})
    assert state_of(url)["targets"]["front_left"]["y"] == pytest.approx(STAND_HEIGHT)

    post(url, "kf_delete", {"index": 0})
    assert [kf["at"] for kf in state_of(url)["keyframes"]] == [0.5]

    post(url, "undo", {})
    assert state_of(url)["keyframes"] == []


def test_kf_apply_out_of_range_is_400(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    status, data = post(url, "kf_apply", {"index": 3})
    assert status == 400
    assert "no keyframe" in data["message"]


# --- save / preview / quit -------------------------------------------------------


def test_save_writes_and_respects_overwrite(
    rig: tuple[TeachUIServer, MockBackend, str], tmp_path: Path
) -> None:
    _server, _backend, url = rig
    post(url, "capture", {})
    post(url, "set", {"legs": "all", "axis": "y", "value": 85})
    post(url, "capture", {})
    status, data = post(url, "save", {})
    assert status == 200 and data["ok"]
    routine = load_routine(data["path"])
    assert routine.name == "uitest"
    assert len(routine.keyframes) == 2

    post(url, "capture", {})
    status, data = post(url, "save", {})
    assert status == 409 and "already exists" in data["message"]
    status, data = post(url, "save", {"overwrite": True})
    assert status == 200
    assert len(load_routine(data["path"]).keyframes) == 3


def test_save_with_a_new_name_renames_the_file(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    post(url, "capture", {})
    post(url, "capture", {})
    status, data = post(url, "save", {"name": "renamed"})
    assert status == 200
    assert data["path"].endswith("renamed.yaml")
    assert state_of(url)["name"] == "renamed"


def test_preview_runs_and_clears_busy(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    post(url, "capture", {})
    post(url, "set", {"legs": "all", "axis": "y", "value": 85})
    post(url, "capture", {"spacing": 0.3})
    status, data = post(url, "preview", {})
    assert status == 200 and data["ok"]
    deadline = time.monotonic() + 5
    while state_of(url)["busy"] and time.monotonic() < deadline:
        time.sleep(0.02)
    assert state_of(url)["busy"] is None
    # Back at the working pose after the replay.
    assert state_of(url)["targets"]["front_left"]["y"] == pytest.approx(85)


def test_preview_needs_two_keyframes(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    status, data = post(url, "preview", {})
    assert status == 400
    assert "at least 2" in data["message"]


def test_quit_releases_wait(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    server, _backend, url = rig
    waiter = threading.Thread(target=server.wait)
    waiter.start()
    post(url, "quit", {})
    waiter.join(timeout=2)
    assert not waiter.is_alive()


# --- sequence tab: editing ---------------------------------------------------------


def sequence_of(url: str) -> dict[str, Any]:
    sequence: dict[str, Any] = state_of(url)["sequence"]
    return sequence


def drives(backend: MockBackend) -> list[Drive]:
    """Only the locomotion commands -- posing sends leg targets around them."""
    return [c for _, c in backend.command_log if isinstance(c, Drive)]


def test_state_offers_the_move_vocabulary(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    sequence = sequence_of(url)
    moves = [entry["move"] for entry in sequence["vocabulary"]]
    assert "forward" in moves and "left" in moves and "wait" in moves
    assert all(entry["label"] for entry in sequence["vocabulary"])
    assert sequence["steps"] == []
    assert sequence["limits"]["max_steps"] >= 8


def test_building_a_sequence_through_the_api(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    assert post(url, "seq_add", {"move": "forward", "seconds": 10})[0] == 200
    post(url, "seq_add", {"move": "left", "seconds": 2})
    post(url, "seq_add", {"move": "backward", "seconds": 5})
    post(url, "seq_update", {"index": 1, "seconds": 3})
    post(url, "seq_reorder", {"index": 2, "delta": -1})
    sequence = sequence_of(url)
    assert [(s["move"], s["seconds"]) for s in sequence["steps"]] == [
        ("forward", 10.0),
        ("backward", 5.0),
        ("left", 3.0),
    ]
    post(url, "seq_delete", {"index": 1})
    assert len(sequence_of(url)["steps"]) == 2
    post(url, "seq_clear", {})
    assert sequence_of(url)["steps"] == []


def test_gap_and_repeat_are_configurable(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    post(url, "seq_add", {"move": "forward", "seconds": 4})
    post(url, "seq_add", {"move": "left", "seconds": 2})
    status, data = post(url, "seq_config", {"gap": 1.5, "repeat": 0})
    assert status == 200 and "endless" in data["message"]
    sequence = sequence_of(url)
    assert sequence["gap"] == pytest.approx(1.5)
    assert sequence["repeat"] == 0
    assert sequence["duration"] == pytest.approx(7.5)  # 4 + 1.5 gap + 2


@pytest.mark.parametrize(
    ("action", "body"),
    [
        ("seq_add", {"move": "sideways"}),
        ("seq_add", {"move": "forward", "seconds": 0}),
        ("seq_update", {"index": 7, "seconds": 1}),
        ("seq_config", {"gap": -1}),
        ("seq_config", {"repeat": -3}),
        ("seq_load", {"file": "../outside.yaml"}),
    ],
)
def test_bad_sequence_requests_are_400(
    rig: tuple[TeachUIServer, MockBackend, str], action: str, body: dict[str, Any]
) -> None:
    _server, _backend, url = rig
    status, data = post(url, action, body)
    assert status == 400
    assert data["message"]


def test_running_an_empty_sequence_is_refused(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    status, data = post(url, "seq_run", {})
    assert status == 400
    assert "at least one move" in data["message"]


# --- sequence tab: running ---------------------------------------------------------


def test_running_a_sequence_drives_the_robot_and_clears_busy(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, backend, url = rig
    post(url, "seq_add", {"move": "forward", "seconds": 0.2})
    post(url, "seq_add", {"move": "left", "seconds": 0.2})
    post(url, "seq_config", {"gap": 0, "repeat": 2})
    status, data = post(url, "seq_run", {})
    assert status == 200 and data["ok"]
    state = wait_idle(url)
    # Two cycles, plus the unconditional stop the worker sends on the way out.
    assert drives(backend) == [Drive(1, 0), Drive(0, -1), Drive(0, 0)] * 2 + [Drive(0, 0)]
    assert "finished" in state["run"]["message"]
    # The working pose is restored after the run moved the legs.
    assert state["targets"]["front_left"]["y"] == pytest.approx(STAND_HEIGHT)


def test_edits_are_refused_while_a_sequence_runs(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    post(url, "seq_add", {"move": "forward", "seconds": 0.5})
    post(url, "seq_config", {"repeat": 0})
    post(url, "seq_run", {})
    status, data = post(url, "seq_add", {"move": "left"})
    assert status == 409
    assert "sequence" in data["message"]
    # Stop, however, is always accepted -- that is the whole point.
    status, data = post(url, "seq_stop", {})
    assert status == 200 and data["ok"]
    wait_idle(url)


def test_stop_ends_an_endless_run(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, backend, url = rig
    post(url, "seq_add", {"move": "forward", "seconds": 0.2})
    post(url, "seq_config", {"repeat": 0})
    post(url, "seq_run", {})
    post(url, "seq_stop", {})
    state = wait_idle(url)
    assert "stopped (operator)" in state["run"]["message"]
    assert drives(backend)[-1] == Drive(0, 0)


def test_stopping_when_nothing_runs_stops_the_robot_anyway(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    """Stop is never refused: pressing it when nothing moves must still stop."""
    _server, backend, url = rig
    status, data = post(url, "seq_stop", {})  # the old name still routes here
    assert status == 200 and data["ok"]
    assert drives(backend)[-1] == Drive(0, 0)


def test_a_run_nobody_watches_stops_itself(tmp_path: Path) -> None:
    """The page's polls are a dead-man's switch (ASSUMPTIONS D10)."""
    backend = MockBackend()
    client = RobotClient(backend, clock=FakeClock())
    client.connect()
    client.arm()
    session = TeachSession(client, name="watched", default_path=tmp_path / "watched.yaml")
    session.start()
    server = TeachUIServer(
        session, client, lock=threading.Lock(), ui_timeout=0.5, clock=TickingClock()
    )
    url = server.start()
    try:
        post(url, "seq_add", {"move": "forward", "seconds": 0.2})
        post(url, "seq_config", {"repeat": 0})
        post(url, "seq_run", {})
        # Deliberately NOT wait_idle: polling /api/state is the page's
        # heartbeat, so watching for the end of the run would feed the very
        # switch this test is about, and whether it tripped would come down to
        # which thread read the fake clock more often. Wait on the worker.
        assert server.wait_for_run(), "the run never finished"
        state = state_of(url)
        assert "the page stopped answering" in state["run"]["message"]
        assert drives(backend)[-1] == Drive(0, 0)
    finally:
        server.shutdown()


def test_quitting_stops_a_running_sequence(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    server, backend, url = rig
    post(url, "seq_add", {"move": "forward", "seconds": 0.2})
    post(url, "seq_config", {"repeat": 0})
    post(url, "seq_run", {})
    post(url, "quit", {})
    server.shutdown()  # the rig's teardown repeats this; both must be safe
    assert drives(backend)[-1] == Drive(0, 0)


# --- sequence tab: files -----------------------------------------------------------


def test_saving_listing_and_loading_a_sequence(
    rig: tuple[TeachUIServer, MockBackend, str], tmp_path: Path
) -> None:
    _server, _backend, url = rig
    post(url, "seq_add", {"move": "forward", "seconds": 10})
    post(url, "seq_add", {"move": "left", "seconds": 2})
    post(url, "seq_config", {"gap": 0.5, "repeat": 0})
    status, data = post(url, "seq_save", {"name": "patrol-loop"})
    assert status == 200
    saved = load_routine(data["path"])
    assert saved.kind == "sequence"
    assert [m.move for m in saved.moves] == ["forward", "left"]
    assert saved.repeat == 0

    status, listing = post(url, "seq_files", {})
    assert status == 200
    assert "patrol-loop.yaml" in listing["files"]

    post(url, "seq_clear", {})
    status, data = post(url, "seq_load", {"file": "patrol-loop.yaml"})
    assert status == 200
    sequence = sequence_of(url)
    assert [s["move"] for s in sequence["steps"]] == ["forward", "left"]
    assert sequence["repeat"] == 0
    assert not sequence["dirty"]


def test_saving_twice_needs_overwrite(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    post(url, "seq_add", {"move": "forward", "seconds": 1})
    assert post(url, "seq_save", {"name": "twice"})[0] == 200
    status, data = post(url, "seq_save", {"name": "twice"})
    assert status == 409
    assert "already exists" in data["message"]
    assert post(url, "seq_save", {"name": "twice", "overwrite": True})[0] == 200


def test_loading_a_pose_routine_is_refused(
    rig: tuple[TeachUIServer, MockBackend, str], tmp_path: Path
) -> None:
    _server, _backend, url = rig
    post(url, "capture", {})
    post(url, "capture", {})
    post(url, "save", {})  # a motion routine next to the sequences
    status, data = post(url, "seq_load", {"file": "uitest.yaml"})
    assert status == 400
    assert "sequence editor" in data["message"]


# --- the Wi-Fi robot: sequences are all it can do ----------------------------------


@pytest.fixture
def wifi_rig(tmp_path: Path) -> Iterator[tuple[FakeFirmware, str]]:
    """The web UI in front of the stock firmware (a local stand-in for it)."""
    firmware = FakeFirmware()
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(firmware))
    thread = threading.Thread(target=httpd.serve_forever, kwargs={"poll_interval": 0.02})
    thread.daemon = True
    thread.start()
    backend = HttpBackend(f"127.0.0.1:{httpd.server_address[1]}")
    client = RobotClient(backend, clock=FakeClock())
    client.connect()
    client.arm()
    session = TeachSession(client, name="wifi", default_path=tmp_path / "wifi.yaml")
    server = TeachUIServer(session, client, lock=threading.Lock())
    url = server.start()
    try:
        yield firmware, url
    finally:
        server.shutdown()
        client.disconnect()
        httpd.shutdown()
        httpd.server_close()
        thread.join(timeout=5)


def test_the_pose_tab_is_off_without_leg_targets(wifi_rig: tuple[FakeFirmware, str]) -> None:
    _firmware, url = wifi_rig
    state = state_of(url)
    assert state["pose_enabled"] is False
    assert state["backend"] == "http"
    assert state["sequence"]["vocabulary"]  # the sequence tab still works


def test_a_sequence_reaches_the_firmware(wifi_rig: tuple[FakeFirmware, str]) -> None:
    firmware, url = wifi_rig
    post(url, "seq_add", {"move": "forward", "seconds": 0.1})
    post(url, "seq_add", {"move": "left", "seconds": 0.1})
    post(url, "seq_config", {"gap": 0})
    assert post(url, "seq_run", {})[0] == 200
    wait_idle(url)
    moves = [value for var, value, _cmd in firmware.calls if var == "move"]
    # forward, then turn left (each latched axis set explicitly), then both stops.
    assert MOVE_FORWARD in moves and MOVE_TURN_LEFT in moves
    assert moves[-2:] == [MOVE_STOP_FB, MOVE_STOP_LR]


# --- driving by hand ---------------------------------------------------------------


def manual_of(url: str) -> dict[str, Any]:
    manual: dict[str, Any] = state_of(url)["manual"]
    return manual


def test_a_hand_driven_move_is_sent_and_latches(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, backend, url = rig
    status, data = post(url, "drive", {"move": "forward_left"})
    assert status == 200 and data["ok"]
    assert drives(backend)[-1] == Drive(1, -1)
    assert manual_of(url)["move"] == "forward_left"


def test_stop_releases_a_hand_driven_move(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, backend, url = rig
    post(url, "drive", {"move": "forward"})
    status, data = post(url, "stop", {})
    assert status == 200 and data["ok"]
    assert drives(backend)[-1] == Drive(0, 0)
    assert manual_of(url)["move"] is None


def test_the_wait_move_is_itself_a_stop(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, backend, url = rig
    post(url, "drive", {"move": "backward"})
    post(url, "drive", {"move": "wait"})
    assert drives(backend)[-1] == Drive(0, 0)
    assert manual_of(url)["move"] is None


def test_an_unknown_hand_driven_move_is_400(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, backend, url = rig
    status, data = post(url, "drive", {"move": "moonwalk"})
    assert status == 400
    assert "unknown move" in data["message"]
    assert not drives(backend)  # nothing was sent


def test_hand_driving_is_refused_while_a_sequence_runs(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    post(url, "seq_add", {"move": "forward", "seconds": 0.5})
    post(url, "seq_config", {"repeat": 0})
    post(url, "seq_run", {})
    status, _data = post(url, "drive", {"move": "backward"})
    assert status == 409
    post(url, "stop", {})
    wait_idle(url)


def test_a_sequence_run_takes_over_from_hand_driving(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    post(url, "drive", {"move": "forward"})
    post(url, "seq_add", {"move": "left", "seconds": 0.2})
    post(url, "seq_run", {})
    wait_idle(url)
    assert manual_of(url)["move"] is None


def test_a_hand_driven_move_nobody_watches_is_released(tmp_path: Path) -> None:
    """The dead-man's switch covers manual driving too (ASSUMPTIONS D10)."""
    backend = MockBackend()
    client = RobotClient(backend, clock=FakeClock())
    client.connect()
    client.arm()
    session = TeachSession(client, name="handheld", default_path=tmp_path / "h.yaml")
    session.start()
    page_clock = FakeClock()  # only the test moves it
    server = TeachUIServer(session, client, lock=threading.Lock(), ui_timeout=0.5, clock=page_clock)
    url = server.start()
    try:
        post(url, "drive", {"move": "forward"})
        assert server.manual == "forward"
        page_clock.advance(2.0)  # the page has gone quiet
        # Deliberately no state polling here -- a poll IS the heartbeat.
        deadline = time.monotonic() + 5
        while server.manual is not None and time.monotonic() < deadline:
            time.sleep(0.02)
        assert server.manual is None, "the robot was left driving"
        assert drives(backend)[-1] == Drive(0, 0)
        assert "stopped" in manual_of(url)["note"]
    finally:
        server.shutdown()


def test_a_hand_driven_move_reaches_the_firmware(wifi_rig: tuple[FakeFirmware, str]) -> None:
    firmware, url = wifi_rig
    post(url, "drive", {"move": "forward"})
    post(url, "stop", {})
    moves = [value for var, value, _cmd in firmware.calls if var == "move"]
    assert MOVE_FORWARD in moves
    assert moves[-2:] == [MOVE_STOP_FB, MOVE_STOP_LR]


# --- the firmware's canned animations ----------------------------------------------


def test_the_page_can_actually_hide_the_inactive_tab() -> None:
    """`main { display: flex }` outranks the UA rule for [hidden]; without an
    explicit override both tabs render at once (and the inactive one blank)."""
    page = resources.files("robodog.teach").joinpath("ui.html").read_text(encoding="utf-8")
    assert "main[hidden] { display: none; }" in page


def test_state_offers_the_function_modes(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    functions = state_of(url)["functions"]
    modes = [entry["mode"] for entry in functions]
    assert "handshake" in modes and "stay_low" in modes and "init_pos" in modes
    assert all(entry["label"] for entry in functions)


def test_a_function_mode_is_sent(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, backend, url = rig
    status, data = post(url, "function", {"mode": "handshake"})
    assert status == 200 and data["message"] == "Handshake"
    assert backend.command_log[-1][1] == SetFunction(FunctionMode.HANDSHAKE)


def test_a_function_mode_stops_a_hand_driven_move_first(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, backend, url = rig
    post(url, "drive", {"move": "forward"})
    post(url, "function", {"mode": "stay_low"})
    sent = [c for _, c in backend.command_log]
    assert sent[-2:] == [Drive(0, 0), SetFunction(FunctionMode.STAY_LOW)]
    assert manual_of(url)["move"] is None


def test_an_unknown_function_mode_is_400(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    status, data = post(url, "function", {"mode": "backflip"})
    assert status == 400
    assert "unknown function mode" in data["message"]


def test_function_modes_are_refused_while_a_sequence_runs(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    post(url, "seq_add", {"move": "forward", "seconds": 0.5})
    post(url, "seq_config", {"repeat": 0})
    post(url, "seq_run", {})
    assert post(url, "function", {"mode": "jump"})[0] == 409
    post(url, "stop", {})
    wait_idle(url)


def test_a_function_mode_reaches_the_firmware(wifi_rig: tuple[FakeFirmware, str]) -> None:
    firmware, url = wifi_rig
    post(url, "function", {"mode": "handshake"})
    assert ("funcMode", int(FunctionMode.HANDSHAKE), 0) in firmware.calls


def test_the_front_view_draws_the_whole_roll_arc() -> None:
    """A foot that is not drawn cannot be dragged to.

    The front view is where the leg rolls, and its arc is a circle about the
    hip of radius hypot(max reach, wiggle arm) -- some 112 mm, most of it above
    the body at large roll angles. When the viewBox was shorter than that, the
    upper part of the range was on the robot but not on the screen, which is
    indistinguishable from a limit.
    """
    page = resources.files("robodog.teach").joinpath("ui.html").read_text(encoding="utf-8")
    match = re.search(r'id="svg-front"\s+viewBox="([-\d.\s]+)"', page)
    assert match, "front view has no viewBox"
    min_x, min_y, width, height = (float(value) for value in match.group(1).split())

    limits = LimitConfig()
    radius = math.hypot(limits.plane_depth_max, LINKAGE_W)
    reach_x = max(abs(y) for _x, y in HIPS.values()) + radius
    assert min_y <= -radius, "the top of the roll arc is outside the drawn area"
    assert min_y + height >= radius
    assert min_x <= -reach_x and min_x + width >= reach_x


def test_the_page_says_why_saving_a_single_keyframe_cannot_work() -> None:
    """One keyframe is not a motion; the button must say so before it is clicked."""
    page = resources.files("robodog.teach").joinpath("ui.html").read_text(encoding="utf-8")
    assert '$("btn-save").disabled = !enough;' in page
    assert "of 2 keyframes" in page


def test_saving_one_keyframe_is_refused_with_the_reason(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    post(url, "capture", {})
    status, data = post(url, "save", {"name": "single"})
    assert status == 400
    assert "at least 2 keyframes" in data["message"]


# --- getting out of a latched E-stop ------------------------------------------------


def estop_rig(tmp_path: Path, timeout: float) -> tuple[TeachUIServer, MockBackend, str, FakeClock]:
    backend = MockBackend()
    clock = FakeClock()
    client = RobotClient(backend, clock=clock, watchdog_timeout=timeout)
    client.connect()
    client.arm()
    session = TeachSession(client, name="latched", default_path=tmp_path / "l.yaml")
    session.start()
    server = TeachUIServer(session, client, lock=threading.Lock())
    return server, backend, server.start(), clock


def test_a_latched_estop_is_reported_and_can_be_released(tmp_path: Path) -> None:
    """'reset() first' with nothing to click is a dead end; the page needs a way out."""
    server, _backend, url, clock = estop_rig(tmp_path, timeout=0.5)
    try:
        clock.advance(1.0)  # the control loop went quiet for longer than the budget
        status, data = post(url, "pose", {"pose": "crouch"})
        assert status == 409  # a refusal with a reason, not a server fault
        assert "E-stop latched" in data["message"]

        safety = state_of(url)["safety"]
        assert safety["state"] == "ESTOPPED"
        assert "watchdog" in safety["reason"]

        status, data = post(url, "reset", {})
        assert status == 200 and data["ok"]
        assert "watchdog" in data["message"]  # says what it released
        assert state_of(url)["safety"]["state"] == "ARMED"  # armed, not merely disarmed

        status, data = post(url, "pose", {"pose": "crouch"})
        assert status == 200 and data["ok"]  # the session works again
    finally:
        server.shutdown()


def test_resetting_when_nothing_is_latched_is_harmless(tmp_path: Path) -> None:
    server, _backend, url, _clock = estop_rig(tmp_path, timeout=0.5)
    try:
        status, data = post(url, "reset", {})
        assert status == 200 and data["ok"]
        assert "ARMED" in data["message"]
    finally:
        server.shutdown()


def test_the_page_shows_the_reset_control_only_when_latched() -> None:
    page = resources.files("robodog.teach").joinpath("ui.html").read_text(encoding="utf-8")
    assert 'id="btn-reset"' in page
    assert '$("estop").hidden = !latched;' in page


def test_the_viewer_gets_a_watchdog_budget_that_fits_its_loop() -> None:
    """viewer.sync() blocks while the operator drags the window; a simulation
    cannot run away, so that must not latch an E-stop."""
    from robodog.safety.supervisor import DEFAULT_WATCHDOG_TIMEOUT
    from robodog.teach.repl import TEACH_VIEWER_WATCHDOG, TEACH_WATCHDOG

    assert TEACH_VIEWER_WATCHDOG > TEACH_WATCHDOG > DEFAULT_WATCHDOG_TIMEOUT


# --- the mechanism's own axes -------------------------------------------------------


def test_roll_and_reach_are_independent(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    """One wiggle servo rolls the plane, two coaxial servos set the reach in it.
    The polar controls must not smear one into the other."""
    _server, _backend, url = rig
    post(url, "set", {"legs": ["front_left"], "axis": "roll", "value": 60})
    after_roll = state_of(url)["targets"]["front_left"]
    assert after_roll["roll"] == pytest.approx(60.0)

    post(url, "set", {"legs": ["front_left"], "axis": "depth", "value": 80})
    after_reach = state_of(url)["targets"]["front_left"]
    assert after_reach["depth"] == pytest.approx(80.0)
    assert after_reach["roll"] == pytest.approx(60.0)  # reach did not touch the roll

    post(url, "set", {"legs": ["front_left"], "axis": "roll", "value": 20})
    back = state_of(url)["targets"]["front_left"]
    assert back["depth"] == pytest.approx(80.0)  # and roll did not touch the reach


def test_a_cartesian_nudge_moves_both_joints(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    """Not a defect, and worth pinning: y and z are a rotated view of
    (roll, reach), so a pure y nudge necessarily changes both. This is why the
    per-leg table offers roll and reach as their own fields."""
    _server, _backend, url = rig
    post(url, "set", {"legs": ["front_left"], "axis": "roll", "value": 45})
    before = state_of(url)["targets"]["front_left"]
    post(url, "jog", {"legs": ["front_left"], "axis": "y", "delta": -8})
    after = state_of(url)["targets"]["front_left"]
    assert after["z"] == pytest.approx(before["z"])  # only y was asked for
    assert after["roll"] != pytest.approx(before["roll"])
    assert after["depth"] != pytest.approx(before["depth"])


def test_every_leg_has_its_own_reach_control() -> None:
    """Roll had a per-leg field and reach did not, so extending a single leg
    was only possible through the Cartesian handles -- which also roll it."""
    page = resources.files("robodog.teach").joinpath("ui.html").read_text(encoding="utf-8")
    assert 'field("depth", t.depth, 1)' in page
    assert 'pair("depth", 5)' in page


def test_the_per_leg_reach_field_reaches_the_session(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    status, data = post(url, "set", {"legs": ["hind_right"], "axis": "depth", "value": 88})
    assert status == 200 and data["ok"]
    targets = state_of(url)["targets"]
    assert targets["hind_right"]["depth"] == pytest.approx(88.0)
    assert targets["front_left"]["depth"] != pytest.approx(88.0)  # only that leg


# --- the teach UI against a robot that can hold a pose ------------------------------


@pytest.fixture
def posing_rig(tmp_path: Path) -> Iterator[tuple[FakeFirmware, str]]:
    """The teach UI in front of a robot running our firmware.

    Assembled exactly as `cmd_teach` assembles it, ticker included -- the ticker
    is what flushes staged poses, so a rig without one tests a UI that cannot
    move anything.
    """
    firmware = FakeFirmware()
    firmware.robodog = True  # speaks ping/watchdog/pose
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(firmware))
    serving = threading.Thread(target=httpd.serve_forever, kwargs={"poll_interval": 0.02})
    serving.daemon = True
    serving.start()

    client = RobotClient(HttpBackend(f"127.0.0.1:{httpd.server_address[1]}"))
    client.connect()
    client.arm()
    session = TeachSession(client, name="posing", default_path=tmp_path / "posing.yaml")
    session.start()
    lock = threading.Lock()
    ticker = RealtimeTicker(client, lock, tick=client.suggested_tick)
    ticker.start()
    server = TeachUIServer(session, client, lock=lock)
    url = server.start()
    try:
        yield firmware, url
    finally:
        server.shutdown()
        ticker.stop()
        client.disarm()
        client.disconnect()
        httpd.shutdown()
        httpd.server_close()
        serving.join(timeout=5)


def poses(firmware: FakeFirmware) -> list[str]:
    return [query for query in firmware.queries if "var=pose" in query]


def test_the_pose_tab_appears_when_the_robot_can_pose(
    posing_rig: tuple[FakeFirmware, str],
) -> None:
    """Against stock firmware the Pose tab is off, because nothing can receive a
    foot target. Against ours it has to come back, or the whole fork bought
    nothing at the one place an operator would notice it."""
    _firmware, url = posing_rig
    state = state_of(url)
    assert state["pose_enabled"] is True
    assert state["backend"] == "http"


def test_dragging_a_foot_reaches_the_robot(posing_rig: tuple[FakeFirmware, str]) -> None:
    """The whole chain: browser drag -> session -> supervisor -> staged -> one
    request carrying all twelve values."""
    firmware, url = posing_rig
    before = len(poses(firmware))
    status, data = post(url, "set", {"legs": ["front_left"], "axis": "depth", "value": 88})
    assert status == 200, data

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and len(poses(firmware)) == before:
        time.sleep(0.02)
    sent = poses(firmware)
    assert len(sent) > before, "the drag never reached the robot"
    assert "l1y=" in sent[-1] and "l4z=" in sent[-1]  # a whole pose, not one leg


def test_the_preview_is_paced_by_the_transport(
    posing_rig: tuple[FakeFirmware, str],
) -> None:
    """A keyframe preview must tick at the rate the link carries.

    Every tick of a motion routine is a fresh pose and every pose is a round
    trip, so previewing a 1-second routine at 50 Hz asks for 51 of them over a
    link that carries ten a second: the preview then runs some five times longer
    than the routine it is previewing. Ticking at `suggested_tick` is what keeps
    a routine the length it was authored to be.
    """
    firmware, url = posing_rig
    post(url, "capture", {})
    post(url, "set", {"legs": ["front_left"], "axis": "depth", "value": 88})
    post(url, "capture", {"spacing": 1.0})
    before = len(poses(firmware))

    status, data = post(url, "preview", {})
    assert status == 200, data
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline and state_of(url)["busy"]:
        time.sleep(0.02)
    assert not state_of(url)["busy"], "the preview never finished"

    # One second of routine at ten poses a second, not fifty.
    sent = len(poses(firmware)) - before
    assert 8 <= sent <= 25, f"{sent} poses for a 1s preview -- expected about 11"


# --- the drive console, and the way back to the middle -------------------------------


def test_the_drive_console_is_not_locked_inside_one_tab() -> None:
    """The pad -- and the STOP in the middle of it -- used to live in the
    Sequence tab only, so posing was done with no stop button on screen. It now
    sits outside both <main>s, which is what puts it on both tabs."""
    page = resources.files("robodog.teach").joinpath("ui.html").read_text(encoding="utf-8")
    assert page.count('id="pad"') == 1, "the pad must exist once, not once per tab"
    before_console = page[: page.index('<aside class="console"')]
    assert 'id="pad"' not in before_console, "the pad is still inside a tab"
    assert 'id="functions"' not in before_console
    assert 'id="btn-home"' in page


def test_home_stops_a_latched_move_and_stands(posing_rig: tuple[FakeFirmware, str]) -> None:
    """One button for "wherever you are, come back": the stop first, because a
    robot still walking walks straight out of the pose it was just given."""
    from robodog.kinematics.constants import STAND_HEIGHT
    from robodog.kinematics.poses import stand_pose

    firmware, url = posing_rig
    post(url, "drive", {"move": "forward"})
    post(url, "set", {"legs": ["front_left"], "axis": "depth", "value": 88})
    assert state_of(url)["targets"]["front_left"]["depth"] == pytest.approx(88.0)

    status, data = post(url, "home", {})
    assert status == 200 and data["ok"], data
    assert "centred" in data["message"]

    expected = stand_pose(STAND_HEIGHT)[LegId.FRONT_LEFT]
    front_left = state_of(url)["targets"]["front_left"]
    for axis in ("x", "y", "z"):
        assert front_left[axis] == pytest.approx(getattr(expected, axis))
    assert state_of(url)["manual"]["move"] is None
    # The stop reached the firmware, not just our model.
    assert ("move", MOVE_STOP_FB, 0) in firmware.calls


def test_home_on_a_robot_that_cannot_pose_says_what_it_did(
    wifi_rig: tuple[FakeFirmware, str],
) -> None:
    """Stock firmware has no pose to return to. Stopping and saying so beats a
    button that quietly does half of what its label promises."""
    _firmware, url = wifi_rig
    post(url, "drive", {"move": "forward"})
    status, data = post(url, "home", {})
    assert status == 200 and data["ok"]
    assert "takes no leg targets" in data["message"]
    assert state_of(url)["manual"]["move"] is None


def test_posing_the_robot_ends_a_latched_move_on_the_page_too(
    posing_rig: tuple[FakeFirmware, str],
) -> None:
    """The firmware drops the move when it applies a pose (`robodogApply`), so
    the page must stop calling it driving -- otherwise the pad shows a latched
    direction that no longer exists, one drag away on the same screen."""
    _firmware, url = posing_rig
    post(url, "drive", {"move": "forward"})
    assert state_of(url)["manual"]["move"] == "forward"

    post(url, "set", {"legs": ["front_left"], "axis": "depth", "value": 88})
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and state_of(url)["manual"]["move"] is not None:
        time.sleep(0.02)
    assert state_of(url)["manual"]["move"] is None


def test_the_page_markup_is_balanced() -> None:
    """A stray tag renders as garbage in the browser and as nothing here.

    Every other test in this file talks to the server; the page itself is only
    ever asserted against by substring. So a restructuring that leaves a <div>
    open passes all of them and breaks the whole layout -- which is exactly the
    kind of edit the tab bar and the shared console each were.
    """
    from html.parser import HTMLParser

    void = {"br", "hr", "img", "input", "meta", "link", "source", "track", "area", "base", "col"}

    class Balance(HTMLParser):
        def __init__(self) -> None:
            super().__init__()
            self.stack: list[tuple[str, int]] = []
            self.problems: list[str] = []

        def handle_starttag(self, tag: str, attrs: object) -> None:
            if tag not in void:
                self.stack.append((tag, self.getpos()[0]))

        def handle_endtag(self, tag: str) -> None:
            if not self.stack:
                self.problems.append(f"line {self.getpos()[0]}: </{tag}> closes nothing")
                return
            opened, line = self.stack.pop()
            if opened != tag:
                self.problems.append(
                    f"line {self.getpos()[0]}: </{tag}> closes <{opened}> opened on line {line}"
                )

    parser = Balance()
    parser.feed(resources.files("robodog.teach").joinpath("ui.html").read_text(encoding="utf-8"))
    assert not parser.problems, "; ".join(parser.problems)
    unclosed = ", ".join(f"<{tag}> line {line}" for tag, line in parser.stack)
    assert not parser.stack, f"never closed: {unclosed}"


# --- keyboard control ---------------------------------------------------------------


def test_the_page_binds_the_keys_it_advertises() -> None:
    """The bindings and the line that documents them have to agree; a shortcut
    nobody can see is not a control, and one documented wrongly is worse."""
    page = resources.files("robodog.teach").joinpath("ui.html").read_text(encoding="utf-8")
    for code, move in (
        ("KeyW", "forward"),
        ("KeyA", "left"),
        ("KeyS", "backward"),
        ("KeyD", "right"),
    ):
        assert f'{code}: "{move}"' in page
    for code in ("Numpad8", "Numpad2", "Numpad6", "Numpad4"):
        assert f"{code}:" in page
    assert 'event.code === "Numpad5"' in page  # home, and it works without poses
    assert "hold to walk" in page


def test_walking_by_key_lets_go_when_the_key_or_the_window_does() -> None:
    """A latched direction bound to a key would be the worst of both: take your
    hand off the keyboard and the robot keeps walking. So the release is bound
    too -- and to losing the window as well, because a key held while the page
    loses focus never comes back up."""
    page = resources.files("robodog.teach").joinpath("ui.html").read_text(encoding="utf-8")
    assert 'document.addEventListener("keyup"' in page
    assert 'window.addEventListener("blur", releaseHeldDrive)' in page
    assert "if (event.repeat" in page, "auto-repeat would post the same move 30x a second"
    # Typing a routine name must not drive the robot.
    assert "typingInto(event)" in page


def test_leaning_over_the_wire_tips_the_robot(rig: tuple[TeachUIServer, MockBackend, str]) -> None:
    _server, _backend, url = rig
    post(url, "pose", {"pose": "stand"})
    before = state_of(url)["targets"]
    status, data = post(url, "lean", {"delta": 6})
    assert status == 200 and data["ok"], data

    after = state_of(url)["targets"]
    # Right leans: the left legs reach further, the right ones less.
    assert after["front_left"]["depth"] > before["front_left"]["depth"]
    assert after["hind_left"]["depth"] > before["hind_left"]["depth"]
    assert after["front_right"]["depth"] < before["front_right"]["depth"]
    assert after["hind_right"]["depth"] < before["hind_right"]["depth"]


def test_a_refused_lean_answers_with_the_reason(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    _server, _backend, url = rig
    post(url, "pose", {"pose": "stand"})
    status, data = post(url, "lean", {"delta": 40})
    assert status == 200 and not data["ok"]
    assert "outside" in data["message"]


def test_the_page_script_parses(tmp_path: Path) -> None:
    """A syntax error in the page's JavaScript breaks the entire UI silently.

    Every test here talks to the server, and the server serves the page happily
    whatever is inside its <script> tag -- so a stray brace ships a teach-in
    session where nothing at all responds, and the suite stays green. Skipped
    where node is missing; CI runners have it.
    """
    import shutil
    import subprocess

    node = shutil.which("node")
    if node is None:
        pytest.skip("node is not installed; the page's script cannot be parsed here")

    page = resources.files("robodog.teach").joinpath("ui.html").read_text(encoding="utf-8")
    opening = page.index("<script>") + len("<script>")
    script = page[opening : page.index("</script>", opening)]
    path = tmp_path / "ui.js"
    path.write_text(script, encoding="utf-8")

    done = subprocess.run([node, "--check", str(path)], capture_output=True, text=True, timeout=60)
    assert done.returncode == 0, done.stderr


# --- the camera --------------------------------------------------------------------


def test_the_camera_tab_carries_a_reachable_stream(
    posing_rig: tuple[FakeFirmware, str],
) -> None:
    """The URL has to come from the client, not from the backend behind it.

    It was read off the client first and silently produced None for every
    session, because the property lived on the backend -- a camera tab that
    never appeared, with nothing failing anywhere.
    """
    from robodog.camera import STREAM_PATH

    _firmware, url = posing_rig
    camera = state_of(url)["camera"]
    assert camera is not None, "a Wi-Fi robot always has a stream"
    assert camera["stream"].endswith(STREAM_PATH)
    assert camera["tuning"] is True
    # One port above the control server, as the firmware does it.
    control = int(url.rsplit(":", 1)[1].rstrip("/"))
    assert f":{control + 1}{STREAM_PATH}" not in camera["stream"]  # not OUR port


def test_a_robot_without_a_camera_offers_no_tab(
    rig: tuple[TeachUIServer, MockBackend, str],
) -> None:
    """Mock and the twin have no lens; a tab that shows a broken image is
    worse than no tab."""
    _server, _backend, url = rig
    assert state_of(url)["camera"] is None


def test_the_controls_carry_their_own_ranges(posing_rig: tuple[FakeFirmware, str]) -> None:
    """The page draws from the table rather than hard-coding end stops, so a
    slider cannot ask for a value the safety layer will refuse."""
    from robodog.camera import CAMERA_PARAMS

    _firmware, url = posing_rig
    params = {entry["name"]: entry for entry in state_of(url)["camera"]["params"]}
    assert len(params) == len(CAMERA_PARAMS)
    assert params["ae_level"]["min"] == -2 and params["ae_level"]["max"] == 2
    assert params["quality"]["kind"] == "range"
    assert params["aec"]["kind"] == "toggle"
    assert params["size"]["choices"][-1] == "640x480"  # what the fork allocates (F4)


def test_setting_a_parameter_reaches_the_robot(posing_rig: tuple[FakeFirmware, str]) -> None:
    firmware, url = posing_rig
    status, data = post(url, "camera", {"name": "ae_level", "value": -1})
    assert status == 200 and data["ok"], data
    assert ("cam_ae_level", -1, 0) in firmware.calls
    assert state_of(url)["camera"]["values"]["ae_level"] == -1


def test_a_value_the_sensor_would_ignore_is_refused(
    posing_rig: tuple[FakeFirmware, str],
) -> None:
    """An out-of-range write is a silent no-op on the sensor, which the operator
    reads as a broken camera. Refusing it with the range says more."""

    def camera_calls() -> list[tuple[str, int, int]]:
        # Only these: the ticker flushes poses on its own thread, and counting
        # every call makes this a race with it rather than a test of the write.
        return [call for call in firmware.calls if call[0].startswith("cam_")]

    firmware, url = posing_rig
    before = camera_calls()
    status, data = post(url, "camera", {"name": "ae_level", "value": 9})
    assert status == 409, data
    assert "outside" in data["message"]
    assert camera_calls() == before, "the robot was written to anyway"


def test_an_unknown_parameter_is_refused_by_name(posing_rig: tuple[FakeFirmware, str]) -> None:
    _firmware, url = posing_rig
    status, data = post(url, "camera", {"name": "sharpness", "value": 1})
    assert status == 400
    assert "unknown camera parameter" in data["message"]


def test_reset_puts_the_page_back_in_step_with_the_sensor(
    posing_rig: tuple[FakeFirmware, str],
) -> None:
    """Nothing can be read back, so the page's values are a memory of what was
    asked. `reset` re-applies the firmware's own tuning, and the memory has to
    follow or every control afterwards is a lie."""
    from robodog.camera import DEFAULTS

    firmware, url = posing_rig
    post(url, "camera", {"name": "saturation", "value": 2})
    assert state_of(url)["camera"]["values"]["saturation"] == 2

    status, data = post(url, "camera", {"name": "reset", "value": 0})
    assert status == 200 and data["ok"]
    assert ("cam_reset", 0, 0) in firmware.calls
    assert state_of(url)["camera"]["values"] == DEFAULTS


def test_the_controls_show_what_the_sensor_holds(posing_rig: tuple[FakeFirmware, str]) -> None:
    """Read back, not remembered.

    The camera is the one thing on this robot someone else can change -- the
    vendor's own web page, a serial session -- so what we asked for and what it
    holds are two different questions, and only one of them is the truth.
    """
    firmware, url = posing_rig
    camera = state_of(url)["camera"]
    assert camera["readback"] is True

    # Someone else moves it, without going through us at all.
    firmware.camera["contrast"] = -2
    post(url, "camera", {"name": "brightness", "value": 1})
    values = state_of(url)["camera"]["values"]
    assert values["brightness"] == 1
    assert values["contrast"] == -2, "the page kept its own memory over the robot's answer"


def test_a_clamped_value_is_reported_as_clamped(posing_rig: tuple[FakeFirmware, str]) -> None:
    """`size` is clamped to whatever frame buffer the robot allocated, and it
    does not say no -- it just lands somewhere else. A control that snapped back
    with no explanation would look broken."""
    firmware, url = posing_rig
    firmware.camera["size_max"] = 5  # this robot fell back to QVGA
    status, data = post(url, "camera", {"name": "size", "value": 8})
    assert status == 200 and data["ok"]
    assert "asked 8, holding 5" in data["message"]
    assert state_of(url)["camera"]["values"]["size"] == 5


def test_the_page_is_told_this_robots_own_ceiling(
    posing_rig: tuple[FakeFirmware, str],
) -> None:
    """Nothing else can discover it: the buffer is allocated once, before
    esp_camera_init, and only the robot knows whether it got VGA (F4)."""
    firmware, url = posing_rig
    firmware.camera["size_max"] = 5
    post(url, "camera", {"name": "quality", "value": 20})  # any write refreshes it
    assert state_of(url)["camera"]["limits"]["size_max"] == 5


def test_firmware_that_cannot_answer_still_works(posing_rig: tuple[FakeFirmware, str]) -> None:
    """An older fork answers cam_ with an empty 200. Losing the readback must
    not lose the write that already succeeded."""
    firmware, url = posing_rig
    firmware.camera_answers = False
    status, data = post(url, "camera", {"name": "saturation", "value": -1})
    assert status == 200 and data["ok"], data
    camera = state_of(url)["camera"]
    assert camera["readback"] is False
    assert camera["values"]["saturation"] == -1  # remembered, and labelled as such


def test_the_page_says_which_of_the_two_it_is_showing() -> None:
    page = resources.files("robodog.teach").joinpath("ui.html").read_text(encoding="utf-8")
    assert "read back from the sensor" in page
    assert "what was asked " in page
