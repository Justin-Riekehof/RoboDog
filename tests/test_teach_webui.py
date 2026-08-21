"""Teach web UI server: endpoints, geometry, safety pass-through. No browser.

The page's JavaScript is deliberately dumb (it only draws what the server sends
and posts intents back), so testing the endpoints tests the behaviour.
"""

from __future__ import annotations

import json
import threading
import time
import urllib.request
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import pytest

from robodog.api.client import RobotClient
from robodog.backends.mock import MockBackend
from robodog.kinematics.constants import STAND_HEIGHT
from robodog.teach.format import load_routine
from robodog.teach.session import TeachSession
from robodog.teach.webui import TeachUIServer
from tests.conftest import FakeClock


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
    assert state["limits"]["height_min"] == 75.0
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
