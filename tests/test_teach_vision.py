"""The teach UI's vision half: re-serving the camera, and running a behaviour.

Everything real except the two things that need hardware. The detector is
scripted (it answers from a list, exactly as MockBackend answers as a robot),
and the language model is the fake OpenAI server from test_ai -- so the path
under test is the whole one: an HTTP request from the page, the intent client,
the vocabulary, the state machine, the runner, the safety supervisor and a
backend.
"""

from __future__ import annotations

import threading
import time
import urllib.error
import urllib.request
from collections.abc import Iterator
from dataclasses import replace
from pathlib import Path
from typing import Any

import pytest

from robodog.ai import IntentClient, LlmConfig
from robodog.api.client import RobotClient
from robodog.api.types import Drive, RobotState, Telemetry
from robodog.backends.mock import MockBackend
from robodog.teach.session import TeachSession
from robodog.teach.webui import TeachUIServer
from robodog.vision import Box, Detection, ScriptedDetector
from robodog.vision.service import VisionService
from robodog.vision.stream import MjpegParser
from tests.conftest import FakeClock
from tests.test_ai import FakeLlm, serve
from tests.test_teach_webui import get_text, post, state_of, wait_idle

FRAME = b"\xff\xd8\xff\xe0a-frame-from-the-robot\xff\xd9"


def seen(bearing: float, height: float, label: str = "person") -> Detection:
    center = (bearing + 1.0) / 2.0
    return Detection(
        label=label,
        confidence=0.9,
        box=Box(
            left=max(center - 0.1, 0.0),
            top=max(0.5 - height / 2, 0.0),
            right=min(center + 0.1, 1.0),
            bottom=min(0.5 + height / 2, 1.0),
        ),
    )


def wait_for_detections(url: str, timeout: float = 10.0) -> dict[str, Any]:
    """Poll until the detector has answered once -- the page does the same."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = state_of(url)
        if (state.get("vision") or {}).get("detections"):
            return state
        time.sleep(0.01)
    raise AssertionError("the detector never reported anything")


Rig = tuple[TeachUIServer, MockBackend, str, FakeLlm]


def make_rig(
    tmp_path: Path,
    *,
    detections: list[Detection] | None = None,
    with_vision: bool = True,
    with_llm: bool = True,
) -> Iterator[Rig]:
    (tmp_path / "frame.jpg").write_bytes(FRAME)
    backend = MockBackend()
    client = RobotClient(backend, clock=FakeClock())
    client.connect()
    client.arm()
    session = TeachSession(client, name="visiontest", default_path=tmp_path / "v.yaml")
    session.start()
    vision = None
    if with_vision:
        vision = VisionService(
            str(tmp_path),
            detector=ScriptedDetector([detections] if detections is not None else [[]]),
        )
        vision.start()
    fake = FakeLlm()
    for llm_url in serve(fake):
        interpreter = IntentClient(LlmConfig(base_url=llm_url)) if with_llm else None
        server = TeachUIServer(
            session, client, lock=threading.Lock(), vision=vision, interpreter=interpreter
        )
        url = server.start()
        try:
            yield server, backend, url, fake
        finally:
            server.shutdown()
            if vision is not None:
                vision.stop()


@pytest.fixture
def rig(tmp_path: Path) -> Iterator[Rig]:
    yield from make_rig(tmp_path, detections=[seen(0.0, 0.25)])


# --- the picture ------------------------------------------------------------


def test_the_page_is_pointed_at_the_host_not_at_the_robot(rig: Rig) -> None:
    """One frame buffer means one consumer (ASSUMPTIONS F4/G3), and it is us."""
    _server, _backend, url, _fake = rig
    assert state_of(url)["camera"]["stream"] == "/camera/stream"


def test_the_rebroadcast_serves_the_frames_the_host_read(rig: Rig) -> None:
    _server, _backend, url, _fake = rig
    parser = MjpegParser()
    frames: list[bytes] = []
    with urllib.request.urlopen(url.rstrip("/") + "/camera/stream", timeout=5) as response:
        assert "multipart/x-mixed-replace" in response.headers["Content-Type"]
        while not frames:
            frames += parser.feed(response.read(512))
    assert frames[0] == FRAME


def test_a_single_frame_can_be_fetched(rig: Rig) -> None:
    with urllib.request.urlopen(rig[2].rstrip("/") + "/camera/frame.jpg", timeout=5) as response:
        assert response.status == 200
        assert response.read() == FRAME


def test_state_carries_the_detections_as_boxes_the_page_can_draw(rig: Rig) -> None:
    _server, _backend, url, _fake = rig
    vision = wait_for_detections(url)["vision"]
    assert vision["detector"] == "scripted"
    found = vision["detections"][0]
    assert found["label"] == "person"
    assert 0.0 <= found["box"]["left"] < found["box"]["right"] <= 1.0
    assert found["bearing"] == pytest.approx(0.0, abs=0.01)
    # The distance estimate travels for display only, and says so in its name.
    assert found["distance_mm"] > 0


# --- saying something -------------------------------------------------------


def test_come_to_me_walks_the_robot_and_ends_stopped(tmp_path: Path) -> None:
    """The acceptance criterion, with everything but the robot and the camera."""
    for _server, backend, url, fake in make_rig(tmp_path, detections=[seen(0.0, 0.9)]):
        wait_for_detections(url)
        fake.content = '{"behaviour": "come_to_me", "target": "person"}'
        status, data = post(url, "say", {"text": "Komm zu mir"})
        assert status == 200 and data["ok"], data
        state = wait_idle(url)
        assert state["behaviour"]["call"] == "come_to_me(target=person)"
        assert state["behaviour"]["state"] == "ARRIVED"
        assert not state["behaviour"]["active"]
        assert backend.state().drive == Drive(0, 0)


def test_a_run_reports_what_it_is_steering_by(tmp_path: Path) -> None:
    for _server, _backend, url, fake in make_rig(tmp_path, detections=[seen(0.6, 0.2)]):
        wait_for_detections(url)
        fake.content = '{"behaviour": "come_to_me"}'
        post(url, "say", {"text": "Komm zu mir"})
        deadline = time.monotonic() + 10.0
        target = None
        while target is None and time.monotonic() < deadline:
            target = (state_of(url)["behaviour"] or {}).get("target")
            time.sleep(0.01)
        assert target is not None and target["bearing"] > 0.5
        post(url, "stop", {})
        wait_idle(url)


def test_stop_ends_a_behaviour_run_like_it_ends_a_sequence(tmp_path: Path) -> None:
    """Deliberately the same path: one button, one meaning, already proven."""
    for _server, backend, url, fake in make_rig(tmp_path, detections=[seen(0.0, 0.15)]):
        wait_for_detections(url)
        fake.content = '{"behaviour": "come_to_me"}'
        post(url, "say", {"text": "Komm zu mir"})
        deadline = time.monotonic() + 10.0
        while not state_of(url)["busy"] and time.monotonic() < deadline:
            time.sleep(0.01)
        status, data = post(url, "stop", {})
        assert status == 200 and data["ok"]
        state = wait_idle(url)
        assert "stopped" in state["behaviour"]["message"]
        assert backend.state().drive == Drive(0, 0)


def test_a_stop_command_stops_without_starting_anything(rig: Rig) -> None:
    _server, _backend, url, fake = rig
    fake.content = '{"behaviour": "stop"}'
    status, data = post(url, "say", {"text": "halt"})
    assert status == 200 and data["ok"]
    assert not state_of(url)["busy"]


def test_a_sentence_outside_the_vocabulary_starts_nothing(rig: Rig) -> None:
    _server, _backend, url, fake = rig
    fake.content = '{"behaviour": "unknown"}'
    status, data = post(url, "say", {"text": "mach einen Rueckwaertssalto"})
    assert status == 200 and not data["ok"]
    assert "no behaviour for that" in data["message"]
    assert not state_of(url)["busy"]


def test_a_language_model_that_fails_is_reported_not_raised(rig: Rig) -> None:
    _server, _backend, url, fake = rig
    fake.status = 503
    status, data = post(url, "say", {"text": "Komm zu mir"})
    assert status == 502
    assert "HTTP 503" in data["message"]


def test_saying_nothing_is_a_bad_request(rig: Rig) -> None:
    status, data = post(rig[2], "say", {"text": "   "})
    assert status == 400
    assert "say what the robot should do" in data["message"]


# --- and without the optional halves ---------------------------------------


def test_without_a_language_model_the_command_box_is_off(tmp_path: Path) -> None:
    for _server, _backend, url, _fake in make_rig(tmp_path, with_llm=False):
        assert state_of(url)["can_talk"] is False
        status, data = post(url, "say", {"text": "Komm zu mir"})
        assert status == 400
        assert "--llm-url" in data["message"]


def test_without_vision_there_is_nothing_to_re_serve(tmp_path: Path) -> None:
    for _server, _backend, url, fake in make_rig(tmp_path, with_vision=False):
        assert state_of(url)["vision"] is None
        with pytest.raises(urllib.error.HTTPError) as refused:
            urllib.request.urlopen(url.rstrip("/") + "/camera/stream", timeout=5)
        assert refused.value.code == 404
        fake.content = '{"behaviour": "come_to_me"}'
        status, data = post(url, "say", {"text": "Komm zu mir"})
        assert status == 400
        assert "needs the camera" in data["message"]


def test_the_page_still_serves_and_the_boxes_have_somewhere_to_go(rig: Rig) -> None:
    """The overlay lives in the shipped page, not in a build step."""
    _status, page = get_text(rig[2], "/")
    assert 'id="cam-boxes"' in page
    assert 'id="say-text"' in page


# --- the body's own attitude, which is the only measured thing here ---------


class TiltedBackend(MockBackend):
    """A mock that also reports an attitude, as the IMU-carrying firmware does."""

    def state(self) -> RobotState:
        base = super().state()
        return replace(
            base,
            telemetry=Telemetry(pitch=-3.5, roll=0.5, turned=91.0, still=False),
        )


def test_the_page_shows_what_the_imu_says(tmp_path: Path) -> None:
    """Nose-down must read as a negative pitch -- that one observation settles
    the axis signs on the robot (ASSUMPTIONS G9)."""
    (tmp_path / "frame.jpg").write_bytes(FRAME)
    backend = TiltedBackend()
    client = RobotClient(backend, clock=FakeClock())
    client.connect()
    client.arm()
    session = TeachSession(client, name="tilt", default_path=tmp_path / "t.yaml")
    session.start()
    server = TeachUIServer(session, client, lock=threading.Lock())
    url = server.start()
    try:
        attitude = state_of(url)["attitude"]
        assert attitude == {
            "pitch": -3.5,
            "roll": 0.5,
            "turned": 91.0,
            "still": False,
            # None, not 0: this mock reports no loop health, and "no idea" must
            # stay distinguishable from "perfect".
            "loop_max_ms": None,
        }
    finally:
        server.shutdown()


def test_a_backend_without_an_imu_says_nothing_rather_than_zero(rig: Rig) -> None:
    """Zero degrees and 'no idea' are different, and only one of them is safe
    to feed into the distance geometry."""
    assert state_of(rig[2])["attitude"] is None


# --- the direct path: behaviours without any language model ------------------


def test_a_behaviour_starts_without_a_language_model(tmp_path: Path) -> None:
    """The model was only ever a translator, and this proves it: the same
    deterministic run, started from explicit parameters, no LLM configured."""
    for _server, backend, url, _fake in make_rig(
        tmp_path, detections=[seen(0.0, 0.9)], with_llm=False
    ):
        wait_for_detections(url)
        status, data = post(url, "behaviour", {"name": "come_to_me", "target": "person"})
        assert status == 200 and data["ok"], data
        state = wait_idle(url)
        assert state["behaviour"]["call"] == "come_to_me(target=person)"
        assert state["behaviour"]["state"] == "ARRIVED"
        assert backend.state().drive == Drive(0, 0)


def test_the_direct_path_validates_against_the_same_vocabulary(rig: Rig) -> None:
    status, data = post(rig[2], "behaviour", {"name": "backflip"})
    assert status == 400
    assert "unknown behaviour" in data["message"]
    status, data = post(rig[2], "behaviour", {"name": "come_to_me", "target": "dragon"})
    assert status == 400
    assert "is not one of" in data["message"]


def test_the_direct_path_takes_a_stop_distance(rig: Rig) -> None:
    """The one thing the model parsed from words arrives here as a number."""
    _server, _backend, url, _fake = rig
    status, data = post(
        url, "behaviour", {"name": "come_to_me", "target": "person", "stop_distance_mm": 2000}
    )
    assert status == 200 and data["ok"], data
    assert "stop_distance_mm=2000" in state_of(url)["behaviour"]["call"]
    post(url, "stop", {})
    wait_idle(url)


def test_direct_stop_needs_no_camera_and_no_model(tmp_path: Path) -> None:
    for _server, _backend, url, _fake in make_rig(tmp_path, with_llm=False, with_vision=False):
        status, data = post(url, "behaviour", {"name": "stop"})
        assert status == 200 and data["ok"]
