"""HttpBackend against a local stand-in for the firmware's HTTP server.

The fake mirrors the real handler's contract (vendor/wavego-firmware/app_httpd.cpp):
all three query keys are mandatory, unknown variables are refused, and successful
requests return an empty 200. No hardware, no network beyond localhost.
"""

from __future__ import annotations

import threading
import urllib.parse
from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from robodog.api.client import RobotClient
from robodog.api.types import (
    BodyPose,
    Buzzer,
    Capability,
    Drive,
    FunctionMode,
    Gesture,
    GestureAxis,
    Led,
    LegId,
    LegTarget,
    SafetyState,
    SetBodyPose,
    SetFunction,
    SetLegTarget,
)
from robodog.backends.http import (
    DEFAULT_TIMEOUT,
    MOVE_BACKWARD,
    MOVE_FORWARD,
    MOVE_STOP_FB,
    MOVE_STOP_LR,
    MOVE_TURN_LEFT,
    MOVE_TURN_RIGHT,
    HttpBackend,
)
from robodog.errors import BackendError, CapabilityError
from tests.conftest import FakeClock

KNOWN_VARS = {"framesize", "funcMode", "sconfig", "sset", "move"}


class FakeFirmware:
    """Records what the firmware would have received."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, int, int]] = []
        self.index_hits = 0
        self.fail_next = 0  # respond 500 to this many upcoming /control calls


def make_handler(state: FakeFirmware) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args: object) -> None:  # keep test output clean
            pass

        def do_GET(self) -> None:
            parsed = urllib.parse.urlparse(self.path)
            if parsed.path == "/":
                state.index_hits += 1
                body = b"<html><body>robot</body></html>"
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if parsed.path != "/control":
                self.send_error(404)
                return

            query = urllib.parse.parse_qs(parsed.query)
            # All three keys are mandatory in the real firmware (ASSUMPTIONS D3).
            if not {"var", "val", "cmd"} <= query.keys():
                self.send_error(404)
                return
            var = query["var"][0]
            if var not in KNOWN_VARS:
                self.send_error(500)
                return
            if state.fail_next > 0:
                state.fail_next -= 1
                self.send_error(500)
                return

            state.calls.append((var, int(query["val"][0]), int(query["cmd"][0])))
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", "0")
            self.end_headers()

    return Handler


@pytest.fixture
def firmware() -> Iterator[tuple[FakeFirmware, str]]:
    state = FakeFirmware()
    server = HTTPServer(("127.0.0.1", 0), make_handler(state))
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.02})
    thread.daemon = True
    thread.start()
    port = server.server_address[1]
    try:
        yield state, f"127.0.0.1:{port}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


@pytest.fixture
def backend(firmware: tuple[FakeFirmware, str]) -> HttpBackend:
    _state, host = firmware
    return HttpBackend(host, timeout=5.0)


def moves(state: FakeFirmware) -> list[int]:
    return [val for var, val, _cmd in state.calls if var == "move"]


# --- capabilities and gating --------------------------------------------------


def test_capabilities_match_what_the_firmware_offers_over_wifi() -> None:
    caps = HttpBackend().capabilities
    assert caps == {Capability.LOCOMOTION, Capability.SERVO_TRIM}
    # Explicitly absent: everything the HTTP handler cannot do (ASSUMPTIONS D2).
    for missing in (
        Capability.GESTURE,
        Capability.PERIPHERALS,
        Capability.BODY_POSE,
        Capability.LEG_TARGET,
        Capability.JOINT_ANGLES,
        Capability.TELEMETRY,
    ):
        assert missing not in caps


@pytest.mark.parametrize(
    "command",
    [
        Gesture(GestureAxis.YAW, 1),
        Led(3),
        Buzzer(True),
        SetBodyPose(BodyPose(pitch=5.0)),
        SetLegTarget(LegId.FRONT_LEFT, LegTarget(16.0, 95.0, 25.0)),
    ],
)
def test_unsupported_commands_are_refused_by_the_supervisor(
    backend: HttpBackend, clock: FakeClock, command: object
) -> None:
    client = RobotClient(backend, clock=clock)
    client.connect()
    client.arm()
    with pytest.raises(CapabilityError):
        client.send(command)  # type: ignore[arg-type]


def test_backend_refuses_unsupported_command_even_if_called_directly(
    backend: HttpBackend,
) -> None:
    backend.connect()
    with pytest.raises(CapabilityError):
        backend.send(Led(1))


def test_commands_before_connect_raise(backend: HttpBackend) -> None:
    with pytest.raises(BackendError):
        backend.send(Drive(1, 0))
    with pytest.raises(BackendError):
        backend.state()


# --- connect / disconnect -----------------------------------------------------


def test_connect_asserts_a_stopped_state_without_fetching_the_index(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str]
) -> None:
    """The index page is the heaviest response the ESP32 serves; never probe it."""
    state, _host = firmware
    backend.connect()
    assert state.index_hits == 0
    assert moves(state) == [MOVE_STOP_FB, MOVE_STOP_LR]


def test_failed_connect_leaves_the_backend_disconnected(
    firmware: tuple[FakeFirmware, str],
) -> None:
    state, host = firmware
    backend = HttpBackend(host, timeout=5.0)
    state.fail_next = 99
    with pytest.raises(BackendError):
        backend.connect()
    with pytest.raises(BackendError, match="not connected"):
        backend.state()


def test_connect_fails_clearly_when_the_robot_is_unreachable() -> None:
    backend = HttpBackend("127.0.0.1:9", timeout=0.5)  # port 9 = discard
    with pytest.raises(BackendError, match="cannot reach robot"):
        backend.connect()


def test_disconnect_stops_motion(backend: HttpBackend, firmware: tuple[FakeFirmware, str]) -> None:
    state, _host = firmware
    backend.connect()
    backend.send(Drive(1, 0))
    state.calls.clear()
    backend.disconnect()
    assert moves(state) == [MOVE_STOP_FB, MOVE_STOP_LR]


def test_disconnect_survives_a_dead_link(firmware: tuple[FakeFirmware, str]) -> None:
    state, host = firmware
    backend = HttpBackend(host, timeout=5.0)
    backend.connect()
    state.fail_next = 99  # link goes bad
    backend.disconnect()  # must not raise


# --- command mapping ----------------------------------------------------------


@pytest.mark.parametrize(
    ("forward", "turn", "expected"),
    [
        (1, 0, [MOVE_FORWARD, MOVE_STOP_LR]),
        (-1, 0, [MOVE_BACKWARD, MOVE_STOP_LR]),
        (0, 1, [MOVE_STOP_FB, MOVE_TURN_RIGHT]),
        (0, -1, [MOVE_STOP_FB, MOVE_TURN_LEFT]),
        (1, 1, [MOVE_FORWARD, MOVE_TURN_RIGHT]),
        (0, 0, [MOVE_STOP_FB, MOVE_STOP_LR]),
    ],
)
def test_drive_maps_onto_both_latched_axes(
    backend: HttpBackend,
    firmware: tuple[FakeFirmware, str],
    forward: int,
    turn: int,
    expected: list[int],
) -> None:
    state, _host = firmware
    backend.connect()
    state.calls.clear()
    backend.send(Drive(forward, turn))
    assert moves(state) == expected


def test_function_modes_are_sent_as_funcmode(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str]
) -> None:
    state, _host = firmware
    backend.connect()
    state.calls.clear()
    backend.send(SetFunction(FunctionMode.HANDSHAKE))
    assert state.calls == [("funcMode", int(FunctionMode.HANDSHAKE), 0)]


def test_every_request_carries_all_three_keys(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str]
) -> None:
    backend.connect()
    backend.send(Drive(1, 0))
    backend.send(SetFunction(FunctionMode.JUMP))
    for path in backend.request_log:
        if path.startswith("/control"):
            query = urllib.parse.parse_qs(urllib.parse.urlparse(path).query)
            assert {"var", "val", "cmd"} <= query.keys()


def test_http_error_becomes_backend_error(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str]
) -> None:
    state, _host = firmware
    backend.connect()
    state.fail_next = 1
    with pytest.raises(BackendError, match="HTTP 500"):
        backend.send(Drive(1, 0))


# --- servo trim (the calibration facility) ------------------------------------


def test_trim_servo_sends_sconfig_with_offset(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str]
) -> None:
    state, _host = firmware
    backend.connect()
    state.calls.clear()
    backend.trim_servo(8, -3)
    assert state.calls == [("sconfig", 8, -3)]


def test_trim_rejects_channels_outside_the_pca9685(backend: HttpBackend) -> None:
    backend.connect()
    with pytest.raises(BackendError, match=r"0\.\.15"):
        backend.trim_servo(16, 1)


def test_sync_baseline_sends_middle_pos_first(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str]
) -> None:
    """ASSUMPTIONS D6: without this the first trim per servo jumps."""
    state, _host = firmware
    backend.connect()
    state.calls.clear()
    backend.sync_servo_baseline()
    assert state.calls == [("funcMode", int(FunctionMode.MIDDLE_POS), 0)]


def test_save_trim_sends_sset(backend: HttpBackend, firmware: tuple[FakeFirmware, str]) -> None:
    state, _host = firmware
    backend.connect()
    state.calls.clear()
    backend.save_servo_trim(8)
    assert state.calls == [("sset", 8, 0)]


def test_servo_channels_match_the_firmware_map() -> None:
    assert HttpBackend.servo_channels(LegId.FRONT_LEFT) == (8, 9, 10)
    assert HttpBackend.servo_channels(LegId.HIND_RIGHT) == (1, 0, 2)


# --- state and safety ---------------------------------------------------------


def test_state_is_always_flagged_as_estimated(backend: HttpBackend) -> None:
    backend.connect()
    state = backend.state()
    assert state.is_estimated
    assert state.telemetry is None


def test_state_model_advances_with_ticks(backend: HttpBackend) -> None:
    backend.connect()
    backend.send(Drive(1, 0))
    before = backend.state().leg_targets[LegId.FRONT_LEFT]
    backend.tick(0.1)
    assert backend.state().leg_targets[LegId.FRONT_LEFT] != before


def test_safe_sequence_stops_both_axes(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str]
) -> None:
    state, _host = firmware
    backend.connect()
    backend.send(Drive(1, 1))
    state.calls.clear()
    backend.safe_sequence()
    assert moves(state) == [MOVE_STOP_FB, MOVE_STOP_LR]
    assert backend.state().drive == Drive(0, 0)


def test_safe_sequence_retries_before_giving_up(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str]
) -> None:
    state, _host = firmware
    backend.connect()
    state.calls.clear()
    state.fail_next = 1  # the first attempt fails, the retry gets through
    backend.safe_sequence()
    assert moves(state) == [MOVE_STOP_FB, MOVE_STOP_LR]


def test_stop_attempts_use_a_short_timeout_so_a_dead_link_fails_fast() -> None:
    """The operator stands next to a moving robot; hanging is worse than failing."""
    from robodog.backends.http import _STOP_RETRIES, STOP_TIMEOUT

    # Worst case for a severed link: both axes, all retries, at the stop timeout.
    worst_case = 2 * _STOP_RETRIES * STOP_TIMEOUT
    assert worst_case <= 5.0
    assert STOP_TIMEOUT < DEFAULT_TIMEOUT  # stops are more urgent than commands


def test_disconnect_after_a_confirmed_stop_sends_nothing_more(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str]
) -> None:
    state, _host = firmware
    backend.connect()
    backend.safe_sequence()
    state.calls.clear()
    backend.disconnect()
    assert state.calls == []  # already known-stopped, no duplicate attempt


def test_disconnect_still_stops_when_the_robot_was_left_moving(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str]
) -> None:
    state, _host = firmware
    backend.connect()
    backend.send(Drive(1, 0))
    state.calls.clear()
    backend.disconnect()
    assert moves(state) == [MOVE_STOP_FB, MOVE_STOP_LR]


def test_safe_sequence_reports_when_the_stop_cannot_be_confirmed(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str]
) -> None:
    """Honest failure: over Wi-Fi we cannot guarantee a stop (ASSUMPTIONS D10)."""
    state, _host = firmware
    backend.connect()
    state.fail_next = 99
    with pytest.raises(BackendError, match="could not confirm stop"):
        backend.safe_sequence()
    # The model still reflects the intent, so the operator sees 'stopped'.
    assert backend.state().drive == Drive(0, 0)


def test_estop_through_the_client_latches(backend: HttpBackend, clock: FakeClock) -> None:
    client = RobotClient(backend, clock=clock)
    client.connect()
    client.arm()
    client.drive(forward=1)
    client.estop("test")
    assert client.safety_state is SafetyState.ESTOPPED


def test_watchdog_timeout_stops_the_robot(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str], clock: FakeClock
) -> None:
    state, _host = firmware
    client = RobotClient(backend, clock=clock, watchdog_timeout=0.5)
    client.connect()
    client.arm()
    client.drive(forward=1)
    state.calls.clear()
    clock.advance(2.0)
    client.tick(0.02)
    assert client.safety_state is SafetyState.ESTOPPED
    assert moves(state) == [MOVE_STOP_FB, MOVE_STOP_LR]


def test_wifi_routine_plays_end_to_end(
    backend: HttpBackend, firmware: tuple[FakeFirmware, str], clock: FakeClock
) -> None:
    from pathlib import Path

    from robodog.teach.format import load_routine
    from robodog.teach.player import play_routine

    state, _host = firmware
    routine = load_routine(Path(__file__).resolve().parent.parent / "routines" / "patrol-wifi.yaml")
    client = RobotClient(backend, clock=clock)
    client.connect()
    client.arm()
    state.calls.clear()
    play_routine(routine, client, tick=0.1)
    # Six drive steps, each mapping to two axis commands.
    assert len(moves(state)) == 12
    assert moves(state)[0] == MOVE_FORWARD
    assert moves(state)[-2:] == [MOVE_STOP_FB, MOVE_STOP_LR]
