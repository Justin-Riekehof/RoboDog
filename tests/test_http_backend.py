"""HttpBackend against a local stand-in for the firmware's HTTP server.

The fake mirrors the real handler's contract (vendor/wavego-firmware/app_httpd.cpp):
all three query keys are mandatory, unknown variables are refused, and successful
requests return an empty 200. No hardware, no network beyond localhost.
"""

from __future__ import annotations

import json
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
from robodog.camera import DEFAULTS as CAMERA_DEFAULTS
from robodog.errors import BackendError, CapabilityError
from tests.conftest import FakeClock

KNOWN_VARS = {"framesize", "funcMode", "sconfig", "sset", "move"}
# What firmware/wavego-robodog adds on top. A fake that speaks them stands in
# for a flashed robot; one that does not stands in for a stock one.
ROBODOG_VARS = {"ping", "watchdog", "pose"}
# ...plus everything matching `cam_*`, which the fork resolves by prefix.


class FakeFirmware:
    """Records what the firmware would have received."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, int, int]] = []
        self.index_hits = 0
        self.fail_next = 0  # respond 500 to this many upcoming /control calls
        self.robodog = False  # True = speaks our firmware's added commands
        self.queries: list[str] = []  # full query strings, for the pose keys
        # The sensor's own state, which our firmware answers every cam_ request
        # with. `size_max` is what this robot managed to allocate at boot -- the
        # one value no host can work out for itself (ASSUMPTIONS F4).
        self.camera: dict[str, int] = {**CAMERA_DEFAULTS, "size_max": 8, "psram": 0}
        self.camera_answers = True  # False = firmware too old to reply with a body


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
            state.queries.append(parsed.query)
            known = KNOWN_VARS | (ROBODOG_VARS if state.robodog else set())
            # The camera family is matched by prefix in the firmware too: the
            # names are the sensor driver's, and listing two dozen of them here
            # would be the same table in two places, drifting apart.
            camera = state.robodog and var.startswith("cam_")
            if var not in known and not camera:
                self.send_error(500)
                return
            if state.fail_next > 0:
                state.fail_next -= 1
                self.send_error(500)
                return

            state.calls.append((var, int(query["val"][0]), int(query["cmd"][0])))
            if camera:
                name, value = var[4:], int(query["val"][0])
                if name == "reset":
                    state.camera.update(CAMERA_DEFAULTS)
                elif name == "size":
                    # Clamped, not rejected, exactly as the firmware does it.
                    state.camera[name] = min(value, state.camera["size_max"])
                elif name in CAMERA_DEFAULTS:
                    state.camera[name] = value
                if state.camera_answers:
                    body = json.dumps({"camera": True, **state.camera}).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
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
    # CAMERA is in the floor: the vendor's own firmware serves the same MJPEG
    # stream, so watching the robot is not what the fork added -- tuning the
    # sensor is, and that is CAMERA_TUNING below.
    assert caps == {Capability.LOCOMOTION, Capability.SERVO_TRIM, Capability.CAMERA}
    # Explicitly absent: everything the HTTP handler cannot do (ASSUMPTIONS D2).
    for missing in (
        Capability.GESTURE,
        Capability.PERIPHERALS,
        Capability.BODY_POSE,
        Capability.LEG_TARGET,
        Capability.JOINT_ANGLES,
        Capability.TELEMETRY,
        Capability.CAMERA_TUNING,
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


# --- which firmware is on the robot -------------------------------------------------


def test_stock_firmware_is_detected(firmware: tuple[FakeFirmware, str]) -> None:
    """A robot that does not know `ping` gets the stock capability set."""
    _state, host = firmware
    backend = HttpBackend(host)
    backend.connect()
    assert backend.capabilities == HttpBackend.STOCK_CAPABILITIES
    assert Capability.LEG_TARGET not in backend.capabilities
    backend.disconnect()


def test_our_firmware_is_detected(firmware: tuple[FakeFirmware, str]) -> None:
    state, host = firmware
    state.robodog = True
    backend = HttpBackend(host)
    backend.connect()
    assert Capability.LEG_TARGET in backend.capabilities
    assert backend.capabilities >= HttpBackend.STOCK_CAPABILITIES  # only ever adds
    backend.disconnect()


def test_the_probe_cannot_move_the_robot(firmware: tuple[FakeFirmware, str]) -> None:
    """Detection asks with `ping`, the one added command that changes nothing."""
    state, host = firmware
    state.robodog = True
    backend = HttpBackend(host)
    backend.connect()
    probes = [c for c in state.calls if c[0] == "ping"]
    assert len(probes) == 1
    assert not [c for c in state.calls if c[0] in ("funcMode", "sconfig", "pose")]
    backend.disconnect()


def test_the_firmware_can_be_stated_instead_of_probed(
    firmware: tuple[FakeFirmware, str],
) -> None:
    """A probe that guesses wrong must be overrulable by hand."""
    state, host = firmware
    state.robodog = True
    backend = HttpBackend(host, firmware="robodog")
    backend.connect()
    assert Capability.LEG_TARGET in backend.capabilities
    assert not [c for c in state.calls if c[0] == "ping"]  # declared, not probed
    backend.disconnect()


def test_declaring_stock_leaves_the_extras_alone(firmware: tuple[FakeFirmware, str]) -> None:
    """Even on a robot that has them -- the operator's word wins."""
    state, host = firmware
    state.robodog = True
    backend = HttpBackend(host, firmware="stock")
    backend.connect()
    assert Capability.LEG_TARGET not in backend.capabilities
    assert not [c for c in state.calls if c[0] in ("ping", "watchdog")]
    backend.disconnect()


def test_declaring_our_firmware_on_a_stock_robot_fails_loudly(
    firmware: tuple[FakeFirmware, str],
) -> None:
    """Asserting a robot that stops itself, when it does not, is worse than a
    wrong guess: it would silently be trusted."""
    _state, host = firmware  # a stock fake: it refuses `watchdog`
    backend = HttpBackend(host, firmware="robodog")
    with pytest.raises(BackendError, match="not running 'robodog' firmware"):
        backend.connect()


def test_an_unknown_firmware_name_is_refused() -> None:
    with pytest.raises(BackendError, match="unknown firmware"):
        HttpBackend("127.0.0.1", firmware="experimental")


# --- poses ---------------------------------------------------------------------------


def stand_targets() -> dict[LegId, LegTarget]:
    return {
        leg: LegTarget(16.0 if leg in (LegId.FRONT_LEFT, LegId.FRONT_RIGHT) else -16.0, 95.0, 25.0)
        for leg in LegId
    }


def test_a_whole_pose_travels_in_one_request(firmware: tuple[FakeFirmware, str]) -> None:
    """Four SetLegTarget commands and a tick must cost exactly one round trip."""
    state, host = firmware
    state.robodog = True
    backend = HttpBackend(host)
    backend.connect()
    before = len(state.calls)
    for leg, target in stand_targets().items():
        backend.send(SetLegTarget(leg=leg, target=target))
    assert len(state.calls) == before  # nothing sent yet: staged only
    backend.tick(0.02)

    poses = [q for q in state.queries if "var=pose" in q]
    assert len(poses) == 1
    query = poses[0]
    for leg in LegId:
        for axis in ("x", "y", "z"):
            assert f"l{int(leg)}{axis}=" in query, f"l{int(leg)}{axis} missing from {query}"
    backend.disconnect()


def test_an_unchanged_pose_is_not_resent(firmware: tuple[FakeFirmware, str]) -> None:
    """The teach UI ticks 50 times a second while the operator thinks."""
    state, host = firmware
    state.robodog = True
    backend = HttpBackend(host)
    backend.connect()
    for leg, target in stand_targets().items():
        backend.send(SetLegTarget(leg=leg, target=target))
    backend.tick(0.02)
    for _ in range(10):
        backend.tick(0.02)
    assert len([q for q in state.queries if "var=pose" in q]) == 1
    backend.disconnect()


def test_a_pose_on_stock_firmware_says_what_is_missing(
    firmware: tuple[FakeFirmware, str],
) -> None:
    """Without our firmware the command has nowhere to go; say so, do not send."""
    state, host = firmware  # state.robodog stays False
    backend = HttpBackend(host)
    backend.connect()
    backend.send(SetLegTarget(leg=LegId.FRONT_LEFT, target=LegTarget(16.0, 95.0, 25.0)))
    with pytest.raises(CapabilityError, match="wavego-robodog"):
        backend.tick(0.02)
    assert not [q for q in state.queries if "var=pose" in q]
    backend.disconnect()


def test_the_robot_watchdog_is_armed_and_disarmed_around_a_session(
    firmware: tuple[FakeFirmware, str],
) -> None:
    """Armed on connect, released on the way out.

    Leaving it armed would stop the robot mid-walk for whoever drives it next
    from the vendor's own page, which sends nothing while the robot walks.
    """
    state, host = firmware
    state.robodog = True
    backend = HttpBackend(host)
    backend.connect()
    armed = [c for c in state.calls if c[0] == "watchdog"]
    assert armed and armed[0][1] > 0
    backend.disconnect()
    disarmed = [c for c in state.calls if c[0] == "watchdog"]
    assert disarmed[-1][1] == 0


def test_a_stock_robot_is_never_told_about_a_watchdog(
    firmware: tuple[FakeFirmware, str],
) -> None:
    state, host = firmware  # stays stock
    backend = HttpBackend(host)
    backend.connect()
    backend.disconnect()
    assert not [c for c in state.calls if c[0] == "watchdog"]


def test_the_host_budget_outlasts_the_transport(firmware: tuple[FakeFirmware, str]) -> None:
    """A pose takes about a second on the robot's own AP, and the supervisor
    checks its deadline right after the request returns. A budget below that
    E-stops a healthy robot -- which it did, twice, before this existed."""
    from robodog.backends.http import STOCK_WATCHDOG, SUGGESTED_WATCHDOG

    state, host = firmware
    backend = HttpBackend(host)
    backend.connect()
    assert backend.suggested_watchdog == STOCK_WATCHDOG  # nothing slow to do
    backend.disconnect()

    state.robodog = True
    posing = HttpBackend(host)
    posing.connect()
    assert posing.suggested_watchdog == SUGGESTED_WATCHDOG
    assert SUGGESTED_WATCHDOG > 1.2  # the slowest pose measured on the device
    posing.disconnect()


def test_every_paced_loop_can_ask_how_fast_to_run(
    firmware: tuple[FakeFirmware, str],
) -> None:
    """`tick_for` is the one answer to "how fast may I drive this?".

    Three loops pace themselves by it -- the player, the teach ticker and the
    UI's preview -- and each used to carry its own hard-coded 50 Hz. A backend
    with no opinion still gets one, so the seam works for mock and sim too.
    """
    from robodog.api.client import RobotClient
    from robodog.backends.base import DEFAULT_TICK, tick_for
    from robodog.backends.http import SUGGESTED_TICK
    from robodog.backends.mock import MockBackend

    assert tick_for(MockBackend()) == DEFAULT_TICK  # no transport, no opinion

    state, host = firmware
    state.robodog = True
    backend = HttpBackend(host)
    client = RobotClient(backend)
    client.connect()
    # Poses stream over the wire here, so the link -- not the loop -- sets the rate.
    assert client.suggested_tick == SUGGESTED_TICK
    assert SUGGESTED_TICK > DEFAULT_TICK
    client.disconnect()


# --- keeping a walking robot alive ---------------------------------------------------


def test_a_latched_move_is_kept_alive(firmware: tuple[FakeFirmware, str]) -> None:
    """The bug this exists for: the robot stopped and crouched mid-walk.

    The firmware watchdog only acts on a robot that is MOVING, and a moving
    robot is exactly what the host sends nothing to -- the move latches and the
    firmware walks on by itself. So the one state the watchdog guards produced
    no traffic to feed it, and a healthy link looked identical to a dead one
    after a second and a half of walking.
    """
    from robodog.backends.http import FIRMWARE_FEED_INTERVAL, FIRMWARE_WATCHDOG_MS

    state, host = firmware
    state.robodog = True
    clock = FakeClock()
    backend = HttpBackend(host, clock=clock)
    backend.connect()

    backend.send(Drive(forward=1, turn=0))
    state.calls.clear()

    # Walk for four watchdog periods, ticking as the teach loop does.
    for _ in range(int(4 * FIRMWARE_WATCHDOG_MS / 1000 / 0.1)):
        clock.advance(0.1)
        backend.tick(0.1)

    feeds = [call for call in state.calls if call[0] == "ping"]
    assert feeds, "the robot heard nothing for six seconds of walking"
    # Never a gap the firmware would call silence.
    assert len(feeds) >= 4 * 3 - 1

    backend.disconnect()
    assert FIRMWARE_FEED_INTERVAL * 1000 * 2 < FIRMWARE_WATCHDOG_MS, (
        "two lost feeds must not be enough to stop the robot"
    )


def test_a_standing_robot_is_left_in_peace(firmware: tuple[FakeFirmware, str]) -> None:
    """The other half: a feed on a robot that is not moving would be pure
    traffic. The watchdog ignores a stopped robot, so there is nothing to keep
    alive -- and over a link this slow, every needless request costs a pose."""
    state, host = firmware
    state.robodog = True
    clock = FakeClock()
    backend = HttpBackend(host, clock=clock)
    backend.connect()
    state.calls.clear()

    for _ in range(100):
        clock.advance(0.1)
        backend.tick(0.1)

    assert not [call for call in state.calls if call[0] == "ping"]
    backend.disconnect()


def test_posing_counts_as_being_alive(firmware: tuple[FakeFirmware, str]) -> None:
    """Any accepted command feeds the firmware watchdog, so a pose already says
    "still here". Sending a ping next to it would be one wasted round trip on a
    link that carries ten of them a second."""
    state, host = firmware
    state.robodog = True
    clock = FakeClock()
    backend = HttpBackend(host, clock=clock)
    backend.connect()
    backend.send(Drive(forward=1, turn=0))
    state.calls.clear()

    for _ in range(20):
        clock.advance(0.1)
        # A pose stops the drive on the robot, so this also proves the feed
        # stops when the move does.
        for leg in LegId:
            backend.send(SetLegTarget(leg, LegTarget(16.0, 95.0, 25.0)))
        backend.tick(0.1)

    assert not [call for call in state.calls if call[0] == "ping"]


def test_the_stock_firmware_is_never_pinged(firmware: tuple[FakeFirmware, str]) -> None:
    """It has no watchdog to feed and answers 500 to `ping` -- a feed there
    would be an error on every tick of every walk."""
    state, host = firmware
    state.robodog = False
    clock = FakeClock()
    backend = HttpBackend(host, clock=clock)
    backend.connect()
    backend.send(Drive(forward=1, turn=0))
    state.calls.clear()

    for _ in range(50):
        clock.advance(0.1)
        backend.tick(0.1)

    assert not [call for call in state.calls if call[0] == "ping"]
    backend.disconnect()


def test_the_watchdog_budget_waits_for_the_firmware_probe(
    firmware: tuple[FakeFirmware, str],
) -> None:
    """Regression: the teach UI E-stopped itself the moment it opened.

    `watchdog timeout (0.500s)` on a robot that had just been recognised as
    running our firmware, whose budget is 3 s. The budget was taken from the
    backend before `connect()`, and before connect a backend that probes for
    its firmware can only report the pessimistic, stock answer -- so a pose,
    which takes longer than the stock budget, latched an E-stop on a perfectly
    healthy robot.
    """
    from robodog.backends.http import STOCK_WATCHDOG, SUGGESTED_WATCHDOG

    state, host = firmware
    state.robodog = True
    backend = HttpBackend(host)
    # What every caller used to read, and why they all read it wrongly.
    assert backend.suggested_watchdog == STOCK_WATCHDOG

    client = RobotClient(backend)
    client.connect()
    assert client.watchdog_timeout == SUGGESTED_WATCHDOG
    client.disconnect()


def test_an_explicit_budget_is_never_overruled(firmware: tuple[FakeFirmware, str]) -> None:
    """Bring-up and calibration set their own, for reasons the transport knows
    nothing about -- a human reading a prompt is not a slow link."""
    state, host = firmware
    state.robodog = True
    client = RobotClient(HttpBackend(host), watchdog_timeout=42.0)
    client.connect()
    assert client.watchdog_timeout == 42.0
    client.disconnect()


def test_a_stock_robot_keeps_the_strict_budget(firmware: tuple[FakeFirmware, str]) -> None:
    """Nothing slow to do there, so nothing to widen the budget for. Widening
    it everywhere would have 'fixed' this bug by blunting the watchdog."""
    from robodog.backends.http import STOCK_WATCHDOG

    state, host = firmware
    state.robodog = False
    client = RobotClient(HttpBackend(host))
    client.connect()
    assert client.watchdog_timeout == STOCK_WATCHDOG
    client.disconnect()
