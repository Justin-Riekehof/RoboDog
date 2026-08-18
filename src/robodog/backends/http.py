"""HttpBackend: the stock WAVEGO firmware over Wi-Fi (bring-up channel).

Speaks the firmware's `GET /control?var=..&val=..&cmd=..` endpoint
(vendor/wavego-firmware/app_httpd.cpp). Only what that handler actually accepts
is implemented — locomotion, function modes and servo trim (ASSUMPTIONS D2).

Two properties of this transport shape the whole class:

* There is **no readback whatsoever** — every response is an empty 200. The
  reported RobotState is therefore a *model* of what the robot should be doing,
  driven by the same kinematics as the simulator, and flagged
  `is_estimated=True`.
* There is **no way to guarantee a stop**. The firmware has no link watchdog and
  Wi-Fi can drop mid-command, after which nothing we send arrives (ASSUMPTIONS
  D10). Operate with the robot on a stand or a hand on the power switch.
"""

from __future__ import annotations

import contextlib
import urllib.error
import urllib.parse
import urllib.request
from typing import Final

from robodog.api.types import (
    Capability,
    Command,
    Drive,
    FunctionMode,
    LegId,
    RobotState,
    SetFunction,
)
from robodog.backends.mock import MockBackend
from robodog.errors import BackendError, CapabilityError
from robodog.kinematics.constants import SERVO_CHANNELS, SERVO_MIDDLE
from robodog.netdiag import diagnose_unreachable

DEFAULT_HOST: Final = "192.168.4.1"
AP_SSID: Final = "WAVESHARE Robot"
AP_PASSWORD: Final = "1234567890"
# An ESP32 serving its own soft access point is slow, and the first request also
# pays for ARP and the TCP handshake. One second is not enough in practice.
DEFAULT_TIMEOUT: Final = 3.0

# Firmware `move` values (ASSUMPTIONS D4).
MOVE_FORWARD: Final = 1
MOVE_TURN_LEFT: Final = 2
MOVE_STOP_FB: Final = 3
MOVE_TURN_RIGHT: Final = 4
MOVE_BACKWARD: Final = 5
MOVE_STOP_LR: Final = 6

_STOP_RETRIES: Final = 2
# A stop attempt over a link that is already gone must fail fast: the operator is
# standing next to a moving robot, and a tool that hangs for half a minute
# retrying is worse than one that says "I cannot reach it, pull the power".
STOP_TIMEOUT: Final = 1.0


class HttpBackend:
    name = "http"
    # Wi-Fi exposes locomotion and the servo-trim facility, and nothing else.
    capabilities = frozenset({Capability.LOCOMOTION, Capability.SERVO_TRIM})

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        *,
        timeout: float = DEFAULT_TIMEOUT,
        opener: urllib.request.OpenerDirector | None = None,
    ) -> None:
        self.host = host
        self.timeout = timeout
        # Bypass any system proxy: the robot is a link-local device on its own
        # access point, and routing 192.168.4.1 through a configured corporate
        # or VPN proxy would simply fail.
        self._opener = (
            opener
            if opener is not None
            else urllib.request.build_opener(urllib.request.ProxyHandler({}))
        )
        self._connected = False
        # True once a stop has been acknowledged and nothing has moved since.
        self._stop_confirmed = False
        # Our belief about the robot, advanced by the same model as the sim.
        self._model = MockBackend()
        self.request_log: list[str] = []

    # --- transport ---

    def _base_url(self) -> str:
        return self.host if "://" in self.host else f"http://{self.host}"

    def _get(self, path: str, *, timeout: float | None = None) -> bytes:
        url = f"{self._base_url()}{path}"
        self.request_log.append(path)
        try:
            with self._opener.open(url, timeout=timeout or self.timeout) as response:
                body: bytes = response.read()
                return body
        except urllib.error.HTTPError as exc:
            raise BackendError(f"{url} returned HTTP {exc.code}") from exc
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            raise BackendError(f"cannot reach robot at {url}: {exc}") from exc

    def _control(self, var: str, val: int, cmd: int = 0, *, timeout: float | None = None) -> None:
        """Issue one /control request. All three keys are mandatory (D3)."""
        query = urllib.parse.urlencode({"var": var, "val": val, "cmd": cmd})
        self._get(f"/control?{query}", timeout=timeout)

    # --- lifecycle ---

    def connect(self) -> None:
        """Assert a known stopped state, which doubles as the reachability probe.

        Deliberately does *not* fetch the index page: that is by far the heaviest
        response the ESP32 serves (a hundred-odd buttons of PROGMEM HTML), while
        `/control` answers with an empty body. Probing with a stop command tests
        exactly the endpoint we rely on, moves nothing, and costs one small packet.
        """
        self._connected = True
        self._model.connect()
        try:
            # Both motion axes latch independently, so a clean start needs both.
            self._control("move", MOVE_STOP_FB)
            self._control("move", MOVE_STOP_LR)
        except BackendError as exc:
            self._connected = False
            self._model.disconnect()
            hints = diagnose_unreachable(self.host, AP_SSID, AP_PASSWORD)
            raise BackendError("\n  ".join([str(exc), *hints])) from exc
        self._model.send(Drive(0, 0))

    def disconnect(self) -> None:
        # Skip the stop when we already know the robot is stopped: repeating it
        # over a dead link only doubles the time spent failing.
        if self._connected and not self._stop_confirmed:
            # Leaving anyway; a failure here only means the link is already bad.
            with contextlib.suppress(BackendError):
                self._stop_motion()
        self._connected = False
        self._model.disconnect()

    def _require_connected(self) -> None:
        if not self._connected:
            raise BackendError("http backend is not connected")

    # --- commands ---

    def send(self, command: Command) -> None:
        self._require_connected()
        self._stop_confirmed = False
        match command:
            case Drive(forward=forward, turn=turn):
                self._send_drive(forward, turn)
            case SetFunction(mode=mode):
                self._control("funcMode", int(mode))
            case _:
                raise CapabilityError(
                    f"{type(command).__name__} is not available over Wi-Fi on the stock "
                    f"firmware (ASSUMPTIONS D2)"
                )
        self._model.send(command)

    def _send_drive(self, forward: int, turn: int) -> None:
        """Map a drive intent onto the firmware's two latched axes."""
        self._control(
            "move",
            {1: MOVE_FORWARD, -1: MOVE_BACKWARD, 0: MOVE_STOP_FB}[forward],
        )
        self._control(
            "move",
            {1: MOVE_TURN_RIGHT, -1: MOVE_TURN_LEFT, 0: MOVE_STOP_LR}[turn],
        )

    def trim_servo(self, channel: int, offset: int) -> None:
        """Nudge one servo by a relative PWM offset (firmware `sconfig`).

        Enters the firmware's debug mode, which suspends gait control until the
        next drive/function command (ASSUMPTIONS D5). Beware the first-call jump
        described in D6: send `sync_servo_baseline()` first.
        """
        self._require_connected()
        if not 0 <= channel <= 15:
            raise BackendError(f"servo channel must be 0..15, got {channel}")
        self._control("sconfig", channel, offset)

    def save_servo_trim(self, channel: int) -> None:
        """Persist one servo's calibrated middle to NVS (firmware `sset`)."""
        self._require_connected()
        self._control("sset", channel)

    def sync_servo_baseline(self) -> None:
        """Send funcMode=9 so the firmware's CurrentPWM[] matches reality (D6).

        Without this, the first trim command per servo snaps it to roughly the
        middle position instead of nudging it.
        """
        self._require_connected()
        self._control("funcMode", int(FunctionMode.MIDDLE_POS))
        self._model.send(SetFunction(FunctionMode.MIDDLE_POS))

    @staticmethod
    def servo_channels(leg: LegId) -> tuple[int, int, int]:
        """(fore, back, wiggle) PCA9685 channels of one leg."""
        return SERVO_CHANNELS[leg]

    @staticmethod
    def baseline_pwm() -> int:
        """PWM count the servos sit at after `sync_servo_baseline()`.

        Only true while the stored calibration is at its default; a trimmed
        robot's baseline is its own ServoMiddlePWM[], which the firmware never
        reports back (ASSUMPTIONS D8).
        """
        return SERVO_MIDDLE

    # --- time & state ---

    def tick(self, dt: float) -> None:
        self._require_connected()
        self._model.tick(dt)

    def state(self) -> RobotState:
        self._require_connected()
        model_state = self._model.state()
        # Same numbers, but relabelled: nothing here was measured.
        return RobotState(
            t=model_state.t,
            drive=model_state.drive,
            body=model_state.body,
            leg_targets=model_state.leg_targets,
            joint_angles=model_state.joint_angles,
            is_estimated=True,
            busy_until=model_state.busy_until,
            telemetry=None,  # the HTTP path returns no data at all (D3)
        )

    # --- safety ---

    def _stop_motion(self) -> None:
        """Stop both latched axes, retrying briefly — this is the safety path."""
        errors: list[str] = []
        for value in (MOVE_STOP_FB, MOVE_STOP_LR):
            for attempt in range(_STOP_RETRIES):
                try:
                    self._control("move", value, timeout=STOP_TIMEOUT)
                    break
                except BackendError as exc:
                    if attempt == _STOP_RETRIES - 1:
                        errors.append(str(exc))
        if errors:
            self._stop_confirmed = False
            raise BackendError(
                "could not confirm stop over Wi-Fi -- if the robot is moving, "
                "cut its power: " + "; ".join(errors)
            )
        self._stop_confirmed = True

    def safe_sequence(self) -> None:
        """Best effort stop. Cannot hold a crouch: the stock firmware over
        Wi-Fi has no pose command, so 'safe' here means standing still."""
        try:
            self._stop_motion()
        finally:
            # Keep the model honest even if the link died mid-stop: the intent
            # is stopped, and stopping is what the firmware does on both stops.
            if self._model_connected():
                self._model.send(Drive(0, 0))

    def _model_connected(self) -> bool:
        try:
            self._model.state()
        except BackendError:
            return False
        return True
