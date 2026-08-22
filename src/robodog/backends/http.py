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

Both change on our own firmware (`firmware/wavego-robodog`), which the backend
detects at connect: it adds a whole-pose command and an on-device watchdog. The
detection is a single `var=ping` request -- the stock firmware answers 500 to a
variable it does not know, ours answers 200. Pass `firmware=` to override it,
because a probe that guesses wrong about *what a robot can be told to do* is
worth being able to overrule by hand.
"""

from __future__ import annotations

import contextlib
import http.client
import json
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Callable
from typing import Final

from robodog.api.types import (
    Capability,
    Command,
    Drive,
    FunctionMode,
    LegId,
    LegTarget,
    RobotState,
    SetCameraParam,
    SetFunction,
    SetLegTarget,
    TrimServo,
)
from robodog.backends.base import DEFAULT_TICK
from robodog.backends.mock import MockBackend
from robodog.camera import STREAM_PATH, STREAM_PORT
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

# What `firmware=` accepts. "auto" probes; the other two state it outright.
FIRMWARE_CHOICES: Final = ("auto", "stock", "robodog")

# Capabilities our own firmware adds on top of the stock set.
ROBODOG_CAPABILITIES: Final = frozenset({Capability.LEG_TARGET, Capability.CAMERA_TUNING})

# How long our firmware may go unheard before it stops itself, in milliseconds.
# It only ever acts on a robot that is actually moving, so a long quiet spell
# while an operator poses and thinks costs nothing -- and a dropped link during
# a walk costs at most this.
FIRMWARE_WATCHDOG_MS: Final = 1500

# How often the host says "still here" while a move is latched.
#
# The firmware watchdog only acts on a robot that is MOVING -- and a latched
# move is the one state in which the host has nothing to say, because the
# firmware walks on by itself until a stop follows (ASSUMPTIONS B3/D4). So the
# single state the watchdog guards is the single state that produces no traffic
# to feed it, and the robot stops itself mid-walk on a perfectly healthy link.
# That inversion is why this exists; "ordinary traffic keeps it alive" is true
# only of posing, which is exactly when the watchdog is asleep.
#
# Three feeds per watchdog period, so two may be lost before the robot acts.
FIRMWARE_FEED_INTERVAL: Final = FIRMWARE_WATCHDOG_MS / 3000.0

# What the host's own watchdog must tolerate on this transport. A pose request
# was measured at 1.0-1.2 s on the robot's own access point (2026-08-22), and
# the supervisor checks its deadline immediately after the request returns --
# so a budget below that latches an E-stop on a perfectly healthy robot, which
# is what happened twice before this number existed.
#
# Raising it is only defensible because the ROBOT now stops itself: with our
# firmware the on-device watchdog is the net that matters for a moving robot,
# and this one is the second line. Against stock firmware it stays strict.
SUGGESTED_WATCHDOG: Final = 3.0
STOCK_WATCHDOG: Final = 0.5

# Seconds per pose this transport can actually sustain. Measured 2026-08-22:
# 94-140 ms per request, so ten a second. Interpolating a routine at the twin's
# 50 Hz and sending every frame does not make the robot smoother -- it makes a
# 3-second bow take fourteen. Playing at the rate the channel has keeps the
# motion the length it was authored to be; it is coarser, not slower.
SUGGESTED_TICK: Final = 0.1

# Bounds on the ramp a pose asks the robot for. The floor keeps a burst of
# poses from asking for a ramp so short it is a jump again; the ceiling keeps a
# stalled sender from leaving the robot creeping towards a pose nobody wants
# any more -- the firmware clamps too, further out.
MIN_RAMP_MS: Final = 40
MAX_RAMP_MS: Final = 400

_STOP_RETRIES: Final = 2
# A stop attempt over a link that is already gone must fail fast: the operator is
# standing next to a moving robot, and a tool that hangs for half a minute
# retrying is worse than one that says "I cannot reach it, pull the power".
STOP_TIMEOUT: Final = 1.0


class HttpBackend:
    name = "http"
    # What the stock firmware offers over Wi-Fi, and the floor for every robot:
    # our own firmware only ever adds to this (see ROBODOG_CAPABILITIES).
    # CAMERA is in the floor because the vendor's own firmware serves the same
    # MJPEG stream -- watching the robot is not what the fork added, tuning the
    # sensor is (CAMERA_TUNING).
    STOCK_CAPABILITIES = frozenset(
        {Capability.LOCOMOTION, Capability.SERVO_TRIM, Capability.CAMERA}
    )

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        *,
        timeout: float = DEFAULT_TIMEOUT,
        firmware: str = "auto",
        clock: Callable[[], float] = time.monotonic,
        opener: urllib.request.OpenerDirector | None = None,
    ) -> None:
        if firmware not in FIRMWARE_CHOICES:
            raise BackendError(
                f"unknown firmware {firmware!r} (valid: {', '.join(FIRMWARE_CHOICES)})"
            )
        self.host = host
        self.timeout = timeout
        self.firmware = firmware
        # Until we have asked the robot, assume the least: a capability claimed
        # and not delivered is a command that vanishes, which is worse than one
        # that is refused.
        self.capabilities = self.STOCK_CAPABILITIES
        # Foot targets accumulate here and go out as one request per tick --
        # `SetLegTarget` is per leg, but the firmware moves a whole pose at once
        # and one round trip beats four.
        self._pose: dict[LegId, LegTarget] = {}
        self._pose_dirty = False
        # What the robot last said its camera holds. None until asked, and on
        # firmware too old to answer.
        self.camera_state: dict[str, int] | None = None
        # When the last pose went out, so the next one can say how long it has.
        self._last_flush: float | None = None
        self._clock = clock
        # When the robot last heard anything from us, and whether it is counting.
        self._last_sent = clock()
        self._watchdog_armed = False
        # Bypass any system proxy: the robot is a link-local device on its own
        # access point, and routing 192.168.4.1 through a configured corporate
        # or VPN proxy would simply fail.
        # None means the pooled connection below. An opener is the escape
        # hatch for anything needing urllib's semantics -- a proxy, a recorder
        # -- and it gives up the held-open connection in exchange.
        self._opener = opener
        self._live: http.client.HTTPConnection | None = None
        self._wire = threading.Lock()
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
        # Any accepted command feeds the firmware watchdog, so this is also the
        # moment the robot last heard from us -- see _feed_firmware_watchdog.
        self._last_sent = self._clock()
        if self._opener is not None:
            try:
                with self._opener.open(url, timeout=timeout or self.timeout) as response:
                    body: bytes = response.read()
                    return body
            except urllib.error.HTTPError as exc:
                raise BackendError(f"{url} returned HTTP {exc.code}") from exc
            except (urllib.error.URLError, OSError, TimeoutError) as exc:
                raise BackendError(f"cannot reach robot at {url}: {exc}") from exc
        return self._get_pooled(url, path, timeout or self.timeout)

    def _get_pooled(self, url: str, path: str, timeout: float) -> bytes:
        """One connection, held open across requests.

        A pose is a request, and a request used to be a fresh TCP connection: a
        handshake to an ESP32 over Wi-Fi is a real part of the 94-140 ms a pose
        costs, paid again on every one. Holding the connection open removes it
        from all but the first.

        Serialised deliberately. The robot answers one request at a time
        whatever we do, and a shared connection is not thread-safe -- the teach
        UI drives this from its ticker and its request handlers at once.

        Retried once, and only on a connection error: an idle connection is the
        server's to close, and finding out that it did is not a failure. A 500
        is an answer and is never retried -- the robot refusing a command twice
        would be the robot doing what it was told, twice.
        """
        with self._wire:
            for attempt in (1, 2):
                try:
                    connection = self._connection(timeout)
                    connection.request("GET", path)
                    response = connection.getresponse()
                    body = response.read()  # always, or the connection is unusable
                    if response.status >= 400:
                        raise BackendError(f"{url} returned HTTP {response.status}")
                    return body
                except (OSError, http.client.HTTPException) as exc:
                    self._drop_connection()
                    if attempt == 2:
                        raise BackendError(f"cannot reach robot at {url}: {exc}") from exc
            raise AssertionError("unreachable")  # pragma: no cover

    def _connection(self, timeout: float) -> http.client.HTTPConnection:
        if self._live is None or self._live.timeout != timeout:
            self._drop_connection()
            parsed = urllib.parse.urlsplit(self._base_url())
            self._live = http.client.HTTPConnection(
                parsed.hostname or self.host, parsed.port or 80, timeout=timeout
            )
        return self._live

    def _drop_connection(self) -> None:
        if self._live is not None:
            with contextlib.suppress(OSError):
                self._live.close()
            self._live = None

    def _control(
        self,
        var: str,
        val: int,
        cmd: int = 0,
        *,
        timeout: float | None = None,
        extra: str = "",
    ) -> bytes:
        """Issue one /control request. All three keys are mandatory (D3).

        `extra` appends already-encoded key/value pairs, which is how a whole
        pose travels in one request without twelve more parameters here.
        """
        query = urllib.parse.urlencode({"var": var, "val": val, "cmd": cmd})
        if extra:
            query = f"{query}&{extra}"
        return self._get(f"/control?{query}", timeout=timeout)

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
        ours = self._detect_robodog_firmware()
        self.capabilities = self.STOCK_CAPABILITIES | (
            ROBODOG_CAPABILITIES if ours else frozenset()
        )
        if ours:
            # Arm the robot's own watchdog. It only acts while the robot is
            # moving, so a long pause during teach-in never trips it -- but a
            # moving robot is exactly what we send nothing to, so it needs
            # feeding: see _feed_firmware_watchdog.
            try:
                self._control("watchdog", FIRMWARE_WATCHDOG_MS)
                self._watchdog_armed = True
            except BackendError as exc:
                # Only reachable when the firmware was declared rather than
                # probed. Failing loudly is the point: the operator asserted a
                # robot that stops itself, and it does not.
                self._connected = False
                self._model.disconnect()
                raise BackendError(
                    f"this robot refused the watchdog command, so it is not running "
                    f"{self.firmware!r} firmware -- drop --firmware, or flash "
                    f"firmware/wavego-robodog ({exc})"
                ) from exc
        self._pose = dict(self._model.state().leg_targets)
        self._pose_dirty = False

    def _remember_camera(self, body: bytes) -> bool:
        """Keep the last camera state the robot reported.

        Every `cam_*` reply carries it, so this costs nothing and needs no
        second request. A body that is not the JSON we expect is ignored rather
        than raised on: an older firmware answers those requests with an empty
        200, and losing the readback is not a reason to fail the write that
        already succeeded.
        """
        try:
            state = json.loads(body)
        except (ValueError, TypeError):
            return False
        if not (isinstance(state, dict) and state.get("camera")):
            return False
        self.camera_state = {
            key: int(value)
            for key, value in state.items()
            if key != "camera" and isinstance(value, int)
        }
        return True

    def read_camera_state(self) -> dict[str, int] | None:
        """Ask the robot what the camera holds, rather than what we asked for.

        `cam_report` changes nothing on the sensor -- it is the firmware's own
        "say what you have" -- so this is safe to call at any time. Returns None
        where the firmware is too old to answer with a body.

        None means *this* read produced nothing, even when an earlier one did:
        handing back the last known state would report freshness that does not
        exist, and a page would go on calling a stale memory a readback.
        """
        if Capability.CAMERA_TUNING not in self.capabilities:
            return None
        fresh = False
        with contextlib.suppress(BackendError):
            fresh = self._remember_camera(self._control("cam_report", 0))
        return self.camera_state if fresh else None

    @property
    def stream_url(self) -> str:
        """Where the robot's MJPEG stream lives.

        The firmware starts a second server one port above the control one
        (`config.server_port += 1`), so the port is derived rather than assumed:
        a control host on a non-standard port has its stream one above that too,
        which is what makes this reachable in a test.
        """
        host = self.host.split("://", 1)[-1].split("/", 1)[0]
        name, _, port = host.rpartition(":")
        if name and port.isdigit():
            return f"http://{name}:{int(port) + 1}{STREAM_PATH}"
        return f"http://{host}:{STREAM_PORT}{STREAM_PATH}"

    @property
    def suggested_tick(self) -> float:
        """Seconds per player tick this transport can keep up with.

        Only poses stream; stock firmware has no pose command, so nothing there
        is paced by the transport and the ordinary default applies.
        """
        return SUGGESTED_TICK if Capability.LEG_TARGET in self.capabilities else DEFAULT_TICK

    @property
    def suggested_watchdog(self) -> float:
        """Host-side watchdog budget this backend needs to work at all.

        The supervisor's watchdog measures time between feeds, and a request on
        this transport IS that time -- a pose takes about a second. Sizing the
        budget below the transport's latency does not make anything safer; it
        just E-stops a healthy robot mid-pose.
        """
        return SUGGESTED_WATCHDOG if Capability.LEG_TARGET in self.capabilities else STOCK_WATCHDOG

    def disconnect(self) -> None:
        # Disarm the robot's watchdog on the way out. Leaving it armed would
        # stop the robot mid-walk for whoever drives it next from the vendor's
        # own web page, which sends nothing while it walks.
        if self._connected and Capability.LEG_TARGET in self.capabilities:
            with contextlib.suppress(BackendError):
                self._control("watchdog", 0)
            self._watchdog_armed = False
        self._drop_connection()
        # Skip the stop when we already know the robot is stopped: repeating it
        # over a dead link only doubles the time spent failing.
        if self._connected and not self._stop_confirmed:
            # Leaving anyway; a failure here only means the link is already bad.
            with contextlib.suppress(BackendError):
                self._stop_motion()
        self._connected = False
        self._model.disconnect()

    def _detect_robodog_firmware(self) -> bool:
        """Ask the robot which firmware it runs, unless we were told.

        The probe is `var=ping`, which our firmware answers with 200 and the
        stock one with 500 -- its handler ends in `res = -1` for any variable it
        does not know. `ping` is deliberate: of our added commands it is the
        only one that changes nothing at all, so probing can never move a robot.

        A failed probe means "stock". Erring the other way would claim a
        capability the robot cannot honour, and the command would be swallowed
        by a 500 somewhere below the safety layer instead of refused above it.
        """
        if self.firmware != "auto":
            return self.firmware == "robodog"
        try:
            self._control("ping", 0)
        except BackendError:
            return False
        return True

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
            case TrimServo(channel=channel, offset=offset):
                self.trim_servo(channel, offset)
            case SetCameraParam(name=name, value=value):
                # Straight through: the sensor holds this itself, there is
                # nothing to stage and nothing to flush. The reply carries the
                # resulting state, so a write confirms itself.
                self._remember_camera(self._control(f"cam_{name}", value))
            case SetLegTarget(leg=leg, target=target):
                # Staged, not sent: the player writes all four legs and then
                # ticks, so flushing per tick turns a pose into one request.
                self._pose[leg] = target
                self._pose_dirty = True
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
        self.flush_pose()
        self._feed_firmware_watchdog()
        self._model.tick(dt)

    def _feed_firmware_watchdog(self) -> None:
        """Tell the robot we are still here, while and only while it is moving.

        Without this the on-device watchdog stops a walk on a healthy link: it
        acts only on a moving robot, and a moving robot is precisely what we
        send nothing to, because the move latches in the firmware and walks on
        by itself. Any command counts as a feed, so this is needed exactly when
        there is no command to send -- hence a `ping`, which changes nothing.

        Feeding does not defeat the watchdog, it is what gives it its meaning.
        Before this it asked "has the host said anything lately", which conflates
        being alive with having something to say. Now it asks "is the host still
        there and does it still want this move" -- and when the host dies mid-walk
        the pings stop with it, which is the case the watchdog exists for.
        """
        if not self._watchdog_armed:
            return
        drive = self._model.state().drive
        if drive.forward == 0 and drive.turn == 0:
            return
        if self._clock() - self._last_sent < FIRMWARE_FEED_INTERVAL:
            return
        self._control("ping", 0)

    def flush_pose(self) -> None:
        """Send the staged foot targets, if any changed since the last flush.

        One request carries all twelve values and lands in 78-94 ms on the
        robot's own access point (measured 2026-08-22). Sending only on change
        matters: the teach UI ticks at 50 Hz while the operator thinks, and
        every unchanged tick would otherwise be another round trip.
        """
        if not self._pose_dirty:
            return
        self._pose_dirty = False
        if Capability.LEG_TARGET not in self.capabilities:
            raise CapabilityError(
                "this robot runs the stock firmware, which has no pose command "
                "(ASSUMPTIONS D2); flash firmware/wavego-robodog for LEG_TARGET"
            )
        query: list[str] = []
        for leg in LegId:
            target = self._pose.get(leg)
            if target is None:
                raise BackendError(f"no target staged for {leg.name}")
            query += [
                f"l{int(leg)}x={target.x:.2f}",
                f"l{int(leg)}y={target.y:.2f}",
                f"l{int(leg)}z={target.z:.2f}",
            ]
        # `val` is how long the robot should take to get there. It ramps
        # GoalPWM across that inside its own 4 ms loop, so a pose stops being a
        # step: five poses a second from here become hundreds on the robot.
        #
        # The duration is the interval we are actually achieving, measured
        # rather than assumed -- a link that slows down gets longer ramps by
        # itself, and the robot is still travelling when the next pose lands,
        # which is what keeps a stream continuous instead of a series of
        # arrivals. First flush has nothing to measure and uses the nominal.
        now = self._clock()
        gap = now - self._last_flush if self._last_flush is not None else self.suggested_tick
        self._last_flush = now
        # Rounded, not truncated: truncation biases every ramp short, so the
        # robot arrives a fraction early on each pose and stands still for the
        # remainder -- which is the step, reintroduced a millisecond at a time.
        span = min(max(round(gap * 1000), MIN_RAMP_MS), MAX_RAMP_MS)
        self._control("pose", span, extra="&".join(query))
        # A pose ends a latched move, and the model has to know. The firmware
        # clears moveFB/moveLR when it applies one (`robodogApply`), because a
        # gait rewrites the servos every pass and would walk straight out of the
        # pose it was just handed. Without this the host would go on reporting a
        # move the robot dropped -- and the teach page would show it.
        self._model.send(Drive(0, 0))

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
