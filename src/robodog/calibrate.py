"""Guided roll-range calibration over the stock firmware's servo-trim path.

The wiggle servo swings a leg's whole plane sideways; how far it can actually
go under power is unknown (ASSUMPTIONS C13). Hand-moving an unpowered leg
back-drives the gearbox and proves nothing about the commandable range, so this
procedure asks the robot instead: nudge the servo by small PWM steps, and after
each one the operator says whether the leg still followed.

Why PWM counts and not degrees: over Wi-Fi the stock firmware has no joint-level
channel at all. `sconfig` is a *calibration* facility that speaks relative PWM
(ASSUMPTIONS D5), and it is the only thing that moves a single joint. Counts are
therefore the honest unit here -- the counts-per-degree factor is itself only an
assumption (C2), and comparing the measured stop against it is a side benefit of
this run rather than something it depends on.

Two things this deliberately never does:

* **`sset` is never sent.** It writes a servo's middle position to NVS
  permanently; a sweep that touched it would leave the robot's zero displaced.
  Nothing in this codebase routes that command.
* **It never jumps.** Every step goes through the safety supervisor, whose
  per-command trim bound caps how far one nudge can travel. Coarse steps find
  the neighbourhood, fine steps find the edge, and the servo is backed off the
  moment it stops following so it does not sit stalled against its end stop.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import date
from functools import partial
from pathlib import Path

from robodog.api.client import RobotClient
from robodog.api.types import Capability, FunctionMode, LegId, SafetyState
from robodog.backends.http import HttpBackend
from robodog.calibration import (
    EMPTY,
    JOINTS,
    MAX_OFFSET,
    SUSPICIOUS_OFFSET,
    ServoCalibration,
    channel_of,
    counts_to_degrees,
    step_degrees,
    step_millimetres,
)
from robodog.errors import BackendError, EStopActiveError, RobodogError
from robodog.kinematics.constants import (
    SERVO_CHANNELS,
)
from robodog.teach.format import LEG_IDS_TO_NAMES, LEG_NAMES

Asker = Callable[[str], str]
Printer = Callable[[str], None]

# Paced by a human answering a question after every step, so the watchdog needs
# the same head room the bring-up gives it: it exists to catch a dead control
# loop, not a thinking operator.
CALIBRATION_WATCHDOG = 15.0

# Counts per step, at ~2.22 counts/degree (ASSUMPTIONS C2). The fine step is
# bounded from below by what a human can actually SEE: 2 counts is 0.9 deg,
# about 1.8 mm at the foot, which is not reliably visible -- answers at that
# scale are noise, and a run built on them produces a confident wrong number.
# 5 counts is 2.25 deg, ~4.5 mm at the foot.
DEFAULT_COARSE_STEP = 20
DEFAULT_FINE_STEP = 5
# Below this, warn that the operator is being asked to see what cannot be seen.
VISIBLE_STEP_MIN = 4

_YES = ("y", "yes", "j", "ja")
_NO = ("n", "no", "nein")


@dataclass(slots=True)
class DirectionResult:
    """How far one servo followed in one direction before it stopped."""

    sign: int
    last_following_counts: int
    reached_end_stop: bool
    note: str = ""
    # Set when the very first step did not move: from the middle position that
    # cannot be an end stop, so the figure must not become a limit.
    suspicious: bool = False
    # The operator ended this direction deliberately. Not a reason to discard
    # what was measured up to that point -- stopping is not invalidating.
    stopped: bool = False

    def degrees(self, channel: int) -> float:
        return counts_to_degrees(self.last_following_counts * self.sign, channel)


@dataclass(slots=True)
class LegResult:
    leg: LegId
    channel: int
    directions: list[DirectionResult] = field(default_factory=list)

    def roll_bounds(self) -> tuple[float | None, float | None]:
        """Measured (roll_min, roll_max) in degrees, where both ends were found."""
        angles = [
            d.degrees(self.channel)
            for d in self.directions
            if d.reached_end_stop and not d.suspicious
        ]
        if not angles:
            return None, None
        low, high = min(angles), max(angles)
        return (low if low < 0 else None), (high if high > 0 else None)


@dataclass(slots=True)
class CalibrationReport:
    host: str
    coarse_step: int
    fine_step: int
    legs: list[LegResult] = field(default_factory=list)
    aborted: str = ""

    def to_markdown(self, today: date) -> str:
        lines = [
            f"# Roll calibration report {today.isoformat()}",
            "",
            f"Host `{self.host}`, steps: coarse {self.coarse_step} counts, "
            f"fine {self.fine_step} counts.",
            "",
            "Measured on the **wiggle** servo of each leg, by nudging it with the stock",
            "firmware's `sconfig` facility and asking after every step whether the leg",
            "still followed. `sset` was never sent, so the stored servo calibration is",
            "unchanged (ASSUMPTIONS D5/D8).",
            "",
        ]
        if self.aborted:
            lines += [f"**Run aborted:** {self.aborted}", ""]
        if not self.legs:
            lines += ["No leg was measured.", ""]
            return "\n".join(lines)

        lines += [
            "| Leg | Channel | Direction | Last following | Angle | End stop | Note |",
            "|---|---|---|---|---|---|---|",
        ]
        for leg in self.legs:
            for d in leg.directions:
                sign = "+" if d.sign > 0 else "-"
                lines.append(
                    f"| {LEG_IDS_TO_NAMES[leg.leg]} | {leg.channel} | {sign} | "
                    f"{d.last_following_counts} counts | {d.degrees(leg.channel):+.1f} deg | "
                    f"{'yes' if d.reached_end_stop else 'not reached'} | {d.note or '--'} |"
                )
        lines += ["", "## Suggested limits", ""]
        lows = [b[0] for leg in self.legs for b in [leg.roll_bounds()] if b[0] is not None]
        highs = [b[1] for leg in self.legs for b in [leg.roll_bounds()] if b[1] is not None]
        if lows and highs:
            # The tightest end across the legs measured: a limit is only safe if
            # it holds for every leg, and the legs need not be identical.
            lines += [
                "Taking the tightest end over every leg measured, so the limit holds for",
                "all of them:",
                "",
                "```python",
                f"LimitConfig(roll_min={max(lows):.1f}, roll_max={min(highs):.1f})",
                "```",
                "",
                "Carry these into `LimitConfig` and update ASSUMPTIONS C13 with this run",
                "as the source. Note what they are *not*: they are the range the servo",
                "drives under no load, on a stand. Under the robot's own weight, and with",
                "the legs able to reach each other and the body, the usable range is",
                "smaller and nothing here checks for that.",
            ]
        elif lows or highs:
            found, missing = ("lower", "upper") if lows else ("upper", "lower")
            other = "up" if lows else "down"
            lines += [
                f"Only the {found} end was established in this run "
                f"({max(lows) if lows else min(highs):+.1f} deg). Measure the {missing} end",
                f"with `--direction {other}`, then combine the two reports -- a limit needs",
                "both ends.",
            ]
        else:
            lines += [
                "No trustworthy end was established. A direction whose *first* step did not",
                "move the leg is not evidence of an end stop, and a sweep stopped short of a",
                "limit is not either -- see the notes above and re-run those directions.",
            ]
        lines.append("")
        return "\n".join(lines)


def _ask_yes_no(ask: Asker, say: Printer, question: str) -> bool | None:
    """True/False, or None when the operator wants to stop."""
    while True:
        answer = ask(question).strip().lower()
        if answer in _YES:
            return True
        if answer in _NO:
            return False
        if answer in ("s", "stop", "q", "quit", "abort"):
            return None
        say("  Please answer y (moved), n (did not move) or s (stop).")


def _send(client: RobotClient, say: Printer, action: Callable[[], None]) -> None:
    """Feed the watchdog, then send -- recovering once from a latched E-stop.

    An operator spends seconds, sometimes minutes, between steps: looking at the
    leg, deciding what they saw, typing. That is not a dead control loop, which
    is what the watchdog exists to catch. Feeding immediately before each
    command leaves it only the command's own round trips to cover -- the same
    arrangement bring-up uses. Without it a human-paced procedure trips the
    watchdog on its very first pause.
    """
    client.heartbeat()
    try:
        action()
    except EStopActiveError:
        say("  E-stop was latched -- releasing it and retrying this step.")
        client.reset()
        client.arm()
        client.heartbeat()
        action()


def _leave_middle_pos_loop(client: RobotClient, channel: int, say: Printer) -> None:
    """Stop the firmware re-asserting the middle position, without moving anything.

    funcMode 2..7 clear themselves once they have run; **funcMode 8 and 9 do
    not** (`ServoCtrl.h` robotCtrl). After our baseline the control loop
    therefore keeps calling `middlePosAll()` forever, rewriting `CurrentPWM[]`
    for all 16 servos hundreds of times a second. A trim sent into that is
    applied and then immediately overwritten -- the leg does not move, and the
    operator correctly reports that it did not, which the sweep would otherwise
    read as an end stop.

    A zero-offset `sconfig` is the way out: the HTTP handler sets `debugMode=1`
    and `funcMode=0` before touching the servo, so this suspends the control
    loop while adding 0 counts to the servo (ASSUMPTIONS D11).
    """
    say("  Taking the firmware out of its middle-position loop (zero-offset trim) ...")
    _send(client, say, partial(client.trim_servo, channel, 0))


def _baseline(client: RobotClient, say: Printer) -> None:
    """Send funcMode 9 so the firmware's CurrentPWM matches reality (D6).

    Without it the first trim command per servo snaps to the middle instead of
    nudging, which would be a large unannounced motion.

    "Middle" is the firmware's word and it means each *servo* at its middle PWM
    count -- which geometrically is a nearly straight leg reaching ~115 mm, some
    5 mm past the walking envelope, not a neutral mid-pose (ASSUMPTIONS C12).
    Saying so plainly matters: an operator who expects a tucked stance sees
    fully extended legs, concludes the robot misbehaved, and aborts a run that
    was going exactly right.
    """
    say("  Sending funcMode 9 (every servo to its middle PWM count) ...")
    say("  EXPECT: the legs STRAIGHTEN and extend nearly vertically downwards,")
    say("  and the body sits HIGHER than when walking. That is correct -- the")
    say("  firmware's 'middle' is per servo, not a neutral pose (ASSUMPTIONS C12).")
    _send(client, say, partial(client.set_function, FunctionMode.MIDDLE_POS))


def _sweep_direction(
    client: RobotClient,
    channel: int,
    sign: int,
    *,
    coarse: int,
    fine: int,
    ask: Asker,
    say: Printer,
) -> DirectionResult:
    """Step one servo one way until it stops following, or the operator ends it.

    Always returns what was measured: an interrupted sweep still carries real
    counts, and throwing them away would mean re-driving a servo into its stop
    to learn something already known.
    """
    label = "outward/up" if sign > 0 else "inward/down"
    say("")
    say(f"  Direction {'+' if sign > 0 else '-'} ({label}), coarse steps of {coarse} counts.")
    say("  After each step: did the leg move? y / n / s to stop.")

    travelled = 0
    reached = False
    flagged = False

    for step, phase in ((coarse, "coarse"), (fine, "fine")):
        while True:
            try:
                _send(client, say, partial(client.trim_servo, channel, step * sign))
            except RobodogError as exc:
                say(f"  Command refused: {exc}")
                return DirectionResult(sign, travelled, reached, note=str(exc))
            moved = _ask_yes_no(ask, say, f"    [{phase} +{step}] did it move? ")
            if moved is None:
                # Relieve whatever the last command asked for before leaving.
                _relieve(client, channel, step * sign, say)
                # Stopping is not invalidating: keep what was measured. Whether
                # it counts as an end stop is the one thing only the operator
                # knows, so ask rather than guess.
                at_limit = _ask_yes_no(
                    ask, say, "    Stopped. Was the leg against its mechanical limit? "
                )
                degrees = counts_to_degrees(travelled * sign, channel)
                say(f"  Recorded: {travelled} counts ({degrees:+.1f} deg)")
                return DirectionResult(
                    sign,
                    travelled,
                    reached_end_stop=at_limit is True,
                    stopped=True,
                    note=(
                        "operator stopped at the mechanical limit"
                        if at_limit is True
                        else "operator stopped before reaching a limit"
                    ),
                )
            if moved:
                travelled += step
                continue
            if travelled == 0 and phase == "coarse":
                say("")
                say("  The FIRST step did not move the leg. From the middle position")
                say("  that should not be an end stop, so treat this as suspicious:")
                say("   * is this really the wiggle servo? (channel map is ASSUMPTIONS C3)")
                say("   * is the leg fouling the body or the stand?")
                say("   * did the firmware overwrite the trim? (ASSUMPTIONS D11)")
                say("  Recording it, but the report will flag this direction.")
            # It stopped following: back the last step out so the servo is not
            # left pushing against its end stop, then refine (or finish).
            _relieve(client, channel, step * sign, say)
            reached = True
            if travelled == 0 and phase == "coarse":
                flagged = True
            break
        if phase == "coarse" and not reached:
            break

    degrees = counts_to_degrees(travelled * sign, channel)
    say(f"  Last following offset: {travelled} counts ({degrees:+.1f} deg)")
    note = "first step did not move: not trustworthy as an end stop" if flagged else ""
    return DirectionResult(sign, travelled, reached, note=note, suspicious=flagged)


def _relieve(client: RobotClient, channel: int, last_offset: int, say: Printer) -> None:
    """Undo the step that did not take, so nothing sits stalled."""
    try:
        _send(client, say, partial(client.trim_servo, channel, -last_offset))
    except RobodogError as exc:
        say(f"  WARNING: could not back the servo off: {exc}")
        say("  Cut power if the servo is buzzing or getting warm.")


def run_roll_calibration(
    host: str,
    *,
    ask: Asker,
    say: Printer,
    legs: tuple[LegId, ...] = (LegId.FRONT_LEFT,),
    directions: tuple[int, ...] = (1, -1),
    coarse_step: int = DEFAULT_COARSE_STEP,
    fine_step: int = DEFAULT_FINE_STEP,
    client_factory: Callable[[str], RobotClient] | None = None,
) -> CalibrationReport:
    """Measure the wiggle servo's usable range, one leg at a time."""
    report = CalibrationReport(host=host, coarse_step=coarse_step, fine_step=fine_step)

    say("")
    say("=" * 72)
    say(" WAVEGO roll calibration (wiggle servo, stock firmware)")
    say("=" * 72)
    say("")
    say(" This drives ONE servo at a time towards its mechanical stop.")
    if fine_step < VISIBLE_STEP_MIN:
        say("")
        say(f" WARNING: a fine step of {fine_step} counts is about")
        say(f" {step_degrees(fine_step):.1f} deg -- roughly")
        say(f" {step_degrees(fine_step) * 2:.1f} mm at the foot. If you cannot")
        say(" reliably SEE that, your answers become guesses and the measured")
        say(" range will be confidently wrong. Consider --fine-step 5 or more.")
    say("")
    say(" SAFETY, read before continuing:")
    say("  * Put the robot ON A STAND, legs hanging free. A leg that folds up")
    say("    will hit the body and the other legs; nothing here detects that.")
    say("  * Keep a hand near the power switch. A servo held against its end")
    say("    stop draws current and heats up -- if one buzzes or stops")
    say("    following, answer 'n' immediately.")
    say("  * The firmware reports nothing back. Your eyes are the only sensor,")
    say("    so answer only what you actually saw move.")
    say("  * `sset` is never sent: your stored servo calibration is not touched.")
    say("  * The run starts by straightening every leg (funcMode 9). The legs")
    say("    end up nearly vertical and the body higher than usual -- expected,")
    say("    and it is also the pose the wiggle servo works hardest in, so any")
    say("    range you measure is a conservative lower bound (ASSUMPTIONS C12).")
    say("")
    if ask("Type 'yes' when the robot is secured and you are ready: ").strip().lower() not in _YES:
        report.aborted = "operator did not confirm the safety prompt"
        say("Aborted; nothing was sent.")
        return report

    factory = client_factory if client_factory is not None else _default_client_factory
    client = factory(host)

    try:
        client.connect()
    except RobodogError as exc:
        report.aborted = f"cannot reach the robot: {exc}"
        say(f"\nCannot reach the robot: {exc}")
        say("Check that you are on the robot's access point and that the host is right.")
        return report

    if Capability.SERVO_TRIM not in client.capabilities:
        report.aborted = f"backend {client.backend_name} has no servo trim"
        say(f"\nBackend {client.backend_name} cannot trim servos; nothing to do.")
        client.disconnect()
        return report

    try:
        client.arm()
        _baseline(client, say)
        baseline_question = "  Did all four legs straighten and extend downwards? "
        if _ask_yes_no(ask, say, baseline_question) is not True:
            report.aborted = "baseline (funcMode 9) not confirmed"
            say("  Without a known baseline every offset below would be meaningless.")
            say("  If the legs DID straighten and you answered no because that looked")
            say("  wrong: that is the expected result, re-run and confirm it.")
            return report

        for leg in legs:
            _, _, wiggle_channel = SERVO_CHANNELS[leg]
            result = LegResult(leg=leg, channel=wiggle_channel)
            say("")
            say("-" * 72)
            say(f" Leg {LEG_IDS_TO_NAMES[leg]}, wiggle servo on channel {wiggle_channel}")
            say("-" * 72)
            _leave_middle_pos_loop(client, wiggle_channel, say)
            for sign in directions:
                direction = _sweep_direction(
                    client,
                    wiggle_channel,
                    sign,
                    coarse=coarse_step,
                    fine=fine_step,
                    ask=ask,
                    say=say,
                )
                result.directions.append(direction)
                if direction.stopped:
                    report.aborted = "stopped by the operator"
                    report.legs.append(result)
                    return report
                _baseline(client, say)
                _leave_middle_pos_loop(client, wiggle_channel, say)
            report.legs.append(result)
    except KeyboardInterrupt:
        report.aborted = "interrupted"
        say("\nInterrupted -- returning the servos to their middle position.")
        raise
    except RobodogError as exc:
        # A failed run is still evidence: record why and let the caller write
        # the report, rather than losing it to a traceback.
        report.aborted = str(exc)
        say(f"Run failed: {exc}")
    finally:
        _restore(client, say)
        client.disconnect()

    return report


def _restore(client: RobotClient, say: Printer) -> None:
    """Always leave the robot at its calibrated middle, never mid-sweep.

    This runs in a `finally`, so it has to cope with the very states that
    brought it there -- above all a latched E-stop, which would otherwise
    refuse the one command that puts the legs back somewhere known.
    """
    try:
        if client.safety_state is SafetyState.ESTOPPED:
            client.reset()
            client.arm()
        client.heartbeat()
        client.set_function(FunctionMode.MIDDLE_POS)
    except (RobodogError, BackendError) as exc:
        say(f"  WARNING: could not restore the middle position: {exc}")
        say("  Check the legs before driving the robot again.")


def _default_client_factory(host: str) -> RobotClient:
    return RobotClient(HttpBackend(host), watchdog_timeout=CALIBRATION_WATCHDOG)


DIRECTION_ARGS: dict[str, tuple[int, ...]] = {
    "both": (1, -1),
    "up": (1,),
    "down": (-1,),
}


def parse_direction_arg(value: str) -> tuple[int, ...]:
    """Which way to sweep. Measuring one end at a time is normal.

    An end that has already been found does not need finding again, and each
    sweep leaves the servo pressed against a stop for a moment.
    """
    key = value.strip().lower()
    if key not in DIRECTION_ARGS:
        valid = ", ".join(DIRECTION_ARGS)
        raise ValueError(f"unknown direction {value.strip()!r} (valid: {valid})")
    return DIRECTION_ARGS[key]


def parse_legs_arg(value: str) -> tuple[LegId, ...]:
    """`all`, or a comma-separated list of leg names."""
    if value.strip().lower() == "all":
        return tuple(LegId)
    legs = []
    for name in value.split(","):
        key = name.strip().lower()
        if key not in LEG_NAMES:
            valid = ", ".join(sorted(LEG_NAMES))
            raise ValueError(f"unknown leg {name.strip()!r} (valid: {valid}, all)")
        legs.append(LEG_NAMES[key])
    if not legs:
        raise ValueError("no leg given")
    return tuple(legs)


def write_report(report: CalibrationReport, directory: Path, today: date) -> Path:
    """Write the report, never overwriting an earlier run (same rule as bring-up)."""
    directory.mkdir(parents=True, exist_ok=True)
    stem = f"roll-calibration-{today.isoformat()}"
    path = directory / f"{stem}.md"
    run = 2
    while path.exists():
        path = directory / f"{stem}-run{run}.md"
        run += 1
    path.write_text(report.to_markdown(today), encoding="utf-8")
    return path


# --- servo zero calibration (the M3 offset table) --------------------------------


# What "aligned" means, per joint, in terms the operator can actually see. The
# geometry is not a guess: at fore=0/back=0 the planar FK puts both upper links
# straight down (servo axis (+/-6.1, 0) -> elbow (+/-6.1, 40)), and at wiggle=0
# the foot sits 19.2 mm outboard purely because of the linkage width.
ZERO_REFERENCE: dict[str, tuple[str, ...]] = {
    "fore": (
        "The FRONT upper arm -- the 40 mm link from the servo horn to the elbow --",
        "must hang EXACTLY VERTICAL.",
    ),
    "back": (
        "The REAR upper arm must hang EXACTLY VERTICAL. Judge it against vertical",
        "itself, NOT against the front arm -- that one was just judged too, and",
        "using it as the reference would carry its error into this number. Once",
        "both are set they should be parallel, 12 mm apart: that is the check.",
    ),
    "wiggle": (
        "The whole leg PLANE must hang vertical -- no sideways lean. The foot sits",
        "about 19 mm outboard even at zero; that is the linkage width, not a tilt.",
    ),
}

DEFAULT_ZERO_STEP = 5


@dataclass(slots=True)
class ZeroResult:
    """Where one servo's zero turned out to be, relative to the firmware middle."""

    leg: LegId
    joint: str
    channel: int
    offset: int = 0
    steps: int = 0
    skipped: bool = False
    note: str = ""
    # What the stored table said before this run, when it said anything. A
    # re-measurement is the only repeatability check this procedure can have:
    # nothing on the robot can be read back, so the second opinion has to be
    # the same operator, later.
    previous: int | None = None

    def degrees(self) -> float:
        return counts_to_degrees(self.offset, self.channel)

    @property
    def drift(self) -> int | None:
        """How far this run landed from the stored value, in counts."""
        return None if self.previous is None else self.offset - self.previous


@dataclass(slots=True)
class ZeroReport:
    host: str
    step: int
    results: list[ZeroResult] = field(default_factory=list)
    aborted: str = ""
    # A rehearsal against the mock backend produces the same shape of numbers
    # from nothing at all. It must be impossible to mistake for a measurement.
    rehearsal: bool = False

    @property
    def measured(self) -> list[ZeroResult]:
        return [r for r in self.results if not r.skipped]

    @property
    def remeasured(self) -> list[ZeroResult]:
        return [r for r in self.measured if r.previous is not None]

    def disagreements(self) -> list[ZeroResult]:
        """Re-measurements that landed further from the stored value than the
        step size -- i.e. further than this procedure claims to be able to see."""
        return [r for r in self.remeasured if abs(r.drift or 0) > self.step]

    def to_markdown(self, today: date) -> str:
        lines = [
            f"# Servo zero calibration {today.isoformat()}",
            "",
        ]
        if self.rehearsal:
            lines += [
                "> **REHEARSAL against the mock backend -- no robot was touched.**",
                "> The numbers below came from a simulation of the procedure, not",
                "> from a machine. They are not a calibration.",
                "",
            ]
        lines += [
            f"Host `{self.host}`, nudge step {self.step} counts "
            f"({step_degrees(self.step):.2f} deg, "
            f"{step_millimetres(self.step):.1f} mm at the foot, per step).",
            "",
            "**That step is the tolerance.** A zero was placed by eye against a",
            "vertical link, so every offset below means *within one step* -- an",
            "offset of 0 is 'zero to within "
            f"{step_degrees(self.step):.2f} deg', never 'exactly zero'.",
            "",
            "Offsets are PWM counts from the firmware's middle position",
            "(`funcMode=9`) to the angle the kinematics calls zero. The degree",
            "column applies the channel's direction sign (ASSUMPTIONS C2/C3).",
            "",
            "| leg | joint | channel | offset (counts) | offset (deg) | steps | note |",
            "| --- | --- | --- | --- | --- | --- | --- |",
        ]
        for result in self.results:
            offset = "skipped" if result.skipped else f"{result.offset:+d}"
            degrees = "--" if result.skipped else f"{result.degrees():+.2f}"
            lines.append(
                f"| {result.leg.name.lower()} | {result.joint} | {result.channel} | "
                f"{offset} | {degrees} | {result.steps} | {result.note or ''} |"
            )
        if self.remeasured:
            lines += [
                "",
                "## Repeatability",
                "",
                "Joints that had a stored value before this run. The procedure",
                f"claims to place a zero to within {self.step} counts, so a",
                "disagreement larger than that means the reference is harder to",
                "judge than the step size suggests -- treat those numbers as soft.",
                "",
                "| leg | joint | stored | now | difference |",
                "| --- | --- | --- | --- | --- |",
            ]
            for result in self.remeasured:
                flag = " **beyond the step**" if abs(result.drift or 0) > self.step else ""
                lines.append(
                    f"| {result.leg.name.lower()} | {result.joint} | "
                    f"{result.previous:+d} | {result.offset:+d} | "
                    f"{result.drift:+d}{flag} |"
                )
        lines += ["", "## Reference used", ""]
        for joint, text in ZERO_REFERENCE.items():
            lines.append(f"- **{joint}**: " + " ".join(text))
        if self.aborted:
            lines += ["", f"**Run did not complete:** {self.aborted}"]
        lines += [
            "",
            "## What this does not say",
            "",
            "- The offsets assume the firmware's stored `ServoMiddlePWM[]` is still",
            "  at its default; `sset` is never sent by this codebase, but another",
            "  tool may have written it and nothing can read it back (ASSUMPTIONS D8).",
            "- Nothing here was measured by the robot. The operator's eye is the",
            "  only sensor on this path (ASSUMPTIONS A3/D3).",
            "",
        ]
        return "\n".join(lines)


def _ask_nudge(ask: Asker, say: Printer, step: int, offset: int, channel: int) -> str | int:
    """One answer from the nudge loop: a count to move, or a word to act on."""
    while True:
        answer = (
            ask(
                f"    [{offset:+d} counts / {counts_to_degrees(offset, channel):+.2f} deg] "
                f"+{step} / -{step} / +n / -n / ok / skip / stop: "
            )
            .strip()
            .lower()
        )
        if answer in ("", "+"):
            return step
        if answer == "-":
            return -step
        if answer in ("ok", "o", "done", "y"):
            return "ok"
        if answer in ("skip", "s"):
            return "skip"
        if answer in ("stop", "q", "quit", "abort"):
            return "stop"
        try:
            return int(answer)
        except ValueError:
            say("    Answer with a signed number of counts, or ok / skip / stop.")


def _measure_zero(
    client: RobotClient,
    leg: LegId,
    joint: str,
    channel: int,
    *,
    step: int,
    ask: Asker,
    say: Printer,
    previous: int | None = None,
) -> tuple[ZeroResult, bool]:
    """Nudge one servo until the operator says it sits at its zero.

    Returns the result and whether the run as a whole should carry on.
    """
    result = ZeroResult(leg=leg, joint=joint, channel=channel, previous=previous)
    say("")
    say(f"  {leg.name.lower()} / {joint} (channel {channel})")
    if previous is not None:
        say(
            f"    Stored: {previous:+d} counts. Judge it fresh -- this run is the"
            " repeatability check."
        )
    for line in ZERO_REFERENCE[joint]:
        say(f"    {line}")
    say("    Nudge until that is true, then answer ok. 'skip' leaves this joint")
    say("    unmeasured -- which is not the same as measuring zero.")

    while True:
        answer = _ask_nudge(ask, say, step, result.offset, channel)
        if answer == "ok":
            if result.steps == 0:
                # Answering ok straight away is a *claim*: that this joint is
                # already exactly at its reference. It writes a measured 0 into
                # the table, which later reads as "calibrated". An operator who
                # simply did not know what was being asked must not be able to
                # produce that by pressing enter three times.
                say("    Nothing was nudged, so this would record a measured zero --")
                say("    a claim that this joint is already exactly at its reference.")
                confirmed = _ask_yes_no(
                    ask, say, "    Is it? y (record 0) / n (keep nudging) / s (stop): "
                )
                if confirmed is None:
                    result.skipped = True
                    result.note = "run stopped here"
                    return result, False
                if not confirmed:
                    continue
                result.note = "already at the reference, no nudge needed"
            return result, True
        if answer == "skip":
            result.skipped = True
            result.note = "operator skipped"
            return result, True
        if answer == "stop":
            result.skipped = True
            result.note = "run stopped here"
            return result, False
        assert isinstance(answer, int)
        if abs(result.offset + answer) > MAX_OFFSET:
            say(
                f"    Refused: that would put the zero {result.offset + answer:+d} counts "
                f"from the middle, past the +/-{MAX_OFFSET} plausibility bound."
            )
            say("    Check that you are looking at the right joint before continuing.")
            continue
        try:
            _send(client, say, partial(client.trim_servo, channel, answer))
        except RobodogError as exc:
            say(f"    Command refused: {exc}")
            result.note = str(exc)
            continue
        result.offset += answer
        result.steps += 1
        if abs(result.offset) > SUSPICIOUS_OFFSET:
            say(
                f"    NOTE: {result.offset:+d} counts is a lot for a zero "
                f"({counts_to_degrees(result.offset, channel):+.1f} deg) -- around one"
            )
            say("    horn tooth (32-40 counts). Check this is the joint you think it is.")


def run_servo_calibration(
    host: str,
    *,
    ask: Asker,
    say: Printer,
    legs: tuple[LegId, ...] = (LegId.FRONT_LEFT,),
    joints: tuple[str, ...] = JOINTS,
    step: int = DEFAULT_ZERO_STEP,
    known: ServoCalibration = EMPTY,
    rehearsal: bool = False,
    client_factory: Callable[[str], RobotClient] | None = None,
) -> tuple[ZeroReport, ServoCalibration]:
    """Measure where each servo's zero really is, one joint at a time."""
    report = ZeroReport(host=host, step=step, rehearsal=rehearsal)
    calibration = known

    say("")
    say("=" * 72)
    say(" WAVEGO servo zero calibration (M3 offset table)")
    say("=" * 72)
    say("")
    say(" For each joint: the robot goes to its middle position, you nudge the")
    say(" servo until the joint matches the printed reference, and the counts it")
    say(" took ARE the calibration. Nothing is written to the robot -- the number")
    say(" lands in a file here (`sset` is never sent).")
    say("")
    say(" NOT the same run as `robodog calibrate-roll`. That one drives a servo")
    say(" towards its mechanical END STOPS to learn how far a leg can travel,")
    say(" and its numbers become workspace limits. This one finds each joint's")
    say(" ZERO -- where angle 0 actually sits in PWM counts -- and its numbers")
    say(" become the servo mapping's offsets. You need both; they measure")
    say(" different things and land in different files.")
    say("")
    say(" SAFETY, read before continuing:")
    say("  * Robot ON A STAND, legs hanging free.")
    say("  * The run starts by straightening every leg (funcMode 9): legs nearly")
    say("    vertical, body higher than when walking. Expected (ASSUMPTIONS C12).")
    say("  * Each leg is re-baselined before it is measured, so a leg aligned")
    say("    earlier snaps back to the middle. That is fine -- what was measured")
    say("    is the number, not the pose.")
    say("  * The firmware reports nothing back; your eye is the only sensor.")
    say("")
    if ask("Type 'yes' when the robot is secured and you are ready: ").strip().lower() not in _YES:
        report.aborted = "operator did not confirm the safety prompt"
        say("Aborted; nothing was sent.")
        return report, calibration

    factory = client_factory if client_factory is not None else _default_client_factory
    client = factory(host)
    try:
        client.connect()
    except RobodogError as exc:
        report.aborted = f"cannot reach the robot: {exc}"
        say(f"\nCannot reach the robot: {exc}")
        say("Check that you are on the robot's access point and that the host is right.")
        return report, calibration

    if Capability.SERVO_TRIM not in client.capabilities:
        report.aborted = f"backend {client.backend_name} has no servo trim"
        say(f"\nBackend {client.backend_name} cannot trim servos; nothing to do.")
        client.disconnect()
        return report, calibration

    client.arm()
    try:
        for leg in legs:
            say("")
            say("-" * 72)
            say(f" {leg.name.lower()}")
            say("-" * 72)
            _baseline(client, say)
            carry_on = True
            for joint in joints:
                channel = channel_of(leg, joint)
                _leave_middle_pos_loop(client, channel, say)
                result, carry_on = _measure_zero(
                    client,
                    leg,
                    joint,
                    channel,
                    step=step,
                    ask=ask,
                    say=say,
                    previous=known.offsets.get(channel),
                )
                if result.drift is not None and abs(result.drift) > step:
                    say(
                        f"    DISAGREES with the stored {result.previous:+d} by "
                        f"{result.drift:+d} counts -- more than the {step}-count step"
                    )
                    say("    this procedure claims to resolve. The number is soft.")
                report.results.append(result)
                if not result.skipped:
                    calibration = calibration.with_offset(channel, result.offset)
                if not carry_on:
                    report.aborted = "operator stopped the run"
                    break
            if not carry_on:
                break
    except RobodogError as exc:
        report.aborted = f"aborted: {exc}"
        say(f"\nAborted: {exc}")
    finally:
        _restore(client, say)
        client.disconnect()

    return report, calibration


def write_servo_report(report: ZeroReport, directory: Path, today: date) -> Path:
    """Write the report next to the bring-up ones, never overwriting a run."""
    directory.mkdir(parents=True, exist_ok=True)
    stem = f"servo-zero-{today.isoformat()}"
    path = directory / f"{stem}.md"
    run = 2
    while path.exists():
        path = directory / f"{stem}-run{run}.md"
        run += 1
    path.write_text(report.to_markdown(today), encoding="utf-8")
    return path


def parse_joints_arg(value: str) -> tuple[str, ...]:
    """`all`, or a comma-separated list of joint names."""
    if value.strip().lower() == "all":
        return JOINTS
    joints: list[str] = []
    for name in value.split(","):
        key = name.strip().lower()
        if key not in JOINTS:
            raise ValueError(f"unknown joint {name.strip()!r} (valid: {', '.join(JOINTS)}, all)")
        if key not in joints:
            joints.append(key)
    if not joints:
        raise ValueError("no joint given")
    return tuple(joints)
