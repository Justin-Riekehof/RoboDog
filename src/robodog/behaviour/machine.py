"""The behaviour state machine: detections and a clock in, drive intents out.

Pure, in the same sense the kinematics are pure. No camera, no model, no
backend, no sleeping -- ``update()`` is a function of (what is visible, what
time it is) and the state carried since the last call. Its tests script a
detector and step a fake clock, exactly as the watchdog tests do.

**The model does not drive.** A language model maps the operator's words to a
behaviour call *once*, before anything moves (:mod:`robodog.ai`); from there on
the loop below is deterministic and every intent it produces still goes through
``SafetySupervisor`` on its way to a backend.

Two invariants are worth stating outright, because they are what makes this
safe to run at a person with no depth sensor, no bumper and no servo feedback:

1. **The robot never walks forward without a fresh detection.** Losing the
   target stops it, immediately -- during the grace period it stands still
   rather than carrying on blind, and after it the robot turns in place. There
   is no state in which it moves towards something it cannot currently see.
2. **Every run is bounded.** A hard timeout ends the behaviour wherever it has
   got to, and the search gives up on its own. Nothing here can walk forever.

Distance is a *guess*: the only cue is how much of the frame's height the box
fills (ASSUMPTIONS G2). So the stop criterion is that fraction itself, not a
converted distance -- the safety path does not wait for a calibration session.
The behaviour owns a hard ceiling (:data:`STOP_HEIGHT_MAX`) that no operator
and no model can raise, and the value inside it is settable per run.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Final

from robodog.api.types import Drive
from robodog.vision import Detection

# --- how a box height becomes a distance, and why it is only ever a guess ----

# Assumed vertical field of view of the OV2640 with the lens this unit ships
# with. Unverified: the vendor states no angle, and the number below is the
# usual one for a 2.1 mm ESP32-CAM module (about 65 deg horizontal on a 4:3
# sensor). Measuring it takes one photograph of a metre rule at a known
# distance -- see ASSUMPTIONS G2.
CAMERA_VFOV_DEG: Final = 50.0
# How high the camera sits above the floor with the robot standing, and which
# way it looks. Both unmeasured (ASSUMPTIONS G2): the twin puts the trunk at
# 100 mm and is 40 mm deep, and the camera board sits on the front above that.
# The height matters more than it looks -- see the clipped branch below, where
# the estimated distance is directly proportional to it.
CAMERA_HEIGHT_MM: Final = 140.0
# What "a person" is assumed to be, in millimetres, when a distance is asked
# for. An adult, standing.
SUBJECT_HEIGHT_MM: Final = 1750.0

# The closest the robot may ever be told to come, as a fraction of frame
# height. Nothing raises this -- an operator or a model asking to stop nearer
# is clamped to it. About 0.5 m under the geometry below.
#
# Raised from 0.70 (0.75 m) on 2026-08-23, on the operator's instruction, and
# the honest statement of what it now means is: **the robot may touch you.**
#
# The risk is NOT that the threshold is imprecise. Close in it is the opposite
# of imprecise: 0.01 of frame height is 16 mm at this ceiling, against 500 mm
# out at 3 m, because distance goes as 1/(f - 0.5) and the resolution improves
# as the target fills more of the frame. The risk is that the whole scale rests
# on two unmeasured constants (ASSUMPTIONS G2): distance is *directly*
# proportional to the assumed camera height, so a camera at 100 mm rather than
# 140 puts this ceiling at 0.36 m instead of 0.50 m, and the field of view
# moves it again. And a stop always acts on a picture tens of milliseconds old.
#
# What makes that acceptable rather than reckless is the machine, not the
# number: 465 g, walking at about 9 cm/s, no sharp edges, stopping on any loss
# of sight, bounded by a run timeout.
STOP_HEIGHT_MAX: Final = 0.80
# And the farthest a request may push it: below this the robot would stop
# before it had meaningfully approached anything.
STOP_HEIGHT_MIN: Final = 0.10
# What a run stops at when nobody says: about 0.95 m, near enough to read as
# "it came to me" and still short of contact.
STOP_HEIGHT_DEFAULT: Final = 0.66


def _half_frame(vfov_deg: float) -> float:
    """Half the image plane, in the tangent units a pinhole camera measures in."""
    return math.tan(math.radians(vfov_deg) / 2.0)


def height_fraction_for_distance(
    distance_mm: float,
    *,
    subject_height_mm: float = SUBJECT_HEIGHT_MM,
    camera_height_mm: float = CAMERA_HEIGHT_MM,
    vfov_deg: float = CAMERA_VFOV_DEG,
) -> float:
    """How much of the frame's height a standing subject fills at that distance.

    Not the textbook ``size / distance``, and the difference is the whole point.
    That formula assumes the subject fits in the picture; this camera sits about
    a hand's width off the floor and looks straight ahead, so **a person is
    clipped by the top of the frame from roughly 3.4 m inward** and the box
    stops growing the way the textbook says. Using the simple formula would have
    the robot expect a person to fill the frame at 1.9 m -- they never do, at
    any distance, and a stop threshold picked from it would never be reached.

    So: project the subject's feet and head onto the image plane (a pinhole maps
    the *tangent* of the angle, not the angle), clip both to the frame, and take
    what is left. Nothing here is calibrated -- field of view, camera height and
    pitch are all assumptions (G2) -- but the *shape* of the curve is right, and
    that is what decides whether a threshold is reachable at all.
    """
    if distance_mm <= 0:
        raise ValueError(f"distance must be positive, got {distance_mm}")
    half = _half_frame(vfov_deg)
    head = min((subject_height_mm - camera_height_mm) / distance_mm, half)
    feet = max(-camera_height_mm / distance_mm, -half)
    return max(head - feet, 0.0) / (2.0 * half)


def distance_mm_for_height_fraction(
    height_fraction: float,
    *,
    subject_height_mm: float = SUBJECT_HEIGHT_MM,
    camera_height_mm: float = CAMERA_HEIGHT_MM,
    vfov_deg: float = CAMERA_VFOV_DEG,
) -> float | None:
    """The inverse: a distance estimate to show the operator. None if unusable.

    Two branches, because the curve has two regimes (see above). Far away the
    whole subject is visible and the fraction falls off as 1/d; nearer, the head
    is out of frame and only the feet still move, which is why the estimate
    becomes *proportional to the assumed camera height* and therefore much
    shakier. The page says "about" for exactly this reason, and the state
    machine never reads it.
    """
    if height_fraction <= 0:
        return None
    half = _half_frame(vfov_deg)
    # Where the two regimes meet: the distance at which the head sits exactly on
    # the top edge of the frame.
    clip_distance = (subject_height_mm - camera_height_mm) / half
    clip_fraction = subject_height_mm / (2.0 * half * clip_distance)
    if height_fraction <= clip_fraction:
        return subject_height_mm / (2.0 * half * height_fraction)
    # Clipped: only the feet carry information, and they carry little.
    denominator = half * (2.0 * height_fraction - 1.0)
    if denominator <= 0:
        return None  # the box is taller than the geometry allows -- say nothing
    return camera_height_mm / denominator


# Above this the box's top edge counts as clipped by the frame -- which is the
# regime the whole near-field distance cue lives in.
_CLIPPED_TOP: Final = 0.002


def level_height_fraction(
    detection: Detection, pitch_deg: float, *, vfov_deg: float = CAMERA_VFOV_DEG
) -> float:
    """The box height the camera would have measured looking horizontally.

    All of the geometry above assumes a level camera, and a walking quadruped's
    body is not level: two degrees of pitch turn an estimated 0.94 m into
    0.76-1.22 m, which is the single largest error in the distance estimate
    (ASSUMPTIONS G2). With an attitude per frame (`robodog.localization`) it
    comes out.

    The correction is **not** simply "subtract the shift from the height", and
    getting that wrong would make things worse rather than better. Pitch moves
    content up and down in the image; it does not change how big a thing is. A
    fully visible person's box therefore keeps its height exactly, and only the
    *clipped* case -- where the top edge is the frame rather than the head --
    grows and shrinks with pitch, because there the height is measuring where
    the feet are and nothing else. So: correct the bottom edge, and only when
    the top is clipped.

    ``pitch_deg`` is positive nose-up, matching ``BodyPose.pitch`` and
    ``Attitude.pitch``. Nose-up moves the world down the frame, so the feet sit
    lower and the box reads nearer than it is.
    """
    box = detection.box
    if box.top > _CLIPPED_TOP or pitch_deg == 0.0:
        return detection.height_fraction
    shift = math.tan(math.radians(pitch_deg)) / (2.0 * _half_frame(vfov_deg))
    return min(max(box.bottom - shift, 0.0), 1.0)


class BehaviourState(Enum):
    """Where a run has got to. ARRIVED and LOST are terminal.

    The states are the phases of an approach that never moves two ways at
    once: LOOKING stands and watches, ALIGNING turns on the spot towards a
    bearing the last look produced, ADVANCING walks dead straight, SEARCHING
    turn-pulses for a target it has not got. There is deliberately no state
    that walks and turns together -- that combination is what produced the
    overshoot this design replaced (see ComeToMe).
    """

    SEARCHING = auto()  # no target: turn in pulses, look between them
    LOOKING = auto()  # standing still, waiting for the camera to speak
    ALIGNING = auto()  # turning on the spot towards the last seen bearing
    ADVANCING = auto()  # walking straight ahead, a bounded burst
    PEEKING = auto()  # kneeling hind legs, camera pitched up: is someone there?
    ARRIVED = auto()  # near enough, stopped -- the successful end
    LOST = auto()  # gave up: the search found nothing, or the run timed out


_TERMINAL: Final = frozenset({BehaviourState.ARRIVED, BehaviourState.LOST})
# The states in which the machine wants a fresh gyro reading each tick: blind
# rotation is closed-loop on `turned`, and blind advance watches it for drift.
ATTITUDE_HUNGRY: Final = frozenset(
    {BehaviourState.ALIGNING, BehaviourState.ADVANCING, BehaviourState.PEEKING}
)


@dataclass(frozen=True, slots=True)
class Intent:
    """One tick's answer: what to drive, where the run stands, and why."""

    drive: Drive
    state: BehaviourState
    reason: str
    target: Detection | None = None
    stance: str = "stand"
    """The whole-body pose this tick wants: "stand", or "peek" -- hind legs
    kneeling, camera pitched up. Declarative on purpose: the machine stays
    pure, and the runner maps the word onto leg targets (or ignores it on a
    backend without LEG_TARGET)."""

    @property
    def finished(self) -> bool:
        return self.state in _TERMINAL


@dataclass(frozen=True, slots=True)
class ApproachConfig:
    """Everything the approach is allowed to argue about, in one place.

    The shape of these numbers changed on 2026-08-25, after the first real
    approach on the robot: the old steering regime (walk and turn at once,
    hysteresis bands on the bearing) overshot so badly under stop-and-look
    that the person left the field of view in a single blind burst -- the
    robot turns at up to 42.7 deg/s (G4) and half the FOV is ~32 deg. The
    operator's redesign: turn ONLY on the spot, closed-loop on the gyro;
    advance ONLY dead straight, in bounded bursts; look between the two.
    """

    target: str = "person"
    stop_height_fraction: float = STOP_HEIGHT_DEFAULT
    # Two confidences, not one: a target already being approached is kept on
    # weaker evidence than an unknown one is acquired on. Walking towards a
    # person fills the frame with a fraction of them, and a fraction scores
    # far below a whole one (G6).
    acquire_confidence: float = 0.40
    keep_confidence: float = 0.25
    # Smoothing time constant for the bearing while LOOKING: the detector
    # answers several times during one look, and the box centre jitters (G7).
    # Only ever applied while standing -- there is no steering to smooth any
    # more, this steadies the number one alignment is based on.
    bearing_tau: float = 0.35
    # --- geometry ---------------------------------------------------------
    # Assumed horizontal field of view; turns a normalised bearing into the
    # degrees the gyro must see. Same assumption family as G2's vertical FOV.
    hfov_deg: float = 65.0
    # --- aligning ---------------------------------------------------------
    # Close enough: within this of the target heading the turn ends. Half a
    # metre out, 7 deg of bearing error is ~6 cm of lateral miss -- the next
    # look absorbs it.
    align_tolerance_deg: float = 7.0
    # The whole alignment, gyro-guided or not, may take at most this long.
    align_timeout: float = 5.0
    # Without a gyro (mock/sim, or firmware without the IMU) alignment falls
    # back to timed pulses: turn for |bearing| / rate, capped here, then look
    # again. Iterative and slow, and honest about it.
    align_max_pulse: float = 0.4
    # The measured turn rates (G4, on the stand -- revise after a floor
    # measurement). Asymmetric because the robot is (F1): left is 2.4x right.
    turn_rate_left_dps: float = 42.7
    turn_rate_right_dps: float = 17.9
    # --- advancing --------------------------------------------------------
    # How long one straight blind burst may last. At ~9 cm/s this is ~11 cm
    # per burst -- the "regularly check" half of the operator's design.
    walk_burst_seconds: float = 1.2
    # Abort the burst early when the gyro says the heading has drifted this
    # far: the robot veers when walking (F1), and a veer the next look would
    # have to hunt for is better cut short.
    drift_abort_deg: float = 15.0
    # A fresh detection mid-burst (continuous vision: sim, or streamgate=0)
    # this far off centre ends the burst for a re-align rather than steering.
    realign_bearing_deg: float = 14.0
    # --- looking ----------------------------------------------------------
    # How long a look dwells on a target before committing to a turn or a
    # burst. The detector answers several times a second while the robot
    # stands; deciding on the very first frame would make the smoothing above
    # decorative -- one jittery box centre would steer the whole alignment
    # (G7). Zero decides immediately (tests use that).
    look_settle_seconds: float = 0.35
    # How long a look waits for the camera before concluding the target is
    # gone. Covers the stream restarting after a stop (~1 s) plus a detector
    # pass; the blind-walk bound is walk_burst_seconds, not this.
    look_patience: float = 2.5
    # --- searching (unchanged: the one state that must rotate) ------------
    search_turn_seconds: float = 0.6
    search_look_seconds: float = 0.5
    search_seconds: float = 20.0
    # --- arrival ----------------------------------------------------------
    # Losing the target while it filled at least stop - margin of the frame is
    # arrival, not loss (G6).
    lost_close_margin: float = 0.10
    # The stop size must hold this long; one frame is not evidence (G2/G10).
    stop_confirm_seconds: float = 0.3
    # --- peeking ----------------------------------------------------------
    # When the target is lost close in, kneel the hind legs and pitch the
    # camera up before declaring arrival: at half a metre a standing person's
    # torso is far above a level lens, and the operator's observation is that
    # the robot could simply look up (2026-08-25). The IMU measures the
    # commanded pitch and the size correction absorbs it, so the reading
    # stays honest while tilted. False disables (backends without
    # LEG_TARGET cannot change stance).
    peek: bool = True
    # How long the upward look waits for the person to reappear before
    # falling back to arrival-by-loss (G6), which was previously immediate.
    peek_seconds: float = 2.5
    # The whole run, however it is going.
    timeout: float = 60.0
    default_search_turn: int = 1  # +1 right, -1 left

    def __post_init__(self) -> None:
        if not STOP_HEIGHT_MIN <= self.stop_height_fraction <= STOP_HEIGHT_MAX:
            raise ValueError(
                f"stop_height_fraction {self.stop_height_fraction} outside "
                f"{STOP_HEIGHT_MIN}..{STOP_HEIGHT_MAX}"
            )
        if self.default_search_turn not in (-1, 1):
            raise ValueError("default_search_turn must be -1 (left) or +1 (right)")
        if not 0.0 < self.keep_confidence <= self.acquire_confidence <= 1.0:
            raise ValueError("confidences must satisfy 0 < keep <= acquire <= 1")
        if not 20.0 <= self.hfov_deg <= 120.0:
            raise ValueError(f"hfov_deg {self.hfov_deg} is not a plausible lens")
        if self.align_tolerance_deg <= 0 or self.align_timeout <= 0:
            raise ValueError("alignment needs positive tolerance and timeout")
        if self.turn_rate_left_dps <= 0 or self.turn_rate_right_dps <= 0:
            raise ValueError("turn rates must be positive")
        if not 0.0 <= self.look_settle_seconds <= 2.0:
            raise ValueError("look_settle_seconds must be within 0..2")
        if not 0.2 <= self.walk_burst_seconds <= 5.0:
            raise ValueError("walk_burst_seconds must be within 0.2..5")
        if self.stop_confirm_seconds < 0.0:
            raise ValueError("stop_confirm_seconds must be >= 0")
        if not 0.0 <= self.lost_close_margin < self.stop_height_fraction:
            raise ValueError(
                f"lost_close_margin {self.lost_close_margin} must be >= 0 and smaller "
                f"than stop_height_fraction {self.stop_height_fraction}"
            )


def pick_target(
    detections: Sequence[Detection], *, label: str, min_confidence: float
) -> Detection | None:
    """The one to walk at: the nearest confident match.

    Nearest, not most central, and "nearest" means the tallest box -- with
    several people in frame, "come to me" means the one in front, and choosing
    by bearing instead would make the robot swap targets every time it turned.
    """
    matches = [d for d in detections if d.label == label and d.confidence >= min_confidence]
    if not matches:
        return None
    return max(matches, key=lambda d: (d.height_fraction, d.confidence))


@dataclass(slots=True)
class ComeToMe:
    """Look, align on the spot, advance dead straight, look again.

    The regime the operator designed after the first stop-and-look drive
    (2026-08-25): the robot had been allowed to walk and turn at once, and a
    blind burst of that at 42.7 deg/s swung the person out of the field of
    view faster than any look could recover. Now no intent ever combines
    forward with turn:

    * **LOOKING** stands still -- which is also what lets the gated camera
      stream flow -- and smooths the bearing the detector reports.
    * **ALIGNING** turns on the spot towards that bearing, closed-loop on the
      gyro when one is reporting (`turned_deg`), in short timed pulses when
      not. Either way the rotation is bounded and verified, never dead
      reckoned on time alone at an unmeasured rate.
    * **ADVANCING** walks straight ahead for a bounded burst, watching the
      gyro only for drift (the robot veers, F1); any needed correction is a
      stop and a fresh look, never a turn on the move.
    * **SEARCHING** is unchanged: it is the one state whose job is rotation,
      and it always pulsed with look-pauses built in.

    ``update`` is the whole behaviour: detections, the clock, and optionally
    the body's pitch and accumulated yaw in; one drive intent out. Terminal
    states latch and keep commanding a stop.
    """

    config: ApproachConfig = field(default_factory=ApproachConfig)
    state: BehaviourState = BehaviourState.LOOKING
    reason: str = ""
    started_at: float | None = None
    _last_seen: float | None = None
    _last_bearing_deg: float = 0.0
    _last_height: float = 0.0
    _steered_bearing: float | None = None
    _holding: bool = False
    _look_since: float | None = None
    _look_target_since: float | None = None
    # Whether THIS tick brought a gyro reading. `_last_turned` remembers the
    # newest value ever seen, which is exactly wrong for closed-loop turning:
    # steering by a stale angle is dead reckoning wearing a sensor's badge.
    _turned_fresh: bool = False
    _align_since: float | None = None
    _align_dir: int = 0
    _align_target_deg: float | None = None
    _align_pulse_until: float | None = None
    _last_turned: float | None = None
    _walk_since: float | None = None
    _walk_heading_ref: float | None = None
    _peek_since: float | None = None
    _peeked: bool = False
    _search_since: float | None = None
    _search_phase_since: float | None = None
    _turning: bool = True
    _big_since: float | None = None

    # --- the loop ---------------------------------------------------------

    def update(
        self,
        detections: Sequence[Detection],
        now: float,
        pitch_deg: float = 0.0,
        turned_deg: float | None = None,
    ) -> Intent:
        """One tick. ``turned_deg`` is the body's accumulated yaw, if known.

        Without it (mock, sim, firmware without the IMU) alignment degrades to
        short timed pulses and drift goes unwatched; everything else is
        unchanged. ``pitch_deg`` corrects the distance cue exactly as before.
        """
        if self.started_at is None:
            self.started_at = now
            self._look_since = now
        self._turned_fresh = turned_deg is not None
        if turned_deg is not None:
            self._last_turned = turned_deg
        if self.state in _TERMINAL:
            return Intent(Drive(0, 0), self.state, self.reason)
        if now - self.started_at >= self.config.timeout:
            return self._give_up(f"timed out after {self.config.timeout:.0f}s")

        threshold = self.config.keep_confidence if self._holding else self.config.acquire_confidence
        target = pick_target(detections, label=self.config.target, min_confidence=threshold)

        if self.state is BehaviourState.PEEKING:
            return self._peek_tick(target, now, pitch_deg)
        if self.state is BehaviourState.ALIGNING:
            return self._align_tick(target, now, pitch_deg)
        if self.state is BehaviourState.ADVANCING:
            return self._advance_tick(target, now, pitch_deg)
        if self.state is BehaviourState.SEARCHING and target is None:
            return self._search_tick(now)
        # LOOKING -- or SEARCHING that just found something.
        return self._look_tick(target, now, pitch_deg)

    # --- looking ----------------------------------------------------------

    def _look_tick(self, target: Detection | None, now: float, pitch_deg: float) -> Intent:
        config = self.config
        if self._look_since is None:
            self._look_since = now
        if target is None:
            waited = now - self._look_since
            if waited < config.look_patience:
                self.state = BehaviourState.LOOKING
                self.reason = f"looking ({waited:.1f}s)"
                return Intent(Drive(0, 0), self.state, self.reason)
            # The camera has had its chance. Near-loss means the person is
            # probably towering over a level lens -- so look up and CHECK,
            # once, before calling it arrival on a heuristic (G6).
            if self._holding and self._last_height >= self.lost_close_height:
                if config.peek and not self._peeked:
                    return self._enter_peek(now)
                self.state = BehaviourState.ARRIVED
                self.reason = (
                    f"arrived: lost sight of the {config.target} at "
                    f"{self._last_height * 100:.0f}% of the frame -- too close to see it whole"
                )
                return Intent(Drive(0, 0), self.state, self.reason)
            # ... and far-loss is a search.
            return self._enter_search(now)

        size = level_height_fraction(target, pitch_deg)
        smoothed = self._smooth(target.bearing, now)
        self._register_sighting(target, size, now)
        if size >= config.stop_height_fraction:
            return self._confirm_arrival(size, now, target)
        self._big_since = None

        if size < self.lost_close_height:
            # Back at ordinary range: a later close approach earns a fresh peek.
            self._peeked = False
        # Dwell before committing: the whole point of smoothing the bearing is
        # that more than one frame contributes to it, and a decision on the
        # first frame would hand one jittery box centre the entire alignment.
        if self._look_target_since is None:
            self._look_target_since = now
        if now - self._look_target_since < config.look_settle_seconds:
            self.state = BehaviourState.LOOKING
            self.reason = f"watching (bearing {smoothed:+.2f})"
            return Intent(Drive(0, 0), self.state, self.reason, target)

        bearing_deg = smoothed * (config.hfov_deg / 2.0)
        if abs(bearing_deg) <= config.align_tolerance_deg:
            return self._enter_advance(now, target)
        return self._enter_align(bearing_deg, now, target)

    def _register_sighting(self, target: Detection, size: float, now: float) -> None:
        self._last_seen = now
        self._last_height = size
        self._last_bearing_deg = target.bearing * (self.config.hfov_deg / 2.0)
        self._holding = True
        self._search_since = None

    def _confirm_arrival(self, size: float, now: float, target: Detection) -> Intent:
        if self._big_since is None:
            self._big_since = now
        held = now - self._big_since
        if held >= self.config.stop_confirm_seconds:
            self.state = BehaviourState.ARRIVED
            self.reason = f"{self.config.target} fills {size * 100:.0f}% of the frame"
            return Intent(Drive(0, 0), self.state, self.reason, target)
        self.state = BehaviourState.LOOKING
        self.reason = f"close enough -- confirming ({size:.2f}, {held:.1f}s)"
        return Intent(Drive(0, 0), self.state, self.reason, target)

    # --- aligning ---------------------------------------------------------

    def _enter_align(self, bearing_deg: float, now: float, target: Detection) -> Intent:
        config = self.config
        self.state = BehaviourState.ALIGNING
        self._align_since = now
        self._align_dir = 1 if bearing_deg > 0 else -1
        if self._last_turned is not None:
            self._align_target_deg = self._last_turned + bearing_deg
            self._align_pulse_until = None
            how = "gyro"
        else:
            rate = config.turn_rate_right_dps if bearing_deg > 0 else config.turn_rate_left_dps
            self._align_target_deg = None
            self._align_pulse_until = now + min(abs(bearing_deg) / rate, config.align_max_pulse)
            how = "timed pulse"
        self.reason = f"aligning {bearing_deg:+.0f} deg ({how})"
        return Intent(Drive(0, self._align_dir), self.state, self.reason, target)

    def _align_tick(self, target: Detection | None, now: float, pitch_deg: float) -> Intent:
        config = self.config
        assert self._align_since is not None
        if target is not None:
            # Continuous vision only (a gated stream is dark while turning):
            # someone who walked up to the robot mid-align is an arrival, not
            # an alignment problem.
            size = level_height_fraction(target, pitch_deg)
            self._register_sighting(target, size, now)
            if size >= config.stop_height_fraction:
                return self._confirm_arrival(size, now, target)
        if now - self._align_since >= config.align_timeout:
            return self._enter_look(now, "alignment timed out -- looking")
        if self._align_target_deg is not None:
            if not self._turned_fresh:
                # The gyro fell silent mid-turn; steering on by the remembered
                # angle would be dead reckoning wearing a sensor's badge, and
                # the overshoot this design exists to end.
                return self._enter_look(now, "gyro went quiet -- looking")
            assert self._last_turned is not None
            remaining = self._align_target_deg - self._last_turned
            if self._align_dir * remaining <= config.align_tolerance_deg:
                return self._enter_look(now, "aligned -- looking")
            self.reason = f"aligning, {abs(remaining):.0f} deg to go"
            return Intent(Drive(0, self._align_dir), self.state, self.reason)
        assert self._align_pulse_until is not None
        if now >= self._align_pulse_until:
            return self._enter_look(now, "align pulse done -- looking")
        self.reason = "aligning (timed pulse)"
        return Intent(Drive(0, self._align_dir), self.state, self.reason)

    def _enter_look(self, now: float, reason: str) -> Intent:
        self.state = BehaviourState.LOOKING
        self._look_since = now
        self._look_target_since = None
        self._steered_bearing = None
        self._align_since = None
        self._align_target_deg = None
        self._align_pulse_until = None
        self._walk_since = None
        self._walk_heading_ref = None
        self.reason = reason
        return Intent(Drive(0, 0), self.state, self.reason)

    # --- advancing --------------------------------------------------------

    def _enter_advance(self, now: float, target: Detection | None) -> Intent:
        self.state = BehaviourState.ADVANCING
        self._walk_since = now
        self._walk_heading_ref = self._last_turned
        self.reason = "advancing straight"
        return Intent(Drive(1, 0), self.state, self.reason, target)

    def _advance_tick(self, target: Detection | None, now: float, pitch_deg: float) -> Intent:
        config = self.config
        assert self._walk_since is not None
        if (
            self._walk_heading_ref is not None
            and self._last_turned is not None
            and abs(self._last_turned - self._walk_heading_ref) >= config.drift_abort_deg
        ):
            return self._enter_look(now, "heading drifted -- stopping to look")
        if now - self._walk_since >= config.walk_burst_seconds:
            return self._enter_look(now, "burst done -- looking")
        if target is not None:
            # Continuous vision (sim, or streamgate=0): the burst may end
            # early on what the camera says, but it never steers.
            size = level_height_fraction(target, pitch_deg)
            self._register_sighting(target, size, now)
            if size >= config.stop_height_fraction:
                return self._confirm_arrival(size, now, target)
            if abs(self._last_bearing_deg) >= config.realign_bearing_deg:
                return self._enter_look(now, "target off centre -- stopping to look")
        self.reason = "advancing straight"
        return Intent(Drive(1, 0), self.state, self.reason, target)

    # --- peeking: kneel, look up, make sure -------------------------------

    def _enter_peek(self, now: float) -> Intent:
        self.state = BehaviourState.PEEKING
        self._peek_since = now
        self._peeked = True
        self._steered_bearing = None
        self.reason = "lost it close in -- kneeling to look up"
        return Intent(Drive(0, 0), self.state, self.reason, stance="peek")

    def _peek_tick(self, target: Detection | None, now: float, pitch_deg: float) -> Intent:
        config = self.config
        assert self._peek_since is not None
        if target is not None:
            # There they are. The pitch the tilt commands is measured by the
            # IMU and fed in here, so the size is corrected for the very tilt
            # that made the sighting possible.
            size = level_height_fraction(target, pitch_deg)
            self._register_sighting(target, size, now)
            if size >= config.stop_height_fraction:
                self.state = BehaviourState.ARRIVED
                self.reason = (
                    f"arrived: looked up and found the {config.target} "
                    f"({size * 100:.0f}% of the frame)"
                )
                return Intent(Drive(0, 0), self.state, self.reason, target, stance="peek")
            # Visible but small: they stepped back. Stand up and resume.
            return self._enter_look(now, "they moved away -- standing back up")
        if now - self._peek_since >= config.peek_seconds:
            # Looked up, saw nobody. The close-loss heuristic stands, minus
            # its confidence: say what was and was not seen.
            self.state = BehaviourState.ARRIVED
            self.reason = (
                f"arrived: lost the {config.target} at "
                f"{self._last_height * 100:.0f}% of the frame; looking up found nothing"
            )
            return Intent(Drive(0, 0), self.state, self.reason)
        self.reason = "peeking up"
        return Intent(Drive(0, 0), self.state, self.reason, stance="peek")

    # --- searching (unchanged in spirit) ----------------------------------

    def _enter_search(self, now: float) -> Intent:
        self._holding = False
        self._steered_bearing = None
        self._look_target_since = None
        self._search_since = now
        self._search_phase_since = now
        self._turning = True
        return self._search_tick(now)

    def _search_tick(self, now: float) -> Intent:
        config = self.config
        if self._search_since is None:
            return self._enter_search(now)
        if now - self._search_since >= config.search_seconds:
            return self._give_up(f"no {config.target} found in {config.search_seconds:.0f}s")
        self.state = BehaviourState.SEARCHING
        turn = config.default_search_turn
        if self._last_seen is not None and self._last_bearing_deg != 0.0:
            turn = 1 if self._last_bearing_deg > 0 else -1
        assert self._search_phase_since is not None
        span = config.search_turn_seconds if self._turning else config.search_look_seconds
        if now - self._search_phase_since >= span:
            self._turning = not self._turning
            self._search_phase_since = now
        left = config.search_seconds - (now - self._search_since)
        if self._turning:
            self.reason = f"searching: turning {'right' if turn > 0 else 'left'} ({left:.0f}s left)"
            return Intent(Drive(0, turn), self.state, self.reason)
        self.reason = f"searching: looking ({left:.0f}s left)"
        return Intent(Drive(0, 0), self.state, self.reason)

    # --- shared -----------------------------------------------------------

    @property
    def lost_close_height(self) -> float:
        """The size at or above which losing the target counts as arrival."""
        return self.config.stop_height_fraction - self.config.lost_close_margin

    def _smooth(self, bearing: float, now: float) -> float:
        """The measured bearing, low-passed while LOOKING (G7)."""
        tau = self.config.bearing_tau
        if tau <= 0 or self._steered_bearing is None or self._last_seen is None:
            self._steered_bearing = bearing
            return bearing
        elapsed = max(now - self._last_seen, 0.0)
        weight = 1.0 - math.exp(-elapsed / tau)
        self._steered_bearing += weight * (bearing - self._steered_bearing)
        return self._steered_bearing

    def _give_up(self, reason: str) -> Intent:
        self.state = BehaviourState.LOST
        self.reason = reason
        return Intent(Drive(0, 0), self.state, self.reason)
