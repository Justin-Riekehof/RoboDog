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
    """Where a run has got to. ARRIVED and LOST are terminal."""

    SEARCHING = auto()  # turning in place, nothing to approach
    APPROACHING = auto()  # the target is visible and the robot is closing on it
    ARRIVED = auto()  # near enough, stopped -- the successful end
    LOST = auto()  # gave up: the search found nothing, or the run timed out


_TERMINAL: Final = frozenset({BehaviourState.ARRIVED, BehaviourState.LOST})


@dataclass(frozen=True, slots=True)
class Intent:
    """One tick's answer: what to drive, where the run stands, and why."""

    drive: Drive
    state: BehaviourState
    reason: str
    target: Detection | None = None

    @property
    def finished(self) -> bool:
        return self.state in _TERMINAL


@dataclass(frozen=True, slots=True)
class ApproachConfig:
    """Everything the approach is allowed to argue about, in one place.

    Defaults are deliberately timid. Every one of them is a guess about a robot
    nobody has run this on yet -- the turn rate in particular is unmeasured
    (ASSUMPTIONS G4), so `search_seconds` is a duration, not "one full circle".
    """

    target: str = "person"
    stop_height_fraction: float = STOP_HEIGHT_DEFAULT
    # Two confidences, not one, and the gap is deliberate: a target that is
    # ALREADY being approached is worth keeping on much weaker evidence than an
    # unknown one is worth acquiring on. Walking towards a person fills the
    # frame with a fraction of them, and a fraction of a person scores far below
    # a whole one -- so a single threshold high enough to acquire cleanly is
    # also high enough to drop the target exactly when the robot arrives.
    acquire_confidence: float = 0.40
    keep_confidence: float = 0.25
    # --- steering, with hysteresis ---------------------------------------
    #
    # Every threshold below is a *pair*, because a single one chatters. The
    # robot turns by latching a move and the picture it steers by is tens of
    # milliseconds old, so it always turns a little past centre; with one
    # threshold that overshoot re-triggers the opposite turn and the robot
    # rocks left-right without closing on anything. Observed on the robot,
    # 2026-08-23.
    #
    # Turn on the spot above this -- a target well off to the side is not in
    # front of the robot in any useful sense, and walking at it would describe
    # an arc through whatever is there.
    turn_enter_bearing: float = 0.35
    # ...and keep turning until it is this well centred. The gap is what the
    # overshoot is allowed to use up: coming to rest anywhere inside +/-0.35
    # cannot start a turn back.
    turn_exit_bearing: float = 0.15
    # While walking, start steering above this...
    correct_enter_bearing: float = 0.20
    # ...and stop steering below this.
    correct_exit_bearing: float = 0.08
    # How long the steered bearing takes to follow the measured one, in
    # seconds. The detector's box centre jitters -- limbs move, the box snaps
    # between poses, and the picture from a walking robot is blurred
    # (ASSUMPTIONS F6) -- and steering on the raw value turns that jitter
    # straight into left-right commands. Hysteresis alone does not fix this:
    # it bounds the overshoot, while this bounds the *noise*, and the rocking
    # observed on 2026-08-23 needed both. Zero disables the filter.
    bearing_tau: float = 0.35
    # How far short of the stop size a target may be lost and still count as
    # arrival rather than as loss (see ComeToMe._without_target). Relative to
    # `stop_height_fraction` rather than absolute, because the two describe the
    # same event -- being there -- and an independent number drifts away from
    # it the moment the operator asks for a different stop distance.
    #
    # Sized against where the detector plausibly gives up, not picked round.
    # With the default stop at 0.66 this covers a target lost anywhere inside
    # about 2.8 m, because the height curve is nearly flat there (0.55 at 3 m,
    # 0.59 at 1.5 m) and a tighter margin stops covering the case it exists for.
    #
    # Both ways of getting this wrong stop the robot, but they are not equally
    # good: too large and a genuine loss at middle distance is called arrival,
    # which is merely wrong; too small and the robot turns away to search for
    # someone standing right in front of it, which is the behaviour that made
    # this rule necessary. It errs towards arrival deliberately. An absolute
    # 0.30 -- **6.25 m** under the geometry -- erred there far too hard for one
    # day, and every dropout in the whole approach counted as arrival.
    lost_close_margin: float = 0.10
    # How long the stop size must hold before the run ends. One frame is not
    # evidence: a walking body pitches, and two degrees of pitch move the
    # estimate from 0.94 m to between 0.76 and 1.22 m (ASSUMPTIONS G2), so a
    # single bad gait phase could end a run half a metre early. The robot holds
    # still while it confirms -- if the reading was real it has already stopped
    # where it meant to, and if it was a bob it carries on having lost 0.3 s.
    #
    # This costs nothing in safety: it can only make the robot stop LATER, by
    # at most 3 cm at 9 cm/s, and it stands still for the whole of it. With the
    # pitch correction fed by a real IMU the spikes get rarer, but they do not
    # vanish -- the correction is only exact while the robot is still (G10).
    stop_confirm_seconds: float = 0.3
    # A search turns in pulses rather than continuously: the picture from a
    # walking robot is blurred (ASSUMPTIONS F6) and the detector runs at a few
    # frames a second, so a robot that never stops turning never gets a clean
    # look at anything.
    search_turn_seconds: float = 0.6
    search_look_seconds: float = 0.5
    # Long enough to come round once, which 12 s was not: pulsing spends about
    # 55% of the time turning, so at a plausible 40 deg/s a full revolution
    # needs some 16 s. A search that cannot complete a circle gives up facing
    # away from a target that was there all along -- reproduced in simulation
    # 2026-08-23, and the reason this is 20 rather than 12. The turn rate is
    # still unmeasured (ASSUMPTIONS G4); measure it and this becomes an angle.
    search_seconds: float = 20.0
    # How long a target may be missing before the robot goes looking. Under it
    # the robot stands still -- it does not keep walking at something it can no
    # longer see.
    lost_grace: float = 0.8
    # The whole run, however it is going.
    timeout: float = 60.0
    # Which way to turn when there is no last known bearing to turn towards.
    default_search_turn: int = 1  # +1 right, -1 left

    def __post_init__(self) -> None:
        if not STOP_HEIGHT_MIN <= self.stop_height_fraction <= STOP_HEIGHT_MAX:
            raise ValueError(
                f"stop_height_fraction {self.stop_height_fraction} outside "
                f"{STOP_HEIGHT_MIN}..{STOP_HEIGHT_MAX}"
            )
        if self.default_search_turn not in (-1, 1):
            raise ValueError("default_search_turn must be -1 (left) or +1 (right)")
        if not 0.0 <= self.correct_exit_bearing <= self.correct_enter_bearing <= 1.0:
            raise ValueError("correct bearings must satisfy 0 <= exit <= enter <= 1")
        if not 0.0 <= self.turn_exit_bearing <= self.turn_enter_bearing <= 1.0:
            raise ValueError("turn bearings must satisfy 0 <= exit <= enter <= 1")
        if self.stop_confirm_seconds < 0.0:
            raise ValueError("stop_confirm_seconds must be >= 0")
        if not 0.0 <= self.lost_close_margin < self.stop_height_fraction:
            raise ValueError(
                f"lost_close_margin {self.lost_close_margin} must be >= 0 and smaller "
                f"than stop_height_fraction {self.stop_height_fraction}"
            )
        if self.turn_enter_bearing < self.correct_enter_bearing:
            raise ValueError("turning on the spot must start further out than steering does")
        if not 0.0 < self.keep_confidence <= self.acquire_confidence <= 1.0:
            raise ValueError("confidences must satisfy 0 < keep <= acquire <= 1")


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
    """Turn towards the target, walk to it, stop at a safe size. Then stay stopped.

    ``update`` is the whole behaviour. Call it once per tick with everything the
    detector last saw and the current time; it returns the drive to send. Once
    it reports a terminal state it keeps reporting it, and keeps commanding a
    stop, so a caller that ticks one more time cannot restart anything.
    """

    config: ApproachConfig = field(default_factory=ApproachConfig)
    state: BehaviourState = BehaviourState.SEARCHING
    reason: str = ""
    started_at: float | None = None
    _last_seen: float | None = None
    _last_bearing: float = 0.0
    # The bearing actually steered by: the measured one, low-passed. None until
    # a target is acquired, and cleared again whenever one is lost.
    _steered_bearing: float | None = None
    _last_height: float = 0.0
    _search_since: float | None = None
    _search_phase_since: float | None = None
    _turning: bool = True
    # Whether a target is currently being held. Decides which confidence the
    # detector's answers are judged against, and nothing else.
    _holding: bool = False
    # The two hysteresis latches, each carrying the direction it committed to.
    # Which side of a threshold the robot is on is state, not a fresh
    # comparison -- and so is which way it decided to go, because a latch that
    # holds only the magnitude reverses the moment the bearing changes sign.
    _turn_direction: int = 0
    _correct_direction: int = 0
    # When the stop size was first reached, or None while it is not.
    _big_since: float | None = None

    # --- the loop ---------------------------------------------------------

    def update(self, detections: Sequence[Detection], now: float, pitch_deg: float = 0.0) -> Intent:
        """One tick. ``pitch_deg`` is the body's own pitch, if it is known.

        Zero means "assume level", which is what every caller did before the
        IMU was on the wire and what a backend without one still does. Passing
        the real thing removes the largest error in the distance estimate;
        passing nothing leaves the behaviour exactly as it was.
        """
        if self.started_at is None:
            self.started_at = now
            self._search_since = now
            self._search_phase_since = now
        if self.state in _TERMINAL:
            return Intent(Drive(0, 0), self.state, self.reason)
        if now - self.started_at >= self.config.timeout:
            return self._give_up(f"timed out after {self.config.timeout:.0f}s")

        # A target already being approached is kept on weaker evidence than an
        # unknown one is acquired on -- see ApproachConfig.keep_confidence.
        threshold = self.config.keep_confidence if self._holding else self.config.acquire_confidence
        target = pick_target(detections, label=self.config.target, min_confidence=threshold)
        if target is not None:
            return self._approach(target, now, pitch_deg)
        return self._without_target(now)

    # --- with something in front of it ------------------------------------

    def _approach(self, target: Detection, now: float, pitch_deg: float = 0.0) -> Intent:
        bearing = self._smooth(target.bearing, now)
        size = level_height_fraction(target, pitch_deg)
        self._last_seen = now
        self._last_bearing = target.bearing
        self._last_height = size
        self._holding = True
        self._search_since = None
        if size >= self.config.stop_height_fraction:
            if self._big_since is None:
                self._big_since = now
            held = now - self._big_since
            if held >= self.config.stop_confirm_seconds:
                self.state = BehaviourState.ARRIVED
                self.reason = f"{self.config.target} fills {size * 100:.0f}% of the frame"
                return Intent(Drive(0, 0), self.state, self.reason, target)
            # Near enough to stop, not yet sure of it. Standing still is the
            # right thing to do while deciding: it is where the robot would end
            # up anyway if the reading holds.
            self.state = BehaviourState.APPROACHING
            self.reason = f"close enough -- confirming ({size:.2f}, {held:.1f}s)"
            return Intent(Drive(0, 0), self.state, self.reason, target)
        self._big_since = None
        self.state = BehaviourState.APPROACHING
        drive, note = self._steer(bearing)
        self.reason = f"{note} (bearing {bearing:+.2f}, size {size:.2f})"
        return Intent(drive, self.state, self.reason, target)

    def _smooth(self, bearing: float, now: float) -> float:
        """The measured bearing, low-passed towards the one to steer by.

        A first-order filter with a time constant rather than a fixed weight,
        because the two rates involved are unrelated and both vary: this is
        called once per control tick, and the detector underneath answers at
        its own pace. Weighting by elapsed time makes the filter behave the
        same whether it is fed twice a second or twenty times.

        Deliberately only on the bearing. The size decides arrival, and
        smoothing that would delay a stop -- which is the one decision here
        that must never be late.
        """
        tau = self.config.bearing_tau
        if tau <= 0 or self._steered_bearing is None or self._last_seen is None:
            self._steered_bearing = bearing
            return bearing
        elapsed = max(now - self._last_seen, 0.0)
        weight = 1.0 - math.exp(-elapsed / tau)
        self._steered_bearing += weight * (bearing - self._steered_bearing)
        return self._steered_bearing

    def _steer(self, bearing: float) -> tuple[Drive, str]:
        """Bearing to drive, with a latch on each band rather than a fresh test.

        Each band is entered at one bearing and left at a smaller one, and the
        gap is what the overshoot is allowed to use up. The robot turns by
        latching a move, and by the time a picture showing the target centred
        has arrived, been detected and been acted on, it has turned further.
        With a single threshold that overshoot lands on the far side and
        immediately commands the opposite turn -- the rocking left and right
        observed on the robot on 2026-08-23, closing on nothing.

        The latch carries the *direction*, not just the magnitude, and that is
        the half that is easy to get wrong: a magnitude-only latch is still
        latched when the bearing crosses centre, so it reverses the turn on the
        first overshoot and rocks exactly as before. Crossing centre therefore
        ENDS a turn rather than reversing it -- turning back needs the full
        entry bearing again, which an overshoot does not reach.
        """
        config = self.config
        offset = abs(bearing)
        heading = 1 if bearing > 0 else -1
        if self._turn_direction and self._turn_direction * bearing <= config.turn_exit_bearing:
            self._turn_direction = 0
        if not self._turn_direction and offset >= config.turn_enter_bearing:
            self._turn_direction = heading
        if (
            self._correct_direction
            and self._correct_direction * bearing <= config.correct_exit_bearing
        ):
            self._correct_direction = 0
        if not self._correct_direction and offset >= config.correct_enter_bearing:
            self._correct_direction = heading
        if self._turn_direction:
            return Drive(0, self._turn_direction), "turning towards it"
        if self._correct_direction:
            return Drive(1, self._correct_direction), "walking, correcting"
        return Drive(1, 0), "walking towards it"

    # --- with nothing in front of it --------------------------------------

    def _without_target(self, now: float) -> Intent:
        config = self.config
        if self._last_seen is not None and now - self._last_seen < config.lost_grace:
            # Deliberately a full stop rather than "carry on for a moment": the
            # robot is walking at a person it can no longer see. A stutter is a
            # cheap price for never moving blind.
            self.state = BehaviourState.APPROACHING
            self.reason = "lost sight of it -- holding"
            return Intent(Drive(0, 0), self.state, self.reason)

        # Losing a target that had grown large is not loss, it is arrival.
        #
        # Walking at a person from a camera a hand's width off the floor ends
        # with the person filling the frame -- and a detector shown a fraction
        # of a person stops calling it one. The old reading of that moment was
        # "gone, go and look for it", so the robot turned away at exactly the
        # point it had succeeded. Observed on the robot 2026-08-23.
        #
        # The failure mode of getting this wrong is benign in the one direction
        # that matters: it stops the robot. A tracker asked to bridge the same
        # gap fails the other way -- it keeps reporting a box, and the robot
        # walks at a drifted guess of a person it can no longer see.
        if self._holding and self._last_height >= self.lost_close_height:
            self.state = BehaviourState.ARRIVED
            self.reason = (
                f"arrived: lost sight of the {config.target} at "
                f"{self._last_height * 100:.0f}% of the frame -- too close to see it whole"
            )
            return Intent(Drive(0, 0), self.state, self.reason)

        if self._search_since is None:
            self._search_since = now
            self._search_phase_since = now
            self._turning = True
            # A fresh look starts with fresh latches: which side of a steering
            # threshold the robot was on before it lost the target says nothing
            # about the one it finds next.
            self._holding = False
            self._big_since = None
            self._steered_bearing = None
            self._turn_direction = 0
            self._correct_direction = 0
        if now - self._search_since >= config.search_seconds:
            return self._give_up(f"no {config.target} found in {config.search_seconds:.0f}s")

        self.state = BehaviourState.SEARCHING
        turn = self.config.default_search_turn
        if self._last_seen is not None and self._last_bearing != 0.0:
            turn = 1 if self._last_bearing > 0 else -1
        # Pulse: turn, then stand still long enough for the detector to get a
        # frame that is not smeared.
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

    @property
    def lost_close_height(self) -> float:
        """The size at or above which losing the target counts as arrival."""
        return self.config.stop_height_fraction - self.config.lost_close_margin

    def _give_up(self, reason: str) -> Intent:
        self.state = BehaviourState.LOST
        self.reason = reason
        return Intent(Drive(0, 0), self.state, self.reason)
