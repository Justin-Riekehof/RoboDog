"""What the camera sees, in terms a behaviour can act on.

Three ideas, and the split between them is the point:

* :class:`Box` and :class:`Detection` are **normalised**. A box is fractions of
  the frame, never pixels, so nothing downstream has to know the resolution --
  which is just as well, since the frame size is a per-robot ceiling the
  operator can change from the Camera tab mid-session (ASSUMPTIONS F4). The two
  numbers a behaviour actually steers by, ``bearing`` and ``height_fraction``,
  are derived from the box rather than stored beside it, so they cannot drift
  apart from the rectangle the page draws.
* :class:`Detector` is a protocol, exactly like ``Backend`` is. The YOLO
  implementation lives behind the optional ``vision`` extra in
  :mod:`robodog.vision.yolo` and is imported nowhere else, so a no-extras
  install and CI stay green without ultralytics or torch.
* :class:`ScriptedDetector` is to vision what ``MockBackend`` is to the robot:
  a deterministic stand-in that needs no model, no camera and no GPU. Tests use
  it, and so does ``--detector scripted``, which is what makes the whole loop
  demonstrable with the robot switched off.

Bearing is signed the way the *picture* is: -1 is the left edge of the frame,
+1 the right. Whether that is also the robot's left and right depends on the
camera not being mirrored -- see ASSUMPTIONS G1, and the Mirror toggle in the
Camera tab if it turns out to be wrong.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

__all__ = [
    "Box",
    "Detection",
    "Detector",
    "ScriptedDetector",
    "load_detector",
]


@dataclass(frozen=True, slots=True)
class Box:
    """An axis-aligned rectangle in frame fractions: 0..1, origin top-left."""

    left: float
    top: float
    right: float
    bottom: float

    def __post_init__(self) -> None:
        if self.right < self.left or self.bottom < self.top:
            raise ValueError(f"box is inside out: {self}")

    @property
    def width(self) -> float:
        return self.right - self.left

    @property
    def height(self) -> float:
        return self.bottom - self.top

    @property
    def center_x(self) -> float:
        return (self.left + self.right) / 2.0

    @property
    def center_y(self) -> float:
        return (self.top + self.bottom) / 2.0

    def clamped(self) -> Box:
        """The part of the box that is actually inside the frame.

        A detector may report a person whose legs leave the bottom of the
        picture, and a box that runs past the edge would make
        ``height_fraction`` claim the target is nearer than the visible
        evidence supports. Clamping keeps the distance cue conservative in the
        one direction that matters -- it can only ever say "further away".
        """
        return Box(
            left=min(max(self.left, 0.0), 1.0),
            top=min(max(self.top, 0.0), 1.0),
            right=min(max(self.right, 0.0), 1.0),
            bottom=min(max(self.bottom, 0.0), 1.0),
        )


@dataclass(frozen=True, slots=True)
class Detection:
    """One thing the detector found, with the two numbers a behaviour steers by.

    ``bearing`` and ``height_fraction`` are derived from ``box`` rather than
    stored: the page draws the box, the state machine reads the derived pair,
    and there is exactly one source for both.
    """

    label: str
    confidence: float
    box: Box

    @property
    def bearing(self) -> float:
        """Where it is across the frame: -1 hard left, 0 centred, +1 hard right."""
        return 2.0 * self.box.center_x - 1.0

    @property
    def height_fraction(self) -> float:
        """How much of the frame's height it fills -- the only distance cue there is.

        Not a distance. Turning it into millimetres needs one calibration
        session against the real robot (ASSUMPTIONS G2), and nothing on the
        safety path waits for that: the behaviour stops on this fraction
        directly.
        """
        return self.box.height


@runtime_checkable
class Detector(Protocol):
    """Frames in, detections out. The one seam every vision source fits."""

    name: str

    def detect(self, frame: bytes) -> tuple[Detection, ...]:
        """Detections in one JPEG frame, strongest first."""
        ...

    def close(self) -> None:
        """Release whatever the implementation holds (a model, a device)."""
        ...


class ScriptedDetector:
    """A detector that answers from a script instead of from a picture.

    The frame is ignored -- deliberately, because the point is to exercise the
    behaviour, the runner and the page without a model or a robot. Each call
    takes the next entry; the last one repeats forever, so a script ends in the
    state it should be tested holding.
    """

    name = "scripted"

    def __init__(self, script: Iterable[Sequence[Detection]] = ()) -> None:
        self._script: list[tuple[Detection, ...]] = [tuple(step) for step in script]
        self._index = 0
        self.calls = 0

    def detect(self, frame: bytes) -> tuple[Detection, ...]:
        self.calls += 1
        if not self._script:
            return ()
        step = self._script[min(self._index, len(self._script) - 1)]
        self._index += 1
        return step

    def close(self) -> None:
        return None


def load_detector(
    name: str,
    *,
    model: str | None = None,
    min_confidence: float = 0.20,
    device: str | None = None,
) -> Detector:
    """Build a detector by name; the only place the optional extra is reached for.

    ``scripted`` needs nothing and finds nothing, which makes it useful for
    walking through the plumbing offline. ``yolo`` needs the ``vision`` extra,
    and the import stays inside this branch so that everything else in the
    package keeps working without it.

    ``device`` is passed through rather than decided here: on a machine that is
    also serving a language model the GPU is not automatically the right answer,
    and nothing in this file is in a position to know.
    """
    if name == "scripted":
        return ScriptedDetector()
    if name == "yolo":
        from robodog.vision.yolo import YoloDetector

        return YoloDetector(model=model, min_confidence=min_confidence, device=device)
    raise ValueError(f"unknown detector {name!r} (valid: scripted, yolo)")
