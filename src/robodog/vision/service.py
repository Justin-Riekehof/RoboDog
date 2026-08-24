"""The vision loop: one reader, one detector, and everyone else reads from here.

Assembles the pieces in :mod:`robodog.vision.stream` into the thing the teach UI
and a behaviour both talk to. Two threads and no more: one reads the robot's
MJPEG stream into a :class:`FrameHub`, the other takes whatever frame is newest
and runs the detector on it.

The detector deliberately does **not** see every frame. It takes the latest one
each time it comes free, so a slow model drops frames instead of building a
queue -- a behaviour steering by a detection from four seconds ago would be
worse than one steering by nothing.

Why the host reads the stream at all, rather than pointing the browser at the
robot: this unit has no PSRAM, so the firmware runs a single frame buffer and a
browser holding `/stream` blocks every other grab (ASSUMPTIONS F4/G3). There can
be one consumer. This is it, and the page gets its picture from here.
"""

from __future__ import annotations

import threading
import time
import urllib.request
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any, Final

from robodog.errors import VisionError
from robodog.vision import Detection, Detector
from robodog.vision.stream import FrameHub, MjpegReader

# What a still source plays at. Fast enough to feel live, slow enough that a
# detector on a busy machine keeps up.
FILE_SOURCE_FPS: Final = 5.0
# How long an answer from the detector stays valid. This is a safety number,
# not a performance one: without it a stalled detector -- a frozen stream, a
# second viewer stealing the robot's single frame buffer (ASSUMPTIONS F4/G3),
# a crashed model -- would go on handing out its last answer forever, and a
# behaviour reading it would keep walking at a person who is no longer there.
# Generous against a normal update interval (a few tenths of a second at the
# rates measured) so that it expires only when something has really stopped.
DETECTION_TTL: Final = 1.0
_JPEG_SUFFIXES: Final = (".jpg", ".jpeg")


class FileFrameSource:
    """Publishes JPEGs from disk in a loop, in place of a camera.

    Exists because the robot spends most of its life switched off and the rest
    of this needs exercising anyway: point ``--vision-source`` at a picture or a
    folder of them and the whole chain -- hub, re-broadcast, detector,
    behaviour, page -- runs with no hardware in it. The frames are real, so a
    real detector really detects on them.
    """

    def __init__(
        self,
        source: str | Path,
        hub: FrameHub,
        *,
        fps: float = FILE_SOURCE_FPS,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        path = Path(source)
        if path.is_dir():
            self.paths = sorted(p for p in path.iterdir() if p.suffix.lower() in _JPEG_SUFFIXES)
        elif path.is_file():
            self.paths = [path]
        else:
            raise VisionError(f"no such frame source: {path}")
        if not self.paths:
            raise VisionError(f"{path} holds no .jpg frames")
        if fps <= 0:
            raise VisionError(f"frame rate must be positive, got {fps}")
        self.hub = hub
        self.interval = 1.0 / fps
        self._sleep = sleep
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def _run(self) -> None:
        index = 0
        while not self._stop.is_set():
            self.hub.publish(self.paths[index % len(self.paths)].read_bytes())
            index += 1
            if self._stop.wait(self.interval):
                return

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="vision-files")
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None


class VisionService:
    """Frames from one source, detections from one detector, for many readers.

    Everything a caller needs is a snapshot: :attr:`detections` is the most
    recent complete answer, never a partially updated one, and it is replaced
    atomically rather than mutated.
    """

    def __init__(
        self,
        source: str,
        *,
        detector: Detector | None = None,
        stream_timeout: float = 5.0,
        detection_ttl: float = DETECTION_TTL,
        clock: Callable[[], float] = time.monotonic,
        opener: urllib.request.OpenerDirector | None = None,
    ) -> None:
        self.source = source
        self.detector = detector
        self.hub = FrameHub(clock=clock)
        self._clock = clock
        self.detection_ttl = detection_ttl
        # When the answer was produced and what it was, as ONE value: a reader
        # that took the timestamp of one answer and the detections of the next
        # would be exactly the reader this expiry exists to protect.
        self._answer: tuple[float, tuple[Detection, ...]] = (float("-inf"), ())
        self._detected_seq = 0
        self._detect_seconds: float | None = None
        self._detect_note = ""
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._reader: MjpegReader | FileFrameSource
        if source.startswith(("http://", "https://")):
            self._reader = MjpegReader(source, self.hub, timeout=stream_timeout, opener=opener)
        else:
            self._reader = FileFrameSource(source, self.hub)

    # --- lifecycle --------------------------------------------------------

    def start(self) -> None:
        self._stop.clear()
        self._reader.start()
        if self.detector is not None:
            self._thread = threading.Thread(
                target=self._detect_loop, daemon=True, name="vision-detect"
            )
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.hub.close()
        self._reader.stop()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        if self.detector is not None:
            self.detector.close()

    # --- what everyone else reads ----------------------------------------

    @property
    def detections(self) -> tuple[Detection, ...]:
        """What is visible *now*: the newest answer, or nothing if it has aged out.

        Never blocks and never partial -- and never stale, which is the part
        that matters. A detector that has stopped answering must look like a
        camera that sees nothing, because to everything downstream those two are
        the same situation: there is no current evidence that anyone is there.
        Handing back the last answer instead would let a behaviour walk forward
        on a picture that no longer exists, which is the one thing the "never
        move blind" invariant is for.
        """
        stamp, found = self._answer
        if self._clock() - stamp > self.detection_ttl:
            return ()
        return found

    @property
    def stale(self) -> bool:
        """True when the detector has not answered inside its time to live."""
        return self._clock() - self._answer[0] > self.detection_ttl

    def status(self) -> dict[str, Any]:
        """Everything the page shows about the vision loop, in one snapshot."""
        seq, frame = self.hub.latest()
        return {
            "source": self.source,
            "detector": self.detector.name if self.detector is not None else None,
            "frames": seq,
            "fps": round(self.hub.rate, 1),
            "live": frame is not None,
            "detect_seconds": self._detect_seconds,
            "detected_frame": self._detected_seq,
            "stale": self.detector is not None and self.stale,
            "note": self._detect_note or self.hub.note,
        }

    # --- the detector thread ---------------------------------------------

    def _detect_loop(self) -> None:
        assert self.detector is not None
        seen = 0
        while not self._stop.is_set():
            seq, frame = self.hub.wait_for(seen)
            if frame is None or seq == seen:
                continue
            seen = seq
            started = self._clock()
            try:
                found = self.detector.detect(frame)
            except VisionError as exc:
                self._detect_note = str(exc)
                # A detector that cannot read this frame is unlikely to read the
                # next one either, but the stream may simply have hiccuped --
                # so back off rather than spin, and clear the answer rather than
                # leave the old one standing.
                self._answer = (self._clock(), ())
                if self._stop.wait(1.0):
                    return
                continue
            except Exception as exc:  # a model can fail in ways we do not own
                self._detect_note = f"detector failed: {exc}"
                self._answer = (self._clock(), ())
                if self._stop.wait(1.0):
                    return
                continue
            self._detect_seconds = round(self._clock() - started, 3)
            self._detected_seq = seq
            self._detect_note = ""
            self._answer = (self._clock(), found)


def detection_json(
    detections: Sequence[Detection], *, distance: Callable[[float], float | None] | None = None
) -> list[dict[str, Any]]:
    """Detections as the page draws them: the box, and the two derived numbers.

    The page draws rectangles the server computed, exactly as it draws leg
    polylines the server computed -- the browser stays a dumb terminal, and the
    boxes it draws are in frame fractions, so they land correctly whatever
    resolution the camera is set to.
    """
    payload: list[dict[str, Any]] = []
    for found in detections:
        entry: dict[str, Any] = {
            "label": found.label,
            "confidence": round(found.confidence, 3),
            "box": {
                "left": round(found.box.left, 4),
                "top": round(found.box.top, 4),
                "right": round(found.box.right, 4),
                "bottom": round(found.box.bottom, 4),
            },
            "bearing": round(found.bearing, 3),
            "height_fraction": round(found.height_fraction, 3),
        }
        if distance is not None:
            estimate = distance(found.height_fraction)
            entry["distance_mm"] = round(estimate) if estimate is not None else None
        payload.append(entry)
    return payload
