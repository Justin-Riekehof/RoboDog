"""YOLO detection, behind the optional ``vision`` extra.

The only module in the package that imports ultralytics, and nothing imports it
unless a detector is actually asked for (``load_detector("yolo")``). That is the
same arrangement ``matplotlib`` and ``mujoco`` have: a no-extras install, and CI,
must stay green without torch on disk.

Boxes come back from ultralytics already normalised (``xyxyn``), which is what
:class:`robodog.vision.Box` wants -- so no resolution ever enters this codebase,
and changing the frame size from the Camera tab mid-session changes nothing
downstream.
"""

from __future__ import annotations

import io
from typing import Any, Final

from robodog.errors import VisionError
from robodog.vision import Box, Detection

# Nano by default: the machine that runs this also runs a 27B language model,
# and the loop needs a few frames a second, not the best mAP available.
DEFAULT_MODEL: Final = "yolo11n.pt"


class YoloDetector:
    """Ultralytics YOLO over JPEG frames.

    The confidence floor is deliberately low. It is not the decision threshold
    -- the behaviour has two of those, one to acquire a target and a weaker one
    to keep it (`ApproachConfig.keep_confidence`) -- and a detector that filters
    at the acquisition level makes the keeping level unreachable. Reporting a
    weak detection and letting the caller ignore it is the layering that works;
    it was the other way round for a day, and the keep threshold was dead code.

    The model file is downloaded by ultralytics on first use and cached in its
    own directory; after that this works offline. Loading is deferred to the
    first frame so that constructing one costs nothing and a bad model name
    fails where the operator can see it.
    """

    name = "yolo"

    def __init__(
        self,
        *,
        model: str | None = None,
        min_confidence: float = 0.20,
        device: str | None = None,
    ) -> None:
        self.model_name = model or DEFAULT_MODEL
        self.min_confidence = min_confidence
        self.device = device
        self._model: Any = None
        self._image: Any = None

    def _load(self) -> Any:
        if self._model is not None:
            return self._model
        try:
            from PIL import Image
            from ultralytics import YOLO
        except ImportError as exc:  # pragma: no cover - depends on the extra
            raise VisionError(
                "the 'vision' extra is not installed, so there is no detector: "
                "uv sync --extra vision (it pulls ultralytics and torch)"
            ) from exc
        self._image = Image
        try:
            self._model = YOLO(self.model_name)
        except Exception as exc:  # ultralytics raises plain exceptions
            raise VisionError(f"cannot load YOLO model {self.model_name!r}: {exc}") from exc
        return self._model

    def detect(self, frame: bytes) -> tuple[Detection, ...]:
        model = self._load()
        try:
            image = self._image.open(io.BytesIO(frame)).convert("RGB")
        except Exception as exc:
            raise VisionError(f"frame is not a readable JPEG: {exc}") from exc
        kwargs: dict[str, Any] = {"conf": self.min_confidence, "verbose": False}
        if self.device:
            kwargs["device"] = self.device
        results = model.predict(image, **kwargs)
        if not results:
            return ()
        boxes = getattr(results[0], "boxes", None)
        if boxes is None or len(boxes) == 0:
            return ()
        names = getattr(model, "names", {})
        found: list[Detection] = []
        for xyxyn, cls, conf in zip(
            boxes.xyxyn.tolist(), boxes.cls.tolist(), boxes.conf.tolist(), strict=True
        ):
            left, top, right, bottom = (float(v) for v in xyxyn)
            label = str(names.get(int(cls), int(cls))) if isinstance(names, dict) else str(int(cls))
            found.append(
                Detection(
                    label=label,
                    confidence=float(conf),
                    box=Box(left=left, top=top, right=right, bottom=bottom).clamped(),
                )
            )
        found.sort(key=lambda d: d.confidence, reverse=True)
        return tuple(found)

    def close(self) -> None:
        self._model = None
