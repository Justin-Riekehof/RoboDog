"""The ESP32 camera's tunable parameters, as one table.

Two consumers read it and must not disagree: the safety layer, which refuses a
value outside its range before it reaches the robot, and the teach UI, which
draws a control per entry. A table means a new parameter appears in both at
once, and that a slider's end stops are the same numbers the validator uses.

The names are the firmware's own (`cam_<name>`, see `robodogCameraSet` in
firmware/wavego-robodog/app_httpd.cpp), so nothing here translates anything.

Two things are worth knowing before trusting a control:

* **There is no readback over Wi-Fi.** `cam_report` prints to the serial
  console, not to the HTTP response, so the values here are what the firmware
  applies at boot (`robodogCameraTune`) plus whatever we have sent since. A
  camera someone tuned over USB will disagree with the sliders until `reset`.
* **Not every setter exists.** The OV2640 driver of this core generation leaves
  some function pointers null -- `sharpness` and `denoise` are the known ones --
  and the firmware skips those writes rather than crashing. A slider for them
  would move and change nothing, so they are deliberately absent below.

See ASSUMPTIONS F4 (resolution ceiling), F5 (what a frame costs) and F6 (what
the fork's defaults buy and what they cost).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Literal

Kind = Literal["range", "toggle", "choice", "action"]


@dataclass(frozen=True, slots=True)
class CameraParam:
    """One tunable, with everything a control and a validator both need."""

    name: str
    label: str
    kind: Kind
    group: str
    minimum: int = 0
    maximum: int = 1
    default: int = 0
    note: str = ""
    # Labels for a "choice", in value order starting at `minimum`.
    choices: tuple[str, ...] = ()

    def clamps(self, value: int) -> bool:
        return self.minimum <= value <= self.maximum


# Frame sizes, in the firmware's own enum order (framesize_t), up to the
# largest the fork will ever allocate.
#
# The ceiling is NOT a constant: the frame buffer is chosen once, before
# esp_camera_init, from how much memory there is, and set_framesize never grows
# it afterwards. The fork asks for VGA and drops to QVGA only if that
# allocation fails -- measured on this robot 2026-08-22: `size=8/8`, VGA, with
# no PSRAM at all. Listing only up to QVGA, as this table first did, hid three
# usable resolutions.
#
# The firmware clamps rather than rejects, so asking for more than a given
# robot allocated is harmless but silent, and there is no way to ask over Wi-Fi
# which it was (`cam_report` answers on the serial console). See ASSUMPTIONS F4.
FRAME_SIZES: Final = (
    "96x96",
    "160x120",
    "176x144",
    "240x176",
    "240x240",
    "320x240",
    "400x296",
    "480x320",
    "640x480",
)

# Gain ceilings, in gainceiling_t order.
GAIN_CEILINGS: Final = ("2x", "4x", "8x", "16x", "32x", "64x", "128x")

WHITE_BALANCE_MODES: Final = ("auto", "sunny", "cloudy", "office", "home")

CAMERA_PARAMS: Final[tuple[CameraParam, ...]] = (
    # --- exposure: the group that actually decides whether you can see -----
    CameraParam(
        "aec",
        "Auto exposure",
        "toggle",
        "Exposure",
        default=1,
        note="off means aec_value sets the integration time by hand",
    ),
    CameraParam("aec2", "DSP auto exposure", "toggle", "Exposure", default=1),
    CameraParam(
        "ae_level",
        "Exposure target",
        "range",
        "Exposure",
        minimum=-2,
        maximum=2,
        default=2,
        note="what the AEC aims for; the fork ships at the top of the range",
    ),
    CameraParam(
        "aec_value",
        "Manual exposure",
        "range",
        "Exposure",
        maximum=1200,
        default=300,
        note="only has an effect with auto exposure off; long is motion blur",
    ),
    CameraParam("agc", "Auto gain", "toggle", "Exposure", default=1),
    CameraParam(
        "agc_gain",
        "Manual gain",
        "range",
        "Exposure",
        maximum=30,
        default=0,
        note="only with auto gain off",
    ),
    CameraParam(
        "gainceiling",
        "Gain ceiling",
        "choice",
        "Exposure",
        maximum=6,
        default=3,
        choices=GAIN_CEILINGS,
        note="only matters in the dark, where more gain is more noise",
    ),
    # --- the picture itself -------------------------------------------------
    CameraParam(
        "size",
        "Frame size",
        "choice",
        "Image",
        maximum=len(FRAME_SIZES) - 1,
        default=8,
        choices=FRAME_SIZES,
        note="the robot silently clamps this to what it allocated at boot (F4)",
    ),
    CameraParam(
        "quality",
        "JPEG quality",
        "range",
        "Image",
        minimum=10,
        maximum=63,
        default=10,
        note="LOWER is better and bigger; 63 is the vendor's, 10 is what we boot with",
    ),
    CameraParam("brightness", "Brightness", "range", "Image", minimum=-2, maximum=2, default=0),
    CameraParam("contrast", "Contrast", "range", "Image", minimum=-2, maximum=2, default=0),
    CameraParam(
        "saturation",
        "Saturation",
        "range",
        "Image",
        minimum=-2,
        maximum=2,
        default=0,
        note="the vendor ships +2, which is why its blues bloom",
    ),
    CameraParam("raw_gma", "Gamma", "toggle", "Image", default=1, note="lifts the shadows"),
    CameraParam("lenc", "Lens correction", "toggle", "Image", default=1, note="the corners"),
    # --- colour -------------------------------------------------------------
    CameraParam("awb", "Auto white balance", "toggle", "Colour", default=1),
    CameraParam("awb_gain", "AWB gain", "toggle", "Colour", default=1, note="the green cast"),
    CameraParam(
        "wb_mode",
        "Lighting",
        "choice",
        "Colour",
        maximum=len(WHITE_BALANCE_MODES) - 1,
        default=0,
        choices=WHITE_BALANCE_MODES,
        note="a preset; only used with auto white balance off",
    ),
    # --- how the picture is oriented ---------------------------------------
    CameraParam("hmirror", "Mirror", "toggle", "Orientation", default=0),
    CameraParam("vflip", "Flip", "toggle", "Orientation", default=0),
    # --- and the way back ---------------------------------------------------
    CameraParam(
        "reset",
        "Back to the fork's defaults",
        "action",
        "Orientation",
        note="re-applies robodogCameraTune, the values above",
    ),
)

PARAMS_BY_NAME: Final[dict[str, CameraParam]] = {p.name: p for p in CAMERA_PARAMS}

DEFAULTS: Final[dict[str, int]] = {p.name: p.default for p in CAMERA_PARAMS if p.kind != "action"}

# Where the MJPEG stream lives. The firmware starts a second server one port
# above the control one and registers /stream on it.
STREAM_PORT: Final = 81
STREAM_PATH: Final = "/stream"
