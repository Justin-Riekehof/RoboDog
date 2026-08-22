"""The camera parameter table: one source for the sliders and the validator."""

from __future__ import annotations

import pytest

from robodog.api.types import SetCameraParam
from robodog.camera import CAMERA_PARAMS, DEFAULTS, FRAME_SIZES, PARAMS_BY_NAME
from robodog.errors import LimitViolationError
from robodog.safety.limits import LimitConfig, check_command


def test_every_default_is_inside_its_own_range() -> None:
    """A table whose defaults its own validator would refuse would open the UI
    on controls the robot rejects the moment they are touched."""
    for param in CAMERA_PARAMS:
        if param.kind == "action":
            continue
        assert param.clamps(param.default), f"{param.name} default is out of range"


def test_choices_cover_their_range_exactly() -> None:
    """A select is drawn from `choices` and validated against min/max; a gap
    between them is either an option that is refused or a value with no name."""
    for param in CAMERA_PARAMS:
        if param.kind != "choice":
            continue
        assert len(param.choices) == param.maximum - param.minimum + 1, param.name


def test_the_frame_size_list_reaches_what_the_fork_allocates() -> None:
    """The ceiling is chosen at boot, not at compile time.

    This table first stopped at QVGA, reading `ROBODOG_CAM_MAX_SIZE` as the
    constant it is initialised to -- but the fork overwrites it from the config
    it hands esp_camera_init, which asks for VGA and only drops to QVGA if that
    allocation fails. Measured on the robot 2026-08-22: `size=8/8`, VGA, on a
    unit with no PSRAM at all. Stopping at QVGA hid three usable resolutions.
    """
    assert FRAME_SIZES[-1] == "640x480"  # FRAMESIZE_VGA, what the fork asks for
    assert len(FRAME_SIZES) == 9  # framesize_t 0..8, contiguous
    assert PARAMS_BY_NAME["size"].maximum == len(FRAME_SIZES) - 1
    # The firmware clamps rather than rejects, so offering more than a given
    # robot allocated is silent but harmless -- and nothing over Wi-Fi can ask
    # which it was, so the note has to say so.
    assert "clamps" in PARAMS_BY_NAME["size"].note


def test_the_setters_this_sensor_does_not_have_are_absent() -> None:
    """The OV2640 driver of this core leaves some function pointers null, and
    the firmware skips those writes. A slider for one would move and change
    nothing, which reads as a broken camera rather than an absent feature."""
    assert "sharpness" not in PARAMS_BY_NAME
    assert "denoise" not in PARAMS_BY_NAME


def test_the_safety_layer_refuses_what_the_sensor_would_ignore() -> None:
    limits = LimitConfig()
    check_command(SetCameraParam("ae_level", 2), limits)  # the top of the range
    with pytest.raises(LimitViolationError, match="outside"):
        check_command(SetCameraParam("ae_level", 3), limits)
    with pytest.raises(LimitViolationError, match="unknown camera parameter"):
        check_command(SetCameraParam("exposure", 1), limits)


def test_an_action_takes_any_value() -> None:
    """`reset` carries no value; refusing it on range would be refusing it for
    a number nobody chose."""
    check_command(SetCameraParam("reset", 0), LimitConfig())


def test_defaults_describe_every_control() -> None:
    assert set(DEFAULTS) == {p.name for p in CAMERA_PARAMS if p.kind != "action"}
