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


def test_the_frame_size_ceiling_matches_the_frame_buffer() -> None:
    """The buffer is allocated once, at esp_camera_init, and set_framesize never
    grows it -- asking for more than the robot booted with ends in no image at
    all, which is exactly the fault F2 turned out not to be (ASSUMPTIONS F4)."""
    assert FRAME_SIZES[-1] == "320x240"  # FRAMESIZE_QVGA, the firmware's cap
    assert PARAMS_BY_NAME["size"].maximum == len(FRAME_SIZES) - 1


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
