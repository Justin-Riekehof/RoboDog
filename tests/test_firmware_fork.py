"""The firmware fork stays a readable delta on the vendored reference.

There is no compiler here, so these tests cannot say the firmware is correct.
What they can say is what a reviewer needs before trusting a flash: that the
fork only *adds* to the upstream source, and that the file our kinematics port
mirrors is untouched -- because the moment `ServoCtrl.h` diverges, the Python
port stops being a port of anything (CLAUDE.md, ASSUMPTIONS section C).
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
VENDOR = ROOT / "vendor/wavego-firmware"
UPSTREAM = ROOT / "firmware/wavego-upstream"
FORK = ROOT / "firmware/wavego-robodog"

# Files the fork carries that the vendored excerpt does not: the reference copy
# is for reading the protocol and the kinematics, the fork has to compile.
FORK_ONLY = ("PreferencesConfig.h", "WebPage.h")
# Files that must stay byte-identical to the reference.
UNTOUCHED = ("ServoCtrl.h", "InitConfig.h")
# Files the watchdog is allowed to change.
PATCHED = ("WAVEGO.ino", "app_httpd.cpp")


def lines(path: Path) -> list[str]:
    """Content lines, line endings normalised -- upstream ships CRLF."""
    return (
        path.read_text(encoding="utf-8", errors="surrogateescape").replace("\r\n", "\n").split("\n")
    )


def test_the_fork_carries_a_complete_sketch() -> None:
    """The vendored excerpt cannot compile: WAVEGO.ino includes
    PreferencesConfig.h and app_httpd.cpp includes WebPage.h, neither of which
    is in vendor/. A fork that inherited that hole would be undiscoverable."""
    for name in (*UNTOUCHED, *PATCHED, *FORK_ONLY, "LICENSE", "README.md"):
        assert (FORK / name).is_file(), f"{name} missing from the fork"


def test_the_kinematics_header_is_untouched() -> None:
    """`robodog.kinematics` is a line-faithful port of ServoCtrl.h. If the fork
    edits it, the port silently describes a firmware that no longer exists."""
    for name in UNTOUCHED:
        assert lines(VENDOR / name) == lines(FORK / name), f"{name} diverged from the reference"


def test_the_baseline_is_flashable_and_pristine() -> None:
    """The vendored excerpt cannot compile and the fork is not a fallback, so
    the way back to a stock robot has to exist as its own complete sketch."""
    for name in (*UNTOUCHED, *PATCHED, *FORK_ONLY, "LICENSE", "README.md"):
        assert (UPSTREAM / name).is_file(), f"{name} missing from the baseline"
    assert (UPSTREAM / "UPSTREAM_COMMIT.txt").is_file()
    # The four files the Python port reads must be the same in all three places,
    # or "line-faithful port" is a claim about nothing in particular.
    for name in (*UNTOUCHED, *PATCHED):
        assert lines(VENDOR / name) == lines(UPSTREAM / name), (
            f"{name}: the pinned reference and the baseline disagree"
        )


@pytest.mark.parametrize("name", PATCHED)
def test_the_fork_only_adds_lines(name: str) -> None:
    """Every upstream line survives, in order. A fork that only adds is one a
    reviewer can check by reading the additions, without re-reading the whole
    firmware to find what was quietly rewritten.
    """
    original, forked = lines(UPSTREAM / name), lines(FORK / name)
    index = 0
    for line in forked:
        if index < len(original) and line == original[index]:
            index += 1
    assert index == len(original), (
        f"{name}: {len(original) - index} upstream line(s) were changed or removed, "
        f"not merely added to"
    )
    assert len(forked) > len(original)


@pytest.mark.parametrize("name", PATCHED)
def test_every_addition_is_marked_as_ours(name: str) -> None:
    """Additions carry the RoboDog marker or sit inside a marked block, so the
    delta is greppable on the robot's own source years from now."""
    original = set(lines(UPSTREAM / name))
    inside_block = False
    unmarked: list[str] = []
    for line in lines(FORK / name):
        if "=== RoboDog" in line:
            inside_block = True
        if line in original or not line.strip():
            continue
        if inside_block or re.search(r"[Rr]obo[Dd]og|ROBODOG|watchdog|ping", line):
            if "=== end RoboDog" in line:
                inside_block = False
            continue
        unmarked.append(line)
    assert not unmarked, f"{name}: unmarked additions: {unmarked[:5]}"


def test_the_watchdog_is_off_until_it_is_asked_for() -> None:
    """The vendor's web UI sends nothing while the robot walks. A watchdog that
    defaulted to on would stop the robot under the stock page every few seconds
    and look like a hardware fault."""
    source = (FORK / "WAVEGO.ino").read_text(encoding="utf-8", errors="surrogateescape")
    assert "extern unsigned long ROBODOG_WATCHDOG_MS = 0;" in source
    assert "if(ROBODOG_WATCHDOG_MS == 0){return;}" in source


def test_the_watchdog_only_stops_something_that_moves() -> None:
    """Trimming a servo (debugMode) and standing still must never be cut into."""
    source = (FORK / "WAVEGO.ino").read_text(encoding="utf-8", errors="surrogateescape")
    check = source.split("robodogWatchdogCheck")[1]
    assert "if(moveFB == 0 && moveLR == 0){return;}" in check
    assert "moveFB = 0;" in check and "moveLR = 0;" in check
    assert "funcMode = 2;" in check  # stop, then crouch


def test_both_transports_learn_the_new_commands() -> None:
    """A watchdog that only exists on one transport is a trap for the other."""
    ino = (FORK / "WAVEGO.ino").read_text(encoding="utf-8", errors="surrogateescape")
    httpd = (FORK / "app_httpd.cpp").read_text(encoding="utf-8", errors="surrogateescape")
    assert 'docReceive["var"] == "watchdog"' in ino
    assert 'docReceive["var"] == "ping"' in ino
    assert '!strcmp(variable, "watchdog")' in httpd
    assert '!strcmp(variable, "ping")' in httpd
    # ... and every accepted command feeds it, so a command added later cannot
    # silently be the one that starves it.
    assert ino.count("robodogWatchdogFeed()") >= 2
    assert "robodogWatchdogFeed();" in httpd


@pytest.mark.parametrize("name", PATCHED)
def test_braces_balance(name: str) -> None:
    """The cheapest possible stand-in for a compiler we do not have here."""
    source = (FORK / name).read_text(encoding="utf-8", errors="surrogateescape")
    assert source.count("{") == source.count("}")
    assert source.count("(") == source.count(")")


# --- both sketches must build the same way ------------------------------------------


def test_both_sketches_share_one_build_configuration() -> None:
    """ "The fork behaves like the baseline" is only evidence if the two were
    built with the same platform, core and partition table. Separate copies of
    those settings would let that quietly stop being true."""
    common = ROOT / "firmware/common.ini"
    assert common.is_file()
    for folder in (UPSTREAM, FORK):
        ini = folder / "platformio.ini"
        assert ini.is_file(), f"{folder.name} has no PlatformIO project"
        text = ini.read_text(encoding="utf-8")
        assert "extra_configs = ../common.ini" in text
        assert "extends = common" in text
        # No local overrides: everything that matters lives in the shared file.
        # `pio pkg install --library X` writes the dependency in here AND
        # inlines the shared section with it, which forks the two builds
        # without saying so. That happened once; this is why it cannot again.
        for forbidden in ("platform =", "board_build", "lib_deps", "[common]"):
            assert forbidden not in text, (
                f"{folder.name}/platformio.ini overrides {forbidden!r} locally; "
                f"build settings belong in firmware/common.ini"
            )


def test_the_pinned_core_matches_what_the_sketch_needs() -> None:
    """app_httpd.cpp includes dl_lib_matrix3d.h, which exists only in the old
    camera driver (ESP32 core 1.0.x). Pinning is not a preference here."""
    source = (UPSTREAM / "app_httpd.cpp").read_text(encoding="utf-8", errors="surrogateescape")
    assert "dl_lib_matrix3d.h" in source
    common = (ROOT / "firmware/common.ini").read_text(encoding="utf-8")
    assert "platform = espressif32@3.5.0" in common


def test_arduinojson_is_pinned_to_the_api_the_sketch_uses() -> None:
    """WAVEGO.ino uses StaticJsonDocument, which is the v6 API."""
    source = (UPSTREAM / "WAVEGO.ino").read_text(encoding="utf-8", errors="surrogateescape")
    assert "StaticJsonDocument" in source
    common = (ROOT / "firmware/common.ini").read_text(encoding="utf-8")
    assert "ArduinoJson@^6" in common


# --- the camera delta ---------------------------------------------------------
#
# There is no compiler here either, so these say nothing about image quality.
# What they protect are the three properties that decide whether a bad camera
# setting is recoverable on a robot that has no screen: the sensor is never
# called through a null function pointer, the frame size can never exceed the
# buffer that was allocated for it, and a failed init falls back instead of
# leaving the robot blind.


def camera_block(name: str = "app_httpd.cpp") -> list[str]:
    """The lines of the fork's camera-quality block."""
    out, inside = [], False
    for line in lines(FORK / name):
        if "=== RoboDog: camera image quality" in line:
            inside = True
        if inside:
            out.append(line)
        if inside and "=== end RoboDog" in line:
            break
    assert out, "the camera block is gone from the fork"
    return out


def test_every_sensor_setter_is_null_guarded() -> None:
    """The OV2640 driver of this core generation leaves some setter pointers
    null -- sharpness and denoise are the known ones. Calling through one
    reboots the ESP32, and robodogCameraTune runs inside webServerInit, before
    Wi-Fi: the reboot loop would be silent and would look like a hardware
    fault. Every call goes through the ROBODOG_CAM_SET macro or carries its
    own `if`."""
    unguarded = []
    for line in camera_block():
        for call in re.finditer(r"(\w+)->(set_\w+)\s*\(", line):
            obj, setter = call.group(1), call.group(2)
            if "ROBODOG_CAM_SET(" in line:  # the macro tests the pointer itself
                continue
            if f"if ({obj}->{setter})" in line:
                continue
            unguarded.append(line.strip())
    assert not unguarded, f"sensor setters called without a null check: {unguarded}"


def test_the_macro_actually_checks_the_pointer() -> None:
    """...because every call above is allowed to lean on it."""
    source = "\n".join(camera_block())
    macro = source.split("#define ROBODOG_CAM_SET")[1].split("while (0)")[0]
    assert "(cam)->setter" in macro and "if ((cam) && (cam)->setter)" in macro


def test_the_frame_size_stays_inside_the_allocated_buffer() -> None:
    """esp_camera_init allocates the frame buffer once, for the size it is
    given; set_framesize afterwards only writes sensor registers and never
    grows it. Letting `cam_size` past that ceiling is the one setting that can
    end in no image at all -- which is exactly what F2 spent a session on."""
    source = "\n".join(camera_block())
    assert "ROBODOG_CAM_MAX_SIZE = config->frame_size" in source, (
        "the ceiling must be recorded from the config that was actually allocated"
    )
    resize = source.split('!strcmp(name, "size")')[1].split("else if")[0]
    assert "ROBODOG_CAM_MAX_SIZE" in resize, "cam_size does not clamp to the ceiling"
    assert "set_framesize" in resize


def test_a_camera_that_does_not_fit_falls_back_instead_of_going_blind() -> None:
    """A larger frame is a want; a working camera is a need. If the buffer does
    not fit, the vendor's own frame size has to still come up."""
    source = "\n".join(camera_block())
    start = source.split("robodogCameraStart")[-1]
    assert "esp_camera_deinit()" in start, "a retry without deinit leaks the first attempt"
    assert "FRAMESIZE_QVGA" in start and "esp_camera_init(config)" in start


def test_a_changed_setting_reaches_the_frame_before_it_is_read() -> None:
    """The sensor exposes the frame already in flight under the old settings.
    Grabbing a picture right after a change would measure the setting before
    it and read as "the parameter does nothing"."""
    source = (FORK / "app_httpd.cpp").read_text(encoding="utf-8", errors="surrogateescape")
    snap = source.split("extern void robodogSnap(int withImage){")[1]
    assert "robodogCameraSettle();" in snap.split("esp_camera_fb_get")[0], (
        "snap reads a frame before dropping the ones exposed under the old settings"
    )


def test_the_camera_is_tunable_from_both_transports() -> None:
    """Same reasoning as the watchdog: a parameter that only exists on one
    transport is a trap for whoever is on the other one. Both route into the
    same robodogCameraSet, so the name list cannot drift apart."""
    ino = (FORK / "WAVEGO.ino").read_text(encoding="utf-8", errors="surrogateescape")
    httpd = (FORK / "app_httpd.cpp").read_text(encoding="utf-8", errors="surrogateescape")
    assert 'strncmp(robodogVar(), "cam_", 4)' in ino
    assert 'strncmp(variable, "cam_", 4)' in httpd
    assert ino.count("robodogCameraSet(robodogVar() + 4, val)") == 1
    assert httpd.count("robodogCameraSet(variable + 4, val)") == 1


def test_the_baseline_keeps_the_vendors_camera_settings() -> None:
    """The way back to a stock robot has to include the stock picture, or
    "flash the baseline again" stops being a way to tell our changes apart
    from the hardware."""
    source = (UPSTREAM / "app_httpd.cpp").read_text(encoding="utf-8", errors="surrogateescape")
    assert "config.jpeg_quality = 63;" in source
    assert "config.frame_size = FRAMESIZE_QVGA;" in source
    assert "robodogCamera" not in source
