# wavego-upstream — the pristine baseline

The complete, **unmodified** Waveshare WAVEGO sketch. This is the firmware the
robot ships with, and the one to flash *first* — before the fork next door, and
before blaming the fork for anything.

- Upstream: <https://github.com/waveshare/WAVEGO>, path `Arduino/WAVEGO/`
- Commit: see [UPSTREAM_COMMIT.txt](UPSTREAM_COMMIT.txt) — `419868d`, 2022-05-11
- License: MIT, Copyright (c) 2022 waveshare

## Why it is in the repo at all

Three jobs that a link to GitHub cannot do:

1. **A baseline you can flash.** If a flash goes wrong, or the fork misbehaves,
   this is the way back to a robot that behaves like the manual says.
2. **The reference the fork is diffed against.** `tests/test_firmware_fork.py`
   proves the fork only *adds* to these files, so a reviewer reads the additions
   instead of the whole firmware.
3. **A witness for the Python port.** The four files that also exist in
   [vendor/](../../vendor/wavego-firmware/) are byte-identical to them, which is
   what lets `robodog.kinematics` claim to be a line-faithful port of
   `ServoCtrl.h`. A test pins that too.

Do not edit anything here. Changes belong in
[../wavego-robodog/](../wavego-robodog/).

## Difference to the fork

```
WAVEGO.ino      +48 / -0     link watchdog, two commands, one feed call
app_httpd.cpp   +15 / -0     the same two commands on the HTTP path
everything else   0 / 0
```
