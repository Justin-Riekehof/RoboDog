# Vendored reference: Waveshare WAVEGO firmware (excerpt)

Read-only reference copy of the files our kinematics port and serial protocol
implementation are based on. **Never edit these files.**

- Upstream: <https://github.com/waveshare/WAVEGO>, path `Arduino/WAVEGO/`
- License: MIT (see [LICENSE](LICENSE), Copyright (c) 2022 waveshare)
- Fetched: 2026-08-10 (upstream unchanged since 2022)
- Files: `ServoCtrl.h` (IK, gaits, poses), `WAVEGO.ino` (serial JSON protocol,
  boot sequence, task loop), `InitConfig.h` (pins, peripherals, servo middle
  position), `app_httpd.cpp` (Wi-Fi setup, HTTP `/control` handler, MJPEG stream)

Note that the serial JSON handler (`WAVEGO.ino`) and the HTTP handler
(`app_httpd.cpp`) are **separate code paths with different command sets** — see
ASSUMPTIONS section D.

The Python port lives in `src/robodog/kinematics/` and must stay line-faithful
to `ServoCtrl.h` (see CLAUDE.md and ASSUMPTIONS.md section C).
