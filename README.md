# RoboDog

[![CI](https://github.com/Justin-Riekehof/RoboDog/actions/workflows/ci.yml/badge.svg)](https://github.com/Justin-Riekehof/RoboDog/actions/workflows/ci.yml)
[![Python 3.12+](https://img.shields.io/badge/python-3.12+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

Software platform for the [Waveshare WAVEGO](https://www.waveshare.com/wiki/WAVEGO)
12-DOF quadruped (Standard BASIC, no Raspberry Pi), built on four pillars:

1. **Motion/control stack** — gait, IK/FK and telemetry, ported faithfully from
   the open (MIT) Waveshare firmware.
2. **Teach-in programming** — record, name, replay and git-version poses,
   trajectories and timed move sequences as human-editable YAML routines.
3. **Gamepad teleoperation** — low-latency Xbox controller driving with a
   hard-wired software E-stop.
4. **Digital twin** — URDF/MJCF simulation; twin and real robot are driven
   through the *same* Robot API.

Safety is part of the architecture: every command passes a supervisor with
E-stop latch, connection watchdog and workspace limits. See
[ARCHITECTURE.md](ARCHITECTURE.md), [ROADMAP.md](ROADMAP.md) and
[ASSUMPTIONS.md](ASSUMPTIONS.md).

## Quickstart (no hardware required)

Requires Python ≥ 3.12 and [uv](https://docs.astral.sh/uv/).

```console
uv sync --all-groups --all-extras

# what can the current backend do?
uv run robodog info --backend mock

# play a command-timeline routine against the in-process mock robot
uv run robodog play routines/patrol-demo.yaml --backend mock

# play a keyframed leg-space motion routine
uv run robodog play routines/bow.yaml --backend mock

# play a drive sequence: named moves with durations, repeated
uv run robodog play routines/patrol-loop.yaml --backend mock

# validate a routine file without running it
uv run robodog validate routines/bow.yaml

# render the robot as a 3D stick figure (needs the `viz` extra)
uv run robodog viz --pose stand --out stand.png
```

## Digital twin (MuJoCo, no hardware)

```console
uv run robodog info --backend sim
uv run robodog play routines/patrol-wifi.yaml --backend sim --viewer
```

The twin's legs are a serial stand-in for the real five-bar linkage, but their
**foot positions match the ported firmware kinematics exactly** — so gait
geometry and stability transfer, while absolute servo loads do not yet
(ASSUMPTIONS E4). Its state is measured from physics, not estimated.

### Teach-in against the twin

```console
uv run robodog teach wave --backend sim --viewer
```

Opens a local web UI in the browser: **drag the feet** in side/top views,
mirror left/right, capture keyframes, preview the motion in physics, save — and
out comes an ordinary routine file under [routines/](routines/). Ghost markers
show where physics actually put each foot, and every pose passes the safety
supervisor while you teach it. A scriptable console is available via `--repl`.
Full guide: [docs/teach-in.md](docs/teach-in.md).

`--viewer` paces playback to wall time (the simulation is otherwise some thirty
times faster than real time) and keeps the window open when the routine ends,
until you close it. Without the viewer, an 8-second routine finishes in about a
quarter of a second — handy for tests, useless for watching.

### Programming move sequences

The same UI has a second tab that needs no pose capability at all: build a
sequence of **standard moves with durations** — 10 s forward, 2 s left, 5 s
backward — set the pause inserted between moves, repeat it a number of times or
loop it until stopped, and run it. A **drive pad** underneath sends the same
moves one at a time, for driving the robot by hand between runs. Stop is on the
page, on the Escape key, and on the page's own heartbeat (close the tab and
whatever moves stops). Saving produces a `kind: sequence` routine that
`robodog play` replays unchanged:

```console
uv run robodog teach patrol --backend sim          # author against the twin
uv run robodog teach patrol --backend http         # ... or on the real robot
uv run robodog play routines/patrol-loop.yaml --backend http
```

## On the real robot (Wi-Fi, stock firmware)

Join the robot's access point (SSID `WAVESHARE Robot`, password `1234567890`),
then:

```console
uv run robodog info --backend http        # connectivity + capability check
uv run robodog bringup                    # guided bring-up, writes a report
uv run robodog play routines/patrol-wifi.yaml --backend http

# author and run move sequences in the browser, on the robot
uv run robodog teach patrol --backend http

# with our own firmware flashed, poses reach the robot too; the backend
# detects which firmware is on it, and --firmware overrules the probe
uv run robodog info --backend http --firmware robodog

# and then a keyframed motion routine plays on the real robot
uv run robodog play routines/bow.yaml --backend http

# measure a leg's real roll range (rehearse with --backend mock first)
uv run robodog calibrate-roll --legs front_left

# measure where each servo's zero really is, into a versioned table
uv run robodog calibrate-servos --legs front_left
uv run robodog calibrate-servos --show
```

**Put the robot on a stand first.** The stock firmware has no link watchdog: if
Wi-Fi drops mid-command it keeps walking and no stop can reach it. Full
procedure and the reasoning: [docs/bringup.md](docs/bringup.md).

Development:

```console
uv run pytest
uv run ruff check .
uv run ruff format .
uv run mypy
```

## Status

**M0** (offline foundations), **M1** (Wi-Fi bring-up, run on the real robot on
2026-08-11) and the core of **M2** (MuJoCo digital twin) are done. Next: **M3**
sim-to-real calibration, then **M4** custom firmware. See
[ROADMAP.md](ROADMAP.md) and [ASSUMPTIONS.md](ASSUMPTIONS.md).

One open hardware issue: our robot walks skewed because its repaired hind-right
leg deviates from the firmware's link geometry (ASSUMPTIONS F1/F3).

## Documentation

| Document | What it answers |
|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | How it fits together: one Robot API, the backend/capability model, the safety design, repository layout |
| [ROADMAP.md](ROADMAP.md) | Milestones M0-M7, each with its acceptance criteria |
| [ASSUMPTIONS.md](ASSUMPTIONS.md) | Every unverified claim about the hardware and firmware, with its source and current status |
| [docs/bringup.md](docs/bringup.md) | Guided first contact with the real robot, and why each step is in that order |
| [docs/teach-in.md](docs/teach-in.md) | Authoring routines: web UI, scriptable console, the routine file format |
| [docs/calibration.md](docs/calibration.md) | Servo zero calibration: the reference, the procedure, what the numbers do and do not say |
| [docs/firmware.md](docs/firmware.md) | Flashing the ESP32: the pristine baseline, then the fork with the link watchdog |
| [docs/research/](docs/research/) | Phase-0 source research on the WAVEGO and WAVEGO Pro, with citations |
| [CONTRIBUTING.md](CONTRIBUTING.md) | Development setup and the project's non-negotiable rules |

Why so much prose about assumptions: the robot has **no servo position
feedback** and the stock firmware returns **no telemetry at all**. Everything
this software believes about the machine is either read out of the vendor
firmware source or verified by watching the robot move. ASSUMPTIONS.md is the
ledger of which is which, and entries are updated rather than deleted — a guess
that turned out wrong is worth keeping.

## Contributing

Issues and pull requests are welcome, particularly from anyone running the same
hardware — much of ASSUMPTIONS.md currently rests on a sample size of one robot.
See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

[MIT](LICENSE). Contains a pinned reference copy of the MIT-licensed Waveshare
WAVEGO firmware under [vendor/wavego-firmware/](vendor/wavego-firmware/),
Copyright (c) 2022 waveshare — read-only, never modified.
