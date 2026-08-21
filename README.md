# RoboDog

Software platform for the [Waveshare WAVEGO](https://www.waveshare.com/wiki/WAVEGO)
12-DOF quadruped (Standard BASIC, no Raspberry Pi), built on four pillars:

1. **Motion/control stack** — gait, IK/FK and telemetry, ported faithfully from
   the open (MIT) Waveshare firmware.
2. **Teach-in programming** — record, name, replay and git-version poses and
   trajectories as human-editable YAML routines.
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

## On the real robot (Wi-Fi, stock firmware)

Join the robot's access point (SSID `WAVESHARE Robot`, password `1234567890`),
then:

```console
uv run robodog info --backend http        # connectivity + capability check
uv run robodog bringup                    # guided bring-up, writes a report
uv run robodog play routines/patrol-wifi.yaml --backend http
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

## License

MIT. Contains a pinned reference copy of the MIT-licensed Waveshare WAVEGO
firmware under [vendor/wavego-firmware/](vendor/wavego-firmware/).
