# RoboDog Architecture

Software platform for the Waveshare WAVEGO (Standard, BASIC — no Raspberry Pi),
built around four pillars: motion/control stack, teach-in programming, gamepad
teleoperation, and a digital twin. High-level code runs on a Windows PC and
talks to the robot's ESP32 over Wi-Fi (the stock firmware's HTTP endpoint, in use
today) and later over USB serial.

Every claim about the vendor firmware in this document is traceable to
[ASSUMPTIONS.md](ASSUMPTIONS.md) and the pinned reference copy in
[vendor/wavego-firmware/](vendor/wavego-firmware/).

## Core principle: one Robot API, two backends

All applications (CLI, teleop, teach-in recorder/player, future GUIs) talk to a
single `RobotClient`. The client wraps a **backend** — an implementation of the
`Backend` protocol — and every command passes through the **safety supervisor**
before it reaches the backend. Backends are swappable at construction time:

```
 apps:      CLI · teach player/recorder · gamepad teleop (M5) · viewer
                              │
                        RobotClient  ──────────  robodog.kinematics
                              │                   (pure functions: IK/FK,
                      SafetySupervisor             gait, poses, limits —
             (E-stop · watchdog · limits · rate)   shared by ALL backends,
                              │                    teach-in and viz)
              ┌───────────────┼────────────────┐
        MockBackend      SimBackend       HttpBackend
        (M0, in-proc     (M2, MuJoCo      (M1, /control over
         kinematic        physics +        Wi-Fi to the stock
         state, CI-safe)  viewer)          firmware)
```

- The **mock backend** is a deterministic, dependency-free kinematic model.
  It is the CI reference target and must always work without hardware.
- The **sim backend** (MuJoCo) is the digital twin. It does not reimplement any
  command semantics: it *composes* the mock backend as the command interpreter
  and adds physics, so the ported firmware logic exists exactly once. Its MJCF
  model is generated from `kinematics/constants.py`, and unlike every other
  backend it reports **measured** state — the twin is allowed to disagree with
  the command, which is what makes it useful.
- The **http backend** talks to the stock firmware over its own Wi-Fi access
  point (`GET /control?var=..&val=..&cmd=..`). It is deliberately limited to
  what that handler accepts, and because the firmware returns no data at all,
  the state it reports is a *model* flagged `is_estimated=True`. A later
  serial backend can add the commands that only the UART path exposes.

### Capability model

The stock firmware accepts only coarse commands and provides **no servo position
feedback** (PWM servos on a PCA9685, write-only). It also differs *per transport*:
the HTTP handler and the serial JSON handler are separate code paths with
different command sets (ASSUMPTIONS D2). Rather than designing the API down to
the lowest common denominator, each backend declares capabilities and
apps/routines degrade gracefully:

| Capability     | Mock | Sim | HTTP (stock fw) | Serial (stock fw) | Custom fw (M4) |
|----------------|------|-----|-----------------|-------------------|----------------|
| `LOCOMOTION`   | ✅   | ✅  | ✅              | ✅                | ✅             |
| `GESTURE`      | ✅   | ✅  | ❌              | ✅                | ✅             |
| `PERIPHERALS`  | ✅   | ✅  | ❌              | ✅                | ✅             |
| `SERVO_TRIM`   | —    | —   | ✅              | ❌                | ✅             |
| `BODY_POSE`    | ✅   | ✅  | ❌              | ❌                | ✅             |
| `LEG_TARGET`   | ✅   | ✅  | ❌              | ❌                | ✅             |
| `JOINT_ANGLES` | ✅   | ✅  | ❌              | ❌                | ✅             |
| `TELEMETRY`    | ✅   | ✅  | ❌              | ❌ (fw sends nothing) | ✅ (voltage, IMU) |

A routine or app states what it `requires`; playing a `LEG_TARGET` routine
against a stock-firmware backend fails fast with a clear error instead of
silently doing something else. This is why `routines/patrol-demo.yaml`
(gestures + LEDs) and `routines/patrol-wifi.yaml` (locomotion only) both exist.

### Command and state flow

- **Commands** are small typed objects (`Drive`, `SetFunction`, `Gesture`,
  `SetBodyPose`, `SetLegTarget`, `SetJointAngles`, `Led`, `Buzzer`). The client
  offers ergonomic methods; each call is validated by the supervisor, then
  translated by the backend (mock/sim: into kinematics calls; hardware: into
  the firmware's own protocol).
- **State** flows back as immutable `RobotState` snapshots: commanded leg
  targets, derived joint angles (via our IK — for the stock hardware this is
  *commanded*, never *measured*; the flag `state.is_estimated` makes that
  explicit), safety state, and telemetry when available.
- Mock and sim advance via an explicit `tick(dt)` — deterministic, fast-forwardable,
  test-friendly. Hardware backends run on wall-clock time.

## Safety (part of the architecture, not an afterthought)

`SafetySupervisor` sits between client and backend. No command bypasses it.

- **States:** `DISARMED → ARMED → ESTOPPED`. Commands are only forwarded when
  `ARMED`. Leaving `ESTOPPED` requires an explicit `reset()` — no auto-re-arm.
- **Software E-stop:** `estop()` immediately sends the backend's *safe
  sequence* and latches `ESTOPPED`. Wired to a CLI command, Ctrl-C handlers,
  and later a gamepad button (M5).
- **Watchdog:** the supervisor must be fed (`feed()`, done implicitly by every
  accepted command and explicitly by app main loops). If the deadline
  (default 500 ms) passes, the safe sequence fires. The clock is injectable so
  tests can simulate timeouts deterministically.
- **Limits:** leg targets and body poses are rejected against the workspace
  limits ported from the firmware (height 75–110 mm, lateral ±30 mm, gesture
  ±15) plus a configurable command-rate limit. Joint-angle commands are checked
  against per-servo angle bounds and linkage assembly.
- **Reachability and self-consistency:** the firmware's box clamps guarantee
  neither. Two failure modes we found in the vendor IK are caught here, on the
  safety path, using our exact FK as the oracle: targets that pass the box but
  lie outside the curved reachable set (the firmware would compute NaN there,
  ASSUMPTIONS C10), and targets where the firmware's IK commands the two coaxial
  servos of a leg to inconsistent knee positions so the linkage fights itself
  (ASSUMPTIONS C11). This is why the exact FK exists: it polices the approximate
  IK before anything reaches a robot.
- **Safe pose:** a defined crouch (stand at minimum height) — low center of
  mass, minimal drop height. Mock/sim hold it. **Known limitation of the stock
  firmware:** the hardware safe sequence can only stop the gait
  (`FBStop`/`LRStop` → standing still); a held crouch and an *on-device* link
  watchdog require the custom firmware milestone (M4). Over Wi-Fi it is worse
  than over USB: if the link drops, no stop command reaches the robot at all
  (ASSUMPTIONS D10), so bring-up runs with the robot on a stand. This residual
  risk is documented, surfaced in the tooling, and closed by M4.

## Kinematics

`robodog.kinematics` is a pure-function port of the firmware's leg model
(five-bar linkage driven by two coaxial servos + hip "wiggle" servo per leg):

- `leg_ik(x, y, z)` — the firmware's three-stage IK (`wigglePlaneIK` →
  `singleLegPlaneIK` → `simpleLinkageIK`), producing the three servo angles.
- `leg_fk(angles)` — closed-form forward kinematics derived from the same
  linkage geometry (crank intersection). Not present in the firmware; verified
  by `fk(ik(p)) ≈ p` round-trip property tests. FK also yields all intermediate
  joint positions, which powers the stick-figure visualization and, later, the
  URDF/twin.
- `servo_pwm(angles)` — angle → PWM count mapping incl. per-channel direction
  and calibration offsets (`≈2.22 counts/°`, middle 300).
- `gait.py` — port of `simpleGait`/`triangularGait` foot-trajectory generators.
- Conventions (from the firmware): lengths in **mm**, angles in **degrees** at
  the API boundary; per-leg frame: x forward, y down (positive toward ground),
  z outward; legs numbered 1=front-left, 2=hind-left, 3=front-right,
  4=hind-right.

The port is line-faithful to the vendored reference (MIT-licensed, attributed
in the module docstring) — including its quirks — so that sim and hardware stay
in agreement. Deviations are only allowed behind clearly named wrappers and get
recorded in ASSUMPTIONS.md.

## Teach-in data format

Routines are single YAML files in [routines/](routines/) — one file per
routine, git-diffable, hand-editable, schema-versioned via a `schema` field.
Two kinds share one envelope:

```yaml
schema: robodog.routine/v1
name: patrol-demo
description: Walk forward, look around, sit briefly.
kind: commands            # timeline of Robot API commands
requires: [LOCOMOTION]
steps:
  - at: 0.0               # seconds from start
    do: drive
    args: {forward: 1}
  - at: 2.0
    do: drive
    args: {forward: 0}
  - at: 2.5
    do: gesture
    args: {axis: yaw, direction: 1}
```

```yaml
schema: robodog.routine/v1
name: bow
kind: motion              # keyframed leg-space trajectory
requires: [LEG_TARGET]
interpolation: cosine     # firmware-style besselCtrl easing (or: linear)
keyframes:
  - at: 0.0
    legs:                 # mm, per-leg frame; omitted legs hold position
      front_left:  {x: 16, y: 95, z: 25}
      front_right: {x: 16, y: 95, z: 25}
  - at: 1.0
    legs:
      front_left:  {x: 16, y: 78, z: 25}
      front_right: {x: 16, y: 78, z: 25}
```

Design intent: `commands` routines run on **every** backend today (recording a
gamepad session produces this kind); `motion` routines are the pose-level
teach-in — authorable and playable on mock/sim now, on hardware once M4 adds
joint-level commands. The player validates `requires` against backend
capabilities and the supervisor clamps every frame, so a hand-edited file
cannot drive the robot outside its workspace.

## Repository layout

```
src/robodog/          Python package (src layout)
  api/                RobotClient, commands, state, capabilities
  backends/           base protocol + mock/ (M0), http/ (M1), sim/ (M2)
  kinematics/         linkage constants, IK/FK, servo map, gait, easing
  safety/             SafetySupervisor, limits, watchdog
  teach/              routine schema, loader/validator, player, teach-in
                      session + web UI + scriptable console
  viz/                stick-figure rendering from FK (optional `viz` extra)
  bringup.py          guided hardware bring-up procedure (M1)
  cli.py              `robodog` entry point
tests/                pytest suite (mirrors package layout)
routines/             teach-in files (YAML, versioned in git)
sim/                  exported models + meshes (the MJCF itself is generated)
cad/                  CAD sources/exports (existing leg STL; CadQuery later)
vendor/wavego-firmware/  pinned upstream firmware reference (MIT, read-only)
docs/                 research notes, bring-up procedure + reports, teach-in guide
```

## Firmware strategy

The Waveshare firmware is treated as a **reference implementation, not a
dependency**: we port the math, pin the exact source we ported from, and record
every protocol behavior we rely on as a verifiable assumption. The planned M4
fork (MIT license permits it) adds what the stock firmware lacks — on-device
watchdog, active telemetry, joint-level commands — while the stock firmware
remains a supported fallback: the hardware backends keep working against both,
distinguished by capability flags.
