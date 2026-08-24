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
            behaviours (M8: vision in, drive intents out)
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
         state, no I/O)   viewer)          firmware)
```

- The **mock backend** is a deterministic, dependency-free kinematic model.
  It is the reference target for the test suite and must always work without
  hardware.
- The **sim backend** (MuJoCo) is the digital twin. It does not reimplement any
  command semantics: it *composes* the mock backend as the command interpreter
  and adds physics, so the ported firmware logic exists exactly once. Its MJCF
  model is generated from `kinematics/constants.py` — **including its joint
  ranges**, which had to be learned the hard way: the roll hinge carried a
  hard-coded ±60° while the measured envelope was −27…+135° (C13), so the twin
  silently clamped a third of the range the supervisor accepted. Both sides now
  read `ROLL_MIN_DEG`/`ROLL_MAX_DEG`, and a test sweeps the permitted workspace
  to prove every pose the safety layer allows is one the model can hold.
  Unlike every other
  backend it reports **measured** state — the twin is allowed to disagree with
  the command, which is what makes it useful.
- The **http backend** talks to the stock firmware over its own Wi-Fi access
  point (`GET /control?var=..&val=..&cmd=..`). It is deliberately limited to
  what that handler accepts, and because the firmware returns no data at all,
  the state it reports is a *model* flagged `is_estimated=True`. A later
  serial backend can add the commands that only the UART path exposes.

### Autonomy sits above the client, never beside it (M8)

A vision-guided behaviour is an *application* in the diagram above, not a
backend and not a shortcut past one. It reads detections, decides a drive, and
sends it through the same `RobotClient` — so the supervisor's E-stop, watchdog,
limits and capability gate apply to a robot walking at a person exactly as they
apply to a keyframe played from a file. Three splits keep it that way:

- `robodog.vision` — frames and detections. `Detection` is normalised (a box in
  frame *fractions*), so nothing downstream knows a resolution and the camera's
  frame size can change mid-session. The YOLO implementation lives behind the
  optional `vision` extra; a `Detector` protocol and a scripted stand-in are
  what everything else depends on. **One process reads the robot's stream** and
  re-serves it, because with `psram=0` the firmware has a single frame buffer
  and a second viewer blinds the first (ASSUMPTIONS F4/G3).
- `robodog.behaviour` — pure logic: detections and a clock in, drive intents
  out, as a state machine (searching / approaching / arrived / lost). No
  camera, no model, no robot, no sleeping, exactly like the kinematics. Its
  runner is a separate, thin file that does the I/O.
- `robodog.ai` — free text to *one named behaviour with parameters*, asked
  once, before anything moves. **The model does not drive.** It cannot: it
  emits a call from a fixed vocabulary, which is validated before a loop
  starts, and that loop is ordinary deterministic code. An 8B-plus model inside
  a 10 Hz control loop would be latency and non-determinism in the one place
  neither belongs — and a model that can steer is a model that can steer around
  the supervisor.

### Capability model

The stock firmware accepts only coarse commands and provides **no servo position
feedback** (PWM servos on a PCA9685, write-only). It also differs *per transport*:
the HTTP handler and the serial JSON handler are separate code paths with
different command sets (ASSUMPTIONS D2). Rather than designing the API down to
the lowest common denominator, each backend declares capabilities and
apps/routines degrade gracefully:

Capabilities are not a fixed property of a transport: `HttpBackend` **asks the
robot** at connect which firmware it runs (a `var=ping` probe -- the stock
firmware answers 500 to a variable it does not know, ours answers 200) and adds
`LEG_TARGET` only when the answer says so. `--firmware stock|robodog` overrules
the probe, because guessing wrong about what a robot can be told to do should be
correctable by hand. The probe errs towards *less*: a capability claimed but not
delivered turns into a command swallowed by a 500 below the safety layer,
instead of one refused above it.

| Capability     | Mock | Sim | HTTP (stock fw) | Serial (stock fw) | Custom fw (M4) |
|----------------|------|-----|-----------------|-------------------|----------------|
| `LOCOMOTION`   | ✅   | ✅  | ✅              | ✅                | ✅             |
| `GESTURE`      | ✅   | ✅  | ❌              | ✅                | ✅             |
| `PERIPHERALS`  | ✅   | ✅  | ❌              | ✅                | ✅             |
| `SERVO_TRIM`   | ✅   | —   | ✅              | ❌                | ✅             |
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
Three kinds share one envelope:

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
name: patrol-loop
kind: sequence            # named drive moves, each with its own duration
requires: [LOCOMOTION]
gap: 0.5                  # seconds of standing still between two moves
repeat: 3                 # passes through the list; 0 = until stopped
moves:
  - {move: forward, seconds: 10}
  - {move: left, seconds: 2}
  - {move: backward, seconds: 5}
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
joint-level commands. `sequence` is the operator-facing shorthand for the first
kind: a list of named moves with durations, expanded into a drive timeline at
load time, so the player and every backend see an ordinary command timeline.
It is the one teach-in that already runs on the real robot, because locomotion
is all the stock firmware offers over Wi-Fi (ASSUMPTIONS D2).

`repeat` belongs to the envelope rather than to one kind, and `repeat: 0` means
*until stopped*. Playback therefore has an outside stop signal
(`play_routine(..., should_stop=...)`), which the web UI drives from its Stop
button, the Escape key and a dead-man's switch on the page's own polling — an
endless routine with no way out would be a safety defect, not a feature
(ASSUMPTIONS D10).

The player validates `requires` against backend capabilities and the supervisor
clamps every frame, so a hand-edited file cannot drive the robot outside its
workspace.

## Calibration data

The kinematic model says PWM 300 is every joint's zero; the robot disagrees by
a constant per servo. That constant is measured once and versioned in
[calibration/](calibration/) as its own tiny schema:

```yaml
schema: robodog.calibration/v1
measured: 2026-08-21
reference: upper arms vertical, leg plane vertical
offsets:
  front_left: {fore: 6, back: -4, wiggle: 2}   # PWM counts from the firmware middle
```

Deliberate properties: offsets are **relative to the firmware's stored middle**
(the only thing the measurement can see, ASSUMPTIONS D8); **partial tables are
valid**, so a leg can be measured per sitting; and the table is *data*, not a
default baked into the mapping — `kinematics.servo.channel_pwm` takes it as an
argument, so the twin keeps running nominal while the hardware path applies the
measured zeros. Procedure and its limits: [docs/calibration.md](docs/calibration.md).

## Repository layout

```
src/robodog/          Python package (src layout)
  api/                RobotClient, commands, state, capabilities
  backends/           base protocol + mock/ (M0), http/ (M1), sim/ (M2)
  kinematics/         linkage constants, IK/FK, servo map, gait, easing
  safety/             SafetySupervisor, limits, watchdog
  vision/             frames and detections (M8): MJPEG parser, frame hub,
                      Detector protocol + scripted stand-in, YOLO behind the
                      optional `vision` extra
  behaviour/          vision-guided behaviours (M8): the pure state machine,
                      the vocabulary the model may speak, the runner
  ai.py               free text -> one behaviour call, against an
                      OpenAI-compatible endpoint (M8)
  teach/              routine schema, loader/validator, player, teach-in
                      session (poses) + sequence session (drive moves)
                      + web UI + scriptable console
  calibration.py      servo zero table: schema, loader, PWM mapping input
  calibrate.py        guided hardware procedures that produce that table
  viz/                stick-figure rendering from FK (optional `viz` extra)
  bringup.py          guided hardware bring-up procedure (M1)
  cli.py              `robodog` entry point
tests/                pytest suite (mirrors package layout)
firmware/             our fork of the vendor firmware (M4) -- editable,
                      adds only; `vendor/` stays the pristine reference
routines/             teach-in files (YAML, versioned in git)
calibration/          measured servo zeros (YAML, versioned in git)
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
