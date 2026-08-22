# Roadmap

Each milestone is independently testable and ends in a demonstrable state.
M0 runs entirely without hardware.

## M0 — Offline foundations (mock backend + kinematic sim) ✅ done

Everything that can be built and proven without a robot:

- Repo skeleton: uv/pyproject, ruff, mypy, pytest, GitHub Actions CI, docs.
- `robodog.kinematics`: faithful port of the firmware leg IK, closed-form FK,
  servo/PWM mapping, gait generators, easing — all pure functions.
- `robodog.api` + `Backend` protocol + capability model.
- `SafetySupervisor`: arm/disarm, E-stop latch, injectable-clock watchdog,
  workspace/joint limit enforcement.
- `MockBackend`: deterministic kinematic robot state, `tick(dt)`.
- Teach-in routine format v1 (YAML): loader, validator, player; two example
  routines under `routines/`.
- CLI: `robodog info`, `robodog validate <file>`,
  `robodog play <file> --backend mock`, `robodog viz` (matplotlib stick figure
  from FK, optional extra).

**Acceptance (no hardware attached):**
1. `uv sync --all-groups && uv run pytest` — all green.
2. `uv run ruff check .`, `uv run ruff format --check .`, `uv run mypy` — clean.
3. `uv run robodog play routines/patrol-demo.yaml --backend mock` plays a
   command routine and prints a state timeline.
4. `uv run robodog play routines/bow.yaml --backend mock` plays a motion
   routine (leg-space keyframes through IK, limits enforced).
5. A watchdog test demonstrates: missed heartbeat → safe sequence → ESTOPPED →
   commands rejected until reset.
6. `uv run robodog viz --pose stand --out stand.png` renders the stick-figure
   robot (visual check of FK/IK agreement).
7. Regression tests pin the two vendor-IK defects found while building M0
   (ASSUMPTIONS C10/C11) and prove the shipped gait never triggers them.

## M1 — Wi-Fi bring-up on the stock firmware ✅ done (2026-08-11)

Get onto the real robot as early as possible, without flashing anything. The
stock firmware's HTTP endpoint is the only channel that needs no hardware
changes — so this milestone is about *learning the real machine* and turning
firmware-read assumptions into verified ones.

- `HttpBackend`: `GET /control?var=..&val=..&cmd=..` over the robot's own access
  point, with the capability set the firmware actually offers over Wi-Fi
  (locomotion + servo trim — **no gestures, LEDs or buzzer**, ASSUMPTIONS D2).
- Capability model refined to match reality: `LOCOMOTION`, `GESTURE`,
  `PERIPHERALS`, `SERVO_TRIM`.
- `robodog bringup`: guided checklist, one observable action per assumption,
  writes a dated report to `docs/bringup/`.
- `routines/patrol-wifi.yaml`: a locomotion-only routine that runs over Wi-Fi.
- Hardware playback safety: confirmation prompt, wall-clock pacing, E-stop on
  Ctrl-C, stop-on-connect and stop-on-disconnect.

**Acceptance:**
1. `uv run robodog info --backend http` reports the robot's capabilities
   (needs the PC joined to the robot's access point).
2. `uv run robodog bringup` completes and produces a report.
3. Every step in that report is transferred into ASSUMPTIONS.md as `verified`
   or `wrong` — that transfer *is* the deliverable of this milestone.
4. `uv run robodog play routines/patrol-wifi.yaml --backend http` walks the
   robot on a stand and stops it cleanly.

**Known residual risk, by design:** over Wi-Fi a dropped link cannot be
recovered — no stop command reaches the robot (ASSUMPTIONS D10). Bring-up runs
with the robot on a stand or a hand on the power switch. Closing this needs an
on-device watchdog, i.e. M4.

## M2 — Digital twin (MuJoCo) ✅ core done

- MJCF model **generated from** `kinematics/constants.py`, so model and code
  cannot drift apart — joint ranges included (they were not, until the roll
  hinge's hard-coded ±60° was found clamping the measured −27…+135° envelope). Legs are a serial stand-in for the five-bar linkage whose
  foot positions match the ported kinematics exactly (ASSUMPTIONS E4).
- `SimBackend` with the full capability set. It composes `MockBackend` as the
  command interpreter — the ported firmware logic exists once — and adds physics
  on top, reporting **measured** state (`is_estimated=False`).
- The same routines from `routines/` play on mock, sim and (where capabilities
  allow) the real robot.
- MuJoCo viewer via `robodog play --backend sim --viewer`.

**Measured on 2026-08-11:** the robot stands (trunk at 100 mm, level), the ported
gait walks it 27 cm in 3 s, and it stays upright; simulated feet track commanded
targets to 0.15 mm under load. 40 headless tests, ~2 s.

**Still open:** masses and inertias are estimates (E1/E5), so absolute servo
loads are not meaningful yet; no camera or IMU noise model.

**Acceptance:** `robodog play routines/patrol-wifi.yaml --backend sim --viewer`
shows the robot walking in the MuJoCo viewer; CI runs the headless sim tests.

This is where virtual teach-in becomes practical: poses and trajectories are
authored against the twin, validated by the safety layer, and stored as routine
files — then deployed to hardware via M3 (calibration) and M4 (firmware).

## M3 — Sim-to-real calibration (no new hardware)

The bridge that makes "author in the twin, deploy to the robot" trustworthy
without any servo feedback. Uses the pre-M4 escape hatch: absolute per-servo
positioning through the firmware's `sconfig` trim facility (ASSUMPTIONS D8).

- Drive individual servos to computed absolute PWM counts over Wi-Fi
  (`funcMode=9` baseline, then one trim request per servo — mind the D6 jump).
- Measure a handful of known poses on the real robot, compute the systematic
  per-servo offset, store it as a calibration table in the repo.
  ✅ **tooling done:** `robodog calibrate-servos` walks a leg joint by joint
  against a reference the kinematics defines (both upper arms vertical, leg
  plane vertical), writes `calibration/servos.yaml` plus a dated report, and
  rehearses against the mock backend. Partial tables are valid, so the robot
  can be measured one leg per sitting.
  ⏳ **open:** the measurement itself — one joint of one leg has been measured
  so far (front-left wiggle, roll range, 2026-08-21).
- Apply that table in the servo mapping so twin poses land correctly on hardware.
  `kinematics.servo.channel_pwm` already takes the table; the consumer that
  pushes a twin pose to the robot (D8: baseline plus one `sconfig` per servo,
  static poses only) is the next piece.

**Acceptance:** a static pose authored in the twin is reproduced on the real
robot within a documented tolerance; the calibration table is version-controlled
and its measurement procedure written down.

## M4 — Custom firmware (fork of the MIT-licensed vendor firmware)

The real unlock, and the reason feedback hardware is not needed: pose-level
commands plus an on-device safety net.

- **On-device link watchdog** — the robot stops and crouches by itself when the
  link dies. Closes the M1 residual risk (ASSUMPTIONS B9/D10).
  ✅ **written** in [firmware/wavego-robodog/](firmware/wavego-robodog/): two
  commands (`watchdog`, `ping`) on both transports, off by default so the stock
  web UI is unaffected, +48/-0 and +15/-0 lines on the upstream sketch with
  `ServoCtrl.h` untouched.
  ✅ **done and measured on the robot** (2026-08-22): armed at 2 s, fed while
  walking without tripping, then stopped and crouched by itself at the deadline.
  +384 B flash, +16 B RAM over the baseline; off by default, so the stock web UI
  is unaffected.
  ✅ **host side done 2026-08-22**: `HttpBackend` arms it at 1500 ms on connect
  and disarms it on disconnect -- leaving it armed would stop the robot for
  whoever drives it next from the vendor's own page, which sends nothing while
  it walks. Every accepted command feeds it, so ordinary traffic keeps it alive
  and no keep-alive thread is needed; and because it only acts on a robot that
  is *moving*, a long pause during teach-in costs nothing. Exercised on the
  robot in the teach session below -- the arming request was accepted, which is
  weaker evidence than the on-device measurement above and is all it is.
- Pose-level commands (leg targets, joint angles) → `LEG_TARGET` and
  `JOINT_ANGLES` on real hardware, so `motion` routines and pose teach-in run on
  the robot.
  ✅ **`leg` + `apply` over serial, on the robot 2026-08-22.** Four foot targets
  are staged into a shadow of `GoalPWM[]` and applied in one copy, so the twelve
  servos move together. Uses the firmware's own `singleLegCtrl`, so twin and
  robot compute the same pose. Cost the discovery that **no task but `loop()`
  may touch I2C** — see the fork's README.
  Verified by driving five poses in sequence (105 → 80 → 105 → 80 → 95 mm of
  reach) and having the operator confirm the robot reached each one, in order,
  with nothing moving during staging. The simultaneity itself rests on the
  shadow-copy design and on no ripple being reported; it was not instrumented.
  ✅ **and over HTTP, measured 2026-08-22**: `var=pose` carries all twelve
  values in one request and lands in **78-94 ms** on the robot's own access
  point — about twelve poses a second, where the stock trim path managed
  static poses only. An incomplete pose is refused with 500 and moves nothing.
  ✅ **and the host side, end to end on the robot 2026-08-22**: `HttpBackend`
  probes the firmware at connect (`var=ping`; stock answers 500, ours 200) and
  reports `LEG_TARGET` only when it is really there; `--firmware` overrules the
  probe and fails loudly if the assertion is wrong. `SetLegTarget` stages and
  flushes one pose per tick — **94-140 ms each** — and `routines/bow.yaml`
  played on the real robot at 10.7 poses/s. A workspace violation was still
  refused above the backend and never reached the robot.
  ✅ **and pose teach-in in the browser, on the robot 2026-08-22**: the operator
  posed the real robot by dragging feet in the teach UI. Two defects surfaced on
  the way and are fixed: the preview and the teach ticker both ran a fixed
  50 Hz, which over a link carrying ten poses a second plays a routine several
  times too long (both now ask the backend, as `robodog play` already did); and
  the drive pad -- with the only STOP button -- was locked inside the Sequence
  tab, so posing happened with no stop on screen. It is now a console beside
  both tabs, with a **Home** button that stops and re-centres.
- Active telemetry: battery voltage and the **full** IMU (the stock firmware
  reads only 2 of the ICM20948's 9 axes) → `TELEMETRY`.
- Current-based stall detection from the INA219 as the open-loop safety net.
- Gestures/LED/buzzer over Wi-Fi too, closing the D2 gap.
- Stock firmware stays a supported fallback, distinguished by capabilities.

**Acceptance:** cut the link mid-gait → the robot enters the safe state on its
own; `robodog play routines/bow.yaml --backend http` works; twin and robot run
the same motion routine side by side.

**Status 2026-08-22: the first two are met.** The link watchdog was measured on
the device, and `bow.yaml` played on the robot through the full stack. What is
left of M4 is the rest of its list: telemetry, stall detection from the INA219,
and gestures/LED/buzzer over Wi-Fi. Two numbers worth carrying forward: a pose
costs 94-140 ms over the robot's own access point, so the player ticks at 0.1 s
there instead of 0.02 — a routine keeps its authored length, coarser rather
than four times too long. And **do not leave a serial console attached** while
timing anything: with one open the same request took ~1050 ms.

## M5 — Gamepad teleoperation (Xbox One controller)

- XInput reader (Windows), mapping profile (sticks → drive/turn, buttons →
  modes/gestures, dedicated E-stop button), deadzones, command rate limiting.
- Latency instrumentation: input-to-command budget measured and logged.
- Works against **all** backends (drive the twin with the gamepad).

**Acceptance:** drive mock/sim/robot with the controller; E-stop button verified
on hardware; measured input→command latency documented.

## M6 — Teach-in v2 (record · name · replay · version)

Motion authoring arrived early (2026-08-19): **`robodog teach`** serves a
local web UI — drag feet in side/top views, mirror left/right, height slider,
capture/preview/save against live MuJoCo physics, ghost markers for the
measured foot positions. Every pose passes the safety supervisor, so invalid
poses are rejected while teaching, and saved files are re-validated through the
player's own parser. A scriptable console remains via `--repl`. See
docs/teach-in.md.

Still open for this milestone:
- Recorder: capture a teleop session as a `commands` routine (timestamped,
  normalized, deduplicated).
- Routine management CLI: list, describe, dry-run.

**Acceptance:** record a gamepad session → YAML file → replay on sim and
hardware; git diff of an edited routine is human-readable.

## M7 — Outlook (not scheduled)

- USB-serial backend as a second transport (the serial JSON path accepts
  gestures, LEDs and buzzer, which Wi-Fi does not).
- URDF/mesh pipeline from the parametric CadQuery leg model (`wavego_leg.py`,
  currently custom/out-of-repo) → higher-fidelity twin.
- IMU-based closed-loop behaviors; OpenCV via the ESP32 camera stream.
- Optional servo feedback if it is ever wanted: potentiometer taps or a swap to
  SC09 bus servos — deliberately **not** on the critical path.
