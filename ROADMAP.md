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
  ✅ **and the host budget, confirmed on the robot 2026-08-22**: a teach
  session over Wi-Fi opened and stayed up. It had been E-stopping itself on
  open with `watchdog timeout (0.500s)` -- the supervisor's budget was read
  from the backend *before* connect, and before connect a transport that probes
  for its firmware can only answer with the pessimistic stock number. Sized at
  connect now.
  ✅ **and the walk keep-alive, on the robot 2026-08-22**: a direction held for
  several seconds keeps walking. It used to stop and crouch after ~1.5 s,
  because the one state the on-device watchdog guards -- a latched move -- is
  the one state that produces no traffic to feed it.
  ✅ **host side done 2026-08-22**: `HttpBackend` arms it at 1500 ms on connect
  and disarms it on disconnect -- leaving it armed would stop the robot for
  whoever drives it next from the vendor's own page, which sends nothing while
  it walks. Because it only acts on a robot that is *moving*, a long pause
  during teach-in costs nothing.
  ⚠️ **and it needed a keep-alive, which the first version did not have.**
  "Any accepted command feeds it, so ordinary traffic keeps it alive" was
  written here and is false where it matters: traffic to the robot is *poses*,
  and poses only flow when the robot is not driving. A latched move is the one
  state that produces no traffic, because the firmware walks on by itself
  (ASSUMPTIONS B3/D4) -- so the single state the watchdog guards was the single
  state that starved it. The robot stopped and crouched after 1.5 s of walking
  on a perfectly healthy link, reported from the teach UI 2026-08-22. The host
  now sends `ping` every 500 ms while and only while a move is latched, which
  also changes what the watchdog means: from "has the host said anything
  lately" to "is the host still there and does it still want this move".
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
- Live camera and sensor tuning from the teach UI -> `CAMERA` (the stream, which
  the vendor firmware serves too) and `CAMERA_TUNING` (the `cam_*` family, ours).
  ✅ **done and confirmed on the robot 2026-08-22.** The operator watched the
  MJPEG feed in the Camera tab and moved contrast, brightness and the rest with
  the change visible in the picture. Three things had to be true at once and
  each was wrong first:
  * the robot ran a build from **before** the camera code, so every write
    answered 500 and the page reported it in a line nobody looks at. Flashed;
    the page now reverts a refused control and says why at the control.
  * the frame-size list stopped at QVGA because `ROBODOG_CAM_MAX_SIZE` was read
    as the constant it is *initialised* to. The fork overwrites it at boot from
    what `esp_camera_init` accepted -- **VGA on this unit** -- so three usable
    resolutions were missing. This is why the picture now "looks a lot sharper".
  * ASSUMPTIONS F4 was open: `psram=0`, measured. No PSRAM, and VGA fits in
    internal DRAM regardless, so the vendor's QVGA was a choice.
  ✅ **and readback over Wi-Fi**: every `cam_*` request now answers with the
  sensor's own state as JSON, so a write confirms itself in the same round trip
  and a change made from the vendor's page or a serial session shows up on the
  next one. It carries `size_max` too -- the frame-buffer ceiling this
  particular robot managed to allocate, which nothing else can discover (F4) --
  so the size list is capped per robot rather than offering settings the
  firmware would silently clamp. Firmware without the reply still works and the
  page says it is remembering rather than reading.
- Active telemetry: battery voltage and the **full** IMU (the stock firmware
  reads only 2 of the ICM20948's 9 axes) → `TELEMETRY`.
  ✅ **the IMU, written 2026-08-23, not yet flashed.** All nine axes: the fork
  configures the gyroscope and magnetometer the vendor leaves untouched, and
  samples accelerometer and gyroscope at 50 Hz from `loop()` -- the only place
  allowed to touch I2C -- into a 64-slot ring. `var=imu&val=<last seq>` returns
  everything since the host's last sequence, on both transports.
  **Buffered, because it has to be:** a request costs 78-140 ms, so polling for
  single samples would yield eight a second, which is useless for integrating a
  gyroscope. Ten requests a second now carry fifty samples a second.
  And the poll costs nothing extra: it *replaces* the watchdog keep-alive
  `ping`, which was already a round trip that carried no data, at exactly the
  moment the attitude matters most -- a robot that is walking is a robot that is
  being sent nothing else.
  Host side: `robodog.localization` folds the samples into a complementary
  filter (pitch, roll, degrees turned, and whether the body is still), which
  `RobotClient.attitude` exposes and the vision geometry uses to remove body
  tilt from its distance estimate -- the largest error in it (G2/G8).
  ⏳ **open:** flashing and the three sign checks in G9; battery voltage from
  the INA219, which is the same pattern and not yet written.
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

## M8 — Vision-guided behaviours ("Komm zu mir") ⏳ built, not yet on the robot

**Scheduled ahead of M5/M6 at the owner's request (2026-08-23)**, and to be
started from the owner's AI machine. This is the first motion with nobody's
hand on the control: a spoken or typed command, a person found in the camera,
and the robot walking towards them.

### The constraint that shapes it

This unit has **no PSRAM** (`psram=0`, read back 2026-08-22), so the firmware
runs `fb_count = 1`: a browser holding `/stream` blocks every other frame grab.
There can be exactly **one** consumer of the robot's stream.

So the host reads it once and re-serves it: the teach UI's Camera tab shows the
same frames with detection boxes drawn on them, and the vision loop never
competes with the browser for the single frame buffer.

### Decided

- **Detector: YOLO via `ultralytics`**, behind a new optional `vision` extra
  like `sim` and `viz` are. Never imported from a mock/http path, so the
  no-extras install stays green without it.
- **LLM: the owner's vLLM server**, an OpenAI-compatible `/v1/chat/completions`
  endpoint. Base URL and model name configurable; no key assumed.
- **The model does not drive.** It maps free text to a *named behaviour plus
  parameters*, once, before anything moves. A deterministic loop then runs the
  behaviour through `SafetySupervisor`. Two reasons, and the first is the hard
  rule of this repo: nothing reaches a backend around the supervisor. The
  second is that an 8B model inside a 10 Hz control loop is latency and
  non-determinism in the one place neither belongs. Emitting a structured call
  is also what a coder model is best at.

### Shape

- `robodog.vision` — `Detection(label, bearing, height_fraction, confidence)`
  and a `Detector` protocol; the YOLO implementation behind the extra. Bearing
  is normalised (-1 left .. +1 right) so the behaviour never sees pixels.
- `robodog.vision.stream` — the MJPEG reader (multipart/x-mixed-replace) and
  the re-broadcaster the UI and the behaviour both read from.
- `robodog.behaviour` — **pure logic**: detections and a clock in, drive
  intents out, as a state machine (searching / approaching / arrived / lost).
  Detector and clock injected, so its tests need no model, no camera and no
  robot, exactly as the kinematics tests need no hardware.
- `robodog.ai` — text to behaviour-call, against the vLLM endpoint. Refuses
  anything not in the behaviour vocabulary rather than improvising.

### Safety, which is new here

The target is a person and the robot has no depth sensor, no bumper and no
servo feedback. Distance is inferred from bounding-box height, which is a
guess, so: a conservative stop size, a hard timeout on the whole behaviour, the
page's dead-man's switch ending it like any run, and STOP/Escape still the
fastest way out. The on-device watchdog keeps being fed while it walks, as it
is now.

### Built 2026-08-23, with the robot offline

Everything on the list above exists and is tested; what is missing is a robot to
point it at. `robodog.vision` (`Box`/`Detection`/`Detector`, the MJPEG parser,
the frame hub, the YOLO detector behind the extra), `robodog.behaviour` (the
state machine, its vocabulary, the runner), `robodog.ai`, and the teach UI's
half: the picture re-served at `/camera/stream` with detection boxes drawn over
it, and a command box beside both tabs. 67 new headless tests, all green on an
install that never fetched the extra.

**Measured against the owner's vLLM server** (Qwen3.8-27B FP8, vLLM 0.27.1):
mapping "Komm zu mir" onto a behaviour call takes **0.68 s with thinking off and
30.8 s with it on**, for an identical answer -- 23 completion tokens against
1225. The client therefore sends `chat_template_kwargs: {enable_thinking:
false}`, constrains the answer with the vocabulary's own JSON schema, and still
handles a reasoning reply in case a server ignores the switch. Six phrasings
were mapped correctly end to end, refusal included ("mach einen Rückwärtssalto"
→ `unknown`).

**The three open questions, answered:**

- *Box height to distance* — the safety path does not wait for it. The
  behaviour stops on the **height fraction itself**; the conversion to
  millimetres exists only to turn an operator's "zwei Meter" into a fraction,
  and its result is clamped. Building it surfaced a real error in the obvious
  approach: the textbook `size / distance` formula assumes the subject fits in
  the picture, and with the camera about 140 mm off the floor **a person is
  clipped by the top of the frame from ~3.4 m inward** — a threshold picked
  from that formula would never have been crossed. The geometry now models the
  clipping (ASSUMPTIONS G2).
- *Searching* — turns in place in **pulses** (0.6 s turning, 0.5 s looking) so
  the detector gets an unsmeared frame, and gives up after 12 s. A duration and
  not an angle, because the robot's turn rate is unmeasured (G4).
- *Stop distance* — **both.** The behaviour owns a hard ceiling nothing can
  raise; the operator sets the value inside it per run, spoken or typed (G5).

**The detector, measured on the AI machine 2026-08-23** against real photographs
(the robot being offline): `yolo11n` on **CPU** takes 40-51 ms a frame, 20-25
fps against a camera that delivers ten-odd — so torch is pinned to the CPU wheel
index, which is 200 MB rather than the CUDA build's 5-6 GB, and leaves both
3090s to the language model. On `bus.jpg` the pipeline found four people and
chose the *nearest* rather than the most central, then turned in place towards
it without walking, because its bearing was outside the walk-at band. That is
the whole chain except the two halves that need hardware.

### First run on the robot, 2026-08-23

Detection worked and **G1 is closed**: a person to one side, "Komm zu mir" typed
into the teach UI, and the robot turned towards them. Two defects showed up in
the approach itself, and both are fixed:

- **It rocked left and right and closed on nothing.** The box centre jitters
  more than the steering law tolerated, and a latched turn always overshoots
  centre because the picture is tens of milliseconds old. The bearing is now
  low-passed (0.35 s) and each steering band has separate entry and exit
  thresholds. Measured in closed-loop simulation at 0.25 of jitter: **68 turn
  reversals per approach before, 0 after**, and 4 of 4 approaches reaching the
  target instead of 2. Worth recording which half did the work -- hysteresis
  alone only got to 42, so **the filter is the fix and the hysteresis is the
  belt to its braces**, the opposite of the order they were written in
  (ASSUMPTIONS G7).
- **It gave up exactly when it arrived.** Up close the detector stops calling a
  fraction of a person a person, and losing the target was read as "go and
  look for it". A target being approached is now kept at confidence 0.25 where
  acquiring one needs 0.40, and a target lost while it filled more than 30% of
  the frame ends the run as ARRIVED, reporting the size it was lost at so the
  threshold can be calibrated (ASSUMPTIONS G6). Deliberately not a tracker:
  a tracker keeps reporting a box after it drifts, and the robot then walks at
  a guess of a person it cannot see.

**Then it stopped too far away, and the cause was the fix above.** The
loss-is-arrival threshold had been written as an absolute 0.30 of frame height,
which under the geometry is **6.25 m** -- so every detection dropout anywhere in
the approach counted as arrival and ended the run. It is now a *margin* below
the stop size (0.10) rather than an independent number, because the two describe
the same event. With that, plus the default stop raised from 0.60 to 0.66 and
the ceiling from 0.70 to 0.80 on the owner's instruction, the robot closes to
**0.95 m by default and 0.5 m on request**, against a flat 1.5 m before -- and in
simulation it now gets as near as the detector allows rather than to a fixed
distance. The ceiling's honest meaning is recorded in ASSUMPTIONS G5: the robot
may touch you.

A third defect turned up in the same round and is worth keeping: the detector's
own confidence floor sat at 0.35, *above* the behaviour's 0.25 keep threshold,
so "hold a faint target" was unreachable code. The floor is now 0.20 -- though
measurement (2026-08-23) says that was not the binding constraint either: YOLO
names a person from feet and shins alone at 0.70 confidence, so whatever made
the robot lose its target up close, it was not the partial view. Most likely the
person left the frame sideways -- at 0.5 m a 65-degree lens sees 0.64 m across.
The Camera tab settles it at the next run (G6).

The same simulation also found that the 12 s search **could not complete a
revolution** -- at a plausible turn rate the pulsing needs some 16 s -- so a
target behind the robot was never found. Now 20 s (G4).

**Still to do, and all of it needs the robot:** the G2 calibration session, the
frame rate against the real MJPEG stream rather than files, calibrating G6's
threshold from a few approaches, and the acceptance run itself.

**Acceptance:** "Komm zu mir" typed into the teach UI turns the robot towards a
person and walks it to a stop at a safe distance; the behaviour's state machine
is covered headlessly with a scripted detector; the suite passes with the
`vision` extra absent. — *The second and third are met: the state machine, the
runner, the parser, the vocabulary and the whole teach-UI path are covered
against a scripted detector and a fake model server, none of which needs the
extra. The first waits for the robot.*

## M7 — Outlook (not scheduled)

- USB-serial backend as a second transport (the serial JSON path accepts
  gestures, LEDs and buzzer, which Wi-Fi does not).
- URDF/mesh pipeline from the parametric CadQuery leg model (`wavego_leg.py`,
  currently custom/out-of-repo) → higher-fidelity twin.
- IMU-based closed-loop behaviors. (Vision moved out of this list into M8,
  which is scheduled.)
- Optional servo feedback if it is ever wanted: potentiometer taps or a swap to
  SC09 bus servos — deliberately **not** on the critical path.
