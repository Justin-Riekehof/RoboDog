# Assumptions

Single registry of everything we believe about the WAVEGO but have not (yet)
verified on the actual robot. Every entry is a testable claim with a source
and a verification status. **Rule: no assumption lives only in code** — if code
depends on an unverified behavior, it must reference an entry here.

Statuses: `unverified` (from documentation/firmware reading only),
`verified` (confirmed on our robot, with date), `wrong` (kept for the record,
with the correction).

Primary sources: firmware pinned in [vendor/wavego-firmware/](vendor/wavego-firmware/)
(from <https://github.com/waveshare/WAVEGO>, MIT, unchanged since 2022);
research notes with full citations in
[docs/research/phase0-source-research.md](docs/research/phase0-source-research.md).

## A. Hardware variant

| # | Claim | Source | Status |
|---|-------|--------|--------|
| A1 | Our unit is a WAVEGO Standard BASIC (SKU 22615, EU), no Raspberry Pi | order #3222, waveshare.com SKU table | verified 2026-08-10 (order) — confirm on unboxing |
| A2 | Shipped firmware equals the 2022 GitHub state we vendored | repo frozen since 2022-05 | unverified — check on first connect |
| A3 | Servos are analog PWM with no position feedback of any kind | firmware has zero read paths; product page "pulse width modification" | unverified on device (firmware evidence strong) |

## B. Serial protocol (stock firmware)

| # | Claim | Source | Status |
|---|-------|--------|--------|
| B1 | UART 115200 baud over the USB Type-C port reaches the JSON parser | `WAVEGO.ino` `Serial.begin(115200)`; serialCtrl reads `Serial` | unverified — Type-C→`Serial` routing must be confirmed |
| B2 | Commands are `{"var": "<name>", "val": <int>}`, one JSON doc per read, polled every 25 ms | `WAVEGO.ino` serialCtrl / robotThreadings | unverified |
| B3 | `move`: 1=Forward 2=TurnLeft 3=FBStop 4=TurnRight 5=Backward 6=LRStop; movement continues until the matching stop | `WAVEGO.ino` L99–112 | **verified 2026-08-22** on the device via HTTP (the two paths share these values): move=1 walked forward, move=2 turned left in place, and each stopped only on its matching stop command. The latching half was watched deliberately -- the robot kept walking with no further commands sent. [Report](docs/bringup/bringup-2026-08-22-run3.md) |
| B4 | `funcMode`: 1=Steady(toggle) 2=StayLow 3=Handshake 4=Jump 5/6/7=ActionA/B/C 8=InitPos 9=MiddlePos; 2–4 are blocking animations | `WAVEGO.ino` L85–98, `ServoCtrl.h` | **verified 2026-08-11** through our own client (funcMode 2 crouched and rose; funcMode 9 moved all servos to their middle) and independently through the firmware's web UI (all other modes) |
| B5 | `ges`: 1/2=pitch ±2 (clamp ±15), 4/5=yaw ∓2 (clamp ±15), 3/6=stop; increments, not absolute | `WAVEGO.ino` L113–126 | unverified |
| B6 | `light` 0–7 and `buzzer` 0/1 work as documented | `WAVEGO.ino` L127–148 | unverified |
| B7 | Firmware sends **no** responses/telemetry over serial except free-text `println`s; `jsonSend()` exists but is never called | grep over firmware | unverified |
| B8 | A `move` command interrupts a running blocking animation only after the animation finishes (single robot task loop) | firmware structure (25 ms task + blocking `functionX` loops) | **verified 2026-08-11**: during the handshake the robot ignored commands until the animation finished |
| B9 | There is **no link watchdog** in the stock firmware: last `move` state persists indefinitely if the host dies | absence in firmware | **verified 2026-08-11**: with the link deliberately cut mid-walk the robot kept going. The step was originally recorded as a deviation because *our tooling* failed at that moment, not the robot (see the session table) |

## C. Kinematics & servo mapping

| # | Claim | Source | Status |
|---|-------|--------|--------|
| C1 | Linkage constants: S=12.2, A=40.0, B=40.0, C=39.8153, D=31.7750, E=30.8076, W=19.15 (mm) | `ServoCtrl.h` L28–56 | unverified (assumed exact for sim) |
| C2 | Angle→PWM: `round(200·deg/90)·direction + middle(300 default)`. **Partly wrong as first read:** the original entry inferred "window 263–463 counts ≈ ±45° usable", but SERVOMIN/SERVOMAX are *scale only* — `goalPWMSet` never clamps, and the firmware's own stay-low commands back=83.3° on the hind legs (485 counts, beyond the window), which ran on our device on 2026-08-11. Our safety limit is therefore ±90° per joint (admits the firmware's own repertoire, catches runaways); the true mechanical end stops remain unmeasured | `ServoCtrl.h` goalPWMSet, defines; crouch geometry via our IK | mapping formula unverified on device; the ±45° inference is **wrong**, corrected 2026-08-19 |
| C3 | Servo channel map: leg1 F/B/W=8/9/10, leg2=14/15/13, leg3=7/6/5, leg4=1/0/2; direction array as in firmware | `ServoCtrl.h` L89–129 | unverified |
| C4 | Workspace limits: height 75–110 mm, lateral ±30 mm, gesture ±15, balance ±21 — safe envelope for the real mechanics | `ServoCtrl.h` constants | unverified — the mechanical envelope might be tighter. Note these are the *firmware's* clamps; ours are no longer shaped the same way (see C12 for the height clamp, C13 for the lateral one) |
| C5 | Leg numbering 1=FL 2=HL 3=FR 4=HR; per-leg frame x forward, y down-positive, z outward; hind legs get mirrored x | Wiki API page + `standUp()` | unverified |
| C6 | `wigglePlaneIK` `bIn==0` branch skips the `−LW²` correction (firmware quirk). **No longer unreachable:** the original note said "unreachable in practice since y ≥ 75", which held only while the workspace was clamped as a Cartesian box. Once the roll envelope was opened (C13), y = 0 sits at roughly 78.6° of roll, in the middle of the usable range. Measured effect at that point: the branch returns 11.4° instead of 78.6° and the commanded pose lands **108.6 mm** from the target. Only exactly `y == 0.0` is affected (y = ±1e-9 is exact); the FK cross-check (`max_ik_deviation`) rejects it, and a regression test pins that | `ServoCtrl.h` L325–329 | verified against source 2026-08-10 (port replicates it); reachability note **corrected 2026-08-21** |
| C7 | Default stand pose (x=±16, y=95, z=25) is statically stable on the real robot | `standUp()` defaults | unverified |
| C8 | Crouch at minimum height (y=75) is a safe/stable E-stop pose | our design choice | unverified — validate on hardware (M2) |
| C12 | The 75–110 mm height clamp is the **gait envelope, not the mechanical limit**: the firmware's own `middlePosAll()` (funcMode 9, all servos to calibrated middle) puts the foot at y ≈ 115.2 mm by our exact FK. So joint-level commands must not be validated against the walking box — they are gated by joint-angle bounds and linkage assembly instead | `ServoCtrl.h` middlePosAll + our FK, 2026-08-11 | **verified 2026-08-11**, and again 2026-08-21 when a calibration run started with it: funcMode 9 straightened all legs and the body sat higher than the walking envelope, as predicted (our FK: foot at 115.16 mm, 5.2 mm past the walking maximum; the linkage's own maximum is ~117 mm at fore=back=10°). Worth stating plainly because it reads as a fault: the legs go nearly vertical and the two coaxial cranks sit at angle 0, which *looks* like they are straining against each other but is a valid, unstrained assembly. Note also that this is the pose in which the wiggle servo carries the longest lever arm, so a roll range measured from it is a conservative lower bound. True mechanical end stops still unmeasured |
| C11 | The firmware's IK is **approximate in one region**: `singleLegPlaneIK` reconstructs the knee via `asin((y − cos(β)·A)/hypot(E, C+D))`, which cannot distinguish a forward from a rearward elbow→foot tilt. When the foot lies behind the rear crank's elbow, the knee is mirrored and the commanded front-crank angle is wrong — the two coaxial servos then command inconsistent knee positions and the rigid linkage fights itself. Measured foot-position deviation up to **10.9 mm**. Region boundary (at z=25): no effect for y ≤ 85 mm; x ≲ −44 at y=90, x ≲ −39 at y=100, x ≲ −29 at y=110. The **default gait never enters it** (deviation 0.000 mm over a full cycle, since its deep-x extremes occur near y≈87). Port stays faithful; the safety layer rejects such targets via `LimitConfig.max_ik_deviation`, using our exact FK as the detector | derived from `ServoCtrl.h` L400–423; measured by our tests 2026-08-11 | verified against source — hardware effect (servo strain) untested, must never be commanded |
| C10 | The firmware's box clamps do **not** guarantee reachability: the true workspace is curved, and corners that pass the clamps (e.g. x=30, y=110, z=50 → required reach 124.6 mm vs. 117.9 mm available) are unreachable. There `acos()` leaves the domain, C++ yields NaN, and the NaN flows unchecked into `round()`/int cast → arbitrary PWM. Our IK raises `KinematicsError` and the safety layer rejects such targets (`LimitConfig.require_reachable`) | derived from `ServoCtrl.h` geometry; verified numerically by our tests 2026-08-11 | verified against source — hardware consequence (what the servo actually does) untested and must never be triggered |
| C13 | **Leg roll (the wiggle servo) spans -27.0 to +135.0 deg, measured on the robot; the workspace is bounded in the leg's own coordinates, not as a Cartesian box.** The linkage is planar and the wiggle servo rotates that whole plane about the fore-aft axis, so height and lateral offset are not independent: rolling trades one for the other along an arc. The original box (y 75-110, z -20...60) therefore admitted only about -23...+22 deg of roll. Limits are now checked on the pair the firmware IK computes first: in-plane reach (75-110 mm, unchanged) and roll angle. **Measured 2026-08-21** with `robodog calibrate-roll` on the front-left wiggle servo (channel 10), 20-count steps: it followed to **+135.0 deg** (300 counts, run 3) and to **-27.0 deg** (60 counts, run 4), meeting its mechanical stop just past each -- the stops themselves lie in +135...+144 and -27...-36. Total span 162 deg, which brackets the ~170 deg the same leg reaches when pushed by hand. Two guesses preceded this and both were wrong: a symmetric +/-85 (wrong in *shape* -- the range is one-sided) and a provisional -30...+170. Our kinematics is exact over the whole circle regardless: fk(ik(p)) reproduces the foot to 0 mm at every angle, `fore`/`back` stay constant, and the decomposition round-trips | `robodog calibrate-roll` runs 3 and 4, `docs/bringup/roll-calibration-2026-08-21-run3.md` (upper end from the terminal transcript; that run recorded nothing because stopping discarded the sweep, fixed since) and `-run4.md` | **verified 2026-08-21 on the device, with three caveats that keep it from being the whole truth.** (1) **One leg only** -- front_left; the other three are unmeasured, and the hind-right leg on this robot is a repaired part with its own geometry (F1/F3). (2) **Measured from the funcMode-9 pose**, the leg fully extended, which is the wiggle servo's longest lever arm, so these are a conservative lower bound on the free range. (3) **Powered range < hand range:** the same leg reaches ~170 deg when back-driven by hand but the servo drives only 135 -- so poses posed by hand are not necessarily commandable. **Self-collision between legs and body is still not checked anywhere**, and large roll angles will produce it |
| C9 | Turn gait quirk: `simpleGait`/`triangularGait` pass `statusInput=1.5`, but the parameter is `uint8_t`, so C++ truncates it to 1 — the intended 1.5× step range for turning never takes effect. Port replicates the truncation | `ServoCtrl.h` L553 vs. L604–615 | verified against source 2026-08-10 |

## D. Wi-Fi / HTTP transport (the bring-up channel)

| # | Claim | Source | Status |
|---|-------|--------|--------|
| D1 | ESP32 boots as AP `WAVESHARE Robot`, password `1234567890`, IP 192.168.4.1; HTTP `GET /control?var=..&val=..&cmd=..` on port 80, index page on `/`, MJPEG on port 81 `/stream` | `app_httpd.cpp` L5–6, L82, L337–368 | **verified 2026-08-11** on the device: joined the AP, `/` served the web UI, our client reached `/control` |
| D2 | **CORRECTED** (earlier entry claimed the full serial command set). The HTTP handler accepts **only five** variables: `framesize`, `funcMode`, `sconfig`, `sset`, `move`. There is **no `ges`, no `light`, no `buzzer` over HTTP** — those exist only in the serial JSON path. Wi-Fi therefore gives locomotion + function modes + servo trim, nothing else | `app_httpd.cpp` `cmd_handler` L246–317 | verified against source 2026-08-11; device-untested |
| D3 | All three query keys `var`, `val` **and** `cmd` are mandatory; a request missing any one returns HTTP 404 without acting. Successful requests return HTTP 200 with an **empty body** (plus `Access-Control-Allow-Origin: *`) — there is no readback of any kind | `app_httpd.cpp` L221–228, L323–324 | **verified 2026-08-11** on the device (two stop commands accepted, empty responses) |
| D4 | `move` values behave as on serial (1=Forward, 2=TurnLeft, 3=FBStop, 4=TurnRight, 5=Backward, 6=LRStop) and set `debugMode=0`; forward/backward and turning are **independent latched axes**, so stopping requires both `move=3` and `move=6` | `app_httpd.cpp` L284–311 | **verified 2026-08-22** on the device: forward walked and stopped on move=3; turning stopped independently on move=6. This is what the `kind: sequence` timing rests on -- a move that did not latch would make every duration in a routine a lie. [Report](docs/bringup/bringup-2026-08-22-run3.md) |
| D5 | `sconfig` (val=servoID, cmd=relative offset) sets `debugMode=1` **directly from HTTP — no G12 jumper needed**, contradicting the wiki's hardware-jumper instruction. In `debugMode` the normal control loop is suspended (LEDs orange), so gait and trim cannot fight each other. Any `move`/`funcMode` command returns `debugMode` to 0 | `app_httpd.cpp` L266-272 vs. `ServoCtrl.h` `wireDebugDetect()`; first device contact 2026-08-21 | verified against source 2026-08-11. **First hardware attempt 2026-08-21:** one `sconfig` reached the robot and moved the leg, so the no-jumper claim holds. But when our watchdog then tripped, the E-stop's `move` stop commands **both timed out** (`var=move&val=3` and `val=6`). Unresolved whether `debugMode` makes the HTTP handler unresponsive to `move` -- which would mean no stop can reach the robot while trimming -- or whether this was a link glitch; a single observation, and the run was aborted by a bug of ours (missing watchdog feed) rather than by the robot. **Re-check deliberately before trusting a stop during trimming.** Mitigating factor: nothing is walking during a calibration sweep |
| D6 | **Safety-critical:** the first `sconfig` call for a given servo makes it **jump**, because the firmware's `CurrentPWM[]` is not updated by `GoalPosAll()`. After boot the legs physically hold the stand pose while `CurrentPWM[]` still reads 300, so the first relative nudge snaps that servo to ≈middle — for the front-left fore servo that is a ≈47° jump. Sending `funcMode=9` (middlePos) or `funcMode=8` (initPos) first synchronises `CurrentPWM[]` with reality and removes the jump | `ServoCtrl.h` `servoDebug`/`GoalPosAll`/`middlePosAll`, computed with our servo map | verified against source 2026-08-11; the `funcMode=9` half is **verified on the device 2026-08-11**. The jump itself is untested (deliberately — it **must be respected during calibration**) |
| D7 | **The robot stands up by itself a second after power-on**: `setup()` calls `standMassCenter(0,0); GoalPosAll()` before Wi-Fi even starts. Powering on is a motion event, not a quiet state | `WAVEGO.ino` L210–217 | **verified 2026-08-11** on the device: the legs move into a stand by themselves at power-on |
| D8 | (Superseded for our own firmware: `firmware/wavego-robodog` adds `var=pose`, which carries all twelve values in one request and applies them together in 78-94 ms, measured 2026-08-22. The entry below still describes the **stock** firmware, where this is the only option.) Absolute per-servo positioning over Wi-Fi is possible without custom firmware: `funcMode=9` to establish a known baseline, then one `sconfig` request per servo with the delta to the target PWM count. One HTTP request per servo, no synchronised multi-servo update, so it suits **static poses only** | derived from D5/D6 | unverified — this is the pre-M5 escape hatch for sim-to-real pose checks |
| D9 | Default Wi-Fi mode is AP (`DEFAULT_WIFI_MODE 1`). Station mode exists but the credentials are **hard-coded in the firmware** (shipped with the developer's own SSID `OnePlus 8`), so joining your own network requires recompiling — i.e. AP mode is the only option before M5 | `app_httpd.cpp` L5–10, L112–114 | verified against source 2026-08-11 |
| D11 | **`funcMode` 8 and 9 never clear themselves, unlike 2-7.** `robotCtrl()` resets `funcMode = 0` after running stayLow/handshake/jump/actionA-C, but the `initPos` (8) and `middlePos` (9) branches do not — so after one `funcMode=9` the control loop keeps calling `middlePosAll()` forever, rewriting `CurrentPWM[]` for all 16 servos on every iteration (`SERVO_MOVE_EVERY = 0`, so the sweep is only I2C-bound). Consequence for trimming: a `sconfig` sent while that loop runs is applied and then immediately overwritten, so **the leg does not move and the operator correctly reports that it did not** — which a naive sweep reads as an end stop. The escape is a zero-offset `sconfig`, because the HTTP handler sets `debugMode=1; funcMode=0` *before* calling `servoDebug`, suspending the loop while moving nothing. `robodog calibrate-roll` does this after every baseline | `ServoCtrl.h` robotCtrl L1030-1080, `middlePosAll` L179-188, `app_httpd.cpp` L266-272 | derived from source 2026-08-21 after a calibration run measured a nonsensical +/-2 counts; **explains that run**, but the fix itself is device-untested |
| D10 | **No link watchdog on this path either** (see B9), and over Wi-Fi we cannot mitigate it at all: if the connection drops mid-`move`, no stop command can reach the robot and it keeps walking until power is removed. Bring-up therefore requires the robot on a stand or a hand on the power switch | absence in firmware | **verified 2026-08-11** together with B9: the robot kept walking with the link gone, and no command could reach it. This is the residual risk M4 closes. **2026-08-22: a re-test appeared to contradict this** -- the operator reported the robot "just straightened the legs out" when the link was cut. Treated as **inconclusive, not a refutation**: the preceding bring-up step had sent `funcMode=9`, which never clears itself (D11), so the firmware was repeating `middlePosAll()` and the walk the test needed most likely never started. Straight legs held forever is what that state looks like. The stock source contains no timer of any kind -- there is nothing that *could* stop it. `robodog bringup` now starts that walk itself and voids the observation if it did not run. **CLOSED on our own firmware 2026-08-22:** `firmware/wavego-robodog` adds an on-device watchdog and it was measured on this robot -- armed at 2000 ms, walking, fed by `ping` every 400 ms for 3.5 s without tripping, then silence: `WATCHDOG: link lost, stopping and crouching` followed by `stayLow`, at the armed deadline. The stock firmware is unchanged and this entry still describes it -- the watchdog is off until a host asks for it. **2026-08-22, from the first walk driven by hand:** the on-device watchdog needs feeding by the host and the first version of it had none, so the robot stopped and crouched after ~1.5 s of walking on a healthy link. The cause is an inversion worth remembering: the watchdog acts only on a *moving* robot, and a moving robot is exactly what the host sends nothing to, because the move latches and the firmware walks on by itself (B3/D4). The host now sends `ping` every 500 ms while and only while a move is latched. `robodog bringup` does **not** tick, so it still starves the watchdog while the operator answers a prompt -- run it with `--firmware stock` or expect its motion steps to be cut short on a flashed robot |
| D11 | No WebSocket, no ESP-NOW, no Bluetooth in stock firmware | grep over firmware | verified against source 2026-08-10 |

## E. Simulation fidelity (M1+)

| # | Claim | Source | Status |
|---|-------|--------|--------|
| E1 | Masses, inertias, CoM are unknown; twin uses estimates until measured | — | open |
| E4 | **The twin's legs are a serial stand-in for the real five-bar linkage.** Simulating the closed loop would need equality constraints and buys little: what determines locomotion is where each *foot* is over time, and `leg_joint_angles()` places the simulated foot **exactly** where the ported firmware kinematics commands it (verified to 1e-6 mm across the workspace by our tests, including the LINKAGE_W hip offset). What the simplification does change is the mass distribution inside the leg and the joint torques needed — so gait geometry and stability are meaningful, absolute servo-load numbers are not | `robodog/sim/model.py`, design decision 2026-08-11 | deliberate simplification, documented; revisit if servo load ever matters |
| E5 | Trunk and hip layout of the twin (140x76x40 mm body, hips at +/-55 x +/-38 mm) is approximate: the firmware never needs these numbers, so they come from the product dimensions (E3) rather than measurement | `robodog/sim/model.py` | unverified — measure on the real robot when convenient |
| E6 | The twin walks nearly straight (yaw drift a few degrees over 3 s, 27 cm travelled), because the model is perfectly symmetric. This is the **reference our real robot deviates from** (F1), and the gap is the measure of the hardware fault | our physics tests 2026-08-11 | verified in simulation |
| E2 | Servo dynamics: 0.1 s/60° at 6 V, stall 2.3 kg·cm (product page) vs 5.2 kg·cm (wiki) — contradictory | product page / wiki | contradictory — measure if it ever matters |
| E3 | Overall robot dims L218×W116×H152 mm, 465 g without batteries | product page (Standard) | unverified |

## F. Condition of our specific robot (not design claims)

| # | Claim | Source | Status |
|---|-------|--------|--------|
| F1 | **The right side of our robot under-performs during walking.** Observed in the firmware's *own* web UI: "Forward" veers right, "Backward" veers left, while "Left" and "Right" turn correctly. That combination is diagnostic. Turning works, so the hip/wiggle servos and lateral motion are fine. The forward gait commands a *perfectly straight* walk (verified numerically: all four legs get an identical 50.00 mm stride with 0.00 mm lateral spread, and the servo direction table is an exact left/right mirror), so the software is exonerated. An asymmetry that reverses its sense between forward and backward travel is the signature of a **left/right thrust asymmetry**: the left side covers more ground per cycle than the right. Suspects are therefore the two right legs — FRONT_RIGHT (PCA9685 channels 7/6/5) and HIND_RIGHT (1/0/2). **The owner's repaired leg is HIND_RIGHT (F3), which makes deviating link geometry the leading hypothesis**: the firmware commands angles, and a link length that differs from C1 turns those angles into a different foot trajectory and therefore a different stride | bring-up session 2026-08-11 (run 2) + operator clarification; symmetry checked against `ServoCtrl.h` by our tests | observed on the device; narrowed to the right side, root cause open — diagnose before trusting any locomotion result (M3) |
| F2 | The ESP32 camera shows no image in the web UI (black frame) while control works | bring-up session 2026-08-11 | **explained 2026-08-22, and it is none of the guessed causes.** Measured step by step: port 80 serves the vendor page; port 81 answers `This URI does not exist` for `/`, so the stream server runs; `esp_camera_init` returns **ESP_OK** (upstream commented out both its error message *and* its `return`, so a failure would have been silent -- our fork prints it); and `esp_camera_fb_get()` delivers a valid **320x240 JPEG within a second**, fetched over USB with the fork's `snap` command. With more light: mean brightness 116.5, spread 72.7, range 10-255, and the text on the object is legible ([image](docs/bringup/camera-2026-08-22.jpg), [same scene dark](docs/bringup/camera-2026-08-22-dark.jpg)). **Sensor, ribbon, init and frame buffer are all fine** -- the black frame of 2026-08-11 was not a dead camera. **And the stream works too, confirmed 2026-08-22** from a laptop on the robot's access point: `http://192.168.4.1:81/stream` delivers live MJPEG. So the whole chain is sound -- sensor, init, frame buffer, stream handler, browser. What the 2026-08-11 black frame actually was is no longer determinable, but note that the vendor page leaves `<img id="stream" src="">` empty until its **Start** button is pressed: an unpressed button and a broken camera look identical. Camera work needs no firmware change |
| F3 | Our unit has a repaired/replaced leg part (owner's earlier CadQuery repair project, `wavego_leg.py`). If that part sits on a **right-side** leg, or if its link lengths deviate from C1 (A=B=40.0, C=39.8153, D=31.7750, E=30.8076 mm), it is the most likely cause of F1: a different link geometry changes that leg's foot trajectory and therefore its stride | owner, 2026-08-11 | **the repair is on HIND_RIGHT** — one of the exactly two legs F1 points at, so this is very probably the cause of F1. Remaining question: do the repaired part's measured link lengths match C1? |
| F4 | **Whether our unit has PSRAM is unknown**, and it sets the ceiling for camera resolution: without it the frame buffer comes out of internal DRAM, shared with Wi-Fi and the OLED buffer. `firmware/common.ini` pins `board = esp32dev`, which neither confirms nor rules it out, and the vendor's `FRAMESIZE_QVGA` with `fb_count = 1` is what a board without PSRAM would need -- but also what a vendor might simply have chosen | reading the fork, the pinned board definition and the core headers, 2026-08-22 | **unverified on the device, and deliberately not assumed.** The fork asks `psramFound()` at boot and picks SVGA/quality 10/two buffers or VGA/quality 12/one from the answer, which is meaningful because the pinned core is built with `CONFIG_SPIRAM_SUPPORT 1` and calls `psramInit()` in `initArduino()` (verified in the installed core, not assumed). If the buffer still does not fit, `robodogCameraStart` falls back to the vendor's QVGA rather than leave the robot blind. `cam_report` prints the answer as `psram=0|1` -- read it once and this entry can be closed |
| F5 | **The serial `snap` path costs seconds per frame, and raising the resolution raises that cost.** At 115200 baud, 8N1, the link carries 11,520 bytes/s, and base64 adds a third: a 40 KB VGA JPEG is ~4.6 s of console, an SVGA one at quality 10 closer to 7 s. The vendor's QVGA at quality 63 was under a second, which is why `snap` felt free | arithmetic from the pinned `monitor_speed` and the JPEG sizes the quality settings imply, 2026-08-22 | **calculated, not measured.** It bounds what the USB transport is good for: diagnosis and tuning, not a video path -- the MJPEG stream on port 81 stays the way to watch. Measure one `snap` per frame size at the next bring-up and replace these numbers with real ones |
| F6 | **The fork's camera defaults produce a better picture than the vendor's, and their cost is motion blur and noise.** Raising `ae_level` to +2, switching on `aec2` and lifting the gain ceiling to 16X all buy brightness with integration time and gain -- on a robot that walks, that is blur and grain. Dropping `jpeg_quality` from 63 to 10-12 costs only bandwidth (F5), and the colour settings (`awb_gain`, `raw_gma`, `lenc`) cost nothing | chosen from F2's two reference frames and the OV2640 driver, 2026-08-22 | **unverified: chosen, compiled, not yet seen.** The firmware builds (860,104 bytes, 65.6% of the default app slot). Every value moves at runtime as `cam_<name>` over both transports and `cam_reset` restores these defaults, so tuning needs no reflash. Judge it at the next bring-up against the same scene as F2, standing still *and* walking |

## How to verify

Each bring-up session (`robodog bringup`, see [docs/bringup.md](docs/bringup.md))
picks entries from sections B/C/D, tests them with the robot on a stand, and
flips the status with a date and a one-line note. Corrections go to the `wrong`
status — never silently edited away — so the history of what we believed stays
auditable.

**A finding is only valid if the command actually reached the robot.** If a step
errored, the observation that interprets it says nothing; record it as skipped,
not as a deviation. The tooling now enforces this (dependent steps are skipped
automatically).

## Bring-up sessions

| Date | Transport | Outcome |
|---|---|---|
| 2026-08-11 (run 3) | Wi-Fi / HTTP | Operator clarifications: turning is correct (so F1 is a right-side thrust deficit, not a hip-servo fault), the repaired leg is HIND_RIGHT, and B9/D10 is confirmed — the robot kept moving with the link cut. What failed at that step was **our tooling**: after a deliberate link loss it spent tens of seconds retrying stops that could not succeed. Fixed (1 s stop timeout, 2 attempts, no duplicate stop on disconnect, and the step now tells the operator to reconnect before answering) |
| 2026-08-11 (run 2) | Wi-Fi / HTTP | Six confirmed: D1, D3, D7 plus **B4, B8 and C12/D6 through our own client** — function modes, blocking behaviour and the middle-position pose all behave as read from the source. The `move` checks were not observed, so B3/D4 stays open. One real new finding: **F1**, the right side under-performs while walking (forward veers right, backward veers left, turning is correct) — visible in the firmware's own web UI, so it is a hardware matter, not ours. [Report](docs/bringup/bringup-2026-08-11-run2.md) |
| 2026-08-11 (run 1) | Wi-Fi / HTTP | Connectivity confirmed (D1, D3, D7). All motion steps void: our own safety watchdog counted the operator's reading time as a dead control loop and latched the E-stop, so **no motion command ever left the PC** — the robot did not move, and the two "differs" verdicts in the report are artefacts, not findings. Fixed (heartbeat before each action, E-stop recovery, dependent steps skipped) and pinned by a regression test. [transcript](docs/bringup/session-2026-08-11-transcript.md). Motion checks to be re-run. |

## verified/wrong

(robodog) PS C:\Users\Justin\Documents\stash\RoboDog> netsh wlan connect name="WAVESHARE Robot"
Die Verbindungsanforderung wurde erfolgreich abgeschlossen.
(robodog) PS C:\Users\Justin\Documents\stash\RoboDog> Start-Sleep 5
(robodog) PS C:\Users\Justin\Documents\stash\RoboDog> uv run robodog bringup

========================================================================
 WAVEGO bring-up over Wi-Fi (stock firmware)
========================================================================

 SAFETY, read before continuing:
  * Put the robot ON A STAND with the legs hanging free, or keep a
    hand on the power switch. Over Wi-Fi a dropped link cannot be
    recovered: no stop command reaches the robot (ASSUMPTIONS D10).
  * The firmware has no watchdog. Whatever it was last told to do, it
    keeps doing.
  * Ctrl-C triggers an E-stop attempt, but it can only work while the
    connection is alive.

Type 'yes' when the robot is secured and you are ready: yes

------------------------------------------------------------------------
[D1] Robot answers on HTTP
  Join the robot's Wi-Fi access point (SSID 'WAVESHARE Robot', password '1234567890'). The connection check already ran.
  Did the connection succeed without you changing anything? [y/n/s] y

------------------------------------------------------------------------
[D7] Robot stands up on power-on
  Recall what happened when you switched the robot on: the firmware commands the stand pose about a second into boot, before Wi-Fi starts.
  Did the legs move into a stand by themselves at power-on? [y/n/s] y

------------------------------------------------------------------------
[D3] /control answers with an empty 200
  Two stop commands were sent during connect. No response body is expected -- the firmware never returns data.
  Did that complete without an error message? [y/n/s] y

------------------------------------------------------------------------
[B3/D4] move=1 starts a forward gait
  The robot will start WALKING and keep walking until the next step stops it. Legs must hang free.
  >>> THIS MOVES THE ROBOT <<<
  Press Enter to run it, or 's' to skip: 
  Did the robot start a forward walking gait? [y/n/s] 

------------------------------------------------------------------------
[B3/D4] Motion latches until an explicit stop
  Watch the robot: no stop command has been sent yet.
  Skipping: the step it interprets (move_forward) did not run, so
  whatever you see now says nothing about this assumption.

------------------------------------------------------------------------
[D4] move=3 + move=6 stop the gait
  Both axes are being stopped now.
  Skipping: the step it interprets (move_forward) did not run, so
  whatever you see now says nothing about this assumption.

------------------------------------------------------------------------
[B3/D4] move=2 turns in place
  The robot will turn left in place until stopped.
  >>> THIS MOVES THE ROBOT <<<
  Press Enter to run it, or 's' to skip: 
  Did it turn left in place (not walk forward)? [y/n/s] 

------------------------------------------------------------------------
[D4] Turning stops independently
  Stopping both axes again.
  Skipping: the step it interprets (turn) did not run, so
  whatever you see now says nothing about this assumption.

------------------------------------------------------------------------
[B4] funcMode=2 runs the stay-low animation
  The robot will crouch and rise again -- one shot.
  >>> THIS MOVES THE ROBOT <<<
  Press Enter to run it, or 's' to skip: 
  Did it crouch and come back up by itself? [y/n/s] y

------------------------------------------------------------------------
[B8] Function animations are blocking
  A handshake will start; it takes about four seconds. While it runs, try clicking Forward in the robot's own web UI.
  >>> THIS MOVES THE ROBOT <<<
  Press Enter to run it, or 's' to skip: 
  Was the robot unresponsive to commands until it finished? [y/n/s] y

------------------------------------------------------------------------
[C12/D6] funcMode=9 moves all servos to the calibrated middle
  All twelve servos go to their stored middle position. The legs will straighten and the body sits HIGHER than the walking envelope -- support the robot.
  >>> THIS MOVES THE ROBOT <<<
  Press Enter to run it, or 's' to skip: 
  Did all legs move to a straight/neutral position? [y/n/s] y

------------------------------------------------------------------------
[B9/D10] No link watchdog in the firmware
  SAFETY TEST, do this with the robot lifted or on a stand: start a forward walk from the robot's web UI, then switch off your PC's Wi-Fi (or walk out of range) without stopping it.
  Did the robot keep walking after the link was gone? [y/n/s] n
  What happened instead? kept in movin, also the web commands are messed up. "Forward" turns right, "backward" turns left. "Left" and "Right" stay the same.
  warning: could not confirm the stop: could not confirm stop over Wi-Fi: cannot reach robot at http://192.168.4.1/control?var=move&val=3&cmd=0: <urlopen error[WinError 10065] Der Host war bei einem Socketvorgang nicht erreichbar>; cannot reach robot at http://192.168.4.1/control?var=move&val=6&cmd=0: <urlopen error [WinError 10065] Der Host war bei einem Socketvorgang nicht erreichbar>

summary: 6 confirmed, 1 differ, 5 skipped, 0 errors
report written to docs\bringup\bringup-2026-08-11.md
Next: transfer these outcomes into ASSUMPTIONS.md (verified / wrong).
(robodog) PS C:\Users\Justin\Documents\stash\RoboDog> 