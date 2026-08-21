# Phase 0 — source research, WAVEGO / WAVEGO Pro (as of 2026-08-10)

Raw reports from two research passes over the official primary sources. Every
protocol or firmware property quoted here counts as an **assumption until
verified on the real device** — the checkable ones are consolidated in
`ASSUMPTIONS.md` (phase 2).

Principal sources:

- Standard: <https://github.com/waveshare/WAVEGO> (note: NOT `waveshareteam/`), MIT licence, last commit 2022
- Pro: <https://github.com/waveshareteam/WAVEGO_Pro>, GPL-3.0, active (last push 2025-07-29, single maintainer)
- Wiki: <https://www.waveshare.com/wiki/WAVEGO>, <https://www.waveshare.com/wiki/WAVEGO:_API>, <https://www.waveshare.com/wiki/WAVEGO_Pro>, <https://www.waveshare.com/wiki/SC09_Servo>
- Product pages: <https://www.waveshare.com/wavego.htm>, <https://www.waveshare.com/wavego-pro.htm>

---

## Report 1: WAVEGO (Standard, 12-DOF)

### a) Hardware architecture

- **Sub-controller: ESP32.** Board target "ESP32 Dev Module", 240 MHz, 4 MB flash, PSRAM "Enabled" (wiki, "Upload example to WAVEGO"). Product page: "Xtensa LX6 dual-core @240MHz, SRAM: 520KB+8MB, Flash: 448KB+4MB". **UNCERTAIN:** the exact module (WROVER?) is named nowhere; 8 MB PSRAM is an inference.
- **Host controller (optional): Raspberry Pi 4B, 4 GB** ("WAVEGO PI4 KIT"). Division of roles per the wiki: the ESP32 does "connecting rod inverse solving and gait generation", the Pi does "high-level decision operating" (OpenCV: face/colour/motion).
- **Cameras:** ESP32 side OV2640 2MP (`app_httpd.cpp` L36-53, JPEG, `FRAMESIZE_QVGA`, XCLK 20 MHz). Pi side (EX/PI4 KIT): 5MP 160-degree wide angle.
- **Display:** SSD1306 OLED, I2C 0x3C, firmware configures **128x32** (`InitConfig.h` L120-125). **CONTRADICTION:** wiki says "0.96 inch", the packing list says "0.91 inch" (0.91 inch matches 128x32).
- **Peripherals (`InitConfig.h`):** INA219 current/voltage sensor @0x42 (shunt 0.01 ohm), ICM20948 IMU @0x68, 2x WS2812 (GPIO 26), buzzer (GPIO 21), PCA9685 @0x40, I2C: SDA=GPIO32 / SCL=GPIO33.
- **Power:** 2x 18650 in series (not included), 7-8.4 V, charging while running is possible, protection circuit, 5 V output for the Pi. The wiki's "5200mAh" figure is **UNCERTAIN** (depends on the cells used).
- Type-C for download/UART; 2x5P expansion port (RX0, TX0, G21, G15, G12, 3V3, 5V, GND).

### b) Servos

- **Type:** no model name in any source. Product page: 23.2x12.1x25.25 mm, 13 g, 6 V, 0.1 s/60 degrees, stall 2.3 kg-cm, rated 0.7 kg-cm, 350 mA, "Control method: Pulse width modification", "Digital comparator". The wiki claims "locked-rotor torque up to 5.2kg.cm" — **CONTRADICTION, not resolvable**.
- **Drive:** 50 Hz PWM via PCA9685 (`ServoCtrl.h` L4-9: `SERVOMIN 263 / SERVOMAX 463 / SERVO_FREQ 50 / SERVO_RANGE 90`). 12 of 16 channels used.
- **Position feedback: NO — evidenced by its absence from the code.** Write paths only (`pwm.setPWM(...)` in `initPosAll/middlePosAll/servoDebug/GoalPosAll`, `ServoCtrl.h` L167-201). There is no read function; "position" is always the last commanded setpoint (`CurrentPWM[]`/`GoalPWM[]`).

### c) JSON command protocol

- **Parsing:** `WAVEGO.ino` L63-77: ArduinoJson, `StaticJsonDocument<200>`, `deserializeJson(docReceive, Serial)` in `serialCtrl()`, every 25 ms (FreeRTOS task).
- **Field names: `var` (string) + `val` (int)** — there is NO "T" protocol.
- **Commands** (`WAVEGO.ino` L85-146; sender: `RPi/robot.py`):
  - `{"var":"move","val":N}` — 1=Forward, 2=TurnLeft, 3=FBStop, 4=TurnRight, 5=Backward, 6=LRStop
  - `{"var":"funcMode","val":N}` — 1=Steady(toggle), 2=StayLow, 3=Handshake, 4=Jump, 5/6/7=ActionA/B/C, 8=InitPos, 9=MiddlePos
  - `{"var":"ges","val":N}` — 1=up, 2=down, 3=stopUD, 4=left, 5=right, 6=stopLR (incremental +/-2, limit +/-15)
  - `{"var":"light","val":0..7}`, `{"var":"buzzer","val":0|1}`
- **Single-joint control over JSON: not available** (only HTTP `sconfig`/`sset` for calibration).
- **Status query: none.** `jsonSend()` (`{"vol":...}`) is defined but **never called anywhere** — telemetry is effectively inactive. Replies are plain-text `Serial.println`.

### d) Transports

- **UART:** 115200 baud (`WAVEGO.ino` L187; Pi: `/dev/ttyS0`). JSON is Pi->ESP32 only.
- **Wi-Fi/HTTP (ESP32):** AP mode by default (SSID "WAVESHARE Robot", password "1234567890", IP 192.168.4.1). Port 80: `GET /` (web UI from `WebPage.h`) and `GET /control?var=<name>&val=<n>&cmd=<n>`; port 81: `GET /stream` (MJPEG). **No WebSocket, no JSON over HTTP.** `/control` additionally understands `framesize`, `sconfig` (servo debug) and `sset` (save calibration).
- **WebSocket/Flask (Pi demo only):** `RPi/webServer.py` WebSocket port 8888 (login "admin:123456"), plain-text commands (`forward`, `jump`, ...); Flask port 5000, `/video_feed` MJPEG; Vue build in `RPi/dist/`.
- **ESP-NOW: not present. Bluetooth: not used.**

### e) Task-file mechanism

- **Not present.** No SPIFFS/LittleFS/FFat; the only persistent storage is NVS preferences (namespace "ServoConfig", keys "PWM0"-"PWM15") for servo mid-positions. Custom actions are compiled in as C++ functions (`functionActionA/B/C`).

### f) IMU

- **ICM20948** (9-axis), I2C 0x68, library `ICM20948_WE`. Only acceleration is used (+/-2g, DLPF_6, `autoOffsets()`); gyro and magnetometer are unused.
- **Self-stabilisation:** `funcMode==1` "Steady": a P controller `BALANCE_P = 0.00018` on ACC_X/ACC_Y, clamped to +/-21, acting on leg heights via `pitchYawRoll()`.

### g) Kinematics

- **Leg structure: a planar linkage** (not a serial leg), 3 DOF per leg: 2 coaxial servos plus 1 "wiggle" servo (swings the leg plane sideways).
- **Linkage constants** (`ServoCtrl.h` L28-56, mm): `Linkage_S=12.2` (servo spacing), `Linkage_A=40.0`, `Linkage_B=40.0`, `Linkage_C=39.8153` (thigh), `Linkage_D=31.7750` (shank), `Linkage_E=30.8076` (foot), `Linkage_W=19.15` (wiggle servo to leg plane).
- **Three-stage IK** (`singleLegCtrl()`, L466-538): `wigglePlaneIK` -> `singleLegPlaneIK` -> `simpleLinkageIK`. Angle to PWM: `pwm = round((463-263)*angle/90)*ServoDirection[n] + ServoMiddlePWM[n]` (about 2.22 counts/degree).
- **Conventions:** `singleLegCtrl(LegNum, x, y, z)`: x = forward/back, y = height (positive downwards, standing 95), z = lateral; x, y, z > 0. Legs: 1=FL, 2=RL, 3=FR, 4=RR. PCA9685 channels: leg 1: 8/9/10; leg 2: 14/15/13; leg 3: 7/6/5; leg 4: 1/0/2.
- **Limits:** no explicit joint-angle limits — only workspace clamps: height 75-110 mm, lateral +/-30 mm, gestures +/-15, balance +/-21. PWM window 263-463 counts.
- **Gaits:** `simpleGait` (diagonal, default) and `triangularGait` (with mass shifting, `WALK_MASS_ADJUST=21`); interpolation via `linearCtrl`/`besselCtrl`.
- **Calibration:** assembly mode by bridging G12 to 3V3 -> servos go to initPos (300); fine adjustment through the web UI (`sconfig`, +/-1 count), saved with `sset` -> NVS.

### h) Licence

- **MIT License, Copyright (c) 2022 waveshare.**

### Not firmly established (Standard)

1. The exact ESP32 module (WROVER is an inference).
2. OLED size 0.96 inch vs 0.91 inch (firmware: 128x32).
3. Servo torque 2.3 vs 5.2 kg-cm.
4. The "5200mAh" battery figure.
5. The servo model name (only "Servo pack").

---

## Report 2: WAVEGO Pro (and how it differs from the Standard)

### a) Differences, Pro vs Standard

| Aspect | WAVEGO (Standard) | WAVEGO Pro |
|---|---|---|
| Servos | Analogue PWM via PCA9685, no feedback | **Bus servos** (SC series, "real-time feedback on position, speed, and input voltage") |
| Sub-controller | ESP32 **with camera** (OV2640) | ESP32-D0WDQ6-V3 + CP2102, **no ESP32 camera** |
| Host | Raspberry Pi 4B | Raspberry Pi 4B **or 5** (PI5 KIT) |
| Camera | BASIC: on the ESP32; kits: 5MP Pi camera | BASIC: none; kits: "RPi Camera (G)" 5MP 160 degrees (CSI) |
| Protocol | `{"var":...,"val":...}` over UART; HTTP `var/val/cmd` | **"T" JSON**, ESP-NOW, mission files |
| Pi software | Flask + custom WebSocket (port 8888) | `ugv_rpi`: Flask-SocketIO + **WebRTC (aiortc)**, JupyterLab |
| Licence/maturity | MIT, frozen since 2022 | GPL-3.0, active in 2025, 1 maintainer |

Mechanics and kinematics are described identically for both variants (12 DOF, same linkage constants in the code).

### b) Servos (Pro)

- **Type:** bus servos; the firmware uses the class `SCSCL` (library `workloads/SCServo@^1.0.1`); the board carries an "SC09 bus servo interface". The SC09 specification matches exactly (2.3 kg-cm @6V, 0.1 s/60 degrees, 300-degree range, position 0-1023, resolution 0.293 degrees). **The "SC09" identification is indirect.**
- **Drive:** half-duplex UART, `Serial1.begin(1000000, SERIAL_8N1, RX=18, TX=19)`. Commands: `WritePos`, `RegWritePos`+`RegWriteAction`, `SyncWritePos`, `EnableTorque(254,0)` (broadcast).
- **Position feedback: YES, evidenced in the code.** `BodyCtrl::getServoFeedback()` reads `sc.ReadPos(jointID[i])` for all 12 servos; JSON `{"T":106}` -> `{"T":-106,"fb":[...12 values...]}`. Calibration `{"T":107}` (current position = zero) builds on it.
- Load/speed/voltage/temperature are available through the library API (`FeedBack`, `ReadSpeed/ReadLoad/ReadVoltage/ReadTemper`) but are **not implemented by the Pro firmware** (only `ReadPos`). Whether a temperature sensor exists at all is uncertain.

### c) JSON command protocol (Pro)

The central field is **`"T"`** (int); parsing via ArduinoJson 7, `jsonCmdReceiveHandler` in `main.cpp`; definitions in `src/Config.h` (the wiki calls it "json_cmd.h").

- **Motion/body:** `{"T":111,"FB":1,"LR":1}` (-1..+1), `{"T":112,"func":N}` (1=stayLow, 2=handShake, 3=jump, 4=steady on, 5=steady off), `{"T":110}` stand, `{"T":113,"leg":1,"x":16,"y":90,"z":25}` single-leg IK, `{"T":114,"h":95}` height, `{"T":115,"delay":5,"iterate":0.02}` interpolation, `{"T":116,...}` gait parameters, `{"T":108,"joint":1,"angle":45}` / `{"T":109,"joint":1,"rad":0.785}` single joint, `{"T":1,"L":0,"R":0}` UGV compatibility, `{"T":133,"X":0,"Y":0}` pan/tilt emulation.
- **Servo/calibration:** `{"T":101}` centre (511), `{"T":102}` torque release, `{"T":103,"id":21,"goal":511,"time":0,"spd":0}` single servo, `{"T":104}` / `{"T":105,"set":[...]}` read/set zero, `{"T":106}` current positions, `{"T":107}` current = zero.
- **Peripherals:** `{"T":201,"set":[...]}` RGB, `{"T":202..205}` OLED, `{"T":206,"freq":1000,"duration":10}` buzzer, `{"T":207}` -> `{"T":-207,"voltage":...}`.
- **Missions:** T:300-309, T:399 (see e). **Wi-Fi/ESP-NOW:** T:400-403, T:410-414. **System:** `{"T":600}` reboot, `{"T":601}` erase NVS.
- **Conventions:** replies carry a negative T; an unsolicited **heartbeat every 5 s**: `{"T":1001,"L":0,"R":0,"r":0,"p":0,"v":<voltage>,"pan":0,"tilt":0}`.

### d) Transports (Pro)

- **UART/USB:** 115200 baud, newline-terminated JSON lines. Type-C (CP2102) or Pi GPIO UART (Pi 5: `/dev/ttyAMA0`, Pi 4: `/dev/serial0`).
- **HTTP (ESP32):** port 80, **`GET /js?json=<JSON>`**, the response is the feedback JSON. AP: SSID "WAVEGO", password "12345678", IP 192.168.4.1; AP+STA by default.
- **No WebSocket on the ESP32.** On the Pi side: Flask-SocketIO (namespaces `/json`, `/ctrl`) plus WebRTC video, port 5000; JupyterLab on 8888.
- **ESP-NOW:** initialised at boot; received packets (max 250 B) are fed as JSON into the same command handler. `{"T":410,"longrange":0/1}`, T:411-414 (mode/MAC/send/peer).
- **Bluetooth:** not used.

### e) Task-file / mission mechanism (Pro)

- **LittleFS in the ESP32 flash**, files named `/<name>.mission`. Format: line 1 is intro text, then **one JSON command per line**.
- Commands: `{"T":300}` scan, `{"T":301,"name":...,"intro":...}` create, `{"T":302}` show, `{"T":303,...,"json":"..."}` append, `{"T":304/305,...,"step":N,...}` insert/replace, `{"T":306}` delete step, `{"T":307}` run step, `{"T":308,"name":...,"interval":1000,"loop":1}` run (aborted by incoming serial data), `{"T":309}` delete, `{"T":399}` format.
- **Boot mission:** `boot.mission` is created and executed automatically; it persists the Wi-Fi configuration (T:400) and the servo zero positions (T:105).

### f) IMU (Pro)

- ICM20948 @0x68 (SDA=32, SCL=33, 400 kHz), library `wollewald/ICM20948_WE@^1.2.5`. Only ACC X/Y are used; gyro and magnetometer are never read; a SimpleKalmanFilter is instantiated but commented out.
- `steadyMode` (T:112 func 4/5): P controller `BALANCE_P = 0.72` on pitch/roll, clamped to +/-21, via `pitchYawRoll()` -> leg IK.
- INA219 @0x42 for battery voltage/current (T:207).

### g) Kinematics (Pro)

- Linkage constants identical to the Standard (`BodyCtrl.cpp` L54-86): `linkage_w=19.15`, `linkage_s=12.2`, `linkage_a=40.0`, `linkage_b=40.0`, `linkage_c=39.8153`, `linkage_d=31.7750`, `linkage_e=30.8076` (mm).
- Three-stage IK in `singleLegCtrl(LegNum, x, y, z)`; ASCII geometry diagrams as code comments.
- Legs: 1=FL, 2=RL, 3=FR, 4=RR; the rear legs use a mirrored x. Default standing height 95.
- Clamps: height 75-110, lateral +/-30, balance +/-21. Gait defaults: `WALK_LIFT 9`, `WALK_RANGE 40`, `WALK_ACC 5`, `WALK_EXTENDED_X 16`, `WALK_EXTENDED_Z 25`, `WALK_MASS_ADJUST 21` — changeable at runtime via T:116. `STEP_DELAY 5 ms` / `STEP_ITERATE 0.02` (T:115).
- **Angle to servo:** 0-300 degrees -> 0-1024 ticks (`map(angleW,0,300,0,1024)`), `ServoDirection[12]`, `ServoMiddlePWM[12]` (default 511). **Servo IDs:** `{53,52,51, 41,42,43, 23,22,21, 31,32,33}` — the tens digit is the leg.
- **Calibration:** "assembly mode" by pulling a hardware pin to 3.3 V (code: `DEBUG_PIN 12`; the wiki contradicts itself, G15 vs G12), then T:102/103 to position freely, T:104-107 to set zero; persisted through the boot mission.

### h) Licence / maturity (Pro)

- **GPL-3.0** ("Copyright (C) 2024 Waveshare"). Repository created 2025-01-11, last push 2025-07-29, 25 commits, **one contributor**, 20 stars. A young, actively maintained one-person project.

### Not firmly established (Pro)

1. The servo model "SC09" is only indirectly evidenced (interface name + identical specifications + the `SCSCL` class).
2. Servo temperature feedback (the API offers `ReadTemper`, but the spec table does not list temperature).
3. Weight and dimensions of the Pro (from a graphic only).
4. ESP-NOW "host-sub control mode": the code is there, but no concrete host example was checked.
5. `ugv_rpi` commands (T:900 and others) come from Waveshare's shared UGV codebase and collide numerically with mission commands — whether they work on the WAVEGO Pro is unverified.
6. Assembly-mode pin: wiki says G15, code uses GPIO12.
7. The OLED size is stated inconsistently (0.96 inch vs 0.91 inch; firmware: SSD1306).
