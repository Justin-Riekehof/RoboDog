# Phase 0 — Quellenrecherche WAVEGO / WAVEGO Pro (Stand: 2026-08-10)

Rohdaten-Reports zweier Recherche-Läufe über die offiziellen Primärquellen.
Jede hier zitierte Protokoll-/Firmware-Eigenschaft gilt als **Annahme, bis am
realen Gerät verifiziert** — überprüfbare Punkte werden in `ASSUMPTIONS.md`
(Phase 2) konsolidiert.

Wichtigste Quellen:

- Standard: <https://github.com/waveshare/WAVEGO> (Achtung: NICHT `waveshareteam/`), MIT-Lizenz, letzter Commit 2022
- Pro: <https://github.com/waveshareteam/WAVEGO_Pro>, GPL-3.0, aktiv (letzter Push 2025-07-29, ein Maintainer)
- Wiki: <https://www.waveshare.com/wiki/WAVEGO>, <https://www.waveshare.com/wiki/WAVEGO:_API>, <https://www.waveshare.com/wiki/WAVEGO_Pro>, <https://www.waveshare.com/wiki/SC09_Servo>
- Produktseiten: <https://www.waveshare.com/wavego.htm>, <https://www.waveshare.com/wavego-pro.htm>

---

## Report 1: WAVEGO (Standard, 12-DOF)

### a) Hardware-Architektur

- **Sub-Controller: ESP32.** Board-Target „ESP32 Dev Module“, 240 MHz, 4 MB Flash, PSRAM „Enabled“ (Wiki „Upload example to WAVEGO“). Produktseite: „Xtensa LX6 dual-core @240MHz, SRAM: 520KB+8MB, Flash: 448KB+4MB“. **UNSICHER:** exaktes Modul (WROVER?) nirgends benannt; 8 MB PSRAM ist Inferenz.
- **Host-Controller (optional): Raspberry Pi 4B, 4 GB** („WAVEGO PI4 KIT“). Rollenteilung laut Wiki: ESP32 macht „connecting rod inverse solving and gait generation“, Pi „high-level decision operating“ (OpenCV: Face/Color/Motion).
- **Kameras:** ESP32-seitig OV2640 2MP (`app_httpd.cpp` Z. 36–53, JPEG, `FRAMESIZE_QVGA`, XCLK 20 MHz). Pi-seitig (EX/PI4 KIT): 5MP 160°-Weitwinkel.
- **Display:** SSD1306-OLED, I2C 0x3C, Firmware konfiguriert **128×32** (`InitConfig.h` Z. 120–125). **WIDERSPRUCH:** Wiki „0.96 inch“ vs. Packliste „0.91 inch“ (0,91″ passt zu 128×32).
- **Peripherie (`InitConfig.h`):** INA219 Strom-/Spannungssensor @0x42 (Shunt 0,01 Ω), ICM20948 IMU @0x68, 2× WS2812 (GPIO 26), Buzzer (GPIO 21), PCA9685 @0x40, I2C: SDA=GPIO32 / SCL=GPIO33.
- **Strom:** 2× 18650 in Serie (nicht enthalten), 7–8,4 V, Laden im Betrieb möglich, Schutzschaltung, 5-V-Ausgang für den Pi. Wiki-Angabe „5200mAh“ **UNSICHER** (zellenabhängig).
- Typ-C für Download/UART; 2×5P-Erweiterungsport (RX0, TX0, G21, G15, G12, 3V3, 5V, GND).

### b) Servos

- **Typ:** Kein Modellname in den Quellen. Produktseite: 23,2×12,1×25,25 mm, 13 g, 6 V, 0,1 s/60°, Stall 2,3 kg·cm, Rated 0,7 kg·cm, 350 mA, „Control method: Pulse width modification“, „Digital comparator“. Wiki nennt „locked-rotor torque up to 5.2kg.cm“ — **WIDERSPRUCH, nicht auflösbar**.
- **Ansteuerung:** 50-Hz-PWM über PCA9685 (`ServoCtrl.h` Z. 4–9: `SERVOMIN 263 / SERVOMAX 463 / SERVO_FREQ 50 / SERVO_RANGE 90`). 12 von 16 Kanälen belegt.
- **Positions-Feedback: NEIN — belegt durch Abwesenheit im Code.** Nur Schreibpfade (`pwm.setPWM(...)` in `initPosAll/middlePosAll/servoDebug/GoalPosAll`, `ServoCtrl.h` Z. 167–201). Keine Lesefunktion; „Position“ ist immer der zuletzt kommandierte Sollwert (`CurrentPWM[]`/`GoalPWM[]`).

### c) JSON-Kommandoprotokoll

- **Parsing:** `WAVEGO.ino` Z. 63–77: ArduinoJson, `StaticJsonDocument<200>`, `deserializeJson(docReceive, Serial)` in `serialCtrl()`, alle 25 ms (FreeRTOS-Task).
- **Feldnamen: `var` (String) + `val` (Int)** — KEIN „T“-Protokoll.
- **Kommandos** (`WAVEGO.ino` Z. 85–146; Sender: `RPi/robot.py`):
  - `{"var":"move","val":N}` — 1=Forward, 2=TurnLeft, 3=FBStop, 4=TurnRight, 5=Backward, 6=LRStop
  - `{"var":"funcMode","val":N}` — 1=Steady(Toggle), 2=StayLow, 3=Handshake, 4=Jump, 5/6/7=ActionA/B/C, 8=InitPos, 9=MiddlePos
  - `{"var":"ges","val":N}` — 1=up, 2=down, 3=stopUD, 4=left, 5=right, 6=stopLR (inkrementell ±2, Limit ±15)
  - `{"var":"light","val":0..7}`, `{"var":"buzzer","val":0|1}`
- **Einzelgelenk per JSON: nicht vorhanden** (nur HTTP `sconfig`/`sset` zur Kalibrierung).
- **Statusabfrage: keine.** `jsonSend()` (`{"vol":…}`) ist definiert, wird aber **nirgends aufgerufen** — Telemetrie faktisch inaktiv. Antworten sind Klartext-`Serial.println`.

### d) Transportwege

- **UART:** 115200 Baud (`WAVEGO.ino` Z. 187; Pi: `/dev/ttyS0`). JSON nur Pi→ESP32.
- **WLAN/HTTP (ESP32):** Default AP-Modus (SSID „WAVESHARE Robot“, PW „1234567890“, IP 192.168.4.1). Port 80: `GET /` (Web-UI aus `WebPage.h`) und `GET /control?var=<name>&val=<n>&cmd=<n>`; Port 81: `GET /stream` (MJPEG). **Kein WebSocket, kein JSON über HTTP.** `/control` kennt zusätzlich `framesize`, `sconfig` (Servo-Debug), `sset` (Kalibrierung speichern).
- **WebSocket/Flask (nur Pi-Demo):** `RPi/webServer.py` WebSocket Port 8888 (Login „admin:123456“), Klartext-Kommandos (`forward`, `jump`, …); Flask Port 5000, `/video_feed` MJPEG; Vue-Build in `RPi/dist/`.
- **ESP-NOW: nicht vorhanden. Bluetooth: nicht genutzt.**

### e) Task-File-Mechanismus

- **Nicht vorhanden.** Kein SPIFFS/LittleFS/FFat; einziger persistenter Speicher: NVS-Preferences (Namespace „ServoConfig“, Keys „PWM0“–„PWM15“) für Servo-Mittelstellungen. Eigene Aktionen werden als C++-Funktionen einkompiliert (`functionActionA/B/C`).

### f) IMU

- **ICM20948** (9-Achsen), I2C 0x68, Lib `ICM20948_WE`. Nur Beschleunigung genutzt (±2g, DLPF_6, `autoOffsets()`); Gyro/Magnetometer ungenutzt.
- **Selbststabilisierung:** `funcMode==1` „Steady“: P-Regler `BALANCE_P = 0.00018` auf ACC_X/ACC_Y, Clamp ±21, wirkt über `pitchYawRoll()` auf Beinhöhen.

### g) Kinematik

- **Beinaufbau: planares Koppelgetriebe** (kein serielles Bein), 3 DOF/Bein: 2 koaxiale Servos + 1 „Wiggle“-Servo (schwenkt Beinebene seitlich).
- **Linkage-Konstanten** (`ServoCtrl.h` Z. 28–56, mm): `Linkage_S=12.2` (Servoabstand), `Linkage_A=40.0`, `Linkage_B=40.0`, `Linkage_C=39.8153` (Oberschenkel), `Linkage_D=31.7750` (Unterschenkel), `Linkage_E=30.8076` (Fuß), `Linkage_W=19.15` (Wiggle-Servo↔Beinebene).
- **IK dreistufig** (`singleLegCtrl()`, Z. 466–538): `wigglePlaneIK` → `singleLegPlaneIK` → `simpleLinkageIK`. Winkel→PWM: `pwm = round((463−263)·angle/90)·ServoDirection[n] + ServoMiddlePWM[n]` (≈2,22 Counts/°).
- **Konventionen:** `singleLegCtrl(LegNum, x, y, z)`: x = vor/zurück, y = Höhe (nach unten positiv, Stand 95), z = seitlich; x,y,z > 0. Beine: 1=VL, 2=HL, 3=VR, 4=HR. PCA9685-Kanäle: Bein 1: 8/9/10; Bein 2: 14/15/13; Bein 3: 7/6/5; Bein 4: 1/0/2.
- **Grenzen:** Keine expliziten Gelenkwinkel-Limits — nur Arbeitsraum-Clamps: Höhe 75–110 mm, seitlich ±30 mm, Gesten ±15, Balance ±21. PWM-Fenster 263–463 Counts.
- **Gaits:** `simpleGait` (Diagonal, Default) und `triangularGait` (mit Schwerpunktverlagerung `WALK_MASS_ADJUST=21`); Interpolation `linearCtrl`/`besselCtrl`.
- **Kalibrierung:** Montagemodus durch Brücken G12↔3V3 → Servos auf initPos (300); Feinjustage via Web-UI (`sconfig` ±1 Count), Speichern `sset` → NVS.

### h) Lizenz

- **MIT License, Copyright (c) 2022 waveshare.**

### Nicht sicher belegt (Standard)

1. Exaktes ESP32-Modul (WROVER-Inferenz).
2. OLED-Größe 0,96″ vs. 0,91″ (Firmware: 128×32).
3. Servo-Drehmoment 2,3 vs. 5,2 kg·cm.
4. „5200mAh“-Akkuangabe.
5. Servo-Modellname (nur „Servo pack“).

---

## Report 2: WAVEGO Pro (und Unterschiede zur Standard-Variante)

### a) Unterschiede Pro vs. Standard

| Aspekt | WAVEGO (Standard) | WAVEGO Pro |
|---|---|---|
| Servos | Analog-PWM via PCA9685, kein Feedback | **Bus-Servos** (SC-Serie, „real-time feedback on position, speed, and input voltage“) |
| Sub-Controller | ESP32 **mit Kamera** (OV2640) | ESP32-D0WDQ6-V3 + CP2102, **keine ESP32-Kamera** |
| Host | Raspberry Pi 4B | Raspberry Pi 4B **oder 5** (PI5 KIT) |
| Kamera | BASIC: am ESP32; Kits: 5MP-Pi-Kamera | BASIC: keine; Kits: „RPi Camera (G)“ 5MP 160° (CSI) |
| Protokoll | `{"var":…,"val":…}` UART; HTTP `var/val/cmd` | **„T“-JSON**, ESP-NOW, Mission-Files |
| Pi-Software | Flask + eigener WebSocket (Port 8888) | `ugv_rpi`: Flask-SocketIO + **WebRTC (aiortc)**, JupyterLab |
| Lizenz/Aktivität | MIT, eingefroren seit 2022 | GPL-3.0, aktiv 2025, 1 Maintainer |

Mechanik/Kinematik beider Varianten identisch beschrieben (12 DOF, gleiche Linkage-Konstanten im Code).

### b) Servos (Pro)

- **Typ:** Bus-Servos, Firmware nutzt Klasse `SCSCL` (Lib `workloads/SCServo@^1.0.1`); Board: „SC09 bus servo interface“. SC09-Spezifikation deckt sich exakt (2,3 kg·cm@6V, 0,1 s/60°, 300°-Bereich, Position 0–1023, Auflösung 0,293°). **Zuordnung „SC09“ ist indirekt belegt.**
- **Ansteuerung:** Half-Duplex-UART, `Serial1.begin(1000000, SERIAL_8N1, RX=18, TX=19)`. Befehle: `WritePos`, `RegWritePos`+`RegWriteAction`, `SyncWritePos`, `EnableTorque(254,0)` (Broadcast).
- **Positions-Feedback: JA, im Code belegt.** `BodyCtrl::getServoFeedback()` liest `sc.ReadPos(jointID[i])` für alle 12 Servos; JSON `{"T":106}` → `{"T":-106,"fb":[…12 Werte…]}`. Kalibrierung `{"T":107}` (Ist = Null) baut darauf auf.
- Load/Speed/Voltage/Temp: per Lib-API möglich (`FeedBack`, `ReadSpeed/ReadLoad/ReadVoltage/ReadTemper`), **von der Pro-Firmware aber nicht implementiert** (nur `ReadPos`). Temperatursensor-Existenz unsicher.

### c) JSON-Kommandoprotokoll (Pro)

Zentrales Feld **`"T"`** (Int); Parsing via ArduinoJson 7, `jsonCmdReceiveHandler` in `main.cpp`; Definitionsdatei `src/Config.h` (Wiki nennt sie „json_cmd.h“).

- **Bewegung/Körper:** `{"T":111,"FB":1,"LR":1}` (−1..+1), `{"T":112,"func":N}` (1=stayLow, 2=handShake, 3=jump, 4=steady on, 5=steady off), `{"T":110}` stand, `{"T":113,"leg":1,"x":16,"y":90,"z":25}` Einzelbein-IK, `{"T":114,"h":95}` Höhe, `{"T":115,"delay":5,"iterate":0.02}` Interpolation, `{"T":116,…}` Gait-Parameter, `{"T":108,"joint":1,"angle":45}` / `{"T":109,"joint":1,"rad":0.785}` Einzelgelenk, `{"T":1,"L":0,"R":0}` UGV-Kompat, `{"T":133,"X":0,"Y":0}` Pan/Tilt-Emulation.
- **Servo/Kalibrierung:** `{"T":101}` Mitte (511), `{"T":102}` Torque release, `{"T":103,"id":21,"goal":511,"time":0,"spd":0}` Einzelservo, `{"T":104}/{"T":105,"set":[…]}` Zero lesen/setzen, `{"T":106}` Ist-Positionen, `{"T":107}` Ist=Zero.
- **Peripherie:** `{"T":201,"set":[…]}` RGB, `{"T":202..205}` OLED, `{"T":206,"freq":1000,"duration":10}` Buzzer, `{"T":207}` → `{"T":-207,"voltage":…}`.
- **Missions:** T:300–309, T:399 (s. e). **WLAN/ESP-NOW:** T:400–403, T:410–414. **System:** `{"T":600}` Reboot, `{"T":601}` NVS löschen.
- **Konventionen:** Antworten mit negativem T; **Heartbeat alle 5 s** unaufgefordert: `{"T":1001,"L":0,"R":0,"r":0,"p":0,"v":<Spannung>,"pan":0,"tilt":0}`.

### d) Transportwege (Pro)

- **UART/USB:** 115200 Baud, JSON-Zeilen mit `\n`. Type-C (CP2102) oder Pi-GPIO-UART (Pi 5: `/dev/ttyAMA0`, Pi 4: `/dev/serial0`).
- **HTTP (ESP32):** Port 80, **`GET /js?json=<JSON>`**, Antwort = Feedback-JSON. AP: SSID „WAVEGO“, PW „12345678“, IP 192.168.4.1; Default AP+STA.
- **Kein WebSocket auf dem ESP32.** Pi-Seite: Flask-SocketIO (Namespaces `/json`, `/ctrl`) + WebRTC-Video, Port 5000; JupyterLab 8888.
- **ESP-NOW:** beim Boot initialisiert; empfangene Pakete (max. 250 B) werden als JSON in denselben Kommando-Handler gespeist. `{"T":410,"longrange":0/1}`, T:411–414 (Mode/MAC/Send/Peer).
- **Bluetooth:** nicht genutzt.

### e) Task-File-/Mission-Mechanismus (Pro)

- **LittleFS im ESP32-Flash**, Dateien `/<name>.mission`. Format: Zeile 1 = Intro-Text, danach **ein JSON-Kommando pro Zeile**.
- Kommandos: `{"T":300}` scan, `{"T":301,"name":…,"intro":…}` create, `{"T":302}` anzeigen, `{"T":303,…,"json":"…"}` append, `{"T":304/305,…,"step":N,…}` insert/replace, `{"T":306}` delete step, `{"T":307}` run step, `{"T":308,"name":…,"interval":1000,"loop":1}` run (Abbruch durch eingehende Serial-Daten), `{"T":309}` delete, `{"T":399}` Format.
- **Boot-Mission:** `boot.mission` wird automatisch angelegt/ausgeführt; persistiert WLAN-Konfig (T:400) und Servo-Nullpositionen (T:105).

### f) IMU (Pro)

- ICM20948 @0x68 (SDA=32, SCL=33, 400 kHz), Lib `wollewald/ICM20948_WE@^1.2.5`. Nur ACC X/Y genutzt; Gyro/Mag ungelesen; SimpleKalmanFilter instanziiert, aber auskommentiert.
- `steadyMode` (T:112 func 4/5): P-Regler `BALANCE_P = 0.72` auf Pitch/Roll, Clamp ±21, via `pitchYawRoll()` → Bein-IK.
- INA219 @0x42 für Batteriespannung/Strom (T:207).

### g) Kinematik (Pro)

- Identische Linkage-Konstanten wie Standard (`BodyCtrl.cpp` Z. 54–86): `linkage_w=19.15`, `linkage_s=12.2`, `linkage_a=40.0`, `linkage_b=40.0`, `linkage_c=39.8153`, `linkage_d=31.7750`, `linkage_e=30.8076` (mm).
- Dreistufige IK in `singleLegCtrl(LegNum, x, y, z)`; ASCII-Geometrie-Diagramme als Code-Kommentare.
- Beine: 1=VL, 2=HL, 3=VR, 4=HR; hintere Beine mit gespiegeltem x. Standhöhe Default 95.
- Clamps: Höhe 75–110, seitlich ±30, Balance ±21. Gait-Defaults: `WALK_LIFT 9`, `WALK_RANGE 40`, `WALK_ACC 5`, `WALK_EXTENDED_X 16`, `WALK_EXTENDED_Z 25`, `WALK_MASS_ADJUST 21` — zur Laufzeit per T:116 änderbar. `STEP_DELAY 5 ms` / `STEP_ITERATE 0.02` (T:115).
- **Winkel→Servo:** 0–300° → 0–1024 Ticks (`map(angleW,0,300,0,1024)`), `ServoDirection[12]`, `ServoMiddlePWM[12]` (Default 511). **Servo-IDs:** `{53,52,51, 41,42,43, 23,22,21, 31,32,33}` — Zehnerstelle = Bein.
- **Kalibrierung:** „Assembly Mode“ per Hardware-Pin auf 3,3 V (Code: `DEBUG_PIN 12`; Wiki widersprüchlich G15/G12), dann T:102/103 frei positionieren, T:104–107 Null setzen; Persistenz via boot-Mission.

### h) Lizenz/Reifegrad (Pro)

- **GPL-3.0** („Copyright (C) 2024 Waveshare“). Repo erstellt 2025-01-11, letzter Push 2025-07-29, 25 Commits, **ein Contributor**, 20 Stars. Junges, aktiv gepflegtes Ein-Personen-Projekt.

### Nicht sicher belegt (Pro)

1. Servo-Modell „SC09“ nur indirekt (Interface-Name + identische Kenndaten + `SCSCL`-Klasse).
2. Temperatur-Feedback der Servos (API bietet `ReadTemper`, Spec-Tabelle nennt Temp nicht).
3. Gewicht/Abmessungen Pro (nur Grafik).
4. ESP-NOW „Host-Sub Control Mode“: Code vorhanden, konkretes Host-Beispiel ungeprüft.
5. `ugv_rpi`-Kommandos (T:900 u. a.) stammen aus Waveshares geteilter UGV-Codebasis und kollidieren nummernmäßig mit Mission-Kommandos — Funktionsfähigkeit am WAVEGO Pro unverifiziert.
6. Assembly-Mode-Pin: Wiki G15 vs. G12; Code nutzt GPIO12.
7. OLED-Größenangabe widersprüchlich (0,96″ vs. 0,91″; Firmware: SSD1306).
