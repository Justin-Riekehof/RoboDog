# Flashing the ESP32 (baseline and fork)

> **Status: both sketches were built and flashed on 2026-08-22** (PlatformIO
> Core 6.1.19), and the fork's watchdog was measured on the robot. Every setting
> and step below is verified unless it says otherwise.

Two things about the serial console that cost time if you do not know them:

- **The firmware prints nothing at boot.** After the ROM bootloader's log there
  is silence. A working robot looks identical to a dead one.
- **`robotThreadings` waits `delay(3000)` before it reads serial at all**, and
  opening the port resets the ESP32 (DTR/RTS). So a command sent in the first
  three seconds is lost — and it can take the next ones with it. Wait ~6 s after
  opening the port before expecting an answer.

The one command that answers without moving anything is `{"var":"move","val":3}`
(FBStop, which it already is) — the liveness check to reach for.

Two sketches live side by side:

| Path | What it is |
| --- | --- |
| [firmware/wavego-upstream/](../firmware/wavego-upstream/) | the pristine Waveshare sketch — **flash this first** |
| [firmware/wavego-robodog/](../firmware/wavego-robodog/) | the fork; adds the link watchdog, nothing else |

The baseline exists so that a bad day ends with a robot that behaves like the
manual says. Flash it once *before* the fork, even though it changes nothing:
that is how you learn whether your toolchain works, at a moment when a failure
means nothing.

## What you need

A USB-C **data** cable (a charge-only cable enumerates nothing) and the
**Silicon Labs CP210x VCP driver** — the board's USB-UART bridge is a CP2102
(`VID_10C4&PID_EA60`), and without the driver Windows enumerates it with
problem code 28 and no COM port appears. Then one of the two toolchains below. **PlatformIO is the recommended one** — not for comfort,
but because it puts every setting that matters into a versioned file instead of
into somebody's memory of which menu they clicked.

### PlatformIO (VS Code) — recommended

Install the **PlatformIO IDE** extension, then open either sketch folder; each
has its own `platformio.ini`, and both extend the shared
[firmware/common.ini](../firmware/common.ini). From the repo root:

```console
pio run -d firmware/wavego-upstream              # build the baseline
pio run -d firmware/wavego-upstream -t upload    # ... and flash it
pio device monitor -b 115200
```

The shared file pins what the source demands and marks what is still unknown:

| Setting | Value | Why |
| --- | --- | --- |
| `platform` | `espressif32@3.5.0` (core 1.0.6) | `app_httpd.cpp:29` includes `dl_lib_matrix3d.h`, from the old camera driver — removed in core 2.x/3.x |
| ArduinoJson | `^6.21.0` | `WAVEGO.ino:65` uses `StaticJsonDocument`, the v6 API |
| ICM20948_WE | `1.2.1` **exactly** | 1.2.2 changed the getters to an out-parameter form; `InitConfig.h:74-76` calls the old `xyzFloat getAccRawValues()` |
| INA219_WE | `~1.3.8` | 1.4.0 prefixed every enum (`PG_320` → `INA219_PG_320`); `InitConfig.h:103-104` uses the old names |
| partitions | the default (no line) | 849,760 bytes fits the default 1.25 MB app slot at 65%. Both tables put `nvs` at 0x9000/0x5000, so the stored servo middles survive either way |

**Three of those five were found the hard way**, by a build that failed with
errors reading like a broken sketch. The 2022 code is fine; its libraries moved.
Unpinned dependencies are the single most likely reason this stops building on
some future machine.

Measured on 2026-08-22:

| | Flash | RAM |
| --- | --- | --- |
| baseline | 849,760 B (64.8%) | 53,308 B (16.3%) |
| fork | 850,144 B (64.9%) | 53,324 B (16.3%) |
| the watchdog costs | **+384 B** | **+16 B** |

Both sketches deliberately share `common.ini`. "The fork behaves like the
baseline" is only evidence if the two were built the same way; a test enforces
that neither project overrides it locally — including via
`pio pkg install --library`, which writes into the per-sketch file and inlines
the shared section with it.

### Arduino IDE — the alternative

Board-manager URL `https://dl.espressif.com/dl/package_esp32_index.json`, board
**ESP32 Dev Module**, and the same core-1.0.x caveat as above (a fresh IDE
installs 3.x and fails on `dl_lib_matrix3d.h`). Six libraries, read off the
`#include <...>` lines: `ArduinoJson` (v6), `ICM20948_WE`, `INA219_WE`,
`Adafruit_NeoPixel`, `Adafruit_PWMServoDriver`, `Adafruit_SSD1306` (pulls in
`Adafruit_GFX`). The rest — `Preferences`, `WiFi`, `Wire`, `esp32-hal-ledc` —
come with the core. Waveshare also ships a `Libraries` archive with the versions
the firmware was written against; if a library version fights you, prefer
theirs.

## Phase 1 — the baseline

1. **Robot on a stand, legs free.** Powering on is a motion event: `setup()`
   calls `standMassCenter(); GoalPosAll()` and the robot stands up by itself
   about a second later (ASSUMPTIONS D7). Every flash reboots it, so this
   happens on every upload.
2. Open the sketch: `firmware/wavego-upstream/` in PlatformIO, or
   `firmware/wavego-upstream/WAVEGO.ino` in the Arduino IDE.
3. Arduino IDE only: **Tools → Board → ESP32 Arduino → ESP32 Dev Module**.
   PlatformIO reads the board from `common.ini`.
4. Connect USB. PlatformIO finds the port by itself (`pio device list` shows
   it); in the Arduino IDE pick it under **Tools → Port** — note which ports
   existed *before* plugging in, that is how you tell which one is the robot.
5. **Leave "Erase All Flash Before Sketch Upload" off.** The servo middle
   positions live in NVS (`PreferencesConfig.h`); erasing throws away the
   robot's own calibration, which nothing can read back afterwards
   (ASSUMPTIONS D8).
6. **Build before uploading** — `pio run -d firmware/wavego-upstream`, or
   Verify (✓) in the IDE. Compiling is free and is where a wrong core version
   announces itself. If it complains that the sketch is too big,
   `board_build.partitions` in `common.ini` is the knob; write down whichever
   value worked.
7. **Upload.**
8. **Tools → Serial Monitor, 115200 baud.** The firmware prints on boot. This
   is also where `sconfig` prints `position:` and `MID:` — the stored servo
   middle, the one number the calibration table has to assume (ASSUMPTIONS D8).
9. **Check it from the outside**, on the robot's own access point:

   ```console
   uv run robodog info --backend http
   uv run robodog bringup
   ```

   If those behave as they did before the flash, the toolchain is proven.

## Phase 2 — the fork

10. Open `firmware/wavego-robodog/WAVEGO.ino`, same board settings, upload.
11. **Nothing should change.** The watchdog is off until a host asks for it, so
    the robot, the vendor web UI and every tool here behave exactly as before.
    If anything is different, that is a bug in the fork, not a feature.
12. **Then test the watchdog deliberately** — this is its acceptance:

    ```console
    # arm it: stop by yourself after 1.5 s of silence
    curl "http://192.168.4.1/control?var=watchdog&val=1500&cmd=0"
    # walk
    curl "http://192.168.4.1/control?var=move&val=1&cmd=0"
    ```

    Now cut the link — switch the PC's Wi-Fi away, or walk out of range. The
    robot must **stop by itself within about 1.5 s and crouch**. On the stock
    firmware it walks until the battery dies (ASSUMPTIONS B9/D10, verified on
    the device 2026-08-11), so this is a difference you can see from across the
    room.

    Do it with the robot on a stand. It is a test of what happens when control
    is lost, which is not a state to first meet on the floor.

13. If it works, ASSUMPTIONS B9/D10 gain a line, and the D10 warnings in the
    tooling can be softened — but only for a robot running this firmware, which
    is why the host side needs to report the capability rather than assume it.

## The camera

It works, and it needs no firmware change: `http://192.168.4.1:81/stream` is
live MJPEG (320x240), confirmed 2026-08-22. On the vendor page at
`http://192.168.4.1` the image stays black until you press **Start** -- that
button is what sets the `<img>` source, and an unpressed button looks exactly
like a dead camera.

The fork adds `{"var":"snap","val":1}` over serial, which returns one frame as
base64. Slower than the stream by far, but it works with no Wi-Fi involved --
worth knowing on a setup where joining the robot's access point means losing
the internet. Note `fb_count = 1`: one frame buffer, so a browser holding the
stream blocks `snap` and the other way round.

## Going back

Flash `firmware/wavego-upstream/WAVEGO.ino` again. There is no state in the
fork that survives it.
