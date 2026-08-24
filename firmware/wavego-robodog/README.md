# wavego-robodog — firmware fork (M4, in progress)

A fork of the Waveshare WAVEGO ESP32 sketch that adds what the stock firmware
cannot do. This directory is **editable** — unlike [vendor/](../../vendor/),
which stays a pristine reference copy so the Python kinematics port has
something unchanging to be line-faithful to.

- Upstream: <https://github.com/waveshare/WAVEGO>, path `Arduino/WAVEGO/`
- License: MIT, Copyright (c) 2022 waveshare — see [LICENSE](LICENSE); the
  additions below are under the same terms
- Forked: 2026-08-22 from the upstream `main` (3 commits, unchanged since 2022),
  verified byte-identical to `vendor/wavego-firmware/` at that point

## Status

> **Flashed and measured on the robot, 2026-08-22.** Built with PlatformIO Core
> 6.1.19 against the same `common.ini` as the baseline: 850,144 bytes of flash
> and 53,324 of RAM, **+384 B / +16 B** over the untouched upstream sketch.
> Those were the watchdog's numbers alone. With the camera controls, the pose
> command and the IMU on top, both sketches rebuilt on 2026-08-24:
>
> | | baseline | fork | delta |
> |---|---|---|---|
> | Flash | 849,632 | 863,252 | **+13,620 B** (1.0% of the slot) |
> | RAM | 53,308 | 55,460 | **+2,152 B** |
>
> Most of that RAM is the IMU ring: 64 samples of 28 bytes is 1,792 B, chosen so
> a host polling ten times a second has more than a second of slack.
>
> **The IMU code has been compiled but never run.** Everything below the camera
> section is untested on the device.
>
> The watchdog was verified on the device: armed at 2000 ms, robot walking, fed
> with `ping` every 400 ms for 3.5 s without tripping; then silence produced
> `WATCHDOG: link lost, stopping and crouching` and `stayLow` at the deadline.
> Everything else behaves as the baseline does, because the watchdog stays off
> until a host arms it. This closes ASSUMPTIONS B9/D10 for robots running this
> firmware.

## Building it anywhere but where it was written

Two things stopped a fresh machine from building this at all, both found the
first time anyone tried on Linux (2026-08-24) and both fixed:

* **`#include <arduino.h>`** — upstream's spelling, and the Arduino core ships
  the header capitalised. That resolves only on a case-insensitive filesystem,
  so the sketch had never been built anywhere but Windows or macOS; on Linux it
  failed at line 1, and so did the baseline it is supposed to be compared
  against. Fixed **without touching either source file**: `firmware/compat/`
  holds a one-line `arduino.h` that includes the real one, and `common.ini`
  puts it on the include path for both sketches. Correcting the include instead
  was the obvious move and the wrong one — the fork's own tests refused it,
  because the baseline must stay byte-identical to `vendor/` to be worth
  calling a baseline and the fork may only ever add lines.
* **`wollewald/ICM20948_WE@1.2.1` had vanished from the registry.** PlatformIO
  keeps only a library's last five versions, and by 2026-08-24 it served 1.2.5
  upwards. The 2026-08-22 build had succeeded on a machine that still held the
  package in its cache — a fresh clone could not build at all, which is exactly
  what pinning was meant to prevent. `common.ini` now takes that library from
  its git tag instead, which is not pruned. `INA219_WE` is three versions from
  the same edge and is the next to move.

Neither was our code. Both are the kind of thing that stays invisible until
somebody builds on a machine that was not there when it was written.

## What is changed

**Two things: an on-device link watchdog, and a way to see the camera.**
Everything else is upstream, byte for byte — `ServoCtrl.h` in particular is
untouched, so the Python port that mirrors it (ASSUMPTIONS section C) keeps its
premise. The fork only ever *adds* lines; a test enforces both properties.

| Command | Transport | What it does |
| --- | --- | --- |
| `watchdog` | serial + HTTP | arm the link watchdog, value in ms, `0` = off |
| `ping` | serial + HTTP | keep-alive that changes nothing else |
| `snap` | serial | grab one camera frame; `val=1` also dumps it as base64 |
| `leg` | serial | stage one foot target: `{"var":"leg","val":1,"x":16,"y":95,"z":25}` |
| `apply` | serial | move every staged leg at once |
| `pose` | HTTP | a whole pose in one request: `/control?var=pose&val=0&cmd=0&l1x=16&l1y=95&l1z=25&l2x=-16&...&l4z=25` |

`pose` is the one that matters for playback. Measured on the robot's own access
point, 2026-08-22: **78-94 ms per pose**, about twelve a second. That is not the
50 Hz a motion routine interpolates at, but it is a long way from the "static
poses only" the stock firmware's trim path is stuck with (ASSUMPTIONS D8). A
missing value gets HTTP 500 and moves nothing -- a half-parsed pose must not
produce half a movement.

### The one rule this firmware taught us the hard way

**Nothing outside `loop()` may touch I2C.** The first version of `apply` called
`GoalPosAll()` directly from the serial task, which is the obvious thing to do
and crashes the robot:

```
Guru Meditation Error: Core 0 panic'ed (IntegerDivideByZero)
GoalPosAll() -> Adafruit_PWMServoDriver::setPWM -> TwoWire::endTransmission
             -> i2cWrite -> i2cProcQueue (esp32-hal-i2c.c:1287)
```

`robotCtrl()` already writes the PCA9685 from `loop()` on every pass; a second
task doing the same corrupts the ESP32 core 1.0.x I2C driver. So commands
**publish values and return** -- the main loop stays the only writer to the bus.
Anything added later that wants to move a servo has to work the same way.

The second rule follows from the first: `leg` stages into a shadow copy of
`GoalPWM[]`, not into `GoalPWM[]` itself. The loop re-applies that array
continuously, so writing legs into it one at a time would move them one at a
time. `apply` copies all sixteen entries across in one go, and the next loop
pass sends the whole pose -- which is what "twelve servos at once" means.

### `snap`, and why it exists

The camera had been written off as broken since 2026-08-11 (ASSUMPTIONS F2:
black frame, cause unidentified). It is not. `snap` proved that over USB, with
no Wi-Fi in the way:

```
{"var":"snap","val":0}   ->  ROBODOG: snap len=3342 w=320 h=240 fmt=3
{"var":"snap","val":1}   ->  ... base64 of a real, legible picture
```

Upstream also commented out the camera-init error message *and* its `return`,
so a failed init was completely silent; the fork prints
`ROBODOG: camera init ok` or the error code. That one line is what turned "the camera is
broken" into a working camera: the live MJPEG stream on port 81 was confirmed
the same day. `snap` remains useful anyway, because it needs no Wi-Fi at all --
on this setup, joining the robot's access point costs the operator their
internet connection.

A frame is ~3 KB, so at 115200 baud one picture takes about a second. It is a
diagnostic, not a stream — but unlike the stream, it does not cost the operator
their internet connection, which is what made it usable at all.

The fork only ever *adds* lines; a test in `tests/test_firmware_fork.py`
enforces that, and that the kinematics header stays identical to the vendored
reference.

### Why

The stock protocol **latches**: one `move` walks the robot until a stop arrives
(ASSUMPTIONS B3/D4). If the link dies in between, the robot keeps walking and
nothing can reach it — verified on the device on 2026-08-11, and the reason
every piece of tooling here warns about stands and power switches
(ASSUMPTIONS B9/D10). No host-side change can fix that. The robot has to be
able to stop itself.

### How

Two new commands, on **both** transports (serial JSON and HTTP `/control`):

| Command | Value | Meaning |
| --- | --- | --- |
| `watchdog` | milliseconds, `0` = off | Arm or disarm the watchdog |
| `ping` | ignored | Keep-alive that changes nothing else |

Any accepted command feeds the watchdog, so a host that is actively driving
never needs `ping`; it exists for the long silences between moves — "walk
forward for ten seconds" is one command and then nine seconds of nothing.

When the deadline passes **and the robot is actually moving**, it sets
`moveFB = moveLR = 0` and `funcMode = 2` (stayLow): stop, then down, low and
stable. A parked robot, or one being trimmed in `debugMode`, is never disturbed.

### Off by default, on purpose

`ROBODOG_WATCHDOG_MS` starts at 0. The vendor's own web UI sends nothing while
the robot walks, so an always-on watchdog would break it — flash this and the
stock page still behaves exactly as before. A host that wants the safety net
asks for it and then accepts the obligation to keep talking.

## What the host side still needs

Not implemented yet in `robodog`:

1. `HttpBackend.connect()` sends `watchdog=<ms>` (a second or two).
2. Something feeds it while a drive is latched — the player already ticks at
   50 Hz and the teach UI has a ticker; both would send `ping` on a slower
   cadence, or simply re-send the current `move`.
3. `Capability`/backend reporting so tooling can tell a watchdog-capable robot
   from a stock one, and so the D10 warnings can be softened only where they
   are actually earned.

Until that exists, flashing this firmware changes nothing observable: the
watchdog stays off because nobody arms it.

## Building

Step-by-step, including the baseline flash that must come first:
[docs/firmware.md](../../docs/firmware.md).


Arduino IDE, board **ESP32 Dev Module**, board-manager URL
`https://dl.espressif.com/dl/package_esp32_index.json`. The sketch needs
ArduinoJson, ICM20948_WE, INA219_WE, Adafruit_SSD1306 (+ Adafruit GFX) and
Adafruit_NeoPixel; Waveshare ships a `Libraries` archive for the versions it was
written against.

**Do not enable "Erase All Flash Before Sketch Upload"** — the servo middle
positions live in NVS (`PreferencesConfig.h`), and an erase throws away the
robot's own calibration.

Powering on is a motion event: `setup()` calls `standMassCenter(); GoalPosAll()`
and the robot stands up by itself about a second later (ASSUMPTIONS D7). Stand
first, then flash.
