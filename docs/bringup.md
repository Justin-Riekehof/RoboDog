# Wi-Fi bring-up procedure (stock firmware)

First contact with the real robot, without flashing anything. Everything here
goes through the firmware's own HTTP endpoint on its access point.

## Read this first

The stock firmware has **no link watchdog**. Whatever it was last told to do, it
keeps doing — and over Wi-Fi a dropped connection means no stop command can
reach it any more (ASSUMPTIONS B9/D10). There is no software fix for this; it is
what milestone M4 (custom firmware) exists to close.

Therefore, for every session:

- Put the robot **on a stand** with the legs hanging free, or keep a hand on the
  power switch.
- Expect the robot to **stand up by itself about a second after power-on** — the
  firmware commands the stand pose before Wi-Fi even starts (ASSUMPTIONS D7).
- Charged batteries. A sagging pack makes servos behave erratically and will
  send you chasing phantom protocol bugs.

## What Wi-Fi can and cannot do

The HTTP handler accepts exactly five variables: `move`, `funcMode`, `sconfig`,
`sset`, `framesize` (ASSUMPTIONS D2). So over Wi-Fi you get **locomotion,
function-mode animations and servo trim** — and nothing else. Gestures, the RGB
LED and the buzzer exist only on the serial JSON path, which is why
`patrol-demo.yaml` will refuse to run over Wi-Fi and `patrol-wifi.yaml` exists.

There is **no readback of any kind**: every response is an empty HTTP 200. Any
state the tooling shows you for this transport is a model, and it says so
(`is_estimated`).

## Steps

1. **Power on** the robot and let it finish booting (it stands up, LEDs settle).
2. **Join its access point** — SSID `WAVESHARE Robot`, password `1234567890`.
   The robot is then at `192.168.4.1`. Note that Windows may warn about "no
   internet"; that is expected. If you use a VPN or a system proxy, the tooling
   already bypasses proxies for this connection.
3. **Check connectivity** without moving anything:

   ```console
   uv run robodog info --backend http
   ```

   This fetches the index page and sends two stop commands, then prints the
   capability set. If this fails, nothing else will work.

4. **Run the guided bring-up:**

   ```console
   uv run robodog bringup
   ```

   It walks the open assumptions one at a time. Each moving step warns first and
   can be skipped with `s`. Answer `y` when reality matches the description, `n`
   when it does not (you are then asked what happened instead). Use
   `--no-motion` for a first dry pass that only asks the non-moving questions.

   Take as long as you like: the procedure is paced by you, and the safety
   watchdog is fed immediately before each command, so reading time never counts
   as a dead control loop. If a step fails to reach the robot, the observation
   that interprets it is skipped automatically rather than recorded as a
   deviation — a report only contains findings whose command actually arrived.

   The result is written to `docs/bringup/bringup-<date>.md`.

5. **Transfer the outcome into [ASSUMPTIONS.md](../ASSUMPTIONS.md).** This is
   the actual deliverable: every confirmed row becomes `verified <date>`, every
   deviation becomes `wrong` **with the correction, keeping the original claim**.
   A bring-up whose findings stay in a report file has not been done.

6. **Play a routine** once the checklist is clean:

   ```console
   uv run robodog play routines/patrol-wifi.yaml --backend http
   ```

   It asks for confirmation, paces the timeline against the wall clock, and
   E-stops on Ctrl-C.

## Servo calibration (when you need it)

The firmware's trim facility is reachable over Wi-Fi and needs **no G12 jumper**
(ASSUMPTIONS D5) — but respect the order, or a servo will jump violently:

1. Send `funcMode=9` first (`HttpBackend.sync_servo_baseline()`). The firmware's
   internal `CurrentPWM[]` does not track what the gait wrote, so without this
   the first trim command snaps that servo to roughly its middle — a ≈47° jump
   on a loaded leg (ASSUMPTIONS D6).
2. Nudge with `trim_servo(channel, ±offset)`. Channels per leg are in
   `HttpBackend.servo_channels(leg)`.
3. Persist with `save_servo_trim(channel)` (firmware `sset` → NVS).

Trimming puts the firmware into debug mode, which suspends gait control; any
`move` or `funcMode` command returns it to normal operation.

## Measuring a leg's roll range

```console
uv run robodog calibrate-roll --backend mock --legs front_left   # rehearse first
uv run robodog calibrate-roll --legs front_left                  # then the robot
uv run robodog calibrate-roll --legs front_left --direction down # one end only
```

How far the wiggle servo can actually swing a leg under power is unknown
(ASSUMPTIONS C13). Moving an unpowered leg by hand back-drives the gearbox and
proves nothing about the commandable range, so this asks the robot: it nudges
the wiggle servo in small PWM steps and after each one you say whether the leg
still followed. Coarse steps (20 counts, ~9°) find the neighbourhood, fine steps
(2 counts, ~0.9°) find the edge, and the servo is backed off the instant it
stops following so it never sits stalled. The result is a dated report under
`docs/bringup/` with the measured counts, the angles they imply, and a
`LimitConfig(...)` line to carry into the code.

**The run opens by straightening every leg.** `funcMode 9` puts each *servo* at
its middle PWM count, which geometrically is a nearly vertical, fully extended
leg reaching ~115 mm — about 5 mm past the walking envelope, with the body
sitting higher than usual (ASSUMPTIONS C12). It looks alarming and it is
correct; the two coaxial cranks at angle 0 are extended, not straining. Confirm
it and carry on. It is also the pose with the longest lever arm on the wiggle
servo, so whatever range you measure is a conservative lower bound.

**Rehearse it against the mock first.** `--backend mock` walks the identical
procedure with no robot attached, so the first time you meet the prompts is not
also the first time a servo is moving.

Three properties worth knowing:

- **The firmware is taken out of its middle-position loop first.** `funcMode 9`
  never clears itself (ASSUMPTIONS D11), so the control loop keeps rewriting
  every servo; a trim sent into that is overwritten and the leg does not move.
  A zero-offset trim suspends the loop without moving anything, and the sweep
  only starts after it.
- **Stopping keeps what was measured.** Pressing `s` ends the sweep and then
  asks the one thing only you know: was the leg against its limit? Either way
  the counts travelled so far go into the report — an interrupted sweep is
  still evidence, and re-driving a servo into a stop to re-learn a known number
  is pure wear. Use `--direction up`/`down` to measure the other end afterwards.
- **A first step that does not move is flagged, not believed.** From the middle
  position that cannot be an end stop, so the direction is recorded with a note
  and excluded from the suggested limits.


- **`sset` is never sent.** Your stored servo calibration is untouched; the run
  always ends by returning every servo to its middle position.
- **Keep the fine step visible.** 2 counts is 0.9°, roughly 1.8 mm at the foot
  — below what you can reliably see, and answers at that scale are guesses that
  produce a confidently wrong range. The default is 5 counts (2.25°) and
  anything smaller prints a warning.
- **The per-command trim bound is the brake.** One command can move a servo at
  most `LimitConfig.servo_trim_offset_max` counts, so a sweep steps toward a
  limit rather than jumping at it. Raising `--coarse-step` above that bound does
  not widen it — the supervisor refuses the command.

What the measured numbers are *not*: they are the free-air range on a stand.
Under the robot's own weight, and with legs able to reach the body and each
other, the usable range is smaller — and **self-collision is not checked
anywhere** (C13).

## Troubleshooting

- **`cannot reach robot`** — the tooling diagnoses this itself: it reports which
  local address your PC would use to reach the robot, your current Wi-Fi network,
  and what to do about it. The two causes seen in practice:
  - **Still on the normal Wi-Fi.** `192.168.4.1` is the ESP32's own access-point
    address; it exists only inside that network. A laptop has one Wi-Fi radio, so
    joining the robot means leaving your home network — you lose internet for the
    duration of the session. The robot's LCD showing `192.168.4.1` only tells you
    the robot is in access-point mode, not that you are connected to it.
  - **An active VPN.** A full-tunnel VPN captures the route (the diagnosis then
    shows a source address like `10.x.x.x`), and VPN kill switches block
    link-local traffic outright. Disconnect the VPN, or enable its "allow LAN
    connections" setting. On the robot's network there is no internet to protect
    anyway.
- **It worked in the browser a minute ago and now nothing works** — Windows
  drops networks that have no internet in favour of a known-good one, so it
  silently reconnects to your normal Wi-Fi while the robot's page is still
  displayed in the browser (that HTML was loaded earlier; it is not proof of a
  live connection). Reconnect and verify immediately with `robodog info
  --backend http`. If it keeps jumping back, set your normal network to manual
  for the session and undo it afterwards:

  ```console
  netsh wlan set profileparameter name="<your normal SSID>" connectionmode=manual
  netsh wlan set profileparameter name="<your normal SSID>" connectionmode=auto
  ```

- **Close the robot's web page before scripting.** The ESP32 runs two HTTP
  servers (port 80 for control, 81 for the camera stream) out of a very small
  socket pool. A page left open — especially with a stalled MJPEG stream — can
  starve new connections, which looks exactly like a timeout.
- **The robot's SSID is not in the Wi-Fi list** — Windows caches scan results;
  opening the Wi-Fi flyout forces a fresh scan. The access point is 2.4 GHz only.
- **Right subnet but still no answer** — then it is the robot's side: it may
  still be booting. Open `http://192.168.4.1/` in a browser; the firmware serves
  its own web UI there, which cross-checks the robot independently of our code.
- **Robot walks but will not stop** — forward/backward and turning are separate
  latched axes; both need a stop. The tooling always sends both.
- **A command does nothing** — the firmware refuses requests that lack any of
  `var`, `val`, `cmd` (HTTP 404) and unknown variables (HTTP 500). Both surface
  as a `BackendError` with the status code.
