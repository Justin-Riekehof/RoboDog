# Teach-in: authoring routines

`robodog teach` opens one page with two tabs, because there are two useful
things to teach this robot and they need different tools:

| Tab | You author | Result | Runs on |
| --- | --- | --- | --- |
| **Pose** | leg poses, captured as keyframes | `kind: motion` | mock, sim (hardware needs M4) |
| **Sequence** | named moves with durations, looped | `kind: sequence` | mock, sim, **the real robot** |

The Pose tab is the software answer to a robot with no servo feedback: instead
of physically guiding the legs (impossible on write-only PWM servos), you pose
the **digital twin** and watch real physics hold — or refuse — every pose. The
Sequence tab is the answer to the opposite question — "walk forward for ten
seconds, then turn" — and needs nothing but locomotion, which is exactly what
the stock firmware offers over Wi-Fi (ASSUMPTIONS D2).

Both produce a plain routine YAML in [routines/](../routines/) that
`robodog play` replays unchanged.

## Pose tab: the web UI (default)

```console
uv run robodog teach wave --backend sim --viewer
```

This starts a local page (127.0.0.1 only, the browser opens automatically) and,
with `--viewer`, the MuJoCo window next to it showing live physics.

- **Three draggable views.** Side view (x forward / height), top view
  (x forward / sideways) and front view (sideways / height): grab a foot and
  pull it where it should go. The browser sends world coordinates; all frame
  conversion, kinematics and safety checking stay in Python.
- **The front view is where the leg rolls.** The linkage is planar and the
  wiggle servo swings that whole plane sideways. That motion is only visible —
  and only draggable — in the front view, which draws each leg's roll arc as a
  dashed guide so the available freedom is on screen rather than implied.
  Rolling does not change how far the leg reaches or where it sits fore/aft; it
  is one joint moving (ASSUMPTIONS C13), and the drag reflects that: **only the
  direction of your pointer is used**, its distance from the hip is ignored, so
  the foot follows the arc instead of falling out of the reach band whenever
  your hand wanders off it. Change the reach with the reach slider, which is
  the control that means it.
- **Type exact angles.** Every leg row has a number field per axis, including
  roll, plus ±1° and ±10° steps. There is a field for all four legs at once
  next to the roll slider. For roll the
  page shows the measured envelope: **-27 to +135 degrees**, established on the
  robot on 2026-08-21 (ASSUMPTIONS C13). Note that this is *less* than the ~170
  degrees the leg reaches when pushed by hand — back-driving a gearbox is not
  the range the servo can drive.
- **Every pose passes the safety supervisor.** A drag into a forbidden,
  unreachable or self-inconsistent pose (ASSUMPTIONS C10/C11) is rejected with
  the supervisor's message in the status bar, and the foot snaps back.
- **Ghost feet** (dashed circles) show where the backend *measured* the foot —
  on the sim backend that is physics, so you see sag and disturbance, not just
  your command.
- **Mirror left/right** applies your drags and nudges symmetrically to the
  paired leg — most poses are symmetric, so this halves the work.
- **Leg reach and roll sliders** move all four feet together — reach is measured
  *in the leg plane*, so it means the same thing at any roll angle;
  **Stand/Crouch** buttons jump to the known poses; per-leg **±5 mm** and
  **±10°** nudges for fine trims. **Home**, in the drive console beside the
  tab, is the bigger hammer: it stops the robot first and then stands.
- **Keyframes**: `Capture pose` records the current pose `dt` seconds after the
  previous one; the table offers *Go to* (drive the robot back into a frame)
  and *Delete*; `Preview` replays everything captured so far and returns to
  your working pose; cosine/linear easing is selectable.
- **Save** writes `routines/<name>.yaml` (rename in the text field; overwrite
  needs the checkbox). The file is re-validated through the same parser the
  player uses — a teach session cannot produce a file playback would reject.
- **Quit** ends the session (it warns about unsaved keyframes); Ctrl-C in the
  terminal works too.

The page has no external dependencies and the server binds to localhost only.
While a preview runs, the page shows a busy state and rejects edits.

**Pacing.** Every keyframe tick is a fresh pose, and over Wi-Fi every pose is a
round trip of roughly 100 ms. The preview therefore ticks at whatever the
backend says it can carry (`suggested_tick`): 50 Hz against mock and the twin,
about 10 Hz on the real robot. A preview on hardware sends *fewer* poses than
the same routine in the twin, but it takes the same wall-clock time — pacing it
faster than the link carries would not play it faster, only late.

**Smoothness does not come from the link.** Ten poses a second would be visibly
stepped if each one were a jump, and they used to be: `GoalPosAll()` writes
straight to the servo driver with no ramp, and the robot's own loop repeats that
write every 4 ms — some 250 times a second, 24 of which carry nothing new. So a
pose now names how long it may take, and the firmware travels there across that
window instead of arriving at once. Five poses a second from the host become
hundreds of servo positions on the robot.

The duration is the interval actually being achieved, measured rather than
assumed, so a link that slows down gets longer ramps by itself and the robot is
still moving when the next pose lands. Interpolation is linear on purpose: the
teach session already eases whole routines, and easing each segment on top of
that would decelerate into every one — a pulse at 10 Hz, worse than the steps it
replaces. Older firmware ignores the duration and jumps, exactly as before.

## Sequence tab: programming move sequences

```console
uv run robodog teach patrol --backend sim     # against the twin
uv run robodog teach patrol --backend http    # on the real robot, over Wi-Fi
```

A sequence is a list of **standard moves, each held for its own time** — the
program an operator dictates out loud: *10 s forward, 2 s left, 5 s backward*.

- **The move vocabulary** is the firmware's two latched axes (ASSUMPTIONS
  B3/D4) under the names the buttons use: forward, backward, turn left, turn
  right, the four diagonals, and `wait` (stand still). Click one to append it,
  then type its seconds; rows can be reordered and deleted.
- **The pause between moves** is one number for the whole sequence (`gap`). It
  is inserted between every two consecutive moves and the robot stands still
  during it — set it to 0 and moves flow directly into one another. For a
  longer, one-off pause, add a `wait` move instead.
- **Repeat** is the loop: a count, or *loop until stopped* — which is the tick
  box, and stored as `repeat: 0`.
- **The timeline** below the list draws the pass to scale, with a cursor that
  moves while the sequence runs and the running move highlighted in the table.
- **Run / STOP.** Running asks for confirmation, then plays the sequence
  through the same player, safety supervisor and backend as `robodog play`,
  always paced against the wall clock — ten seconds of forward take ten
  seconds on the twin as well, so the timeline you watch is the one the robot
  will walk. While it runs the editor is locked
  (edits answer *busy*), but stopping never is.

**Four ways to stop**, because anything that latches with one way out is a
defect: the STOP button next to Run, the STOP in the middle of the drive pad,
the **Escape** key, and simply closing the page — the server watches the page's
own polling and ends anything nobody is watching any more (10 seconds of
silence). All four post the same action, which stops a running sequence and a
hand-driven move alike, and stopping is never refused: press it with nothing
moving and the robot still gets a stop. Quitting the session stops it too. What
none of this can fix is a **dropped Wi-Fi link**: no stop command reaches a
robot that is no longer listening (ASSUMPTIONS D10), so the robot belongs on a
stand with the power switch in reach.

## Camera tab: the live feed, and the sensor behind it

```console
uv run robodog teach patrol --backend http    # the tab appears on Wi-Fi only
```

The tab shows the robot's MJPEG stream and, on our own firmware, a control for
every sensor register the fork exposes (`cam_<name>`, see
[docs/firmware.md](firmware.md)). Mock and the twin have no lens, so the tab is
not there at all.

- **Start / Stop.** The robot serves **one viewer at a time**, and an `<img>` on
  a hidden tab keeps its connection open — so leaving the tab drops the stream
  rather than holding it against the next person. Starting again reconnects with
  a fresh URL; a cached dead stream otherwise shows the last frame of the
  previous session forever.
- **Exposure** is the group that decides whether you can see anything: auto
  exposure and its DSP variant, the level the AEC aims for, and the manual
  integration time and gain for when you switch the automatics off. Everything
  here is bought with something — longer integration is motion blur on a robot
  that walks, more gain is grain (ASSUMPTIONS F6).
- **Image** carries the frame size and JPEG quality. Quality is inverted:
  *lower* is better and bigger. The size list stops at 320x240 because the frame
  buffer is allocated once at boot and never grows — asking for more ends in no
  image at all (F4).
- **Colour** is white balance and its presets; **Orientation** is mirror and
  flip.

**The controls show what the sensor holds.** Every `cam_*` request answers with
the camera's own state, so a write confirms itself in the same round trip and a
change made elsewhere — the vendor's web page, a serial session — shows up on
the next one. The panel says which of the two you are looking at: firmware older
than that reply answers with an empty 200, and then the controls fall back to
remembering what was asked for and label themselves accordingly.

Two things follow from reading rather than remembering. **Frame size is clamped,
not refused** — the robot silently lands on whatever frame buffer it allocated
at boot, so asking for 640x480 on a robot that fell back to QVGA answers *asked
8, holding 5* instead of pretending. And the list of sizes is **capped to that
robot's own ceiling**, which is the one value nothing else can work out: the
buffer is chosen once, before `esp_camera_init`, and only the robot knows
whether it got what it asked for (ASSUMPTIONS F4).

Every value goes through the safety supervisor like any other command — not
because a white balance setting can hurt anyone, but because a value outside a
register's range is a silent no-op that reads as a broken camera, and one
particular value (frame size) really does end in a black screen. The ranges the
sliders draw and the ranges the validator enforces are the same table
([src/robodog/camera.py](../src/robodog/camera.py)).

## The drive console: hand control on both tabs

To the right of whichever tab is open sits the **drive console**, which belongs
to the robot rather than to a tab: the eight directions laid out as the
controller they are with STOP in the middle, the **Home** button, and the
firmware's canned animations. One click sends one move straight through the
safety supervisor to the robot — the same command a sequence step would send,
just without the timer.

### Keyboard

The console is driveable from the keyboard, in both tabs:

| Key | Does |
| --- | --- |
| `W` `A` `S` `D` | walk forward / turn left / backward / turn right — **hold**, release stops |
| numpad `8` / `2` | extend the legs / go down on all four (±5 mm of reach) |
| numpad `4` / `6` | lean left / right (±5 mm, one side extends as the other shortens) |
| numpad `5` | Home |
| `Esc` | stop |

Two things are deliberate. **WASD is hold-to-walk**, unlike the pad's
click-to-latch: a latched direction bound to a key would be the worst of both,
because you take your hand off the keyboard and the robot keeps going. Losing
the window releases it too — a key held while the page loses focus never comes
back up. And the keys do nothing while you are typing in a field, so naming a
routine `wasd` is safe.

**Leaning is not the roll slider.** Roll lives in each leg's own frame and those
frames mirror left to right, so one roll value on all four legs swings both feet
outward and leaves the body dead level — a splay, not a lean. A quadruped with
no spine leans by standing differently on each side: the legs on one side reach
further down, and with the feet planted the body follows. All four legs move or
none do; half a lean is a robot standing crooked in a way nobody asked for.

**Home** is the way back to the middle from wherever the robot ended up: it
stops a latched move *first*, then stands in the pose the session opened in.
The order is the point — a robot still walking walks straight out of the pose
it was just given. On a robot that takes no leg targets (stock firmware) Home
is the stop alone, and says so rather than pretending.

Two things follow from the firmware's model and are worth internalising:

- **A move latches.** The robot walks until a stop follows (ASSUMPTIONS B3/D4);
  there is no key to release and no timeout on the robot. The pad shows the
  latched direction highlighted and says so in words underneath.
- **The page is the dead-man's switch.** Exactly like a running sequence, a
  hand-driven move is released when the page stops answering — close the tab,
  lose the browser, sleep the laptop, and the robot stops within ten seconds.

- **A pose ends the move.** Our firmware clears both drive axes when it applies
  a pose (`robodogApply`), because a gait rewrites the servos every pass and
  would walk out of the pose within milliseconds. So dragging a foot — or
  pressing Home, Stand or Crouch — stops the robot, and the pad stops showing a
  direction. Against mock and the twin the two coexist, since nothing there
  overwrites anything.

The pad is locked while a sequence runs (its STOP is not), and starting a
sequence takes over from hand driving.

**Save** writes a `kind: sequence` file; **Load** reopens one for further
editing (only sequence files — a pose routine is refused with a clear message).
The saved file is the program, not its expansion:

```yaml
schema: robodog.routine/v1
name: patrol-loop
kind: sequence
requires: [LOCOMOTION]
gap: 0.5
repeat: 3
moves:
  - {move: forward, seconds: 10}
  - {move: left, seconds: 2}
  - {move: backward, seconds: 5}
```

The player expands that into the drive timeline (forward at 0 s, stop at 10 s,
turn left at 10.5 s, ...) and repeats it. Two details are worth knowing: a
drive intent that is already in effect is never re-sent — over Wi-Fi every
command is two HTTP requests — and the timeline always **ends stopped**, whether
it ran to the end or was interrupted.

That "10 s forward" means ten seconds of walking rests on the firmware latching
a move until the matching stop arrives (ASSUMPTIONS **B3/D4**) — **verified on
this robot on 2026-08-22**: it walked, kept walking with nothing further sent,
and stopped only on the matching stop command. Each axis stops independently,
which is why the compiled timeline always sends both.

On the real robot (`--backend http`) the Pose tab is switched off: the stock
firmware takes no leg targets, so there is nothing for it to send. The CLI asks
for confirmation before opening the UI, exactly like `robodog play`
(`--yes` skips it).

## The console (`--repl`)

The pose session is scriptable as a terminal REPL — useful headless, over SSH,
or in tests: `uv run robodog teach wave --backend mock --repl`. It authors
poses only, so it is refused on the Wi-Fi backend. Commands:
`leg fl fr`, `x +5`, `y = 80`, `roll = 60`, `depth -5`, `pose stand`,
`cap 0.8`, `undo`, `preview`, `save`, `quit` — type `help` for the full
grammar. Axes per leg: x forward, y down toward the ground, z outward, plus
`roll` (degrees) and `depth` (reach in the leg plane), which describe the same
foot in the terms the mechanism actually moves in.

## Replay

```console
uv run robodog play routines/wave.yaml --backend sim --viewer
uv run robodog play routines/wave.yaml --backend mock
uv run robodog play routines/patrol-loop.yaml --backend http   # a saved sequence
```

`robodog validate <file>` prints a sequence back as the move list it is, and
`robodog play` honours `repeat` — including `repeat: 0`, where Ctrl-C is the
only way out and triggers an E-stop.

## Properties worth knowing

- **Everything is validated twice** — live at every drag/nudge, and again when
  saving.
- **The workspace is bounded in the leg's own coordinates**, not as a box in
  (height, sideways). Those two are coupled by the roll, so a box would cut the
  roll off after some 20 degrees; reach and roll angle are bounded separately
  instead (ASSUMPTIONS C13). Beware what is *not* checked: at large roll angles
  legs will intersect the body and each other, and nothing detects that yet.
  One consequence to know: at about 78.6 degrees of
  roll the foot passes exactly y = 0, where the vendor IK has a defective branch
  (C6). The supervisor rejects that single point; a drag will never land on it,
  but `y = 0` typed in the console will be refused.
- **The file is the artifact.** Saved routines are ordinary v1 routine files:
  git-diffable, hand-editable, no session state left behind.
- **Physics is the honesty check.** If a pose tips the twin over, you watch it
  fall — information no kinematic editor gives you.
- **Deployment of poses to hardware** waits on M4 (the stock firmware has no
  pose-level commands, ASSUMPTIONS D2) and on M3 calibration for the repaired
  hind-right leg (F1/F3). The routine files stay valid; only the backend
  changes. Sequences do not wait for either: locomotion is what Wi-Fi already
  carries.
