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
  **±10°** nudges for fine trims.
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

### Driving by hand

Below the sequence sits a **drive pad**: the eight directions laid out as the
controller they are, with STOP in the middle. One click sends one move straight
through the safety supervisor to the robot — the same command a sequence step
would send, just without the timer.

Two things follow from the firmware's model and are worth internalising:

- **A move latches.** The robot walks until a stop follows (ASSUMPTIONS B3/D4);
  there is no key to release and no timeout on the robot. The pad shows the
  latched direction highlighted and says so in words underneath.
- **The page is the dead-man's switch.** Exactly like a running sequence, a
  hand-driven move is released when the page stops answering — close the tab,
  lose the browser, sleep the laptop, and the robot stops within ten seconds.

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
