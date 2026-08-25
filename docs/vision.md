# Vision-guided behaviours ("Komm zu mir")

M8. The first motion on this robot with nobody's hand on the control: you type
or say something, a person is found in the camera, and the robot walks towards
them.

Three things run at once, and the split between them is the whole design:

| Piece | What it does | What it needs |
| --- | --- | --- |
| `robodog.vision` | reads the camera **once**, hands frames to everyone else, finds things in them | the `vision` extra for YOLO; nothing for the scripted detector |
| `robodog.behaviour` | detections and a clock in, drive intents out | nothing at all -- pure logic |
| `robodog.ai` | your words → **one** named behaviour with parameters | an OpenAI-compatible endpoint |

**The model does not drive.** It is asked once, before anything moves, to pick
one entry out of a three-entry vocabulary. From there a deterministic loop runs,
and every drive it produces goes through `SafetySupervisor` like every other
command in this repo. That is not a style choice: a model that can steer is a
model that can steer around the safety layer.

## Running it

```console
uv run robodog teach patrol --backend http --vision
```

That is the full thing: the teach UI as before, plus a Camera tab with detection
boxes and a command box beside both tabs. Type **Komm zu mir** and press Enter.

With no robot on the desk, the same page runs against anything:

```console
# a folder of JPEGs, played in a loop, with a real detector on them
uv run robodog teach demo --backend mock --vision-source ~/frames --detector yolo

# no model, no pictures -- just the plumbing
uv run robodog teach demo --backend mock --vision-source frame.jpg --detector scripted
```

And the language half alone, which needs neither:

```console
$ uv run robodog intent "komm her aber bleib zwei meter weg"
said:      'komm her aber bleib zwei meter weg'
model:     qwen-coder at http://127.0.0.1:8000/v1
behaviour: come_to_me(stop_distance_mm=2000, target=person)
target:    person
stops at:  0.58 of frame height (about 2000 mm, ASSUMPTIONS G2 -- uncalibrated)
gives up:  after 60s, or 12s searching
```

`robodog intent --vocabulary` prints exactly what the model is told.

## The camera has one viewer, and it is not your browser

This unit has **no PSRAM** (`psram=0`, measured 2026-08-22), so the firmware
runs `fb_count = 1`. A browser holding `http://192.168.4.1:81/stream` blocks
every other frame grab — there can be exactly one consumer of the robot's
camera.

So the teach session reads it, and re-serves it at `/camera/stream`. The page
and the detector both read that copy, and the Camera tab points at the host
rather than at the robot as soon as `--vision` is on. If the picture freezes
during a run, the first thing to check is whether something else opened the
robot's stream directly — the Vision panel's `fps` falls to zero (ASSUMPTIONS
G3).

The detection boxes are drawn **over** the picture as HTML, positioned at the
fractions the server computed. Nothing decodes a JPEG on either side, and the
boxes land correctly whatever resolution the camera is set to.

## Stop-and-look: the stream pauses while the robot moves

The robot's firmware sends no video frames while a move is latched. That is
not a limitation to work around -- it is the design, and it was the operator's
own suggestion after a day of measuring: a held stream degrades the gait
through mechanisms no task priority reaches (ASSUMPTIONS G12), and the frames
it delivers mid-stride are motion-blurred and pitch-skewed -- precisely the
ones that lie about distance (G2, F6).

Nothing in the behaviour was changed to support this, which is the elegant
part. Walking blanks the detections; the detections expire; the
never-walk-blind invariant halts the robot within its grace period; the halt
restarts the stream; the next look re-acquires. The approach becomes what a
careful animal does anyway: look, commit to a bounded advance, stop, look
again. The search was already built of turn-pulses with look-pauses, so it
sees between pulses for free.

Two consequences worth knowing at the controls: the Camera tab freezes while
the robot drives (the vision panel says so rather than crying failure), and an
approach takes longer than a continuous walk would -- trading speed for a gait
that stays smooth and pictures that tell the truth. `streamgate=0` over
`/control` restores the old behaviour for A/B.

## What the behaviour actually does

```
        no target            target off to the side       target centred
   ┌── SEARCHING ──────────►  APPROACHING ──────────────►  APPROACHING
   │   turn 0.6 s,            turn in place                walk (and correct)
   │   look 0.5 s,            (no forward)                       │
   │   give up at 12 s                                           ▼
   │        ▲                       ▲                        box big enough
   │        │ lost > 0.8 s          │ target seen again          │
   │        └───────────────────────┘                            ▼
   ▼                                                          ARRIVED
 LOST  ◄── the 60 s run timeout, from any state                (stopped)
```

Steering is not a fresh comparison per tick. The bearing is **low-passed**
(0.35 s) before anything reads it, and each band has a separate **entry and
exit** threshold. Both exist because of the first run on the robot, where
neither did: the box centre jitters enough that a bang-bang law turns the
jitter itself into alternating left/right commands, and a latched turn always
overshoots centre because the picture it steers by is tens of milliseconds old.
In simulation with realistic jitter that was 68 turn reversals per approach;
the filter alone takes it to zero, the hysteresis keeps an overshoot from
starting a turn back (ASSUMPTIONS G7).

**Losing the target close in is arrival, not loss.** The robot used to turn
away at exactly the point it had succeeded. Now: a target already being
approached is kept on weaker evidence than an unknown one is acquired on (0.25
against 0.40, and the detector's own floor sits below both so the weaker one is
reachable), and a target lost within `lost_close_margin` of the stop size ends
the run as ARRIVED, reporting the size it was lost at (ASSUMPTIONS G6).

That margin is worth understanding before changing it, because the height curve
is nearly flat here — 0.55 is 3 m and 0.59 is 1.5 m — so a small change in the
threshold is a large one in metres. Both ways of getting it wrong stop the
robot, but they are not equally good: too large calls a genuine mid-range loss
an arrival, which is merely wrong; too small sends the robot turning away to
search for someone standing right in front of it. It errs towards arrival.

That is deliberately *not* a tracker seeded from the last box. A tracker's
failure mode is to keep reporting a box after it has drifted, and the robot
then walks at a guess of a person it can no longer see; stopping early is the
failure this design would rather have. A tracker earns its place in a
`follow_me` behaviour, where the target stays at distance and the point is to
keep up — not in one whose whole purpose is to end in front of someone.

Two invariants, and they are why this is safe to point at a person with no
depth sensor, no bumper and no servo feedback:

1. **The robot never walks forward without a detection in that very tick.**
   Losing sight of the target stops it immediately — during the grace period it
   stands still rather than carrying on blind, and after it, it turns in place.
2. **Every run is bounded.** A hard timeout ends it wherever it got to, and the
   search gives up on its own.

On top of that the ordinary teach-UI safety applies unchanged, because the run
uses the same machinery a drive sequence does: the STOP button, Escape, the
page's dead-man's switch (close the tab and the robot stops), and the on-device
watchdog being fed while and only while a move is latched.

### Distance is a guess, and the stop does not depend on it

The only distance cue is how much of the frame's height the box fills, so the
behaviour stops on **that fraction** and never on a converted distance. The
conversion exists only to turn your "zwei Meter" into a fraction — and its
result is clamped, so nobody can ask the robot closer than it allows
(ASSUMPTIONS G5).

Worth knowing why the obvious formula is not used: with the camera about
140 mm off the floor, **a person is clipped by the top of the frame from about
3.4 m inward**. The textbook `size / distance` curve says a person fills the
picture at 1.9 m; they never do, at any distance. The geometry here models the
clipping, which is why the fraction saturates the way it does:

| Distance | 0.5 m | 1 m | 2 m | 3 m | 5 m | 10 m |
| --- | --- | --- | --- | --- | --- | --- |
| Frame height filled | 0.80 | 0.65 | 0.58 | 0.55 | 0.38 | 0.19 |

Field of view, camera height and pitch are all assumptions (ASSUMPTIONS G2).
Calibrating them is one session with a tape measure: stand at 1, 2 and 3 m and
read the fraction off the Camera tab.

## The language model is optional, and here is the proof

The control loop never contained it -- "the model does not drive" has been the
design rule since the first sketch -- but for a while the command box was the
only *trigger*, which quietly made the model load-bearing. No longer: the
**Komm zu mir button** starts the behaviour as a named call with explicit
parameters, through exactly the same vocabulary validation, with no model
configured at all. `--no-llm` now costs you free-text parsing and nothing
else.

What the model still buys, when it is there: turning words into parameters --
"bleib zwei Meter weg" becomes `stop_distance_mm=2000` -- once, before
anything moves. One request per typed command, one model-list probe at session
start, zero traffic otherwise.

## The language model

Any OpenAI-compatible `/v1/chat/completions` endpoint. Configure it with
`--llm-url` / `--llm-model`, or `$ROBODOG_LLM_URL` / `$ROBODOG_LLM_MODEL` /
`$ROBODOG_LLM_KEY`. With no model name it asks the server what it serves and
takes the first.

**Switch reasoning off.** Measured against the owner's vLLM box (Qwen3.8-27B
FP8, 2026-08-23), the same mapping took **0.68 s with thinking off and 30.8 s
with it on**, for an identical answer — 23 completion tokens against 1225. The
client sends `chat_template_kwargs: {"enable_thinking": false}` for exactly this
reason. A server that has never heard of that field is asked again without it,
with room to think.

The answer is grammar-constrained against a JSON schema generated from the
vocabulary, so the model *cannot* emit a behaviour that does not exist. It can
only choose badly — and `unknown` is one of the choices, deliberately: a model
with no way to say "not in my vocabulary" will always improvise one.

## Installing the detector

```console
uv sync --all-groups --extra viz --extra sim --extra vision
```

Not `uv sync --extra vision` on its own: uv matches the environment to exactly
what you ask for, so that would *uninstall* matplotlib and mujoco and two tests
would fail on a tree that is perfectly fine.

Optional, like `viz` and `sim`. Without it everything still runs — the page, the
re-served picture, the behaviour, the model — with `--detector scripted` or
`--detector none`. Nothing in the vision path needs a model to be tested, so
`uv sync --all-groups --extra viz --extra sim` runs the whole suite green
without ever fetching torch.

**torch comes from the CPU wheel index**, pinned in `pyproject.toml`. The CUDA
build pulls sixteen `nvidia-*` packages and lands at 5-6 GB against about
200 MB, and it buys nothing here: measured 2026-08-23 on the owner's machine,
`yolo11n` on CPU takes **40-51 ms a frame** — 20-25 fps, against a camera that
delivers ten-odd. Going back to CUDA is deleting two blocks in `pyproject.toml`
and re-locking; `--yolo-device` already lets a single run choose.

The YOLO model file (`yolo11n.pt` by default, `--yolo-model` to change it) is
downloaded by ultralytics **on first use**, not at install time. Fetch it while
you still have internet -- the robot's access point has no uplink, so the first
frame on the robot's own network would otherwise be a timeout:

```console
uv run python -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"
```

`--yolo-device` decides where it runs (`cpu`, `cuda:0`, ...). The default is
whatever ultralytics picks, which is the GPU -- worth overriding on a machine
that is also serving the language model from the same card.

## Before the first run on the robot

Four things are unverified and each is quick (ASSUMPTIONS G1-G5):

1. **Which way is right.** Stand clearly to one side and say "Komm zu mir". If
   the robot turns the wrong way, flip **Mirror** in the Camera tab — the sign
   comes from the picture, and nothing else depends on it (G1).
2. **The frame rate you actually get** with YOLO on your machine against the
   real stream. The Vision panel reports both `fps` and ms/frame.
3. **The distance calibration** above (G2).
4. **The single-viewer failure**, on purpose: open the robot's own stream in a
   second browser during a run and watch the detector go blind (G3). It is
   worth recognising once deliberately.

Keep the robot on a stand for the first two.
