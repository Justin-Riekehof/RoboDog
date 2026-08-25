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
             nichts gefunden                  Person gesehen
   SEARCHING ──────────────► LOOKING ◄───────────────────────────┐
   dreht in Pulsen,          steht still, Stream fliesst,        │
   schaut dazwischen         glaettet die Peilung (~0.35 s)      │
        ▲                       │                                │
        │ look_patience         │ Peilung > 7 deg   Peilung klein│
        │ abgelaufen            ▼                                │
        │                    ALIGNING ────────────► ADVANCING ───┘
        │                    dreht AUF DER STELLE,  geht NUR geradeaus,
        │                    gyro-geregelt bis zum  max. 1.2 s pro Schub,
        │                    Zielwinkel (±7 deg)    Gyro wacht ueber Drift
        │                                                │
      LOST ◄── 60 s Timeout, aus jedem Zustand           ▼ gross genug, 0.3 s
                                                      ARRIVED
```

The regime is the operator's, designed after the first stop-and-look drive on
the robot (2026-08-25). The robot had been allowed to walk and turn at once,
and one blind burst of turning at the measured 42.7 deg/s (G4) swung the
person clean out of a ~65 deg field of view -- overshoot, lost target, a
search that had to start over. Hence three rules, each pinned by tests:

1. **Nothing ever walks and turns at once.** Turning happens standing, walking
   happens dead straight. The steering bands, their hysteresis and the whole
   walk-while-correcting mode are gone.
2. **Blind rotation is closed-loop on the gyro.** A look yields a bearing; the
   turn runs until `turned` has covered it (±7 deg) -- not until a timer
   guesses it has. Overshoot in a single coarse reading ends the turn; a gyro
   that goes silent ends it too, because steering by a remembered angle is
   dead reckoning wearing a sensor's badge. Without any IMU (mock, sim) the
   fallback is short timed pulses at the measured, asymmetric rates.
3. **Blind advance is a bounded burst.** At most `walk_burst_seconds` (1.2 s,
   ~11 cm) between looks -- the "regularly check" half of the design -- and
   the gyro aborts the burst early if the heading drifts (the robot veers
   when walking, F1).

**And once you are near, it kneels to look at you between steps.** A close
sighting on this robot is always clipped by the top of the frame (the camera
rides a hand's width off the floor), so the clipped edge itself is the
trigger: from ~45% of frame height on, every standing look is taken with the
hindquarters dropped and the camera pitched up. The gait owns the servos
while the robot moves, so the tilt cannot persist through a walk burst -- the
runner re-kneels at every halt, which gives the approach its final rhythm:
step, kneel, look up, step. The overlay only draws boxes above 60% confidence
(the operator's request -- weak guesses cluttered the picture), with one
exception: the box the robot is actually following is always drawn, whatever
its score, because a robot following something the page refuses to show would
be debugging blindfolded.

**And when it gets so close that a level lens loses you entirely, the same
kneel becomes the verdict.** The operator's observation: at arrival distance a standing person's
torso is far above the camera's view, so "lost close in" used to be settled
by a heuristic. Now the robot drops its hindquarters, pitches the camera up
~15 deg (the vendor's own `pitchYawRollHeightCtrl` port supplies the pose),
and checks. Finding you turns a guess into a visual arrival -- and it stays
kneeling, looking up at you, which is the right ending for "Komm zu mir".
Finding nobody falls back to the old heuristic, saying so. The IMU measures
the commanded tilt and the distance correction absorbs it, so the size
reading stays honest while tilted; the pose itself is validated against the
leg workspace with margin (a first draft was refused by the supervisor --
pitching from full stand exceeds the reach envelope, which is why the kneel
is not decoration).

On top of that the ordinary teach-UI safety applies unchanged: STOP button,
Escape, the page's dead-man's switch, the on-device watchdog.

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
