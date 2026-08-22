# Servo zero calibration (M3)

> Not to be confused with [`robodog calibrate-roll`](bringup.md), which drives a
> servo towards its **mechanical end stops** to learn how far a leg can travel;
> those numbers become workspace limits (ASSUMPTIONS C13). This procedure finds
> each joint's **zero** and its numbers become the servo mapping's offsets. Both
> are needed, they measure different things, and they write different files.

The twin and the robot share one kinematic model, so a pose authored against
the twin only lands on the robot if both agree on where each joint's **zero**
is. The model assumes PWM count 300 for every servo, because that is the
firmware's default table. The real machine disagrees: horns sit on a coarse
spline, linkages have play, arms get remounted, and the hind-right leg on this
robot is a repaired part with its own geometry (ASSUMPTIONS F1/F3).

That disagreement is a **constant per servo**. Measuring it once and storing it
is what this procedure does.

```console
# rehearse the whole flow with no robot attached (writes no table: a
# rehearsal measures nothing, and its report says so)
uv run robodog calibrate-servos --backend mock --legs front_left

# the real thing, one leg at a time
uv run robodog calibrate-servos --legs front_left
uv run robodog calibrate-servos --legs hind_left,front_right

# what has been measured so far
uv run robodog calibrate-servos --show
```

## What is measured

For each servo: **how many PWM counts it sits away from the angle the
kinematics calls zero**, seen from the baseline `funcMode=9` establishes.

The reference is not a matter of taste — it comes out of the ported forward
kinematics. At `fore = 0, back = 0` the planar FK puts both upper links
straight down: the front servo axis is at (+6.1, 0) and its elbow at
(+6.1, 40), the rear at (-6.1, 0) and (-6.1, 40). In other words:

| joint | at zero, this is true |
| --- | --- |
| `fore` | the **front** upper arm (40 mm, horn to elbow) hangs exactly vertical |
| `back` | the **rear** upper arm hangs exactly vertical, parallel to the front one, 12 mm behind it |
| `wiggle` | the whole leg **plane** hangs vertical; the foot sits ~19 mm outboard even so, which is the linkage width, not a tilt |

Two arms that are parallel *and* vertical is a thing an eye judges well, which
matters, because the eye is the only sensor on this path: the firmware returns
no data whatsoever (ASSUMPTIONS D3/A3).

## How a run goes

1. **Safety prompt.** Robot on a stand, legs free. Nothing is sent before you
   confirm.
2. **Baseline** (`funcMode=9`, ASSUMPTIONS D6). Every servo goes to its stored
   middle. Expect the legs to *straighten and extend* and the body to sit
   higher than when walking — the firmware's "middle" is per servo, not a
   neutral pose (ASSUMPTIONS C12).
3. **Escape the middle-position loop** (a zero-offset `sconfig`, ASSUMPTIONS
   D11). Without it the firmware keeps rewriting every servo hundreds of times
   a second and your nudges are silently overwritten — which reads exactly like
   a joint that will not move.
4. **Nudge, per joint.** `+5` / `-5` / any signed count / `ok` / `skip` /
   `stop`. Enter alone repeats the step. Each nudge goes through the safety
   supervisor, which caps a single command at 20 counts.
   Answering `ok` **without having nudged at all** is a claim -- that the joint
   is already exactly at its reference -- so it asks you to confirm it. That is
   deliberate: pressing enter through three joints would otherwise write a
   fully "calibrated" leg of zeros that measured nothing. `skip` is the honest
   answer when you do not want to judge a joint; it leaves it unmeasured.
5. **Restore.** Whatever happens, including an abort, the run ends by sending
   the robot back to its middle position.

Each leg is re-baselined before it is measured, so a leg you aligned earlier
snaps back. That is expected: what the run keeps is the *number*, not the pose.

## The result

A versioned file, by default `calibration/servos.yaml`:

```yaml
schema: robodog.calibration/v1
robot: wavego-standard-basic
measured: 2026-08-21
reference: upper arms vertical, leg plane vertical
offsets:
  front_left: {fore: 6, back: -4, wiggle: 2}
```

Offsets are raw PWM counts; the sign of the *angle* they correspond to depends
on the channel's direction (ASSUMPTIONS C2/C3), which is why
`calibrate-servos --show` prints both. `resolution` is the nudge step the run
used and therefore **the tolerance of every number in the file**: a zero placed
by eye at a 5-count step means *zero to within 2.25 degrees*, which is about
4.6 mm at the foot -- never "exactly zero". A run updates only the joints it
measured, so **partial tables are normal** — measuring twelve servos is a long
session with a human in it, and one leg per sitting is a sensible pace.

Every run also writes a dated report to `docs/bringup/`, like bring-up does.

## What this does not do

- **It never writes to the robot.** The firmware's `sset` stores a servo middle
  in NVS permanently; nothing in this codebase routes that command, so your
  robot's stored calibration is untouched and the numbers live in git instead.
- **It cannot verify its own premise.** The offsets are relative to the
  firmware's stored `ServoMiddlePWM[]`, and that table cannot be read back
  (ASSUMPTIONS D8). If another tool ever wrote it, everything here needs
  re-measuring and no software can detect it.
- **It does not yet move poses onto the robot.** The table is the prerequisite;
  the path that consumes it — twin pose → IK → 12 absolute PWM counts →
  baseline plus one `sconfig` per servo — is the next step (ASSUMPTIONS D8).
  Note that this suits *static poses only*: one HTTP request per servo, with no
  synchronised multi-servo update, so a keyframed trajectory stays an M4 topic.
- **A stop during trimming is not proven.** On 2026-08-21 a `sconfig` reached
  the robot and moved the leg, but the stop commands that followed both timed
  out (ASSUMPTIONS D5). Nothing walks during a calibration sweep, which is the
  mitigating factor — but do not treat this path as one you can interrupt.

## Reading the numbers

Scale matters when reading an offset. One tooth of a servo horn's spline is
14-18 degrees depending on the servo (25T or 20T; this robot's servo type is
not documented anywhere we trust), which is **32 to 40 PWM counts**. So:

- **Under ~30 counts**: not a mis-mounted horn. That is linkage play, horn
  slop and servo centre variance — exactly the systematic error this table
  exists to absorb.
- **Around 32-40 counts**: suspiciously close to one tooth. Worth checking that
  it is the joint you think it is; the tool flags anything past 40.
- **Past 120 counts**: refused outright, because that is not a zero.

## Repeatability

Nothing on this robot can be read back, so the only second opinion available is
the same operator measuring again. **Re-running a leg that already has stored
values is therefore the repeatability check**, and the tool does it for free:
each joint shows its stored value before you judge it, and any result that
lands further from it than the step size is called out in the run, in the
report, and in the summary.

That check is worth doing at least once, because it answers the question the
table cannot answer about itself: whether "the arm is vertical" is a judgement
you can reproduce to within one step, or whether the numbers are eyeball noise
wearing a unit.

**A table of zeros is a real result**, not a failed run. On a factory-assembled
robot the stored middles *are* the geometric zeros, so every arm hangs vertical
at `funcMode=9` and every offset is zero to within the step you judged at. What
the file then records is that this was checked, when, and how exactly -- which
is what makes a later re-check after a repair meaningful, and which turns
"we assume 300 is zero" (ASSUMPTIONS C2/D8) into something observed on this
machine.

Two consequences worth drawing:

- The **pose tolerance** you can expect on hardware is bounded by this
  resolution, not by the kinematics. At a 5-count step that is roughly +/-4.6 mm
  at the foot per joint.
- If the robot still walks crooked with a table of zeros, the cause is not
  servo zeros. The right side under-performing while walking (ASSUMPTIONS F1)
  is then mechanical or electrical, and a calibration table will not fix it.

To tighten the tolerance, run with `--step 2` (0.9 degrees, 1.8 mm at the
foot). Below that the operator is being asked to see what cannot be seen, and
the answers become noise dressed as precision.
