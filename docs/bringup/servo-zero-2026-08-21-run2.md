# Servo zero calibration 2026-08-21

Host `192.168.4.1`, nudge step 5 counts (2.25 deg, 4.6 mm at the foot, per step).

**That step is the tolerance.** A zero was placed by eye against a
vertical link, so every offset below means *within one step* -- an
offset of 0 is 'zero to within 2.25 deg', never 'exactly zero'.

Offsets are PWM counts from the firmware's middle position
(`funcMode=9`) to the angle the kinematics calls zero. The degree
column applies the channel's direction sign (ASSUMPTIONS C2/C3).

| leg | joint | channel | offset (counts) | offset (deg) | steps | note |
| --- | --- | --- | --- | --- | --- | --- |
| front_left | fore | 8 | +0 | +0.00 | 4 | servo trim offset -25 exceeds +/-20 counts per command; step towards a limit, do not jump at it |
| front_left | back | 9 | +0 | +0.00 | 0 | already at the reference, no nudge needed |
| front_left | wiggle | 10 | +15 | +6.75 | 3 |  |
| hind_left | fore | 14 | +20 | -9.00 | 1 |  |
| hind_left | back | 15 | +0 | +0.00 | 4 |  |
| hind_left | wiggle | 13 | +0 | +0.00 | 0 | already at the reference, no nudge needed |
| front_right | fore | 7 | +10 | +4.50 | 2 |  |
| front_right | back | 6 | +0 | +0.00 | 2 |  |
| front_right | wiggle | 5 | -10 | +4.50 | 9 |  |
| hind_right | fore | 1 | +10 | +4.50 | 1 |  |
| hind_right | back | 0 | +0 | +0.00 | 0 | already at the reference, no nudge needed |
| hind_right | wiggle | 2 | +0 | +0.00 | 4 |  |

## Reference used

- **fore**: The FRONT upper arm -- the 40 mm link from the servo horn to the elbow -- must hang EXACTLY VERTICAL.
- **back**: The REAR upper arm must hang EXACTLY VERTICAL, i.e. parallel to the front arm you have just set; at zero the two are parallel, 12 mm apart.
- **wiggle**: The whole leg PLANE must hang vertical -- no sideways lean. The foot sits about 19 mm outboard even at zero; that is the linkage width, not a tilt.

## What this does not say

- The offsets assume the firmware's stored `ServoMiddlePWM[]` is still
  at its default; `sset` is never sent by this codebase, but another
  tool may have written it and nothing can read it back (ASSUMPTIONS D8).
- Nothing here was measured by the robot. The operator's eye is the
  only sensor on this path (ASSUMPTIONS A3/D3).
