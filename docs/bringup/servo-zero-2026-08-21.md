# Servo zero calibration 2026-08-21

Host `192.168.4.1`, nudge step 5 counts (2.25 deg per step).

Offsets are PWM counts from the firmware's middle position
(`funcMode=9`) to the angle the kinematics calls zero. The degree
column applies the channel's direction sign (ASSUMPTIONS C2/C3).

| leg | joint | channel | offset (counts) | offset (deg) | steps | note |
| --- | --- | --- | --- | --- | --- | --- |
| front_left | fore | 8 | +0 | +0.00 | 0 |  |
| front_left | back | 9 | +0 | +0.00 | 0 |  |
| front_left | wiggle | 10 | +0 | +0.00 | 0 |  |

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
