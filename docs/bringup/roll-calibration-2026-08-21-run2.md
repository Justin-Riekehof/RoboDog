# Roll calibration report 2026-08-21

Host `192.168.4.1`, steps: coarse 20 counts, fine 2 counts.

Measured on the **wiggle** servo of each leg, by nudging it with the stock
firmware's `sconfig` facility and asking after every step whether the leg
still followed. `sset` was never sent, so the stored servo calibration is
unchanged (ASSUMPTIONS D5/D8).

| Leg | Channel | Direction | Last following | Angle | End stop |
|---|---|---|---|---|---|
| front_left | 10 | + | 2 counts | +0.9 deg | yes |
| front_left | 10 | - | 2 counts | -0.9 deg | yes |

## Suggested limits

Taking the tightest end over every leg measured, so the limit holds for
all of them:

```python
LimitConfig(roll_min=-0.9, roll_max=0.9)
```

Carry these into `LimitConfig` and update ASSUMPTIONS C13 with this run
as the source. Note what they are *not*: they are the range the servo
drives under no load, on a stand. Under the robot's own weight, and with
the legs able to reach each other and the body, the usable range is
smaller and nothing here checks for that.
