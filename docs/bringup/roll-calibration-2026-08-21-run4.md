# Roll calibration report 2026-08-21

Host `192.168.4.1`, steps: coarse 20 counts, fine 5 counts.

Measured on the **wiggle** servo of each leg, by nudging it with the stock
firmware's `sconfig` facility and asking after every step whether the leg
still followed. `sset` was never sent, so the stored servo calibration is
unchanged (ASSUMPTIONS D5/D8).

**Run aborted:** stopped by the operator

| Leg | Channel | Direction | Last following | Angle | End stop | Note |
|---|---|---|---|---|---|---|
| front_left | 10 | - | 60 counts | -27.0 deg | yes | operator stopped at the mechanical limit |

## Suggested limits

Not enough trustworthy ends were found to suggest a limit. A direction
whose *first* step did not move the leg is not evidence of an end stop --
see its note above and re-run that direction.
