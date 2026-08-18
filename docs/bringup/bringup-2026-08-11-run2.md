# Bring-up report 2026-08-11

Transport: Wi-Fi / HTTP, host `192.168.4.1` (stock firmware).

Confirmed: 6 · Differs: 1 · Skipped: 5 · Errors: 0

| Assumption | Check | Outcome | Note |
|---|---|---|---|
| D1 | Robot answers on HTTP | **confirmed** | — |
| D7 | Robot stands up on power-on | **confirmed** | — |
| D3 | /control answers with an empty 200 | **confirmed** | — |
| B3/D4 | move=1 starts a forward gait | **skipped** | not observed |
| B3/D4 | Motion latches until an explicit stop | **skipped** | invalid without 'move_forward', which did not run |
| D4 | move=3 + move=6 stop the gait | **skipped** | invalid without 'move_forward', which did not run |
| B3/D4 | move=2 turns in place | **skipped** | not observed |
| D4 | Turning stops independently | **skipped** | invalid without 'turn', which did not run |
| B4 | funcMode=2 runs the stay-low animation | **confirmed** | — |
| B8 | Function animations are blocking | **confirmed** | — |
| C12/D6 | funcMode=9 moves all servos to the calibrated middle | **confirmed** | — |
| B9/D10 | No link watchdog in the firmware | **differs** | kept in movin, also the web commands are messed up. "Forward" turns right, "backward" turns left. "Left" and "Right" stay the same. |

## What to do with this

Transfer every `confirmed` row into ASSUMPTIONS.md as `verified <date>`, and every `differs` row as `wrong` with the correction — do not delete the original claim.
