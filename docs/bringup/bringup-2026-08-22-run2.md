# Bring-up report 2026-08-22

Transport: Wi-Fi / HTTP, host `192.168.4.1` (stock firmware).

Confirmed: 3 · Differs: 1 · Skipped: 3 · Errors: 5

| Assumption | Check | Outcome | Note |
|---|---|---|---|
| D1 | Robot answers on HTTP | **confirmed** | — |
| D7 | Robot stands up on power-on | **confirmed** | — |
| D3 | /control answers with an empty 200 | **confirmed** | — |
| B3/D4 | move=1 starts a forward gait | **error** | cannot reach robot at http://192.168.4.1/control?var=move&val=1&cmd=0: <urlopen error timed out> |
| B3/D4 | Motion latches until an explicit stop | **skipped** | invalid without 'move_forward', which did not run |
| D4 | move=3 + move=6 stop the gait | **skipped** | invalid without 'move_forward', which did not run |
| B3/D4 | move=2 turns in place | **error** | cannot reach robot at http://192.168.4.1/control?var=move&val=3&cmd=0: <urlopen error timed out> |
| D4 | Turning stops independently | **skipped** | invalid without 'turn', which did not run |
| B4 | funcMode=2 runs the stay-low animation | **error** | cannot reach robot at http://192.168.4.1/control?var=funcMode&val=2&cmd=0: <urlopen error timed out> |
| B8 | Function animations are blocking | **error** | cannot reach robot at http://192.168.4.1/control?var=funcMode&val=3&cmd=0: <urlopen error timed out> |
| C12/D6 | funcMode=9 moves all servos to the calibrated middle | **error** | cannot reach robot at http://192.168.4.1/control?var=funcMode&val=9&cmd=0: <urlopen error timed out> |
| B9/D10 | No link watchdog in the firmware | **differs** | did not move at any time |

## What to do with this

Transfer every `confirmed` row into ASSUMPTIONS.md as `verified <date>`, and every `differs` row as `wrong` with the correction — do not delete the original claim.
