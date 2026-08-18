# Bring-up session transcript, 2026-08-11

Raw terminal log of the first Wi-Fi bring-up attempt, kept for the record.
The motion steps all failed on a bug in our own tooling (the safety
watchdog counted the operator's reading time as a dead control loop), so
the motion findings in this run are **void**. See the structured outcome in
[bringup-2026-08-11.md](bringup-2026-08-11.md) and the corrected transfer in
[../../ASSUMPTIONS.md](../../ASSUMPTIONS.md).

```text
(robodog) PS C:\Users\Justin\Documents\stash\RoboDog> uv run robodog info --backend http
robodog 0.1.0
backend:      http
capabilities: LOCOMOTION, SERVO_TRIM
safety state: DISARMED
limits:       y [75.0, 110.0] mm, x +/-45.0 mm, z [-20.0, 60.0] mm, joints +/-65.0 deg
NOTE:         state below is a MODEL, not measured -- this
              transport returns no data at all.
initial pose (stand):
  FRONT_LEFT  target=(  16.0,  95.0,  25.0)mm servos=(w +3.50 f+46.76 b+35.02)deg
  HIND_LEFT   target=( -16.0,  95.0,  25.0)mm servos=(w +3.50 f+25.31 b+56.57)deg
  FRONT_RIGHT target=(  16.0,  95.0,  25.0)mm servos=(w +3.50 f+46.76 b+35.02)deg
  HIND_RIGHT  target=( -16.0,  95.0,  25.0)mm servos=(w +3.50 f+25.31 b+56.57)deg
(robodog) PS C:\Users\Justin\Documents\stash\RoboDog> uv run robodog bringup

========================================================================
 WAVEGO bring-up over Wi-Fi (stock firmware)
========================================================================

 SAFETY, read before continuing:
  * Put the robot ON A STAND with the legs hanging free, or keep a
    hand on the power switch. Over Wi-Fi a dropped link cannot be
    recovered: no stop command reaches the robot (ASSUMPTIONS D10).
  * The firmware has no watchdog. Whatever it was last told to do, it
    keeps doing.
  * Ctrl-C triggers an E-stop attempt, but it can only work while the
    connection is alive.

Type 'yes' when the robot is secured and you are ready: yes

------------------------------------------------------------------------
[D1] Robot answers on HTTP
  Join the robot's Wi-Fi access point (SSID 'WAVESHARE Robot', password '1234567890'). The connection check already ran.
  Did the connection succeed without you changing anything? [y/n/s] y

------------------------------------------------------------------------
[D7] Robot stands up on power-on
  Recall what happened when you switched the robot on: the firmware commands the stand pose about a second into boot, before Wi-Fi starts.
  Did the legs move into a stand by themselves at power-on? [y/n/s] y

------------------------------------------------------------------------
[D3] /control answers with an empty 200
  Two stop commands were sent during connect. No response body is expected -- the firmware never returns data.
  Did that complete without an error message? [y/n/s] y

------------------------------------------------------------------------
[B3/D4] move=1 starts a forward gait
  The robot will start WALKING and keep walking until the next step stops it. Legs must hang free.
  >>> THIS MOVES THE ROBOT <<<
  Press Enter to run it, or 's' to skip: 
  command failed: E-stop latched (watchdog timeout (0.500s)); reset() first

------------------------------------------------------------------------
[B3/D4] Motion latches until an explicit stop
  Watch the robot: no stop command has been sent yet.
  Did it keep walking on its own, without further commands? [y/n/s] n
  What happened instead? Nothing

------------------------------------------------------------------------
[D4] move=3 + move=6 stop the gait
  Both axes are being stopped now.
  >>> THIS MOVES THE ROBOT <<<
  Press Enter to run it, or 's' to skip:  
  command failed: E-stop latched (watchdog timeout (0.500s)); reset() first

------------------------------------------------------------------------
[B3/D4] move=2 turns in place
  The robot will turn left in place until stopped.
  >>> THIS MOVES THE ROBOT <<<
  Press Enter to run it, or 's' to skip: 
  command failed: E-stop latched (watchdog timeout (0.500s)); reset() first

------------------------------------------------------------------------
[D4] Turning stops independently
  Stopping both axes again.
  >>> THIS MOVES THE ROBOT <<<
  Press Enter to run it, or 's' to skip: 
  command failed: E-stop latched (watchdog timeout (0.500s)); reset() first

------------------------------------------------------------------------
[B4] funcMode=2 runs the stay-low animation
  The robot will crouch and rise again -- one shot.
  >>> THIS MOVES THE ROBOT <<<
  Press Enter to run it, or 's' to skip: 
  command failed: E-stop latched (watchdog timeout (0.500s)); reset() first

------------------------------------------------------------------------
[B8] Function animations are blocking
  A handshake will start; it takes about four seconds. While it runs, try clicking Forward in the robot's own web UI.
  >>> THIS MOVES THE ROBOT <<<
  Press Enter to run it, or 's' to skip: 
  command failed: E-stop latched (watchdog timeout (0.500s)); reset() first

------------------------------------------------------------------------
[C12/D6] funcMode=9 moves all servos to the calibrated middle
  All twelve servos go to their stored middle position. The legs will straighten and the body sits HIGHER than the walking envelope -- support the robot.
  >>> THIS MOVES THE ROBOT <<<
  Press Enter to run it, or 's' to skip: 
  command failed: E-stop latched (watchdog timeout (0.500s)); reset() first

------------------------------------------------------------------------
[B9/D10] No link watchdog in the firmware
  SAFETY TEST, do this with the robot lifted or on a stand: start a forward walk from the robot's web UI, then switch off your PC's Wi-Fi (or walk out of range) without stopping it.
  Did the robot keep walking after the link was gone? [y/n/s] n
  What happened instead? nothing

summary: 3 confirmed, 2 differ, 0 skipped, 7 errors
report written to docs\bringup\bringup-2026-08-11.md
Next: transfer these outcomes into ASSUMPTIONS.md (verified / wrong).
(robodog) PS C:\Users\Justin\Documents\stash\RoboDog> 
```
