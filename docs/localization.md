# Knowing where the body is: the IMU, and the case for stop-and-shoot VIO

The robot has an ICM20948 — three accelerometer axes, three gyroscope axes, a
three-axis magnetometer. The vendor firmware initialises it, reads three of the
nine axes into globals, and then never calls the function: `accXYZUpdate()` is
commented out of `loop()`. Six axes had never left the chip.

They do now. This document is what the IMU is for here, what it is not, and the
honest case for and against visual-inertial odometry on a robot whose camera is
a 2022 OV2640 behind a Wi-Fi link.

## Why the IMU, and why now

Not navigation. The vision geometry assumes the camera looks horizontally, and
a walking quadruped's body does not:

| Body pitch | reads instead of 0.94 m |
| --- | --- |
| ±0,5° | 0,89 – 1,00 m |
| ±1° | 0,84 – 1,06 m |
| ±2° | 0,76 – 1,22 m |
| ±3° | 0,69 – 1,45 m |

Beside that, the detector's box noise is ±9 mm (ASSUMPTIONS G8). **The largest
error in the distance estimate is not perception, it is not knowing which way
the camera was pointing** — and an attitude per frame removes it.

## How it travels

A request over Wi-Fi costs 78–140 ms, so polling for single samples would yield
about eight a second: useless for integrating a gyroscope, which is the entire
reason to have one. So the device **buffers**:

- `loop()` samples at 50 Hz into a 64-slot ring — and `loop()` only, because
  nothing else may touch I2C on this firmware (see the fork's README; a second
  task on the bus crashes the robot).
- `var=imu&val=<last sequence seen>` returns everything newer, in one reply,
  with a per-sample device timestamp and a count of anything the ring dropped.
- The host folds the batch into a complementary filter and asks again.

Ten requests a second therefore carry fifty samples a second. And the request
costs nothing extra: it **replaces the watchdog keep-alive**. That ping already
cost a round trip while the robot walked and returned nothing; now the same
round trip returns the IMU, and a walking robot is exactly when its attitude
matters and exactly when nothing else is being sent.

## What the host does with it

[`robodog.localization`](../src/robodog/localization.py) is pure, like the
kinematics: samples in, attitude out, no sockets and no clock of its own.

- **Attitude** by complementary filter. The gyroscope is fast and drifts; the
  accelerometer is absolutely referenced to gravity and is ruined by every step
  the robot takes. So integrate the gyroscope, and let gravity pull the result
  back — but only from samples where the specific force *is* gravity and
  nothing else. A robot mid-footfall measures gravity plus the step, and taking
  the arctangent of that is how a level robot decides it is tilted.
- **Heading** as *turned since the run started*, never as a compass bearing.
  Nothing corrects yaw drift: a magnetometer sitting beside twelve servos and a
  PCA9685 is not an absolute reference until someone proves it is (G9).
- **Stillness**, which matters more than it looks — see below.

Deliberately not a Kalman filter. Every covariance in one would be a number
nobody has measured on this robot, and a filter tuned by guesswork gets the
same answer as two lines of blending, with its error budget hidden instead of
written down.

### The pitch correction, and its limit

The correction is *not* "subtract the shift from the box height". Pitch moves
content up and down the image; it does not change how big a thing is. A fully
visible person's box keeps its height exactly. Only the **clipped** case — where
the box's top edge is the frame rather than the head, which from about 3.4 m
inward is every case — grows and shrinks with pitch, because there the height is
measuring where the feet are and nothing else. So: correct the bottom edge, and
only when the top is clipped.

It is **exact when the robot is still and approximate while it walks**
(ASSUMPTIONS G10), and the reason is not the filter. It is that the frame and
the IMU sample cannot be aligned in time: MJPEG parts carry no timestamps, the
Wi-Fi latency varies request by request, and the gait's pitch oscillates at
roughly the step rate. The correction uses the most recent attitude, which is
right when nothing is moving and a guess mid-stride.

Which is the whole argument for what follows.

## Stop-and-shoot, and why it changes the answer on VIO

The objection to visual-inertial odometry here was never the resolution. It was
four things, and the fourth was fatal:

1. **Rolling shutter.** The OV2640 reads the sensor line by line; a moving
   camera shears the image.
2. **Motion blur**, made worse by the fork's own exposure settings, which buy
   brightness with integration time (ASSUMPTIONS F6).
3. **A jerky gait** — periodic impulses, not the smooth motion a monocular
   front-end likes.
4. **No time alignment.** Frames arrive over MJPEG with no timestamps and a
   variable latency; VIO lives or dies on knowing which IMU samples belong to
   which frame.

**Capturing only while standing still defeats all four.** Nothing moves during
the exposure, so there is no shear and no blur. The gait is not in the picture
because the gait has stopped. And the frame belongs to an interval in which the
attitude was constant — so it does not need to be aligned to a millisecond, it
needs to be labelled "the still period between step 7 and step 8". The hardest
requirement disappears rather than being met.

This is not a workaround invented here. It is how planetary rovers navigate,
for the same reason: when the visual channel is expensive or fragile, you stop
paying for motion you do not need.

Two further gains come free:

- **Zero-velocity updates.** Inertial position is double integration and
  diverges within seconds — unless you can periodically assert that velocity is
  exactly zero. A robot that stops between shots hands you that assertion on a
  plate, which is precisely why pedestrian dead reckoning keys on the stance
  phase. `still_for()` exists for this.
- **Metric scale.** Monocular SLAM is scale-free: it recovers structure up to
  an unknown factor, so on its own it cannot answer "how far away is the
  person". The IMU with ZUPT supplies that factor. So does the known camera
  height against the ground plane — and those two cross-check each other, which
  is worth more than either.

### What stop-and-shoot costs, honestly

- **Speed.** Walk, stop, settle, capture, process, walk. A quadruped needs time
  to stop oscillating after the gait halts — unmeasured, and the first thing to
  measure.
- **No odometry between stops except the IMU.** There are no encoders on this
  robot and there never will be (ASSUMPTIONS A3), so the inter-stop motion is
  inertial dead reckoning over a short hop. That is exactly the regime where it
  works, but the hop has to stay short.
- **It changes the behaviour.** "Come to me" currently walks continuously. A
  stop-and-shoot version arrives later and looks more deliberate. Whether that
  is worse is a question about what the robot is for.

### Measuring the rolling shutter anyway

Worth doing once, and cheap: photograph a fast, known periodic motion, or strobe
an LED at a known rate, and read the line time off the skew. It bounds how much
motion a frame can tolerate before it is unusable — which tells you how *short*
the stop needs to be, and whether some frames can be taken on the move after all.
With stop-and-shoot it is not on the critical path, which is the point.

## Why a bad camera is the interesting case

None of this is specific to a cheap sensor. A camera with rain on the lens, or
one in a dark room, or one on a machine that vibrates, degrades along exactly
these axes: blur, missing detections, geometry you cannot trust frame to frame.
The techniques that survive here — an inertial reference for the horizon,
capturing only when the platform is quiet, treating a stalled detector as an
empty room rather than a full one, keeping the safety decision on the *measured*
quantity rather than the *inferred* one — are the ones that survive there.

Good hardware lets you skip the question. It does not answer it.

## Order of work

1. **Flash and check the axis signs** (G9). Tip the robot nose-down: the pitch
   on the teach page must go negative. One observation.
2. **Camera height and field of view with a tape measure** (G2). Ten minutes,
   and it turns every metre in this system from a guess into a measurement.
3. **Turn rate** (G4), which the gyroscope now gives directly — and the search
   stops being a stopwatch and becomes an angle.
4. **Settling time after the gait stops.** The number that decides whether
   stop-and-shoot is a second or a fifth of one.
5. Then, and only then, the stop-and-go loop and visual odometry on top of it.

Steps 1–4 are minutes of work each and are the whole foundation. Step 5 is a
project.
