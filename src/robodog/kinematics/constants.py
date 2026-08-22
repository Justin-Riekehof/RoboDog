"""Geometry and tuning constants ported from the WAVEGO firmware.

Source: vendor/wavego-firmware/ServoCtrl.h and InitConfig.h (MIT, (c) 2022
waveshare). Values are treated as exact for simulation; hardware verification
tracked in ASSUMPTIONS.md section C.
"""

from __future__ import annotations

from robodog.api.types import LegId

# Leg linkage geometry, millimeters (ServoCtrl.h L28-L56, ASSUMPTIONS C1).
LINKAGE_S = 12.2  # distance between the two coaxial leg servos
LINKAGE_A = 40.0  # servo crank
LINKAGE_B = 40.0  # direction-limiting link
LINKAGE_C = 39.8153  # upper leg
LINKAGE_D = 31.7750  # lower leg
LINKAGE_E = 30.8076  # foot
LINKAGE_W = 19.15  # wiggle servo to leg-linkage plane

# Workspace / gait tuning (ServoCtrl.h L58-L72, ASSUMPTIONS C4).
WALK_HEIGHT_MAX = 110.0
WALK_HEIGHT_MIN = 75.0

# Roll (wiggle) envelope of the mechanism, MEASURED on the robot 2026-08-21
# with `robodog calibrate-roll` (ASSUMPTIONS C13). This is a property of the
# machine, so everything that has to agree about how far a leg can swing reads
# it from here: the safety limits (policy, which may be tightened at runtime)
# and the MJCF model (the simulated machine, which must be able to express
# every pose the policy permits). A literal in either of those is how the two
# drift apart -- and they did, until the model's hard-coded +/-60 deg silently
# swallowed a third of the measured range.
ROLL_MIN_DEG = -27.0
ROLL_MAX_DEG = 135.0
WALK_HEIGHT = 95.0
WALK_LIFT = 9.0
WALK_RANGE = 40.0
WALK_ACC = 5.0
WALK_EXTENDED_X = 16.0
WALK_EXTENDED_Z = 25.0
WALK_SIDE_MAX = 30.0
WALK_MASS_ADJUST = 21.0
STAND_HEIGHT = 95.0
WALK_LIFT_PROP = 0.25

# Gesture increments (WAVEGO.ino L49-L50, ASSUMPTIONS B5).
GESTURE_SPEED = 2.0
GESTURE_OFFSET_MAX = 15.0

# Balance clamp used by the firmware's steady mode (ServoCtrl.h balancing()).
BALANCE_CLAMP = 21.0

# Servo PWM mapping (ServoCtrl.h L7-L10, InitConfig.h L22, ASSUMPTIONS C2).
SERVO_PWM_MIN = 263
SERVO_PWM_MAX = 463
SERVO_RANGE_DEG = 90.0
SERVO_MIDDLE = 300
PWM_COUNTS_PER_90_DEG = SERVO_PWM_MAX - SERVO_PWM_MIN  # 200 counts / 90 deg

# PCA9685 channel assignment per leg: (fore, back, wiggle) (ServoCtrl.h L89-L103,
# ASSUMPTIONS C3).
SERVO_CHANNELS: dict[LegId, tuple[int, int, int]] = {
    LegId.FRONT_LEFT: (8, 9, 10),
    LegId.HIND_LEFT: (14, 15, 13),
    LegId.FRONT_RIGHT: (7, 6, 5),
    LegId.HIND_RIGHT: (1, 0, 2),
}

# Per-channel rotation direction (ServoCtrl.h L126-L129).
SERVO_DIRECTION: tuple[int, ...] = (-1, 1, 1, 1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1)
