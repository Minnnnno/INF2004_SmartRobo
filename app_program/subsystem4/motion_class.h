/* Buddy 4 motion classification (brief: motion classification, collision
 * detection, turn-rate monitoring, motion event reporting).
 * Fusion by design: the IMU supplies what only it can sense (impact latch,
 * one-wheel hump phase from sideways tilt); Buddy 2's encoder status supplies
 * movement (stationary, speed change, yaw rate). The GY-511 has no gyroscope,
 * and the robot's ~0.03 g acceleration sits in the accelerometer's vibration
 * noise, so encoder speed is the acceleration source (IMU logged alongside).
 * Portable: no kernel calls; one owning task; zero-fill or reset before use.
 */
#ifndef MOTION_CLASS_H
#define MOTION_CLASS_H
#include <stdbool.h>
#include <stdint.h>

/* Class codes (3 bits, CR1). Published in the terrain slot and M_CLASS. */
#define MC_STATIONARY   (0U)
#define MC_MOVING       (1U)
#define MC_ACCELERATING (2U) /* Speed changing, either direction (braking included). */
#define MC_REVERSING    (3U) /* Encoders measure the robot driving backwards. */
#define MC_TURNING      (4U)
#define MC_CLIMBING     (5U)
#define MC_DESCENDING   (6U)
#define MC_IMPACT       (7U)
#define MC_CLASS_MASK   (0x07U)

/* Thresholds (tune from robot logs). */
#define MC_IMPACT_HOLD_MS  (300U) /* IMPACT stays reported this long (frame is 100 ms). */
/* TURNING = the robot's heading really changed: |net heading change over the
 * last ~0.5 s| >= 15 deg (an average of ~30 deg/s held). Line-following
 * corrections flip left/right and net close to 0, so they stay MOVING. */
#define MC_TURN_NET_DDEG   (150)  /* 15.0 deg, in 0.1 deg. */
#define MC_TURN_WINDOW_MS  (500U) /* Nominal window; 5 samples, 100 ms apart. */
#define MC_HEADING_SLOTS   (5U)
#define MC_HEADING_STEP_MS (100U)
#define MC_HEADING_JUMP_DDEG (900) /* > 90 deg in one 100 ms step: odometry reset. */
#define MC_TURN_MOVING_DEG_S (10)  /* A turn's duration ends at the last |yaw| >= this. */
#define MC_ACCEL_MM_S2     (150)  /* |change in centre speed size| at or above this. */
#define MC_REVERSE_MM_S    (-20)  /* Centre speed at or below this is REVERSING; the
                                     margin rejects encoder jitter at standstill. */
#define MC_ACCEL_WINDOW_MS (100U) /* Speed change measured over this window. */
#define MC_CONFIRM_SAMPLES (3U)   /* A new class must repeat this many cycles. */
#define MC_MAX_GAP_MS      (100U) /* Longer input gaps reseed the speed window. */

/* One fresh observation from the owning task. */
typedef struct
{
    bool valid;          /* Buddy 4 MONITORING and motion status available. */
    bool stationary;     /* Buddy 2 encoders report the robot at rest. */
    bool impact;         /* IMU inertial latch fired since the previous step. */
    uint8_t terrain_phase; /* TERRAIN_WAITING/CLIMBING/DESCENDING (terrain.h). */
    int16_t speed_mm_s[2]; /* Buddy 2 wheel speeds, left then right. */
    uint16_t track_mm;   /* Wheel track for the yaw rate (B4_TRACK_MM). */
    bool heading_valid;  /* Buddy 2 odometry heading is finite. */
    int32_t heading_ddeg; /* Buddy 2 odometry heading, unwrapped, 0.1 deg, + = left. */
    bool compass_valid;  /* Calibrated, fresh, field in range, not on a hump. */
    int32_t compass_ddeg; /* Compass heading, unwrapped, 0.1 deg, + = left. */
} motion_class_input_t;

/* Packed byte: reported class bits 0..2, candidate class bits 3..5,
 * candidate repeat count bits 6..7 (0..3). Flags: bit 0 valid, bit 1 impact
 * hold active, bit 2 speed window seeded, bit 3 turn in progress, bit 4
 * completed turn waiting. ring: write index bits 0..2, filled count bits 3..5,
 * turn start/end debounce count bits 6..7.
 */
typedef struct
{
    uint32_t impact_since;  /* Start of the current impact hold. */
    uint32_t last_ms;       /* Time of the previous valid step. */
    uint32_t window_ms;     /* Start of the current speed-change window. */
    int16_t window_mm_s;    /* Centre speed at the window start. */
    int16_t accel_mm_s2;    /* Rate of change of centre speed SIZE, mm/s^2:
                               + speeding up, - slowing down (either direction). */
    int16_t yaw_deg_s;      /* Yaw rate, + = left/anticlockwise. */
    int32_t heading_ring[MC_HEADING_SLOTS]; /* Heading samples, 0.1 deg. */
    uint32_t heading_ms;    /* Time of the newest ring sample. */
    int32_t turn_start_ddeg; /* Heading where the current turn began. */
    uint32_t turn_start_ms;
    uint32_t turn_moving_ms; /* Last time |yaw| >= MC_TURN_MOVING_DEG_S in the turn. */
    int16_t turn_peak_deg_s; /* Highest |yaw| during the current turn. */
    int16_t last_turn_ddeg;  /* Completed turn angle, 0.1 deg, + = left. */
    uint16_t last_turn_ms;   /* Completed turn duration. */
    int16_t last_turn_peak;  /* Completed turn peak |yaw|, deg/s. */
    int32_t compass_ring[MC_HEADING_SLOTS]; /* Compass samples beside heading_ring. */
    int32_t turn_compass_start;
    int16_t last_turn_compass; /* Completed turn compass angle, 0.1 deg. */
    uint8_t compass_bits;    /* Bit i: compass_ring[i] was valid (5 bits). */
    uint8_t turn_info;       /* Bit 0 compass valid so far in this turn, bit 1 last
                                turn compass valid. */
    uint8_t ring;
    uint8_t packed;
    uint8_t flags;
} motion_class_t;

void motion_class_reset(motion_class_t *p_context);
/* One step per task cycle. Invalid input publishes STATIONARY with the valid
 * flag clear and discards the speed window and any impact hold.
 */
void motion_class_step(motion_class_t *p_context, motion_class_input_t const *p_input,
                       uint32_t now_ms);
uint8_t motion_class_get(motion_class_t const *p_context);
bool motion_class_valid(motion_class_t const *p_context);
/* True while a turn (net heading change) is in progress, before it is reported. */
bool motion_class_turn_active(motion_class_t const *p_context);
int16_t motion_class_yaw_deg_s(motion_class_t const *p_context);
int16_t motion_class_accel_mm_s2(motion_class_t const *p_context);
/* Consume one completed turn (true at most once per turn): signed angle in
 * 0.1 deg (+ = left), duration in ms and peak |yaw| in deg/s. Any output
 * pointer may be NULL. The angle comes from Buddy 2's encoders: wheel slip
 * inflates it. */
bool motion_class_take_turn(motion_class_t *p_context, int16_t *p_angle_ddeg,
                            uint16_t *p_duration_ms, int16_t *p_peak_deg_s);
/* Compass angle for the turn last returned by motion_class_take_turn():
 * true with the angle (0.1 deg, + = left) when the compass was valid for the
 * whole turn; false = "compass n/a". Shown next to the encoder angle for a
 * by-eye comparison; no slip verdict is made (removed for now). */
bool motion_class_turn_compass(motion_class_t const *p_context, int16_t *p_compass_ddeg);
/* Yaw rate from wheel speeds: (right - left) / track, deg/s, rounded. */
int16_t motion_class_yaw_from_speeds(int16_t left_mm_s, int16_t right_mm_s, uint16_t track_mm);
char const *motion_class_name(uint8_t motion_class);
#endif
