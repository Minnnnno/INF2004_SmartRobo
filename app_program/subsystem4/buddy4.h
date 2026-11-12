/* Task-owned Buddy 4 lifecycle. No motor writes, queues or console calls.
 * Integration stage 1: acceleration, tilt and terrain phases only. Magnetic
 * heading and its calibration are deliberately not integrated yet.
 */
#ifndef BUDDY4_H
#define BUDDY4_H
#include "gy511_read.h"
#include "tilt.h"
#include "terrain.h"
#include "lift.h"

/* Names are constants; retained state occupies three bits in control. */
#define B4_INIT       (0U)
#define B4_MONITORING (1U)
#define B4_RECOVERY   (2U)
#define B4_FAULT      (3U)
#define B4_STOP       (4U)

/* Caller supplies a coherent snapshot from the owning task each update.
 * RESET is a one-cycle command. Sensor recovery is automatic and bounded.
 * STOPPED and CAUSE_CLEARED are controller-confirmed facts, never inferred.
 * LEVEL permits capture only when the user/controller guarantees level/still.
 */
/* Input bit 0 is reserved (formerly the recovery command). */
#define B4_MOTORS_STOPPED (0x02U)
#define B4_CMD_STOP       (0x04U)
#define B4_CMD_RESET      (0x08U)
#define B4_CAUSE_CLEARED  (0x10U)
#define B4_LEVEL_ALLOWED  (0x20U)
/* Bits 0x40 and 0x80 are reserved for the heading stage (MARK, CAL_DONE). */

#define B4_STALE_MS      (100U)
#define B4_LOSS_MS       (500U)

#define B4_RETRY_MS      (3000U)
#define B4_VERIFY_MS     (500U)
#define B4_CAPTURE_MS    (10000U)
/* Settling time before capture. The robot is placed level before power-up, so
 * 1 s (not the bench board's 5 s hand-placement time) lets the capture finish
 * about 2 s after boot, before line following starts driving.
 */
#define B4_POSITION_MS   (1000U)
#define B4_MAX_ATTEMPTS  (3U)
/* Distance between the two drive wheels' ground-contact centres, mm. Hump height
 * = this x sideways gravity fraction, so a 1 mm error scales heights by ~0.9%.
 * Measure on the robot (Buddy 2's 0.115 m is the turning-calibrated track).
 */
#define B4_TRACK_MM      (115U)

/* Lift/overturn detection is armed this long after entering MONITORING, so
 * the hand placing the robot during capture is not taken for a lift. */
#define B4_LIFT_SETTLE_MS (100U)

/* Named unsigned bit-fields keep the required 3+2+2 bits readable.
 * Internal task-owned storage only: never transmit its raw bytes as a protocol.
 * Allocation order is compiler-dependent; named access does not depend on it.
 */
typedef struct
{
    uint8_t state : 3;
    uint8_t attempts : 2;
    uint8_t phase : 2;
} buddy4_control_t;

_Static_assert(sizeof(buddy4_control_t) == 1U, "Buddy 4 control must occupy one byte");

typedef struct
{
    tilt_context_t tilt;
    terrain_t terrain;        /* One hump orientation sequence; no motor commands. */
    uint32_t result;          /* Existing 27-bit tilt result; zero unavailable. */
    uint32_t phase_started;   /* Monotonic millisecond low words, wrap-safe. */
    uint32_t last_fresh;
    ER error;                 /* Last kernel/acquisition diagnostic code. */
    int16_t forward_mg;       /* IMU forward dynamic acceleration, mg (diagnostic, 12 bits). */
    lift_t lift;              /* Lift/overturn latch; reset on every state change. */
    int16_t mag[3];           /* Last good magnetometer counts X/Y/Z (12-bit signed). */
    uint32_t mag_ms;          /* Time of the last good magnetometer sample. */
    uint8_t mag_flags;        /* Bit 0: mag[] holds a good sample (B4_MAG_HAVE). */
    buddy4_control_t control; /* Readable fields, one byte total. */
} buddy4_t;

/* Cold-start a fresh context. Only call once; reset commands preserve reference. */
void buddy4_begin(buddy4_t *p_context, uint32_t now_ms);
/* Advance one state-machine step in task context. Checks STOP before I2C.
 * Monitoring reads acceleration only; no retry loops or delays.
 * Driver calls are synchronous and may exceed the nominal task period.
 * This context belongs to one task; external tasks must not mutate it directly.
 */
void buddy4_step(buddy4_t *p_context, uint8_t inputs);
uint8_t buddy4_state(buddy4_t const *p_context);
uint8_t buddy4_attempts(buddy4_t const *p_context);
/* Latched lift/overturn condition (LIFT_NONE/LIFTED/OVERTURNED). Detection
 * runs only in MONITORING; any state change clears it (lift detection is
 * unavailable while the sensor recovers or has faulted). */
uint8_t buddy4_lift(buddy4_t const *p_context);
/* Latest good magnetometer sample no older than max_age_ms (raw counts X/Y).
 * False when none (not read yet, out of range, stale, or not MONITORING). */
#define B4_MAG_HAVE (0x01U)
bool buddy4_magnetic(buddy4_t const *p_context, uint32_t now_ms, uint32_t max_age_ms,
                     int16_t *p_x, int16_t *p_y);
char const *buddy4_state_name(uint8_t state);
#endif
