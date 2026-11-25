/* Task-owned Buddy 4 lifecycle. No motor writes, queues or console calls.
 * Reads acceleration (tilt, hump phases, lift/overturn latch) and the
 * magnetometer (raw counts for the compass in terrain_task.c).
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
/* WAIT_START (bit 0, formerly the recovery command): no dashboard START yet.
 * INIT stays in POSITION with its timer restarted, so the level reference is
 * captured only after START and the capture timeout cannot fire while waiting.
 * Ignored once a reference exists (recovery goes straight to MONITORING). */
#define B4_WAIT_START (0x01U)
/* MOTORS_STOPPED, CMD_RESET and CAUSE_CLEARED only drive the STOP/FAULT -> INIT
 * reset. The firmware never sends RESET (terrain_inputs): a new run and a
 * latched fault are cleared by power-up. Only the host tests use this path. */
#define B4_MOTORS_STOPPED (0x02U)
#define B4_CMD_STOP       (0x04U)
#define B4_CMD_RESET      (0x08U)
#define B4_CAUSE_CLEARED  (0x10U)
#define B4_LEVEL_ALLOWED  (0x20U)
/* HUMP_PAUSE: controller hold (dashboard STOP or any controlled stop): the
 * robot is stopped, so MONITORING measures no humps and discards the one in
 * progress; run maximum and last peak are kept. Recording resumes when the
 * bit clears (START). Tilt and lift/overturn detection keep running. */
#define B4_HUMP_PAUSE (0x40U)
/* TURN_PAUSE: the robot is pivoting or spinning on the spot (one wheel stopped
 * or the wheels opposite), or did so in the last 0.5 s. The accelerometer has
 * no gyroscope and reads the sideways jolt of a starting or stopping turn as
 * tilt, i.e. as a false hump. Same handling as HUMP_PAUSE: no humps, the one in
 * progress is discarded, run maximum and last peak kept. Lift detection keeps
 * running. */
#define B4_TURN_PAUSE (0x80U)

#define B4_STALE_MS (100U)
#define B4_LOSS_MS  (500U)

#define B4_RETRY_MS   (3000U)
#define B4_VERIFY_MS  (500U)
#define B4_CAPTURE_MS (10000U)
/* Settling time before capture. The robot is placed level before power-up, so
 * 1 s (not the bench board's 5 s hand-placement time) lets the capture finish
 * about 2 s after boot, before line following starts driving.
 */
#define B4_POSITION_MS  (1000U)
#define B4_MAX_ATTEMPTS (3U)
/* Distance between the two drive wheels' ground-contact centres, mm. Hump
 * height = this x sideways gravity fraction, so a 1 mm error scales heights by
 * ~0.9%. Measure on the robot (Buddy 2's 0.115 m is the turning-calibrated
 * track).
 */
#define B4_TRACK_MM (115U)

/* Lift/overturn detection is armed this long after entering MONITORING, so
 * the hand placing the robot during capture is not taken for a lift. */
#define B4_LIFT_SETTLE_MS (100U)

/* Named unsigned bit-fields keep the required 3+2+2 bits readable.
 * Internal task-owned storage only: never transmit its raw bytes as a protocol.
 * Allocation order is compiler-dependent; named access does not depend on it.
 */
typedef struct
{
    uint8_t state    : 3;
    uint8_t attempts : 2;
    uint8_t phase    : 2;
} buddy4_control_t;

_Static_assert(1U == sizeof(buddy4_control_t),
               "Buddy 4 control must occupy one byte");

typedef struct
{
    tilt_context_t tilt;
    /* One hump orientation sequence; no motor commands. */
    terrain_t terrain;
    /* Existing 27-bit tilt result; zero unavailable. */
    uint32_t result;
    /* Monotonic millisecond low words, wrap-safe. */
    uint32_t phase_started;
    uint32_t last_fresh;
    ER       error; /* Last kernel/acquisition diagnostic code. */
    /* IMU forward dynamic acceleration, mg (diagnostic, 12 bits). */
    int16_t forward_mg;
    /* Lift/overturn latch; reset on every state change. */
    lift_t lift;
    /* Last good magnetometer counts X/Y/Z (12-bit signed). */
    int16_t  mag[3];
    uint32_t mag_ms; /* Time of the last good magnetometer sample. */
    /* Bit 0: mag[] holds a good sample (B4_MAG_HAVE). */
    uint8_t mag_flags;
    /* B4_FREEZE_*: hump recording frozen after OVERTURNED. Survives state
     * changes; cleared only by START. */
    uint8_t          hump_freeze;
    buddy4_control_t control; /* Readable fields, one byte total. */
} buddy4_t;

/* Cold-start a fresh context. Only call once; reset commands preserve
 * reference. */
void buddy4_begin(buddy4_t * p_context, uint32_t now_ms);
/* Advance one state-machine step in task context. Checks STOP before I2C.
 * Monitoring reads acceleration and the magnetometer; no retry loops or delays.
 * Driver calls are synchronous and may exceed the nominal task period.
 * This context belongs to one task; external tasks must not mutate it directly.
 */
void    buddy4_step(buddy4_t * p_context, uint8_t inputs);
uint8_t buddy4_state(buddy4_t const * p_context);
uint8_t buddy4_attempts(buddy4_t const * p_context);
/* Latched lift/overturn condition (LIFT_NONE/LIFTED/OVERTURNED). Detection
 * runs only in MONITORING; any state change clears it (lift detection is
 * unavailable while the sensor recovers or has faulted). */
uint8_t buddy4_lift(buddy4_t const * p_context);
/* Hump freeze (demo: the run ends with the robot lifted and turned over). An
 * OVERTURNED latch freezes hump recording: no hump is measured and the run
 * maximum and last peak stay as they were, even after the lift latch clears,
 * so setting the robot down can never record a peak. START unfreezes it:
 * call buddy4_hump_start() every cycle with Buddy 2's status. It unfreezes
 * only after the controller hold has been seen and then released (START
 * accepted) with the lift latch clear; it returns true on that cycle. A LIFTED
 * latch alone does not freeze. Power-up (buddy4_begin) also clears it. */
#define B4_FREEZE_NONE (0U)
#define B4_FREEZE_WAIT (1U) /* Frozen; controller hold not seen yet. */
#define B4_FREEZE_HELD (2U) /* Frozen; hold seen, START will unfreeze. */
bool buddy4_hump_start(buddy4_t * p_context, bool b_available, bool b_held);
bool buddy4_hump_frozen(buddy4_t const * p_context);
/* Latest good magnetometer sample no older than max_age_ms (raw counts X/Y).
 * False when none (not read yet, out of range, stale, or not MONITORING). */
#define B4_MAG_HAVE (0x01U)
bool         buddy4_magnetic(buddy4_t const * p_context, uint32_t now_ms,
                             uint32_t max_age_ms, int16_t * p_x, int16_t * p_y);
char const * buddy4_state_name(uint8_t state);
#endif

/*** end of file ***/
