/* Buddy 4 terrain task: sole owner of the GY-511 and of the buddy4_t context.
 * Acceleration, tilt, hump phases and peak height, lift/overturn detection,
 * motion classification with turn rate (from Buddy 2 wheel speeds; no
 * gyroscope), and a calibrated compass whose angle is shown next to each
 * turn's encoder angle.
 * Motion commands: only motion_stop() when the robot is lifted or overturned
 * (Change Requirement: replaces IMU collision detection; lifting the robot
 * ends the run), and the self-pivot of the dedicated magnetometer
 * calibration build (ROBO_MAG_CAL), both through the Controller.
 */
#include "robot_rtos.h" /* Standard size_t before the kernel headers. */
#include "../../include/float_types.h"
#include "robot.h"
#include "terrain_status.h"
#include "subsystems.h"
#include "motion_hal.h"
#include "telemetry_frame.h"
#include "../subsystem1/comms.h"
#include "buddy4.h"
#include "motion_class.h"
#include "mag_heading.h"
#include "mag_cal.h"
#if defined(ROBO_MAG_CAL)
#include "steering.h"
#endif
#include <tm/tmonitor.h>

#define S4_PERIOD_MS     (20U)
#define S4_REPORT_MS     (500U)
#define S4_TENTHS_TO_RAD (0.0017453292519943295) /* pi / 1800 */
#define S4_DEG_TO_RAD    (0.017453292519943295)  /* pi / 180 */
/* Compass sample freshness (30 Hz sensor). */
#define S4_MAG_MAX_AGE_MS (100U)

/* Compass calibration constants from mag_cal.h (pasted from the cal build). */
static mag_cal_t const g_mag_cal
    = {(int16_t)B4_MAG_OFFSET_X,     (int16_t)B4_MAG_OFFSET_Y,
       (uint16_t)B4_MAG_SCALE_Y_Q12, (uint16_t)B4_MAG_FIELD,
       (int8_t)B4_MAG_HEADING_SIGN,  (0 != B4_MAG_CALIBRATED)};

/* Why the compass is unusable ("compass n/a (...)" on the console). */
#define S4_MAG_OK     (0U)
#define S4_MAG_UNCAL  (1U)
#define S4_MAG_NODATA (2U)
#define S4_MAG_HUMP   (3U)
#define S4_MAG_FIELD  (4U)
#define S4_MAG_WINDOW (5U)
typedef struct
{
    int32_t  unwrapped;   /* Compass heading, unwrapped, 0.1 deg, + = left. */
    int16_t  heading;     /* Last valid heading 0..3599. */
    uint16_t field;       /* Last horizontal field magnitude, counts. */
    uint8_t  reason;      /* Current S4_MAG_*. */
    uint8_t  turn_reason; /* First invalid reason since the last [TURN] line. */
    bool     have;        /* unwrapped holds a value. */
} s4_compass_t;

static s4_compass_t g_s_compass; /* Task-owned. */
/* Task-owned web Start gate, S4_START_* (terrain_wait_start). */
static uint8_t g_s_start_gate;
/* Task-owned: hump recording paused while stopped. */
static bool gb_s_hump_paused;
/* Task-owned: hump recording paused while turning on the spot. */
static bool gb_s_turn_paused;
/* Task-owned: s_turn_ms holds the last pivot time. */
static bool gb_s_turn_seen;
/* Task-owned: last cycle the wheels were commanded to pivot. */
static uint32_t g_s_turn_ms;
/* Turning on the spot (terrain_turning): the faster commanded wheel at least
 * this fast, with the other stopped or opposite. Line-following steering keeps
 * both wheels forward (about 70 +/- 37.5 mm/s), so it never matches. */
#define S4_TURN_MIN_MM_S (20)
/* Pause kept after the turn: braking jolt, filters settle. */
#define S4_TURN_SETTLE_MS (500U)
#define S4_RAD_TO_DDEG    (572.95779513082323) /* 1800 / pi */
/* int32 range for the unwrapped heading */
#define S4_HEADING_LIMIT_DDEG (2.0e9)

/* T_STATE telemetry codes (0..7, 3 bits, include/telemetry_frame.h). Defined
 * once in terrain_status.h, which other subsystems use to decode this slot. */
#define S4_TS_INIT TERRAIN_TS_INIT
/* MONITORING + terrain WAITING */
#define S4_TS_WAITING TERRAIN_TS_WAITING
/* MONITORING + terrain CLIMBING */
#define S4_TS_CLIMBING TERRAIN_TS_CLIMBING
/* MONITORING + terrain DESCENDING */
#define S4_TS_DESCENDING TERRAIN_TS_DESCENDING
#define S4_TS_RECOVERY   TERRAIN_TS_RECOVERY
#define S4_TS_FAULT      TERRAIN_TS_FAULT
#define S4_TS_STOP       TERRAIN_TS_STOP
/* MONITORING + lift latch (not overturned) */
#define S4_TS_LIFTED TERRAIN_TS_LIFTED

/* The few Buddy 2 facts Buddy 4 uses, copied once per cycle (the full
 * robot_status_t is large; it is only held briefly on the stack). */
typedef struct
{
    bool           available;
    bool           stationary;
    motion_state_t state;
    /* Controller holds the robot (STOP/fault): no motion until START. */
    bool    hold;
    int16_t speed_mm_s[2];
    /* Commanded wheel speeds: they lead the real movement. */
    int16_t target_mm_s[2];
    bool    heading_valid;
    /* Buddy 2 odometry heading, unwrapped, 0.1 deg, + = left. */
    int32_t heading_ddeg;
} s4_motion_t;

static uint32_t terrain_time_ms(void);
static void     terrain_read_motion(s4_motion_t * p_motion);
static bool     terrain_wait_start(s4_motion_t const * p_motion);
static uint8_t  terrain_inputs(uint8_t state, s4_motion_t const * p_motion,
                               bool b_wait_start, bool b_turning);
static bool     terrain_turning(buddy4_t const *    p_context,
                                s4_motion_t const * p_motion, uint32_t now_ms);
static char const * terrain_hump_label(buddy4_t const * p_context,
                                       uint8_t          terrain);
static void terrain_classify(buddy4_t * p_context, motion_class_t * p_class,
                             s4_motion_t const * p_motion);
static void terrain_lift(buddy4_t const *    p_context,
                         s4_motion_t const * p_motion, uint8_t * p_previous);
static void terrain_hump_freeze(buddy4_t *          p_context,
                                s4_motion_t const * p_motion,
                                bool *              p_previous);
static bool terrain_hump_paused(buddy4_t const *    p_context,
                                s4_motion_t const * p_motion);
static void terrain_hump_pause(buddy4_t const *    p_context,
                               s4_motion_t const * p_motion, bool * p_previous);
static uint8_t      terrain_state_code(buddy4_t const * p_context);
static void         terrain_publish(buddy4_t const *       p_context,
                                    motion_class_t const * p_class,
                                    float64_t *            p_sample_time);
static void         terrain_report(buddy4_t const *       p_context,
                                   motion_class_t const * p_class);
static bool         terrain_compass(buddy4_t const * p_context, uint8_t phase,
                                    uint32_t now_ms, int32_t * p_ddeg);
static char const * terrain_mag_reason(uint8_t reason);
#if defined(ROBO_MAG_CAL)
static void terrain_mag_cal(buddy4_t const *    p_context,
                            s4_motion_t const * p_motion);
#endif

/* Started by usermain at priority 8 when ROBO_SUBSYSTEM4 is enabled.
 * One lifecycle step per 20 ms release. Missed releases are skipped, never
 * replayed. Startup contract: the robot is level and still when it boots.
 */
void
terrain_task (INT code, void * p_context)
{
    /* Task-owned; other tasks see only the bus snapshot. */
    static buddy4_t       s_context;
    static motion_class_t s_class; /* Task-owned motion classifier. */
    s4_motion_t           motion;
    float64_t             sample_time            = -1.0;
    uint32_t              release                = 0U;
    uint32_t              last_report            = 0U;
    uint8_t               previous_state         = B4_INIT;
    uint8_t               previous_phase         = TERRAIN_WAITING;
    uint8_t               previous_class         = MC_STATIONARY;
    uint8_t               previous_lift          = LIFT_NONE;
    bool                  b_previous_frozen      = false;
    bool                  b_previous_paused      = false;
    bool                  b_previous_class_valid = false;

    (void)code;
    (void)p_context;
    release     = terrain_time_ms();
    last_report = release;
    buddy4_begin(&s_context, release);
    motion_class_reset(&s_class);
    g_s_compass      = (s4_compass_t){0};
    g_s_start_gate   = 0U; /* S4_START_NONE */
    gb_s_hump_paused = false;
    gb_s_turn_paused = false;
    gb_s_turn_seen   = false;
    g_s_turn_ms      = 0U;
#ifndef ROBO_IR_MONITOR /* Keep the IR monitor's single console line clean. */
#ifdef ROBO_ENCODER_BUILD_ID
    /* Build ID next to the Buddy 4 start-up lines, to confirm which image is
     * flashed. */
    tm_printf((UB const *)"[B4] firmware=%s\n",
              (UB const *)ROBO_ENCODER_BUILD_ID);
#endif
    tm_printf((UB const *)"[B4] GY-511 on I2C1 SDA=GP6 SCL=GP7. Keep the robot "
                          "LEVEL and STILL:\n");
    tm_printf((UB const *)"[B4] tilt reference is captured about 1-2 s after "
                          "boot (50 stable samples).\n");
#endif
    for (;;)
    {
        uint32_t now           = 0U;
        uint32_t elapsed       = 0U;
        uint32_t wait          = 0U;
        uint8_t  state         = buddy4_state(&s_context);
        uint8_t  phase         = 0U;
        uint8_t  motion_class  = MC_STATIONARY;
        bool     b_class_valid = false;

        terrain_read_motion(&motion);
        buddy4_step(&s_context,
                    terrain_inputs(state, &motion, terrain_wait_start(&motion),
                                   terrain_turning(&s_context, &motion,
                                                   terrain_time_ms())));
        terrain_lift(&s_context, &motion, &previous_lift);
        /* Pause before freeze: on the START that unfreezes, the pause still
         * sees the freeze and leaves the single "resumed" line to it. */
        terrain_hump_pause(&s_context, &motion, &b_previous_paused);
        terrain_hump_freeze(&s_context, &motion, &b_previous_frozen);
        terrain_classify(&s_context, &s_class, &motion);
        terrain_publish(&s_context, &s_class, &sample_time);
#if defined(ROBO_MAG_CAL)
        terrain_mag_cal(&s_context, &motion);
#endif
        /* Owning task consumes the latch; the comms latch is interrupt-safe. */
        if (terrain_take_completed(&s_context.terrain))
        {
            comms_latch_event(TF_EV_HUMP_PEAK);
#ifndef ROBO_IR_MONITOR
            uint8_t peak = terrain_last_peak(&s_context.terrain);

            tm_printf((UB const *)"[TERRAIN] Hump completed: peak=%d mm under "
                                  "%s wheel | run max=%d mm\n",
                      (INT)(peak & TERRAIN_PEAK_MM_MASK),
                      (0U != (peak & TERRAIN_PEAK_RIGHT)) ? "RIGHT" : "LEFT",
                      (INT)(terrain_run_max(&s_context.terrain)
                            & TERRAIN_PEAK_MM_MASK));
#endif
        }

        state         = buddy4_state(&s_context);
        phase         = terrain_status(&s_context.terrain) & TERRAIN_PHASE_MASK;
        motion_class  = motion_class_get(&s_class);
        b_class_valid = motion_class_valid(&s_class);
#ifndef ROBO_IR_MONITOR
        if (b_class_valid
            && ((motion_class != previous_class) || (!b_previous_class_valid)))
        {
            tm_printf((UB const *)"[MOTION] %s -> %s yaw=%d deg/s accel=%d "
                                  "mm/s2 imu_fwd=%d mg\n",
                      b_previous_class_valid ? motion_class_name(previous_class)
                                             : "START",
                      motion_class_name(motion_class),
                      (INT)motion_class_yaw_deg_s(&s_class),
                      (INT)motion_class_accel_mm_s2(&s_class),
                      (INT)s_context.forward_mg);
        }

        if (state != previous_state)
        {
            tm_printf((UB const *)"[B4-STATE] %s -> %s error=%d\n",
                      buddy4_state_name(previous_state),
                      buddy4_state_name(state), (INT)s_context.error);
        }

        if (phase != previous_phase)
        {
            tm_printf((UB const *)"[TERRAIN-STATE] %s -> %s\n",
                      terrain_phase_name(previous_phase),
                      terrain_phase_name(phase));
        }
#endif
        previous_state         = state;
        previous_phase         = phase;
        previous_class         = motion_class;
        b_previous_class_valid = b_class_valid;

        now = terrain_time_ms();

        if ((now - last_report) >= S4_REPORT_MS)
        {
            terrain_report(&s_context, &s_class);
            last_report = now;
        }

        elapsed = now - release;
        wait    = (elapsed < S4_PERIOD_MS)
                      ? (S4_PERIOD_MS - elapsed)
                      : (S4_PERIOD_MS - (elapsed % S4_PERIOD_MS));
        release = now + wait;
        (void)tk_dly_tsk((TMO)wait);
    }
}

/* Low monotonic milliseconds; unsigned differences handle wrap. A clock
 * failure is handled by buddy4_step() (FAULT); here it only shortens a wait.
 */
static uint32_t
terrain_time_ms (void)
{
    SYSTIM time = {0};

    return (E_OK == tk_get_otm(&time)) ? (uint32_t)time.lo : 0U;
}

/* Lifecycle input facts, taken from Buddy 2's published motion status (one
 * short copy under the bus mutex per cycle, as line following does).
 * Motion SHUTDOWN -> B4 STOP (completed peaks kept, T_STATE 6 published).
 * In INIT, level permission for reference capture is withheld only when
 * motion reports that the wheels may be moving. Unavailable status means
 * the motion task is not running (motors cannot be driven), which permits
 * capture but is never reported as confirmed stopped motors.
 * New run and fault reset are by design a power-up (statechart NEW_RUN): the
 * level reference needs a level, still boot before every run anyway.
 */
static uint8_t
terrain_inputs (uint8_t state, s4_motion_t const * p_motion, bool b_wait_start,
                bool b_turning)
{
    uint8_t start  = b_wait_start ? B4_WAIT_START : 0U;
    uint8_t result = 0U;

    if ((p_motion->available) && (SHUTDOWN == p_motion->state))
    {
        result = B4_CMD_STOP;
    }
    else if (B4_INIT != state)
    {
        /* Controller hold (dashboard STOP or any controlled stop): no humps.
         * Turning on the spot: no humps either (sideways jolt reads as tilt).
         */
        uint8_t pause
            = ((p_motion->available) && (p_motion->hold)) ? B4_HUMP_PAUSE : 0U;

        result = (uint8_t)(pause | (b_turning ? B4_TURN_PAUSE : 0U));
    }
    else if (!p_motion->available)
    {
        result = (uint8_t)(B4_LEVEL_ALLOWED | start);
    }
    else if (p_motion->stationary)
    {
        result = (uint8_t)(B4_LEVEL_ALLOWED | B4_MOTORS_STOPPED | start);
    }
    else
    {
        result = (uint8_t)(((INITIALISING == p_motion->state) ? B4_LEVEL_ALLOWED
                                                              : 0U)
                           | start);
    }

    return result;
}

/* Web Start gate (ROBO_REQUIRE_START: MQTT production images boot held until
 * the dashboard START). Buddy 4 captures its level reference, and so starts
 * hump recording and lift detection, only after START: the robot can be
 * powered up anywhere and placed on the course before the run. START = the
 * controller hold seen, then released; latched for the rest of the run (a
 * later STOP/START keeps the reference). Before any hold is seen, missing
 * motion status means the motion task is not running: no gate, as before.
 * Without ROBO_REQUIRE_START the reference is captured at boot as before. */
#define S4_START_NONE (0U) /* No hold seen yet. */
#define S4_START_HELD (1U) /* Held: waiting for START. */
/* START seen (or no Start gate): capture allowed. */
#define S4_START_GIVEN (2U)
static bool
terrain_wait_start (s4_motion_t const * p_motion)
{
#ifdef ROBO_REQUIRE_START
    uint8_t before = g_s_start_gate;

    if (S4_START_GIVEN == g_s_start_gate)
    {
        return false;
    }

    if ((p_motion->available) && (p_motion->hold))
    {
        g_s_start_gate = S4_START_HELD;
    }
    else if ((p_motion->available) && (S4_START_HELD == g_s_start_gate))
    {
        g_s_start_gate = S4_START_GIVEN;
    }
#ifndef ROBO_IR_MONITOR
    if ((S4_START_HELD == g_s_start_gate) && (S4_START_NONE == before))
    {
        tm_printf((UB const *)"[B4] waiting for dashboard START before "
                              "capturing the level reference (keep the robot "
                              "level and still after START)\n");
    }
    else if (S4_START_GIVEN == g_s_start_gate)
    {
        tm_printf((UB const *)"[B4] START: capturing level reference; keep the "
                              "robot level and still ~2 s\n");
    }
#else
    (void)before;
#endif
    /* Held, or motion reports but its boot hold has not been seen yet. */
    return (S4_START_HELD == g_s_start_gate)
           || ((S4_START_NONE == g_s_start_gate) && (p_motion->available));
#else
    (void)p_motion;
    g_s_start_gate = S4_START_GIVEN;
    return false;
#endif
}

/* m/s to rounded mm/s. Clamp before converting: a non-finite or huge speed
 * never wraps (NaN becomes 0). */
static int16_t
terrain_mm_s (float64_t m_s)
{
    float64_t mm_s = m_s * 1000.0;

    if (!(mm_s > -2000.0))
    {
        mm_s = (mm_s < 0.0) ? -2000.0 : 0.0;
    }

    if (mm_s > 2000.0)
    {
        mm_s = 2000.0;
    }

    return (int16_t)((mm_s >= 0.0) ? (mm_s + 0.5) : (mm_s - 0.5));
}

/* One short copy of the Buddy 2 status under the bus mutex; speeds in mm/s. */
static void
terrain_read_motion (s4_motion_t * p_motion)
{
    robot_status_t status;

    *p_motion = (s4_motion_t){0};

    if (!motion_get_status(&status))
    {
        return;
    }

    p_motion->available  = true;
    p_motion->stationary = status.motion.stationary;
    p_motion->state      = status.motion.state;
    p_motion->hold       = status.motion.hold;

    for (uint8_t wheel = 0U; wheel < 2U; wheel++)
    {
        p_motion->speed_mm_s[wheel] = terrain_mm_s(status.motion.speed[wheel]);
        p_motion->target_mm_s[wheel]
            = terrain_mm_s(status.motion.target[wheel]);
    }

    {
        float64_t ddeg = status.motion.heading * S4_RAD_TO_DDEG;

        /* NaN fails both comparisons; a heading beyond int32 is not used. */
        if ((ddeg > -S4_HEADING_LIMIT_DDEG) && (ddeg < S4_HEADING_LIMIT_DDEG))
        {
            p_motion->heading_valid = true;
            p_motion->heading_ddeg
                = (int32_t)((ddeg >= 0.0) ? (ddeg + 0.5) : (ddeg - 0.5));
        }
    }
}

/* Lift/overturn stop (Change Requirement: replaces IMU collision detection).
 * On a new LIFTED or OVERTURNED latch: motion_stop() at once (controlled
 * STOP: Buddy 2 brakes and holds). While still latched, if the robot is no
 * longer held (START was pressed in the air), stop it again, so the wheels
 * never drive while lifted. After the latch clears (upright and still for
 * 0.5 s) nothing restarts automatically: START resumes the run.
 * Lift detection needs Buddy 4 MONITORING: while the IMU recovers or has
 * faulted it is unavailable (sensor faults still never stop the motors).
 */
static void
terrain_lift (buddy4_t const * p_context, s4_motion_t const * p_motion,
              uint8_t * p_previous)
{
    uint8_t        lift   = buddy4_lift(p_context);
    lift_t const * p_lift = &p_context->lift;

    (void)p_lift; /* Only the console lines use it. */

    if (lift != *p_previous)
    {
        if (LIFT_NONE != lift)
        {
            motion_stop();
#ifndef ROBO_IR_MONITOR
            tm_printf((UB const *)"[LIFT] %s (upright=%d%% vertical=%d mg) -> "
                                  "motors stopped, hump in progress discarded; "
                                  "set it down level, then START\n",
                      (UB const *)lift_name(lift),
                      (INT)lift_upright_pct(p_lift),
                      (INT)lift_trigger_mg(p_lift));
#endif
        }
#ifndef ROBO_IR_MONITOR
        else if (B4_MONITORING == buddy4_state(p_context))
        {
            tm_printf((UB const *)"[LIFT] cleared: upright and still; motors "
                                  "stay stopped until START\n");
        }
        else
        {
            /* Nothing to do in any other case. */
        }
#endif
        *p_previous = lift;
    }
    else if ((LIFT_NONE != lift) && (p_motion->available) && (!p_motion->hold))
    {
        motion_stop(); /* The robot was released while still lifted. */
#ifndef ROBO_IR_MONITOR
        tm_printf((UB const *)"[LIFT] still %s: motors stopped again\n",
                  (UB const *)lift_name(lift));
#endif
    }
    else
    {
        /* Nothing to do in any other case. */
    }
}

/* Hump freeze after OVERTURNED (buddy4.h): the demo ends with the robot lifted
 * and turned over, so nothing measured while it is picked up or set down may
 * become a peak. buddy4_step() sets the freeze; START (controller hold seen,
 * then released with the robot back down) clears it. */
static void
terrain_hump_freeze (buddy4_t * p_context, s4_motion_t const * p_motion,
                     bool * p_previous)
{
    bool b_resumed
        = buddy4_hump_start(p_context, p_motion->available, p_motion->hold);
    bool b_frozen = buddy4_hump_frozen(p_context);

    if (b_frozen && (!*p_previous))
    {
#ifndef ROBO_IR_MONITOR
        tm_printf(
            (UB const *)"[TERRAIN] hump recording frozen (OVERTURNED): run max "
                        "kept at %d mm until START\n",
            (INT)(terrain_run_max(&p_context->terrain) & TERRAIN_PEAK_MM_MASK));
#endif
    }
    else if (b_resumed)
    {
#ifndef ROBO_IR_MONITOR
        tm_printf((UB const *)"[TERRAIN] hump recording resumed (START)\n");
#endif
    }
    else
    {
        /* Nothing to do in any other case. */
    }

    *p_previous = b_frozen;
}

/* Hump pause while turning on the spot (B4_TURN_PAUSE). The GY-511 has no
 * gyroscope: when a pivot or spin starts or stops, the body's angular
 * acceleration pushes the sensor sideways (about alpha x its distance from the
 * pivot), and hump height is read from the sideways component, so the jolt
 * looks like a 4-6 mm hump. A hump is crossed by driving forward, so nothing
 * real is measured while turning on the spot. Turning = the faster commanded
 * wheel at least S4_TURN_MIN_MM_S with the other stopped or opposite
 * (left x right <= 0): route turns and U-turns (spin), the line search (one
 * wheel pivot) and on-the-spot detour turns. The commanded speeds change before
 * the wheels move, so the pause starts before the jolt; it lasts until
 * S4_TURN_SETTLE_MS after the last such cycle (the stop at the end of the turn
 * jolts too). Line-following steering keeps both wheels forward: not paused.
 * Prints one line when the pause starts and one when it ends (MONITORING only;
 * a frozen robot's freeze lines already say what happens). */
static bool
terrain_turning (buddy4_t const * p_context, s4_motion_t const * p_motion,
                 uint32_t now_ms)
{
    int32_t left     = p_motion->target_mm_s[0];
    int32_t right    = p_motion->target_mm_s[1];
    int32_t fast     = (left < 0) ? -left : left;
    int32_t other    = (right < 0) ? -right : right;
    bool    b_paused = false;

    if (other > fast)
    {
        fast = other;
    }

    if ((p_motion->available) && (fast >= S4_TURN_MIN_MM_S)
        && ((left * right) <= 0))
    {
        gb_s_turn_seen = true;
        g_s_turn_ms    = now_ms;
    }

    b_paused = gb_s_turn_seen && ((now_ms - g_s_turn_ms) < S4_TURN_SETTLE_MS);
#ifndef ROBO_IR_MONITOR
    if ((b_paused != gb_s_turn_paused)
        && (B4_MONITORING == buddy4_state(p_context))
        && (!buddy4_hump_frozen(p_context)))
    {
        if (b_paused)
        {
            tm_printf((UB const *)"[TERRAIN] turning on the spot: hump "
                                  "recording paused (run max %d mm)\n",
                      (INT)(terrain_run_max(&p_context->terrain)
                            & TERRAIN_PEAK_MM_MASK));
        }
        else
        {
            tm_printf((
                UB const *)"[TERRAIN] turn finished: hump recording resumed\n");
        }
    }
#else
    (void)p_context;
#endif
    gb_s_turn_paused = b_paused;
    return b_paused;
}

/* [TILT] line terrain label: why humps are not measured, else the phase. */
static char const *
terrain_hump_label (buddy4_t const * p_context, uint8_t terrain)
{
    const char * p_result = NULL;

    if (buddy4_hump_frozen(p_context))
    {
        p_result = "FROZEN";
    }
    else if (gb_s_hump_paused)
    {
        p_result = "PAUSED";
    }
    else if (gb_s_turn_paused)
    {
        p_result = "TURNING";
    }
    else
    {
        p_result = terrain_phase_name(terrain);
    }

    return p_result;
}

/* Hump pause while stopped: same rule as the B4_HUMP_PAUSE input (MONITORING
 * with the controller hold set). */
static bool
terrain_hump_paused (buddy4_t const * p_context, s4_motion_t const * p_motion)
{
    return (B4_MONITORING == buddy4_state(p_context)) && (p_motion->available)
           && (p_motion->hold);
}

/* Console lines for the pause. The overturn freeze has its own lines and
 * takes precedence, so a frozen robot never prints both. */
static void
terrain_hump_pause (buddy4_t const * p_context, s4_motion_t const * p_motion,
                    bool * p_previous)
{
    bool b_paused = terrain_hump_paused(p_context, p_motion);

#ifndef ROBO_IR_MONITOR
    if (b_paused != *p_previous)
    {
        if (buddy4_hump_frozen(p_context))
        {
            /* The freeze lines already say what happens. */
        }
        else if (b_paused)
        {
            tm_printf(
                (UB const *)"[TERRAIN] hump recording paused (robot stopped): "
                            "run max kept at %d mm until START\n",
                (INT)(terrain_run_max(&p_context->terrain)
                      & TERRAIN_PEAK_MM_MASK));
        }
        else if (B4_MONITORING == buddy4_state(p_context))
        {
            tm_printf((UB const *)"[TERRAIN] hump recording resumed (START)\n");
        }
        else
        {
            /* Nothing to do in any other case. */
        }
    }
#endif
    *p_previous      = b_paused;
    gb_s_hump_paused = b_paused;
}

/* Motion class from the IMU (overturned latch, hump phase) and Buddy 2
 * (stationary, wheel speeds). Valid only while MONITORING with motion status
 * available. */
static void
terrain_classify (buddy4_t * p_context, motion_class_t * p_class,
                  s4_motion_t const * p_motion)
{
    motion_class_input_t input = {0};

    input.overturned = (LIFT_OVERTURNED == buddy4_lift(p_context));
    input.valid
        = (B4_MONITORING == buddy4_state(p_context)) && (p_motion->available);
    input.stationary    = p_motion->stationary;
    input.speed_mm_s[0] = p_motion->speed_mm_s[0];
    input.speed_mm_s[1] = p_motion->speed_mm_s[1];
    input.terrain_phase
        = terrain_status(&p_context->terrain) & TERRAIN_PHASE_MASK;
    input.track_mm      = (uint16_t)B4_TRACK_MM;
    input.heading_valid = p_motion->heading_valid;
    input.heading_ddeg  = p_motion->heading_ddeg;
    input.compass_valid = terrain_compass(
        p_context, input.terrain_phase, terrain_time_ms(), &input.compass_ddeg);
    motion_class_step(p_class, &input, terrain_time_ms());
    {
        int16_t  angle    = 0;
        uint16_t duration = 0U;
        int16_t  peak     = 0;
        int16_t  compass  = 0;

        if (motion_class_take_turn(p_class, &angle, &duration, &peak))
        {
#ifndef ROBO_IR_MONITOR
            INT size = (angle < 0) ? -(INT)angle : (INT)angle;

            if (motion_class_turn_compass(p_class, &compass))
            {
                INT csize = (compass < 0) ? -(INT)compass : (INT)compass;

                tm_printf((UB const *)"[TURN] %s %d.%d deg in %d.%d s (peak %d "
                                      "deg/s) compass %s%d.%d deg\n",
                          (angle < 0) ? "right" : "left", size / 10, size % 10,
                          (INT)(duration / 1000U),
                          (INT)((duration % 1000U) / 100U), (INT)peak,
                          (compass < 0) ? "-" : "", csize / 10, csize % 10);
            }
            else
            {
                tm_printf(
                    (UB const *)"[TURN] %s %d.%d deg in %d.%d s (peak %d "
                                "deg/s) compass n/a (%s)\n",
                    (angle < 0) ? "right" : "left", size / 10, size % 10,
                    (INT)(duration / 1000U), (INT)((duration % 1000U) / 100U),
                    (INT)peak,
                    terrain_mag_reason((S4_MAG_OK != g_s_compass.turn_reason)
                                           ? g_s_compass.turn_reason
                                           : S4_MAG_WINDOW));
            }
#endif
        }

        /* Outside a turn the n/a reason follows the compass's current state, so
         * a stale reason (e.g. "no data" at boot) never labels a later turn;
         * during a turn it keeps the first problem seen (set in
         * terrain_compass). */
        if (!motion_class_turn_active(p_class))
        {
            g_s_compass.turn_reason = g_s_compass.reason;
        }
    }
}

/* Lifecycle state and terrain phase combined into one 3-bit code. */
static uint8_t
terrain_state_code (buddy4_t const * p_context)
{
    uint8_t phase = terrain_status(&p_context->terrain) & TERRAIN_PHASE_MASK;

    switch (buddy4_state(p_context))
    {
        case B4_INIT:
            return S4_TS_INIT;
        case B4_MONITORING:
            if (LIFT_LIFTED == buddy4_lift(p_context))
            {
                /* OVERTURNED is reported as motion class 7. */
                return S4_TS_LIFTED;
            }

            if (TERRAIN_CLIMBING == phase)
            {
                return S4_TS_CLIMBING;
            }

            return (TERRAIN_DESCENDING == phase) ? S4_TS_DESCENDING
                                                 : S4_TS_WAITING;
        case B4_RECOVERY:
            return S4_TS_RECOVERY;
        case B4_STOP:
            return S4_TS_STOP;
        default:
            return S4_TS_FAULT;
    }
}

/* Publish the shared Terrain slot (docs/API.md): values[0] turn rate rad/s
 * (when motion-valid), [1] pitch rad, [2] highest hump peak this run, m,
 * [3] validity bits (TF_TV_*) | state code << 3. The state itself is
 * genuine data, so the slot is available even while pitch is invalid.
 * The timestamp is the time of the last valid tilt sample, not publish time.
 */
static void
terrain_publish (buddy4_t const * p_context, motion_class_t const * p_class,
                 float64_t * p_sample_time)
{
    uint32_t           word    = 0U;
    subsystem_status_t status  = {0};
    uint8_t            valid   = 0U;
    uint8_t            run_max = 0U;
    float64_t          now     = motion_hal_time();

    if (0U != (p_context->result & TILT_RESULT_VALID))
    {
        valid            = TF_TV_PITCH;
        *p_sample_time   = now;
        status.values[1] = (float64_t)tilt_angle_tenths(p_context->result, 0U)
                           * S4_TENTHS_TO_RAD;
    }

    run_max = terrain_run_max(&p_context->terrain);

    if (0U != (run_max & TERRAIN_RUN_VALID))
    {
        valid |= TF_TV_HUMP;
        status.values[2] = (float64_t)(run_max & TERRAIN_PEAK_MM_MASK) / 1000.0;
    }

    status.available = true;
    status.timestamp = (*p_sample_time >= 0.0) ? *p_sample_time : now;
    word = (uint32_t)valid | ((uint32_t)terrain_state_code(p_context) << 3U);

    if (motion_class_valid(p_class))
    {
        /* Turn rate rad/s (+ = left) and class share TERRAIN_MOTION_VALID. */
        status.values[0]
            = (float64_t)motion_class_yaw_deg_s(p_class) * S4_DEG_TO_RAD;
        word |= ((uint32_t)motion_class_get(p_class) << TERRAIN_MC_SHIFT)
                | TERRAIN_MOTION_VALID;
    }

    status.values[3] = (float64_t)word;
    (void)robot_update_subsystem(SOURCE_TERRAIN, &status);
}

/* Diagnostic console line only; the controller interface is the bus slot. */
static void
terrain_report (buddy4_t const * p_context, motion_class_t const * p_class)
{
#ifndef ROBO_IR_MONITOR
    uint8_t state   = buddy4_state(p_context);
    uint8_t terrain = terrain_status(&p_context->terrain);

    if ((B4_INIT == state) && (S4_START_GIVEN != g_s_start_gate))
    {
        tm_printf((UB const *)"[B4] INIT: waiting for dashboard START (level "
                              "reference not captured yet)\n");
    }
    else if (B4_INIT == state)
    {
        tm_printf((UB const *)"[B4] INIT: capturing level reference %d/%d; "
                              "keep level and still\n",
                  (INT)(p_context->tilt.progress & TILT_COUNT_MASK),
                  (INT)TILT_CAPTURE_SAMPLES);
    }
    else if ((B4_MONITORING == state)
             && (0U != (p_context->result & TILT_RESULT_VALID)))
    {
        INT pitch = (INT)tilt_angle_tenths(p_context->result, 0U);
        INT roll  = (INT)tilt_angle_tenths(p_context->result, 1U);
        INT height
            = (INT)tilt_side_height_tenths(&p_context->tilt, B4_TRACK_MM);
        char const * p_side = (height > 0) ? "R" : ((height < 0) ? "L" : "-");
        char const * p_pitch_sign = (pitch < 0) ? "-" : "";
        char const * p_roll_sign  = (roll < 0) ? "-" : "";

        if (pitch < 0)
        {
            pitch = -pitch;
        }

        if (roll < 0)
        {
            roll = -roll;
        }

        if (height < 0)
        {
            height = -height;
        }

        if (S4_MAG_OK == g_s_compass.reason)
        {
            tm_printf(
                (UB const *)"[TILT] pitch=%s%d.%d roll=%s%d.%d deg | %s, "
                            "%s | height=%d.%d mm %s | terrain=%s "
                            "run_max=%d mm | class=%s yaw=%d deg/s | "
                            "compass=%d.%d deg field=%d | lift=%s\n",
                p_pitch_sign, pitch / 10, pitch % 10, p_roll_sign, roll / 10,
                roll % 10, tilt_condition_name(p_context->result, 0U),
                tilt_condition_name(p_context->result, 1U), height / 10,
                height % 10, p_side, terrain_hump_label(p_context, terrain),
                (INT)(terrain_run_max(&p_context->terrain)
                      & TERRAIN_PEAK_MM_MASK),
                motion_class_valid(p_class)
                    ? motion_class_name(motion_class_get(p_class))
                    : "-",
                (INT)motion_class_yaw_deg_s(p_class),
                (INT)(g_s_compass.heading / 10),
                (INT)(g_s_compass.heading % 10), (INT)g_s_compass.field,
                (UB const *)lift_name(buddy4_lift(p_context)));
        }
        else
        {
            tm_printf(
                (UB const *)"[TILT] pitch=%s%d.%d roll=%s%d.%d deg | %s, "
                            "%s | height=%d.%d mm %s | terrain=%s "
                            "run_max=%d mm | class=%s yaw=%d deg/s | "
                            "compass n/a (%s) field=%d | lift=%s\n",
                p_pitch_sign, pitch / 10, pitch % 10, p_roll_sign, roll / 10,
                roll % 10, tilt_condition_name(p_context->result, 0U),
                tilt_condition_name(p_context->result, 1U), height / 10,
                height % 10, p_side, terrain_hump_label(p_context, terrain),
                (INT)(terrain_run_max(&p_context->terrain)
                      & TERRAIN_PEAK_MM_MASK),
                motion_class_valid(p_class)
                    ? motion_class_name(motion_class_get(p_class))
                    : "-",
                (INT)motion_class_yaw_deg_s(p_class),
                terrain_mag_reason(g_s_compass.reason), (INT)g_s_compass.field,
                (UB const *)lift_name(buddy4_lift(p_context)));
        }
    }
    else if (B4_MONITORING == state)
    {
        /* Overturned or tilted past 75 deg lands here: tilt cannot measure it,
         * the lift detector can. */
        tm_printf((UB const *)"[TILT] UNAVAILABLE (stale or unsuitable sample) "
                              "| terrain last=%s | lift=%s upright=%d%% "
                              "vertical=%d mg\n",
                  terrain_phase_name(terrain),
                  (UB const *)lift_name(buddy4_lift(p_context)),
                  (INT)lift_upright_pct(&p_context->lift),
                  (INT)lift_vertical_mg(&p_context->lift));
    }
    else
    {
        tm_printf(
            (UB const *)"[B4] %s error=%d attempts=%d; tilt unavailable\n",
            buddy4_state_name(state), (INT)p_context->error,
            (INT)buddy4_attempts(p_context));
    }
#else
    (void)p_context;
    (void)p_class;
#endif
}

static char const *
terrain_mag_reason (uint8_t reason)
{
    static char const * const g_names[]
        = {"ok", "uncal", "no data", "hump", "field", "window"};

    return (reason <= S4_MAG_WINDOW) ? g_names[reason] : "?";
}

/* Calibrated compass heading for the turn cross-check. Usable only when
 * calibrated, fresh, on level ground (not mid-hump) and with the field within
 * +/-20 % of the calibrated reference (no steel nearby). Unwrapped across
 * 0/360.
 */
static bool
terrain_compass (buddy4_t const * p_context, uint8_t phase, uint32_t now_ms,
                 int32_t * p_ddeg)
{
    int16_t axis_x   = 0;
    int16_t axis_y   = 0;
    int16_t heading  = 0;
    uint8_t reason   = S4_MAG_OK;
    bool    b_sample = buddy4_magnetic(p_context, now_ms, S4_MAG_MAX_AGE_MS,
                                       &axis_x, &axis_y);

    bool b_result = false;

    if (b_sample)
    {
        g_s_compass.field = mag_field(&g_mag_cal, axis_x, axis_y);
    }

    if (!g_mag_cal.calibrated)
    {
        reason = S4_MAG_UNCAL;
    }
    else if (!b_sample)
    {
        reason = S4_MAG_NODATA;
    }
    else if (TERRAIN_WAITING != phase)
    {
        reason = S4_MAG_HUMP;
    }
    else if (!mag_heading_ddeg(&g_mag_cal, axis_x, axis_y, &heading))
    {
        reason = S4_MAG_FIELD;
    }
    else
    {
        /* Nothing to do in any other case. */
    }

    g_s_compass.reason = reason;

    if (S4_MAG_OK != reason)
    {
        if (S4_MAG_OK == g_s_compass.turn_reason)
        {
            g_s_compass.turn_reason = reason;
        }

        b_result = false;
    }
    else
    {
        g_s_compass.unwrapped = g_s_compass.have
                                    ? mag_unwrap(g_s_compass.unwrapped, heading)
                                    : (int32_t)heading;
        g_s_compass.have      = true;
        g_s_compass.heading   = heading;
        *p_ddeg               = g_s_compass.unwrapped;
        b_result              = true;
    }

    return b_result;
}

#if defined(ROBO_MAG_CAL)
/* Magnetometer calibration build only: once Buddy 4 is monitoring and the
 * robot is still, count down 3 s, pivot LEFT 360 deg then RIGHT 360 deg back
 * at 30 deg/s through the Vehicle Controller (motion_steer, refreshed every
 * cycle; Buddy 2 alone drives the motors), stop, and print the constants for
 * mag_cal.h. Turning back the other way leaves the USB cable untwisted.
 */
#define S4_CAL_SOURCE_ID  (4001U)
#define S4_CAL_RATE_DEG_S (30.0)
#define S4_CAL_SETTLE_MS  (3000U)
#define S4_CAL_TIMEOUT_MS (40000U)
#define S4_CAL_STALL_MS   (4000U) /* Must have turned 10 deg by then. */
/* Result repeats for a late serial monitor. */
#define S4_CAL_REPEAT_MS (5000U)
#define S4_CAL_WAIT      (0U)
#define S4_CAL_SETTLE    (1U)
#define S4_CAL_LEFT      (2U)
#define S4_CAL_RIGHT     (3U)
#define S4_CAL_DONE      (4U)

typedef struct
{
    mag_cal_acc_t    acc;
    mag_cal_result_t result;
    uint32_t         phase_ms;
    uint32_t         start_ms;
    uint32_t         last_mag_ms;
    int32_t          start_heading;
    char const *     failure; /* NULL = result from mag_cal_finish(). */
    uint8_t          phase;
    uint8_t          status;
    uint8_t          countdown;
    motion_result_t  request;
} s4_cal_t;

static s4_cal_t g_s_cal;

static void
terrain_mag_cal_stop (char const * p_failure)
{
    (void)motion_steer(SOURCE_TERRAIN, S4_CAL_SOURCE_ID, 0.0, 0.0);
    motion_stop();
    g_s_cal.failure = p_failure;

    if (NULL == p_failure)
    {
        g_s_cal.status = mag_cal_finish(&g_s_cal.acc, &g_s_cal.result);
    }

    g_s_cal.phase = S4_CAL_DONE;
    /* Print at once. */
    g_s_cal.phase_ms = terrain_time_ms() - S4_CAL_REPEAT_MS;
}

static void
terrain_mag_cal_print (void)
{
    mag_cal_t const * p_cal = &g_s_cal.result.cal;

    if (NULL != g_s_cal.failure)
    {
        tm_printf((UB const *)"[MAGCAL] FAIL %s (last request result=%d). "
                              "Motors stopped; power-cycle to retry.\n",
                  g_s_cal.failure, (INT)g_s_cal.request);
        return;
    }

    tm_printf((UB const *)"[MAGCAL] %s | offset_x=%d offset_y=%d "
                          "scale_y=%d/4096 field=%d sign=%d | coverage %d/36 "
                          "bins, fit error %d.%d %%\n",
              mag_cal_status_name(g_s_cal.status), (INT)p_cal->offset_x,
              (INT)p_cal->offset_y, (INT)p_cal->scale_y_q12, (INT)p_cal->field,
              (INT)p_cal->sign, (INT)g_s_cal.result.bins_covered,
              (INT)(g_s_cal.result.fit_permille / 10U),
              (INT)(g_s_cal.result.fit_permille % 10U));

    if (MAG_CAL_PASS == g_s_cal.status)
    {
        tm_printf((UB const *)"[MAGCAL] paste into "
                              "app_program/subsystem4/mag_cal.h:\n");
        tm_printf((UB const *)"[MAGCAL] #define B4_MAG_CALIBRATED   (1)\n");
        tm_printf((UB const *)"[MAGCAL] #define B4_MAG_OFFSET_X     (%d)\n",
                  (INT)p_cal->offset_x);
        tm_printf((UB const *)"[MAGCAL] #define B4_MAG_OFFSET_Y     (%d)\n",
                  (INT)p_cal->offset_y);
        tm_printf((UB const *)"[MAGCAL] #define B4_MAG_SCALE_Y_Q12  (%d)\n",
                  (INT)p_cal->scale_y_q12);
        tm_printf((UB const *)"[MAGCAL] #define B4_MAG_FIELD        (%d)\n",
                  (INT)p_cal->field);
        tm_printf((UB const *)"[MAGCAL] #define B4_MAG_HEADING_SIGN (%d)\n",
                  (INT)p_cal->sign);
    }
}

static void
terrain_mag_cal (buddy4_t const * p_context, s4_motion_t const * p_motion)
{
    uint32_t now    = terrain_time_ms();
    int32_t  turned = p_motion->heading_ddeg - g_s_cal.start_heading;
    int16_t  axis_x = 0;
    int16_t  axis_y = 0;

    if (S4_CAL_DONE == g_s_cal.phase)
    {
        if ((now - g_s_cal.phase_ms) >= S4_CAL_REPEAT_MS)
        {
            terrain_mag_cal_print();
            g_s_cal.phase_ms = now;
        }

        return;
    }

    if (S4_CAL_WAIT == g_s_cal.phase)
    {
        if ((B4_MONITORING == buddy4_state(p_context)) && (p_motion->available)
            && (p_motion->stationary) && (p_motion->heading_valid)
            && (READY == p_motion->state))
        {
            mag_cal_begin(&g_s_cal.acc);
            g_s_cal.phase     = S4_CAL_SETTLE;
            g_s_cal.phase_ms  = now;
            g_s_cal.countdown = 4U;
            tm_printf((UB const *)"[MAGCAL] magnetometer calibration: keep 30 "
                                  "cm clear and away from steel.\n");
        }

        return;
    }

    if ((B4_MONITORING != buddy4_state(p_context)) || (!p_motion->available)
        || (!p_motion->heading_valid))
    {
        terrain_mag_cal_stop("IMU or motion status lost");
        return;
    }

    if (S4_CAL_SETTLE == g_s_cal.phase)
    {
        uint32_t left
            = ((now - g_s_cal.phase_ms) >= S4_CAL_SETTLE_MS)
                  ? 0U
                  : ((S4_CAL_SETTLE_MS - (now - g_s_cal.phase_ms) + 999U)
                     / 1000U);

        if (left < g_s_cal.countdown)
        {
            g_s_cal.countdown = (uint8_t)left;

            if (0U != left)
            {
                tm_printf((UB const *)"[MAGCAL] pivot starts in %d s\n",
                          (INT)left);
            }
        }

        if (0U == left)
        {
            g_s_cal.phase         = S4_CAL_LEFT;
            g_s_cal.start_ms      = now;
            g_s_cal.start_heading = p_motion->heading_ddeg;
            tm_printf((UB const *)"[MAGCAL] pivoting LEFT 360 deg, then RIGHT "
                                  "360 deg back\n");
        }

        return;
    }

    if ((now - g_s_cal.start_ms) > S4_CAL_TIMEOUT_MS)
    {
        terrain_mag_cal_stop("timeout");
        return;
    }

    if ((S4_CAL_LEFT == g_s_cal.phase)
        && ((now - g_s_cal.start_ms) > S4_CAL_STALL_MS) && (turned < 100))
    {
        terrain_mag_cal_stop("robot not turning");
        return;
    }

    if ((buddy4_magnetic(p_context, now, S4_MAG_MAX_AGE_MS, &axis_x, &axis_y))
        && (p_context->mag_ms != g_s_cal.last_mag_ms))
    {
        mag_cal_add(&g_s_cal.acc, axis_x, axis_y, turned);
        g_s_cal.last_mag_ms = p_context->mag_ms;
    }

    if ((S4_CAL_LEFT == g_s_cal.phase) && (turned >= 3600))
    {
        g_s_cal.phase = S4_CAL_RIGHT;
        tm_printf((UB const *)"[MAGCAL] left 360 done, turning back\n");
    }
    else if ((S4_CAL_RIGHT == g_s_cal.phase) && (turned <= 0))
    {
        terrain_mag_cal_stop(NULL);
        return;
    }
    else
    {
        /* Nothing to do in any other case. */
    }

    g_s_cal.request
        = motion_steer(SOURCE_TERRAIN, S4_CAL_SOURCE_ID, 0.0,
                       (S4_CAL_LEFT == g_s_cal.phase) ? S4_CAL_RATE_DEG_S
                                                      : -S4_CAL_RATE_DEG_S);
}
#endif

/*** end of file ***/
