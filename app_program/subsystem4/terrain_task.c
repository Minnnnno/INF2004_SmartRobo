/* Buddy 4 terrain task: sole owner of the GY-511 and of the buddy4_t context.
 * Integration stage 1: acceleration, tilt, terrain phases and hump peak height.
 * Magnetic heading and its calibration are not integrated yet (slot invalid).
 * The Controller owns all motor decisions; this task never commands motion.
 */
#include "robot_rtos.h" /* Standard size_t before the kernel headers. */
#include "robot.h"
#include "subsystems.h"
#include "motion_hal.h"
#include "telemetry_frame.h"
#include "../subsystem1/comms.h"
#include "buddy4.h"
#include <tm/tmonitor.h>

#define S4_PERIOD_MS    (20U)
#define S4_REPORT_MS    (500U)
#define S4_TENTHS_TO_RAD (0.0017453292519943295) /* pi / 1800 */

/* T_STATE telemetry codes (0..6, 3 bits, include/telemetry_frame.h). */
#define S4_TS_INIT       (0U)
#define S4_TS_WAITING    (1U) /* MONITORING + terrain WAITING */
#define S4_TS_CLIMBING   (2U) /* MONITORING + terrain CLIMBING */
#define S4_TS_DESCENDING (3U) /* MONITORING + terrain DESCENDING */
#define S4_TS_RECOVERY   (4U)
#define S4_TS_FAULT      (5U)
#define S4_TS_STOP       (6U)

static uint32_t terrain_time_ms(void);
static uint8_t terrain_inputs(uint8_t state);
static uint8_t terrain_state_code(buddy4_t const *p_context);
static void terrain_publish(buddy4_t const *p_context, double *p_sample_time);
static void terrain_report(buddy4_t const *p_context);

/* Started by usermain at priority 8 when ROBO_SUBSYSTEM4 is enabled.
 * One lifecycle step per 20 ms release. Missed releases are skipped, never
 * replayed. Startup contract: the robot is level and still when it boots.
 */
void terrain_task(INT code, void *p_context)
{
    static buddy4_t s_context; /* Task-owned; other tasks see only the bus snapshot. */
    double sample_time = -1.0;
    uint32_t release = 0U;
    uint32_t last_report = 0U;
    uint8_t previous_state = B4_INIT;
    uint8_t previous_phase = TERRAIN_WAITING;

    (void)code;
    (void)p_context;
    release = terrain_time_ms();
    last_report = release;
    buddy4_begin(&s_context, release);
#ifndef ROBO_IR_MONITOR /* Keep the IR monitor's single console line clean. */
#ifdef ROBO_ENCODER_BUILD_ID
    /* Build ID next to the Buddy 4 start-up lines, to confirm which image is flashed. */
    tm_printf((UB const *)"[B4] firmware=%s\n", (UB const *)ROBO_ENCODER_BUILD_ID);
#endif
    tm_printf((UB const *)"[B4] GY-511 on I2C1 SDA=GP6 SCL=GP7. Keep the robot LEVEL and STILL:\n");
    tm_printf((UB const *)"[B4] tilt reference is captured about 1-2 s after boot (50 stable samples).\n");
#endif
    for (;;)
    {
        uint32_t now = 0U;
        uint32_t elapsed = 0U;
        uint32_t wait = 0U;
        uint8_t state = buddy4_state(&s_context);
        uint8_t phase = 0U;

        buddy4_step(&s_context, terrain_inputs(state));
        terrain_publish(&s_context, &sample_time);
        /* Owning task consumes the latch; the comms latch is interrupt-safe. */
        if (terrain_take_completed(&s_context.terrain))
        {
            comms_latch_event(TF_EV_HUMP_PEAK);
#ifndef ROBO_IR_MONITOR
            uint8_t peak = terrain_last_peak(&s_context.terrain);
            tm_printf((UB const *)"[TERRAIN] Hump completed: peak=%d mm under %s wheel | run max=%d mm\n",
                      (INT)(peak & TERRAIN_PEAK_MM_MASK),
                      (0U != (peak & TERRAIN_PEAK_RIGHT)) ? "RIGHT" : "LEFT",
                      (INT)(terrain_run_max(&s_context.terrain) & TERRAIN_PEAK_MM_MASK));
#endif
        }
        state = buddy4_state(&s_context);
        phase = terrain_status(&s_context.terrain) & TERRAIN_PHASE_MASK;
#ifndef ROBO_IR_MONITOR
        if (state != previous_state)
        {
            tm_printf((UB const *)"[B4-STATE] %s -> %s error=%d\n",
                      buddy4_state_name(previous_state), buddy4_state_name(state),
                      (INT)s_context.error);
        }
        if (phase != previous_phase)
        {
            tm_printf((UB const *)"[TERRAIN-STATE] %s -> %s\n",
                      terrain_phase_name(previous_phase), terrain_phase_name(phase));
        }
#endif
        previous_state = state;
        previous_phase = phase;

        now = terrain_time_ms();
        if ((now - last_report) >= S4_REPORT_MS)
        {
            terrain_report(&s_context);
            last_report = now;
        }
        elapsed = now - release;
        wait = (elapsed < S4_PERIOD_MS) ? (S4_PERIOD_MS - elapsed)
                                        : (S4_PERIOD_MS - (elapsed % S4_PERIOD_MS));
        release = now + wait;
        (void)tk_dly_tsk((TMO)wait);
    }
}

/* Low monotonic milliseconds; unsigned differences handle wrap. A clock
 * failure is handled by buddy4_step() (FAULT); here it only shortens a wait.
 */
static uint32_t terrain_time_ms(void)
{
    SYSTIM time = {0};
    return (E_OK == tk_get_otm(&time)) ? (uint32_t)time.lo : 0U;
}

/* Lifecycle input facts, taken from Buddy 2's published motion status.
 * Only INIT needs them (level permission for reference capture), so the
 * shared bus is not locked on every cycle. Level permission is withheld only
 * when motion reports that the wheels may be moving. Unavailable status means
 * the motion task is not running (motors cannot be driven), which permits
 * capture but is never reported as confirmed stopped motors.
 * No STOP/RESET source exists yet: a latched SENSOR_FAULT needs a reboot.
 */
static uint8_t terrain_inputs(uint8_t state)
{
    robot_status_t status;

    if (B4_INIT != state)
    {
        return 0U;
    }
    if (!motion_get_status(&status))
    {
        return B4_LEVEL_ALLOWED;
    }
    if (status.motion.stationary)
    {
        return B4_LEVEL_ALLOWED | B4_MOTORS_STOPPED;
    }
    return (INITIALISING == status.motion.state) ? B4_LEVEL_ALLOWED : 0U;
}

/* Lifecycle state and terrain phase combined into one 3-bit code. */
static uint8_t terrain_state_code(buddy4_t const *p_context)
{
    uint8_t phase = terrain_status(&p_context->terrain) & TERRAIN_PHASE_MASK;

    switch (buddy4_state(p_context))
    {
        case B4_INIT:
            return S4_TS_INIT;
        case B4_MONITORING:
            if (TERRAIN_CLIMBING == phase)
            {
                return S4_TS_CLIMBING;
            }
            return (TERRAIN_DESCENDING == phase) ? S4_TS_DESCENDING : S4_TS_WAITING;
        case B4_RECOVERY:
            return S4_TS_RECOVERY;
        case B4_STOP:
            return S4_TS_STOP;
        default:
            return S4_TS_FAULT;
    }
}

/* Publish the shared Terrain slot (docs/API.md): values[0] heading rad
 * (invalid at this stage), [1] pitch rad, [2] highest hump peak this run, m,
 * [3] validity bits (TF_TV_*) | state code << 3. The state itself is
 * genuine data, so the slot is available even while pitch is invalid.
 * The timestamp is the time of the last valid tilt sample, not publish time.
 */
static void terrain_publish(buddy4_t const *p_context, double *p_sample_time)
{
    subsystem_status_t status = {0};
    uint8_t valid = 0U;
    uint8_t run_max = 0U;
    double now = motion_hal_time();

    if (0U != (p_context->result & TILT_RESULT_VALID))
    {
        valid = TF_TV_PITCH;
        *p_sample_time = now;
        status.values[1] = (double)tilt_angle_tenths(p_context->result, 0U) *
                           S4_TENTHS_TO_RAD;
    }
    run_max = terrain_run_max(&p_context->terrain);
    if (0U != (run_max & TERRAIN_RUN_VALID))
    {
        valid |= TF_TV_HUMP;
        status.values[2] = (double)(run_max & TERRAIN_PEAK_MM_MASK) / 1000.0;
    }
    status.available = true;
    status.timestamp = (*p_sample_time >= 0.0) ? *p_sample_time : now;
    status.values[3] = (double)(valid | (terrain_state_code(p_context) << 3U));
    (void)robot_update_subsystem(SOURCE_TERRAIN, &status);
}

/* Diagnostic console line only; the controller interface is the bus slot. */
static void terrain_report(buddy4_t const *p_context)
{
#ifndef ROBO_IR_MONITOR
    uint8_t state = buddy4_state(p_context);
    uint8_t terrain = terrain_status(&p_context->terrain);

    if (B4_INIT == state)
    {
        tm_printf((UB const *)"[B4] INIT: capturing level reference %d/%d; keep level and still\n",
                  (INT)(p_context->tilt.progress & TILT_COUNT_MASK), (INT)TILT_CAPTURE_SAMPLES);
    }
    else if ((B4_MONITORING == state) && (0U != (p_context->result & TILT_RESULT_VALID)))
    {
        INT pitch = (INT)tilt_angle_tenths(p_context->result, 0U);
        INT roll = (INT)tilt_angle_tenths(p_context->result, 1U);
        INT height = (INT)tilt_side_height_tenths(&p_context->tilt, B4_TRACK_MM);
        char const *side = (height > 0) ? "R" : ((height < 0) ? "L" : "-");
        char const *pitch_sign = (pitch < 0) ? "-" : "";
        char const *roll_sign = (roll < 0) ? "-" : "";
        if (pitch < 0) { pitch = -pitch; }
        if (roll < 0) { roll = -roll; }
        if (height < 0) { height = -height; }
        tm_printf((UB const *)"[TILT] pitch=%s%d.%d roll=%s%d.%d deg | %s, %s | height=%d.%d mm %s | terrain=%s slow=%d run_max=%d mm\n",
                  pitch_sign, pitch / 10, pitch % 10, roll_sign, roll / 10, roll % 10,
                  tilt_condition_name(p_context->result, 0U),
                  tilt_condition_name(p_context->result, 1U),
                  height / 10, height % 10, side,
                  terrain_phase_name(terrain), (INT)(0U != (terrain & TERRAIN_SLOW)),
                  (INT)(terrain_run_max(&p_context->terrain) & TERRAIN_PEAK_MM_MASK));
    }
    else if (B4_MONITORING == state)
    {
        tm_printf((UB const *)"[TILT] UNAVAILABLE (stale or unsuitable sample) | terrain last=%s\n",
                  terrain_phase_name(terrain));
    }
    else
    {
        tm_printf((UB const *)"[B4] %s error=%d attempts=%d; tilt unavailable\n",
                  buddy4_state_name(state), (INT)p_context->error,
                  (INT)buddy4_attempts(p_context));
    }
#else
    (void)p_context;
#endif
}
