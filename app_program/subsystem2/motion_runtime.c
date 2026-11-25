/* Buddy 2 runtime: owns the live motion controller, 10 ms task and motor
 * diagnostics. */
#include "../../include/float_types.h"
#include <stdint.h>
#include "motion_runtime.h"
#include "board_config.h"
#include "motion_hal.h"
#include "subsystems.h"
#include "crash_log.h"
#include "../app.h"
#include "../controller/robot_runtime.h"
#include "../system/system_runtime.h"
#include <math.h>

static motion_t g_motion;

typedef struct
{
    bool                  recorded;
    uint32_t              command_id;
    int32_t               fault_time_ms;
    motor_diagnostics_t   motors;
    encoder_diagnostics_t encoders;
} motor_fault_snapshot_t;

static motor_fault_snapshot_t g_motor_fault_snapshot;

static void            sync_encoder_fault_latch(void);
static void            capture_motor_fault(void);
static bool            get_motor_fault(const motion_status_t *  p_status,
                                       motor_fault_snapshot_t * p_snapshot);
static motion_result_t magnitude(uint32_t command_id, motion_mode_t mode,
                                 float64_t value, int sign);

motion_result_t
motion_move_forward (uint32_t command_id, float64_t metres)
{
    return magnitude(command_id, DISTANCE_MOVE, metres, 1);
}

motion_result_t
motion_move_backward (uint32_t command_id, float64_t metres)
{
    return magnitude(command_id, DISTANCE_MOVE, metres, -1);
}

motion_result_t
motion_turn_left (uint32_t command_id, float64_t radians)
{
    return magnitude(command_id, ENCODER_TURN, radians, 1);
}

motion_result_t
motion_turn_right (uint32_t command_id, float64_t radians)
{
    return magnitude(command_id, ENCODER_TURN, radians, -1);
}

motion_result_t
motion_set_wheel_speeds (uint32_t command_id, float64_t left, float64_t right)
{
    return robot_submit(SOURCE_MANUAL, command_id, WHEEL_SPEED_TRACKING, 0.0,
                        left, right);
}

motion_result_t
motion_manual_output (uint32_t command_id, float64_t left, float64_t right)
{
    return robot_submit(SOURCE_MANUAL, command_id, MANUAL_OUTPUT, 0.0, left,
                        right);
}

void
motion_runtime_init (void)
{
    motion_config_t config;
    float64_t       start;

    config            = board_motion_config();
    start             = motion_hal_time();
    bool      b_setup = motion_hal_init(&config);
    float64_t now     = motion_hal_time();

    motion_init(&g_motion, &config, motion_hal_sample(), now, b_setup);

    if (b_setup && (INITIALISING == g_motion.status.state)
        && ((now - start) > config.setup_timeout))
    {
        motion_raise_fault(&g_motion, F_SETUP_TIMEOUT, now);
    }

    if (system_encoder_fault_latched())
    {
        motion_restore_encoder_fault(&g_motion, motion_hal_sample(), now);
    }

    sync_encoder_fault_latch();
#ifdef ROBO_REQUIRE_START
    /* Buddy 1 web Start gate: boot held. INITIALISING then settles into STOPPED
       (not READY) once still, and only the dashboard START releases it
       (vehicle_controller_remote; robot_runtime_prepare sets the
       latch). */
    g_motion.status.hold = true;
#endif
}

const motion_config_t *
motion_runtime_config (void)
{
    return &g_motion.config;
}

/* Read only before tasks start; runtime readers use motion_get_status(). */
const motion_status_t *
motion_runtime_initial_status (void)
{
    return &g_motion.status;
}

void
motion_runtime_emergency_stop (void)
{
    motion_hal_emergency_stop();
}

void
motion_runtime_report_startup (void)
{
#ifdef ROBO_PRODUCTION_BUILD
    const motion_config_t * p_config = &g_motion.config;

    tm_printf(
        (const UB *)"firmware=%s hardware_confirmed=%u manual_enabled=%u "
                    "max_output_percent=%u production=1 line_task_enabled=%u "
                    "subsystem4_enabled=%u subsystem5_enabled=%u\n",
        (const UB *)ROBO_ENCODER_BUILD_ID,
        (unsigned)p_config->hardware_confirmed,
        (unsigned)p_config->manual_enabled,
        (unsigned)(p_config->max_output * 100.0), (unsigned)ROBO_LINE_TASK,
        (unsigned)ROBO_SUBSYSTEM4, (unsigned)ROBO_SUBSYSTEM5);
#ifdef ROBO_MOTION_FAULTS_DISABLED
    tm_printf((const UB *)"[MOTION] faults_enabled=0; diagnostic override "
                          "disables fault latching.\n");
#else
    tm_printf((const UB *)"[MOTION] faults_enabled=1; failures stop motors and "
                          "require checked reset/release; fault reports repeat "
                          "every 2 s.\n");
#endif
#endif
}

void
motion_task (INT code, void * p_context)
{
    float64_t next           = motion_hal_time();
    bool      b_first_sample = true;

    (void)code;
    (void)p_context;
    /* Boot banner, communications and task creation are not control-loop
     * intervals. */
    motion_begin_sampling(&g_motion, motion_hal_sample(), next);
    sync_encoder_fault_latch();
    system_watchdog_start();

    for (;;)
    {
        float64_t now = motion_hal_time();

        system_watchdog_feed(now);
        crash_mark(CRASH_MARK_MOTION, CRASH_MOTION_INPUTS);
        float64_t        previous_time = g_motion.last_time;
        encoder_sample_t sample        = motion_hal_sample();

        if (robot_runtime_wait())
        {
            robot_runtime_motion_inputs(&g_motion, sample, now);
            robot_runtime_unlock();
        }
        else
        {
            motion_raise_fault(&g_motion, F_SYSTEM, now);
        }

        /* Skip equal timestamps only on startup/reset; a stalled or backwards
         * runtime clock must still fault instead of repeating stale PWM. */
        crash_mark(CRASH_MARK_MOTION, CRASH_MOTION_STEP);

        if ((now != g_motion.last_time)
            || ((!b_first_sample) && (g_motion.last_time == previous_time)))
        {
            motion_step(&g_motion, sample, now);
        }

        b_first_sample = false;
        sync_encoder_fault_latch();
        crash_mark(CRASH_MARK_MOTION, CRASH_MOTION_OUTPUT);

        if (robot_runtime_wait())
        {
            robot_runtime_publish(&g_motion, now);

            /* A stop arriving while calculating wins before the hardware write.
             */
            if (robot_runtime_stop_pending())
            {
                robot_runtime_motion_inputs(&g_motion, sample, now);
                robot_runtime_publish(&g_motion, now);
            }

            capture_motor_fault();
            motion_hal_write(g_motion.status.output, g_motion.status.inhibited,
                             now);
            robot_runtime_unlock();
        }
        else
        {
            motion_raise_fault(&g_motion, F_SYSTEM, now);
            motion_hal_write(g_motion.status.output, true, now);
        }

        next += g_motion.config.period;
        now = motion_hal_time();

        if (next <= now)
        {
            next = now + g_motion.config.period;
        }

        crash_mark(CRASH_MARK_MOTION, CRASH_MOTION_SLEEP);
        (void)tk_dly_tsk((RELTIM)fmax(1.0, ceil((next - now) * 1000.0)));
    }
}

static void
sync_encoder_fault_latch (void)
{
    bool b_latched
        = 0U != (g_motion.fault_flags & (1U << ((unsigned)F_FEEDBACK - 1U)));

    crash_encoder_fault_update(b_latched, (UW)g_motion.status.feedback_cause);
}

/* Capture while the bus is locked, before the hardware output cutoff. */
static void
capture_motor_fault (void)
{
    if (F_NONE == g_motion.status.fault)
    {
        g_motor_fault_snapshot.recorded = false;
    }
    else if (g_motor_fault_snapshot.recorded)
    {
        /* Nothing to do. */
    }
    else
    {
        motion_hal_motor_diagnostics(&g_motor_fault_snapshot.motors);
        motion_hal_encoder_diagnostics(&g_motor_fault_snapshot.encoders);
        g_motor_fault_snapshot.command_id    = g_motion.status.debug.id;
        g_motor_fault_snapshot.fault_time_ms = g_motion.status.debug.time_ms;
        g_motor_fault_snapshot.recorded      = true;
    }
}

static bool
get_motor_fault (const motion_status_t *  p_status,
                 motor_fault_snapshot_t * p_snapshot)
{
    bool b_available;

    bool b_result = false;

    if (!robot_runtime_lock())
    {
        b_result = false;
    }
    else
    {
        b_available
            = (g_motor_fault_snapshot.recorded) && (p_status->debug.recorded)
              && (g_motor_fault_snapshot.command_id == p_status->debug.id)
              && (g_motor_fault_snapshot.fault_time_ms
                  == p_status->debug.time_ms);

        if (b_available)
        {
            *p_snapshot = g_motor_fault_snapshot;
        }

        robot_runtime_unlock();
        b_result = b_available;
    }

    return b_result;
}

static motion_result_t
magnitude (uint32_t command_id, motion_mode_t mode, float64_t value, int sign)
{
    motion_result_t result = R_INVALID;

    if ((!isfinite(value)) || (value < 0.0))
    {
        result = R_INVALID;
    }
    else
    {
        result = robot_submit(SOURCE_MANUAL, command_id, mode, value * sign,
                              0.0, 0.0);
    }

    return result;
}

#ifdef ROBO_PRODUCTION_BUILD

static void report_encoder_error(const motion_status_t * p_status);
static void report_wheel_registers(const char * p_name, unsigned wheel,
                                   const motor_diagnostics_t *   p_motor,
                                   const encoder_diagnostics_t * p_encoder);
static void report_wheel_checks(const char * p_name, unsigned wheel,
                                const motor_diagnostics_t *  p_motor,
                                const motion_fault_debug_t * p_debug);
static void report_motor_fault(const motion_status_t * p_status);
static void report_fault_cause(const motion_status_t * p_status,
                               const motion_config_t * p_config);
static void report_fault_wheels(const motion_status_t * p_status);
static void report_motion_fault(const motion_status_t * p_status,
                                const motion_config_t * p_config);

/* Task context only: the motion task already stopped the outputs and saved
 * this snapshot. Missing counts cannot distinguish a stalled wheel from a
 * disconnected/faulty encoder, so report the observed failure explicitly. */
static void
report_encoder_error (const motion_status_t * p_status)
{
    const motion_fault_debug_t * p_debug = &p_status->debug;

    if (F_FEEDBACK != p_status->fault)
    {
        return;
    }

    if ((FB_NO_PROGRESS == p_status->feedback_cause) && (p_debug->recorded))
    {
        for (unsigned wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
        {
            if (0U == (p_debug->stalled_wheels & (1U << wheel)))
            {
                /* Nothing to do. */
            }
            else
            {
                const char * p_name = (0U == wheel) ? "LEFT" : "RIGHT";

                tm_printf(
                    (const UB *)"[ERROR] %s ENCODER: wheel commanded to move "
                                "but no encoder count change for %ld ms "
                                "(limit %ld ms).\n",
                    p_name, (int32_t)p_debug->no_progress_ms[wheel],
                    (int32_t)p_debug->feedback_limit_ms[wheel]);
                tm_printf(
                    (const UB *)"[ERROR] %s ENCODER: wheel may be stalled or "
                                "encoder/wiring may be faulty.\n",
                    p_name);
            }
        }
    }
    else if (FB_INVALID_TRANSITION == p_status->feedback_cause)
    {
        for (unsigned wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
        {
            if (p_status->feedback_fault_sample.invalid[wheel]
                == p_status->feedback_previous_invalid[wheel])
            {
                /* Nothing to do. */
            }
            else
            {
                tm_printf(
                    (const UB *)"[ERROR] %s ENCODER: invalid A/B transition "
                                "detected; check encoder wiring and signal "
                                "quality.\n",
                    (0U == wheel) ? "LEFT" : "RIGHT");
            }
        }
    }
    else if (FB_UNRELIABLE == p_status->feedback_cause)
    {
        tm_printf(
            (const UB *)"[ERROR] ENCODER FEEDBACK: readings unavailable or "
                        "unreliable; affected wheel cannot be identified.\n");
    }
    else if (FB_RESTART_LATCH == p_status->feedback_cause)
    {
        tm_printf((const UB *)"[ERROR] ENCODER FAULT RETAINED: pre-restart "
                              "cause=%s(%u); manual checks and checked "
                              "reset/release required.\n",
                  motion_feedback_name(
                      (motion_feedback_cause_t)system_encoder_fault_cause()),
                  (unsigned)system_encoder_fault_cause());
    }
    else
    {
        tm_printf((const UB *)"[ERROR] ENCODER FEEDBACK: invalid movement "
                              "measurement (%s).\n",
                  motion_feedback_name(p_status->feedback_cause));
    }
}

/* Driver and encoder registers of one wheel before the cutoff. */
static void
report_wheel_registers (const char * p_name, unsigned wheel,
                        const motor_diagnostics_t *   p_motor,
                        const encoder_diagnostics_t * p_encoder)
{
    unsigned reversing
        = (unsigned)((p_motor->reversal_wait_mask >> wheel) & 1U);

    /* Targets/speeds/PID requests are already printed in the [FAULT] lines.
     */
    tm_printf((const UB *)"[PWM] %s prior_request_permille=%ld "
                          "applied_permille=%ld reverse_wait=%u enabled=%u\n",
              p_name, (int32_t)p_motor->requested_permille[wheel],
              (int32_t)p_motor->applied_permille[wheel], reversing,
              (unsigned)((p_motor->enabled_mask >> wheel) & 1U));
    tm_printf((const UB *)"[PWM REGS] %s GP%u/GP%u actual=%u,%u "
                          "expected=%u,%u top=%u div16=%u func=%u,%u\n",
              p_name, g_motor_pins[wheel][0], g_motor_pins[wheel][1],
              (unsigned)p_motor->compare[wheel][0],
              (unsigned)p_motor->compare[wheel][1],
              (unsigned)p_motor->expected_compare[wheel][0],
              (unsigned)p_motor->expected_compare[wheel][1],
              (unsigned)p_motor->top[wheel],
              (unsigned)p_motor->divider_16[wheel],
              (unsigned)p_motor->pin_function[wheel][0],
              (unsigned)p_motor->pin_function[wheel][1]);
    tm_printf((const UB *)"[ENCODER] %s GP%u/GP%u levels=%u,%u edges=%u,%u "
                          "count=%u invalid=%u irq_mask=%u,%u\n",
              p_name, p_encoder->pins[wheel][0], p_encoder->pins[wheel][1],
              (unsigned)((p_encoder->levels >> p_encoder->pins[wheel][0]) & 1U),
              (unsigned)((p_encoder->levels >> p_encoder->pins[wheel][1]) & 1U),
              (unsigned)p_encoder->edges[wheel][0],
              (unsigned)p_encoder->edges[wheel][1],
              (unsigned)p_encoder->sample.count[wheel],
              (unsigned)p_encoder->sample.invalid[wheel],
              (unsigned)p_encoder->irq_events[wheel][0],
              (unsigned)p_encoder->irq_events[wheel][1]);
}

/* What those registers say about one wheel. */
static void
report_wheel_checks (const char * p_name, unsigned wheel,
                     const motor_diagnostics_t *  p_motor,
                     const motion_fault_debug_t * p_debug)
{
    unsigned reversing
        = (unsigned)((p_motor->reversal_wait_mask >> wheel) & 1U);

    if (0U != (p_motor->mismatch_mask & (1U << wheel)))
    {
        tm_printf(
            (const UB *)"[ERROR] %s MOTOR PWM: register/pin configuration "
                        "differs from the last driver write.\n",
            p_name);
    }

    if (0U != (p_debug->stalled_wheels & (1U << wheel)))
    {
        tm_printf((const UB *)"[ERROR] %s MOTOR: commanded movement has no "
                              "encoder progress; inspect motor/load/power "
                              "and encoder.\n",
                  p_name);

        if ((!p_motor->ready) || (p_motor->inhibited) || reversing
            || (0 == p_motor->applied_permille[wheel]))
        {
            tm_printf(
                (const UB *)"[MOTOR CHECK] %s had zero/inhibited/reversing "
                            "PWM in the prior cycle; movement was not "
                            "being applied normally.\n",
                p_name);
        }
        else
        {
            tm_printf((const UB *)"[MOTOR CHECK] %s had nonzero PWM before "
                                  "cutoff; physical stall and missing "
                                  "encoder signals remain possible.\n",
                      p_name);
        }
    }

    if (((p_debug->target_mm_s[wheel] > 5) && (p_debug->speed_mm_s[wheel] < -5)
         && (INT32_MIN != p_debug->speed_mm_s[wheel]))
        || ((p_debug->target_mm_s[wheel] < -5)
            && (INT32_MIN != p_debug->target_mm_s[wheel])
            && (p_debug->speed_mm_s[wheel] > 5)))
    {
        tm_printf(
            (const UB *)"[MOTOR CHECK] %s encoder speed opposed its target "
                        "at fault; check polarity or reversal/coasting.\n",
            p_name);
    }

    if ((p_motor->cap_permille > 0)
        && (INT32_MIN != p_debug->output_permille[wheel])
        && (((int64_t)p_debug->output_permille[wheel] >= p_motor->cap_permille)
            || ((int64_t)p_debug->output_permille[wheel]
                <= -p_motor->cap_permille)))
    {
        tm_printf((const UB *)"[MOTOR CHECK] %s requested output reached "
                              "the configured cap (%ld permille).\n",
                  p_name, (int32_t)p_motor->cap_permille);
    }

    if ((0 == p_debug->target_mm_s[wheel])
        && (0 == p_debug->output_permille[wheel]))
    {
        tm_printf((const UB *)"[MOTOR] %s was not commanded to move at the "
                              "captured fault.\n",
                  p_name);
    }
}

static void
report_motor_fault (const motion_status_t * p_status)
{
    motor_fault_snapshot_t       saved;
    const motion_fault_debug_t * p_debug = &p_status->debug;

    if (!get_motor_fault(p_status, &saved))
    {
        tm_printf((const UB *)"[MOTOR] pre-stop hardware snapshot unavailable; "
                              "next report will retry.\n");
        return;
    }

    const motor_diagnostics_t *   p_motor   = &saved.motors;
    const encoder_diagnostics_t * p_encoder = &saved.encoders;

    tm_printf((const UB *)"[DRIVER] BEFORE fault cutoff: ready=%u inhibited=%u "
                          "output_age_ms=%u configured_top=%u div16=%u\n",
              (unsigned)p_motor->ready, (unsigned)p_motor->inhibited,
              (unsigned)p_motor->output_age_ms,
              (unsigned)p_motor->configured_top,
              (unsigned)p_motor->configured_divider_16);
    tm_printf((const UB *)"[ENCODER] BEFORE cutoff: initialised=%u "
                          "irq_registered=%u nvic13_enabled=%u\n",
              (unsigned)p_encoder->initialised,
              (unsigned)p_encoder->irq_registered,
              (unsigned)p_encoder->nvic_enabled);

    for (unsigned wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        const char * p_name = (0U == wheel) ? "LEFT" : "RIGHT";

        report_wheel_registers(p_name, wheel, p_motor, p_encoder);
        report_wheel_checks(p_name, wheel, p_motor, p_debug);
    }

    tm_printf((const UB *)"[MOTOR] PWM is controller evidence; motor "
                          "voltage/current are not measured.\n");
}

/* The fault in words. */
static void
report_fault_cause (const motion_status_t * p_status,
                    const motion_config_t * p_config)
{
    switch (p_status->fault)
    {
        case F_CONFIG:
            tm_printf((const UB *)"[ERROR] MOTION CONFIG: invalid "
                                  "configuration or initial timestamp; "
                                  "hardware_confirmed=%u pins_valid=%u.\n",
                      (unsigned)p_config->hardware_confirmed,
                      (unsigned)board_pins_valid());
        break;

        case F_SETUP:
            tm_printf((const UB *)"[ERROR] MOTION HARDWARE INIT: "
                                  "motion_hal_init failed; inspect "
                                  "motor/PWM/encoder/GPIO initialization.\n");
        break;

        case F_SETUP_TIMEOUT:
            tm_printf(
                (const UB *)"[ERROR] MOTION SETUP TIMEOUT: initialization or "
                            "startup qualification exceeded %lu ms.\n",
                (uint32_t)(p_config->setup_timeout * 1000.0));
            tm_printf((const UB *)"[SETUP] startup_gaps=%u max_gap_us=%lu "
                                  "stationary=%u feedback_trusted=%u\n",
                      (unsigned)p_status->startup_gaps,
                      (uint32_t)(p_status->max_startup_gap * 1000000.0),
                      (unsigned)p_status->stationary,
                      (unsigned)p_status->feedback_trusted);
        break;

        case F_TIMING:
            tm_printf(
                (const UB *)"[ERROR] MOTOR CONTROL TIMING: interval must be "
                            "finite, positive and <= %lu us; nominal=%lu us.\n",
                (uint32_t)(p_config->max_dt * 1000000.0),
                (uint32_t)(p_config->period * 1000000.0));
        break;

        case F_FEEDBACK:
            /* report_encoder_error above identifies the observed encoder
             * failure. */
        break;

        case F_DEADLINE:
            tm_printf(
                (const UB *)"[ERROR] MOTOR MOVEMENT: command deadline reached "
                            "before completion; movement_timeout_ms=%lu.\n",
                (uint32_t)(p_config->movement_timeout * 1000.0));
        break;

        case F_EXPIRED:
            tm_printf(
                (const UB *)"[ERROR] STEERING EXPIRED: wheel-speed command was "
                            "not refreshed before expiry; lifetime_ms=%lu.\n",
                (uint32_t)(p_config->command_lifetime * 1000.0));
        break;

        case F_STOP_TIMEOUT:
            tm_printf(
                (const UB *)"[ERROR] MOTOR STOP: stationary confirmation did "
                            "not arrive within %lu ms; both speed magnitudes "
                            "must be <= %lu mm/s for %lu ms.\n",
                (uint32_t)(p_config->stop_timeout * 1000.0),
                (uint32_t)(p_config->stopped_speed * 1000.0),
                (uint32_t)(p_config->settle_time * 1000.0));
        break;

        case F_FINAL_GOAL:
            tm_printf(
                (const UB *)"[ERROR] MOTOR MOVEMENT: final distance/heading "
                            "check failed; distance_tolerance_um=%lu "
                            "angle_tolerance_mrad=%lu.\n",
                (uint32_t)(p_config->distance_tolerance * 1000000.0),
                (uint32_t)(p_config->angle_tolerance * 1000.0));
        break;

        case F_EVENT_OVERFLOW:
            tm_printf((const UB *)"[ERROR] MOTION EVENTS: event buffer full "
                                  "(capacity=%u); notifications were not "
                                  "drained in time.\n",
                      (unsigned)MOTION_EVENT_CAPACITY);
        break;

        case F_SYSTEM:
            tm_printf((const UB *)"[ERROR] SYSTEM FAULT: internal "
                                  "robot_system_fault, motion synchronization "
                                  "or command decoding failure.\n");
        break;

        case F_ESTOP:
            tm_printf((const UB *)"[ERROR] EMERGENCY STOP: remote ESTOP "
                                  "command received; movement held until "
                                  "checked reset/release.\n");
        break;

        default:
            tm_printf(
                (const UB *)"[ERROR] MOTION: unrecognized fault code %u.\n",
                (unsigned)p_status->fault);
        break;
    }
}

/* What each wheel was doing at the fault. */
static void
report_fault_wheels (const motion_status_t * p_status)
{
    const motion_fault_debug_t * p_debug = &p_status->debug;

    for (unsigned wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        const char * p_name = (0U == wheel) ? "LEFT" : "RIGHT";

        tm_printf((const UB *)"[FAULT] %s stalled=%u target_mm_s=%ld "
                              "speed_mm_s=%ld request_permille=%ld\n",
                  p_name, (unsigned)((p_debug->stalled_wheels >> wheel) & 1U),
                  (int32_t)p_debug->target_mm_s[wheel],
                  (int32_t)p_debug->speed_mm_s[wheel],
                  (int32_t)p_debug->output_permille[wheel]);
        tm_printf((const UB *)"[FAULT] %s remaining_um=%ld no_progress_ms=%ld "
                              "limit_ms=%ld count=%u invalid=%u\n",
                  p_name, (int32_t)p_debug->remaining_um[wheel],
                  (int32_t)p_debug->no_progress_ms[wheel],
                  (int32_t)p_debug->feedback_limit_ms[wheel],
                  (unsigned)p_status->feedback_fault_sample.count[wheel],
                  (unsigned)p_status->feedback_fault_sample.invalid[wheel]);

        if (F_FEEDBACK == p_status->fault)
        {
            tm_printf((const UB *)"[FEEDBACK] %s previous_count=%u "
                                  "fault_count=%u previous_invalid=%u "
                                  "fault_invalid=%u reliable=%u\n",
                      p_name,
                      (unsigned)p_status->feedback_previous_count[wheel],
                      (unsigned)p_status->feedback_fault_sample.count[wheel],
                      (unsigned)p_status->feedback_previous_invalid[wheel],
                      (unsigned)p_status->feedback_fault_sample.invalid[wheel],
                      (unsigned)p_status->feedback_fault_sample.reliable);
        }
    }
}

static void
report_motion_fault (const motion_status_t * p_status,
                     const motion_config_t * p_config)
{
    const motion_fault_debug_t * p_debug = &p_status->debug;

    if (F_NONE == p_status->fault)
    {
        return;
    }

    report_encoder_error(p_status);

    if (FAULT == p_status->state)
    {
        tm_printf((const UB *)"[STATE] Buddy 2 ENTERED FAULT (state=5): "
                              "%s(%u). Motors stopped; movement held.\n",
                  motion_fault_name(p_status->fault),
                  (unsigned)p_status->fault);
    }

    report_fault_cause(p_status, p_config);
    tm_printf((const UB *)"[FAULT] %s(%u) feedback=%s(%u) id=%u held=%u; "
                          "firmware running\n",
              motion_fault_name(p_status->fault), (unsigned)p_status->fault,
              motion_feedback_name(p_status->feedback_cause),
              (unsigned)p_status->feedback_cause, (unsigned)p_status->id,
              (unsigned)p_status->hold);

    if (!p_debug->recorded)
    {
        return;
    }

    tm_printf((const UB *)"[FAULT] captured_ms=%ld mode=%s previous_state=%u "
                          "dt_us=%ld finite_goal=%u (INT32_MIN=invalid)\n",
              (int32_t)p_debug->time_ms, motion_mode_name(p_debug->mode),
              (unsigned)p_debug->state, (int32_t)p_debug->dt_us,
              (unsigned)p_debug->finite_goal);

    report_fault_wheels(p_status);
    report_motor_fault(p_status);
}

#endif

void
motion_runtime_report_fault (void)
{
#ifdef ROBO_PRODUCTION_BUILD
    robot_status_t status;

    if (motion_get_status(&status))
    {
        report_motion_fault(&status.motion, &g_motion.config);
    }
#endif
}

/*** end of file ***/
