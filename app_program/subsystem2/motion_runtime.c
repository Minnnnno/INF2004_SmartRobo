/* Buddy 2 runtime: owns the live motion controller, 10 ms task and motor diagnostics. */
#include "motion_runtime.h"
#include "board_config.h"
#include "motion_hal.h"
#include "subsystems.h"
#include "crash_log.h"
#include "../app.h"
#include "../controller/robot_runtime.h"
#include "../system/system_runtime.h"
#include <math.h>

static motion_t motion;

static void sync_encoder_fault_latch(void)
{
    bool latched = (motion.fault_flags & (1U << ((unsigned)F_FEEDBACK - 1U))) != 0U;
    crash_encoder_fault_update(latched, (UW)motion.status.feedback_cause);
}

typedef struct {
    bool recorded;
    uint32_t command_id;
    int32_t fault_time_ms;
    motor_diagnostics_t motors;
    encoder_diagnostics_t encoders;
} motor_fault_snapshot_t;
static motor_fault_snapshot_t motor_fault_snapshot;

/* Capture while the bus is locked, before the hardware output cutoff. */
static void capture_motor_fault(void)
{
    if (motion.status.fault == F_NONE) { motor_fault_snapshot.recorded = false; return; }
    if (motor_fault_snapshot.recorded) { return; }
    motion_hal_motor_diagnostics(&motor_fault_snapshot.motors);
    motion_hal_encoder_diagnostics(&motor_fault_snapshot.encoders);
    motor_fault_snapshot.command_id = motion.status.debug.id;
    motor_fault_snapshot.fault_time_ms = motion.status.debug.time_ms;
    motor_fault_snapshot.recorded = true;
}

static bool get_motor_fault(const motion_status_t *s, motor_fault_snapshot_t *snapshot)
{
    bool available;
    if (!robot_runtime_lock()) { return false; }
    available = motor_fault_snapshot.recorded && s->debug.recorded &&
                motor_fault_snapshot.command_id == s->debug.id &&
                motor_fault_snapshot.fault_time_ms == s->debug.time_ms;
    if (available) { *snapshot = motor_fault_snapshot; }
    robot_runtime_unlock();
    return available;
}

static motion_result_t magnitude(uint32_t id, motion_mode_t mode, double value, int sign)
{
    if (!isfinite(value) || (value < 0.0)) { return R_INVALID; }
    return robot_submit(SOURCE_MANUAL, id, mode, value * sign, 0.0, 0.0);
}
motion_result_t motion_move_forward(uint32_t id, double m) { return magnitude(id, DISTANCE_MOVE, m, 1); }
motion_result_t motion_move_backward(uint32_t id, double m) { return magnitude(id, DISTANCE_MOVE, m, -1); }
motion_result_t motion_turn_left(uint32_t id, double r) { return magnitude(id, ENCODER_TURN, r, 1); }
motion_result_t motion_turn_right(uint32_t id, double r) { return magnitude(id, ENCODER_TURN, r, -1); }
motion_result_t motion_set_wheel_speeds(uint32_t id, double l, double r)
{ return robot_submit(SOURCE_MANUAL, id, WHEEL_SPEED_TRACKING, 0.0, l, r); }
motion_result_t motion_manual_output(uint32_t id, double l, double r)
{ return robot_submit(SOURCE_MANUAL, id, MANUAL_OUTPUT, 0.0, l, r); }

void motion_runtime_init(void)
{
    motion_config_t config;
    double start;
    config = board_motion_config();
    start = motion_hal_time();
    bool setup = motion_hal_init(&config);
    double now = motion_hal_time();
    motion_init(&motion, &config, motion_hal_sample(), now, setup);
    if (setup && motion.status.state == INITIALISING && ((now - start) > config.setup_timeout)) {
        motion_raise_fault(&motion, F_SETUP_TIMEOUT, now);
    }
    if (system_encoder_fault_latched()) {
        motion_restore_encoder_fault(&motion, motion_hal_sample(), now);
    }
    sync_encoder_fault_latch();
}

const motion_config_t *motion_runtime_config(void) { return &motion.config; }
/* Read only before tasks start; runtime readers use motion_get_status(). */
const motion_status_t *motion_runtime_initial_status(void) { return &motion.status; }
void motion_runtime_emergency_stop(void) { motion_hal_emergency_stop(); }

void motion_runtime_report_startup(void)
{
#ifdef ROBO_PRODUCTION_BUILD
    const motion_config_t *config = &motion.config;
    tm_printf((const UB *)"firmware=%s hardware_confirmed=%u manual_enabled=%u max_output_percent=%u production=1 line_task_enabled=%u subsystem4_enabled=%u subsystem5_enabled=%u\n",
              (const UB *)ROBO_ENCODER_BUILD_ID, (unsigned)config->hardware_confirmed,
              (unsigned)config->manual_enabled, (unsigned)(config->max_output * 100.0),
              (unsigned)ROBO_LINE_TASK, (unsigned)ROBO_SUBSYSTEM4, (unsigned)ROBO_SUBSYSTEM5);
#ifdef ROBO_MOTION_FAULTS_DISABLED
    tm_printf((const UB *)"[MOTION] faults_enabled=0; diagnostic override disables fault latching.\n");
#else
    tm_printf((const UB *)"[MOTION] faults_enabled=1; failures stop motors and require checked reset/release; fault reports repeat every 2 s.\n");
#endif
#endif
}

void motion_task(INT code, void *context)
{
    double next = motion_hal_time();
    bool first_sample = true;
    (void)code; (void)context;
    /* Boot banner, communications and task creation are not control-loop intervals. */
    motion_begin_sampling(&motion, motion_hal_sample(), next);
    sync_encoder_fault_latch();
    system_watchdog_start();
    for (;;) {
        double now = motion_hal_time();
        system_watchdog_feed(now);
        crash_mark(CRASH_MARK_MOTION, CRASH_MOTION_INPUTS);
        double previous_time = motion.last_time;
        encoder_sample_t sample = motion_hal_sample();
        if (robot_runtime_wait()) {
            robot_runtime_motion_inputs(&motion, sample, now);
            robot_runtime_unlock();
        } else { motion_raise_fault(&motion, F_SYSTEM, now); }
        /* Skip equal timestamps only on startup/reset; a stalled or backwards
         * runtime clock must still fault instead of repeating stale PWM. */
        crash_mark(CRASH_MARK_MOTION, CRASH_MOTION_STEP);
        if (now != motion.last_time || (!first_sample && motion.last_time == previous_time)) {
            motion_step(&motion, sample, now);
        }
        first_sample = false;
        sync_encoder_fault_latch();
        crash_mark(CRASH_MARK_MOTION, CRASH_MOTION_OUTPUT);
        if (robot_runtime_wait()) {
            robot_runtime_publish(&motion, now);
            /* A stop arriving while calculating wins before the hardware write. */
            if (robot_runtime_stop_pending()) {
                robot_runtime_motion_inputs(&motion, sample, now);
                robot_runtime_publish(&motion, now);
            }
            capture_motor_fault();
            motion_hal_write(motion.status.output, motion.status.inhibited, now);
            robot_runtime_unlock();
        } else {
            motion_raise_fault(&motion, F_SYSTEM, now);
            motion_hal_write(motion.status.output, true, now);
        }
        next += motion.config.period;
        now = motion_hal_time();
        if (next <= now) { next = now + motion.config.period; }
        crash_mark(CRASH_MARK_MOTION, CRASH_MOTION_SLEEP);
        (void)tk_dly_tsk((RELTIM)fmax(1.0, ceil((next - now) * 1000.0)));
    }
}

#ifdef ROBO_PRODUCTION_BUILD
/* Task context only: the motion task already stopped the outputs and saved
 * this snapshot. Missing counts cannot distinguish a stalled wheel from a
 * disconnected/faulty encoder, so report the observed failure explicitly. */
static void report_encoder_error(const motion_status_t *s)
{
    const motion_fault_debug_t *d = &s->debug;
    if (s->fault != F_FEEDBACK) { return; }
    if (s->feedback_cause == FB_NO_PROGRESS && d->recorded) {
        for (unsigned w = 0U; w < 2U; ++w) {
            if ((d->stalled_wheels & (1U << w)) == 0U) { continue; }
            const char *name = w == 0U ? "LEFT" : "RIGHT";
            tm_printf((const UB *)"[ERROR] %s ENCODER: wheel commanded to move but no encoder count change for %ld ms (limit %ld ms).\n",
                      name, (long)d->no_progress_ms[w], (long)d->feedback_limit_ms[w]);
            tm_printf((const UB *)"[ERROR] %s ENCODER: wheel may be stalled or encoder/wiring may be faulty.\n", name);
        }
    } else if (s->feedback_cause == FB_INVALID_TRANSITION) {
        for (unsigned w = 0U; w < 2U; ++w) {
            if (s->feedback_fault_sample.invalid[w] == s->feedback_previous_invalid[w]) { continue; }
            tm_printf((const UB *)"[ERROR] %s ENCODER: invalid A/B transition detected; check encoder wiring and signal quality.\n",
                      w == 0U ? "LEFT" : "RIGHT");
        }
    } else if (s->feedback_cause == FB_UNRELIABLE) {
        tm_printf((const UB *)"[ERROR] ENCODER FEEDBACK: readings unavailable or unreliable; affected wheel cannot be identified.\n");
    } else if (s->feedback_cause == FB_RESTART_LATCH) {
        tm_printf((const UB *)"[ERROR] ENCODER FAULT RETAINED: pre-restart cause=%s(%u); manual checks and checked reset/release required.\n",
                  motion_feedback_name((motion_feedback_cause_t)system_encoder_fault_cause()),
                  (unsigned)system_encoder_fault_cause());
    } else {
        tm_printf((const UB *)"[ERROR] ENCODER FEEDBACK: invalid movement measurement (%s).\n",
                  motion_feedback_name(s->feedback_cause));
    }
}

static void report_motor_fault(const motion_status_t *s)
{
    motor_fault_snapshot_t saved;
    const motion_fault_debug_t *d = &s->debug;
    if (!get_motor_fault(s, &saved)) {
        tm_printf((const UB *)"[MOTOR] pre-stop hardware snapshot unavailable; next report will retry.\n");
        return;
    }
    const motor_diagnostics_t *p = &saved.motors;
    const encoder_diagnostics_t *e = &saved.encoders;
    tm_printf((const UB *)"[DRIVER] BEFORE fault cutoff: ready=%u inhibited=%u output_age_ms=%u configured_top=%u div16=%u\n",
              (unsigned)p->ready, (unsigned)p->inhibited, (unsigned)p->output_age_ms,
              (unsigned)p->configured_top, (unsigned)p->configured_divider_16);
    tm_printf((const UB *)"[ENCODER] BEFORE cutoff: initialised=%u irq_registered=%u nvic13_enabled=%u\n",
              (unsigned)e->initialised, (unsigned)e->irq_registered, (unsigned)e->nvic_enabled);
    for (unsigned w = 0U; w < 2U; ++w) {
        const char *name = w == 0U ? "LEFT" : "RIGHT";
        unsigned reversing = (unsigned)((p->reversal_wait_mask >> w) & 1U);
        /* Targets/speeds/PID requests are already printed in the [FAULT] lines. */
        tm_printf((const UB *)"[PWM] %s prior_request_permille=%ld applied_permille=%ld reverse_wait=%u enabled=%u\n",
                  name, (long)p->requested_permille[w], (long)p->applied_permille[w], reversing,
                  (unsigned)((p->enabled_mask >> w) & 1U));
        tm_printf((const UB *)"[PWM REGS] %s GP%u/GP%u actual=%u,%u expected=%u,%u top=%u div16=%u func=%u,%u\n",
                  name, motor_pins[w][0], motor_pins[w][1], (unsigned)p->compare[w][0], (unsigned)p->compare[w][1],
                  (unsigned)p->expected_compare[w][0], (unsigned)p->expected_compare[w][1],
                  (unsigned)p->top[w], (unsigned)p->divider_16[w],
                  (unsigned)p->pin_function[w][0], (unsigned)p->pin_function[w][1]);
        tm_printf((const UB *)"[ENCODER] %s GP%u/GP%u levels=%u,%u edges=%u,%u count=%u invalid=%u irq_mask=%u,%u\n",
                  name, e->pins[w][0], e->pins[w][1],
                  (unsigned)((e->levels >> e->pins[w][0]) & 1U), (unsigned)((e->levels >> e->pins[w][1]) & 1U),
                  (unsigned)e->edges[w][0], (unsigned)e->edges[w][1], (unsigned)e->sample.count[w],
                  (unsigned)e->sample.invalid[w], (unsigned)e->irq_events[w][0], (unsigned)e->irq_events[w][1]);
        if ((p->mismatch_mask & (1U << w)) != 0U) {
            tm_printf((const UB *)"[ERROR] %s MOTOR PWM: register/pin configuration differs from the last driver write.\n", name);
        }
        if ((d->stalled_wheels & (1U << w)) != 0U) {
            tm_printf((const UB *)"[ERROR] %s MOTOR: commanded movement has no encoder progress; inspect motor/load/power and encoder.\n", name);
            if (!p->ready || p->inhibited || reversing || p->applied_permille[w] == 0) {
                tm_printf((const UB *)"[MOTOR CHECK] %s had zero/inhibited/reversing PWM in the prior cycle; movement was not being applied normally.\n", name);
            } else {
                tm_printf((const UB *)"[MOTOR CHECK] %s had nonzero PWM before cutoff; physical stall and missing encoder signals remain possible.\n", name);
            }
        }
        if ((d->target_mm_s[w] > 5 && d->speed_mm_s[w] < -5 && d->speed_mm_s[w] != INT32_MIN) ||
            (d->target_mm_s[w] < -5 && d->target_mm_s[w] != INT32_MIN && d->speed_mm_s[w] > 5)) {
            tm_printf((const UB *)"[MOTOR CHECK] %s encoder speed opposed its target at fault; check polarity or reversal/coasting.\n", name);
        }
        if (p->cap_permille > 0 && d->output_permille[w] != INT32_MIN &&
            ((int64_t)d->output_permille[w] >= p->cap_permille || (int64_t)d->output_permille[w] <= -p->cap_permille)) {
            tm_printf((const UB *)"[MOTOR CHECK] %s requested output reached the configured cap (%ld permille).\n", name, (long)p->cap_permille);
        }
        if (d->target_mm_s[w] == 0 && d->output_permille[w] == 0) {
            tm_printf((const UB *)"[MOTOR] %s was not commanded to move at the captured fault.\n", name);
        }
    }
    tm_printf((const UB *)"[MOTOR] PWM is controller evidence; motor voltage/current are not measured.\n");
}

static void report_motion_fault(const motion_status_t *s, const motion_config_t *config)
{
    const motion_fault_debug_t *d = &s->debug;
    if (s->fault == F_NONE) { return; }
    report_encoder_error(s);
    if (s->state == FAULT) {
        tm_printf((const UB *)"[STATE] Buddy 2 ENTERED FAULT (state=5): %s(%u). Motors stopped; movement held.\n",
                  motion_fault_name(s->fault), (unsigned)s->fault);
    }
    switch (s->fault) {
    case F_CONFIG:
        tm_printf((const UB *)"[ERROR] MOTION CONFIG: invalid configuration or initial timestamp; hardware_confirmed=%u pins_valid=%u.\n",
                  (unsigned)config->hardware_confirmed, (unsigned)board_pins_valid());
        break;
    case F_SETUP:
        tm_printf((const UB *)"[ERROR] MOTION HARDWARE INIT: motion_hal_init failed; inspect motor/PWM/encoder/GPIO initialization.\n");
        break;
    case F_SETUP_TIMEOUT:
        tm_printf((const UB *)"[ERROR] MOTION SETUP TIMEOUT: initialization or startup qualification exceeded %lu ms.\n",
                  (unsigned long)(config->setup_timeout * 1000.0));
        tm_printf((const UB *)"[SETUP] startup_gaps=%u max_gap_us=%lu stationary=%u feedback_trusted=%u\n",
                  (unsigned)s->startup_gaps, (unsigned long)(s->max_startup_gap * 1000000.0),
                  (unsigned)s->stationary, (unsigned)s->feedback_trusted);
        break;
    case F_TIMING:
        tm_printf((const UB *)"[ERROR] MOTOR CONTROL TIMING: interval must be finite, positive and <= %lu us; nominal=%lu us.\n",
                  (unsigned long)(config->max_dt * 1000000.0), (unsigned long)(config->period * 1000000.0));
        break;
    case F_FEEDBACK:
        /* report_encoder_error above identifies the observed encoder failure. */
        break;
    case F_DEADLINE:
        tm_printf((const UB *)"[ERROR] MOTOR MOVEMENT: command deadline reached before completion; movement_timeout_ms=%lu.\n",
                  (unsigned long)(config->movement_timeout * 1000.0));
        break;
    case F_EXPIRED:
        tm_printf((const UB *)"[ERROR] STEERING EXPIRED: wheel-speed command was not refreshed before expiry; lifetime_ms=%lu.\n",
                  (unsigned long)(config->command_lifetime * 1000.0));
        break;
    case F_STOP_TIMEOUT:
        tm_printf((const UB *)"[ERROR] MOTOR STOP: stationary confirmation did not arrive within %lu ms; both speed magnitudes must be <= %lu mm/s for %lu ms.\n",
                  (unsigned long)(config->stop_timeout * 1000.0), (unsigned long)(config->stopped_speed * 1000.0),
                  (unsigned long)(config->settle_time * 1000.0));
        break;
    case F_FINAL_GOAL:
        tm_printf((const UB *)"[ERROR] MOTOR MOVEMENT: final distance/heading check failed; distance_tolerance_um=%lu angle_tolerance_mrad=%lu.\n",
                  (unsigned long)(config->distance_tolerance * 1000000.0), (unsigned long)(config->angle_tolerance * 1000.0));
        break;
    case F_EVENT_OVERFLOW:
        tm_printf((const UB *)"[ERROR] MOTION EVENTS: event buffer full (capacity=%u); notifications were not drained in time.\n",
                  (unsigned)MOTION_EVENT_CAPACITY);
        break;
    case F_SYSTEM:
        tm_printf((const UB *)"[ERROR] SYSTEM FAULT: internal robot_system_fault, motion synchronization or command decoding failure.\n");
        break;
    case F_ESTOP:
        tm_printf((const UB *)"[ERROR] EMERGENCY STOP: remote ESTOP command received; movement held until checked reset/release.\n");
        break;
    default:
        tm_printf((const UB *)"[ERROR] MOTION: unrecognized fault code %u.\n", (unsigned)s->fault);
        break;
    }
    tm_printf((const UB *)"[FAULT] %s(%u) feedback=%s(%u) id=%u held=%u; firmware running\n",
              motion_fault_name(s->fault), (unsigned)s->fault,
              motion_feedback_name(s->feedback_cause), (unsigned)s->feedback_cause,
              (unsigned)s->id, (unsigned)s->hold);
    if (!d->recorded) { return; }
    tm_printf((const UB *)"[FAULT] captured_ms=%ld mode=%s previous_state=%u dt_us=%ld finite_goal=%u (INT32_MIN=invalid)\n",
              (long)d->time_ms, motion_mode_name(d->mode), (unsigned)d->state,
              (long)d->dt_us, (unsigned)d->finite_goal);
    for (unsigned w = 0U; w < 2U; ++w) {
        const char *name = w == 0U ? "LEFT" : "RIGHT";
        tm_printf((const UB *)"[FAULT] %s stalled=%u target_mm_s=%ld speed_mm_s=%ld request_permille=%ld\n",
                  name, (unsigned)((d->stalled_wheels >> w) & 1U),
                  (long)d->target_mm_s[w], (long)d->speed_mm_s[w], (long)d->output_permille[w]);
        tm_printf((const UB *)"[FAULT] %s remaining_um=%ld no_progress_ms=%ld limit_ms=%ld count=%u invalid=%u\n",
                  name, (long)d->remaining_um[w], (long)d->no_progress_ms[w],
                  (long)d->feedback_limit_ms[w], (unsigned)s->feedback_fault_sample.count[w],
                  (unsigned)s->feedback_fault_sample.invalid[w]);
        if (s->fault == F_FEEDBACK) {
            tm_printf((const UB *)"[FEEDBACK] %s previous_count=%u fault_count=%u previous_invalid=%u fault_invalid=%u reliable=%u\n",
                      name, (unsigned)s->feedback_previous_count[w], (unsigned)s->feedback_fault_sample.count[w],
                      (unsigned)s->feedback_previous_invalid[w], (unsigned)s->feedback_fault_sample.invalid[w],
                      (unsigned)s->feedback_fault_sample.reliable);
        }
    }
    report_motor_fault(s);
}

#endif

void motion_runtime_report_fault(void)
{
#ifdef ROBO_PRODUCTION_BUILD
    robot_status_t status;
    if (motion_get_status(&status)) { report_motion_fault(&status.motion, &motion.config); }
#endif
}
