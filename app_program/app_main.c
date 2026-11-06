#include "robot_rtos.h"
#include "board_config.h"
#include "motion_hal.h"
#include "robot.h"
#include "subsystems.h"
#include "app.h"
#include "subsystem1/comms.h"
#include "crash_log.h"
#include "stack_debug.h"
_Static_assert((int)COMMS_CMD_START == (int)REMOTE_START && (int)COMMS_CMD_STOP == (int)REMOTE_STOP &&
               (int)COMMS_CMD_ESTOP == (int)REMOTE_ESTOP, "Buddy 1 command mapping changed");
#include <math.h>
#ifdef ROBO_PRODUCTION_BUILD
#include <tm/tmonitor.h>
#endif

static robot_bus_t bus;
static motion_t motion;
static ID status_mutex;
static bool running;
typedef struct {
    bool recorded;
    uint32_t command_id;
    int32_t fault_time_ms;
    motor_diagnostics_t motors;
    encoder_diagnostics_t encoders;
} motor_fault_snapshot_t;
static motor_fault_snapshot_t motor_fault_snapshot;

/* The private idle context also carries interrupts on this MSP-based port. */
extern UW knl_idle_stack_used(UINT core);
extern UW knl_idle_stack_size(void);
extern BOOL knl_idle_stack_intact(UINT core);
extern BOOL knl_guards_intact(void);

/* Buddy 1's communications owner uses the shared task-creation helper. */
ID app_start_task(FP entry, PRI priority, UW processor, const char *name)
{
    T_CTSK task = {.tskatr = TA_HLNG | TA_RNG0, .task = entry,
                  .itskpri = priority, .stksz = TASK_STACK_SIZE};
    ID id;
    ER result;
    (void)processor; (void)name; /* This integration requires SMP=OFF. */
    id = tk_cre_tsk(&task);
    if (id <= 0) { return id; }
    result = tk_sta_tsk(id, 0);
    if (result < E_OK) { (void)tk_del_tsk(id); return result; }
    return id;
}

static bool lock_bus(void)
{
    return (status_mutex > 0) && (tk_loc_mtx(status_mutex, TMO_POL) == E_OK);
}
static void unlock_bus(void) { (void)tk_unl_mtx(status_mutex); }

/* Motion task, with the bus locked, before the fault's zero-output write.
 * Keep this separately from motion_status_t to avoid growing every status copy. */
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
    if (!lock_bus()) { return false; }
    available = motor_fault_snapshot.recorded && s->debug.recorded &&
                motor_fault_snapshot.command_id == s->debug.id &&
                motor_fault_snapshot.fault_time_ms == s->debug.time_ms;
    if (available) { *snapshot = motor_fault_snapshot; }
    unlock_bus();
    return available;
}

static motion_result_t submit(request_source_t source, uint32_t id, motion_mode_t mode,
                             double value, double left, double right, double angle, double duration)
{
    robot_request_t request;
    motion_result_t result;
    double now = motion_hal_time();
    if (!running || !lock_bus()) { return R_BUSY; }
    request.source = source;
    request.command = (motion_command_t){.id=id, .generation=bus.generation, .mode=mode,
        .value=value, .speed={left, right}, .expires=now + motion.config.command_lifetime,
        .deadline=now + ((mode == MANUAL_OUTPUT) ? motion.config.manual_timeout : motion.config.movement_timeout),
        .angle=angle, .duration=duration};
    result = robot_request(&bus, &request);
    unlock_bus();
    return result;
}

motion_result_t robot_submit(request_source_t source, uint32_t id, motion_mode_t mode,
                             double value, double left, double right)
{ return submit(source, id, mode, value, left, right, 0.0, 0.0); }
motion_result_t robot_submit_arc(request_source_t source, uint32_t id, double metres,
                                 double radians, double seconds, double left, double right)
{ return submit(source, id, ARC_MOVE, metres, left, right, radians, seconds); }

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

static void control(robot_control_t request, bool checks, bool rest)
{
    /* Priority inheritance, bounded short critical sections. Never enqueue stop. */
    if (running && (tk_loc_mtx(status_mutex, TMO_FEVR) == E_OK)) {
        robot_control(&bus, request, checks, rest);
        unlock_bus();
    }
}
void motion_stop(void) { control(CONTROL_STOP, false, false); }
void motion_controller_release(bool checks) { control(CONTROL_RELEASE, checks, false); }
void motion_fault_reset(bool cleared, bool rest) { control(CONTROL_RESET, cleared, rest); }
void motion_request_shutdown(void) { control(CONTROL_SHUTDOWN, false, false); }
void robot_system_fault(void) { control(CONTROL_FAULT, false, false); }
bool motion_get_status(robot_status_t *status)
{
    if ((status == NULL) || !running || !lock_bus()) { return false; }
    *status = bus.status;
    unlock_bus();
    return true;
}

bool robot_update_subsystem(request_source_t source, const subsystem_status_t *status)
{
    subsystem_status_t *destination = NULL;
    unsigned i;
    if ((status == NULL) || !isfinite(status->timestamp)) { return false; }
    for (i = 0; i < 4U; ++i) { if (!isfinite(status->values[i])) { return false; } }
    if (!running || !lock_bus()) { return false; }
    switch (source) {
    case SOURCE_LINE: destination = &bus.status.line; break;
    case SOURCE_BARCODE: destination = &bus.status.barcode; break;
    case SOURCE_TERRAIN: destination = &bus.status.terrain; break;
    case SOURCE_OBSTACLE: destination = &bus.status.obstacle; break;
    case SOURCE_WIFI: destination = &bus.status.wifi; break;
    default: break;
    }
    if (destination != NULL) { *destination = *status; }
    unlock_bus();
    return destination != NULL;
}

/* Freeze catcher: TIMER alarm 2, routed to the NMI, checks that motion_task still
 * feeds the watchdog. A stall is recorded with the code the CPU was running before
 * the hardware watchdog fires. As an NMI it also sees code that masked interrupts,
 * a handler spinning at priority 0 (Default_Handler is while(1)) and a hung hard
 * fault handler. A plain watchdog timeout means no NMI report was saved;
 * it does not distinguish a core lockup from failure of the catcher itself. */
#define FREEZE_ALARM 2U
#define SYSCFG_PROC0_NMI_MASK 0x40004000U
#define EXC_NMI 2U
static volatile UW last_feed_us;
static volatile UW freeze_check_count;
extern UW exchdr_tbl[]; /* Kernel's RAM vector table; VTOR points here. */
extern void robo_freeze_entry(void);
void robo_freeze_check(const UW *frame);

/* NMI entry: the exception frame is at the stack pointer (tasks and handlers
 * both run on MSP here). Returns through the C function with EXC_RETURN still
 * in lr. No kernel services: it can interrupt the kernel anywhere. */
__asm__(
"	.syntax	unified			\n"
"	.thumb				\n"
"	.text				\n"
"	.align	2			\n"
"	.globl	robo_freeze_entry	\n"
"	.thumb_func			\n"
"robo_freeze_entry:			\n"
"	mov	r0, lr			\n"
"	movs	r1, #4			\n"
"	tst	r0, r1			\n"
"	beq	1f			\n"
"	mrs	r0, psp			\n"
"	b	2f			\n"
"1:	mrs	r0, msp			\n"
"2:	ldr	r1, =robo_freeze_check	\n"
"	bx	r1			\n"
"	.ltorg				\n"
);

__attribute__((used)) void robo_freeze_check(const UW *frame)
{
    UW now = in_w(TIMER_TIMERAWL);
    UW where;
    ++freeze_check_count;
    out_w(TIMER_INTR, 1UL << FREEZE_ALARM);
    out_w(TIMER_ALARM2, now + CRASH_FREEZE_CHECK_US);
    if ((now - last_feed_us) < CRASH_FREEZE_LIMIT_US) { return; }
    /* Save the frame before consulting scheduler memory: stack corruption can
     * invalidate its current-TCB pointer and fault even inside this NMI.
     * Task 255 means the lookup did not finish; the watchdog still keeps PC/LR. */
    where = ((frame[7] & 0x3fU) << 24U) | (0xffU << 16U) |
            ((in_w(CRASH_MARK_MOTION) & 0xffU) << 8U) | (in_w(CRASH_MARK_OBSTACLE) & 0xffU);
    crash_log_save(CRASH_MAGIC_FREEZE, frame[6], frame[5], where);
    where = (where & ~(0xffU << 16U)) | (((UW)tk_get_tid() & 0xffU) << 16U);
    out_w(CRASH_WDT_SCRATCH0 + 12U, where);
    crash_log_reboot();
}

static void freeze_catcher_start(void)
{
    UINT state;
    /* The kernel leaves SYSCFG, which holds the NMI routing, in reset. */
    clr_w(RESETS_RESET, RESETS_RESET_SYSCFG);
    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_SYSCFG) == 0U) { }
    DI(state);
    last_feed_us = in_w(TIMER_TIMERAWL);
    exchdr_tbl[EXC_NMI] = (UW)robo_freeze_entry;
    out_w(TIMER_INTR, 1UL << FREEZE_ALARM);
    set_w(TIMER_INTE, 1UL << FREEZE_ALARM);
    out_w(TIMER_ALARM2, last_feed_us + CRASH_FREEZE_CHECK_US);
    /* NMI only: the IRQ stays disabled in the NVIC. */
    set_w(SYSCFG_PROC0_NMI_MASK, 1UL << FREEZE_ALARM);
    EI(state);
}

static void motion_task(INT code, void *context)
{
    double next = motion_hal_time();
    bool first_sample = true;
    (void)code; (void)context;
    /* Boot banner, communications and task creation are not control-loop intervals. */
    motion_begin_sampling(&motion, motion_hal_sample(), next);
    crash_watchdog_start(1000U); /* Reboot and report if this loop stalls for 1 s. */
    freeze_catcher_start();
    for (;;) {
        double now = motion_hal_time();
        crash_watchdog_feed(1000U);
        last_feed_us = in_w(TIMER_TIMERAWL);
        crash_mark(CRASH_MARK_FEED_MS, (UW)(now * 1000.0));
        crash_mark(CRASH_MARK_MOTION, CRASH_MOTION_INPUTS);
        double previous_time = motion.last_time;
        encoder_sample_t sample = motion_hal_sample();
        if (tk_loc_mtx(status_mutex, TMO_FEVR) == E_OK) {
            robot_motion_inputs(&bus, &motion, sample, now);
            unlock_bus();
        } else { motion_raise_fault(&motion, F_SYSTEM, now); }
        /* Skip equal timestamps only on startup/reset; a stalled or backwards
         * runtime clock must still fault instead of repeating stale PWM. */
        crash_mark(CRASH_MARK_MOTION, CRASH_MOTION_STEP);
        if (now != motion.last_time || (!first_sample && motion.last_time == previous_time)) {
            motion_step(&motion, sample, now);
        }
        first_sample = false;
        crash_mark(CRASH_MARK_MOTION, CRASH_MOTION_OUTPUT);
        if (tk_loc_mtx(status_mutex, TMO_FEVR) == E_OK) {
            robot_publish(&bus, &motion, now);
            /* A stop arriving while calculating wins before the hardware write. */
            if ((bus.controls & (CONTROL_STOP | CONTROL_FAULT | CONTROL_SHUTDOWN)) != 0U) {
                robot_motion_inputs(&bus, &motion, sample, now);
                robot_publish(&bus, &motion, now);
            }
            capture_motor_fault();
            motion_hal_write(motion.status.output, motion.status.inhibited, now);
            unlock_bus();
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

static void controller_task(INT code, void *context)
{
    (void)code; (void)context;
    for (;;) {
        /* Consume Buddy 1's command buffer without blocking motion scheduling. */
        for (unsigned budget = 0U; budget < 4U; ++budget) {
            COMMS_CMD command;
            INT received = tk_rcv_mbf(mbf_comms_cmd, &command, TMO_POL);
            if (received < E_OK) { break; }
            if (received != (INT)sizeof(command)) { continue; }
            if (tk_loc_mtx(status_mutex, TMO_FEVR) == E_OK) {
                (void)vehicle_controller_remote_command(&bus, (robot_remote_command_t)command);
                unlock_bus();
            }
        }
        if (lock_bus()) { vehicle_controller_cycle(&bus, motion_hal_time()); unlock_bus(); }
        (void)tk_dly_tsk(10U);
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

static void report_motion_fault(const motion_status_t *s)
{
    const motion_fault_debug_t *d = &s->debug;
    if (s->fault == F_NONE) { return; }
    report_encoder_error(s);
    if (s->state == FAULT) {
        tm_printf((const UB *)"[STATE] Buddy 2 ENTERED FAULT (state=5): %s(%u). Motors stopped; movement held.\n",
                  motion_fault_name(s->fault), (unsigned)s->fault);
    }
    if (s->fault == F_TIMING) {
        tm_printf((const UB *)"[ERROR] MOTOR CONTROL TIMING: control-loop timing became invalid; outputs were stopped.\n");
    } else if (s->fault == F_STOP_TIMEOUT) {
        tm_printf((const UB *)"[ERROR] MOTOR STOP: stationary confirmation did not arrive before the stop timeout.\n");
    } else if (s->fault == F_FINAL_GOAL) {
        tm_printf((const UB *)"[ERROR] MOTOR MOVEMENT: robot stopped outside the final distance/heading tolerance.\n");
    } else if (s->fault == F_DEADLINE) {
        tm_printf((const UB *)"[ERROR] MOTOR MOVEMENT: command did not complete before its deadline.\n");
    }
    tm_printf((const UB *)"[FAULT] %s(%u) feedback=%s(%u) id=%u held=%u; firmware running\n",
              motion_fault_name(s->fault), (unsigned)s->fault,
              motion_feedback_name(s->feedback_cause), (unsigned)s->feedback_cause,
              (unsigned)s->id, (unsigned)s->hold);
    if (!d->recorded) { return; }
    tm_printf((const UB *)"[FAULT] captured_ms=%ld mode=%s previous_state=%u dt_us=%ld finite_goal=%u\n",
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
    }
    report_motor_fault(s);
}

/* The barcode test image prints only barcode output; the periodic memory checks still run. */
#if defined(ROBO_BARCODE_TEST) && !defined(ROBO_LINE_STEERING)
#define ROBO_DIAG_QUIET 1
#else
#define ROBO_DIAG_QUIET 0
#endif

static bool report_task_stack(ID id, const char *name)
{
    stack_debug_t info;
    ER result = knl_task_stack_debug(id, &info);
    if (result == E_NOEXS) { return true; }
    if (result != E_OK) {
        tm_printf((const UB *)"[MEMORY] task=%u name=%s stack metadata invalid error=%d\n",
                  (unsigned)id, name, (int)result);
        return false;
    }
#if !ROBO_DIAG_QUIET
    tm_printf((const UB *)"[STACK] task=%u name=%s used=%u capacity=%u guard_ok=%u saved_sp_ok=%u\n",
              (unsigned)id, name, (unsigned)info.used, (unsigned)info.capacity,
              (unsigned)info.guard_ok, (unsigned)info.saved_sp_ok);
#endif
    return info.guard_ok && info.saved_sp_ok;
}
#endif

INT usermain(void)
{
    /* Interrupts run on the interrupted task's stack (everything is on MSP).
     * The detour call chain needs over 4 KiB before nested interrupt frames. */
    static const struct { FP entry; PRI priority; SZ stack; const char *name; } tasks[] = {
        {(FP)motion_task, 2, 4096, "motion"}, {(FP)controller_task, 4, 4096, "controller"},
#if ROBO_SUBSYSTEM5
        {(FP)obstacle_task, 4, 8192, "obstacle"},
#endif
#if ROBO_LINE_TASK
        {(FP)line_task, 8, 4096, "line"},
#endif
#if ROBO_BARCODE_TASK
        {(FP)barcode_task, 8, 4096, "barcode"},
#endif
#if ROBO_SHUTTLE_TASK
        {(FP)shuttle_task, 8, 4096, "shuttle"},
#endif
#if ROBO_SUBSYSTEM4
        {(FP)terrain_task, 8, 4096, "terrain"},
#endif
        {(FP)wifi_task, 12, 4096, "wifi"}
    };
    ID ids[sizeof(tasks) / sizeof(tasks[0])];
    unsigned i;
    T_CMTX mutex = {.mtxatr = TA_INHERIT};
    motion_config_t config;
    double start;
    UW crash_reason = in_w(CRASH_WDT_REASON);
    crash_watchdog_stop();
    /* After a crash reboot, report it with the motors never enabled. Repeats so
     * it is seen after the serial port reconnects; power-on clears it. */
    if (in_w(CRASH_WDT_SCRATCH0) == CRASH_MAGIC_FAULT || in_w(CRASH_WDT_SCRATCH0) == CRASH_MAGIC_FREEZE ||
        ((crash_reason & CRASH_REASON_TIMER) != 0U && in_w(CRASH_WDT_SCRATCH0) == CRASH_MAGIC_ARMED)) {
        for (;;) {
            if (in_w(CRASH_WDT_SCRATCH0) == CRASH_MAGIC_FAULT) {
                tm_printf((const UB *)"LAST CRASH: hard fault task=%u pc=%08x lr=%08x (motors held; power-cycle to run)\n",
                          (unsigned)in_w(CRASH_WDT_SCRATCH0 + 12U), (unsigned)in_w(CRASH_WDT_SCRATCH0 + 4U),
                          (unsigned)in_w(CRASH_WDT_SCRATCH0 + 8U));
            } else if (in_w(CRASH_WDT_SCRATCH0) == CRASH_MAGIC_FREEZE) {
                /* exception: 0 task code, 3 hard fault, 11 SVC, 14 PendSV, 15 SysTick,
                 * 16+n IRQ n (29 GPIO, 21 USB). */
                UW where = in_w(CRASH_WDT_SCRATCH0 + 12U);
                tm_printf((const UB *)"LAST CRASH: froze, motion not run for 0.8 s (motors held; power-cycle to run) "
                          "pc=%08x lr=%08x exception=%u task=%u motion_stage=%u obstacle_stage=%u\n",
                          (unsigned)in_w(CRASH_WDT_SCRATCH0 + 4U), (unsigned)in_w(CRASH_WDT_SCRATCH0 + 8U),
                          (unsigned)(where >> 24U), (unsigned)((where >> 16U) & 0xffU),
                          (unsigned)((where >> 8U) & 0xffU), (unsigned)(where & 0xffU));
            } else {
                /* motion: 1 inputs 2 step 3 output 4 sleeping (never woke: CPU taken or locked up).
                 * obstacle: 1 ping 2 decide 3 stop 4 settle 5 scan 6 report 7 detour;
                 * 8 out 9 pass 10 rear clearance 11 return turn 12 return look
                 * 13 return drive 14 line alignment. */
                tm_printf((const UB *)"LAST CRASH: watchdog timeout, code froze >1 s (motors held; power-cycle to run) "
                          "motion_stage=%u obstacle_stage=%u last_feed_ms=%u\n",
                          (unsigned)in_w(CRASH_MARK_MOTION), (unsigned)in_w(CRASH_MARK_OBSTACLE),
                          (unsigned)in_w(CRASH_MARK_FEED_MS));
            }
            (void)tk_dly_tsk(2000U);
        }
    }
    config = board_motion_config();
    start = motion_hal_time();
    bool setup = motion_hal_init(&config);
    double now = motion_hal_time();
    motion_init(&motion, &config, motion_hal_sample(), now, setup && ((now - start) <= config.setup_timeout));
    robot_bus_init(&bus, &motion.status);
#ifdef ROBO_PRODUCTION_BUILD
    tm_printf((const UB *)"firmware=%s hardware_confirmed=%u manual_enabled=%u max_output_percent=%u production=1 subsystem4_enabled=%u subsystem5_enabled=%u\n",
              (const UB *)ROBO_ENCODER_BUILD_ID, (unsigned)config.hardware_confirmed,
              (unsigned)config.manual_enabled, (unsigned)(config.max_output * 100.0),
              (unsigned)ROBO_SUBSYSTEM4, (unsigned)ROBO_SUBSYSTEM5);
#if !ROBO_DIAG_QUIET
    tm_printf((const UB *)"[STACK] idle_capacity=%u bytes (interrupts share the idle stack)\n",
              (unsigned)knl_idle_stack_size());
#endif
#endif
    status_mutex = tk_cre_mtx(&mutex);
    if (status_mutex <= 0) {
        motion_hal_emergency_stop();
        tm_printf((const UB *)"[INIT ERROR] status mutex creation failed: %d\n", (int)status_mutex);
        return 0;
    }
    ER init_result = subsystem1_init();
    if (init_result < E_OK) {
        motion_hal_emergency_stop();
        tm_printf((const UB *)"[INIT ERROR] communications init failed: %d\n", (int)init_result);
        return 0;
    }
    for (i = 0; i < sizeof(tasks) / sizeof(tasks[0]); ++i) {
        T_CTSK task = {.tskatr = TA_HLNG | TA_RNG0, .task = tasks[i].entry,
                      .itskpri = tasks[i].priority, .stksz = tasks[i].stack};
        ids[i] = tk_cre_tsk(&task);
        if (ids[i] <= 0) {
            motion_hal_emergency_stop();
            tm_printf((const UB *)"[INIT ERROR] task=%s stack=%u creation failed: %d\n",
                      tasks[i].name, (unsigned)tasks[i].stack, (int)ids[i]);
            return 0;
        }
    }
    running = true;
    init_result = subsystem1_start();
    if (init_result < E_OK) {
        motion_hal_emergency_stop();
        tm_printf((const UB *)"[INIT ERROR] communications start failed: %d\n", (int)init_result);
        return 0;
    }
    for (i = 0; i < sizeof(tasks) / sizeof(tasks[0]); ++i) {
        init_result = tk_sta_tsk(ids[i], 0);
        if (init_result < E_OK) {
            control(CONTROL_SHUTDOWN, false, false);
            motion_hal_emergency_stop();
            tm_printf((const UB *)"[INIT ERROR] task=%s start failed: %d\n", tasks[i].name, (int)init_result);
            return 0;
        }
    }
    /* Startup needs priority 1; periodic diagnostics must yield to motion/USB. */
    if (tk_chg_pri(TSK_SELF, 16) != E_OK) {
        motion_hal_emergency_stop();
        control(CONTROL_SHUTDOWN, false, false);
        tm_printf((const UB *)"[INIT ERROR] could not lower diagnostics task priority\n");
    }
    for (;;) {
        (void)tk_dly_tsk(2000U);
        bool memory_ok = knl_idle_stack_intact(0U) && knl_guards_intact();
        if (!memory_ok) {
            motion_hal_emergency_stop();
            control(CONTROL_SHUTDOWN, false, false);
            tm_printf((const UB *)"[MEMORY] idle stack or scheduler guard damaged; motors shut down\n");
        }
#ifdef ROBO_PRODUCTION_BUILD
#if !ROBO_DIAG_QUIET
        tm_printf((const UB *)"[STACK] idle_used=%u capacity=%u guard_ok=%u\n",
                  (unsigned)knl_idle_stack_used(0U), (unsigned)knl_idle_stack_size(),
                  (unsigned)knl_idle_stack_intact(0U));
#endif
        for (ID id = 1; id <= CNF_MAX_TSKID; ++id) {
            const char *name = id == tk_get_tid() ? "diagnostics" : "kernel";
            for (i = 0U; i < sizeof(tasks) / sizeof(tasks[0]); ++i) {
                if (ids[i] == id) { name = tasks[i].name; break; }
            }
            if (!report_task_stack(id, name)) {
                motion_hal_emergency_stop();
                control(CONTROL_SHUTDOWN, false, false);
            }
        }
        {
            robot_status_t status;
            UW dropped = 0U;
#if TM_CONSOLE_USB_CDC
            extern volatile UW tm_usb_dropped;
            dropped = tm_usb_dropped;
#endif
#if ROBO_DIAG_QUIET
            (void)dropped;
#else
            tm_printf((const UB *)"[DEBUG] firmware=%s\n", ROBO_ENCODER_BUILD_ID);
            tm_printf((const UB *)"[DEBUG] feed_age_ms=%u nmi_checks=%u usb_dropped=%u scheduler_guard=%u\n",
                      (unsigned)((in_w(TIMER_TIMERAWL) - last_feed_us) / 1000U),
                      (unsigned)freeze_check_count, (unsigned)dropped, (unsigned)knl_guards_intact());
#endif
            if (motion_get_status(&status)) { report_motion_fault(&status.motion); }
        }
#endif
    }
}
