#include "robot_rtos.h"
#include "board_config.h"
#include "motion_hal.h"
#include "robot.h"
#include "subsystems.h"
#include <math.h>

static robot_bus_t bus;
static motion_t motion;
static ID status_mutex;
static bool running;

static bool lock_bus(void)
{
    return (status_mutex > 0) && (tk_loc_mtx(status_mutex, TMO_POL) == E_OK);
}
static void unlock_bus(void) { (void)tk_unl_mtx(status_mutex); }

motion_result_t robot_submit(request_source_t source, uint32_t id, motion_mode_t mode,
                             double value, double left, double right)
{
    robot_request_t request;
    motion_result_t result;
    double now = motion_hal_time();
    if (!running || !lock_bus()) { return R_BUSY; }
    request.source = source;
    request.command = (motion_command_t){id, bus.generation, mode, value, {left, right},
        now + motion.config.command_lifetime,
        now + ((mode == MANUAL_OUTPUT) ? motion.config.manual_timeout : motion.config.movement_timeout)};
    result = robot_request(&bus, &request);
    unlock_bus();
    return result;
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

static void motion_task(INT code, void *context)
{
    double next = motion_hal_time();
    (void)code; (void)context;
    for (;;) {
        double now = motion_hal_time();
        encoder_sample_t sample = motion_hal_sample();
        if (tk_loc_mtx(status_mutex, TMO_FEVR) == E_OK) {
            robot_motion_inputs(&bus, &motion, sample, now);
            unlock_bus();
        } else { motion_raise_fault(&motion, F_SYSTEM, now); }
        /* Reset establishes a new sample/time origin; avoid an artificial zero-dt fault. */
        if (now > motion.last_time) { motion_step(&motion, sample, now); }
        if (tk_loc_mtx(status_mutex, TMO_FEVR) == E_OK) {
            robot_publish(&bus, &motion, now);
            /* A stop arriving while calculating wins before the hardware write. */
            if ((bus.controls & (CONTROL_STOP | CONTROL_FAULT | CONTROL_SHUTDOWN)) != 0U) {
                robot_motion_inputs(&bus, &motion, sample, now);
                robot_publish(&bus, &motion, now);
            }
            motion_hal_write(motion.status.output, motion.status.inhibited, now);
            unlock_bus();
        } else {
            motion_raise_fault(&motion, F_SYSTEM, now);
            motion_hal_write(motion.status.output, true, now);
        }
        next += motion.config.period;
        now = motion_hal_time();
        if (next <= now) { next = now + motion.config.period; }
        (void)tk_dly_tsk((RELTIM)fmax(1.0, ceil((next - now) * 1000.0)));
    }
}

static void controller_task(INT code, void *context)
{
    (void)code; (void)context;
    for (;;) {
        if (lock_bus()) { vehicle_controller_cycle(&bus, motion_hal_time()); unlock_bus(); }
        (void)tk_dly_tsk(10U);
    }
}

INT usermain(void)
{
    static const struct { FP entry; PRI priority; } tasks[] = {
        {(FP)motion_task, 2}, {(FP)controller_task, 4}, {(FP)obstacle_task, 4},
        {(FP)line_task, 8}, {(FP)barcode_task, 8}, {(FP)terrain_task, 8}, {(FP)wifi_task, 12}
    };
    ID ids[7];
    unsigned i;
    T_CMTX mutex = {.mtxatr = TA_INHERIT};
    motion_config_t config = board_motion_config();
    double start = motion_hal_time();
    bool setup = motion_hal_init(&config);
    double now = motion_hal_time();
    motion_init(&motion, &config, motion_hal_sample(), now, setup && ((now - start) <= config.setup_timeout));
    robot_bus_init(&bus, &motion.status);
    status_mutex = tk_cre_mtx(&mutex);
    if (status_mutex <= 0) { motion_hal_emergency_stop(); return 0; }
    for (i = 0; i < 7U; ++i) {
        T_CTSK task = {.tskatr = TA_HLNG | TA_RNG0, .task = tasks[i].entry,
                      .itskpri = tasks[i].priority, .stksz = 4096};
        ids[i] = tk_cre_tsk(&task);
        if (ids[i] <= 0) { motion_hal_emergency_stop(); return 0; }
    }
    running = true;
    for (i = 0; i < 7U; ++i) {
        if (tk_sta_tsk(ids[i], 0) < E_OK) {
            control(CONTROL_SHUTDOWN, false, false);
            motion_hal_emergency_stop();
            return 0;
        }
    }
    for (;;) { (void)tk_slp_tsk(TMO_FEVR); }
}
