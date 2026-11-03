#ifdef ROBO_LINE_TEST_HOST
#include "line_test_fake_hal.h"
#else
#include "subsystems.h"
#if defined(ROBO_SAFE_MOTOR_TEST) || defined(ROBO_LINE_STEERING)
#include "../subsystem2/template_hal.h"
#include <tm/tmonitor.h>
#endif
#endif
#include "board_config.h"

/* Each firmware profile supplies its own line_loop(); CMake keeps them exclusive. */
#if defined(ROBO_SAFE_MOTOR_TEST)
#include "motor_test.h"
static volatile bool sensor_paused;
void line_forward_pause(void) { sensor_paused = true; }

/* Either sensor HIGH drives forward; both LOW stops. A console pause lasts until both read LOW. */
static void line_loop(void)
{
    bool sensor_running = false;
    for (;;) {
        /* Console labels predate the right/centre naming used by line steering. */
        unsigned left = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned right = (unsigned)gpio_get(BOARD_LINE_C);
        bool move = (left == 1U) || (right == 1U);
        if (!motor_test_active()) { sensor_running = false; }
        if (!move) { sensor_paused = false; }
        if (move && !sensor_paused) {
            sensor_running = motor_test_sensor_forward();
        } else if (sensor_running) {
            motor_test_stop();
            sensor_running = false;
        }
        tm_printf((const UB *)"left_digital=%u right_digital=%u move_condition=%u sensor_forward_active=%u\n",
                  left, right, (unsigned)move, (unsigned)sensor_running);
        (void)tk_dly_tsk(100U);
    }
}

#elif defined(ROBO_IR_MONITOR)
/* Sensor-only image: no motion requests; '\r' rewrites one console line.
 * Also shows the barcode IR (GP28) so its wiring can be checked; barcode_task stays idle. */
static void line_loop(void)
{
    gpio_init(BOARD_BARCODE);
    gpio_set_dir(BOARD_BARCODE, false);
    gpio_disable_pulls(BOARD_BARCODE);
    for (;;) {
        unsigned right = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre = (unsigned)gpio_get(BOARD_LINE_C);
        unsigned barcode = (unsigned)gpio_get(BOARD_BARCODE);
        tm_printf((const UB *)"\rcentre_ir=%u right_ir=%u barcode_ir=%u", centre, right, barcode);
        (void)tk_dly_tsk(50U);
    }
}

#elif defined(ROBO_LINE_STEERING)
/* Line-tracking states (project brief / Week 6 statechart):
 *   INIT    line_task: pin setup and settle delay.
 *   WAIT    debounce gives LINE_WAIT until a stable reading; no request yet.
 *   FOLLOW  line_follow(): straight, or correct towards the side the line is on.
 *   GRACE   repeat the last follow command for LINE_GRACE_TIME after a loss.
 *   SEARCH  line_search_step(): sweep both sides, turn back, reverse, sweep again.
 *   HOLD    zero-speed refreshes after the search or LINE_SEARCH_TIMEOUT.
 * Not implemented yet: junction detection, barcode decode/actions (barcode.c),
 * and acknowledged handover back from obstacle avoidance. */
#include "steering.h"
#include "motion_hal.h"
/* Both IR sensors read black (1) when centred on the line; GP26 sits slightly right of centre. */
#define DEG_TO_RAD          (0.017453292519943295)
#define LINE_SESSION        3001U
#define LINE_SPEED          0.1   /* m/s forward while following. */
#define LINE_CORRECTION     0.0375 /* m/s per wheel when one sensor is off the line. */
#define LINE_SEARCH_SPEED   0.1   /* m/s outer wheel while pivoting on the stopped inner wheel.
                                   * Spinning in place stalled the reversing wheel. */
#define LINE_GRACE_TIME     0.2   /* s to keep the last command after a loss (gaps, flicker). */
#define LINE_SWEEP_ANGLE    (60.0 * DEG_TO_RAD) /* rad each side of the last heading on the line. */
#define LINE_BACKUP_SPEED   0.075 /* m/s reversing back to where the line was last seen. */
#define LINE_BACKUP_EXTRA   0.02  /* m reversed beyond the overshoot. */
#define LINE_SEARCH_TIMEOUT 15.0  /* s backstop before holding still, e.g. if a wheel slips. */

/* Numeric order is printed as sweep=%u in the diagnostics. */
typedef enum { SEARCH_IDLE, SEARCH_FIRST_SIDE, SEARCH_OTHER_SIDE, SEARCH_TURN_BACK,
               SEARCH_REVERSE, SEARCH_HOLD } line_search_phase_t;
typedef struct {
    line_search_phase_t phase;
    double dir;          /* +1 pivot left, -1 pivot right. */
    bool backed;         /* Already reversed once during this loss. */
    double seen_heading; /* Heading while the line was last seen. */
    double seen_centre;  /* Centre travel (m) while the line was last seen. */
    double lost_centre;  /* Centre travel when the search started. */
    double backup_from;  /* Centre travel when reversing started. */
} line_search_t;

static void search_toward(line_search_t *s, line_direction_t last_side)
{
    s->phase = SEARCH_FIRST_SIDE;
    s->dir = (last_side == LINE_RIGHT) ? -1.0 : 1.0; /* Left if the side is unknown. */
}

/* Inner wheel stopped, outer wheel at LINE_SEARCH_SPEED. */
static motion_result_t line_pivot(double dir)
{
    const double half = LINE_SEARCH_SPEED / 2.0;
    return (dir < 0.0) ? motion_steer_right(SOURCE_LINE, LINE_SESSION, half, half)
                       : motion_steer_left(SOURCE_LINE, LINE_SESSION, half, half);
}

/* Pivot towards the last side seen up to the sweep angle, then the other way past
 * the original heading. If both miss, turn back to the heading on the line, reverse
 * past the overshoot and sweep once more, then hold still.
 * Zero-speed refreshes keep ownership without a latched hold, so following resumes
 * when the line is seen again. Stopping the refreshes instead would raise F_EXPIRED. */
static motion_result_t line_search_step(line_search_t *s, const motion_status_t *motion,
                                        line_direction_t last_side, double lost_for)
{
    double turned = motion->heading - s->seen_heading;
    if (s->phase == SEARCH_IDLE) {
        search_toward(s, last_side);
        s->lost_centre = motion->centre;
    }
    if (s->phase <= SEARCH_OTHER_SIDE && s->dir * turned >= LINE_SWEEP_ANGLE) {
        if (s->phase == SEARCH_FIRST_SIDE) { s->phase = SEARCH_OTHER_SIDE; s->dir = -s->dir; }
        else if (!s->backed) { s->phase = SEARCH_TURN_BACK; s->dir = -s->dir; }
        else { s->phase = SEARCH_HOLD; }
    }
    if (s->phase == SEARCH_TURN_BACK && s->dir * turned >= 0.0) {
        s->phase = SEARCH_REVERSE;
        s->backup_from = motion->centre;
    }
    if (s->phase == SEARCH_REVERSE &&
        motion->centre <= s->backup_from - (s->lost_centre - s->seen_centre + LINE_BACKUP_EXTRA)) {
        search_toward(s, last_side);
        s->backed = true;
    }
    if (s->phase == SEARCH_HOLD || lost_for >= LINE_SEARCH_TIMEOUT) {
        return motion_steer(SOURCE_LINE, LINE_SESSION, 0.0, 0.0);
    }
    if (s->phase == SEARCH_REVERSE) { return motion_steer(SOURCE_LINE, LINE_SESSION, -LINE_BACKUP_SPEED, 0.0); }
    return line_pivot(s->dir);
}

static motion_result_t line_follow(line_direction_t drive)
{
    if (drive == LINE_LEFT) { return motion_steer_left(SOURCE_LINE, LINE_SESSION, LINE_SPEED, LINE_CORRECTION); }
    if (drive == LINE_RIGHT) { return motion_steer_right(SOURCE_LINE, LINE_SESSION, LINE_SPEED, LINE_CORRECTION); }
    return motion_steer(SOURCE_LINE, LINE_SESSION, LINE_SPEED, 0.0);
}

/* Existing line-error/confidence slots; no junction detector. */
static void line_publish(line_direction_t direction, double now)
{
    double error = 0.0;
    if (direction == LINE_LEFT) { error = -1.0; }
    else if (direction == LINE_RIGHT) { error = 1.0; }
    subsystem_status_t line_status = {.available = true, .timestamp = now,
        .values = {error, (direction != LINE_LOST && direction != LINE_WAIT) ? 1.0 : 0.0, 0.0, 0.0}};
    (void)robot_update_subsystem(SOURCE_LINE, &line_status);
}

static void line_report(unsigned centre, unsigned right, line_direction_t direction,
                        line_search_phase_t phase, motion_result_t result, const motion_status_t *m)
{
    tm_printf((const UB *)"line_pid centre_ir=%u right_ir=%u direction=%u sweep=%u request=%u state=%u fault=%u speed_mm_s=%d,%d output_permille=%d,%d\n",
        centre, right, (unsigned)direction, (unsigned)phase, (unsigned)result,
        (unsigned)m->state, (unsigned)m->fault,
        (int)(m->speed[0] * 1000.0), (int)(m->speed[1] * 1000.0),
        (int)(m->output[0] * 1000.0), (int)(m->output[1] * 1000.0));
    if (m->fault == F_FEEDBACK) {
        /* Encoder fault details recorded by Motion Control (see motion.h enums). */
        const encoder_sample_t *f = &m->feedback_fault_sample;
        tm_printf((const UB *)"line_fb cause=%u reliable=%u counts=%lu,%lu invalid=%lu,%lu previous_invalid=%lu,%lu\n",
            (unsigned)m->feedback_cause, (unsigned)f->reliable,
            (unsigned long)f->count[0], (unsigned long)f->count[1],
            (unsigned long)f->invalid[0], (unsigned long)f->invalid[1],
            (unsigned long)m->feedback_previous_invalid[0], (unsigned long)m->feedback_previous_invalid[1]);
    }
}

static void line_loop(void)
{
    line_steering_t filter = {0};
    line_search_t search = {0};
    line_direction_t last_side = LINE_STRAIGHT; /* Raw side the line was last seen on; STRAIGHT = unknown. */
    line_direction_t follow = LINE_STRAIGHT;    /* Last follow command, repeated during the grace time. */
    double lost_since = 0.0;
    double next_report = 0.0;
    for (;;) {
        unsigned right = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre = (unsigned)gpio_get(BOARD_LINE_C);
        /* Centre only black: robot drifted right, line is to the left -> LINE_LEFT.
         * Right only black: robot drifted left, line is to the right -> LINE_RIGHT. */
        line_direction_t direction = motion_line_direction(&filter, centre == 1U, right == 1U);
        robot_status_t status;
        motion_result_t result = R_HELD;
        double now = motion_hal_time();
        /* Raw, not debounced: a single centre-only sample before the loss still counts. */
        if (centre != right) { last_side = (centre == 1U) ? LINE_LEFT : LINE_RIGHT; }
        else if (centre == 1U) { last_side = LINE_STRAIGHT; }
        if (direction != LINE_LOST) { lost_since = now; }
        line_publish(direction, now);
        if (motion_get_status(&status)) {
            bool owns = status.owner_valid && status.owner == SOURCE_LINE && status.motion.id == LINE_SESSION;
            bool usable = !status.motion.hold && status.motion.fault == F_NONE;
            line_direction_t drive = direction;
            if (direction != LINE_LOST) {
                search.seen_heading = status.motion.heading;
                search.seen_centre = status.motion.centre;
                search.phase = SEARCH_IDLE;
                search.backed = false;
            }
            else if (owns && (now - lost_since) < LINE_GRACE_TIME) { drive = follow; } /* Bridge short dropouts. */
            if (drive == LINE_LOST && owns && usable && status.motion.state == DRIVING) {
                result = line_search_step(&search, &status.motion, last_side, now - lost_since);
            } else if (drive != LINE_WAIT && drive != LINE_LOST && usable &&
                       (status.motion.state == READY || (status.motion.state == DRIVING && owns))) {
                follow = drive;
                result = line_follow(drive);
            }
            if (now >= next_report) {
                next_report = now + 0.5;
                line_report(centre, right, direction, search.phase, result, &status.motion);
            }
        }
        (void)tk_dly_tsk(20U);
    }
}
#endif

#if defined(ROBO_SAFE_MOTOR_TEST) || defined(ROBO_LINE_STEERING)
static void line_sensors_init(void)
{
    static const unsigned pins[] = {BOARD_LINE_R, BOARD_LINE_C};
    for (unsigned i = 0U; i < 2U; ++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], false);
        gpio_disable_pulls(pins[i]);
    }
}
#endif

void line_task(INT code, void *context)
{
    (void)code; (void)context;
#if defined(ROBO_SAFE_MOTOR_TEST) || defined(ROBO_LINE_STEERING)
    line_sensors_init();
    (void)tk_dly_tsk(2000U);
    line_loop();
#else
    for (;;) { (void)tk_slp_tsk(TMO_FEVR); }
#endif
}
