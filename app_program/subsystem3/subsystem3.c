#ifdef ROBO_LINE_TEST_HOST
#include "line_test_fake_hal.h"
#else
#include "subsystems.h"
#if defined(ROBO_SAFE_MOTOR_TEST) || defined(ROBO_LINE_STEERING)
#include "../subsystem2/template_hal.h"
#include <tm/tmonitor.h>
#endif
#endif
#ifdef ROBO_LINE_STEERING
#include "steering.h"
#include "motion_hal.h"
/* Both IR sensors read black (1) when centred on the line. */
#define LINE_PIN_RIGHT      26U   /* Slightly right of centre. */
#define LINE_PIN_CENTRE     27U
#define LINE_SPEED          0.08  /* m/s forward while following. */
#define LINE_CORRECTION     0.03  /* m/s per wheel when one sensor is off the line. */
#define LINE_SEARCH_SPEED   0.08  /* m/s outer wheel while pivoting on the stopped inner wheel.
                                   * Spinning in place stalled the reversing wheel. */
#define LINE_GRACE_TIME     0.2   /* s to keep the last command after a loss (gaps, flicker). */
#define LINE_SWEEP_ANGLE    (60.0 * 0.017453292519943295) /* rad each side of the last heading on the line. */
#define LINE_BACKUP_SPEED   0.06  /* m/s reversing back to where the line was last seen. */
#define LINE_BACKUP_EXTRA   0.02  /* m reversed beyond the overshoot. */
#define LINE_SEARCH_TIMEOUT 15.0  /* s backstop before holding still, e.g. if a wheel slips. */
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
#include "motor_test.h"
static volatile bool sensor_paused;
void line_forward_pause(void) { sensor_paused = true; }

#endif

void line_task(INT code, void *context)
{
    (void)code; (void)context;
#if defined(ROBO_SAFE_MOTOR_TEST) || defined(ROBO_LINE_STEERING)
#ifdef ROBO_SAFE_MOTOR_TEST
    bool sensor_running = false;
#else
    line_steering_t filter = {0};
    const uint32_t session = 3001U;
    line_direction_t last_side = LINE_STRAIGHT; /* Raw side the line was last seen on; STRAIGHT = unknown. */
    line_direction_t follow = LINE_STRAIGHT;    /* Last follow command, repeated during the grace time. */
    unsigned sweep = 0U;                        /* 0 not searching, 1 first side, 2 other side,
                                                 * 3 turn back, 4 reverse, 5 hold. */
    double sweep_dir = 1.0;                     /* +1 pivot left, -1 pivot right. */
    bool backed = false;                        /* Already reversed once during this loss. */
    double seen_heading = 0.0;                  /* Heading while the line was last seen. */
    double seen_centre = 0.0;                   /* Centre travel (m) while the line was last seen. */
    double lost_centre = 0.0;                   /* Centre travel when the search started. */
    double backup_from = 0.0;                   /* Centre travel when reversing started. */
    double lost_since = 0.0;
    double next_report = 0.0;
#endif
    gpio_init(26U);
    gpio_init(27U);
    gpio_set_dir(26U, false);
    gpio_set_dir(27U, false);
    gpio_disable_pulls(26U);
    gpio_disable_pulls(27U);
    (void)tk_dly_tsk(2000U);
    for (;;) {
#ifdef ROBO_SAFE_MOTOR_TEST
        unsigned left = (unsigned)gpio_get(26U);
        unsigned right = (unsigned)gpio_get(27U);
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
                  left, right,
                  (unsigned)move, (unsigned)sensor_running);
        (void)tk_dly_tsk(100U);
#else
        unsigned right = (unsigned)gpio_get(LINE_PIN_RIGHT);
        unsigned centre = (unsigned)gpio_get(LINE_PIN_CENTRE);
#ifdef ROBO_IR_MONITOR
        /* Sensor-only image: no motion requests; '\r' rewrites one console line. */
        tm_printf((const UB *)"\rcentre_ir=%u right_ir=%u", centre, right);
        (void)tk_dly_tsk(50U);
        continue;
#endif
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
        subsystem_status_t line_status = {.available=true, .timestamp=now,
            .values={direction == LINE_LEFT ? -1.0 : (direction == LINE_RIGHT ? 1.0 : 0.0),
                     direction != LINE_LOST && direction != LINE_WAIT ? 1.0 : 0.0,
                     0.0, 0.0}}; /* Existing line-error/confidence slots; no junction detector. */
        (void)robot_update_subsystem(SOURCE_LINE, &line_status);
        if (motion_get_status(&status)) {
            bool owns = status.owner_valid && status.owner == SOURCE_LINE && status.motion.id == session;
            bool usable = !status.motion.hold && status.motion.fault == F_NONE;
            line_direction_t drive = direction;
            if (direction != LINE_LOST) {
                seen_heading = status.motion.heading;
                seen_centre = status.motion.centre;
                sweep = 0U;
                backed = false;
            }
            else if (owns && (now - lost_since) < LINE_GRACE_TIME) { drive = follow; } /* Bridge short dropouts. */
            if (drive == LINE_LOST && owns && usable && status.motion.state == DRIVING) {
                /* Pivot towards the last side seen (left if unknown) up to the sweep
                 * angle, then the other way past the original heading. If both miss,
                 * turn back to the heading on the line, reverse past the overshoot and
                 * sweep once more, then hold still.
                 * Zero-speed refreshes keep ownership without a latched hold, so
                 * following resumes when the line is seen again. Stopping the
                 * refreshes instead would raise F_EXPIRED. */
                double turned = status.motion.heading - seen_heading;
                if (sweep == 0U) {
                    sweep = 1U;
                    sweep_dir = (last_side == LINE_RIGHT) ? -1.0 : 1.0;
                    lost_centre = status.motion.centre;
                }
                if (sweep <= 2U && sweep_dir * turned >= LINE_SWEEP_ANGLE) {
                    if (sweep == 1U) { sweep = 2U; sweep_dir = -sweep_dir; }
                    else if (!backed) { sweep = 3U; sweep_dir = -sweep_dir; }
                    else { sweep = 5U; }
                }
                if (sweep == 3U && sweep_dir * turned >= 0.0) { sweep = 4U; backup_from = status.motion.centre; }
                if (sweep == 4U &&
                    status.motion.centre <= backup_from - (lost_centre - seen_centre + LINE_BACKUP_EXTRA)) {
                    sweep = 1U;
                    backed = true;
                    sweep_dir = (last_side == LINE_RIGHT) ? -1.0 : 1.0;
                }
                if (sweep == 5U || (now - lost_since) >= LINE_SEARCH_TIMEOUT) { result = motion_steer(SOURCE_LINE, session, 0.0, 0.0); }
                else if (sweep == 4U) { result = motion_steer(SOURCE_LINE, session, -LINE_BACKUP_SPEED, 0.0); }
                else if (sweep_dir < 0.0) { result = motion_steer_right(SOURCE_LINE, session, LINE_SEARCH_SPEED / 2.0, LINE_SEARCH_SPEED / 2.0); }
                else { result = motion_steer_left(SOURCE_LINE, session, LINE_SEARCH_SPEED / 2.0, LINE_SEARCH_SPEED / 2.0); }
            } else if (drive != LINE_WAIT && drive != LINE_LOST && usable &&
                       (status.motion.state == READY || (status.motion.state == DRIVING && owns))) {
                follow = drive;
                if (drive == LINE_LEFT) { result = motion_steer_left(SOURCE_LINE, session, LINE_SPEED, LINE_CORRECTION); }
                else if (drive == LINE_RIGHT) { result = motion_steer_right(SOURCE_LINE, session, LINE_SPEED, LINE_CORRECTION); }
                else { result = motion_steer(SOURCE_LINE, session, LINE_SPEED, 0.0); }
            }
            if (now >= next_report) {
                next_report = now + 0.5;
                tm_printf((const UB *)"line_pid centre_ir=%u right_ir=%u direction=%u sweep=%u request=%u state=%u fault=%u speed_mm_s=%d,%d output_permille=%d,%d\n",
                    centre, right, (unsigned)direction, sweep, (unsigned)result,
                    (unsigned)status.motion.state, (unsigned)status.motion.fault,
                    (int)(status.motion.speed[0] * 1000.0), (int)(status.motion.speed[1] * 1000.0),
                    (int)(status.motion.output[0] * 1000.0), (int)(status.motion.output[1] * 1000.0));
                if (status.motion.fault == F_FEEDBACK) {
                    /* Encoder fault details recorded by Motion Control (see motion.h enums). */
                    tm_printf((const UB *)"line_fb cause=%u reliable=%u counts=%lu,%lu invalid=%lu,%lu previous_invalid=%lu,%lu\n",
                        (unsigned)status.motion.feedback_cause, (unsigned)status.motion.feedback_fault_sample.reliable,
                        (unsigned long)status.motion.feedback_fault_sample.count[0], (unsigned long)status.motion.feedback_fault_sample.count[1],
                        (unsigned long)status.motion.feedback_fault_sample.invalid[0], (unsigned long)status.motion.feedback_fault_sample.invalid[1],
                        (unsigned long)status.motion.feedback_previous_invalid[0], (unsigned long)status.motion.feedback_previous_invalid[1]);
                }
            }
        }
        (void)tk_dly_tsk(20U);
#endif
    }
#else
    for (;;) { (void)tk_slp_tsk(TMO_FEVR); }
#endif
}
void barcode_task(INT code, void *context)
{ (void)code; (void)context; for (;;) { (void)tk_slp_tsk(TMO_FEVR); } }
