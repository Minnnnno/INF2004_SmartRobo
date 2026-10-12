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
    uint32_t session = 3001U;
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
        unsigned left = (unsigned)gpio_get(26U);
        unsigned right = (unsigned)gpio_get(27U);
#ifdef ROBO_SAFE_MOTOR_TEST
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
        line_direction_t direction = motion_line_direction(&filter, left == 1U, right == 1U);
        robot_status_t status;
        motion_result_t result = R_HELD;
        double now = motion_hal_time();
        subsystem_status_t line_status = {.available=true, .timestamp=now,
            .values={direction == LINE_LEFT ? -1.0 : (direction == LINE_RIGHT ? 1.0 : 0.0),
                     direction != LINE_LOST && direction != LINE_WAIT ? 1.0 : 0.0,
                     0.0, 0.0}}; /* Existing line-error/confidence slots; no junction detector. */
        (void)robot_update_subsystem(SOURCE_LINE, &line_status);
        if (motion_get_status(&status)) {
            bool owns = status.owner_valid && status.owner == SOURCE_LINE && status.motion.id == session;
            if (direction == LINE_LOST && owns && status.motion.state == DRIVING) {
                motion_stop(); /* Controller must approve release after a lost-line hold. */
                ++session;
            } else if (direction != LINE_WAIT && direction != LINE_LOST &&
                       !status.motion.hold && status.motion.fault == F_NONE &&
                       (status.motion.state == READY || (status.motion.state == DRIVING && owns))) {
                if (direction == LINE_LEFT) { result = motion_steer_left(SOURCE_LINE, session, 0.08, 0.01); }
                else if (direction == LINE_RIGHT) { result = motion_steer_right(SOURCE_LINE, session, 0.08, 0.01); }
                else { result = motion_steer(SOURCE_LINE, session, 0.08, 0.0); }
            }
            if (now >= next_report) {
                next_report = now + 0.5;
                tm_printf((const UB *)"line_pid left_digital=%u right_digital=%u direction=%u request=%u state=%u fault=%u speed_mm_s=%d,%d output_permille=%d,%d\n",
                    left, right, (unsigned)direction, (unsigned)result,
                    (unsigned)status.motion.state, (unsigned)status.motion.fault,
                    (int)(status.motion.speed[0] * 1000.0), (int)(status.motion.speed[1] * 1000.0),
                    (int)(status.motion.output[0] * 1000.0), (int)(status.motion.output[1] * 1000.0));
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
