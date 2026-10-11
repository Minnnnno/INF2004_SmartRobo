#ifdef ROBO_LINE_TEST_HOST
#include "line_test_fake_hal.h"
#else
#include "subsystems.h"
#ifdef ROBO_SAFE_MOTOR_TEST
#include "../subsystem2/template_hal.h"
#include <tm/tmonitor.h>
#endif
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
#include "motor_test.h"
static volatile bool sensor_paused;
void line_forward_pause(void) { sensor_paused = true; }

#endif

void line_task(INT code, void *context)
{
    (void)code; (void)context;
#ifdef ROBO_SAFE_MOTOR_TEST
    bool sensor_running = false;
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
    }
#else
    for (;;) { (void)tk_slp_tsk(TMO_FEVR); }
#endif
}
void barcode_task(INT code, void *context)
{ (void)code; (void)context; for (;;) { (void)tk_slp_tsk(TMO_FEVR); } }
