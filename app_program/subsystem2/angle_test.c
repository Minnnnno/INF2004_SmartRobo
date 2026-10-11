#ifdef ROBO_ANGLE_TEST
#include "angle_test.h"
#include "motion_hal.h"
#include "board_config.h"
#ifdef ROBO_MOTOR_TEST_HOST
#include "motor_test_fake_hal.h"
#else
#include "template_hal.h"
#endif
#include <math.h>
static angle_control_t control;
static volatile bool expired;
static bool timer_ready;
static void brake(void)
{
    pwm_set_both_levels(4U, 0U, 0U); pwm_set_both_levels(5U, 0U, 0U);
    pwm_set_enabled(4U, false); pwm_set_enabled(5U, false);
    motion_hal_emergency_stop();
}
static void deadline_irq(UINT number)
{
    (void)number; brake(); expired = true;
    out_w(TIMER_INTR, 8U); ClearInt(3U);
}
bool angle_test_init(void)
{
    motion_config_t config = board_motion_config();
    pwm_config pwm = pwm_get_default_config();
    T_DINT definition = {.intatr = TA_HLNG, .inthdr = (FP)deadline_irq};
    brake(); angle_control_init(&control, &config);
    if (!control.ready || !board_pins_valid() || !motion_hal_sample().reliable) { return false; }
    if (tk_def_int(3U, &definition) != E_OK) { return false; }
    pwm_config_set_wrap(&pwm, 3999U); pwm_config_set_clkdiv(&pwm, 1.5625f);
    pwm_init(4U, &pwm, false); pwm_init(5U, &pwm, false);
    out_w(TIMER_ARMED, 8U); out_w(TIMER_INTR, 8U); ClearInt(3U);
    set_w(TIMER_INTE, 8U); EnableInt(3U, 0); timer_ready = true;
    return true;
}
bool angle_test_start(int direction)
{
    if (!timer_ready || !angle_control_start(&control, direction, motion_hal_sample(), motion_hal_time())) { return false; }
    uint32_t irq_state = save_and_disable_interrupts();
    brake(); expired = false;
    out_w(TIMER_ARMED, 8U); out_w(TIMER_INTR, 8U); ClearInt(3U);
    out_w(TIMER_ALARM3, (uint32_t)(time_us_64() + 8000000U));
    restore_interrupts(irq_state);
    angle_test_poll();
    return true;
}
void angle_test_poll(void)
{
    unsigned w;
    angle_control_step(&control, motion_hal_sample(), motion_hal_time());
    uint32_t irq_state = save_and_disable_interrupts();
    if (expired) {
        if (control.state != ANGLE_TIMEOUT) { control.quiet_since = motion_hal_time(); }
        control.state = ANGLE_TIMEOUT; control.output[0] = 0.0; control.output[1] = 0.0;
    }
    if (control.state != ANGLE_TURNING) {
        brake();
        if (control.state != ANGLE_BRAKING) { out_w(TIMER_ARMED, 8U); }
    } else {
        for (w = 0U; w < 2U; ++w) {
            double output = control.output[w] * control.config.motor_sign[w];
            uint16_t duty = (uint16_t)(fmin(0.25, fabs(output)) * 4000.0);
            pwm_set_both_levels(4U + w, output > 0.0 ? duty : 0U, output < 0.0 ? duty : 0U);
            gpio_set_function(motor_pins[w][0], GPIO_FUNC_PWM);
            gpio_set_function(motor_pins[w][1], GPIO_FUNC_PWM);
            pwm_set_enabled(4U + w, true);
        }
    }
    restore_interrupts(irq_state);
}
void angle_test_stop(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    brake(); out_w(TIMER_ARMED, 8U); out_w(TIMER_INTR, 8U); ClearInt(3U); expired = false;
    angle_control_stop(&control); control.quiet_since = motion_hal_time();
    restore_interrupts(irq_state);
}
angle_control_t angle_test_status(void) { return control; }
bool angle_test_active(void) { return control.state == ANGLE_TURNING || control.state == ANGLE_BRAKING; }
#endif
