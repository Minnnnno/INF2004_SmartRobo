#ifdef ROBO_SAFE_MOTOR_TEST
#include "motor_test.h"
#include "motion_hal.h"
#include "board_config.h"
#ifdef ROBO_MOTOR_TEST_HOST
#include "motor_test_fake_hal.h"
#else
#include "template_hal.h"
#endif

/* No production configuration gates are changed. Only this isolated profile
 * can call the test actuator. Alarm3/IRQ3 is unused elsewhere in this build. */
static volatile bool ready, active, result_pending;
static bool sensor_forward;
static encoder_sample_t baseline;
static volatile motor_test_result_t completed;
static uint64_t cooldown_until;
static int motor_sign[2];
enum { TEST_DUTY = 1000, TEST_DURATION_US = 1000000, TEST_COOLDOWN_US = 300000,
       SENSOR_WATCHDOG_US = 300000 };

static void stop_outputs(void)
{
    pwm_set_both_levels(4U, 0U, 0U);
    pwm_set_both_levels(5U, 0U, 0U);
    pwm_set_enabled(4U, false);
    pwm_set_enabled(5U, false);
    motion_hal_emergency_stop(); /* GP8..11 LOW, PWM disconnected. */
    out_w(TIMER_ARMED, 1U << 3U); /* Write-one-to-disarm. */
    out_w(TIMER_INTR, 1U << 3U);
    ClearInt(3U);
}

static void finish(bool timed_out)
{
    unsigned wheel;
    encoder_sample_t end;
    stop_outputs(); /* Stop before sampling, publishing, or printing anything. */
    if (!active) { return; }
    end = motion_hal_sample();
    for (wheel = 0U; wheel < 2U; ++wheel) {
        completed.delta[wheel] = end.count[wheel] - baseline.count[wheel];
        completed.invalid_delta[wheel] = end.invalid[wheel] - baseline.invalid[wheel];
    }
    completed.timed_out = timed_out;
    cooldown_until = time_us_64() + TEST_COOLDOWN_US;
    active = false;
    sensor_forward = false;
    result_pending = true;
}

static void deadline_irq(UINT number)
{
    (void)number;
    finish(true); /* Serial cutoff or missed-sensor-update watchdog. */
}

bool motor_test_init(void)
{
    motion_config_t config = board_motion_config();
    encoder_sample_t sample = motion_hal_sample();
    pwm_config pwm = pwm_get_default_config();
    T_DINT definition = {.intatr = TA_HLNG, .inthdr = (FP)deadline_irq};
    stop_outputs();
    if (!board_pins_valid() || !sample.reliable ||
        (encoder_pins[0][0] != 4U) || (encoder_pins[0][1] != 6U) ||
        (encoder_pins[1][0] != 5U) || (encoder_pins[1][1] != 7U)) { return false; }
    if (tk_def_int(3U, &definition) != E_OK) { return false; }
    motor_sign[0] = config.motor_sign[0];
    motor_sign[1] = config.motor_sign[1];
    /* 125 MHz / 1.5625 / 4000 = 20 kHz; 1000/4000 is exactly 25%. */
    pwm_config_set_wrap(&pwm, 3999U);
    pwm_config_set_clkdiv(&pwm, 1.5625f);
    pwm_init(4U, &pwm, false);
    pwm_init(5U, &pwm, false);
    set_w(TIMER_INTE, 1U << 3U);
    EnableInt(3U, 0); /* Highest available priority; no console work in the ISR. */
    ready = true;
    return true;
}

static bool start_test(unsigned wheel_mask, int direction, bool sensor)
{
    uint32_t irq_state = save_and_disable_interrupts();
    uint64_t now = time_us_64();
    unsigned wheel;
    if (!ready || active || result_pending || (now < cooldown_until) ||
        (wheel_mask == 0U) || (wheel_mask > 3U) || ((direction != 1) && (direction != -1))) {
        restore_interrupts(irq_state);
        return false; /* Never queue another pulse or extend the running deadline. */
    }
    stop_outputs();
    baseline = motion_hal_sample();
    completed.wheel = wheel_mask == 3U ? 2U : (wheel_mask == 1U ? 0U : 1U);
    completed.direction = direction;
    active = true;
    sensor_forward = sensor;
    /* Arm the serial cutoff or sensor watchdog BEFORE enabling PWM. */
    out_w(TIMER_ALARM3, (uint32_t)(time_us_64() +
          (sensor ? SENSOR_WATCHDOG_US : TEST_DURATION_US)));
    for (wheel = 0U; wheel < 2U; ++wheel) {
        unsigned slice = 4U + wheel;
        int electrical_direction = direction * motor_sign[wheel];
        if ((wheel_mask & (1U << wheel)) == 0U) { continue; }
        gpio_set_function(motor_pins[wheel][0], GPIO_FUNC_PWM);
        gpio_set_function(motor_pins[wheel][1], GPIO_FUNC_PWM);
        pwm_set_both_levels(slice, electrical_direction > 0 ? TEST_DUTY : 0U,
                                  electrical_direction < 0 ? TEST_DUTY : 0U);
        pwm_set_enabled(slice, true);
    }
    restore_interrupts(irq_state);
    return true;
}

bool motor_test_start(unsigned wheel, int direction)
{
    if (wheel > 1U) { return false; }
    return start_test(1U << wheel, direction, false);
}

bool motor_test_start_pair(int direction)
{
    return start_test(3U, direction, false);
}

bool motor_test_sensor_forward(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    bool accepted;
    if (ready && active && sensor_forward) {
        /* Fresh sensor readings extend the watchdog without touching PWM.
         * Normal 100 ms updates therefore drive continuously, with no pauses. */
        out_w(TIMER_ALARM3, (uint32_t)(time_us_64() + SENSOR_WATCHDOG_US));
        accepted = true;
    } else { accepted = start_test(3U, 1, true); }
    restore_interrupts(irq_state);
    return accepted;
}

void motor_test_stop(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    finish(false);
    restore_interrupts(irq_state);
}

bool motor_test_active(void)
{
    return active;
}

void motor_test_shutdown(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    ready = false; /* Block both serial and sensor entry points until reboot. */
    finish(false);
    restore_interrupts(irq_state);
}

bool motor_test_result(motor_test_result_t *result)
{
    uint32_t irq_state = save_and_disable_interrupts();
    bool available = result_pending;
    if (available) { *result = completed; result_pending = false; }
    restore_interrupts(irq_state);
    return available;
}
#endif
