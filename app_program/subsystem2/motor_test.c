#ifdef ROBO_SAFE_MOTOR_TEST
#include "motor_test.h"
#include "motion_hal.h"
#include "board_config.h"
#include <stddef.h>
#ifdef ROBO_MOTOR_TEST_HOST
#include "motor_test_fake_hal.h"
#else
#include "template_hal.h"
#endif

/* No production configuration gates are changed. Only this isolated profile
 * can call the test actuator. Alarm3/IRQ3 is unused elsewhere in this build. */
static volatile bool ready, active, result_pending;
static bool sensor_forward;
#ifdef ROBO_BATTERY_FORWARD_TEST
static volatile bool sensor_paused = true;
static volatile bool button_armed, release_tracking, press_tracking;
static uint64_t released_since, pressed_since;
static uint32_t button_polls, button_stop_actions;
static bool button_irq_registered;
static volatile bool stop_release_seen;
static volatile uint64_t stop_released_at;
static volatile uint32_t button_irq_events;
static void disarm_buttons(void)
{ button_armed = false; release_tracking = false; press_tracking = false; stop_release_seen = false; }
#else
static volatile bool sensor_paused; /* Only reboot clears a serial stop/manual-test latch. */
#endif
static encoder_sample_t baseline;
static volatile motor_test_result_t completed;
static uint64_t cutoff_at;
static uint32_t cutoff_irq_count, polled_cutoffs;
static uint64_t cooldown_until;
static int motor_sign[2];
enum { TEST_DUTY = 1000, PAIR_LEFT_DUTY = 996,
       TEST_DURATION_US = 1000000, TEST_COOLDOWN_US = 300000,
       SENSOR_WATCHDOG_US = 300000, BUTTON_DURATION_US = 2000000 };

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
#ifdef ROBO_BATTERY_FORWARD_TEST
    disarm_buttons(); /* Stop/timeout requires release and a fresh start press. */
#endif
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
    ++cutoff_irq_count;
    finish(true); /* Serial cutoff or missed-sensor-update watchdog. */
}

void motor_test_poll(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    if (active && time_us_64() >= cutoff_at) {
        ++polled_cutoffs;
        finish(true); /* Independent fallback if Alarm3 delivery is missed. */
    }
    restore_interrupts(irq_state);
}

#ifdef ROBO_BATTERY_FORWARD_TEST
static void button_stop_now(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    if (active) { ++button_stop_actions; finish(false); }
    restore_interrupts(irq_state);
}
static void button_irq(void)
{
    uint32_t start_events = gpio_get_irq_event_mask(20U);
    uint32_t stop_events = gpio_get_irq_event_mask(21U);
    uint64_t now;
    if (start_events != 0U) { gpio_acknowledge_irq(20U, start_events); }
    if (stop_events != 0U) { gpio_acknowledge_irq(21U, stop_events); }
    if ((start_events | stop_events) == 0U) { return; }
    ++button_irq_events;
    if (!active) { return; }
    if ((stop_events & GPIO_IRQ_EDGE_FALL) != 0U) { button_stop_now(); return; }
    now = time_us_64();
    if ((start_events & GPIO_IRQ_EDGE_RISE) != 0U) {
        stop_released_at = now;
        stop_release_seen = true;
    }
    if ((start_events & GPIO_IRQ_EDGE_FALL) != 0U) {
        if (stop_release_seen && now - stop_released_at >= 40000U) { button_stop_now(); }
        stop_release_seen = false;
    }
}
#endif

bool motor_test_init(void)
{
    motion_config_t config = board_motion_config();
    encoder_sample_t sample = motion_hal_sample();
    pwm_config pwm = pwm_get_default_config();
    T_DINT definition = {.intatr = TA_HLNG, .inthdr = (FP)deadline_irq};
    ready = false;
    stop_outputs();
#ifdef ROBO_BATTERY_FORWARD_TEST
    disarm_buttons();
    gpio_init(20U); gpio_set_dir(20U, false); gpio_pull_up(20U);
    gpio_init(21U); gpio_set_dir(21U, false); gpio_pull_up(21U);
    gpio_set_irq_enabled(20U, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    gpio_set_irq_enabled(21U, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
#endif
    if (!board_pins_valid() || !sample.reliable ||
        (encoder_pins[0][0] != 4U) || (encoder_pins[0][1] != 6U) ||
        (encoder_pins[1][0] != 5U) || (encoder_pins[1][1] != 7U)) { return false; }
    if (tk_def_int(3U, &definition) != E_OK) { return false; }
#ifdef ROBO_BATTERY_FORWARD_TEST
    if (!button_irq_registered) {
        if (!board_gpio_irq_register((1UL << 20U) | (1UL << 21U), button_irq)) { return false; }
        button_irq_registered = true;
    }
    gpio_acknowledge_irq(20U, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_acknowledge_irq(21U, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_set_irq_enabled(20U, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(21U, GPIO_IRQ_EDGE_FALL, true);
#endif
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

static bool start_test(unsigned wheel_mask, int direction, bool sensor, uint32_t duration_us)
{
    uint32_t irq_state = save_and_disable_interrupts();
    uint64_t now = time_us_64();
    unsigned wheel;
#ifdef ROBO_BATTERY_FORWARD_TEST
    if (!gpio_get(21U)) { /* Raw stop level also blocks explicit serial starts. */
        restore_interrupts(irq_state);
        return false;
    }
#endif
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
#ifdef ROBO_BATTERY_FORWARD_TEST
    stop_release_seen = gpio_get(20U);
    stop_released_at = now;
#endif
    /* Arm the serial cutoff or sensor watchdog BEFORE enabling PWM. */
    cutoff_at = time_us_64() + duration_us;
    out_w(TIMER_ALARM3, (uint32_t)cutoff_at);
    for (wheel = 0U; wheel < 2U; ++wheel) {
        unsigned slice = 4U + wheel;
        int electrical_direction = direction * motor_sign[wheel];
        /* Ground-test setting: serial pairs/button runs use L=24.9%, R=25%.
         * Individual and automatic IR tests retain their original 25% duty. */
        uint16_t duty = (uint16_t)((wheel_mask == 3U && !sensor && wheel == 0U) ?
                                   PAIR_LEFT_DUTY : TEST_DUTY);
        if ((wheel_mask & (1U << wheel)) == 0U) { continue; }
        gpio_set_function(motor_pins[wheel][0], GPIO_FUNC_PWM);
        gpio_set_function(motor_pins[wheel][1], GPIO_FUNC_PWM);
        pwm_set_both_levels(slice, electrical_direction > 0 ? duty : 0U,
                                  electrical_direction < 0 ? duty : 0U);
        pwm_set_enabled(slice, true);
    }
    restore_interrupts(irq_state);
    return true;
}

bool motor_test_start(unsigned wheel, int direction)
{
    if (wheel > 1U) { return false; }
    return start_test(1U << wheel, direction, false, TEST_DURATION_US);
}

bool motor_test_start_pair(int direction)
{
    return start_test(3U, direction, false, TEST_DURATION_US);
}

#ifdef ROBO_BATTERY_FORWARD_TEST
void motor_test_poll_buttons(void)
{
    motor_test_poll();
    uint64_t now = time_us_64();
    ++button_polls;
    if (!ready) { return; }
    if (!gpio_get(21U)) { /* Stop wins immediately on the next 20 ms poll. */
        disarm_buttons();
        button_stop_now();
        return;
    }
    if (gpio_get(20U)) {
        press_tracking = false;
        if (!release_tracking) { released_since = now; release_tracking = true; }
        if (now - released_since >= 40000U) { button_armed = true; }
        return;
    }
    release_tracking = false;
    if (!button_armed) { return; }
    if (active) {
        button_stop_now(); /* GPIO interrupt also handles stop between task polls. */
        return;
    }
    if (!press_tracking) { pressed_since = now; press_tracking = true; return; }
    if (now - pressed_since >= 40000U) {
        disarm_buttons();
        (void)start_test(3U, 1, false, BUTTON_DURATION_US);
    }
}
bool motor_test_get_buttons(motor_test_button_diag_t *d)
{
    uint32_t irq_state;
    if (d == NULL) { return false; }
    irq_state = save_and_disable_interrupts();
    *d = (motor_test_button_diag_t){.start_high=gpio_get(20U), .stop_high=gpio_get(21U),
        .ready=ready, .armed=button_armed, .active=active,
        .polls=button_polls, .stop_actions=button_stop_actions, .irq_registered=button_irq_registered,
        .irq_events=button_irq_events};
    d->cutoff_irq_count = cutoff_irq_count;
    d->polled_cutoffs = polled_cutoffs;
    uint64_t now = time_us_64();
    d->remaining_ms = active && cutoff_at > now ? (uint32_t)((cutoff_at - now + 999U) / 1000U) : 0U;
    restore_interrupts(irq_state);
    return true;
}
#endif

bool motor_test_sensor_forward(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    bool accepted;
    if (sensor_paused) {
        accepted = false;
    } else if (ready && active && sensor_forward) {
        /* Fresh sensor readings extend the watchdog without touching PWM.
         * Normal 100 ms updates therefore drive continuously, with no pauses. */
        cutoff_at = time_us_64() + SENSOR_WATCHDOG_US;
        out_w(TIMER_ALARM3, (uint32_t)cutoff_at);
        accepted = true;
    } else { accepted = start_test(3U, 1, true, SENSOR_WATCHDOG_US); }
    restore_interrupts(irq_state);
    return accepted;
}

void motor_test_pause_sensor(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    sensor_paused = true;
    if (active && sensor_forward) { finish(false); }
    restore_interrupts(irq_state);
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
