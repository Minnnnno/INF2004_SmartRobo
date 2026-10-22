#include "motor_test.h"
#include "motion_hal.h"
#include "board_config.h"
#include "motor_test_fake_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); exit(1); } } while (0)
static unsigned checks, writes;
static bool enabled[2], alarm_armed, expect_pair;
static uint16_t duty[2][2];
static uint64_t now;
static uint32_t deadline;
static FP deadline_handler;
static encoder_sample_t sample = {.reliable = true};
#ifdef ROBO_BATTERY_FORWARD_TEST
static bool start_high = true, stop_high = true;
static bool allow_button_irq = true;
static void (*stop_irq_handler)(void);
static uint32_t button_pending[2], button_enabled[2];
bool board_gpio_irq_register(uint32_t mask, void (*handler)(void))
{
    CHECK(mask == ((1UL << 20U) | (1UL << 21U)) && handler != NULL);
    if (!allow_button_irq) { return false; }
    CHECK(stop_irq_handler == NULL); stop_irq_handler = handler; return true;
}
void gpio_set_irq_enabled(unsigned pin, uint32_t events, bool enable)
{
    CHECK((pin == 20U || pin == 21U) && (events & ~12U) == 0U);
    if (enable) { button_enabled[pin-20U] |= events; }
    else { button_enabled[pin-20U] &= ~events; }
}
uint32_t gpio_get_irq_event_mask(unsigned pin)
{ CHECK(pin == 20U || pin == 21U); return button_pending[pin-20U] & button_enabled[pin-20U]; }
void gpio_acknowledge_irq(unsigned pin, uint32_t events)
{ CHECK(pin == 20U || pin == 21U); button_pending[pin-20U] &= ~events; }
static void fire_button_irq(unsigned pin, uint32_t events)
{
    CHECK(stop_irq_handler != NULL);
    button_pending[pin-20U] |= events; stop_irq_handler();
    CHECK(button_pending[pin-20U] == 0U);
}
void gpio_init(unsigned pin) { CHECK(pin == 20U || pin == 21U); }
void gpio_set_dir(unsigned pin, bool output) { CHECK((pin == 20U || pin == 21U) && !output); }
void gpio_pull_up(unsigned pin) { CHECK(pin == 20U || pin == 21U); }
bool gpio_get(unsigned pin) { CHECK(pin == 20U || pin == 21U); return pin == 20U ? start_high : stop_high; }
static void release_buttons(void)
{
    start_high = stop_high = true;
    motor_test_poll_buttons(); now += 40000U; motor_test_poll_buttons();
}
static void press_start(void)
{
    start_high = false;
    motor_test_poll_buttons(); now += 40000U; motor_test_poll_buttons();
}
#endif
uint32_t save_and_disable_interrupts(void) { return 0U; }
void restore_interrupts(uint32_t state) { (void)state; }
uint64_t time_us_64(void) { return now; }
pwm_config pwm_get_default_config(void) { pwm_config config = {4095U, 1.0f}; return config; }
void pwm_config_set_wrap(pwm_config *config, uint16_t wrap) { config->wrap = wrap; }
void pwm_config_set_clkdiv(pwm_config *config, float divider) { config->divider = divider; }
void pwm_init(unsigned slice, const pwm_config *config, bool enable)
{ CHECK(slice == 4U || slice == 5U); CHECK(config->wrap == 3999U && config->divider == 1.5625f); CHECK(!enable); }
encoder_sample_t motion_hal_sample(void) { return sample; }
void motion_hal_emergency_stop(void)
{ enabled[0] = false; enabled[1] = false; }
void pwm_set_both_levels(unsigned slice, uint16_t a, uint16_t b)
{ CHECK(slice == 4U || slice == 5U); duty[slice - 4U][0] = a; duty[slice - 4U][1] = b; ++writes; }
void pwm_set_enabled(unsigned slice, bool enable)
{
    CHECK(slice == 4U || slice == 5U);
    if (enable) {
        CHECK(alarm_armed); CHECK(expect_pair || !enabled[1U - (slice - 4U)]);
#ifdef ROBO_BATTERY_FORWARD_TEST
        CHECK(stop_irq_handler != NULL && button_enabled[0] == 12U && button_enabled[1] == 4U);
#endif
    }
    enabled[slice - 4U] = enable; ++writes;
}
void gpio_set_function(unsigned pin, unsigned function)
{ CHECK(pin >= 8U && pin <= 11U); CHECK(function == GPIO_FUNC_PWM); }
void out_w(unsigned address, uint32_t value)
{
    if (address == TIMER_ALARM3) { deadline = value; alarm_armed = true; }
    else if (address == TIMER_ARMED) { CHECK(value == 8U); alarm_armed = false; }
    else { CHECK(address == TIMER_INTR); CHECK(value == 8U); }
}
void set_w(unsigned address, uint32_t value)
{ CHECK(address == TIMER_INTE && value == 8U); }
void ClearInt(unsigned number) { CHECK(number == 3U); }
void EnableInt(unsigned number, int priority) { CHECK(number == 3U && priority == 0); }
int tk_def_int(unsigned number, const T_DINT *definition)
{ CHECK(number == 3U); deadline_handler = definition->inthdr; return E_OK; }

int main(int argc, char **argv)
{
#ifdef ROBO_BATTERY_FORWARD_TEST
    bool toggle_stop = argc > 1 && strcmp(argv[1], "toggle") == 0;
    bool poll_cutoff = argc > 1 && strcmp(argv[1], "poll_cutoff") == 0;
    bool irq20_stop = argc > 1 && strcmp(argv[1], "irq20") == 0;
    bool irq21_stop = argc > 1 && strcmp(argv[1], "irq21") == 0;
    bool early_stop = toggle_stop || irq20_stop || irq21_stop || (argc > 1 && strcmp(argv[1], "stop") == 0);
    motor_test_result_t result;
    motion_config_t config = board_motion_config();
    motor_test_button_diag_t buttons;
    CHECK(!motor_test_get_buttons(NULL));
    CHECK(!config.hardware_confirmed && !config.manual_enabled);
    sample.reliable = false;
    CHECK(!motor_test_init());
    CHECK(!enabled[0] && !enabled[1] && !motor_test_sensor_forward());
    sample.reliable = true;
    allow_button_irq = false;
    CHECK(!motor_test_init()); /* Cannot enable motors without the stop interrupt. */
    CHECK(!motor_test_active() && !enabled[0] && !enabled[1]);
    allow_button_irq = true;
    CHECK(motor_test_init());
    CHECK(!motor_test_active() && !enabled[0] && !enabled[1]);
    CHECK(!motor_test_sensor_forward());
    CHECK(motor_test_get_buttons(&buttons));
    CHECK(buttons.start_high && buttons.stop_high && buttons.ready && !buttons.active);
    CHECK(buttons.irq_registered);
    fire_button_irq(20U, GPIO_IRQ_EDGE_FALL); /* An interrupt never starts a stopped robot. */
    CHECK(!motor_test_active() && !enabled[0] && !enabled[1]);
    start_high = false; /* Holding start at power-on must never move the robot. */
    for (unsigned tick = 0U; tick < 10U; ++tick) {
        now += 100000U; motor_test_poll_buttons();
        CHECK(!motor_test_active() && !enabled[0] && !enabled[1]);
    }
    release_buttons();
    start_high = stop_high = false; /* Simultaneous presses: stop wins. */
    motor_test_poll_buttons(); now += 100000U; motor_test_poll_buttons();
    CHECK(motor_test_get_buttons(&buttons));
    CHECK(!buttons.start_high && !buttons.stop_high && !buttons.active && buttons.polls > 0U);
    CHECK(!motor_test_active());
    CHECK(!motor_test_start_pair(1)); /* Held GP21 blocks serial commands too. */
    stop_high = true;
    now += 100000U; motor_test_poll_buttons();
    CHECK(!motor_test_active()); /* Releasing stop while start is held cannot start. */
    now = UINT32_MAX - 100U; /* Button deadline must survive timer rollover. */
    release_buttons();
    start_high = false; motor_test_poll_buttons();
    now += 20000U; start_high = true; motor_test_poll_buttons(); /* Contact bounce. */
    start_high = false; motor_test_poll_buttons();
    now += 20000U; motor_test_poll_buttons(); CHECK(!motor_test_active());
    expect_pair = true;
    now += 20000U; motor_test_poll_buttons();
    CHECK(enabled[0] && enabled[1] && motor_test_active());
    CHECK(duty[0][0] == 996U && duty[0][1] == 0U);
    CHECK(duty[1][0] == 0U && duty[1][1] == 1000U);
    CHECK(deadline == (uint32_t)(now + 2000000U));
    CHECK(motor_test_get_buttons(&buttons));
    CHECK(buttons.remaining_ms == 2000U && buttons.polled_cutoffs == 0U);
    unsigned startup_writes = writes;
    for (unsigned refresh = 0U; refresh < 10U; ++refresh) {
        motor_test_poll_buttons(); /* Holding start neither repeats nor extends the run. */
        CHECK(!motor_test_sensor_forward());
        CHECK(deadline == (uint32_t)(now + 2000000U) && writes == startup_writes);
    }
    sample.count[0] += 100U; sample.count[1] += 110U;
    now += early_stop ? 100000U : 2000000U;
    if (irq20_stop || irq21_stop) {
        uint32_t polls_before = buttons.polls;
        CHECK(motor_test_get_buttons(&buttons)); polls_before = buttons.polls;
        if (irq20_stop) {
            start_high = true; fire_button_irq(20U, GPIO_IRQ_EDGE_RISE);
            now += 40000U;
            start_high = false; fire_button_irq(20U, GPIO_IRQ_EDGE_FALL);
        } else { stop_high = false; fire_button_irq(21U, GPIO_IRQ_EDGE_FALL); }
        CHECK(motor_test_get_buttons(&buttons));
        CHECK(buttons.polls == polls_before && !buttons.active && buttons.irq_events > 1U);
    }
    else if (toggle_stop) {
        release_buttons();
        start_high = false; motor_test_poll_buttons();
        CHECK(!motor_test_active()); /* A single sampled GP20 tap must stop, without 40 ms delay. */
        now += 20000U; motor_test_poll_buttons();
        CHECK(!motor_test_active()); /* Holding the stop press cannot restart. */
        start_high = true; motor_test_poll_buttons();
        now += 20000U;
        start_high = false; motor_test_poll_buttons();
        now += 40000U; motor_test_poll_buttons();
        CHECK(!motor_test_active()); /* A short release/bounce cannot re-arm a start. */
    }
    else if (early_stop) { stop_high = false; motor_test_poll_buttons(); }
    else if (poll_cutoff) { motor_test_poll_buttons(); } /* No alarm interrupt delivered. */
    else { deadline_handler(3U); } /* Actual hardware-cutoff handler; no task polling. */
    CHECK(!enabled[0] && !enabled[1] && !alarm_armed && !motor_test_active());
    CHECK(motor_test_get_buttons(&buttons));
    CHECK(!buttons.active && buttons.stop_actions == (early_stop ? 1U : 0U));
    CHECK(buttons.remaining_ms == 0U);
    CHECK(buttons.polled_cutoffs == (poll_cutoff ? 1U : 0U));
    CHECK(motor_test_result(&result));
    CHECK(result.wheel == 2U && result.direction == 1 && result.timed_out == !early_stop);
    CHECK(result.delta[0] == 100U && result.delta[1] == 110U);
    CHECK(result.invalid_delta[0] == 0U && result.invalid_delta[1] == 0U);
    now += 400000U;
    stop_high = true;
    motor_test_poll_buttons();
    CHECK(!motor_test_active() && !motor_test_sensor_forward());
    CHECK(motor_test_start_pair(1)); /* Explicit serial tests retain one-second cutoff. */
    CHECK(deadline == (uint32_t)(now + 1000000U));
    motor_test_stop(); CHECK(motor_test_result(&result));
    now += 400000U;
    motor_test_poll_buttons(); CHECK(!motor_test_active());
    release_buttons(); press_start(); /* A fresh press permits another bounded trial. */
    CHECK(motor_test_active() && deadline == (uint32_t)(now + 2000000U));
    motor_test_shutdown();
    CHECK(!enabled[0] && !enabled[1] && !alarm_armed && !motor_test_active());
    release_buttons(); press_start();
    CHECK(!motor_test_active() && !motor_test_sensor_forward());
    printf("PASS: %u button forward checks (%s; mock registers)\n", checks,
           irq20_stop ? "GP20 interrupt stop without task polling" :
           (irq21_stop ? "GP21 interrupt stop without task polling" :
           (toggle_stop ? "GP20 quick-tap stop" : (early_stop ? "GP21 stop" : "two-second cutoff"))));
    return 0;
#else
    (void)argc; (void)argv;
    motor_test_result_t result;
    unsigned wheel, direction;
    motion_config_t config = board_motion_config();
    CHECK(config.counts_per_rev[0] == 2542.4 && config.counts_per_rev[1] == 2543.4);
    CHECK(!config.hardware_confirmed && !config.manual_enabled);
    CHECK(config.motor_sign[0] == 1 && config.motor_sign[1] == -1);
    CHECK(config.encoder_sign[0] == -1 && config.encoder_sign[1] == 1);
    CHECK(config.diameter[0] == 0.0675 && config.diameter[1] == 0.0679 && config.track == 0.115);
    CHECK(encoder_pins[0][0] == 4U && encoder_pins[0][1] == 6U);
    CHECK(encoder_pins[1][0] == 5U && encoder_pins[1][1] == 7U);
    sample.reliable = false;
    CHECK(!motor_test_init());
    CHECK(!motor_test_start(0U, 1));
    CHECK(!motor_test_sensor_forward());
    CHECK(!enabled[0] && !enabled[1]);
    sample.reliable = true;
    CHECK(motor_test_init());
    CHECK(!motor_test_active() && !enabled[0] && !enabled[1]);
    CHECK(!motor_test_result(&result));
    CHECK(!motor_test_start(2U, 1));
    CHECK(!motor_test_start(0U, 0));
    for (wheel = 0U; wheel < 2U; ++wheel) {
        for (direction = 0U; direction < 2U; ++direction) {
            int sign = direction == 0U ? 1 : -1;
            uint32_t expected = direction == 0U ? 123U : UINT32_MAX - 122U;
            unsigned before;
            now += 400000U;
            sample.count[wheel] = UINT32_MAX - 10U;
            CHECK(motor_test_start(wheel, sign));
            CHECK(enabled[wheel] && !enabled[1U - wheel]);
            /* Right wiring requires the opposite electrical channel. */
            unsigned driven_channel = wheel == 0U ? direction : 1U - direction;
            CHECK(duty[wheel][driven_channel] == 1000U && duty[wheel][1U - driven_channel] == 0U);
            CHECK(deadline == (uint32_t)(now + 1000000U));
            before = writes;
            CHECK(!motor_test_start(1U - wheel, sign));
            CHECK(!motor_test_start(wheel, sign));
            CHECK(writes == before && deadline == (uint32_t)(now + 1000000U));
            sample.count[wheel] += expected;
            now += 1000000U;
            deadline_handler(3U); /* Exercise the actual cutoff handler. */
            CHECK(!enabled[0] && !enabled[1] && !alarm_armed && !motor_test_active());
            CHECK(!motor_test_start(wheel, sign)); /* Result still pending. */
            CHECK(motor_test_result(&result));
            CHECK(result.wheel == wheel && result.direction == sign && result.timed_out);
            CHECK(result.delta[wheel] == expected && result.delta[1U - wheel] == 0U);
            CHECK(result.invalid_delta[0] == 0U && result.invalid_delta[1] == 0U);
            CHECK(!motor_test_result(&result));
            CHECK(!motor_test_start(wheel, -sign)); /* Reversal cooldown. */
        }
    }
    now += 400000U;
    CHECK(motor_test_start(0U, 1));
    now += 100000U;
    sample.count[0] += 5U;
    sample.invalid[0] += 2U;
    motor_test_stop();
    CHECK(!motor_test_active() && !enabled[0] && !enabled[1]);
    CHECK(motor_test_result(&result) && !result.timed_out && result.delta[0] == 5U);
    CHECK(result.invalid_delta[0] == 2U && result.invalid_delta[1] == 0U);
    motor_test_stop();
    CHECK(!motor_test_result(&result));
    now = UINT32_MAX - 100U;
    CHECK(motor_test_start(1U, -1));
    CHECK(deadline == (uint32_t)(now + 1000000U)); /* Timer low-word rollover. */
    motor_test_stop();
    CHECK(motor_test_result(&result));
    for (direction = 0U; direction < 2U; ++direction) {
        int sign = direction == 0U ? 1 : -1;
        unsigned before;
        now += 400000U;
        expect_pair = true;
        CHECK(!motor_test_start_pair(0));
        CHECK(motor_test_start_pair(sign));
        CHECK(enabled[0] && enabled[1]);
        CHECK(duty[0][direction] == 996U && duty[0][1U - direction] == 0U);
        CHECK(duty[1][1U - direction] == 1000U && duty[1][direction] == 0U);
        CHECK(deadline == (uint32_t)(now + 1000000U));
        before = writes;
        CHECK(!motor_test_start(0U, sign) && !motor_test_start(1U, sign));
        CHECK(!motor_test_start_pair(-sign));
        CHECK(writes == before && deadline == (uint32_t)(now + 1000000U));
        sample.count[0] += direction == 0U ? 100U : UINT32_MAX - 99U;
        sample.count[1] += direction == 0U ? 110U : UINT32_MAX - 109U;
        now += direction == 0U ? 1000000U : 200000U;
        if (direction == 0U) { deadline_handler(3U); } else { motor_test_stop(); }
        CHECK(!enabled[0] && !enabled[1] && !motor_test_active());
        CHECK(!motor_test_start_pair(sign));
        CHECK(motor_test_result(&result));
        CHECK(result.wheel == 2U && result.direction == sign);
        CHECK(result.timed_out == (direction == 0U));
        CHECK(result.delta[0] == (direction == 0U ? 100U : UINT32_MAX - 99U));
        CHECK(result.delta[1] == (direction == 0U ? 110U : UINT32_MAX - 109U));
        CHECK(!motor_test_start_pair(-sign));
        expect_pair = false;
    }
    now += 400000U;
    expect_pair = true;
    CHECK(motor_test_sensor_forward());
    CHECK(enabled[0] && enabled[1] && motor_test_active());
    CHECK(duty[0][0] == 1000U && duty[0][1] == 0U);
    CHECK(duty[1][0] == 0U && duty[1][1] == 1000U);
    unsigned continuous_writes = writes;
    for (unsigned refresh = 0U; refresh < 50U; ++refresh) {
        now += 100000U; /* Five seconds of continuous 100 ms sensor updates. */
        CHECK(motor_test_sensor_forward());
        CHECK(enabled[0] && enabled[1] && motor_test_active());
        CHECK(writes == continuous_writes); /* No PWM reset, stop or restart. */
        CHECK(deadline == (uint32_t)(now + 300000U));
        CHECK(!motor_test_start_pair(-1) && !motor_test_start(0U, 1));
    }
    motor_test_stop();
    CHECK(!motor_test_active() && !enabled[0] && !enabled[1]);
    CHECK(motor_test_result(&result) && !result.timed_out && result.wheel == 2U);
    CHECK(!motor_test_sensor_forward()); /* Cooldown only after an actual stop. */
    now += 400000U;
    CHECK(motor_test_sensor_forward());
    now += 300000U;
    deadline_handler(3U); /* Sensor readings stop: watchdog turns motors off. */
    CHECK(!motor_test_active() && !enabled[0] && !enabled[1] && !alarm_armed);
    CHECK(motor_test_result(&result) && result.timed_out);
    now = ((uint64_t)UINT32_MAX + 1U) * 2U - 100U;
    CHECK(motor_test_sensor_forward());
    CHECK(deadline == (uint32_t)(now + 300000U)); /* Watchdog timer rollover. */
    motor_test_pause_sensor(); /* A stop preempting the IR task cannot restart either motor. */
    CHECK(!motor_test_active() && !enabled[0] && !enabled[1] && !alarm_armed);
    CHECK(motor_test_result(&result) && !result.timed_out);
    now += 400000U;
    unsigned paused_writes = writes;
    for (unsigned retry = 0U; retry < 50U; ++retry) {
        now += 100000U;
        CHECK(!motor_test_sensor_forward());
        CHECK(!enabled[0] && !enabled[1] && writes == paused_writes);
    }
    CHECK(motor_test_start(0U, 1)); /* Explicit manual tests remain available. */
    CHECK(enabled[0] && !enabled[1]);
    CHECK(!motor_test_sensor_forward()); /* IR cannot extend a manual pulse. */
    CHECK(deadline == (uint32_t)(now + 1000000U));
    now += 1000000U;
    deadline_handler(3U);
    CHECK(motor_test_result(&result) && result.timed_out);
    CHECK(!enabled[0] && !enabled[1] && !motor_test_sensor_forward());
    now += 400000U;
    CHECK(motor_test_start_pair(1));
    motor_test_shutdown();
    CHECK(!motor_test_active() && !enabled[0] && !enabled[1] && !alarm_armed);
    CHECK(motor_test_result(&result));
    now += 400000U;
    CHECK(!motor_test_start_pair(1) && !motor_test_start(0U, 1));
    CHECK(!motor_test_sensor_forward());
    printf("PASS: %u motor-test checks (mock registers; no physical hardware claim)\n", checks);
    return 0;
#endif
}
