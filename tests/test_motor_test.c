#include "motor_test.h"
#include "motion_hal.h"
#include "board_config.h"
#include "motor_test_fake_hal.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); exit(1); } } while (0)
static unsigned checks, writes;
static bool enabled[2], alarm_armed, expect_pair;
static uint16_t duty[2][2];
static uint64_t now;
static uint32_t deadline;
static FP deadline_handler;
static encoder_sample_t sample = {.reliable = true};
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
    if (enable) { CHECK(alarm_armed); CHECK(expect_pair || !enabled[1U - (slice - 4U)]); }
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

int main(void)
{
    motor_test_result_t result;
    unsigned wheel, direction;
    motion_config_t config = board_motion_config();
    CHECK(config.counts_per_rev[0] == 2542.4 && config.counts_per_rev[1] == 2543.4);
    CHECK(!config.hardware_confirmed && !config.manual_enabled);
    CHECK(config.motor_sign[0] == 1 && config.motor_sign[1] == 1);
    CHECK(config.encoder_sign[0] == 1 && config.encoder_sign[1] == 1);
    CHECK(config.diameter[0] == 0.065 && config.diameter[1] == 0.065 && config.track == 0.115);
    CHECK(encoder_pins[0][0] == 4U && encoder_pins[0][1] == 6U);
    CHECK(encoder_pins[1][0] == 5U && encoder_pins[1][1] == 7U);
    sample.reliable = false;
    CHECK(!motor_test_init());
    CHECK(!motor_test_start(0U, 1));
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
            unsigned driven_channel = direction;
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
        CHECK(duty[0][direction] == 1000U && duty[0][1U - direction] == 0U);
        CHECK(duty[1][direction] == 1000U && duty[1][1U - direction] == 0U);
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
    printf("PASS: %u motor-test checks (mock registers; no physical hardware claim)\n", checks);
    return 0;
}
