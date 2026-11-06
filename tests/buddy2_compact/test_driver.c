/** @file test_driver.c
 * @brief Test real driver polarity, packed state and reversal/stop behaviour.
 */
#include "driver_fake.h"
#include "motion_hal.h"
#include "board_config.h"
#include <stdio.h>

static uint32_t g_failures = 0u;
static uint32_t g_lock_depth = 0u;
static uint16_t g_duty[2][2] = {{0u}};
static uint32_t g_levels = 0u;
static uint32_t g_pending = 0u;
static uint32_t g_pwm_enable[2], g_functions[30], g_irq_enable[4], g_time_us = 1000000U;
static uint16_t g_top[2], g_divider[2];
static void (*gp_encoder_handler)(void) = NULL;
#define CHECK(CONDITION) do { if (!(CONDITION)) { ++g_failures; \
    (void)fprintf(stderr, "FAIL driver line %d: %s\n", __LINE__, #CONDITION); } } while (false)

uint32_t save_and_disable_interrupts(void) { return g_lock_depth++; }
void restore_interrupts(uint32_t state) { CHECK(g_lock_depth > 0u); g_lock_depth = state; }
uint64_t time_us_64(void) { return g_time_us; }
uint32_t in_w(uint32_t address)
{
    if (GPIO_IN == address) { return g_levels; }
    if (IO_BANK0_BASE + 0x120u == address) { return g_pending; }
    if (0xe000e100u == address) { return 1u << 13u; }
    if (TIMER_TIMERAWL == address) { return g_time_us; }
    for (unsigned w = 0U; w < 2U; ++w) {
        uint32_t base = PWM_BASE + (w + 4U) * 0x14U;
        if (address == base) { return g_pwm_enable[w]; }
        if (address == base + 4U) { return g_divider[w]; }
        if (address == base + 12U) { return ((uint32_t)g_duty[w][1] << 16U) | g_duty[w][0]; }
        if (address == base + 16U) { return g_top[w]; }
    }
    for (unsigned pin = 0U; pin < 30U; ++pin) {
        if (address == GPIO_CTRL(pin)) { return g_functions[pin]; }
    }
    for (unsigned bank = 0U; bank < 4U; ++bank) {
        if (address == IO_BANK0_BASE + 0x100U + bank * 4U) { return g_irq_enable[bank]; }
    }
    return 0u;
}
void out_w(unsigned address, uint32_t value)
{
    if (IO_BANK0_BASE + 0xf0u == address) { g_pending &= ~value; }
}
uint32_t irq_reg(unsigned pin, uint32_t base) { return IO_BANK0_BASE + base + (pin/8u)*4u; }
void gpio_init(unsigned pin) { g_functions[pin] = 5U; }
void gpio_put(unsigned pin, bool b_high) { (void)pin; CHECK(!b_high); }
void gpio_set_dir(unsigned pin, bool b_output) { (void)pin; (void)b_output; }
void gpio_disable_pulls(unsigned pin) { (void)pin; }
void gpio_set_function(unsigned pin, unsigned function) { CHECK(pin >= 8u && pin <= 11u); CHECK(GPIO_FUNC_PWM == function); g_functions[pin] = function; }
void gpio_set_irq_enabled(unsigned pin, uint32_t events, bool b_enabled)
{
    uint32_t mask = events << ((pin % 8U) * 4U);
    if (b_enabled) { g_irq_enable[pin / 8U] |= mask; }
    else { g_irq_enable[pin / 8U] &= ~mask; }
}
bool board_gpio_irq_register(uint32_t mask, void (*p_handler)(void))
{
    CHECK(0x3cu == mask); gp_encoder_handler = p_handler; return true;
}
pwm_config pwm_get_default_config(void) { pwm_config config = {4095u, 1.0f}; return config; }
void pwm_config_set_wrap(pwm_config *p_config, uint16_t value) { p_config->wrap = value; }
void pwm_config_set_clkdiv(pwm_config *p_config, float value) { p_config->divider = value; }
void pwm_init(unsigned slice, const pwm_config *p_config, bool b_enabled)
{
    CHECK(slice >= 4u && slice <= 5u); CHECK(4095u == p_config->wrap); CHECK(!b_enabled);
    g_top[slice - 4U] = p_config->wrap;
    g_divider[slice - 4U] = (uint16_t)(p_config->divider * 16.0f);
}
void pwm_set_both_levels(unsigned slice, uint16_t phase_a, uint16_t phase_b)
{
    CHECK(slice >= 4u && slice <= 5u);
    CHECK(0u == phase_a || 0u == phase_b);
    g_duty[slice-4u][0] = phase_a; g_duty[slice-4u][1] = phase_b;
}
void pwm_set_enabled(unsigned slice, bool b_enabled) { g_pwm_enable[slice - 4U] = b_enabled ? 1U : 0U; }
unsigned pwm_gpio_to_slice_num(unsigned pin) { return (pin >> 1u) & 7u; }
int tm_usb_console_read(void) { return -1; }

int main(void)
{
    motion_config_t config = board_motion_config();
    CHECK(motion_hal_init(&config));
    double outputs[2] = {0.1, 0.1};
    motion_hal_write(outputs, false, 1.0);
    CHECK(409u == g_duty[0][0] && 0u == g_duty[0][1]);
    CHECK(0u == g_duty[1][0] && 409u == g_duty[1][1]);
    motor_diagnostics_t motor, before_cutoff;
    g_time_us += 5000U;
    motion_hal_motor_diagnostics(&motor);
    CHECK(motor.ready && !motor.inhibited && motor.enabled_mask == 3U && motor.mismatch_mask == 0U);
    CHECK(motor.applied_permille[0] == 100 && motor.applied_permille[1] == 100);
    CHECK(motor.compare[0][0] == 409U && motor.compare[1][1] == 409U);
    CHECK(motor.configured_top == 4095U && motor.top[0] == 4095U && motor.divider_16[1] == motor.configured_divider_16);
    CHECK(motor.output_age_ms == 5U && motor.cap_permille == 1000);
    before_cutoff = motor;
    /* Register interference must appear in diagnostics without changing drive. */
    g_duty[0][0] = 0U;
    motion_hal_motor_diagnostics(&motor);
    CHECK(motor.mismatch_mask == 1U && motor.expected_compare[0][0] == 409U && motor.compare[0][0] == 0U);
    g_duty[0][0] = 409U;
    g_functions[10] = 5U;
    motion_hal_motor_diagnostics(&motor);
    CHECK(motor.mismatch_mask == 2U);
    g_functions[10] = GPIO_FUNC_PWM;
    outputs[0] = -0.1;
    motion_hal_write(outputs, false, 1.01);
    CHECK(0u == g_duty[0][0] && 0u == g_duty[0][1] && 409u == g_duty[1][1]);
    motion_hal_motor_diagnostics(&motor);
    CHECK(motor.reversal_wait_mask == 1U && motor.requested_permille[0] == -100);
    CHECK(motor.applied_permille[0] == 0 && motor.applied_permille[1] == 100);
    CHECK(motor.mismatch_mask == 0U);
    motion_hal_write(outputs, false, 1.06);
    CHECK(0u == g_duty[0][1]);
    motion_hal_write(outputs, false, 1.12);
    CHECK(409u == g_duty[0][1] && 409u == g_duty[1][1]);
    motion_hal_motor_diagnostics(&motor);
    CHECK(motor.applied_permille[0] == -100 && motor.reversal_wait_mask == 0U);
    motion_hal_write(outputs, true, 1.13);
    CHECK(0u == g_duty[0][1] && 0u == g_duty[1][1]);
    motion_hal_motor_diagnostics(&motor);
    CHECK(motor.inhibited && motor.applied_permille[0] == 0 && motor.applied_permille[1] == 0);
    CHECK(before_cutoff.compare[0][0] == 409U && before_cutoff.applied_permille[0] == 100);
    /* Production accepts the full PWM range and clamps over-range requests
     * to 100%, including after direction reversal and during inhibition. */
    outputs[0] = outputs[1] = 1.2;
    motion_hal_write(outputs, false, 1.25);
    CHECK(g_duty[0][0] == 4096U && g_duty[1][1] == 4096U);
    motion_hal_motor_diagnostics(&motor);
    CHECK(motor.cap_permille == 1000 && motor.applied_permille[0] == 1000 &&
          motor.applied_permille[1] == 1000 && motor.mismatch_mask == 0U);
    outputs[0] = outputs[1] = -1.2;
    motion_hal_write(outputs, false, 1.26);
    CHECK(g_duty[0][0] == 0U && g_duty[1][1] == 0U);
    motion_hal_write(outputs, false, 1.37);
    CHECK(g_duty[0][1] == 4096U && g_duty[1][0] == 4096U);
    motion_hal_motor_diagnostics(&motor);
    CHECK(motor.applied_permille[0] == -1000 && motor.applied_permille[1] == -1000);
    motion_hal_write(outputs, true, 1.38);
    CHECK(g_duty[0][1] == 0U && g_duty[1][0] == 0U);
    /* Simultaneous B edges must update each wheel independently, with its
     * own polarity; the returned pair is copied under interrupt exclusion. */
    g_levels = (1u << 3u) | (1u << 5u);
    g_pending = (8u << 12u) | (8u << 20u);
    CHECK(NULL != gp_encoder_handler);
    gp_encoder_handler();
    encoder_sample_t sample = motion_hal_sample();
    CHECK(sample.reliable && UINT32_MAX == sample.count[0] && 1u == sample.count[1]);
    CHECK(0u == sample.invalid[0] && 0u == sample.invalid[1] && 0u == g_lock_depth);
    encoder_diagnostics_t diagnostics;
    motion_hal_encoder_diagnostics(&diagnostics);
    CHECK(diagnostics.initialised && diagnostics.irq_registered && !diagnostics.motors_inhibited);
    CHECK(1u == diagnostics.previous_ab[0] && 1u == diagnostics.previous_ab[1]);
    motion_hal_emergency_stop();
    motion_hal_encoder_diagnostics(&diagnostics);
    CHECK(diagnostics.motors_inhibited && diagnostics.initialised && diagnostics.irq_registered);
    motion_hal_motor_diagnostics(&motor);
    CHECK(!motor.ready && motor.inhibited && motor.applied_permille[0] == 0 && motor.applied_permille[1] == 0);
    (void)printf("Buddy 2 driver: %s (polarity, independent reversal, inhibition, coherent capture, emergency stop)\n",
                 0u == g_failures ? "PASS" : "FAIL");
    return 0u == g_failures ? 0 : 1;
}
