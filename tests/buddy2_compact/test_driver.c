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
static void (*gp_encoder_handler)(void) = NULL;
#define CHECK(CONDITION) do { if (!(CONDITION)) { ++g_failures; \
    (void)fprintf(stderr, "FAIL driver line %d: %s\n", __LINE__, #CONDITION); } } while (false)

uint32_t save_and_disable_interrupts(void) { return g_lock_depth++; }
void restore_interrupts(uint32_t state) { CHECK(g_lock_depth > 0u); g_lock_depth = state; }
uint64_t time_us_64(void) { return 0u; }
uint32_t in_w(uint32_t address)
{
    if (GPIO_IN == address) { return g_levels; }
    if (IO_BANK0_BASE + 0x120u == address) { return g_pending; }
    if (0xe000e100u == address) { return 1u << 13u; }
    return 0u;
}
void out_w(unsigned address, uint32_t value)
{
    if (IO_BANK0_BASE + 0xf0u == address) { g_pending &= ~value; }
}
uint32_t irq_reg(unsigned pin, uint32_t base) { return IO_BANK0_BASE + base + (pin/8u)*4u; }
void gpio_init(unsigned pin) { (void)pin; }
void gpio_put(unsigned pin, bool b_high) { (void)pin; CHECK(!b_high); }
void gpio_set_dir(unsigned pin, bool b_output) { (void)pin; (void)b_output; }
void gpio_disable_pulls(unsigned pin) { (void)pin; }
void gpio_set_function(unsigned pin, unsigned function) { CHECK(pin >= 8u && pin <= 11u); CHECK(GPIO_FUNC_PWM == function); }
void gpio_set_irq_enabled(unsigned pin, uint32_t events, bool b_enabled) { (void)pin; (void)events; (void)b_enabled; }
bool board_gpio_irq_register(uint32_t mask, void (*p_handler)(void))
{
    CHECK(0xf0u == mask); gp_encoder_handler = p_handler; return true;
}
pwm_config pwm_get_default_config(void) { pwm_config config = {4095u, 1.0f}; return config; }
void pwm_config_set_wrap(pwm_config *p_config, uint16_t value) { p_config->wrap = value; }
void pwm_config_set_clkdiv(pwm_config *p_config, float value) { p_config->divider = value; }
void pwm_init(unsigned slice, const pwm_config *p_config, bool b_enabled)
{
    CHECK(slice >= 4u && slice <= 5u); CHECK(4095u == p_config->wrap); CHECK(!b_enabled);
}
void pwm_set_both_levels(unsigned slice, uint16_t phase_a, uint16_t phase_b)
{
    CHECK(slice >= 4u && slice <= 5u);
    CHECK(0u == phase_a || 0u == phase_b);
    g_duty[slice-4u][0] = phase_a; g_duty[slice-4u][1] = phase_b;
}
void pwm_set_enabled(unsigned slice, bool b_enabled) { (void)slice; (void)b_enabled; }
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
    outputs[0] = -0.1;
    motion_hal_write(outputs, false, 1.01);
    CHECK(0u == g_duty[0][0] && 0u == g_duty[0][1] && 409u == g_duty[1][1]);
    motion_hal_write(outputs, false, 1.06);
    CHECK(0u == g_duty[0][1]);
    motion_hal_write(outputs, false, 1.12);
    CHECK(409u == g_duty[0][1] && 409u == g_duty[1][1]);
    motion_hal_write(outputs, true, 1.13);
    CHECK(0u == g_duty[0][1] && 0u == g_duty[1][1]);
    /* Simultaneous B edges must update each wheel independently, with its
     * own polarity; the returned pair is copied under interrupt exclusion. */
    g_levels = (1u << 6u) | (1u << 7u);
    g_pending = (8u << 24u) | (8u << 28u);
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
    (void)printf("Buddy 2 driver: %s (polarity, independent reversal, inhibition, coherent capture, emergency stop)\n",
                 0u == g_failures ? "PASS" : "FAIL");
    return 0u == g_failures ? 0 : 1;
}
