/* Exercise the actual servo PWM mapping against the confirmed physical mounting:
   low pulse = right, high pulse = left. No GPIO or hardware is touched. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define ROBO_TEMPLATE_HAL_H
#define TMCLK_MHz 125
#define GPIO_FUNC_PWM 4U
typedef uint32_t RELTIM;
typedef struct { uint16_t wrap; float divider; } pwm_config;
static struct { unsigned slice, pin; uint16_t a, b; uint32_t delay; bool enabled; } rig;
static unsigned pwm_gpio_to_slice_num(unsigned pin) { return (pin >> 1U) & 7U; }
static pwm_config pwm_get_default_config(void) { return (pwm_config){0U, 0.0f}; }
static void pwm_config_set_wrap(pwm_config *p, uint16_t wrap) { p->wrap = wrap; }
static void pwm_config_set_clkdiv(pwm_config *p, float divider) { p->divider = divider; }
static void pwm_init(unsigned slice, const pwm_config *p, bool enable)
{ (void)p; rig.slice = slice; rig.enabled = enable; }
static void pwm_set_both_levels(unsigned slice, uint16_t a, uint16_t b)
{ rig.slice = slice; rig.a = a; rig.b = b; }
static void gpio_set_function(unsigned pin, unsigned function) { (void)function; rig.pin = pin; }
static void pwm_set_enabled(unsigned slice, bool enabled) { rig.slice = slice; rig.enabled = enabled; }
static int tk_dly_tsk(RELTIM delay) { rig.delay = delay; return 0; }

#include "../../app_program/subsystem5/servo.c"

static unsigned checks, failures;
#define CHECK(test) do { ++checks; if (!(test)) { ++failures; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #test); } } while (0)

int main(void)
{
    s5_servo_init(90);
    CHECK(rig.enabled && rig.pin == 15U && rig.slice == 7U && rig.a == 0U);
    CHECK(rig.b == 1511U); /* Keep the established ahead pulse (91 physical degrees). */
    uint16_t ahead = rig.b;
    s5_servo_move(20);
    uint16_t left = rig.b;
    CHECK(left == 2289U && left > ahead); /* Logical left needs the high physical pulse. */
    CHECK(s5_servo_angle() == 20);
    s5_servo_move(160);
    CHECK(rig.b == 733U && rig.b < ahead); /* Logical right needs the low physical pulse. */
    CHECK(s5_servo_angle() == 160);
    s5_servo_move(-10);
    CHECK(s5_servo_angle() == 20 && rig.b == left);
    s5_servo_move(200);
    CHECK(s5_servo_angle() == 160 && rig.b == 733U);
    s5_servo_move(90);
    CHECK(rig.b == ahead);
    rig.delay = 0U;
    CHECK(s5_servo_aim(20) == 380U); /* 100 ms + 70 deg at 4 ms/deg, and no wait here. */
    CHECK(rig.delay == 0U && s5_servo_angle() == 20 && rig.b == left);
    printf("%u/%u obstacle servo checks passed\n", checks - failures, checks);
    return failures == 0U ? 0 : 1;
}
