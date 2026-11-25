/*
 *  servo.c
 *  Subsystem 5 -- hobby servo on GP15: 50 Hz, 500..2500 us for 0..180 deg.
 *
 *  The ultrasonic sensor rides on the horn, so every move waits long enough
 *  for the horn to arrive AND the sensor to stop swinging before a ping.
 */

#include "../../include/float_types.h"
#include "servo.h"
#include "../subsystem2/template_hal.h"
#include <math.h>

#define SERVO_PERIOD_US      20000U /* 50 Hz frame */
#define SERVO_US_AT_0        500.0
#define SERVO_US_AT_180      2500.0
#define SERVO_SETTLE_BASE_MS 100U /* mechanical ringing after any move */
/* SG90 ~1.7 ms/deg unloaded; loaded margin */
#define SERVO_SETTLE_PER_DEG_MS 4U
/* unknown start position: up to 180 deg */
#define SERVO_INIT_SETTLE_MS 600U

#define SERVO_SLICE ((S5_SERVO_PIN >> 1U) & 7U) /* fixed by the pin */

static uint8_t g_servo_deg; /* 0: not started */

static void
servo_write (int angle_deg)
{
    float64_t mapped = (S5_SERVO_REVERSED != 0) ? (180.0 - (float64_t)angle_deg)
                                                : (float64_t)angle_deg;
    float64_t position
        = fmin(fmax(mapped + (float64_t)S5_SERVO_TRIM_DEG, 0.0), 180.0);
    float64_t pulse
        = SERVO_US_AT_0
          + ((position * (SERVO_US_AT_180 - SERVO_US_AT_0)) / 180.0);

    /* Odd GPIOs use channel B; the alternate diagnostic pin uses channel A. */
    uint16_t level = (uint16_t)(pulse + 0.5);

    pwm_set_both_levels(SERVO_SLICE, (S5_SERVO_PIN & 1U) ? 0U : level,
                        (S5_SERVO_PIN & 1U) ? level : 0U);
}

static int
servo_clamp (int angle_deg)
{
    return (angle_deg < S5_SERVO_MIN_DEG)
               ? S5_SERVO_MIN_DEG
               : ((angle_deg > S5_SERVO_MAX_DEG) ? S5_SERVO_MAX_DEG
                                                 : angle_deg);
}

void
s5_servo_init (int angle_deg)
{
    pwm_config_t config = pwm_get_default_config();

    pwm_config_set_wrap(&config, (uint16_t)(SERVO_PERIOD_US - 1U));
    pwm_config_set_clkdiv(&config, (float32_t)TMCLK_MHz); /* 1 count per us */
    pwm_init(SERVO_SLICE, &config, false);

    g_servo_deg = (uint8_t)servo_clamp(angle_deg);
    servo_write(g_servo_deg); /* pulse set before output */
/* diagnostic 4: slice runs, pin untouched */
#if !defined(S5_DIAG) || (S5_DIAG != 4)
    gpio_set_function(S5_SERVO_PIN, GPIO_FUNC_PWM);
#endif
    pwm_set_enabled(SERVO_SLICE, true);
    (void)tk_dly_tsk(SERVO_INIT_SETTLE_MS);
}

unsigned
s5_servo_aim (int angle_deg)
{
    int target = servo_clamp(angle_deg);
    int travel = (target > g_servo_deg) ? (target - g_servo_deg)
                                        : (g_servo_deg - target);

    if (0U == g_servo_deg)
    {
        s5_servo_init(target); /* waits for itself */
        return 0U;
    }

    servo_write(target);
    g_servo_deg = (uint8_t)target;
    return SERVO_SETTLE_BASE_MS + ((unsigned)travel * SERVO_SETTLE_PER_DEG_MS);
}

void
s5_servo_move (int angle_deg)
{
    unsigned settle_ms = s5_servo_aim(angle_deg);

    if (settle_ms > 0U)
    {
        (void)tk_dly_tsk((RELTIM)settle_ms);
    }
}

int
s5_servo_angle (void)
{
    return g_servo_deg;
}

/*** end of file ***/
