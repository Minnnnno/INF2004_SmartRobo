/*
 *	servo.c
 *	Subsystem 5 -- hobby servo on GP15: 50 Hz, 500..2500 us for 0..180 deg.
 *
 *	The ultrasonic sensor rides on the horn, so every move waits long enough
 *	for the horn to arrive AND the sensor to stop swinging before a ping.
 */

#include "servo.h"
#include "../subsystem2/template_hal.h"
#include <math.h>

#define SERVO_PERIOD_US		20000U	/* 50 Hz frame */
#define SERVO_US_AT_0		500.0
#define SERVO_US_AT_180		2500.0
#define SERVO_SETTLE_BASE_MS	100U	/* mechanical ringing after any move */
#define SERVO_SETTLE_PER_DEG_MS	4U	/* SG90 ~1.7 ms/deg unloaded; loaded margin */
#define SERVO_INIT_SETTLE_MS	600U	/* unknown start position: up to 180 deg */

static unsigned	servo_slice;
static int	servo_deg = -1;

static void servo_write(int angle_deg)
{
	double	trimmed = fmin(fmax((double)(angle_deg + S5_SERVO_TRIM_DEG), 0.0), 180.0);
	double	position = (S5_SERVO_REVERSED != 0) ? (180.0 - trimmed) : trimmed;
	double	pulse = SERVO_US_AT_0 + position * (SERVO_US_AT_180 - SERVO_US_AT_0) / 180.0;

	/* Odd GPIOs use channel B; the alternate diagnostic pin uses channel A. */
	uint16_t level = (uint16_t)(pulse + 0.5);
	pwm_set_both_levels(servo_slice, (S5_SERVO_PIN & 1U) ? 0U : level,
			    (S5_SERVO_PIN & 1U) ? level : 0U);
}

static int servo_clamp(int angle_deg)
{
	return (angle_deg < S5_SERVO_MIN_DEG) ? S5_SERVO_MIN_DEG :
	       ((angle_deg > S5_SERVO_MAX_DEG) ? S5_SERVO_MAX_DEG : angle_deg);
}

void s5_servo_init(int angle_deg)
{
	pwm_config	config = pwm_get_default_config();

	servo_slice = pwm_gpio_to_slice_num(S5_SERVO_PIN);
	pwm_config_set_wrap(&config, (uint16_t)(SERVO_PERIOD_US - 1U));
	pwm_config_set_clkdiv(&config, (float)TMCLK_MHz);	/* 1 count per us */
	pwm_init(servo_slice, &config, false);

	servo_deg = servo_clamp(angle_deg);
	servo_write(servo_deg);					/* pulse set before output */
#if !defined(S5_DIAG) || (S5_DIAG != 4)	/* diagnostic 4: slice runs, pin untouched */
	gpio_set_function(S5_SERVO_PIN, GPIO_FUNC_PWM);
#endif
	pwm_set_enabled(servo_slice, true);
	(void)tk_dly_tsk(SERVO_INIT_SETTLE_MS);
}

void s5_servo_move(int angle_deg)
{
	int	target = servo_clamp(angle_deg);
	int	travel = (target > servo_deg) ? (target - servo_deg) : (servo_deg - target);

	if (servo_deg < 0) {
		s5_servo_init(target);
		return;
	}
	servo_write(target);
	servo_deg = target;
	(void)tk_dly_tsk((RELTIM)(SERVO_SETTLE_BASE_MS + (unsigned)travel * SERVO_SETTLE_PER_DEG_MS));
}

int s5_servo_angle(void)
{
	return servo_deg;
}
