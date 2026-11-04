/*
 *	ultrasonic.c
 *	Subsystem 5 -- HC-SR04 style ultrasonic ranger.
 *
 *	The echo pulse is timed in hardware.  GP17 is the B pin of PWM slice 0;
 *	in "level" divider mode the slice counter advances once per microsecond
 *	only while that pin is high.  The task sleeps until the pulse is over and
 *	then reads the counter, so interrupt latency and scheduling never change
 *	the measured width (1 us = 0.17 mm).
 *
 *	Timing: 10+ us trigger; echo accepted for 30 ms (5 m round trip, beyond
 *	the 4 m range); triggers at least 60 ms apart so late echoes of one ping
 *	cannot be heard by the next.
 */

#include "ultrasonic.h"
#include "../subsystem2/template_hal.h"

#define US_TRIGGER_US		12U
#define US_CYCLE_US		((uint64_t)S5_US_CYCLE_MS * 1000U)
#define US_ECHO_TIMEOUT_US	30000U
#define US_IDLE_TIMEOUT_MS	200U	/* modules hold ECHO high ~38 ms or more on a miss */
#define US_MIN_PULSE_US		117U	/* 2 cm, the sensor's minimum range */
#define US_AIR_TEMP_C		25.0	/* room temperature, for the speed of sound */
#define US_SOUND_MM_PER_US	((331.3 + 0.606 * US_AIR_TEMP_C) / 1000.0)

#define US_PWM_CSR_EN		(1U << 0U)
#define US_PWM_CSR_LEVEL	(1U << 4U)	/* DIVMODE: count while B is high */

static unsigned	us_slice;
static uint64_t	us_last_trigger;
static bool	us_ready;

static UW us_reg(UW offset)
{
	return PWM_BASE + us_slice * 0x14U + offset;	/* 0 CSR, 8 CTR */
}

static UW us_counter(void)
{
	return in_w(us_reg(8U)) & 0xffffU;
}

bool s5_us_init(void)
{
	pwm_config	config = pwm_get_default_config();

	gpio_init(S5_US_TRIG_PIN);
	gpio_put(S5_US_TRIG_PIN, false);
	gpio_set_dir(S5_US_TRIG_PIN, true);

	/* Pull-down: an unplugged ECHO reads "no pulse" instead of floating */
	gpio_init(S5_US_ECHO_PIN);
	gpio_set_dir(S5_US_ECHO_PIN, false);
	clr_w(GPIO(S5_US_ECHO_PIN), GPIO_PUE);
	set_w(GPIO(S5_US_ECHO_PIN), GPIO_PDE);

	us_slice = pwm_gpio_to_slice_num(S5_US_ECHO_PIN);
	pwm_config_set_wrap(&config, 0xffffU);			/* 65 ms > echo timeout */
	pwm_config_set_clkdiv(&config, (float)TMCLK_MHz);	/* 1 count per us */
	pwm_init(us_slice, &config, false);
	out_w(us_reg(0U), US_PWM_CSR_LEVEL | US_PWM_CSR_EN);
	gpio_set_function(S5_US_ECHO_PIN, GPIO_FUNC_PWM);	/* B is an input in this mode */

	us_last_trigger = time_us_64() - US_CYCLE_US;
	us_ready = true;
	return true;
}

s5_us_status_t s5_us_ping(double *p_range_mm, double *p_time_s)
{
	uint64_t	start;
	uint64_t	since;
	UW		pulse;
	unsigned	waited;

	*p_range_mm = 0.0;
	*p_time_s = 0.0;
	if (!us_ready) {
		return S5_US_NO_PULSE;
	}

	/* Let a previous pulse finish; the sensor ignores triggers until then */
	for (waited = 0U; gpio_get(S5_US_ECHO_PIN); waited++) {
		if (waited >= US_IDLE_TIMEOUT_MS) {
			return S5_US_BUSY;
		}
		(void)tk_dly_tsk(1U);
	}
	since = time_us_64() - us_last_trigger;
	if (since < US_CYCLE_US) {
		(void)tk_dly_tsk((RELTIM)((US_CYCLE_US - since + 999U) / 1000U));
	}

	out_w(us_reg(8U), 0U);				/* clear the pulse timer */
	gpio_put(S5_US_TRIG_PIN, true);
	start = time_us_64();
	while (time_us_64() - start < US_TRIGGER_US) {
		;					/* 12 us, too short to sleep */
	}
	gpio_put(S5_US_TRIG_PIN, false);
	us_last_trigger = start;

	for (;;) {
		UW	before;
		bool	b_high;

		(void)tk_dly_tsk(1U);
		/* Counter unchanged across a low pin read: the pulse has ended */
		before = us_counter();
		b_high = gpio_get(S5_US_ECHO_PIN);
		pulse = us_counter();
		if (!b_high && (pulse != 0U) && (pulse == before)) {
			break;
		}
		if (time_us_64() - start >= US_ECHO_TIMEOUT_US) {
			return (pulse == 0U && !b_high) ? S5_US_NO_PULSE : S5_US_NO_ECHO;
		}
	}

	if (pulse < US_MIN_PULSE_US) {
		return S5_US_NO_ECHO;
	}
	*p_range_mm = (double)pulse * US_SOUND_MM_PER_US / 2.0;
	if (*p_range_mm > S5_US_MAX_MM) {
		return S5_US_NO_ECHO;
	}
	/* The burst leaves ~0.45 ms after the trigger; the reflection is mid-pulse */
	*p_time_s = (double)(start + 450U + pulse / 2U) / 1000000.0;
	return S5_US_OK;
}
