/*
 *	ultrasonic.h
 *	Subsystem 5 -- HC-SR04 style ultrasonic ranger on TRIG GP16, ECHO GP17.
 *	Task context only (it sleeps while the echo travels).  Owner: obstacle_task.
 */

#ifndef S5_ULTRASONIC_H
#define S5_ULTRASONIC_H

#include <stdbool.h>

#define S5_US_TRIG_PIN		16U
#define S5_US_ECHO_PIN		17U	/* PWM slice 0 channel B: hardware pulse timer */
#define S5_US_CYCLE_MS		60U	/* trigger-to-trigger minimum (datasheet) */
#define S5_US_MAX_MM		4000.0	/* sensor range limit */

typedef enum {
	S5_US_OK,		/* echo received, range valid */
	S5_US_NO_ECHO,		/* nothing within range (or a reading below 2 cm) */
	S5_US_NO_PULSE,		/* the sensor never raised ECHO: check wiring/power */
	S5_US_BUSY		/* ECHO stuck high from an earlier ping */
} s5_us_status_t;

bool		s5_us_init(void);
/* One ping.  Waits for the 60 ms cycle, triggers, and sleeps until the echo
   pulse ends (<= ~31 ms).  *p_time_s is when the sound hit the obstacle,
   on the motion_hal_time() clock. */
s5_us_status_t	s5_us_ping(double *p_range_mm, double *p_time_s);

#endif /* S5_ULTRASONIC_H */
