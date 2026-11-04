/*
 *	servo.h
 *	Subsystem 5 -- hobby servo on GP15 that carries the ultrasonic sensor.
 *	Task context only.  Owner: obstacle_task.
 */

#ifndef S5_SERVO_H
#define S5_SERVO_H

#if defined(S5_DIAG) && (S5_DIAG == 6)
#define S5_SERVO_PIN		14U	/* diagnostic: slice 7 channel A instead */
#else
#define S5_SERVO_PIN		15U	/* PWM slice 7 channel B, 50 Hz */
#endif
/* 1 if the sweep starts on the RIGHT: reverses the angle-to-pulse mapping */
#define S5_SERVO_REVERSED	1 /* confirmed 2026-11-04: increasing pulse points left */
/* Added AFTER direction reversal at the pulse so that 90 deg points straight ahead.
   Five scans of centred objects put their middle at 90.8 deg on average
   without trim (95, 85.5, 91, 89.5, 93; 2026-11-03). */
#define S5_SERVO_TRIM_DEG	1
/* Mechanical limits: every request is clamped into this range */
#define S5_SERVO_MIN_DEG	20
#define S5_SERVO_MAX_DEG	160

/* Start the PWM at angle_deg and wait for the horn to get there */
void	s5_servo_init(int angle_deg);
/* Move and block until the sensor on top has stopped swinging */
void	s5_servo_move(int angle_deg);
/* Start the same move without waiting; returns the ms to wait before a ping */
unsigned s5_servo_aim(int angle_deg);
int	s5_servo_angle(void);

#endif /* S5_SERVO_H */
