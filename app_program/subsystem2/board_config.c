#include "../../include/float_types.h"
#include "motion.h"
#include "board_config.h"

#define ENCODER_PIN_COUNT (MOTION_WHEELS * ENCODER_CHANNELS)
#include <math.h>

const unsigned g_motor_pins[2][2] = {{8U, 9U}, {10U, 11U}};
/* Encoder connector pin 2 = A, pin 3 = B; pin 1 = GND, pin 4 = 3V3.
 * Rows are left/right; columns are A/B. Values below are Pico GPIO numbers. */
const unsigned g_encoder_pins[2][2] = {
    {BOARD_ENC_L_A, BOARD_ENC_L_B}, /* M1: ENC_L_A -> GP2; ENC_L_B -> GP3 */
    {BOARD_ENC_R_A, BOARD_ENC_R_B}  /* M2: ENC_R_A -> GP4; ENC_R_B -> GP5 */
};

bool
board_pins_valid (void)
{
    unsigned       index;
    unsigned       other;
    const unsigned enc[] = {g_encoder_pins[0][0], g_encoder_pins[0][1],
                            g_encoder_pins[1][0], g_encoder_pins[1][1]};

    if ((8U != g_motor_pins[0][0]) || (9U != g_motor_pins[0][1])
        || (10U != g_motor_pins[1][0]) || (11U != g_motor_pins[1][1]))
    {
        return false;
    }

    for (index = 0; index < ENCODER_PIN_COUNT; ++index)
    {
        if (enc[index] > 29U)
        {
            return false;
        }

        if ((enc[index] < 2U) || (enc[index] > 5U))
        {
            return false;
        }

        if ((BOARD_I2C_SDA == enc[index]) || (BOARD_I2C_SCL == enc[index]))
        {
            return false;
        }

        for (other = 0; other < index; ++other)
        {
            if (enc[index] == enc[other])
            {
                return false;
            }
        }
    }

    return true;
}

motion_config_t
board_motion_config (void)
{
    /* Provisional effective diameters from three passive 1000 mm rolls.
     * Validate under power; these include rolling/measurement effects. */
    motion_config_t config
        = {.counts_per_rev = {2542.4, 2543.4},
           .diameter       = {0.0675, 0.0679},
           .track          = 0.115,
           /* Motor polarity retains the verified forward drive direction.
            * 2026-11-04: user confirmed physical forward motion while BOTH
            * measured speeds were negative with encoder signs {+1,-1}; invert
            * feedback only. */
           .motor_sign   = {1, -1},
           .encoder_sign = {-1, 1},
           .quadrature   = {true, true},
           /* Provisional tuning only; validate on the actual robot. */
           .max_speed          = 0.25,
           .acceleration       = 0.3,
           .max_output         = 1.0,
           .period             = 0.01,
           .max_dt             = 0.05,
           .distance_tolerance = 0.01,
           .angle_tolerance    = 0.05,
           .stopped_speed      = 0.005,
           .settle_time        = 0.3,
           .startup_grace      = 0.4,
           .feedback_timeout   = 0.4,
           .stop_timeout       = 3.0,
           .setup_timeout      = 2.0,
           .command_lifetime   = 0.3,
           .movement_timeout   = 30.0,
           .max_distance       = 5.0,
           .max_angle          = 6.283185307179586,
           .straight_gain      = 1.0,
           .reversal_time      = 0.1,
           .manual_limit       = 0.15,
           .manual_timeout     = 0.5,
           /* 2026-11-18 COM4 logs: the sqrt deceleration ramp reached 35 mm/s
            * about 2 mm before the goal; both wheels stopped at ~16-18% PWM and
            * the no-progress check faulted 0.6 mm short of a detour turn. At 70
            * mm/s the P term alone requests ~21%. Braking from it measured well
            * inside the goal tolerance (detour turn 5102 stopped ~0 mm past its
            * trigger). */
           .min_goal_speed = 0.07,
           /* 2026-11-18 COM4 logs: from rest the PID reached only 11.5% (31
            * mm/s line correction) and 19.2% (50 mm/s junction advance) on the
            * right wheel within the 0.4 s no-progress limit, and the wheel
            * never turned. While moving it needs ~19.5% for 38 mm/s; the left
            * does 60 mm/s on 10% but stalled at 16-18% in a pivot. Applied only
            * while stationary and not slowing down, doubling over
            * feedback_timeout so a higher real threshold is still passed. */
           .breakaway = {0.18, 0.22},
           /* Raised from {1.0, 0.5}: 0.08 m/s started at ~8% PWM, below the
            * right motor's starting threshold, so the 0.8 s no-progress check
            * faulted (Buddy 3 test). */
           .pid                = {{3.0, 2.0, 0.0, 0.05}, {3.0, 2.0, 0.0, 0.05}},
           .pwm_hz             = 20000U,
           .hardware_confirmed = true,
           .brake              = true,
           .manual_enabled     = false};
#if defined(ROBO_ENCODER_DIAGNOSTICS_ONLY) || defined(ROBO_SAFE_MOTOR_TEST) \
    || defined(ROBO_ANGLE_TEST) || defined(ROBO_MANUAL_CONSOLE)
    /* Calibration images retain their isolated actuator and inhibited
     * controller. */
    config.hardware_confirmed = false;
#endif
    return config;
}

bool
motion_config_valid (const motion_config_t * p_config)
{
    unsigned        wheel;
    const float64_t positive[] = {p_config->track,
                                  p_config->max_speed,
                                  p_config->acceleration,
                                  p_config->max_output,
                                  p_config->period,
                                  p_config->max_dt,
                                  p_config->distance_tolerance,
                                  p_config->angle_tolerance,
                                  p_config->stopped_speed,
                                  p_config->settle_time,
                                  p_config->startup_grace,
                                  p_config->feedback_timeout,
                                  p_config->stop_timeout,
                                  p_config->setup_timeout,
                                  p_config->command_lifetime,
                                  p_config->movement_timeout,
                                  p_config->max_distance,
                                  p_config->max_angle,
                                  p_config->reversal_time,
                                  p_config->manual_limit,
                                  p_config->manual_timeout};

    for (wheel = 0; wheel < (sizeof(positive) / sizeof(positive[0])); ++wheel)
    {
        if ((!isfinite(positive[wheel])) || (positive[wheel] <= 0.0))
        {
            return false;
        }
    }

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        const pid_config_t * p_pid_config = &p_config->pid[wheel];

        if ((!isfinite(p_config->counts_per_rev[wheel]))
            || (p_config->counts_per_rev[wheel] < 1.0)
            || (!isfinite(p_config->diameter[wheel]))
            || (p_config->diameter[wheel] <= 0.0)
            || ((1 != p_config->motor_sign[wheel])
                && (-1 != p_config->motor_sign[wheel]))
            || ((1 != p_config->encoder_sign[wheel])
                && (-1 != p_config->encoder_sign[wheel]))
            || (!isfinite(p_config->breakaway[wheel]))
            || (p_config->breakaway[wheel] < 0.0)
            || (p_config->breakaway[wheel] > p_config->max_output)
            || (!isfinite(p_pid_config->kp)) || (!isfinite(p_pid_config->ki))
            || (!isfinite(p_pid_config->kd))
            || (!isfinite(p_pid_config->derivative_tau))
            || (p_pid_config->kp < 0.0) || (p_pid_config->ki < 0.0)
            || (p_pid_config->kd < 0.0) || (p_pid_config->derivative_tau < 0.0))
        {
            return false;
        }
    }

    return (p_config->hardware_confirmed) && (p_config->max_output <= 1.0)
           && (p_config->manual_limit <= p_config->max_output)
           && (p_config->manual_timeout <= p_config->movement_timeout)
           && (p_config->period <= p_config->max_dt)
           && (p_config->settle_time < p_config->stop_timeout)
           && (p_config->stopped_speed < p_config->max_speed)
           && (p_config->pwm_hz >= 1000U) && (p_config->pwm_hz <= 20000U)
           && (isfinite(p_config->straight_gain))
           && (p_config->straight_gain >= 0.0)
           && (isfinite(p_config->min_goal_speed))
           && (p_config->min_goal_speed >= 0.0)
           && (p_config->min_goal_speed < p_config->max_speed);
}

/*** end of file ***/
