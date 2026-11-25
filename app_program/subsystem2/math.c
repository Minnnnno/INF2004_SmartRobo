/** @file math.c
 *
 * @brief PID, quadrature and odometry arithmetic.
 */

#include "../../include/float_types.h"
#include "motion.h"
#include <math.h>
#include <string.h>

static float64_t clamp(float64_t value, float64_t limit);

/*! @brief Clear the controller's history. */
void
pid_reset (pid_t * p_pid)
{
    memset(p_pid, 0, sizeof(*p_pid));
}

/*! @brief One PID cycle, output limited. */
float64_t
pid_step (pid_t * p_pid, const pid_config_t * p_config, float64_t target,
          float64_t measured, float64_t delta_s, float64_t limit)
{
    float64_t error = target - measured;
    float64_t derivative
        = p_pid->seeded ? -(measured - p_pid->previous) / delta_s : 0.0;
    float64_t candidate;
    float64_t output;

    p_pid->derivative += (delta_s / (p_config->derivative_tau + delta_s))
                         * (derivative - p_pid->derivative);
    p_pid->previous = measured;
    p_pid->seeded   = true;
    candidate       = p_pid->integral + (p_config->ki * error * delta_s);
    output          = (p_config->kp * error) + candidate
                      + (p_config->kd * p_pid->derivative);

    /* Conditional integration: retain changes that unwind saturation. */
    if ((fabs(output) <= limit) || ((output > limit) && (error < 0.0))
        || ((output < -limit) && (error > 0.0)))
    {
        p_pid->integral = clamp(candidate, limit);
    }

    return clamp((p_config->kp * error) + p_pid->integral
                     + (p_config->kd * p_pid->derivative),
                 limit);
}

/*! @brief Signed difference of two modular counts. */
int64_t
encoder_delta (uint32_t now, uint32_t before)
{
    uint32_t difference = now - before;

    return (difference <= INT32_MAX)
               ? (int64_t)difference
               : (int64_t)difference - INT64_C(4294967296);
}

/*! @brief Step from one A/B state to the next; invalid jumps are flagged. */
int
encoder_quadrature (uint8_t before, uint8_t after, bool * p_invalid)
{
    /* Positive cycle 00 -> 01 -> 11 -> 10 -> 00; A is bit 1. */
    static const int8_t transitions[16]
        = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

    before &= 3U;
    after &= 3U;
    *p_invalid = (3U == (before ^ after));
    return transitions[(before * 4U) + after];
}

/*! @brief Update travel, speed and heading from new counts. */
bool
odometry_step (odometry_t * p_odom, const motion_config_t * p_config,
               const encoder_sample_t * p_sample, float64_t delta_s)
{
    float64_t change[2];
    unsigned  wheel;

    if ((!isfinite(delta_s)) || (delta_s <= 0.0)
        || (delta_s > p_config->max_dt))
    {
        return false;
    }

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        float64_t quantum = (3.141592653589793 * p_config->diameter[wheel])
                            / p_config->counts_per_rev[wheel];

        change[wheel] = (float64_t)encoder_delta(p_sample->count[wheel],
                                                 p_odom->previous[wheel])
                        * quantum;

        if ((!isfinite(change[wheel]))
            || (fabs(change[wheel])
                > ((p_config->max_speed * delta_s * 3.0) + (2.0 * quantum))))
        {
            return false;
        }
    }

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        p_odom->previous[wheel] = p_sample->count[wheel];
        p_odom->travel[wheel] += change[wheel];
        p_odom->speed[wheel] = change[wheel] / delta_s;
    }

    p_odom->centre += (change[0] + change[1]) / 2.0;
    p_odom->path += fabs((change[0] + change[1]) / 2.0);
    p_odom->heading += (change[1] - change[0]) / p_config->track;
    return true;
}

/*! @brief Limit a value to a range. */
static float64_t
clamp (float64_t value, float64_t limit)
{
    return fmax(-limit, fmin(limit, value));
}

/*** end of file ***/
