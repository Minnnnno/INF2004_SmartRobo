#include "../../include/float_types.h"
#include "motion.h"
#include <math.h>
#include <string.h>

static float64_t
clamp (float64_t value, float64_t limit)
{
    return fmax(-limit, fmin(limit, value));
}

void
pid_reset (pid_t * pid)
{
    memset(pid, 0, sizeof(*pid));
}

float64_t
pid_step (pid_t * pid, const pid_config_t * config, float64_t target,
          float64_t measured, float64_t dt, float64_t limit)
{
    float64_t error      = target - measured;
    float64_t derivative = pid->seeded ? -(measured - pid->previous) / dt : 0.0;
    float64_t candidate;
    float64_t output;

    pid->derivative += (dt / (config->derivative_tau + dt))
                       * (derivative - pid->derivative);
    pid->previous = measured;
    pid->seeded   = true;
    candidate     = pid->integral + (config->ki * error * dt);
    output = (config->kp * error) + candidate + (config->kd * pid->derivative);

    /* Conditional integration: retain changes that unwind saturation. */
    if ((fabs(output) <= limit) || ((output > limit) && (error < 0.0))
        || ((output < -limit) && (error > 0.0)))
    {
        pid->integral = clamp(candidate, limit);
    }

    return clamp((config->kp * error) + pid->integral
                     + (config->kd * pid->derivative),
                 limit);
}

int64_t
encoder_delta (uint32_t now, uint32_t before)
{
    uint32_t difference = now - before;

    return (difference <= INT32_MAX)
               ? (int64_t)difference
               : (int64_t)difference - INT64_C(4294967296);
}

int
encoder_quadrature (uint8_t before, uint8_t after, bool * invalid)
{
    /* Positive cycle 00 -> 01 -> 11 -> 10 -> 00; A is bit 1. */
    static const int8_t transitions[16]
        = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

    before &= 3U;
    after &= 3U;
    *invalid = (3U == (before ^ after));
    return transitions[(before * 4U) + after];
}

bool
odometry_step (odometry_t * odom, const motion_config_t * config,
               const encoder_sample_t * sample, float64_t dt)
{
    float64_t change[2];
    unsigned  wheel;

    if ((!isfinite(dt)) || (dt <= 0.0) || (dt > config->max_dt))
    {
        return false;
    }

    for (wheel = 0; wheel < 2U; ++wheel)
    {
        float64_t quantum = (3.141592653589793 * config->diameter[wheel])
                            / config->counts_per_rev[wheel];

        change[wheel] = (float64_t)encoder_delta(sample->count[wheel],
                                                 odom->previous[wheel])
                        * quantum;

        if ((!isfinite(change[wheel]))
            || (fabs(change[wheel])
                > ((config->max_speed * dt * 3.0) + (2.0 * quantum))))
        {
            return false;
        }
    }

    for (wheel = 0; wheel < 2U; ++wheel)
    {
        odom->previous[wheel] = sample->count[wheel];
        odom->travel[wheel] += change[wheel];
        odom->speed[wheel] = change[wheel] / dt;
    }

    odom->centre += (change[0] + change[1]) / 2.0;
    odom->path += fabs((change[0] + change[1]) / 2.0);
    odom->heading += (change[1] - change[0]) / config->track;
    return true;
}

/*** end of file ***/
