#include "../../include/float_types.h"
#include "angle_test.h"
#include <math.h>
#include <string.h>
static const float64_t g_pi_value = 3.141592653589793;

static void finish(angle_control_t * p_control, angle_state_t state);

void
angle_control_init (angle_control_t *       p_control,
                    const motion_config_t * p_config)
{
    unsigned wheel;

    memset(p_control, 0, sizeof(*p_control));
    p_control->config = *p_config;
    p_control->ready  = (isfinite(p_config->track)) && (p_config->track > 0.0);

    for (wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        p_control->ready = (p_control->ready)
                           && (isfinite(p_config->diameter[wheel]))
                           && (p_config->diameter[wheel] > 0.0)
                           && (isfinite(p_config->counts_per_rev[wheel]))
                           && (p_config->counts_per_rev[wheel] > 0.0)
                           && (p_config->quadrature[wheel]);

        if (p_control->ready)
        {
            p_control->goal[wheel]
                = (p_config->counts_per_rev[wheel] * p_config->track)
                  / (4.0 * p_config->diameter[wheel]);
        }
    }
}

bool
angle_control_start (angle_control_t * p_control, int direction,
                     encoder_sample_t sample, float64_t now)
{
    bool b_result = false;

    if ((!p_control->ready) || (!sample.reliable) || (!isfinite(now))
        || ((1 != direction) && (-1 != direction))
        || (ANGLE_TURNING == p_control->state)
        || (ANGLE_BRAKING == p_control->state)
        || ((ANGLE_IDLE != p_control->state)
            && (now < (p_control->quiet_since + 0.3))))
    {
        b_result = false;
    }
    else
    {
        p_control->baseline       = sample;
        p_control->previous       = sample;
        p_control->delta[0]       = 0U;
        p_control->delta[1]       = 0U;
        p_control->angle          = 0.0;
        p_control->direction      = direction;
        p_control->deadline       = now + 8.0;
        p_control->last_change[0] = now;
        p_control->last_change[1] = now;
        p_control->quiet_since    = now;
        p_control->state          = ANGLE_TURNING;
        p_control->output[0]      = -0.25 * direction;
        p_control->output[1]      = 0.25 * direction;
        b_result                  = true;
    }

    return b_result;
}

void
angle_control_step (angle_control_t * p_control, encoder_sample_t sample,
                    float64_t now)
{
    unsigned wheel;

    float64_t travel[2];
    float64_t progress[2];
    bool      b_changed = false;

    if ((ANGLE_TURNING != p_control->state)
        && (ANGLE_BRAKING != p_control->state))
    {
        return;
    }

    if ((!sample.reliable) || (!isfinite(now)))
    {
        finish(p_control, ANGLE_FEEDBACK);
        return;
    }

    for (wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        int expected
            = (0U == wheel) ? -p_control->direction : p_control->direction;
        int64_t delta = encoder_delta(sample.count[wheel],
                                      p_control->baseline.count[wheel]);

        p_control->delta[wheel]
            = sample.count[wheel] - p_control->baseline.count[wheel];
        travel[wheel]   = ((float64_t)delta * g_pi_value
                           * p_control->config.diameter[wheel])
                          / p_control->config.counts_per_rev[wheel];
        progress[wheel] = (float64_t)delta * expected;

        if (sample.count[wheel] != p_control->previous.count[wheel])
        {
            p_control->last_change[wheel] = now;
            b_changed                     = true;
        }

        if ((sample.invalid[wheel] != p_control->baseline.invalid[wheel])
            || (progress[wheel] < -4.0))
        {
            finish(p_control, ANGLE_FEEDBACK);
            p_control->quiet_since = now;
            return;
        }
    }

    p_control->angle    = (travel[1] - travel[0]) / p_control->config.track;
    p_control->previous = sample;

    if (now >= p_control->deadline)
    {
        finish(p_control, ANGLE_TIMEOUT);
        p_control->quiet_since = now;
        return;
    }

    if (ANGLE_BRAKING == p_control->state)
    {
        if (b_changed)
        {
            p_control->quiet_since = now;
        }

        if ((now - p_control->quiet_since) >= 0.3)
        {
            finish(p_control,
                   (fabs(p_control->angle
                         - ((p_control->direction * g_pi_value) / 2.0))
                    <= (g_pi_value / 60.0))
                       ? ANGLE_DONE
                       : ANGLE_OVERSHOOT);
        }
        else if (now >= p_control->brake_deadline)
        {
            finish(p_control, ANGLE_TIMEOUT);
        }
        else
        {
            /* Nothing to do in any other case. */
        }

        return;
    }

    for (wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        float64_t remaining
            = fmax(0.0, 1.0 - (progress[wheel] / p_control->goal[wheel]));
        float64_t duty
            = fmin(0.25, 0.12 + ((0.13 * remaining) / (25.0 / 90.0)));

        if (progress[wheel] >= p_control->goal[wheel])
        {
            p_control->output[wheel] = 0.0;
        }
        else if (0.0 != p_control->output[wheel])
        {
            p_control->output[wheel] = duty
                                       * ((0U == wheel) ? -p_control->direction
                                                        : p_control->direction);
        }
        else
        {
            /* Nothing to do in any other case. */
        }

        /* A wheel already braked stays braked; never restart after overshoot.
         */
        if ((0.0 != p_control->output[wheel])
            && ((now - p_control->last_change[wheel]) >= 1.0))
        {
            finish(p_control, ANGLE_FEEDBACK);
            p_control->quiet_since = now;
            return;
        }
    }

    if ((0.0 == p_control->output[0]) && (0.0 == p_control->output[1]))
    {
        p_control->state          = ANGLE_BRAKING;
        p_control->quiet_since    = now;
        p_control->brake_deadline = now + 1.0;
    }
}

void
angle_control_stop (angle_control_t * p_control)
{
    finish(p_control, ANGLE_STOPPED);
}

static void
finish (angle_control_t * p_control, angle_state_t state)
{
    p_control->state     = state;
    p_control->output[0] = 0.0;
    p_control->output[1] = 0.0;
}

/*** end of file ***/
