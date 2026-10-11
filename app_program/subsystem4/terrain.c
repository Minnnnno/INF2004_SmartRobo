/* Gravity-based tilt and a simple motion heuristic, without sensor fusion. */
#include "terrain.h"
#include <math.h>
#include <stddef.h>

#define TERRAIN_FILTER_ALPHA (0.2F)
#define TERRAIN_RAD_TO_DEG   (57.2957795F)
#define TERRAIN_MIN_G_MG     (700.0F)
#define TERRAIN_MAX_G_MG     (1300.0F)

static float terrain_length(float axis_x, float axis_y, float axis_z);
static void terrain_set_reference(terrain_calibration_t const *p_calibration,
                                  terrain_filter_t *p_filter);
static void terrain_report_axes(terrain_filter_t *p_filter,
                                 terrain_result_t *p_result);
static void terrain_confirm_axis(terrain_axis_state_t *p_axis,
                                  terrain_condition_t candidate);
static void terrain_classify(terrain_filter_t *p_filter,
                             terrain_result_t const *p_result);

bool terrain_use_fixed_reference(terrain_filter_t *p_filter)
{
    terrain_calibration_t reference = {0};
    float magnitude = terrain_length(TERRAIN_REFERENCE_X_MG,
                                     TERRAIN_REFERENCE_Y_MG,
                                     TERRAIN_REFERENCE_Z_MG);

    if ((p_filter == NULL) ||
        !((magnitude >= 850.0F) && (magnitude <= 1150.0F)))
    {
        return false;
    }
    reference.sum_x = TERRAIN_REFERENCE_X_MG;
    reference.sum_y = TERRAIN_REFERENCE_Y_MG;
    reference.sum_z = TERRAIN_REFERENCE_Z_MG;
    terrain_set_reference(&reference, p_filter);
    return true;
}

void terrain_reset(terrain_filter_t *p_filter)
{
    if (p_filter != NULL)
    {
        p_filter->pitch_axis = (terrain_axis_state_t){0};
        p_filter->roll_axis = (terrain_axis_state_t){0};
        p_filter->gravity_x = 0.0F;
        p_filter->gravity_y = 0.0F;
        p_filter->gravity_z = 0.0F;
        p_filter->confirmation_count = 0U;
        p_filter->candidate = TERRAIN_UNKNOWN;
        p_filter->condition = TERRAIN_UNKNOWN;
        p_filter->b_initialized = false;
    }
}

bool terrain_calibration_add(terrain_calibration_t *p_calibration,
                             terrain_filter_t *p_filter,
                             int16_t axis_x, int16_t axis_y, int16_t axis_z)
{
    float accel_x = (float)axis_x;
    float accel_y = (float)axis_y;
    float accel_z = (float)axis_z;
    float magnitude = terrain_length(accel_x, accel_y, accel_z);

    if ((p_calibration == NULL) || (p_filter == NULL))
    {
        return false;
    }
    if (p_filter->b_calibrated)
    {
        return true;
    }
    /* Near 1 g and stationary; neither free fall nor a shock is a baseline. */
    if ((magnitude < 850.0F) || (magnitude > 1150.0F))
    {
        *p_calibration = (terrain_calibration_t){0};
        return false;
    }
    if ((p_calibration->count > 0U) &&
        (terrain_length(accel_x - p_calibration->first_x,
                        accel_y - p_calibration->first_y,
                        accel_z - p_calibration->first_z) >
         TERRAIN_CALIBRATION_MOTION))
    {
        *p_calibration = (terrain_calibration_t){0};
        return false;
    }
    if (p_calibration->count == 0U)
    {
        p_calibration->first_x = accel_x;
        p_calibration->first_y = accel_y;
        p_calibration->first_z = accel_z;
    }
    p_calibration->sum_x += accel_x;
    p_calibration->sum_y += accel_y;
    p_calibration->sum_z += accel_z;
    p_calibration->count++;
    if (p_calibration->count < TERRAIN_CALIBRATION_SAMPLES)
    {
        return false;
    }

    terrain_set_reference(p_calibration, p_filter);
    return true;
}

bool terrain_update(terrain_filter_t *p_filter, int16_t axis_x, int16_t axis_y,
                    int16_t axis_z, terrain_result_t *p_result)
{
    float accel_x = (float)axis_x;
    float accel_y = (float)axis_y;
    float accel_z = (float)axis_z;
    float magnitude = terrain_length(accel_x, accel_y, accel_z);
    float forward_accel = 0.0F;
    float sideways_accel = 0.0F;
    terrain_result_t result = {0};

    if ((p_filter == NULL) || (p_result == NULL))
    {
        return false;
    }

    if (!p_filter->b_calibrated)
    {
        *p_result = result;
        return true;
    }

    /* Rotate into the captured level frame, retaining gravity magnitude. */
    accel_x = ((float)axis_x * p_filter->reference_x[0]) +
         ((float)axis_y * p_filter->reference_x[1]) +
         ((float)axis_z * p_filter->reference_x[2]);
    accel_y = ((float)axis_x * p_filter->reference_y[0]) +
         ((float)axis_y * p_filter->reference_y[1]) +
         ((float)axis_z * p_filter->reference_y[2]);
    accel_z = ((float)axis_x * p_filter->reference_z[0]) +
         ((float)axis_y * p_filter->reference_z[1]) +
         ((float)axis_z * p_filter->reference_z[2]);

    /* A small/large total acceleration cannot give reliable gravity tilt. */
    if ((magnitude < TERRAIN_MIN_G_MG) || (magnitude > TERRAIN_MAX_G_MG))
    {
        terrain_reset(p_filter);
        result.condition = TERRAIN_UNKNOWN;
        *p_result = result;
        return true;
    }

    if (!p_filter->b_initialized)
    {
        p_filter->gravity_x = accel_x;
        p_filter->gravity_y = accel_y;
        p_filter->gravity_z = accel_z;
        p_filter->b_initialized = true;
    }

    /* Residual before smoothing: bumps, turns and braking can all raise it. */
    result.motion_mg = terrain_length(accel_x - p_filter->gravity_x,
                                     accel_y - p_filter->gravity_y,
                                     accel_z - p_filter->gravity_z);
    p_filter->gravity_x += TERRAIN_FILTER_ALPHA *
                          (accel_x - p_filter->gravity_x);
    p_filter->gravity_y += TERRAIN_FILTER_ALPHA *
                          (accel_y - p_filter->gravity_y);
    p_filter->gravity_z += TERRAIN_FILTER_ALPHA *
                          (accel_z - p_filter->gravity_z);

#if TERRAIN_FORWARD_AXIS_Y
    forward_accel = p_filter->gravity_y;
    sideways_accel = p_filter->gravity_x;
#else
    forward_accel = p_filter->gravity_x;
    sideways_accel = p_filter->gravity_y;
#endif
    result.pitch_deg = atan2f(TERRAIN_PITCH_SIGN * forward_accel,
                             terrain_length(0.0F, sideways_accel,
                                            p_filter->gravity_z)) *
                       TERRAIN_RAD_TO_DEG;
    result.roll_deg = atan2f(TERRAIN_ROLL_SIGN * sideways_accel,
                            p_filter->gravity_z) * TERRAIN_RAD_TO_DEG;
    result.tilt_deg = atan2f(terrain_length(p_filter->gravity_x,
                                          p_filter->gravity_y, 0.0F),
                            p_filter->gravity_z) * TERRAIN_RAD_TO_DEG;
    result.b_tilt_valid = true;

    terrain_report_axes(p_filter, &result);
    terrain_classify(p_filter, &result);
    result.condition = p_filter->condition;
    *p_result = result;
    return true;
}

char const *terrain_condition_name(terrain_condition_t condition)
{
    switch (condition)
    {
        case TERRAIN_LEVEL:
            return "LEVEL";
        case TERRAIN_SLOPE:
            return "SLOPE";
        case TERRAIN_UPHILL:
            return "UPHILL";
        case TERRAIN_DOWNHILL:
            return "DOWNHILL";
        case TERRAIN_SIDE_SLOPE:
            return "SIDE_SLOPE";
        case TERRAIN_SIDE_LEFT:
            return "SIDE_LEFT";
        case TERRAIN_SIDE_RIGHT:
            return "SIDE_RIGHT";
        case TERRAIN_ROUGH:
            return "ROUGH";
        case TERRAIN_EXCESS_TILT:
            return "EXCESS_TILT";
        default:
            return "UNKNOWN";
    }
}

char const *terrain_action_name(terrain_condition_t condition)
{
    switch (condition)
    {
        case TERRAIN_LEVEL:
            return "NORMAL";
        case TERRAIN_SLOPE:
        case TERRAIN_UPHILL:
        case TERRAIN_DOWNHILL:
        case TERRAIN_SIDE_SLOPE:
        case TERRAIN_SIDE_LEFT:
        case TERRAIN_SIDE_RIGHT:
        case TERRAIN_ROUGH:
            return "SLOW";
        default:
            return "STOP";
    }
}

static float terrain_length(float axis_x, float axis_y, float axis_z)
{
    return sqrtf((axis_x * axis_x) + (axis_y * axis_y) + (axis_z * axis_z));
}

/* Build the level frame from the stationary gravity average. */
static void terrain_set_reference(terrain_calibration_t const *p_calibration,
                                  terrain_filter_t *p_filter)
{
    float length = 0.0F;
    float *p_x = NULL;
    float *p_y = NULL;
    float *p_z = NULL;
    p_x = p_filter->reference_x;
    p_y = p_filter->reference_y;
    p_z = p_filter->reference_z;
    length = terrain_length(p_calibration->sum_x, p_calibration->sum_y,
                            p_calibration->sum_z);
    p_z[0] = p_calibration->sum_x / length;
    p_z[1] = p_calibration->sum_y / length;
    p_z[2] = p_calibration->sum_z / length;
    /* Project sensor X onto the calibrated horizontal plane. */
    p_x[0] = 1.0F - (p_z[0] * p_z[0]);
    p_x[1] = -p_z[0] * p_z[1];
    p_x[2] = -p_z[0] * p_z[2];
    length = terrain_length(p_x[0], p_x[1], p_x[2]);
    if (length < 0.1F)
    {
        /* Sensor X is vertical: use projected Y as the horizontal axis. */
        p_x[0] = -p_z[1] * p_z[0];
        p_x[1] = 1.0F - (p_z[1] * p_z[1]);
        p_x[2] = -p_z[1] * p_z[2];
        length = terrain_length(p_x[0], p_x[1], p_x[2]);
    }
    for (uint32_t index = 0U; index < 3U; index++)
    {
        p_x[index] /= length;
    }
    /* Y = Z cross X: complete a right-handed reference frame. */
    p_y[0] = (p_z[1] * p_x[2]) - (p_z[2] * p_x[1]);
    p_y[1] = (p_z[2] * p_x[0]) - (p_z[0] * p_x[2]);
    p_y[2] = (p_z[0] * p_x[1]) - (p_z[1] * p_x[0]);
    terrain_reset(p_filter);
    p_filter->b_calibrated = true;
}

/* Confirm terrain changes; excessive tilt remains an immediate decision. */
static void terrain_classify(terrain_filter_t *p_filter,
                             terrain_result_t const *p_result)
{
    terrain_condition_t candidate = TERRAIN_LEVEL;
    uint32_t required_count = TERRAIN_CONFIRM_SAMPLES;
    /* Total tilt catches combined pitch/roll and reference inversion. */
    if (p_result->tilt_deg >= TERRAIN_STOP_DEG)
    {
        candidate = TERRAIN_EXCESS_TILT;
    }
    else if (p_result->motion_mg >= TERRAIN_ROUGH_MG)
    {
        candidate = TERRAIN_ROUGH;
    }
    else if (p_result->tilt_deg >= TERRAIN_SLOPE_DEG)
    {
        /* Label the dominant tilt axis; safety still uses total tilt. */
        if (fabsf(p_result->roll_deg) > fabsf(p_result->pitch_deg))
        {
            if ((p_result->roll_deg * TERRAIN_LEFT_ROLL_SIGN) > 0.0F)
            {
                candidate = TERRAIN_SIDE_LEFT;
            }
            else
            {
                candidate = TERRAIN_SIDE_RIGHT;
            }
        }
        else if (p_result->pitch_deg > 0.0F)
        {
            candidate = TERRAIN_UPHILL;
        }
        else
        {
            candidate = TERRAIN_DOWNHILL;
        }
    }

    if (candidate != p_filter->candidate)
    {
        p_filter->candidate = candidate;
        p_filter->confirmation_count = 0U;
    }
    if (candidate == TERRAIN_LEVEL)
    {
        required_count = TERRAIN_RECOVER_SAMPLES;
    }
    if (p_filter->confirmation_count < required_count)
    {
        p_filter->confirmation_count++;
    }
    /* Excess filtered tilt stops immediately; recovery must be confirmed. */
    if ((candidate == TERRAIN_EXCESS_TILT) ||
        (p_filter->confirmation_count >= required_count))
    {
        p_filter->condition = candidate;
    }
}

/* A direction on either axis is independent of the other axis and motion. */
static void terrain_report_axes(terrain_filter_t *p_filter,
                                 terrain_result_t *p_result)
{
    terrain_condition_t pitch_candidate = TERRAIN_LEVEL;
    terrain_condition_t roll_candidate = TERRAIN_LEVEL;

    if (fabsf(p_result->pitch_deg) >= TERRAIN_SLOPE_DEG)
    {
        pitch_candidate = (p_result->pitch_deg > 0.0F) ?
                          TERRAIN_UPHILL : TERRAIN_DOWNHILL;
    }
    if (fabsf(p_result->roll_deg) >= TERRAIN_SLOPE_DEG)
    {
        roll_candidate = ((p_result->roll_deg * TERRAIN_LEFT_ROLL_SIGN) >
                          0.0F) ? TERRAIN_SIDE_LEFT : TERRAIN_SIDE_RIGHT;
    }
    terrain_confirm_axis(&p_filter->pitch_axis, pitch_candidate);
    terrain_confirm_axis(&p_filter->roll_axis, roll_candidate);
    p_result->pitch_condition = p_filter->pitch_axis.condition;
    p_result->roll_condition = p_filter->roll_axis.condition;
}

/* Three samples confirm slope direction; five samples confirm level. */
static void terrain_confirm_axis(terrain_axis_state_t *p_axis,
                                  terrain_condition_t candidate)
{
    uint32_t required_count = (candidate == TERRAIN_LEVEL) ?
                             TERRAIN_RECOVER_SAMPLES :
                             TERRAIN_CONFIRM_SAMPLES;

    if (candidate != p_axis->candidate)
    {
        p_axis->candidate = candidate;
        p_axis->confirmation_count = 0U;
    }
    if (p_axis->confirmation_count < required_count)
    {
        p_axis->confirmation_count++;
    }
    if (p_axis->confirmation_count >= required_count)
    {
        p_axis->condition = candidate;
    }
}
