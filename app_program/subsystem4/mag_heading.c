/* Buddy 4 compass calibration and heading; see mag_heading.h. */
#include "../../include/float_types.h"
#include "mag_heading.h"
#include <math.h>
#include <stddef.h>

#define MAG_RAD_TO_DDEG (572.957795F) /* 1800 / pi */

void
mag_cal_begin (mag_cal_acc_t * p_acc)
{
    if (NULL != p_acc)
    {
        *p_acc        = (mag_cal_acc_t){0};
        p_acc->min[0] = INT16_MAX;
        p_acc->min[1] = INT16_MAX;
        p_acc->max[0] = INT16_MIN;
        p_acc->max[1] = INT16_MIN;
    }
}

void
mag_cal_add (mag_cal_acc_t * p_acc, int16_t axis_x, int16_t axis_y,
             int32_t rel_heading_ddeg)
{
    int32_t  wrapped = 0;
    uint32_t bin     = 0U;

    if (NULL == p_acc)
    {
        return;
    }

    if (axis_x < p_acc->min[0])
    {
        p_acc->min[0] = axis_x;
    }

    if (axis_x > p_acc->max[0])
    {
        p_acc->max[0] = axis_x;
    }

    if (axis_y < p_acc->min[1])
    {
        p_acc->min[1] = axis_y;
    }

    if (axis_y > p_acc->max[1])
    {
        p_acc->max[1] = axis_y;
    }

    if (p_acc->samples < UINT16_MAX)
    {
        p_acc->samples++;
    }

    wrapped = rel_heading_ddeg % 3600;

    if (wrapped < 0)
    {
        wrapped += 3600;
    }

    bin = (uint32_t)wrapped / (uint32_t)MAG_BIN_DDEG;

    if (0U == (p_acc->bins & (1ULL << bin)))
    {
        p_acc->bins |= (1ULL << bin);
        p_acc->bin_x[bin] = axis_x;
        p_acc->bin_y[bin] = axis_y;
    }
}

static uint8_t
mag_popcount (uint64_t bits)
{
    uint8_t count = 0U;

    while (0ULL != bits)
    {
        bits &= bits - 1ULL;
        count++;
    }

    return count;
}

/* Corrected X/Y for a raw sample. */
static void
mag_correct (mag_cal_t const * p_cal, int16_t axis_x, int16_t axis_y,
             float32_t * p_cx, float32_t * p_cy)
{
    *p_cx = (float32_t)((int32_t)axis_x - (int32_t)p_cal->offset_x);
    *p_cy = (float32_t)((int32_t)axis_y - (int32_t)p_cal->offset_y)
            * ((float32_t)p_cal->scale_y_q12 / (float32_t)MAG_SCALE_ONE_Q12);
}

static float32_t
mag_wrap_deg (float32_t degrees)
{
    while (degrees > 180.0F)
    {
        degrees -= 360.0F;
    }

    while (degrees <= -180.0F)
    {
        degrees += 360.0F;
    }

    return degrees;
}

uint8_t
mag_cal_finish (mag_cal_acc_t const * p_acc, mag_cal_result_t * p_result)
{
    int32_t   half_x   = 0;
    int32_t   half_y   = 0;
    float32_t sum      = 0.0F;
    float32_t sum_sq   = 0.0F;
    float32_t mean     = 0.0F;
    float32_t turn     = 0.0F;
    float32_t previous = 0.0F;
    uint32_t  scale    = 0U;

    if ((NULL == p_acc) || (NULL == p_result))
    {
        return MAG_CAL_FAIL_SAMPLES;
    }

    *p_result              = (mag_cal_result_t){0};
    p_result->bins_covered = mag_popcount(p_acc->bins);

    if (p_acc->samples < MAG_BINS)
    {
        return MAG_CAL_FAIL_SAMPLES;
    }

    half_x = ((int32_t)p_acc->max[0] - (int32_t)p_acc->min[0]) / 2;
    half_y = ((int32_t)p_acc->max[1] - (int32_t)p_acc->min[1]) / 2;
    p_result->cal.offset_x
        = (int16_t)(((int32_t)p_acc->max[0] + (int32_t)p_acc->min[0]) / 2);
    p_result->cal.offset_y
        = (int16_t)(((int32_t)p_acc->max[1] + (int32_t)p_acc->min[1]) / 2);

    if (p_result->bins_covered < MAG_BINS)
    {
        return MAG_CAL_FAIL_COVERAGE;
    }

    if ((half_x < MAG_MIN_HALF_RANGE) || (half_y < MAG_MIN_HALF_RANGE))
    {
        return MAG_CAL_FAIL_RANGE;
    }

    scale = (uint32_t)(((half_x * (int32_t)MAG_SCALE_ONE_Q12) + (half_y / 2))
                       / half_y);

    if ((scale < (MAG_SCALE_ONE_Q12 / 2U))
        || (scale > (MAG_SCALE_ONE_Q12 * 2U)))
    {
        return MAG_CAL_FAIL_RANGE; /* Axes far too unequal: not a clean spin. */
    }

    p_result->cal.scale_y_q12 = (uint16_t)scale;
    p_result->cal.sign        = 1;

    /* Circle fit over one sample per bin, and the rotation sense. */
    for (uint32_t bin = 0U; bin < MAG_BINS; bin++)
    {
        float32_t centre_x = 0.0F;
        float32_t centre_y = 0.0F;
        float32_t angle    = 0.0F;

        mag_correct(&p_result->cal, p_acc->bin_x[bin], p_acc->bin_y[bin],
                    &centre_x, &centre_y);
        sum += sqrtf((centre_x * centre_x) + (centre_y * centre_y));
        angle = atan2f(centre_y, centre_x) * 57.2957795F;

        if (bin > 0U)
        {
            turn += mag_wrap_deg(angle - previous);
        }

        previous = angle;
    }

    mean = sum / (float32_t)MAG_BINS;

    for (uint32_t bin = 0U; bin < MAG_BINS; bin++)
    {
        float32_t centre_x = 0.0F;
        float32_t centre_y = 0.0F;
        float32_t error    = 0.0F;

        mag_correct(&p_result->cal, p_acc->bin_x[bin], p_acc->bin_y[bin],
                    &centre_x, &centre_y);
        error = (sqrtf((centre_x * centre_x) + (centre_y * centre_y)) / mean)
                - 1.0F;
        sum_sq += error * error;
    }

    p_result->fit_permille
        = (uint16_t)((sqrtf(sum_sq / (float32_t)MAG_BINS) * 1000.0F) + 0.5F);
    p_result->cal.field = (uint16_t)(mean + 0.5F);
    /* Bins follow increasing encoder heading (left). If the raw angle fell,
     * the sensor's sense is opposite: flip so + = left like the encoders. */
    p_result->cal.sign = (turn >= 0.0F) ? (int8_t)1 : (int8_t)-1;

    if (p_result->fit_permille > MAG_FIT_MAX_PERMILLE)
    {
        return MAG_CAL_FAIL_FIT;
    }

    p_result->cal.calibrated = true;
    return MAG_CAL_PASS;
}

char const *
mag_cal_status_name (uint8_t status)
{
    static char const * const g_names[]
        = {"PASS", "FAIL too few samples",
           "FAIL coverage (not all directions seen)",
           "FAIL range (axis dead or saturated)",
           "FAIL fit (not a clean circle: steel nearby?)"};

    return (status <= MAG_CAL_FAIL_FIT) ? g_names[status] : "FAIL";
}

uint16_t
mag_field (mag_cal_t const * p_cal, int16_t axis_x, int16_t axis_y)
{
    float32_t centre_x  = (float32_t)axis_x;
    float32_t centre_y  = (float32_t)axis_y;
    float32_t magnitude = 0.0F;

    if ((NULL != p_cal) && (p_cal->calibrated))
    {
        mag_correct(p_cal, axis_x, axis_y, &centre_x, &centre_y);
    }

    magnitude = sqrtf((centre_x * centre_x) + (centre_y * centre_y));
    return (magnitude > 65535.0F) ? (uint16_t)65535U
                                  : (uint16_t)(magnitude + 0.5F);
}

bool
mag_heading_ddeg (mag_cal_t const * p_cal, int16_t axis_x, int16_t axis_y,
                  int16_t * p_heading_ddeg)
{
    float32_t centre_x  = 0.0F;
    float32_t centre_y  = 0.0F;
    float32_t heading   = 0.0F;
    int32_t   field     = 0;
    int32_t   reference = 0;
    int32_t   ddeg      = 0;

    if ((NULL == p_cal) || (NULL == p_heading_ddeg) || (!p_cal->calibrated)
        || (0U == p_cal->field))
    {
        return false;
    }

    field     = (int32_t)mag_field(p_cal, axis_x, axis_y);
    reference = (int32_t)p_cal->field;

    if ((((field - reference) * 100) > (reference * MAG_FIELD_TOL_PERCENT))
        || (((reference - field) * 100) > (reference * MAG_FIELD_TOL_PERCENT)))
    {
        return false;
    }

    mag_correct(p_cal, axis_x, axis_y, &centre_x, &centre_y);
    heading
        = (float32_t)p_cal->sign * atan2f(centre_y, centre_x) * MAG_RAD_TO_DDEG;
    ddeg = (int32_t)((heading >= 0.0F) ? (heading + 0.5F) : (heading - 0.5F));
    ddeg %= 3600;

    if (ddeg < 0)
    {
        ddeg += 3600;
    }

    *p_heading_ddeg = (int16_t)ddeg;
    return true;
}

int32_t
mag_unwrap (int32_t previous_ddeg, int16_t heading_ddeg)
{
    int32_t previous_wrapped = previous_ddeg % 3600;
    int32_t step             = 0;

    if (previous_wrapped < 0)
    {
        previous_wrapped += 3600;
    }

    step = (int32_t)heading_ddeg - previous_wrapped;

    if (step > 1800)
    {
        step -= 3600;
    }

    if (step <= -1800)
    {
        step += 3600;
    }

    return previous_ddeg + step;
}

/*** end of file ***/
