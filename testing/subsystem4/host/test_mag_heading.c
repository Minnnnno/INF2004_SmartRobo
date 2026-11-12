/* Real mag_heading.c with a synthetic magnetometer: a known hard-iron offset,
 * unequal axis gains and noise; the robot turning left by phi rotates the
 * field in the sensor frame by -phi (the world field stays put).
 */
#include "../../../app_program/subsystem4/mag_heading.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI_F (3.14159265F)
static unsigned g_checks;
static uint32_t g_seed = 12345U;

static void check(int yes, char const *why)
{
    g_checks++;
    if (!yes) { fprintf(stderr, "FAIL: %s\n", why); exit(1); }
}

/* Deterministic noise in [-amp, +amp] counts. */
static float noise(float amp)
{
    g_seed = (g_seed * 1103515245U) + 12345U;
    return (((float)((g_seed >> 16U) & 0x7FFFU) / 32767.0F) * 2.0F - 1.0F) * amp;
}

/* Raw sensor X/Y with the robot at encoder heading phi (deg, + = left). */
static void sensor(float phi_deg, float radius_scale, int16_t *p_x, int16_t *p_y)
{
    float theta = (30.0F - phi_deg) * PI_F / 180.0F; /* 30 deg arbitrary start */
    float r = 400.0F * radius_scale;
    *p_x = (int16_t)lroundf(-123.0F + r * cosf(theta) + noise(4.0F));
    *p_y = (int16_t)lroundf(48.0F + (r / 1.06F) * sinf(theta) + noise(4.0F));
}

/* Left 360 then right 360, one sample per degree, like the calibration build. */
static uint8_t spin(mag_cal_result_t *p_result, float stop_deg, int distorted)
{
    mag_cal_acc_t acc;
    mag_cal_begin(&acc);
    for (int d = 0; d <= (int)stop_deg; d++)
    {
        int16_t x, y;
        float scale = (distorted && (d >= 100) && (d < 140)) ? 1.3F : 1.0F;
        sensor((float)d, scale, &x, &y);
        mag_cal_add(&acc, x, y, (int32_t)d * 10);
    }
    for (int d = (int)stop_deg; d >= 0; d--)
    {
        int16_t x, y;
        sensor((float)d, 1.0F, &x, &y);
        mag_cal_add(&acc, x, y, (int32_t)d * 10);
    }
    return mag_cal_finish(&acc, p_result);
}

static int32_t heading_change(mag_cal_t const *p_cal, float phi)
{
    int16_t x, y, h0 = 0, h1 = 0;
    sensor(0.0F, 1.0F, &x, &y);
    check(mag_heading_ddeg(p_cal, x, y, &h0), "reference heading valid");
    sensor(phi, 1.0F, &x, &y);
    check(mag_heading_ddeg(p_cal, x, y, &h1), "turned heading valid");
    return mag_unwrap(h0, h1) - h0;
}

int main(void)
{
    mag_cal_result_t result;
    mag_cal_t cal;

    /* Clean full spin: constants recovered. */
    check(MAG_CAL_PASS == spin(&result, 360.0F, 0), "full spin passes");
    cal = result.cal;
    check(cal.calibrated && (36U == result.bins_covered), "calibrated, 36 bins");
    check(abs(cal.offset_x + 123) <= 4 && abs(cal.offset_y - 48) <= 4, "hard-iron offset recovered");
    check(abs((int)cal.scale_y_q12 - 4342) <= 44, "Y scale 1.06 recovered within 1%");
    check(abs((int)cal.field - 400) <= 8, "reference field about 400 counts");
    check(-1 == cal.sign, "sign flipped so + = left");
    check(result.fit_permille <= 20U, "clean circle fit");

    /* Heading change follows the turn within 2 deg, both directions. */
    check(labs(heading_change(&cal, 90.0F) - 900) <= 20, "left 90 deg measured");
    check(labs(heading_change(&cal, -90.0F) + 900) <= 20, "right 90 deg measured");
    check(labs(heading_change(&cal, 45.0F) - 450) <= 20, "left 45 deg measured");
    check(labs(heading_change(&cal, 170.0F) - 1700) <= 20, "left 170 deg measured");

    /* Field validity: steel nearby changes the strength. */
    {
        int16_t x, y, h = 0;
        sensor(10.0F, 1.3F, &x, &y);
        check(!mag_heading_ddeg(&cal, x, y, &h), "field +30% is invalid");
        sensor(10.0F, 0.75F, &x, &y);
        check(!mag_heading_ddeg(&cal, x, y, &h), "field -25% is invalid");
        sensor(10.0F, 0.88F, &x, &y);
        check(mag_heading_ddeg(&cal, x, y, &h) && (h >= 0) && (h < 3600), "field -12% still valid, 0..3599");
    }

    /* Unwrap across 0/360. */
    check(3610 == mag_unwrap(3590, 10), "unwrap forward across 360");
    check(-10 == mag_unwrap(10, 3590), "unwrap backward across 0");
    check(-3590 == mag_unwrap(-3605, 10), "unwrap from a negative history");
    check(7210 == mag_unwrap(7190, 10), "unwrap second lap");

    /* Failures. */
    check(MAG_CAL_FAIL_COVERAGE == spin(&result, 180.0F, 0), "half spin fails coverage");
    check(MAG_CAL_FAIL_FIT == spin(&result, 360.0F, 1), "distorted region fails the fit");
    {
        mag_cal_acc_t acc;
        mag_cal_begin(&acc);
        for (int d = 0; d < 720; d++) { mag_cal_add(&acc, 5, 7, (int32_t)d * 5); }
        check(MAG_CAL_FAIL_RANGE == mag_cal_finish(&acc, &result), "flat axes fail range");
        mag_cal_begin(&acc);
        mag_cal_add(&acc, 5, 7, 0);
        check(MAG_CAL_FAIL_SAMPLES == mag_cal_finish(&acc, &result), "too few samples");
        check(MAG_CAL_FAIL_SAMPLES == mag_cal_finish(NULL, &result), "NULL accumulator");
    }

    /* Uncalibrated: no heading; field is the raw magnitude. */
    {
        mag_cal_t none = {0};
        int16_t h = 7;
        check(!mag_heading_ddeg(&none, 300, 400, &h) && (7 == h), "uncalibrated: no heading");
        check(500U == mag_field(&none, 300, 400), "raw field magnitude");
        check(!mag_heading_ddeg(NULL, 1, 1, &h), "NULL cal");
    }
    check('P' == mag_cal_status_name(MAG_CAL_PASS)[0], "status names");

    printf("PASS: %u compass checks (calibration, heading, validity, unwrap, failures)\n", g_checks);
    return 0;
}
