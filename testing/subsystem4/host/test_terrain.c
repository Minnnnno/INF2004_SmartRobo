/* Regression suite: compile with TERRAIN_FORWARD_AXIS_Y=0. */
#include "terrain.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned int g_checks;

static void require(bool b_condition, char const *p_message)
{
    if (!b_condition)
    {
        fprintf(stderr, "FAIL: %s\n", p_message);
        exit(EXIT_FAILURE);
    }
    g_checks++;
}

static void calibrate(terrain_filter_t *p_filter,
                      int16_t x, int16_t y, int16_t z)
{
    terrain_calibration_t calibration = {0};
    for (uint32_t index = 0U; index < TERRAIN_CALIBRATION_SAMPLES; index++)
    {
        (void)terrain_calibration_add(&calibration, p_filter, x, y, z);
    }
}

static void fixed_angle_sample(terrain_filter_t *p_filter,
                                float forward, float sideways, float upward,
                                terrain_result_t *p_result)
{
    int16_t sample[3] = {0};
    for (uint32_t index = 0U; index < 3U; index++)
    {
        sample[index] = (int16_t)lroundf(
            (forward * p_filter->reference_x[index]) +
            (sideways * p_filter->reference_y[index]) +
            (upward * p_filter->reference_z[index]));
    }
    (void)terrain_update(p_filter, sample[0], sample[1], sample[2], p_result);
}

static terrain_result_t settle(int16_t x, int16_t y, int16_t z)
{
    terrain_filter_t filter = {0};
    terrain_result_t result = {0};
    calibrate(&filter, 0, 0, 1000);
    for (unsigned int index = 0U; index < 40U; index++)
    {
        (void)terrain_update(&filter, x, y, z, &result);
    }
    return result;
}

int main(void)
{
    terrain_filter_t filter = {0};
    terrain_result_t result = {0};
    terrain_result_t saved = {0};
    calibrate(&filter, 0, 0, 1000);
    require(!terrain_update(NULL, 0, 0, 1000, &result), "null filter");
    require(!terrain_update(&filter, 0, 0, 1000, NULL), "null output");
    terrain_reset(NULL);
    require(memcmp(&result, &saved, sizeof(result)) == 0,
            "rejected update leaves output unchanged");
    (void)terrain_update(&filter, 0, 0, 1000, &result);
    require(result.condition == TERRAIN_UNKNOWN, "startup holds STOP");
    for (unsigned int index = 0U; index < 3U; index++)
    {
        (void)terrain_update(&filter, 0, 0, 1000, &result);
    }
    require(result.condition == TERRAIN_UNKNOWN, "level needs five samples");
    (void)terrain_update(&filter, 0, 0, 1000, &result);
    require(result.condition == TERRAIN_LEVEL, "flat settles LEVEL");
    require(result.b_tilt_valid && fabsf(result.tilt_deg) < 0.01F,
            "flat tilt is zero");
    result = settle(-174, 0, 985);
    require(result.condition == TERRAIN_DOWNHILL, "ten degree downhill");
    require(fabsf(result.pitch_deg + 10.0F) < 0.1F, "pitch degrees");
    result = settle(174, 0, 985);
    require(result.pitch_deg > 9.9F, "opposite pitch sign");
    result = settle(0, 174, 985);
    require(fabsf(result.roll_deg - 10.0F) < 0.1F, "roll degrees");
    result = settle(0, -174, 985);
    require(result.roll_deg < -9.9F, "opposite roll sign");
    result = settle(-643, 0, 766);
    require(result.condition == TERRAIN_EXCESS_TILT, "forty degrees STOP");
    result = settle(174, 174, 969);
    require(((result.condition == TERRAIN_SIDE_LEFT) ||
             (result.condition == TERRAIN_SIDE_RIGHT)), "combined small tilt");
    result = settle(410, 410, 815);
    require(result.condition == TERRAIN_EXCESS_TILT, "combined tilt STOP");
    result = settle(0, 0, -1000);
    require(result.condition == TERRAIN_EXCESS_TILT, "upside down STOP");
    require(fabsf(result.tilt_deg - 180.0F) < 0.01F, "upside down angle");
    result = settle(1000, 0, 0);
    require(result.condition == TERRAIN_EXCESS_TILT, "vertical STOP");
    terrain_reset(&filter);
    (void)terrain_update(&filter, 0, 0, 0, &result);
    require(!result.b_tilt_valid && result.condition == TERRAIN_UNKNOWN,
            "zero vector rejected");
    (void)terrain_update(&filter, 0, 0, 1500, &result);
    require(!result.b_tilt_valid, "high acceleration rejects tilt");
    (void)terrain_update(&filter, 0, 0, 600, &result);
    require(!result.b_tilt_valid, "low acceleration rejects tilt");
    (void)terrain_update(&filter, INT16_MIN, INT16_MAX, INT16_MIN, &result);
    require(!result.b_tilt_valid, "integer limits cannot overflow");
    terrain_reset(&filter);
    for (unsigned int index = 0U; index < 5U; index++)
    {
        (void)terrain_update(&filter, 0, 0, 1000, &result);
    }
    (void)terrain_update(&filter, 0, 0, 1270, &result);
    require(result.condition == TERRAIN_LEVEL, "single bump not ROUGH");
    for (unsigned int index = 0U; index < 6U; index++)
    {
        int16_t z = ((index % 2U) == 0U) ? 730 : 1270;
        (void)terrain_update(&filter, 0, 0, z, &result);
    }
    require(result.condition == TERRAIN_ROUGH, "repeated bumps ROUGH");
    for (unsigned int index = 0U; index < 4U; index++)
    {
        (void)terrain_update(&filter, 0, 0, 1000, &result);
    }
    require(result.condition == TERRAIN_ROUGH, "recovery waits five samples");
    (void)terrain_update(&filter, 0, 0, 1000, &result);
    require(result.condition == TERRAIN_LEVEL, "confirmed recovery LEVEL");
    terrain_reset(&filter);
    (void)terrain_update(&filter, -643, 0, 766, &result);
    require(result.condition == TERRAIN_EXCESS_TILT,
            "initial excessive tilt stops without confirmation");
    terrain_reset(&filter);
    require(!filter.b_initialized && filter.condition == TERRAIN_UNKNOWN,
            "missing-data reset clears history");
    require(strcmp(terrain_action_name(TERRAIN_LEVEL), "NORMAL") == 0,
            "level action");
    require(strcmp(terrain_action_name(TERRAIN_ROUGH), "SLOW") == 0,
            "rough action");
    require(strcmp(terrain_action_name(TERRAIN_SLOPE), "SLOW") == 0,
            "slope action");
    require(strcmp(terrain_action_name(TERRAIN_UNKNOWN), "STOP") == 0,
            "unknown action");
    require(strcmp(terrain_action_name(TERRAIN_EXCESS_TILT), "STOP") == 0,
            "tilt action");
    require(strcmp(terrain_condition_name(TERRAIN_ROUGH), "ROUGH") == 0,
            "condition label");
    filter = (terrain_filter_t){0};
    (void)terrain_update(&filter, 0, 0, 1000, &result);
    require(!result.b_tilt_valid && result.condition == TERRAIN_UNKNOWN,
            "uncalibrated data cannot produce an action");
    terrain_calibration_t calibration = {0};
    require(!terrain_calibration_add(NULL, &filter, 0, 0, 1000),
            "null calibration");
    require(!terrain_calibration_add(&calibration, NULL, 0, 0, 1000),
            "null calibration filter");
    for (uint32_t index = 0U; index < 39U; index++)
    {
        (void)terrain_calibration_add(&calibration, &filter, 0, 0, -1000);
    }
    require(!filter.b_calibrated, "39 samples are insufficient");
    require(terrain_calibration_add(&calibration, &filter, 0, 0, -1000),
            "40 samples capture inverted mounting");
    for (uint32_t index = 0U; index < 5U; index++)
    {
        (void)terrain_update(&filter, 0, 0, -1000, &result);
    }
    require(result.condition == TERRAIN_LEVEL && result.tilt_deg < 0.01F,
            "inverted sensor at reference reads level");
    terrain_reset(&filter);
    require(filter.b_calibrated, "missing data preserves baseline");
    (void)terrain_update(&filter, 0, 0, 0, &result);
    require(filter.b_calibrated, "invalid acceleration preserves baseline");
    filter = (terrain_filter_t){0};
    calibration = (terrain_calibration_t){0};
    (void)terrain_calibration_add(&calibration, &filter, 0, 0, 1000);
    (void)terrain_calibration_add(&calibration, &filter, 200, 0, 980);
    require(calibration.count == 0U, "movement restarts calibration");
    (void)terrain_calibration_add(&calibration, &filter, 0, 0, 1000);
    (void)terrain_calibration_add(&calibration, &filter, 0, 0, 0);
    require(calibration.count == 0U, "free fall restarts calibration");
    (void)terrain_calibration_add(&calibration, &filter, 0, 0, 1500);
    require(!filter.b_calibrated, "high acceleration cannot calibrate");
    calibrate(&filter, 500, 0, 866);
    for (uint32_t index = 0U; index < 5U; index++)
    {
        (void)terrain_update(&filter, 500, 0, 866, &result);
    }
    require(result.condition == TERRAIN_LEVEL && result.tilt_deg < 0.01F,
            "30 degree mounting offset removed");
    for (uint32_t index = 0U; index < 40U; index++)
    {
        (void)terrain_update(&filter, 643, 0, 766, &result);
    }
    require(result.condition == TERRAIN_UPHILL &&
            fabsf(result.tilt_deg - 10.0F) < 0.1F,
            "ten degree change relative to offset base");
    for (uint32_t index = 0U; index < 40U; index++)
    {
        (void)terrain_update(&filter, 940, 0, 342, &result);
    }
    require(result.condition == TERRAIN_EXCESS_TILT,
            "40 degree relative tilt still stops");
    filter = (terrain_filter_t){0};
    calibrate(&filter, 1000, 0, 0);
    for (uint32_t index = 0U; index < 5U; index++)
    {
        (void)terrain_update(&filter, 1000, 0, 0, &result);
    }
    require(result.condition == TERRAIN_LEVEL && result.tilt_deg < 0.01F,
            "vertical X uses alternate horizontal axis");
    filter = (terrain_filter_t){0};
    calibration = (terrain_calibration_t){0};
    for (uint32_t index = 0U; index < 40U; index++)
    {
        int16_t x = ((index % 2U) == 0U) ? 10 : -10;
        (void)terrain_calibration_add(&calibration, &filter, x, 0, 1000);
    }
    require(filter.b_calibrated && fabsf(filter.reference_z[0]) < 0.001F,
            "small noise is averaged out");
    float reference = filter.reference_z[2];
    require(terrain_calibration_add(&calibration, &filter, 1000, 0, 0) &&
            filter.reference_z[2] == reference,
            "running motion cannot silently change baseline");
    result = settle(174, 0, 985);
    require(result.condition == TERRAIN_UPHILL, "front raised UPHILL");
    result = settle(-174, 0, 985);
    require(result.condition == TERRAIN_DOWNHILL, "front lowered DOWNHILL");
    result = settle(0, 174, 985);
    require(((result.condition == TERRAIN_SIDE_LEFT) ||
             (result.condition == TERRAIN_SIDE_RIGHT)), "positive side slope");
    result = settle(0, -174, 985);
    require(((result.condition == TERRAIN_SIDE_LEFT) ||
             (result.condition == TERRAIN_SIDE_RIGHT)), "negative side slope");
    result = settle(174, 70, 982);
    require(result.condition == TERRAIN_UPHILL, "pitch dominant UPHILL");
    result = settle(-174, 70, 982);
    require(result.condition == TERRAIN_DOWNHILL, "pitch dominant DOWNHILL");
    result = settle(70, 174, 982);
    require(((result.condition == TERRAIN_SIDE_LEFT) ||
             (result.condition == TERRAIN_SIDE_RIGHT)), "roll dominates pitch");
    result = settle(70, 0, 998);
    require(result.condition == TERRAIN_LEVEL, "small tilt remains level");
    result = settle(643, 0, 766);
    require(result.condition == TERRAIN_EXCESS_TILT, "uphill stop priority");
    result = settle(-643, 0, 766);
    require(result.condition == TERRAIN_EXCESS_TILT, "downhill stop priority");
    require(strcmp(terrain_action_name(TERRAIN_UPHILL), "SLOW") == 0,
            "uphill action");
    require(strcmp(terrain_action_name(TERRAIN_DOWNHILL), "SLOW") == 0,
            "downhill action");
    require(strcmp(terrain_action_name(TERRAIN_SIDE_SLOPE), "SLOW") == 0,
            "side slope action");
    require(strcmp(terrain_condition_name(TERRAIN_UPHILL), "UPHILL") == 0,
            "uphill label");
    require(strcmp(terrain_condition_name(TERRAIN_DOWNHILL), "DOWNHILL") == 0,
            "downhill label");
    require(strcmp(terrain_condition_name(TERRAIN_SIDE_SLOPE),
                   "SIDE_SLOPE") == 0, "side slope label");
    filter = (terrain_filter_t){0};
    calibrate(&filter, 0, 0, 1000);
    (void)terrain_update(&filter, 174, 0, 985, &result);
    require(result.condition == TERRAIN_UNKNOWN,
            "direction needs confirmation");
    (void)terrain_update(&filter, 174, 0, 985, &result);
    require(result.condition == TERRAIN_UNKNOWN, "two samples insufficient");
    (void)terrain_update(&filter, 174, 0, 985, &result);
    require(result.condition == TERRAIN_UPHILL, "third sample confirms");
    terrain_reset(&filter);
    (void)terrain_update(&filter, -174, 0, 985, &result);
    (void)terrain_update(&filter, -174, 0, 985, &result);
    (void)terrain_update(&filter, -174, 0, 985, &result);
    require(result.condition == TERRAIN_DOWNHILL,
            "downhill confirmation after reset");
    require(!terrain_use_fixed_reference(NULL), "null fixed reference");
    filter = (terrain_filter_t){0};
    require(terrain_use_fixed_reference(&filter), "fixed reference loaded");
    for (uint32_t index = 0U; index < 5U; index++)
    {
        (void)terrain_update(&filter, (int16_t)TERRAIN_REFERENCE_X_MG,
                             (int16_t)TERRAIN_REFERENCE_Y_MG,
                             (int16_t)TERRAIN_REFERENCE_Z_MG, &result);
    }
    require(result.condition == TERRAIN_LEVEL && result.tilt_deg < 0.1F,
            "negative Z reference is level");
    require(fabsf(result.pitch_deg) < 0.1F &&
            fabsf(result.roll_deg) < 0.1F, "fixed pitch and roll zero");
    int16_t const resting[][3] =
    {
        {-33, 44, -1037}, {-39, 48, -1032}, {-37, 51, -1044},
        {-42, 42, -1035}, {-33, 46, -1031}, {-31, 44, -1041},
        {-36, 39, -1037}, {-35, 41, -1047}, {-37, 50, -1028},
        {-36, 41, -1042}, {-34, 28, -1039}, {-37, 26, -1039},
        {-36, 30, -1041}, {-37, 49, -1031}
    };
    for (uint32_t index = 0U;
         index < (sizeof(resting) / sizeof(resting[0])); index++)
    {
        (void)terrain_update(&filter, resting[index][0], resting[index][1],
                             resting[index][2], &result);
        require(result.condition == TERRAIN_LEVEL && result.tilt_deg < 2.0F,
                "screenshot resting samples remain LEVEL");
    }
    terrain_reset(&filter);
    require(filter.b_calibrated, "fixed reference survives a data gap");
    for (uint32_t index = 0U; index < 3U; index++)
    {
        /* Ten degrees of positive pitch relative to the saved base. */
        fixed_angle_sample(&filter, 181.0F, 0.0F, 1025.0F, &result);
    }
    require(result.condition == TERRAIN_UPHILL &&
            fabsf(result.pitch_deg - 10.0F) < 0.2F,
            "startup on slope uses saved reference, not current data");
    terrain_reset(&filter);
    for (uint32_t index = 0U; index < 3U; index++)
    {
        fixed_angle_sample(&filter, -181.0F, 0.0F, 1025.0F, &result);
    }
    require(result.condition == TERRAIN_DOWNHILL,
            "fixed base downhill direction");
    terrain_reset(&filter);
    for (uint32_t index = 0U; index < 3U; index++)
    {
        fixed_angle_sample(&filter, 0.0F, 181.0F, 1025.0F, &result);
    }
    require(((result.condition == TERRAIN_SIDE_LEFT) ||
             (result.condition == TERRAIN_SIDE_RIGHT)), "fixed base side tilt");
    terrain_reset(&filter);
    fixed_angle_sample(&filter, 669.0F, 0.0F, 797.0F, &result);
    require(result.condition == TERRAIN_EXCESS_TILT,
            "fixed base still enforces excessive tilt");
    printf("PASS: %u terrain checks\n", g_checks);
    return EXIT_SUCCESS;
}
