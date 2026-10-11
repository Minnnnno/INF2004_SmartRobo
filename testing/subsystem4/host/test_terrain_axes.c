/* Exercise the actual default Y-forward mapping and saved mounting. */
#include "terrain.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t g_checks = 0U;

static void require(bool b_condition, char const *p_message)
{
    if (!b_condition)
    {
        fprintf(stderr, "FAIL: %s\n", p_message);
        exit(EXIT_FAILURE);
    }
    g_checks++;
}

static terrain_result_t settle(float forward, float sideways, float upward)
{
    terrain_filter_t filter = {0};
    terrain_result_t result = {0};
    int16_t sample[3] = {0};

    require(terrain_use_fixed_reference(&filter), "fixed base loaded");
    for (uint32_t index = 0U; index < 3U; index++)
    {
        sample[index] = (int16_t)lroundf(
            (forward * filter.reference_y[index]) +
            (sideways * filter.reference_x[index]) +
            (upward * filter.reference_z[index]));
    }
    for (uint32_t index = 0U; index < 40U; index++)
    {
        (void)terrain_update(&filter, sample[0], sample[1], sample[2], &result);
    }
    return result;
}

int main(void)
{
    terrain_result_t result = settle(0.0F, 0.0F, 1041.0F);
    require(TERRAIN_FORWARD_AXIS_Y == 1, "test uses the firmware mapping");
    require(result.condition == TERRAIN_LEVEL, "fixed position LEVEL");
    result = settle(174.0F, 0.0F, 985.0F);
    require(result.condition == TERRAIN_UPHILL, "reference Y uphill");
    require(fabsf(result.pitch_deg - 10.0F) < 0.2F, "Y gives pitch");
    require(fabsf(result.roll_deg) < 0.2F, "Y is not sideways tilt");
    result = settle(-174.0F, 0.0F, 985.0F);
    require(result.condition == TERRAIN_DOWNHILL, "opposite Y downhill");
    require(result.pitch_deg < -9.8F, "downhill pitch negative");
    result = settle(0.0F, 174.0F, 985.0F);
    require(((result.condition == TERRAIN_SIDE_LEFT) ||
             (result.condition == TERRAIN_SIDE_RIGHT)), "reference X sideways");
    require(fabsf(result.roll_deg - 10.0F) < 0.2F, "X gives roll");
    require(fabsf(result.pitch_deg) < 0.2F, "X is not forward tilt");
    result = settle(0.0F, -174.0F, 985.0F);
    require(((result.condition == TERRAIN_SIDE_LEFT) ||
             (result.condition == TERRAIN_SIDE_RIGHT)), "opposite side slope");
    require(result.roll_deg < -9.8F, "opposite side roll sign");
    result = settle(174.0F, 70.0F, 982.0F);
    require(result.condition == TERRAIN_UPHILL, "forward dominates");
    result = settle(70.0F, 174.0F, 982.0F);
    require(((result.condition == TERRAIN_SIDE_LEFT) ||
             (result.condition == TERRAIN_SIDE_RIGHT)), "sideways dominates");
    result = settle(643.0F, 0.0F, 766.0F);
    require(result.condition == TERRAIN_EXCESS_TILT, "forward stop limit");
    result = settle(0.0F, 643.0F, 766.0F);
    require(result.condition == TERRAIN_EXCESS_TILT, "sideways stop limit");
    result = settle(342.0F, 0.0F, 940.0F);
    require(result.condition == TERRAIN_UPHILL, "20 degree uphill allowed");
    result = settle(-342.0F, 0.0F, 940.0F);
    require(result.condition == TERRAIN_DOWNHILL, "20 degree downhill allowed");
    result = settle(485.0F, 0.0F, 875.0F);
    require(result.condition == TERRAIN_UPHILL, "29 degree uphill allowed");
    result = settle(-485.0F, 0.0F, 875.0F);
    require(result.condition == TERRAIN_DOWNHILL, "29 degree downhill allowed");
    result = settle(515.0F, 0.0F, 857.0F);
    require(result.condition == TERRAIN_EXCESS_TILT, "31 degree uphill stops");
    result = settle(-515.0F, 0.0F, 857.0F);
    require(result.condition == TERRAIN_EXCESS_TILT,
            "31 degree downhill stops");
    result = settle(0.0F, 485.0F, 875.0F);
    require(((result.condition == TERRAIN_SIDE_LEFT) ||
             (result.condition == TERRAIN_SIDE_RIGHT)), "29 degree side allowed");
    result = settle(0.0F, 515.0F, 857.0F);
    require(result.condition == TERRAIN_EXCESS_TILT, "31 degree side stops");
    result = settle(0.0F, 174.0F, 985.0F);
    require(result.condition == TERRAIN_SIDE_LEFT,
            "positive roll selects SIDE_LEFT with default mapping");
    result = settle(0.0F, -174.0F, 985.0F);
    require(result.condition == TERRAIN_SIDE_RIGHT,
            "negative roll selects SIDE_RIGHT with default mapping");
    require(strcmp(terrain_action_name(TERRAIN_SIDE_LEFT), "SLOW") == 0,
            "left side action SLOW");
    require(strcmp(terrain_action_name(TERRAIN_SIDE_RIGHT), "SLOW") == 0,
            "right side action SLOW");
    require(strcmp(terrain_condition_name(TERRAIN_SIDE_LEFT), "SIDE_LEFT") == 0,
            "left label");
    require(strcmp(terrain_condition_name(TERRAIN_SIDE_RIGHT),
                   "SIDE_RIGHT") == 0, "right label");
    result = settle(0.0F, 70.0F, 998.0F);
    require(result.condition == TERRAIN_LEVEL, "small side tilt is level");
    result = settle(0.0F, -515.0F, 857.0F);
    require(result.condition == TERRAIN_EXCESS_TILT,
            "right side also respects stop threshold");
    result = settle(174.0F, 174.0F, 969.0F);
    require(result.pitch_condition == TERRAIN_UPHILL &&
            result.roll_condition == TERRAIN_SIDE_LEFT,
            "uphill and left reported simultaneously");
    result = settle(-174.0F, -174.0F, 969.0F);
    require(result.pitch_condition == TERRAIN_DOWNHILL &&
            result.roll_condition == TERRAIN_SIDE_RIGHT,
            "downhill and right reported simultaneously");
    result = settle(174.0F, -174.0F, 969.0F);
    require(result.pitch_condition == TERRAIN_UPHILL &&
            result.roll_condition == TERRAIN_SIDE_RIGHT,
            "uphill and right reported simultaneously");
    result = settle(-174.0F, 174.0F, 969.0F);
    require(result.pitch_condition == TERRAIN_DOWNHILL &&
            result.roll_condition == TERRAIN_SIDE_LEFT,
            "downhill and left reported simultaneously");
    result = settle(174.0F, 0.0F, 985.0F);
    require(result.pitch_condition == TERRAIN_UPHILL &&
            result.roll_condition == TERRAIN_LEVEL, "pitch only");
    result = settle(0.0F, 174.0F, 985.0F);
    require(result.pitch_condition == TERRAIN_LEVEL &&
            result.roll_condition == TERRAIN_SIDE_LEFT, "roll only");
    result = settle(0.0F, 0.0F, 1041.0F);
    require(result.pitch_condition == TERRAIN_LEVEL &&
            result.roll_condition == TERRAIN_LEVEL, "both axes level");
    result = settle(410.0F, 410.0F, 815.0F);
    require(result.pitch_condition == TERRAIN_UPHILL &&
            result.roll_condition == TERRAIN_SIDE_LEFT &&
            result.condition == TERRAIN_EXCESS_TILT,
            "combined tilt still stops while reporting both axes");
    terrain_filter_t axis_filter = {0};
    require(terrain_use_fixed_reference(&axis_filter), "axes test base");
    (void)terrain_update(&axis_filter, 0, 0, 0, &result);
    require(result.pitch_condition == TERRAIN_UNKNOWN &&
            result.roll_condition == TERRAIN_UNKNOWN && !result.b_tilt_valid,
            "invalid data cannot report directions");
    require(axis_filter.pitch_axis.condition == TERRAIN_UNKNOWN &&
            axis_filter.roll_axis.condition == TERRAIN_UNKNOWN,
            "invalid data clears direction history");
    printf("PASS: %u default mounting axis checks\n", (unsigned int)g_checks);
    return EXIT_SUCCESS;
}
