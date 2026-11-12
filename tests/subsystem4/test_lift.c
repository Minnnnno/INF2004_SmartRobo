/* Real lift.c and tilt.c: lift / overturn rules, timing, clearing, hump
 * immunity, gaps and NULL safety. Portable; no fakes needed.
 * Change Requirement: IMU collision detection was replaced by this.
 */
#include "../../app_program/subsystem4/lift.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned g_checks;
static uint32_t g_now;
static tilt_context_t g_tilt;
static lift_t g_lift;

static void check(int yes, char const *why)
{
    g_checks++;
    if (!yes) { fprintf(stderr, "FAIL: %s\n", why); exit(1); }
}

/* Level reference from 50 still samples; gravity_mg models sensor scale. */
static void start(int16_t gravity_mg)
{
    g_tilt = (tilt_context_t){0};
    for (unsigned i = 0U; i < TILT_CAPTURE_SAMPLES; i++)
    {
        (void)tilt_capture(&g_tilt, 0, 0, (int16_t)-gravity_mg);
    }
    check(0U != (g_tilt.progress & TILT_CALIBRATED), "reference captured");
    lift_reset(&g_lift);
    g_now = 1000U;
}

/* One 20 ms sample, armed. */
static uint8_t sample(int16_t x, int16_t y, int16_t z)
{
    uint8_t state = lift_step(&g_lift, &g_tilt, x, y, z, g_now, true);
    g_now += 20U;
    return state;
}

/* Sample with the body rolled by deg about the forward axis (gravity_mg size). */
static uint8_t rolled(double deg, double gravity_mg)
{
    double r = deg * 3.14159265358979323846 / 180.0;
    return sample((int16_t)lround(-gravity_mg * sin(r)), 0, (int16_t)lround(-gravity_mg * cos(r)));
}

/* Samples until the state changes from 'from' (at most limit); returns count. */
static unsigned until_not(uint8_t from, int16_t x, int16_t y, int16_t z, unsigned limit)
{
    unsigned n = 0U;
    while ((n < limit) && (from == sample(x, y, z))) { n++; }
    return n + 1U;
}

int main(void)
{
    /* Level and still for 10 s: never lifted. */
    start(1000);
    for (unsigned i = 0U; i < 500U; i++) { check(LIFT_NONE == sample(0, 0, -1000), "still: not lifted"); }
    check(100 == lift_upright_pct(&g_lift) && 0 == lift_vertical_mg(&g_lift), "level: 100 % upright, 0 mg");
    check(!lift_suspect(&g_lift), "level is not suspect");

    /* Driving vibration: +/-150 mg on every axis, alternating: never lifted. */
    for (unsigned i = 0U; i < 500U; i++)
    {
        int16_t v = (0U == (i % 2U)) ? 150 : -150;
        check(LIFT_NONE == sample(v, (int16_t)-v, (int16_t)(-1000 + v)), "vibration is not a lift");
    }

    /* Upside down: OVERTURNED after 200 ms (11th sample: 0..200 ms). */
    start(1000);
    check(11U == until_not(LIFT_NONE, 0, 0, 1000, 50U), "upside down latches after 200 ms");
    check(LIFT_OVERTURNED == lift_state(&g_lift), "upside down is OVERTURNED");
    check(-100 == lift_upright_pct(&g_lift), "upside down: -100 % upright");

    /* On its side (90 deg roll): OVERTURNED. */
    start(1000);
    for (unsigned i = 0U; i < 11U; i++) { (void)rolled(90.0, 1000.0); }
    check(LIFT_OVERTURNED == lift_state(&g_lift), "on its side is OVERTURNED");

    /* Tilting to one side is never a lift (35 deg rule removed): 45 deg held
     * 2 s stops nothing, but pauses hump measurement. */
    start(1000);
    for (unsigned i = 0U; i < 100U; i++) { check(LIFT_NONE == rolled(45.0, 1000.0), "45 deg held 2 s: NONE"); }
    check(lift_suspect(&g_lift), "45 deg pauses hump measurement");

    /* Overturn boundary: 59 deg never, 61 deg after 200 ms. */
    start(1000);
    for (unsigned i = 0U; i < 100U; i++) { check(LIFT_NONE == rolled(59.0, 1000.0), "59 deg held 2 s: NONE"); }
    start(1000);
    {
        unsigned n = 0U;
        while ((n < 50U) && (LIFT_NONE == rolled(61.0, 1000.0))) { n++; }
        check(10U == n && LIFT_OVERTURNED == lift_state(&g_lift), "61 deg: OVERTURNED after 200 ms");
    }
    start(1000);
    for (unsigned i = 0U; i < 200U; i++) { check(LIFT_NONE == rolled(25.0, 1000.0), "25 deg stays NONE"); }
    check(!lift_suspect(&g_lift), "25 deg is not suspect (hump measurement continues)");

    /* The hump pause lasts ~0.5 s after the robot is level again (tilt filter settles). */
    start(1000);
    for (unsigned i = 0U; i < 9U; i++) { check(LIFT_NONE == rolled(45.0, 1000.0), "short tilt"); }
    check(lift_suspect(&g_lift), "steep tilt suspends hump measurement");
    for (unsigned i = 0U; i < 24U; i++)
    {
        check(LIFT_NONE == rolled(0.0, 1000.0) && lift_suspect(&g_lift), "pause held while the filter settles");
    }
    check(LIFT_NONE == rolled(0.0, 1000.0) && !lift_suspect(&g_lift), "pause ends 25 samples after level");

    /* Picked up, then tipped past 60 deg: LIFTED upgrades to OVERTURNED. */
    start(1000);
    for (unsigned i = 0U; i < 4U; i++) { (void)sample(0, 0, -1300); }
    check(LIFT_LIFTED == lift_state(&g_lift), "lifted first");
    for (unsigned i = 0U; i < 11U; i++) { (void)rolled(120.0, 1000.0); }
    check(LIFT_OVERTURNED == lift_state(&g_lift), "then tipped over: OVERTURNED");

    /* Picked straight up: 4 consecutive samples >= +200 mg. */
    start(1000);
    check(4U == until_not(LIFT_NONE, 0, 0, -1250, 20U), "upward push latches on the 4th sample (~80 ms)");
    check(LIFT_LIFTED == lift_state(&g_lift) && 250 == lift_trigger_mg(&g_lift), "LIFTED with its trigger value");
    /* 199 mg never; three samples of 900 mg (a hump landing) never. */
    start(1000);
    for (unsigned i = 0U; i < 100U; i++) { check(LIFT_NONE == sample(0, 0, -1199), "199 mg is below the threshold"); }
    for (unsigned k = 0U; k < 20U; k++)
    {
        for (unsigned i = 0U; i < 3U; i++) { check(LIFT_NONE == sample(0, 0, -1900), "60 ms landing spike"); }
        for (unsigned i = 0U; i < 5U; i++) { check(LIFT_NONE == sample(0, 0, -1000), "between spikes"); }
    }
    check(!lift_suspect(&g_lift), "vertical spikes alone never suspend hump measurement");

    /* Sensor scale: gravity reads 1040 mg. Still is still; +200 above 1040 lifts. */
    start(1040);
    for (unsigned i = 0U; i < 100U; i++) { check(LIFT_NONE == sample(0, 0, -1040), "scaled sensor still"); }
    for (unsigned i = 0U; i < 10U; i++) { check(LIFT_NONE == sample(0, 0, -1230), "+190 above scaled gravity"); }
    check(4U == until_not(LIFT_NONE, 0, 0, -1245, 20U), "+205 above scaled gravity lifts");

    /* One-wheel hump profile: up to 20 deg roll over 1 s with +/-300 mg bumps. */
    start(1000);
    for (unsigned i = 0U; i < 150U; i++)
    {
        double deg = 20.0 * sin((double)i * 3.14159265358979323846 / 50.0);
        double bump = ((i % 7U) == 0U) ? 300.0 : 0.0;
        check(LIFT_NONE == rolled(deg < 0.0 ? -deg : deg, 1000.0 + bump), "hump profile never lifts");
    }

    /* Clearing: upright and still for 500 ms; held in the air shaking does not clear. */
    start(1000);
    for (unsigned i = 0U; i < 11U; i++) { (void)sample(0, 0, 1000); }
    check(LIFT_OVERTURNED == lift_state(&g_lift), "overturned before clearing");
    for (unsigned i = 0U; i < 25U; i++) { check(LIFT_OVERTURNED == sample(0, 0, -1000), "latched for 480 ms"); }
    check(LIFT_NONE == sample(0, 0, -1000), "cleared at 500 ms upright and still");
    for (unsigned i = 0U; i < 11U; i++) { (void)sample(0, 0, 1000); }
    for (unsigned i = 0U; i < 100U; i++)
    {
        int16_t z = (0U == (i % 2U)) ? -1250 : -800; /* carried around upright */
        check(LIFT_NONE != sample(0, 0, z), "moving in the air never clears");
    }
    for (unsigned i = 0U; i < 25U; i++) { (void)rolled(30.0, 1000.0); }
    check(LIFT_NONE != lift_state(&g_lift), "30 deg is not upright enough to clear");

    /* Not armed (settle window): values computed, nothing latched. */
    start(1000);
    for (unsigned i = 0U; i < 50U; i++)
    {
        check(LIFT_NONE == lift_step(&g_lift, &g_tilt, 0, 0, 1000, g_now, false), "not armed");
        g_now += 20U;
    }
    check(-100 == lift_upright_pct(&g_lift), "values computed while not armed");
    check(11U == until_not(LIFT_NONE, 0, 0, 1000, 50U), "timer starts only once armed");

    /* A gap restarts the timers. */
    start(1000);
    for (unsigned i = 0U; i < 9U; i++) { (void)sample(0, 0, 1000); }
    g_now += 500U;
    check(LIFT_NONE == sample(0, 0, 1000), "gap restarts the overturn timer");
    check(10U == until_not(LIFT_NONE, 0, 0, 1000, 50U), "full 200 ms needed after the gap");

    /* Near free fall: no direction, never overturned. */
    start(1000);
    for (unsigned i = 0U; i < 50U; i++) { check(LIFT_NONE == sample(0, 0, 100), "free fall has no direction"); }

    /* No reference yet / NULL safety. */
    {
        tilt_context_t blank = {0};
        lift_reset(&g_lift);
        for (unsigned i = 0U; i < 50U; i++)
        {
            check(LIFT_NONE == lift_step(&g_lift, &blank, 0, 0, 1000, g_now, true), "no reference: nothing");
            g_now += 20U;
        }
        check(LIFT_NONE == lift_step(NULL, &g_tilt, 0, 0, 1000, g_now, true), "NULL lift");
        check(LIFT_NONE == lift_step(&g_lift, NULL, 0, 0, 1000, g_now, true), "NULL tilt");
        check(LIFT_NONE == lift_state(NULL) && !lift_suspect(NULL) && 0 == lift_vertical_mg(NULL) &&
              0 == lift_trigger_mg(NULL) && 100 == lift_upright_pct(NULL), "NULL getters");
        lift_reset(NULL);
    }
    check(0 == strcmp("NONE", lift_name(LIFT_NONE)) && 0 == strcmp("LIFTED", lift_name(LIFT_LIFTED)) &&
          0 == strcmp("OVERTURNED", lift_name(LIFT_OVERTURNED)) && 0 == strcmp("INVALID", lift_name(3U)),
          "names");
    printf("PASS: %u lift checks (overturn, lift, hold, clear, humps, scale, gaps, NULL)\n", g_checks);
    return 0;
}
