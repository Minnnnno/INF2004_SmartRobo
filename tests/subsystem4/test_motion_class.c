/* Real motion_class.c: class rules, priority, debounce, overturned, yaw rate,
 * speed-change window, invalid input and gaps. No fakes needed (portable).
 */
#include "../../app_program/subsystem4/motion_class.h"
#include "../../app_program/subsystem4/terrain.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned g_checks;
static uint32_t g_now;
static motion_class_t g_mc;
static motion_class_input_t g_in;
static double g_heading_ddeg; /* Simulated Buddy 2 odometry heading, 0.1 deg. */
static double g_compass_ddeg; /* Simulated compass: the body's real rotation. */
static double g_compass_ratio; /* Body rotation / wheel rotation: 1.0 = no slip. */

static void check(int yes, char const *why)
{
    g_checks++;
    if (!yes) { fprintf(stderr, "FAIL: %s\n", why); exit(1); }
}

static void start(void)
{
    motion_class_reset(&g_mc);
    memset(&g_in, 0, sizeof(g_in));
    g_in.valid = true;
    g_in.stationary = true;
    g_in.track_mm = 115U;
    g_in.heading_valid = true;
    g_in.compass_valid = true;
    g_heading_ddeg = 0.0;
    g_compass_ddeg = 1234.0; /* Any starting direction: only changes matter. */
    g_compass_ratio = 1.0;
    g_now = 1000U;
}

/* One 20 ms cycle with the current inputs; overturned is a held level input.
 * The heading integrates the wheel-speed difference like Buddy 2's odometry. */
static uint8_t step(void)
{
    double rate_ddeg_s = ((double)g_in.speed_mm_s[1] - (double)g_in.speed_mm_s[0]) /
                         (double)g_in.track_mm * 572.957795; /* rad/s -> 0.1 deg/s */
    g_heading_ddeg += rate_ddeg_s * 0.020;
    g_compass_ddeg += rate_ddeg_s * 0.020 * g_compass_ratio;
    g_in.compass_ddeg = (int32_t)((g_compass_ddeg >= 0.0) ? (g_compass_ddeg + 0.5)
                                                          : (g_compass_ddeg - 0.5));
    g_in.heading_ddeg = (int32_t)((g_heading_ddeg >= 0.0) ? (g_heading_ddeg + 0.5)
                                                          : (g_heading_ddeg - 0.5));
    motion_class_step(&g_mc, &g_in, g_now);
    g_now += 20U;
    return motion_class_get(&g_mc);
}

static void wheels(int16_t left, int16_t right)
{
    g_in.stationary = false;
    g_in.speed_mm_s[0] = left;
    g_in.speed_mm_s[1] = right;
}

/* Drive steadily long enough to settle on MOVING. */
static void cruise(void)
{
    start();
    (void)step();
    wheels(100, 100);
    for (unsigned i = 0U; i < 30U; i++) { (void)step(); }
    check(MC_MOVING == motion_class_get(&g_mc), "steady straight driving is MOVING");
}

int main(void)
{
    /* Defaults and NULL safety. */
    motion_class_reset(&g_mc);
    check(!motion_class_valid(&g_mc) && (MC_STATIONARY == motion_class_get(&g_mc)), "reset: invalid STATIONARY");
    check(!motion_class_valid(NULL) && (MC_STATIONARY == motion_class_get(NULL)), "NULL is invalid STATIONARY");
    motion_class_step(NULL, &g_in, 0U); motion_class_step(&g_mc, NULL, 0U);
    motion_class_reset(NULL);
    check(0 == strcmp("OVERTURNED", motion_class_name(MC_OVERTURNED)) &&
          0 == strcmp("STATIONARY", motion_class_name(8U)), "names, masked code");

    /* Yaw rate from wheel speeds: + = left/anticlockwise (right wheel faster). */
    check(50 == motion_class_yaw_from_speeds(0, 100, 115U), "pivot left about 50 deg/s");
    check(-50 == motion_class_yaw_from_speeds(100, 0, 115U), "pivot right negative");
    check(37 == motion_class_yaw_from_speeds(63, 138, 115U), "line correction 63/138 is 37 deg/s");
    check(0 == motion_class_yaw_from_speeds(100, 100, 115U), "straight is zero");
    check(0 == motion_class_yaw_from_speeds(0, 100, 0U), "zero track is safe");
    check(32767 == motion_class_yaw_from_speeds(-32768, 32767, 1U), "extreme input clamps");

    /* First valid sample is reported directly. */
    start();
    check(MC_STATIONARY == step() && motion_class_valid(&g_mc), "at rest: STATIONARY, valid");

    /* Acceleration from encoder speed change: 0 -> 100 mm/s over 400 ms = 250 mm/s^2. */
    {
        bool saw_accel = false;
        for (int16_t v = 5; v <= 100; v = (int16_t)(v + 5))
        {
            wheels(v, v);
            if (MC_ACCELERATING == step()) { saw_accel = true; }
        }
        check(saw_accel, "speed ramp reports ACCELERATING");
        check(motion_class_accel_mm_s2(&g_mc) >= MC_ACCEL_MM_S2, "measured acceleration above threshold");
        for (unsigned i = 0U; i < 20U; i++) { (void)step(); }
        check(MC_MOVING == motion_class_get(&g_mc), "steady after the ramp: MOVING");
    }

    /* Braking (merged into ACCELERATING): 100 -> 0 over 400 ms, accel negative. */
    {
        bool saw_brake = false;
        bool negative = false;
        for (int16_t v = 95; v >= 0; v = (int16_t)(v - 5))
        {
            wheels(v, v);
            if (MC_ACCELERATING == step()) { saw_brake = true; }
            if (motion_class_accel_mm_s2(&g_mc) <= -MC_ACCEL_MM_S2) { negative = true; }
        }
        check(saw_brake, "slowing down reports ACCELERATING (braking merged)");
        check(negative, "slowing down has negative accel");
        g_in.stationary = true;
        for (unsigned i = 0U; i < 5U; i++) { (void)step(); }
        check(MC_STATIONARY == motion_class_get(&g_mc), "stopped: STATIONARY");
    }

    /* Reversing: 0 -> -75 mm/s like the line search back-up. */
    start();
    (void)step();
    {
        bool saw_other = false;
        int16_t v;
        for (v = -5; v >= -75; v = (int16_t)(v - 5))
        {
            uint8_t mc;
            wheels(v, v);
            mc = step();
            if ((v <= -40) && (MC_REVERSING != mc)) { saw_other = true; }
        }
        check(MC_REVERSING == motion_class_get(&g_mc), "back-up reports REVERSING");
        check(!saw_other, "REVERSING from -40 mm/s on (debounced)");
        check(motion_class_accel_mm_s2(&g_mc) > 0, "reversing away is speeding up (+ accel)");
        for (unsigned i = 0U; i < 20U; i++) { check(MC_REVERSING == step(), "steady reverse stays REVERSING"); }
        for (v = -70; v <= -25; v = (int16_t)(v + 5))
        {
            wheels(v, v);
            check(MC_REVERSING == step(), "slowing reverse is still REVERSING");
        }
        g_in.stationary = true;
        for (unsigned i = 0U; i < 5U; i++) { (void)step(); }
        check(MC_STATIONARY == motion_class_get(&g_mc), "stopped after reverse: STATIONARY");
    }

    /* Standstill jitter is never REVERSING. */
    start();
    (void)step();
    for (unsigned i = 0U; i < 20U; i++)
    {
        wheels((0U == (i % 2U)) ? -10 : 0, (0U == (i % 2U)) ? -10 : 0);
        check(MC_REVERSING != step(), "-10 mm/s jitter is not REVERSING");
    }

    /* While reversing: sharp steering is TURNING, a hump is CLIMBING, overturned is OVERTURNED. */
    start();
    (void)step();
    wheels(-75, -75);
    for (unsigned i = 0U; i < 20U; i++) { (void)step(); }
    check(MC_REVERSING == motion_class_get(&g_mc), "reverse established");
    wheels(-120, -30); /* +45 deg/s while reversing */
    {
        unsigned n = 0U;
        while ((MC_TURNING != step()) && (n < 40U)) { n++; }
        check(n < 40U, "reverse with sharp steering becomes TURNING");
    }
    wheels(-75, -75);
    g_in.terrain_phase = TERRAIN_CLIMBING;
    (void)step(); (void)step();
    check(MC_CLIMBING == step(), "hump while reversing is CLIMBING");
    g_in.overturned = true;
    check(MC_OVERTURNED == step(), "overturned while reversing is OVERTURNED at once");
    g_in.overturned = false;
    g_in.terrain_phase = TERRAIN_WAITING;

    /* Gentle acceleration below the threshold stays MOVING (no flicker). */
    cruise();
    for (unsigned i = 0U; i < 25U; i++) { wheels((int16_t)(100 + i), (int16_t)(100 + i)); (void)step(); }
    check(MC_MOVING == motion_class_get(&g_mc), "50 mm/s^2 drift is not ACCELERATING");

    /* TURNING needs a real heading change: a 50 deg/s pivot is reported
     * once 15 deg have accumulated (~0.3 s) plus the 60 ms debounce. */
    cruise();
    wheels(0, 100);
    {
        unsigned n = 1U;
        while ((MC_TURNING != step()) && (n < 40U)) { n++; }
        check((n >= 14U) && (n <= 22U), "pivot reported as TURNING after ~0.3-0.4 s");
    }
    check(50 == motion_class_yaw_deg_s(&g_mc), "yaw rate still published");
    wheels(100, 100);
    check(MC_TURNING == step(), "single straight cycle does not flicker back");

    /* Line-following wiggle: +/-37 deg/s flipping every 200 ms nets ~0. */
    cruise();
    for (unsigned i = 0U; i < 150U; i++)
    {
        if (0U == ((i / 10U) % 2U)) { wheels(63, 138); } else { wheels(138, 63); }
        check(MC_TURNING != step(), "line-following wiggle is never TURNING");
    }
    check(!motion_class_take_turn(&g_mc, NULL, NULL, NULL), "no turn reported for wiggle");

    /* Curves: ~20 deg/s stays MOVING, ~40 deg/s is TURNING. */
    cruise();
    wheels(80, 120);
    for (unsigned i = 0U; i < 100U; i++) { check(MC_TURNING != step(), "20 deg/s curve is not TURNING"); }
    wheels(60, 140);
    {
        unsigned n = 0U;
        while ((MC_TURNING != step()) && (n < 40U)) { n++; }
        check(n < 40U, "40 deg/s curve is TURNING");
    }

    /* 90 deg left turn at 50 deg/s: angle, duration and peak reported once. */
    cruise();
    wheels(0, 100);
    for (unsigned i = 0U; i < 90U; i++) { (void)step(); } /* 1.8 s */
    wheels(100, 100);
    {
        int16_t angle = 0; uint16_t duration = 0U; int16_t peak = 0;
        unsigned n = 0U;
        while (!motion_class_take_turn(&g_mc, &angle, &duration, &peak) && (n < 60U))
        {
            (void)step(); n++;
        }
        check(n < 60U, "turn completes after the robot straightens");
        check((angle >= 850) && (angle <= 950), "90 deg left turn measured (+900 +/-50)");
        check((duration >= 1600U) && (duration <= 2300U), "turn duration about 1.8-2 s");
        check(50 == peak, "peak yaw 50 deg/s");
        /* The class may lag the turn report by a few cycles (ACCELERATING after
         * straightening restarts its debounce); it must settle on MOVING soon. */
        for (unsigned i = 0U; (i < 10U) && (MC_MOVING != motion_class_get(&g_mc)); i++) { (void)step(); }
        check(MC_MOVING == motion_class_get(&g_mc), "back to MOVING shortly after the turn");
        check(!motion_class_take_turn(&g_mc, NULL, NULL, NULL), "turn reported only once");
    }

    /* A brief overturned flag and a hump during a 90 deg turn do not split it. */
    cruise();
    wheels(0, 100);
    for (unsigned i = 0U; i < 90U; i++)
    {
        g_in.overturned = (30U == i);
        g_in.terrain_phase = ((i >= 50U) && (i < 60U)) ? TERRAIN_CLIMBING : TERRAIN_WAITING;
        (void)step();
        if ((i > 30U) && (i < 80U))
        {
            check(!motion_class_take_turn(&g_mc, NULL, NULL, NULL), "turn not closed by OVERTURNED/CLIMBING on top");
        }
    }
    g_in.terrain_phase = TERRAIN_WAITING;
    wheels(100, 100);
    {
        int16_t angle = 0;
        unsigned n = 0U;
        while (!motion_class_take_turn(&g_mc, &angle, NULL, NULL) && (n < 60U)) { (void)step(); n++; }
        check((n < 60U) && (angle >= 850) && (angle <= 950), "one 90 deg turn reported across a knock and a hump");
    }

    /* Compass angle per turn (no slip verdict): recorded when valid, n/a otherwise. */
    {
        int16_t angle = 0;
        int16_t compass = 0;
        unsigned n = 0U;

        /* Body turns 98 % of what the wheels say: compass about 88 deg. */
        cruise();
        g_compass_ratio = 0.98;
        wheels(0, 100);
        for (unsigned i = 0U; i < 90U; i++) { (void)step(); }
        wheels(100, 100);
        while (!motion_class_take_turn(&g_mc, &angle, NULL, NULL) && (n < 60U)) { (void)step(); n++; }
        check(motion_class_turn_compass(&g_mc, &compass), "compass valid for the whole turn");
        check((compass >= 840) && (compass <= 920), "compass angle recorded (~88 deg)");

        /* Body turns only 75 %: the angle is still just reported (~68 deg). */
        cruise();
        g_compass_ratio = 0.75;
        wheels(0, 100);
        for (unsigned i = 0U; i < 90U; i++) { (void)step(); }
        wheels(100, 100);
        n = 0U;
        while (!motion_class_take_turn(&g_mc, &angle, NULL, NULL) && (n < 60U)) { (void)step(); n++; }
        check(motion_class_turn_compass(&g_mc, &compass), "compass valid");
        check((compass >= 630) && (compass <= 720) && (angle >= 850), "encoder 90 vs compass ~68 reported side by side");

        /* Compass invalid mid-turn (steel, hump): n/a. */
        cruise();
        g_compass_ratio = 0.5;
        wheels(0, 100);
        for (unsigned i = 0U; i < 90U; i++)
        {
            g_in.compass_valid = !((i >= 40U) && (i < 45U));
            (void)step();
        }
        g_in.compass_valid = true;
        wheels(100, 100);
        n = 0U;
        while (!motion_class_take_turn(&g_mc, &angle, NULL, NULL) && (n < 60U)) { (void)step(); n++; }
        check((n < 60U) && !motion_class_turn_compass(&g_mc, &compass), "compass n/a after an invalid sample");
        check(!motion_class_turn_compass(NULL, NULL), "NULL safe");
    }

    /* Right U-turn: about -180 deg. */
    cruise();
    wheels(100, 0);
    for (unsigned i = 0U; i < 180U; i++) { (void)step(); } /* 3.6 s */
    wheels(100, 100);
    {
        int16_t angle = 0;
        unsigned n = 0U;
        while (!motion_class_take_turn(&g_mc, &angle, NULL, NULL) && (n < 60U)) { (void)step(); n++; }
        check((n < 60U) && (angle <= -1750) && (angle >= -1850), "right U-turn measured (-1800 +/-50)");
    }

    /* Odometry reset (heading jumps 200 deg in one step): never a false turn. */
    cruise();
    g_heading_ddeg += 2000.0;
    for (unsigned i = 0U; i < 40U; i++) { check(MC_TURNING != step(), "odometry jump is not TURNING"); }

    /* Invalid input mid-turn drops it (not reported); invalid heading never turns. */
    cruise();
    wheels(0, 100);
    for (unsigned i = 0U; i < 40U; i++) { (void)step(); }
    check(MC_TURNING == motion_class_get(&g_mc), "turning before the dropout");
    g_in.valid = false; (void)step(); g_in.valid = true;
    wheels(100, 100);
    for (unsigned i = 0U; i < 60U; i++) { (void)step(); }
    check(!motion_class_take_turn(&g_mc, NULL, NULL, NULL), "turn interrupted by invalid input is dropped");
    cruise();
    g_in.heading_valid = false;
    wheels(0, 100);
    for (unsigned i = 0U; i < 60U; i++) { check(MC_TURNING != step(), "no heading, no TURNING"); }
    g_in.heading_valid = true;
    check(!motion_class_take_turn(NULL, NULL, NULL, NULL), "NULL take_turn is safe");

    /* OVERTURNED: immediate, held while the lift detector holds it, then
     * debounced back like any other class (no 300 ms hold any more). */
    cruise();
    g_in.overturned = true;
    check(MC_OVERTURNED == step(), "overturned reported in the same cycle");
    for (unsigned i = 0U; i < 50U; i++) { check(MC_OVERTURNED == step(), "overturned held while latched"); }
    g_in.overturned = false;
    (void)step();
    (void)step();
    check(MC_MOVING == step(), "back to MOVING after the latch clears plus confirmation");

    /* Priority: OVERTURNED > CLIMBING/DESCENDING > STATIONARY > TURNING > REVERSING > ACCEL > MOVING. */
    cruise();
    wheels(0, 100);
    g_in.terrain_phase = TERRAIN_CLIMBING;
    (void)step(); (void)step();
    check(MC_CLIMBING == step(), "hump phase beats turning");
    g_in.overturned = true;
    check(MC_OVERTURNED == step(), "overturned beats climbing");
    g_in.overturned = false;
    for (unsigned i = 0U; i < 20U; i++) { (void)step(); }
    g_in.terrain_phase = TERRAIN_DESCENDING;
    (void)step(); (void)step();
    check(MC_DESCENDING == step(), "DESCENDING from terrain phase");
    g_in.terrain_phase = TERRAIN_WAITING;
    g_in.stationary = true;
    (void)step(); (void)step();
    check(MC_STATIONARY == step(), "stationary beats a stale speed difference");

    /* Invalid input: STATIONARY, invalid, even when overturned. */
    cruise();
    g_in.valid = false;
    g_in.overturned = true;
    check(MC_STATIONARY == step() && !motion_class_valid(&g_mc), "invalid publishes STATIONARY, invalid");
    check(0 == motion_class_yaw_deg_s(&g_mc), "invalid clears yaw");
    g_in.valid = true;
    g_in.overturned = false;
    check(MC_OVERTURNED != step(), "overturned seen while invalid is not replayed");

    /* Gap: a long pause reseeds the speed window (no false ACCELERATING). */
    cruise();
    g_now += 500U;
    wheels(200, 200);
    for (unsigned i = 0U; i < 4U; i++) { check(MC_ACCELERATING != step(), "no acceleration from a gap"); }
    check(0 == motion_class_accel_mm_s2(&g_mc), "window reseeded after the gap");

    /* Packed byte: count never exceeds its 2 bits; class bits valid. */
    cruise();
    for (unsigned i = 0U; i < 50U; i++)
    {
        wheels((0U == (i % 2U)) ? 63 : 100, (0U == (i % 2U)) ? 138 : 100);
        (void)step();
        check(((g_mc.packed >> 6U) & 3U) < MC_CONFIRM_SAMPLES, "count within 2 bits");
    }
    check(sizeof(g_mc.packed) == 1U, "class, candidate and count share one byte");

    printf("PASS: %u motion class checks (rules, priority, debounce, overturned, yaw, gaps)\n", g_checks);
    return 0;
}
