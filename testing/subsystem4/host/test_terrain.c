/* Height-based hump detection: one wheel of a 2-wheel + caster robot climbs.
 * Input is the right-minus-left wheel contact height in 0.1 mm; the detector
 * smooths it (s += (h - s) / 4) and reseeds after gaps/invalid samples. */
#include "../../../app_program/subsystem4/terrain.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static unsigned checks;
static uint32_t now;
static void check(int ok, char const *why)
{
    checks++;
    if (!ok) { printf("FAIL terrain: %s\n", why); exit(1); }
}
static void hold(terrain_t *t, int tenths, unsigned duration)
{
    for (unsigned elapsed = 0U; elapsed <= duration; elapsed += 20U)
    {
        terrain_step(t, true, (int16_t)tenths, now); now += 20U;
    }
}
static unsigned phase(terrain_t const *t) { return terrain_status(t) & TERRAIN_PHASE_MASK; }
static unsigned peak_mm(terrain_t const *t) { return terrain_last_peak(t) & TERRAIN_PEAK_MM_MASK; }
static unsigned run_mm(terrain_t const *t) { return terrain_run_max(t) & TERRAIN_PEAK_MM_MASK; }
/* Flat-floor driving sway as in the robot log: a slow +/-3.0 mm steering wave
 * (about one swing per second) plus fast vibration jitter, peaking near 3.3 mm. */
static int16_t sway(unsigned i)
{
    static const int8_t jitter[5] = {3, -2, 1, -3, 2};
    return (int16_t)(lround(30.0 * sin(6.283185307 * (double)i / 50.0)) + jitter[i % 5U]);
}

/* One-sided hump: ramp 0 -> peak over 0.5 s, top 0.3 s, ramp down, flat 0.8 s. */
static void hump(terrain_t *t, int peak_tenths)
{
    for (int i = 0; i <= 25; i++) { terrain_step(t, true, (int16_t)(peak_tenths * i / 25), now); now += 20U; }
    hold(t, peak_tenths, 300U);
    for (int i = 25; i >= 0; i--) { terrain_step(t, true, (int16_t)(peak_tenths * i / 25), now); now += 20U; }
    hold(t, 0, 800U);
}

int main(void)
{
    terrain_t t = {0};

    check(terrain_status(&t) == 0U && terrain_run_max(&t) == 0U && terrain_last_peak(&t) == 0U, "cold zero");
    hold(&t, 0, 400U);
    check(phase(&t) == TERRAIN_WAITING && (terrain_status(&t) & TERRAIN_VALID), "level waits");

    /* Flat-floor driving sway like the robot log: +/-3.3 mm swings, never a hump. */
    for (unsigned i = 0U; i < 500U; i++) { terrain_step(&t, true, sway(i), now); now += 20U; }
    check(phase(&t) == TERRAIN_WAITING && terrain_run_max(&t) == 0U, "driving sway of 3.3 mm ignored");
    hold(&t, 49, 1000U);
    check(phase(&t) == TERRAIN_WAITING && terrain_run_max(&t) == 0U, "steady 4.9 mm ignored");

    /* A short 8 mm jolt (60 ms) is smoothed below the 100 ms hold. */
    hold(&t, 0, 600U);
    hold(&t, 80, 60U); hold(&t, 0, 600U);
    check(phase(&t) == TERRAIN_WAITING && terrain_run_max(&t) == 0U, "short jolt rejected");

    /* 6 mm step: smoothing delay plus the 100 ms hold, then CLIMBING. */
    hold(&t, 60, 100U);
    check(phase(&t) == TERRAIN_WAITING, "not immediately (smoothing + hold)");
    hold(&t, 60, 300U);
    check(phase(&t) == TERRAIN_CLIMBING, "climbing");
    check(terrain_run_max(&t) == (TERRAIN_RUN_VALID | 6U), "run max follows live peak");
    hold(&t, 0, 800U);
    check(terrain_take_completed(&t) && peak_mm(&t) == 6U, "small hump completes");

    /* 30 mm hump under the right wheel. */
    terrain_reset(&t);
    hump(&t, 300);
    check(phase(&t) == TERRAIN_WAITING, "hump done, back to waiting");
    check(terrain_take_completed(&t), "completion event");
    check(!terrain_take_completed(&t), "event consumed once");
    check(peak_mm(&t) == 30U && (terrain_last_peak(&t) & TERRAIN_PEAK_RIGHT), "30 mm under right wheel");
    check(terrain_run_max(&t) == (TERRAIN_RUN_VALID | 30U), "run max 30");

    /* Left wheel, then a sequence: run max keeps the highest. */
    hump(&t, -200);
    check(terrain_take_completed(&t) && peak_mm(&t) == 20U && !(terrain_last_peak(&t) & TERRAIN_PEAK_RIGHT),
          "20 mm under left wheel");
    hump(&t, 354); check(terrain_take_completed(&t) && peak_mm(&t) == 35U, "35.4 rounds to 35");
    hump(&t, -100); check(terrain_take_completed(&t) && peak_mm(&t) == 10U, "smallest course hump, 10 mm");
    check(run_mm(&t) == 35U, "run max is the highest peak");

    /* 10 mm hump with the driving sway on top: one hump, peak ~10 mm. */
    terrain_reset(&t);
    for (unsigned i = 0U; i < 150U; i++)
    {
        int base = (i < 25U) ? (int)(i * 4U) : ((i < 60U) ? 100 : ((i < 85U) ? (int)((85U - i) * 4U) : 0));
        terrain_step(&t, true, (int16_t)(base + sway(i)), now); now += 20U;
    }
    check(terrain_take_completed(&t) && !terrain_take_completed(&t), "noisy 10 mm hump counts once");
    check(peak_mm(&t) >= 9U && peak_mm(&t) <= 12U, "noisy 10 mm hump peak within a few mm");

    /* Phase sequence, re-climb margin and exit hold (levels settled each step). */
    terrain_reset(&t);
    hold(&t, 100, 600U); check(phase(&t) == TERRAIN_CLIMBING, "climbing");
    hold(&t, 300, 600U); check(phase(&t) == TERRAIN_CLIMBING && run_mm(&t) == 30U, "rising stays climbing");
    hold(&t, 280, 600U); check(phase(&t) == TERRAIN_CLIMBING, "2 mm below peak still climbing (crest)");
    hold(&t, 265, 600U); check(phase(&t) == TERRAIN_DESCENDING, "3 mm below peak descends");
    hold(&t, 310, 600U);
    check(phase(&t) == TERRAIN_DESCENDING && run_mm(&t) == 30U, "rise of under 2 mm is noise, no re-climb");
    hold(&t, 330, 600U);
    check(phase(&t) == TERRAIN_CLIMBING && run_mm(&t) == 33U, "rise of 2 mm+ re-climbs (double hump)");
    hold(&t, 100, 600U); check(phase(&t) == TERRAIN_DESCENDING, "descending again");
    hold(&t, 30, 600U); check(phase(&t) == TERRAIN_DESCENDING, "3.0 mm is not done (exit < 2.5 mm)");
    hold(&t, 20, 300U); check(phase(&t) == TERRAIN_DESCENDING, "exit needs 300 ms below 2.5 mm");
    hold(&t, 20, 100U);
    check(phase(&t) == TERRAIN_WAITING && terrain_take_completed(&t) && peak_mm(&t) == 33U, "completes");

    /* Up-only ramp under one wheel: stays climbing, run max still reported. */
    terrain_reset(&t);
    hold(&t, 400, 2000U);
    check(phase(&t) == TERRAIN_CLIMBING && run_mm(&t) == 40U && !terrain_take_completed(&t), "up-only ramp counts");

    /* Invalid observations keep phase/peak, mark unknown, restart timing,
     * and reseed the smoothing from the next sample. */
    terrain_reset(&t);
    hold(&t, 300, 600U); hold(&t, 10, 200U);
    check(phase(&t) == TERRAIN_DESCENDING, "descending before gap");
    terrain_step(&t, false, 0, now); now += 20U;
    check(phase(&t) == TERRAIN_DESCENDING && !(terrain_status(&t) & TERRAIN_VALID) &&
          run_mm(&t) == 30U, "invalid keeps state, unknown");
    hold(&t, 10, 280U); check(phase(&t) == TERRAIN_DESCENDING, "exit timing restarted after invalid");
    terrain_step(&t, true, 10, now); now += 20U;
    check(phase(&t) == TERRAIN_WAITING && peak_mm(&t) == 30U, "completes after a fresh 300 ms");
    check(terrain_take_completed(&t), "event after invalid gap");

    /* Unconsumed event survives valid and invalid observations. */
    hump(&t, 150);
    terrain_step(&t, false, 0, now); now += 20U; hold(&t, 0, 60U);
    check(terrain_take_completed(&t) && !terrain_take_completed(&t), "event preserved until consumed");

    /* A sample gap reseeds and restarts the entry hold. */
    terrain_reset(&t);
    terrain_step(&t, true, 60, now); now += TERRAIN_MAX_GAP_MS;
    terrain_step(&t, true, 60, now); now += 20U;
    hold(&t, 60, 60U); check(phase(&t) == TERRAIN_WAITING, "gap restarts entry hold");
    terrain_step(&t, true, 60, now); now += 20U; check(phase(&t) == TERRAIN_CLIMBING, "entry after fresh hold");

    /* Clock wrap and saturation of the 7-bit field. */
    terrain_reset(&t); now = UINT32_MAX - 100U;
    hold(&t, 2000, 200U);
    check(phase(&t) == TERRAIN_CLIMBING && run_mm(&t) == 127U, "wrap; peak saturates at 127 mm");
    check((terrain_status(&t) & 0xF0U) == 0U, "private bits not exposed");

    check(strcmp(terrain_phase_name(TERRAIN_DESCENDING), "DESCENDING") == 0, "names");
    check(!terrain_take_completed(NULL) && terrain_status(NULL) == 0U && terrain_run_max(NULL) == 0U &&
          terrain_last_peak(NULL) == 0U, "NULL safety");
    terrain_reset(NULL); terrain_step(NULL, true, 0, 0U);
    printf("PASS: %u height hump, sway, side, peak, run max, hold, gap and wrap checks\n", checks);
    return 0;
}
