#include "terrain.h"
#include <stddef.h>
#define TERRAIN_PENDING (0x10U)
#define TERRAIN_SIDE_RIGHT (0x20U)
#define TERRAIN_PUBLIC_MASK (0x0FU)
#define TERRAIN_COMPLETED_EVENT (0x80U)
_Static_assert(TERRAIN_EXIT_TENTHS < TERRAIN_ENTER_TENTHS,
               "completion level must be below the entry level (hysteresis)");
/* Kept across an invalid observation or a sample gap. */
#define TERRAIN_RETAIN_MASK (TERRAIN_PHASE_MASK | TERRAIN_SLOW | TERRAIN_SIDE_RIGHT | \
                             TERRAIN_COMPLETED_EVENT)

/* Reset the event, peaks and run maximum only when explicitly starting a new run. */
void terrain_reset(terrain_t *p_terrain)
{
    if (NULL != p_terrain)
    {
        *p_terrain = (terrain_t){0};
    }
}

/* Take the event without disturbing phase, validity or pending confirmation. */
bool terrain_take_completed(terrain_t *p_terrain)
{
    bool completed;
    if (NULL == p_terrain)
    {
        return false;
    }
    completed = (0U != (p_terrain->flags & TERRAIN_COMPLETED_EVENT));
    p_terrain->flags &= (uint8_t)~TERRAIN_COMPLETED_EVENT;
    return completed;
}

/* Read public phase/validity/request without exposing candidate bookkeeping. */
uint8_t terrain_status(terrain_t const *p_terrain)
{
    return (NULL == p_terrain) ? 0U : (p_terrain->flags & TERRAIN_PUBLIC_MASK);
}

uint8_t terrain_last_peak(terrain_t const *p_terrain)
{
    return (NULL == p_terrain) ? 0U : p_terrain->last_peak;
}

uint8_t terrain_run_max(terrain_t const *p_terrain)
{
    return (NULL == p_terrain) ? 0U : p_terrain->run_max;
}

/* Fixed names for console reports; an invalid observation retains last phase. */
char const *terrain_phase_name(uint8_t status)
{
    switch (status & TERRAIN_PHASE_MASK)
    {
        case TERRAIN_WAITING: return "WAITING";
        case TERRAIN_CLIMBING: return "CLIMBING";
        case TERRAIN_DESCENDING: return "DESCENDING";
        default: return "INVALID";
    }
}

/* Whole millimetres from 0.1 mm, rounded, limited to the 7-bit field. */
static uint8_t terrain_mm(uint16_t tenths)
{
    uint32_t mm = ((uint32_t)tenths + 5U) / 10U;
    return (uint8_t)((mm > TERRAIN_PEAK_MM_MASK) ? TERRAIN_PEAK_MM_MASK : mm);
}

/* Record a new live peak (and its side); the run maximum follows it. */
static void terrain_raise_peak(terrain_t *p_terrain, uint16_t magnitude, bool right)
{
    uint8_t mm;
    p_terrain->peak_tenths = magnitude;
    p_terrain->flags = (uint8_t)((p_terrain->flags & (uint8_t)~TERRAIN_SIDE_RIGHT) |
                                 (right ? TERRAIN_SIDE_RIGHT : 0U));
    mm = terrain_mm(magnitude);
    if (mm > (p_terrain->run_max & TERRAIN_PEAK_MM_MASK))
    {
        p_terrain->run_max = mm;
    }
    p_terrain->run_max |= TERRAIN_RUN_VALID;
}

/* Fall from the peak that counts as descending: 3 mm, or half the peak when
 * that is smaller, so a hump that peaks below ~3 mm can still complete.
 * Peaks are at least TERRAIN_ENTER_TENTHS, so the result is always positive.
 */
static int32_t terrain_drop(uint16_t peak_tenths)
{
    int32_t half = (int32_t)peak_tenths / 2;
    return (half < TERRAIN_DROP_TENTHS) ? half : TERRAIN_DROP_TENTHS;
}

/* Start or continue a timed confirmation; true once held for hold_ms. */
static bool terrain_confirm(terrain_t *p_terrain, uint32_t now_ms, uint32_t hold_ms)
{
    if (0U == (p_terrain->flags & TERRAIN_PENDING))
    {
        p_terrain->flags |= TERRAIN_PENDING;
        p_terrain->candidate_since = now_ms;
        return false;
    }
    return (now_ms - p_terrain->candidate_since) >= hold_ms;
}

/* All tests use the smoothed height |h| (seeded from the first sample after a
 * gap or invalid observation, so no false ramp from zero).
 * WAITING: |h| >= 5 mm held 100 ms -> CLIMBING. CLIMBING: track the peak;
 * |h| <= peak - min(3 mm, peak/2) -> DESCENDING. DESCENDING: |h| >= peak + 2 mm
 * returns to CLIMBING (double hump; smaller rises are noise and do not move the
 * peak); |h| < 2.5 mm held 300 ms completes: the peak and its
 * side are stored, an event latches and WAITING rearms. An up-only ramp
 * stays CLIMBING with the peak (and run maximum) still updated.
 */
void terrain_step(terrain_t *p_terrain, bool valid, int16_t height_tenths, uint32_t now_ms)
{
    uint8_t phase;
    uint16_t magnitude;
    bool right;

    if (NULL == p_terrain)
    {
        return;
    }
    if (!valid)
    {
        p_terrain->flags &= TERRAIN_RETAIN_MASK;
        return;
    }
    if ((0U == (p_terrain->flags & TERRAIN_VALID)) ||
        ((now_ms - p_terrain->last_sample) >= TERRAIN_MAX_GAP_MS))
    {
        p_terrain->flags &= TERRAIN_RETAIN_MASK; /* Restart any timed confirmation. */
        p_terrain->smooth_tenths = height_tenths;  /* Reseed the smoothing. */
    }
    else
    {
        /* Rounded integer low-pass; |h| <= 3276 so no overflow in int32. */
        int32_t step = (int32_t)height_tenths - (int32_t)p_terrain->smooth_tenths;
        int32_t half = (step >= 0) ? (1 << (TERRAIN_SMOOTH_SHIFT - 1U))
                                   : -(1 << (TERRAIN_SMOOTH_SHIFT - 1U));
        p_terrain->smooth_tenths = (int16_t)(p_terrain->smooth_tenths +
                                             (step + half) / (1 << TERRAIN_SMOOTH_SHIFT));
    }
    height_tenths = p_terrain->smooth_tenths;
    p_terrain->last_sample = now_ms;
    p_terrain->flags |= TERRAIN_VALID;
    right = (height_tenths > 0);
    magnitude = (uint16_t)((height_tenths < 0) ? -(int32_t)height_tenths : (int32_t)height_tenths);
    phase = p_terrain->flags & TERRAIN_PHASE_MASK;

    if (TERRAIN_WAITING == phase)
    {
        if (magnitude < (uint16_t)TERRAIN_ENTER_TENTHS)
        {
            p_terrain->flags &= (uint8_t)~TERRAIN_PENDING;
        }
        else if (terrain_confirm(p_terrain, now_ms, TERRAIN_ENTER_HOLD_MS))
        {
            p_terrain->flags = (uint8_t)((p_terrain->flags &
                                          (TERRAIN_VALID | TERRAIN_COMPLETED_EVENT)) |
                                         TERRAIN_CLIMBING | TERRAIN_SLOW);
            terrain_raise_peak(p_terrain, magnitude, right);
        }
        return;
    }
    if ((TERRAIN_CLIMBING == phase) && (magnitude > p_terrain->peak_tenths))
    {
        terrain_raise_peak(p_terrain, magnitude, right);
    }
    else if ((TERRAIN_DESCENDING == phase) &&
             ((int32_t)magnitude >= ((int32_t)p_terrain->peak_tenths + TERRAIN_RECLIMB_TENTHS)))
    {
        terrain_raise_peak(p_terrain, magnitude, right);
        phase = TERRAIN_CLIMBING;
    }
    else if ((TERRAIN_CLIMBING == phase) &&
             (((int32_t)magnitude + terrain_drop(p_terrain->peak_tenths)) <=
              (int32_t)p_terrain->peak_tenths))
    {
        phase = TERRAIN_DESCENDING;
    }
    p_terrain->flags = (uint8_t)((p_terrain->flags & (uint8_t)~TERRAIN_PHASE_MASK) | phase);
    if ((TERRAIN_DESCENDING != phase) || (magnitude >= (uint16_t)TERRAIN_EXIT_TENTHS))
    {
        p_terrain->flags &= (uint8_t)~TERRAIN_PENDING;
        return;
    }
    if (terrain_confirm(p_terrain, now_ms, TERRAIN_EXIT_HOLD_MS))
    {
        p_terrain->last_peak = (uint8_t)(terrain_mm(p_terrain->peak_tenths) |
                                         ((0U != (p_terrain->flags & TERRAIN_SIDE_RIGHT)) ?
                                          TERRAIN_PEAK_RIGHT : 0U));
        p_terrain->peak_tenths = 0U;
        p_terrain->flags = (uint8_t)(TERRAIN_WAITING | TERRAIN_VALID | TERRAIN_COMPLETED_EVENT);
    }
}
