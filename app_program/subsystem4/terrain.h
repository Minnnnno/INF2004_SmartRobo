/* Hump phases and peak height, nested inside Buddy 4 MONITORING.
 * Course model: 2 driven wheels + centre caster, each hump under ONE wheel
 * side. The input is the right-minus-left wheel contact height from the
 * sideways component of gravity (tilt_side_height_tenths), so body pitch and
 * the caster do not affect it. A hump under both wheels at once cancels out.
 */
#ifndef TERRAIN_H
#define TERRAIN_H
#include <stdbool.h>
#include <stdint.h>
#define TERRAIN_WAITING (0U)
#define TERRAIN_CLIMBING (1U)
#define TERRAIN_DESCENDING (2U)
#define TERRAIN_PHASE_MASK (0x03U)
#define TERRAIN_VALID (0x04U)
#define TERRAIN_SLOW (0x08U)
/* Height thresholds in 0.1 mm and confirmation times. */
/* Driving on flat floor sways the reading by up to ~3.3 mm (steering, vibration),
 * so the entry level sits above that; the smallest course hump is ~10 mm. */
#define TERRAIN_ENTER_TENTHS (50)  /* 5.0 mm wheel height difference starts a hump ... */
#define TERRAIN_ENTER_HOLD_MS (100U) /* ... once held this long (rejects jolts). */
#define TERRAIN_DROP_TENTHS (30)   /* Descending: 3.0 mm below the peak, or half */
                                   /* the peak if that is smaller (small humps). */
#define TERRAIN_RECLIMB_TENTHS (20) /* DESCENDING -> CLIMBING only 2.0 mm above the peak. */
#define TERRAIN_SMOOTH_SHIFT (2U)  /* Height smoothing: s += (h - s) / 4 per sample. */
#define TERRAIN_EXIT_TENTHS (25)   /* Below 2.5 mm (under the entry level) ... */
#define TERRAIN_EXIT_HOLD_MS (300U) /* ... for this long completes the hump. */
#define TERRAIN_MAX_GAP_MS (100U)
/* Peak bytes (CR1: 0..127 mm in 7 bits + one flag bit). */
#define TERRAIN_PEAK_MM_MASK (0x7FU)
#define TERRAIN_PEAK_RIGHT (0x80U) /* terrain_last_peak(): hump was under the right wheel. */
#define TERRAIN_RUN_VALID (0x80U)  /* terrain_run_max(): at least one hump confirmed. */
/* Status byte: phase bits 0..1, current observation valid bit 2, reduced-speed
 * request bit 3. Private: bit 4 confirmation pending, bit 5 current hump side,
 * bit 7 completion event latched until consumed by the owning task.
 * Invalid status is UNKNOWN, never permission to drive at normal speed.
 */
typedef struct
{
    uint32_t candidate_since;
    uint32_t last_sample;
    int16_t smooth_tenths; /* Smoothed signed height, 0.1 mm; reseeded after gaps. */
    uint16_t peak_tenths; /* Current hump peak |height|, 0.1 mm (0..3276). */
    uint8_t flags;
    uint8_t last_peak;    /* Last completed hump: mm | TERRAIN_PEAK_RIGHT. */
    uint8_t run_max;      /* Highest peak this run: mm | TERRAIN_RUN_VALID. */
} terrain_t;
/* Explicit new-run reset: clears phases, peaks and the run maximum.
 * Cold initialisation is also a zero-filled context. Completion automatically rearms.
 */
void terrain_reset(terrain_t *p_terrain);
/* Consume a latched completion once, from the owning task. Consume after each
 * update; multiple events coalesce if ignored. Not an inter-task event queue.
 */
bool terrain_take_completed(terrain_t *p_terrain);
/* One fresh observation per call: valid=false (stale/unsuitable tilt) cancels
 * pending timing and keeps the confirmed phase, peaks and slow request.
 * height_tenths: right-minus-left wheel contact height, 0.1 mm (sign = side).
 * The run maximum follows the live peak, so an up-only ramp still counts.
 */
void terrain_step(terrain_t *p_terrain, bool valid, int16_t height_tenths, uint32_t now_ms);
/* Coherent four-bit status value; copy through a protected interface across tasks.
 * Internal flags/padding must not be transmitted directly. NULL gives UNKNOWN.
 */
uint8_t terrain_status(terrain_t const *p_terrain);
/* Last completed hump peak byte (mm | TERRAIN_PEAK_RIGHT); 0 before the first. */
uint8_t terrain_last_peak(terrain_t const *p_terrain);
/* Highest peak this run (mm | TERRAIN_RUN_VALID); 0 before the first hump. */
uint8_t terrain_run_max(terrain_t const *p_terrain);
char const *terrain_phase_name(uint8_t status);
#endif
