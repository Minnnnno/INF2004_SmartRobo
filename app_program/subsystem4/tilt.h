/* Bench orientation relative to a stationary, known-level startup reference. */
#ifndef TILT_H
#define TILT_H
#include <stdbool.h>
#include <stdint.h>

/* Project alias for the BARR-C single-precision naming convention. */
typedef float float32_t;
_Static_assert(sizeof(float32_t) == 4U, "32-bit floating point required");

/* Floating-point working precision is retained for reference rotation and
 * filtering. Packed output is quantised to 0.1 degree, not claimed accuracy.
 * No dynamic allocation or task sharing; zero-initialise once at startup.
 */
typedef struct
{
    float32_t sum[3];
    float32_t anchor[3];
    float32_t up[3];
    float32_t forward[3];
    float32_t side[3];
    float32_t filtered[3];
    float32_t gravity;  /* Magnitude of the captured level reference, mg (~1000). */
    /* Bits 0..5 capture count (0..50), bit 6 calibrated, bit 7 filter ready. */
    uint8_t progress;
    /* Pitch condition bits 0..1, roll bits 2..3: 0 level, 1 positive, 2 negative. */
    uint8_t conditions;
} tilt_context_t;

/* Packed result: pitch offset +900 in bits 0..10; roll offset +900 in 11..21;
 * pitch condition in 22..23, roll condition in 24..25, valid in bit 26.
 * 27 useful bits; remaining bits zero. Only +/-75 deg is supported this stage.
 */
#define TILT_RESULT_VALID (1UL << 26U)
#define TILT_CALIBRATED (0x40U)
#define TILT_COUNT_MASK (0x3FU)
#define TILT_CAPTURE_SAMPLES (50U)

/* Collect 50 fresh, stable samples; restart on motion/invalid acceleration.
 * Returns true when a reference is ready. Stability cannot prove levelness:
 * the user MUST place the board on known level ground. No flash persistence.
 */
bool tilt_capture(tilt_context_t *p_context, int16_t x, int16_t y, int16_t z);
/* Compute a filtered result from one fresh sample. Zero means unavailable.
 * Invalid input resets filter/conditions, but preserves the level reference.
 */
uint32_t tilt_update(tilt_context_t *p_context, int16_t x, int16_t y, int16_t z);
/* Discard pending capture/filter history after a long gap, preserve calibration. */
void tilt_discard_history(tilt_context_t *p_context);
/* Right wheel contact height above the left, 0.1 mm, for an axle of track_mm
 * (track_mm <= 3276). Independent of pitch; positive = right wheel higher.
 * Zero before calibration. Call only after tilt_update() returned a valid result.
 */
int16_t tilt_side_height_tenths(tilt_context_t const *p_context, uint16_t track_mm);
/* Forward dynamic acceleration, mg (+ = speeding up forward): the fresh sample
 * minus the smoothed gravity vector, projected on the reference forward axis.
 * A high-passed diagnostic for comparing with encoder acceleration; it is not
 * used for classification. Zero before calibration/filtering. Call only after
 * tilt_update() returned a valid result for the same x, y, z.
 */
int16_t tilt_forward_dynamic_mg(tilt_context_t const *p_context, int16_t x, int16_t y, int16_t z);
/* Decode pitch (axis 0) or roll (axis 1); caller requires a valid packed result. */
int16_t tilt_angle_tenths(uint32_t result, uint8_t axis);
/* Return display labels for axis 0=pitch or 1=roll; invalid result -> UNAVAILABLE. */
char const *tilt_condition_name(uint32_t result, uint8_t axis);
#endif
