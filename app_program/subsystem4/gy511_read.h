/* Fresh acceleration and magnetic acquisition; requires successful subsystem4_init(). */
#ifndef GY511_READ_H
#define GY511_READ_H

#include "subsystem4.h"
#include <stdint.h>

/* Three 12-bit two's-complement values, packed into 5 bytes (4 padding bits).
 * X: bits 0..11, Y: 12..23, Z: 24..35; least-significant byte first.
 * Owned by one task. Never publish this multi-byte object without protection.
 */
typedef struct
{
    uint8_t bytes[5];
} gy511_acceleration_t;

/* Task-context operation. E_OK supplies a fresh complete sample; E_BUSY means
 * no fresh XYZ set. Driver errors propagate. Output is unchanged on failure.
 * Opens/closes I2C1 per call; the application is the sole sensor owner.
 */
ER gy511_read_acceleration(gy511_acceleration_t *p_sample);

/* Decode axis 0=X, 1=Y, 2=Z into signed counts (nominally mg at +/-2 g HR).
 * E_PAR on NULL/invalid axis; otherwise E_OK. Output unchanged on error.
 * No calibration or physical sensor-accuracy correction is applied.
 */
ER gy511_acceleration_mg(gy511_acceleration_t const *p_sample,
                       uint8_t axis, int16_t *p_mg);

/* Magnetic counts use the same 36-bit XYZ packing, but different units.
 * Valid output range is -2048..2047 per axis (datasheet table 75).
 * Out-of-range sensor words are rejected, not truncated into plausible data.
 */
typedef struct
{
    uint8_t bytes[5];
} gy511_magnetic_t;
_Static_assert(sizeof(gy511_magnetic_t) == 5U, "Magnetic sample must occupy five bytes");

/* Fresh magnetic sample only. E_BUSY: no new data (or locked set drained).
 * E_IO: out-of-range output or incomplete transfer. Other driver errors propagate.
 * Output is unchanged unless all reads, validation and close succeed.
 */
ER gy511_read_magnetic(gy511_magnetic_t *p_sample);
/* Diagnostic categories describe the final outcome, including close failures. */
#define GY511_MAG_ERROR (0U)
#define GY511_MAG_FRESH (1U)
#define GY511_MAG_NOT_READY (2U)
#define GY511_MAG_LOCKED (3U)
/* Optional outcome pointer; preserves the ordinary reader's return contract. */
ER gy511_read_magnetic_diagnostic(gy511_magnetic_t *p_sample, uint8_t *p_outcome);
/* Axis 0=X, 1=Y, 2=Z; signed raw counts, not mg, gauss or degrees. */
ER gy511_magnetic_counts(gy511_magnetic_t const *p_sample,
                         uint8_t axis, int16_t *p_counts);

#endif /* GY511_READ_H */
