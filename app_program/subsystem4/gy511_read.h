/* Raw LSM303DLHC readings for the background monitor; no terrain processing. */
#ifndef GY511_READ_H
#define GY511_READ_H

#include "subsystem4.h"
#include <stdint.h>

typedef struct
{
    int16_t acceleration_x;
    int16_t acceleration_y;
    int16_t acceleration_z;
    int16_t magnetic_x;
    int16_t magnetic_y;
    int16_t magnetic_z;
} gy511_sample_t;

/* Call after configuration, from the sole I2C1-owning task.
 * Acceleration: signed 12-bit counts (nominally 1 mg/count at +/-2 g).
 * Magnetic data: signed register counts, not calibrated field units.
 * E_BUSY means fresh data is not ready. Output is unchanged on any error.
 */
ER gy511_read_sample(gy511_sample_t *p_sample);

#endif /* GY511_READ_H */
