/* Buddy 4 compass: 2D magnetometer calibration from a flat self-pivot, and a
 * calibrated heading used only to cross-check the encoder turn angle (wheel
 * slip). The robot rotates about its vertical axis on a level floor, so the
 * sensor's X/Y readings trace an ellipse; small mounting tilt is absorbed.
 * Only heading CHANGES are used, so true north and declination are irrelevant.
 * Portable: no kernel calls. One owning task.
 */
#ifndef MAG_HEADING_H
#define MAG_HEADING_H
#include <stdbool.h>
#include <stdint.h>

#define MAG_BINS              (36U)  /* Coverage bins of 10 deg of encoder heading. */
#define MAG_BIN_DDEG          (100)
#define MAG_FIELD_TOL_PERCENT (20)   /* Valid when |field - reference| <= 20 %. */
#define MAG_FIT_MAX_PERMILLE  (40U)  /* Circle-fit RMS error limit, 4 % (clean spin ~1 %). */
#define MAG_MIN_HALF_RANGE    (20)   /* Counts; a dead or flat axis fails. */
#define MAG_SCALE_ONE_Q12     (4096U)

/* Calibration constants (mag_cal.h) or a calibration result. */
typedef struct
{
    int16_t offset_x;      /* Hard-iron offset, raw counts. */
    int16_t offset_y;
    uint16_t scale_y_q12;  /* Y scale so the ellipse becomes a circle (X = 1). */
    uint16_t field;        /* Reference horizontal field after correction, counts. */
    int8_t sign;           /* +1/-1 so that + heading change = left (encoder sign). */
    bool calibrated;
} mag_cal_t;

/* Accumulator for one calibration spin: min/max per axis and one sample per
 * 10-degree bin of encoder heading (for coverage and the circle-fit check). */
typedef struct
{
    int16_t min[2];
    int16_t max[2];
    int16_t bin_x[MAG_BINS];
    int16_t bin_y[MAG_BINS];
    uint64_t bins;         /* Bit i set = bin i holds a sample (36 bits used). */
    uint16_t samples;
} mag_cal_acc_t;

#define MAG_CAL_PASS          (0U)
#define MAG_CAL_FAIL_SAMPLES  (1U)
#define MAG_CAL_FAIL_COVERAGE (2U)
#define MAG_CAL_FAIL_RANGE    (3U)
#define MAG_CAL_FAIL_FIT      (4U)

typedef struct
{
    mag_cal_t cal;
    uint8_t bins_covered;
    uint16_t fit_permille;
} mag_cal_result_t;

void mag_cal_begin(mag_cal_acc_t *p_acc);
/* rel_heading_ddeg: encoder heading since the spin started, 0.1 deg, any sign. */
void mag_cal_add(mag_cal_acc_t *p_acc, int16_t x, int16_t y, int32_t rel_heading_ddeg);
/* Fills *p_result (also on FAIL, for printing) and returns MAG_CAL_*. */
uint8_t mag_cal_finish(mag_cal_acc_t const *p_acc, mag_cal_result_t *p_result);
char const *mag_cal_status_name(uint8_t status);

/* Horizontal field magnitude, counts: corrected when calibrated, raw otherwise. */
uint16_t mag_field(mag_cal_t const *p_cal, int16_t x, int16_t y);
/* Calibrated heading 0..3599 (0.1 deg, + = left). False when uncalibrated or
 * the field is outside +/-20 % of the reference (steel nearby). */
bool mag_heading_ddeg(mag_cal_t const *p_cal, int16_t x, int16_t y, int16_t *p_heading_ddeg);
/* Continue an unwrapped heading with a new 0..3599 reading (step < 180 deg). */
int32_t mag_unwrap(int32_t previous_ddeg, int16_t heading_ddeg);
#endif
