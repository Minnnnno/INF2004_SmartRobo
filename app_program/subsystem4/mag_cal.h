/* Buddy 4 magnetometer calibration constants.
 * Produced by the self-pivot calibration build (-DROBO_MAG_CAL=ON, see
 * app_program/subsystem4/README.md): paste the printed #define lines here.
 * Valid while the GY-511 mounting, wiring and nearby metal stay the same.
 * While B4_MAG_CALIBRATED is 0 the compass reports "n/a (uncal)"; nothing
 * else changes.
 */
#ifndef MAG_CAL_H
#define MAG_CAL_H

/* Self-pivot calibration, 2026-11-06: PASS, coverage 36/36, fit error 0.9 %. */
#define B4_MAG_CALIBRATED   (1)
#define B4_MAG_OFFSET_X     (-68)
#define B4_MAG_OFFSET_Y     (-152)
#define B4_MAG_SCALE_Y_Q12  (4028)
#define B4_MAG_FIELD        (348)
#define B4_MAG_HEADING_SIGN (1)

#endif
