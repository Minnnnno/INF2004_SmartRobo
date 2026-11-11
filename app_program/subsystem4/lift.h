/* Buddy 4 lift / overturn detection (Change Requirement: replaces IMU
 * collision detection). When the robot is lifted off the ground or
 * overturned, the owning task stops the motors; in the demonstration,
 * lifting the robot ends the run.
 *
 * Uses the raw accelerometer sample against the level reference captured at
 * boot (tilt_context_t up vector and gravity size). tilt_update() rejects
 * exactly these poses (> 75 deg, upside down, outside 700..1300 mg), so this
 * module computes its own values from the unfiltered sample.
 *
 * Portable: no kernel calls; one owning task; lift_reset() before use.
 */
#ifndef LIFT_H
#define LIFT_H
#include <stdbool.h>
#include <stdint.h>
#include "tilt.h"

/* Latched condition (2 bits). OVERTURNED wins over LIFTED. */
#define LIFT_NONE        (0U)
#define LIFT_LIFTED      (1U)
#define LIFT_OVERTURNED  (2U)

/* Thresholds (tune from robot logs: hand lifts, overturns and hump runs).
 * "Upright" = component of the sample along the reference up axis divided by
 * the sample size: 100 % level, 0 % on its side, negative upside down.
 */
#define LIFT_OVERTURN_UPRIGHT_PCT (50)   /* < 50 %: more than 60 deg from upright. */
#define LIFT_TILT_UPRIGHT_PCT     (82)   /* < 82 %: more than ~35 deg (lifted by one end).
                                            One-wheel humps tilt about 20 deg at most. */
#define LIFT_HOLD_MS              (200U) /* Tilt conditions must hold this long. */
#define LIFT_VERTICAL_MG          (200)  /* Upward acceleration above gravity, mg... */
#define LIFT_VERTICAL_SAMPLES     (4U)   /* ...on this many consecutive samples (~80 ms):
                                            hump landings and knocks are shorter. */
#define LIFT_CLEAR_UPRIGHT_PCT    (95)   /* Back down: >= 95 % upright (~18 deg)... */
#define LIFT_CLEAR_STILL_MG       (100)  /* ...sample size within 100 mg of gravity... */
#define LIFT_CLEAR_MS             (500U) /* ...held this long. */
#define LIFT_MIN_MAGNITUDE_MG     (300)  /* Smaller sample: near free fall, no direction. */
#define LIFT_MAX_GAP_MS           (100U) /* Longer input gaps restart the timers. */

typedef struct
{
    uint32_t over_since;  /* Start of the current overturned-pose interval. */
    uint32_t tilt_since;  /* Start of the current steep-tilt interval. */
    uint32_t clear_since; /* Start of the current upright-and-still interval. */
    uint32_t last_ms;     /* Time of the previous sample. */
    int16_t vertical_mg;  /* Last upward acceleration above gravity, mg. */
    int16_t trigger_mg;   /* vertical_mg when the latch was set (diagnostic). */
    int8_t upright_pct;   /* Last upright percentage, -100..100. */
    uint8_t vertical_count; /* Consecutive samples >= LIFT_VERTICAL_MG. */
    /* Bits 0..1 latched condition, bit 2 over timing, bit 3 tilt timing,
     * bit 4 clear timing, bit 5 last_ms valid. */
    uint8_t flags;
} lift_t;

void lift_reset(lift_t *p_lift);
/* One fresh accelerometer sample (mg). armed = false during the settle time
 * after entering MONITORING: values are computed, nothing is latched.
 * Returns the latched condition. Does nothing before the tilt reference is
 * captured. The latch clears only after LIFT_CLEAR_MS upright and still.
 */
uint8_t lift_step(lift_t *p_lift, tilt_context_t const *p_tilt,
                  int16_t x, int16_t y, int16_t z, uint32_t now_ms, bool armed);
uint8_t lift_state(lift_t const *p_lift);
/* True while latched or while a steep tilt (> ~35 deg) is being timed: hump
 * measurement is suspended meanwhile, so a robot lifted by one end does not
 * record a fake hump. Single vibration spikes never suspend it. */
bool lift_suspect(lift_t const *p_lift);
int16_t lift_vertical_mg(lift_t const *p_lift);
int16_t lift_trigger_mg(lift_t const *p_lift);
int8_t lift_upright_pct(lift_t const *p_lift);
char const *lift_name(uint8_t condition);
#endif
