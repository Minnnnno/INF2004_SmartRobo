/** @file motion_hal.h
 *
 * @brief Motor and encoder driver interface.
 */

#ifndef ROBO_MOTION_HAL_H
#define ROBO_MOTION_HAL_H
#include "float_types.h"
#include "motion.h"
bool             motion_hal_init(const motion_config_t * p_config);
encoder_sample_t motion_hal_sample(void);

typedef struct
{
    encoder_sample_t sample;
    unsigned         pins[2][2];

    uint32_t levels;
    uint32_t edges[2][2];
    uint8_t  previous_ab[2];
    uint8_t  irq_events[2][2];
    uint8_t  function[2][2];
    bool     initialised;
    bool     irq_registered;
    bool     nvic_enabled;
    bool     motors_inhibited;
} encoder_diagnostics_t;

void motion_hal_encoder_diagnostics(encoder_diagnostics_t * p_diagnostics);

/* Software/PWM-register evidence, not motor voltage or current measurement.
 * Motion-task context: capture before a fault write clears the outputs. */
typedef struct
{
    int32_t  requested_permille[2];
    int32_t  applied_permille[2];
    int32_t  cap_permille;
    uint16_t compare[2][2];
    uint16_t expected_compare[2][2];
    uint16_t top[2];
    uint16_t divider_16[2];
    uint16_t configured_top;
    uint16_t configured_divider_16;
    uint8_t  pin_function[2][2];
    uint8_t  enabled_mask;
    uint8_t  reversal_wait_mask;
    uint8_t  mismatch_mask;
    bool     ready;
    bool     inhibited;
    uint32_t output_age_ms;
} motor_diagnostics_t;

void      motion_hal_motor_diagnostics(motor_diagnostics_t * p_diagnostics);
void      motion_hal_write(const float64_t output[2], bool b_inhibited,
                           float64_t now);
void      motion_hal_emergency_stop(void);
float64_t motion_hal_time(void);
void      motion_hal_console_init(void);
int       motion_hal_console_read(void);
#endif

/*** end of file ***/
