#ifndef ROBO_MOTION_HAL_H
#define ROBO_MOTION_HAL_H
#include "motion.h"
bool motion_hal_init(const motion_config_t *config);
encoder_sample_t motion_hal_sample(void);
typedef struct {
    encoder_sample_t sample;
    unsigned pins[2][2];
    uint32_t levels, edges[2][2];
    uint8_t previous_ab[2], irq_events[2][2], function[2][2];
    bool initialised, irq_registered, nvic_enabled, motors_inhibited;
} encoder_diagnostics_t;
void motion_hal_encoder_diagnostics(encoder_diagnostics_t *diagnostics);
/* Software/PWM-register evidence, not motor voltage or current measurement.
 * Motion-task context: capture before a fault write clears the outputs. */
typedef struct {
    int32_t requested_permille[2], applied_permille[2], cap_permille;
    uint16_t compare[2][2], expected_compare[2][2];
    uint16_t top[2], divider_16[2], configured_top, configured_divider_16;
    uint8_t pin_function[2][2], enabled_mask, reversal_wait_mask, mismatch_mask;
    bool ready, inhibited;
    uint32_t output_age_ms;
} motor_diagnostics_t;
void motion_hal_motor_diagnostics(motor_diagnostics_t *diagnostics);
void motion_hal_write(const double output[2], bool inhibited, double now);
void motion_hal_emergency_stop(void);
double motion_hal_time(void);
void motion_hal_console_init(void);
int motion_hal_console_read(void);
#endif
