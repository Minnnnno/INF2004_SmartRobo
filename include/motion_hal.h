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
void motion_hal_write(const double output[2], bool inhibited, double now);
void motion_hal_emergency_stop(void);
double motion_hal_time(void);
void motion_hal_console_init(void);
int motion_hal_console_read(void);
#endif
