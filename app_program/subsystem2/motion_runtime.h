#ifndef ROBO_MOTION_RUNTIME_H
#define ROBO_MOTION_RUNTIME_H
#include "robot_rtos.h"
#include "motion.h"

/* Production Buddy 2 entry points; legacy producer examples are excluded. */
void motion_runtime_init(void);
const motion_config_t *motion_runtime_config(void);
const motion_status_t *motion_runtime_initial_status(void);
void motion_runtime_emergency_stop(void);
void motion_task(INT code, void *context);
/* Called by startup/the diagnostics task, never the motion loop or encoder ISR. */
void motion_runtime_report_startup(void);
void motion_runtime_report_fault(void);
#endif
