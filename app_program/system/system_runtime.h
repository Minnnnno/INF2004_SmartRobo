#ifndef ROBO_SYSTEM_RUNTIME_H
#define ROBO_SYSTEM_RUNTIME_H
#include <stdbool.h>
#include "robot_rtos.h"

typedef struct { FP entry; PRI priority; SZ stack; const char *name; } app_task_spec_t;

/* Startup and diagnostics execute in usermain's existing task. */
void system_boot_init(void);
void system_report_startup(void);
void system_report_previous_crash(void);
bool system_encoder_fault_latched(void);
UW system_encoder_fault_cause(void);
void system_supervise(const app_task_spec_t *tasks, const ID *ids, unsigned task_count,
                      void (*report_motion_fault)(void));

/* Motion owner calls these at exactly the existing start/feed points. */
void system_watchdog_start(void);
void system_watchdog_feed(double now);
#endif
