/** @file system_runtime.h
 *
 * @brief Watchdog, crash handling and supervision; see system_runtime.c.
 */

#ifndef ROBO_SYSTEM_RUNTIME_H
#define ROBO_SYSTEM_RUNTIME_H
#include "../../include/float_types.h"
#include <stdbool.h>
#include "robot_rtos.h"

typedef struct
{
    FP           entry;
    PRI          priority;
    SZ           stack;
    const char * name;
} app_task_spec_t;

/*! @brief Startup and diagnostics execute in usermain's existing task. */
void system_boot_init(void);
void system_report_startup(void);
void system_report_previous_crash(void);
bool system_encoder_fault_latched(void);
UW   system_encoder_fault_cause(void);
void system_supervise(const app_task_spec_t * p_tasks, const ID * p_ids,
                      unsigned task_count, void (*p_report_motion_fault)(void));

/*! @brief Motion owner calls these at exactly the existing start/feed points.
 */
void system_watchdog_start(void);
void system_watchdog_feed(float64_t now);
#endif

/*** end of file ***/
