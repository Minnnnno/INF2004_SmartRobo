/** @file robot_runtime.h
 *
 * @brief Shared request bus for tasks; see robot_runtime.c.
 */

#ifndef ROBO_ROBOT_RUNTIME_H
#define ROBO_ROBOT_RUNTIME_H
#include "robot_rtos.h"
#include "../../include/float_types.h"
#include "robot.h"

/*! @brief Startup: prepare the bus, create its mutex, then enable submissions
 * immediately before communications/tasks start. Configuration belongs to the
 * motion owner. */
void robot_runtime_prepare(const motion_config_t * p_config,
                           const motion_status_t * p_initial_status);
ID   robot_runtime_init(void);
void robot_runtime_enable(void);
void controller_task(INT code, void * p_context);

/*! @brief Internal motion-task bridge. Inputs/publication/stop checks require
 * this lock. The bus and its mutex remain private to robot_runtime.c. */
bool robot_runtime_lock(void);
bool robot_runtime_wait(void);
void robot_runtime_unlock(void);
void robot_runtime_motion_inputs(motion_t * p_owner, encoder_sample_t sample,
                                 float64_t now);
void robot_runtime_publish(motion_t * p_owner, float64_t now);
bool robot_runtime_stop_pending(void);
#endif

/*** end of file ***/
