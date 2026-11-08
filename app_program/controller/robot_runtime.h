#ifndef ROBO_ROBOT_RUNTIME_H
#define ROBO_ROBOT_RUNTIME_H
#include "robot_rtos.h"
#include "robot.h"

/* Startup: prepare the bus, create its mutex, then enable submissions immediately
 * before communications/tasks start. Configuration belongs to the motion owner. */
void robot_runtime_prepare(const motion_config_t *config, const motion_status_t *initial_status);
ID robot_runtime_init(void);
void robot_runtime_enable(void);
void controller_task(INT code, void *context);

/* Internal motion-task bridge. Inputs/publication/stop checks require this lock.
 * The bus and its mutex remain private to robot_runtime.c. */
bool robot_runtime_lock(void);
bool robot_runtime_wait(void);
void robot_runtime_unlock(void);
void robot_runtime_motion_inputs(motion_t *owner, encoder_sample_t sample, double now);
void robot_runtime_publish(motion_t *owner, double now);
bool robot_runtime_stop_pending(void);
#endif
