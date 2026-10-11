/* GY-511 configuration; terrain_task is started by usermain at priority 8. */
#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H
#include "robot_rtos.h"
#define S4_GY511_I2C_DEVICE "iicb"
#define S4_GY511_ACC_ADDRESS (0x19U)
#define S4_GY511_MAG_ADDRESS (0x1EU)
/* Task context only. Configure once, then sample from the sole bus owner. */
ER subsystem4_gy511_configure(void);
void terrain_task(INT code, void *p_context);
#endif
