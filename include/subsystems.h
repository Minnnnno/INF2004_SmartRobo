#ifndef ROBO_SUBSYSTEMS_H
#define ROBO_SUBSYSTEMS_H
#include "robot_rtos.h"
void line_task(INT code, void *context);
#ifdef ROBO_SAFE_MOTOR_TEST
void line_forward_pause(void); /* Serial stop blocks restart until sensor condition clears. */
#endif
void barcode_task(INT code, void *context);
void terrain_task(INT code, void *context);
void obstacle_task(INT code, void *context);
void wifi_task(INT code, void *context);
#endif
