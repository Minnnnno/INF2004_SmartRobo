#ifndef ROBO_SUBSYSTEMS_H
#define ROBO_SUBSYSTEMS_H
#include "robot_rtos.h"
void line_task(INT code, void *context);
void barcode_task(INT code, void *context);
void terrain_task(INT code, void *context);
void obstacle_task(INT code, void *context);
void wifi_task(INT code, void *context);
#endif
