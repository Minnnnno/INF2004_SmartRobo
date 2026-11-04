#ifndef ROBO_SUBSYSTEMS_H
#define ROBO_SUBSYSTEMS_H
#include "robot_rtos.h"

/* Subsystem 3 tasks exist only in images where they do something; elsewhere
 * usermain does not create them, so they cost no stack. */
#if defined(ROBO_SAFE_MOTOR_TEST) || defined(ROBO_LINE_STEERING)
#define ROBO_LINE_TASK 1
#else
#define ROBO_LINE_TASK 0
#endif
#if defined(ROBO_BARCODE_TEST) && defined(ROBO_PRODUCTION_BUILD) && !defined(ROBO_LINE_STEERING)
#define ROBO_BARCODE_TASK 1 /* Opt-in hand-scan test, see barcode.c. */
#else
#define ROBO_BARCODE_TASK 0
#endif

#if ROBO_LINE_TASK
void line_task(INT code, void *context);
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
void line_forward_pause(void); /* Serial stop blocks restart until sensor condition clears. */
#endif
#if ROBO_BARCODE_TASK
void barcode_task(INT code, void *context);
#endif
void terrain_task(INT code, void *context);
void obstacle_task(INT code, void *context);
void wifi_task(INT code, void *context);
#endif
