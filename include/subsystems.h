#ifndef ROBO_SUBSYSTEMS_H
#define ROBO_SUBSYSTEMS_H
#include "robot_rtos.h"

/* Subsystem 3 tasks exist only in images where they do something; elsewhere
 * usermain does not create them, so they cost no stack. */
/* The barcode shuttle test image (line steering profile plus -DROBO_BARCODE_SHUTTLE=1) drives
 * the car forward and back with shuttle_task instead of following the line. */
#if defined(ROBO_BARCODE_SHUTTLE) && defined(ROBO_LINE_STEERING) && !defined(ROBO_SAFE_MOTOR_TEST) && \
    !defined(ROBO_IR_MONITOR) && !defined(ROBO_MANUAL_CONSOLE)
#define ROBO_SHUTTLE_TASK 1
#else
#define ROBO_SHUTTLE_TASK 0
#endif
#if (defined(ROBO_SAFE_MOTOR_TEST) || defined(ROBO_LINE_STEERING)) && !ROBO_SHUTTLE_TASK
#define ROBO_LINE_TASK 1
#else
#define ROBO_LINE_TASK 0
#endif
/* The reader runs alongside line following, and alone in the opt-in barcode test image.
 * The IR monitor image reads GP28 as a digital input, so it keeps the task off. */
#if defined(ROBO_PRODUCTION_BUILD) && !defined(ROBO_IR_MONITOR) && \
    (defined(ROBO_LINE_STEERING) || defined(ROBO_BARCODE_TEST))
#define ROBO_BARCODE_TASK 1 /* See barcode.c. */
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
#if ROBO_SHUTTLE_TASK
void shuttle_task(INT code, void *context);
#endif
void terrain_task(INT code, void *context);
void obstacle_task(INT code, void *context);
void wifi_task(INT code, void *context);
#endif
