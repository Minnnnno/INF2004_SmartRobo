#ifndef ROBO_MOTOR_TEST_H
#define ROBO_MOTOR_TEST_H
#include "motion.h"
typedef struct {
    unsigned wheel; /* 0=left, 1=right, 2=both */
    int direction;
    uint32_t delta[2], invalid_delta[2];
    bool timed_out;
} motor_test_result_t;
#ifdef ROBO_SAFE_MOTOR_TEST
bool motor_test_init(void);
bool motor_test_start(unsigned wheel, int direction);
bool motor_test_start_pair(int direction);
void motor_test_stop(void);
bool motor_test_result(motor_test_result_t *result);
bool motor_test_active(void);
#endif
#endif
