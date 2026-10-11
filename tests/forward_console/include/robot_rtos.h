#ifndef FORWARD_TEST_RTOS_H
#define FORWARD_TEST_RTOS_H
#include <stdint.h>
typedef int32_t ER;
typedef int32_t INT;
typedef uint32_t UW;
typedef uint8_t UB;
#define E_OK 0
ER tk_dly_tsk(UW delay);
#endif
