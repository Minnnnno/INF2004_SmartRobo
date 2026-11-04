#ifndef TM_TEST_KERNEL_H
#define TM_TEST_KERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef int32_t INT;
typedef int32_t W;
typedef uint32_t UINT;
typedef uint32_t UW;
typedef int16_t H;
typedef uint16_t UH;
typedef uint8_t UB;
typedef int BOOL;
#define TRUE 1
#define FALSE 0
#define EXPORT
#define IMPORT extern
#define LOCAL static
#define USE_TMONITOR 1
#define TK_SUPPORT_SMP 0
#endif
