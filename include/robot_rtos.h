/** @file robot_rtos.h
 *
 * @brief Kernel headers in an order that works with the C library.
 */

#ifndef ROBO_RTOS_H
#define ROBO_RTOS_H
/* BSP provides this switch for applications using the standard C size_t. */
#include <stddef.h>
#define PROHIBIT_DEF_SIZE_T
#include <tk/tkernel.h>
#if TK_SUPPORT_SMP
#error "Motion integration currently requires single-core SMP=OFF."
#endif
#endif

/*** end of file ***/
