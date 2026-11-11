/* Host declarations used by the lifecycle and terrain-task tests. */
#ifndef HOST_TKERNEL_H
#define HOST_TKERNEL_H
#include "device.h"
typedef int32_t TMO;
typedef int32_t BOOL;
typedef struct { int32_t hi; uint32_t lo; } SYSTIM;
ER tk_get_otm(SYSTIM *time);
ER tk_dly_tsk(TMO delay);
#endif
