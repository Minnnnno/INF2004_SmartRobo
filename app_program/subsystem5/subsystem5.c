#include "subsystems.h"
void obstacle_task(INT code, void *context)
{ (void)code; (void)context; for (;;) { (void)tk_slp_tsk(TMO_FEVR); } }
