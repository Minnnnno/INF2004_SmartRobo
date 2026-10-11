#include "subsystems.h"
/* Buddy 3: two independent task entry points; no fabricated sensor readings. */
void line_task(INT code, void *context)
{ (void)code; (void)context; for (;;) { (void)tk_slp_tsk(TMO_FEVR); } }
void barcode_task(INT code, void *context)
{ (void)code; (void)context; for (;;) { (void)tk_slp_tsk(TMO_FEVR); } }
