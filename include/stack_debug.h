#ifndef ROBO_STACK_DEBUG_H
#define ROBO_STACK_DEBUG_H
#include <tk/tkernel.h>

typedef struct {
    UW used, capacity, saved_sp, state;
    BOOL guard_ok, saved_sp_ok;
} stack_debug_t;

/* Diagnostic snapshot of a public task's painted stack (single-core port).
 * E_NOEXS: no such task; E_SYS: corrupted stack metadata. No console I/O. */
ER knl_task_stack_debug(ID id, stack_debug_t *info);
#endif
