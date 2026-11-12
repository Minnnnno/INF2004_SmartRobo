/* Shared task-creation helper used by Buddy 1. */
#include "robot_rtos.h"
#include "../app.h"

ID app_start_task(FP entry, PRI priority, UW processor, const char *name)
{
    T_CTSK task = {.tskatr = TA_HLNG | TA_RNG0, .task = entry,
                  .itskpri = priority, .stksz = TASK_STACK_SIZE};
    ID id;
    ER result;
    (void)processor; (void)name; /* This integration requires SMP=OFF. */
    id = tk_cre_tsk(&task);
    if (id <= 0) { return id; }
    result = tk_sta_tsk(id, 0);
    if (result < E_OK) { (void)tk_del_tsk(id); return result; }
    return id;
}
