/* Shared task-creation helper used by Buddy 1. */
#include "robot_rtos.h"
#include "../app.h"

ID
app_start_task (FP p_entry, PRI priority, UW processor, const char * p_name)
{
    T_CTSK task = {.tskatr  = TA_HLNG | TA_RNG0,
                   .task    = p_entry,
                   .itskpri = priority,
                   .stksz   = TASK_STACK_SIZE};
    ID     task_id;
    ER     result;

    (void)processor;
    (void)p_name; /* This integration requires SMP=OFF. */
    task_id = tk_cre_tsk(&task);

    if (task_id <= 0)
    {
        return task_id;
    }

    result = tk_sta_tsk(task_id, 0);

    if (result < E_OK)
    {
        (void)tk_del_tsk(task_id);
        return result;
    }

    return task_id;
}

/*** end of file ***/
