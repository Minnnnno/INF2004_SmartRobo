/* Application composition: initialize components, create/start tasks, supervise. */
#include "robot_rtos.h"
#include "app.h"
#include "robot.h"
#include "subsystems.h"
#include "subsystem2/motion_runtime.h"
#include "controller/robot_runtime.h"
#include "system/system_runtime.h"

INT usermain(void)
{
    /* Interrupts run on the interrupted task's stack (everything is on MSP).
     * The detour call chain needs over 4 KiB before nested interrupt frames. */
    static const app_task_spec_t tasks[] = {
        {(FP)motion_task, 2, 4096, "motion"}, {(FP)controller_task, 4, 4096, "controller"},
#if ROBO_SUBSYSTEM5
        {(FP)obstacle_task, 4, 8192, "obstacle"},
#endif
#if ROBO_LINE_TASK
        {(FP)line_task, 8, 4096, "line"},
#endif
#if ROBO_BARCODE_TASK
        {(FP)barcode_task, 8, 4096, "barcode"},
#endif
#if ROBO_SHUTTLE_TASK
        {(FP)shuttle_task, 8, 4096, "shuttle"},
#endif
#if ROBO_SUBSYSTEM4
        {(FP)terrain_task, 8, 4096, "terrain"},
#endif
        {(FP)wifi_task, 12, 4096, "wifi"}
    };
    ID ids[sizeof(tasks) / sizeof(tasks[0])];
    unsigned i;

    system_boot_init();
    motion_runtime_init();
    robot_runtime_prepare(motion_runtime_config(), motion_runtime_initial_status());
    motion_runtime_report_startup();
    system_report_startup();

    ID mutex_result = robot_runtime_init();
    if (mutex_result <= 0) {
        motion_runtime_emergency_stop();
        tm_printf((const UB *)"[INIT ERROR] status mutex creation failed: %d\n", (int)mutex_result);
        return 0;
    }
    ER init_result = subsystem1_init();
    if (init_result < E_OK) {
        motion_runtime_emergency_stop();
        tm_printf((const UB *)"[INIT ERROR] communications init failed: %d\n", (int)init_result);
        return 0;
    }
    for (i = 0; i < sizeof(tasks) / sizeof(tasks[0]); ++i) {
        T_CTSK task = {.tskatr = TA_HLNG | TA_RNG0, .task = tasks[i].entry,
                      .itskpri = tasks[i].priority, .stksz = tasks[i].stack};
        ids[i] = tk_cre_tsk(&task);
        if (ids[i] <= 0) {
            motion_runtime_emergency_stop();
            tm_printf((const UB *)"[INIT ERROR] task=%s stack=%u creation failed: %d\n",
                      tasks[i].name, (unsigned)tasks[i].stack, (int)ids[i]);
            return 0;
        }
    }
    robot_runtime_enable();
    init_result = subsystem1_start();
    if (init_result < E_OK) {
        motion_runtime_emergency_stop();
        tm_printf((const UB *)"[INIT ERROR] communications start failed: %d\n", (int)init_result);
        return 0;
    }
    for (i = 0; i < sizeof(tasks) / sizeof(tasks[0]); ++i) {
        init_result = tk_sta_tsk(ids[i], 0);
        if (init_result < E_OK) {
            motion_request_shutdown();
            motion_runtime_emergency_stop();
            tm_printf((const UB *)"[INIT ERROR] task=%s start failed: %d\n", tasks[i].name, (int)init_result);
            return 0;
        }
    }
    /* Startup needs priority 1; periodic diagnostics must yield to motion/USB. */
    if (tk_chg_pri(TSK_SELF, 16) != E_OK) {
        motion_runtime_emergency_stop();
        motion_request_shutdown();
        tm_printf((const UB *)"[INIT ERROR] could not lower diagnostics task priority\n");
    }
    for (;;) {
        (void)tk_dly_tsk(2000U);
        system_supervise(tasks, ids, sizeof(tasks) / sizeof(tasks[0]), motion_runtime_report_fault);
    }
}
