#include "robot_rtos.h"
#include "subsystems.h"

/* Disabled owners sleep indefinitely. Replace each entry with the real subsystem.
 * Submit typed requests through robot_submit; never access DC motor GPIO/PWM. */
static void inactive(void)
{
    for (;;) { (void)tk_slp_tsk(TMO_FEVR); }
}
void line_task(INT code, void *context) { (void)code; (void)context; inactive(); }
void barcode_task(INT code, void *context) { (void)code; (void)context; inactive(); }
void terrain_task(INT code, void *context) { (void)code; (void)context; inactive(); }
void obstacle_task(INT code, void *context) { (void)code; (void)context; inactive(); }
void wifi_task(INT code, void *context)
{
    (void)code; (void)context;
#ifdef ROBO_MANUAL_CONSOLE
    extern void manual_console(void);
    manual_console();
#else
    inactive();
#endif
}
