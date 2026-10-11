#include "subsystems.h"
/* Buddy 1: replace this inactive owner with genuine WiFi/MQTT processing. */
void wifi_task(INT code, void *context)
{
    (void)code; (void)context;
#ifdef ROBO_MANUAL_CONSOLE
    extern void manual_console(void);
    manual_console();
#else
    for (;;) { (void)tk_slp_tsk(TMO_FEVR); }
#endif
}
