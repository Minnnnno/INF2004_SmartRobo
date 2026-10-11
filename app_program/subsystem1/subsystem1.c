/*
 *	subsystem1.c
 *	Subsystem 1 -- WiFi communication, command and telemetry: creates its
 *	objects and starts its task.  The work is in comms.c.
 */

#include "subsystem1.h"

EXPORT ER subsystem1_init(void)
{
	return comms_init();		/* telemetry, event and command queues */
}

EXPORT ER subsystem1_start(void)
{
	return comms_start();
}

/* Retain Buddy 2's auxiliary console task for isolated calibration builds. */
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
