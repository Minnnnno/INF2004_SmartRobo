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
