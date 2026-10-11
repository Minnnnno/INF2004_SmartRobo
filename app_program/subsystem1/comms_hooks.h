/*
 *	comms_hooks.h
 *	Subsystem 1 -- the link between the comms queues and the MQTT client.
 *
 *	Included by comms.c (which implements the hooks) and by
 *	lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c (which calls them from
 *	the kernel's WiFi service task).  Plain C types only: mqtt_utk.c must
 *	not see tk/tkernel.h.  Other subsystems use comms.h, not this file.
 */

#ifndef COMMS_HOOKS_H
#define COMMS_HOOKS_H

#define COMMS_TOPIC_STATUS	"robot/status"		/* retained: online/offline */
#define COMMS_TOPIC_HEARTBEAT	"robot/heartbeat"
#define COMMS_TOPIC_TELEMETRY	"robot/telemetry/"	/* + name */
#define COMMS_TOPIC_EVENT	"robot/event"
#define COMMS_TOPIC_CMD		"robot/cmd"
#define COMMS_TOPIC_ACK		"robot/cmd/ack"

#define COMMS_NAME_MAX		16	/* telemetry name, including the '\0' */
#define COMMS_TOPIC_MAX		32	/* full topic, including the '\0' */
#define COMMS_PAYLOAD_MAX	96	/* payload, including the '\0' */

/* Next queued message, events before telemetry.  Never waits.  topic must
   hold COMMS_TOPIC_MAX bytes and payload COMMS_PAYLOAD_MAX bytes.
   Returns the payload length, or -1 if nothing is queued. */
int comms_hook_next(char *topic, char *payload, int *qos);

/* A message arrived on COMMS_TOPIC_CMD.  Valid commands are passed on to
   the Vehicle Controller; either way an acknowledgement is queued. */
void comms_hook_command(const char *payload, int len);

/* The MQTT connection came up (1) or went down (0). */
void comms_hook_online(int online);

#endif /* COMMS_HOOKS_H */
