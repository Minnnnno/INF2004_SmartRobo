/*
 *	comms_hooks.h
 *	Subsystem 1 -- the link between the comms queues and the MQTT client.
 *
 *	Included by comms.c (which implements the hooks) and by
 *	lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c (which calls them from
 *	the kernel's WiFi service task).  Plain C types only: mqtt_utk.c must
 *	not see tk/tkernel.h.  Other subsystems use comms.h, not this file.
 *
 *	Payloads are binary (include/telemetry_frame.h): they are passed with
 *	an explicit length and are NOT '\0'-terminated.
 */

#ifndef COMMS_HOOKS_H
#define COMMS_HOOKS_H

#define COMMS_TOPIC_STATUS	"robot/status"		/* retained text: online/offline */
#define COMMS_TOPIC_HEARTBEAT	"robot/heartbeat"
#define COMMS_TOPIC_FRAME	"robot/frame"		/* 21-byte status frame, QoS 0 */
#define COMMS_TOPIC_TELEMETRY	"robot/telemetry/"	/* + name; text diagnostics only */
#define COMMS_TOPIC_EVENT	"robot/event"		/* 2-byte event frame, QoS 1 */
#define COMMS_TOPIC_CMD		"robot/cmd"		/* 2-byte command, QoS 1 */
#define COMMS_TOPIC_ACK		"robot/cmd/ack"		/* 2-byte acknowledgement, QoS 1 */

#define COMMS_NAME_MAX		16	/* telemetry name, including the '\0' */
#define COMMS_TOPIC_MAX		32	/* full topic, including the '\0' */
#define COMMS_PAYLOAD_MAX	96	/* payload bytes */

/* Next queued message, events before telemetry.  Never waits.  topic must
   hold COMMS_TOPIC_MAX bytes and payload COMMS_PAYLOAD_MAX bytes.
   Returns the payload length in bytes, or -1 if nothing is queued. */
int comms_hook_next(char *topic, unsigned char *payload, int *qos);

/* A message arrived on COMMS_TOPIC_CMD (len < 0: truncated).  Valid
   commands are passed on to the Vehicle Controller; either way a binary
   acknowledgement is queued. */
void comms_hook_command(const unsigned char *payload, int len);

/* The MQTT connection came up (1) or went down (0). */
void comms_hook_online(int online);

#endif /* COMMS_HOOKS_H */
