/*
 *	comms.h
 *	Subsystem 1 -- communication API for the other subsystems.
 *	Include it as "../subsystem1/comms.h".
 *
 *	Messages are queued here and published over MQTT by the kernel's WiFi
 *	service task.  Nothing in this API ever waits, so it is safe to call
 *	from a time-critical task.  Payloads are '\0'-terminated text of at
 *	most COMMS_PAYLOAD_MAX - 1 characters.
 *
 *	Errors: E_PAR    name or payload missing or too long
 *	        E_TMOUT  queue full, message dropped
 */

#ifndef COMMS_H
#define COMMS_H

#include "../app.h"
#include "comms_hooks.h"	/* topics and size limits */

/* Commands accepted on COMMS_TOPIC_CMD: "start", "stop", "estop" */
typedef enum {
	COMMS_CMD_START = 1,
	COMMS_CMD_STOP,
	COMMS_CMD_ESTOP
} COMMS_CMD;

/* IF-06: periodic data, published at QoS 0 on robot/telemetry/<name> */
IMPORT ER comms_send_telemetry(const char *name, const char *payload);

/* IF-06: one-off events (barcode, hump, obstacle, fault), QoS 1 on robot/event */
IMPORT ER comms_send_event(const char *payload);

/* TRUE while connected to the MQTT broker */
IMPORT BOOL comms_is_online(void);

/* IF-07: each message is one COMMS_CMD.  The Vehicle Controller reads it:
       COMMS_CMD cmd;
       tk_rcv_mbf(mbf_comms_cmd, &cmd, TMO_FEVR);                          */
IMPORT ID	mbf_comms_cmd;

/* Private to subsystem 1 */
IMPORT ER	comms_init(void);
IMPORT ER	comms_start(void);
IMPORT UW	comms_dropped(void);

#endif /* COMMS_H */
