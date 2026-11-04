/*
 *	comms.h
 *	Subsystem 1 -- communication API for the other subsystems.
 *	Include it as "../subsystem1/comms.h".
 *
 *	Dashboard data is NOT sent through this API.  Subsystem 1 copies the
 *	shared robot status (robot_update_subsystem / motion_get_status) every
 *	100 ms, packs it into the 21-byte frame in include/telemetry_frame.h
 *	and publishes it.  To get a value onto the dashboard, write it to your
 *	subsystem_status_t slot as described in docs/API.md and
 *	app_program/subsystem1/telemetry_map.c.
 *
 *	Nothing in this API ever waits, so it is safe to call from a
 *	time-critical task.
 */

#ifndef COMMS_H
#define COMMS_H

#include "robot_rtos.h"		/* standard size_t before the kernel headers */
#include "../app.h"
#include "comms_hooks.h"	/* topics and size limits */
#include "telemetry_frame.h"	/* TF_EV_* event bits, binary formats */

/* Commands forwarded to the Vehicle Controller.  Values equal tf_opcode_t
   and robot_remote_command_t (checked in comms.c and app_main.c). */
typedef enum {
	COMMS_CMD_START = 1,
	COMMS_CMD_STOP,
	COMMS_CMD_ESTOP
} COMMS_CMD;

/* Latch one or more TF_EV_* events.  They are published once, together,
   at QoS 1 within 100 ms of the broker being reachable.  Safe from tasks
   and interrupt handlers (interrupts are disabled for a few instructions). */
IMPORT void comms_latch_event(UB events);

/* Text diagnostics on robot/telemetry/<name>, QoS 0.  Not decoded by the
   dashboard; prefer the console log.  payload is '\0'-terminated text of
   at most COMMS_PAYLOAD_MAX - 1 characters.
   Errors: E_PAR name or payload missing or too long, E_TMOUT queue full. */
IMPORT ER comms_send_telemetry(const char *name, const char *payload);

/* TRUE while connected to the MQTT broker */
IMPORT BOOL comms_is_online(void);

/* IF-07: each message is one COMMS_CMD.  The Vehicle Controller reads it:
       COMMS_CMD cmd;
       tk_rcv_mbf(mbf_comms_cmd, &cmd, TMO_POL);                           */
IMPORT ID	mbf_comms_cmd;

/* Diagnostics and NFR6 timing evidence.  Each field has one writer; a copy
   taken while running may mix two updates. */
typedef struct {
	UW	frames;		/* status frames queued */
	UW	skipped;	/* periods skipped because the bus mutex was busy */
	UW	clamped;	/* frames with at least one clamped field */
	UW	events;		/* event frames queued */
	UW	cmd_ok, cmd_duplicate, cmd_busy, cmd_rejected;
	UW	copy_us_max;	/* longest snapshot copy, includes the mutex hold */
	UW	pack_us_max;	/* longest map + pack, done with no lock held */
} COMMS_STATS;

IMPORT void	comms_stats(COMMS_STATS *stats);

/* Private to subsystem 1 */
IMPORT ER	comms_init(void);
IMPORT ER	comms_start(void);
IMPORT UW	comms_dropped(void);

#endif /* COMMS_H */
