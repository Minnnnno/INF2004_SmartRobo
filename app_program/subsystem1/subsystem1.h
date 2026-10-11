/*
 *	subsystem1.h
 *	Subsystem 1 -- WiFi communication, command and telemetry.
 *	Student ID: 2500842.
 *
 *	This header is subsystem 1's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem1/subsystem1.h".
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "comms.h"			/* IF-06/07: the communication API */

#define S1_PRI_COMMS	20		/* lowest application task (TEAM.md) */

#endif /* SUBSYSTEM1_H */
