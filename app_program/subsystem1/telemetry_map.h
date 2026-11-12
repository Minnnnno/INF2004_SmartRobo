/*
 *	telemetry_map.h
 *	Subsystem 1 -- converts a copied robot_status_t snapshot into status
 *	frame field values (telemetry_frame.h units).  No kernel calls, no
 *	locks: the caller copies the snapshot under the bus mutex, releases
 *	it, and only then calls these functions.
 */

#ifndef TELEMETRY_MAP_H
#define TELEMETRY_MAP_H

#include "robot.h"
#include "telemetry_frame.h"

typedef struct {
	uint32_t	seq;		/* frame counter; low 8 bits are sent */
	double		uptime_s;	/* monotonic seconds */
	uint32_t	rejected;	/* controller rejections since last frame */
	uint32_t	dropped;	/* comms drops since last frame */
} tm_frame_info_t;

/* Fill values[] from a snapshot.  Values may be out of range; tf_pack()
   clamps them.  Non-finite inputs become 0 before clamping. */
void tm_fill(const robot_status_t *status, const tm_frame_info_t *info,
	     int32_t values[TF_FIELD_COUNT]);

/* Fill an encoder frame from a snapshot copied at now_s (robot clock,
   seconds).  Freshness and age are judged on the robot's clock only.
   seq is the status frame counter of the same period (low 8 bits sent). */
void tm_fill_encoder(const robot_status_t *status, uint32_t seq, double now_s,
		     tf_encoder_t *enc);

/* Events that can be seen by comparing two consecutive snapshots:
   TF_EV_MOTION_FAULT, TF_EV_LINE_LOST, TF_EV_BARCODE. */
uint8_t tm_edge_events(const robot_status_t *previous, const robot_status_t *current);

#endif /* TELEMETRY_MAP_H */
