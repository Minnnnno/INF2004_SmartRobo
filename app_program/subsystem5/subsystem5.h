/*
 *	subsystem5.h
 *	Subsystem 5 -- obstacle detection, width, which side to pass on, and
 *	driving round it back to the line.
 *
 *	obstacle_task (include/subsystems.h) is created by usermain at priority 4.
 *	Hardware: ultrasonic TRIG GP16 / ECHO GP17, mounted on the servo on GP15.
 *	Active in production builds only; idle (pins untouched) in calibration
 *	profiles and in ROBO_IR_MONITOR.
 *
 *	While watching it only calls motion_stop(), which latches a hold that
 *	the line follower respects.  After the scan it releases that hold and
 *	drives round the obstacle itself (SOURCE_OBSTACLE finite moves,
 *	detour.c); values[3] = S5_STATE_AVOIDING tells the line follower to
 *	keep off the wheels meanwhile.  Back on the line it publishes
 *	S5_STATE_CLEAR and the line follower drives on.
 *
 *	Shared status, written with robot_update_subsystem(SOURCE_OBSTACLE):
 *	  values[0]  distance straight ahead, m (S5_US_MAX_MM / 1000 when clear)
 *	  values[1]  centre of the last measured obstacle, m, positive = left
 *	  values[2]  width of the last measured obstacle, m (0 before the first)
 *	  values[3]  S5_STATE_*: clear, held (scanning), pass left, pass right,
 *	             no way around, driving round it, detour abandoned, or
 *	             stopped from outside (STOP / lifted)
 *	  available  FALSE while the ultrasonic sensor is not answering
 *
 *	Two bounded MQTT text diagnostics when online (robot/telemetry/obstacle):
 *	  "obstacle width_mm=<w> dist_mm=<d> offset_mm=<o> complete=<0|1>
 *	   position=<centre|left|right|unknown>"
 *	  "obstacle avoid=<left|right|none> shift_l_mm=<l> shift_r_mm=<r>"
 *	and, when a scan or detour ends without driving round it:
 *	  "obstacle abort=<code> a=<a> b=<b> x=<x> y=<y> tries=<n> state=<failed|stopped>"
 *	  "obstacle spurious_stop ahead_mm=<d> resumed=<0|1>"
 *	(codes and their a/b: README.md "Why a detour stopped").
 *	offset is the obstacle centre, + left; complete=0 means an edge lay
 *	beyond the 20..160 deg sweep, so the width is a lower bound.
 *	position compares how far each edge is from 90 deg (obstacle_scan.h);
 *	avoid is the side to pass on, shift_* how far the robot's centre must
 *	move sideways to pass that side with S5_PASS_MARGIN_MM to spare.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "subsystems.h"
#include "obstacle_scan.h"

#define S5_SLOW_MM	450.0	/* approach zone: reported, see README.md */
#define S5_STOP_MM	150.0	/* stop with the sensor this far from the obstacle;
				   closer makes the width scan less sensitive to the
				   wide ultrasonic beam (README.md) */
#define S5_CLEAR_MM	500.0	/* path counts as clear again beyond this */
/* 1: drive round the obstacle after the scan.  0: stay stopped (the
   decision is still published for another subsystem to act on). */
#define S5_AVOID_DRIVE	1

/* values[3] of the shared status */
#define S5_STATE_CLEAR		0.0	/* not held for an obstacle */
#define S5_STATE_HELD		1.0	/* held: scanning, or nothing measured */
#define S5_STATE_PASS_LEFT	2.0
#define S5_STATE_PASS_RIGHT	3.0
#define S5_STATE_NO_WAY		4.0	/* neither side has room */
#define S5_STATE_AVOIDING	5.0	/* driving round it: the line follower must not drive
					   (mirrored as LINE_OBSTACLE_AVOIDING in line_following.c) */
#define S5_STATE_AVOID_FAILED	6.0	/* gave up on the detour: stopped and held */
#define S5_STATE_STOPPED	7.0	/* scan or detour stopped from outside (STOP,
					   Buddy 4 lifted): held, nothing went wrong */

#endif /* SUBSYSTEM5_H */
