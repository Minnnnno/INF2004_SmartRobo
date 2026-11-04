/*
 *	subsystem5.c
 *	Subsystem 5 -- obstacle detection and width measurement.
 *
 *	1. Watch straight ahead (servo 90 deg) while the line follower drives.
 *	2. Stop the robot with the sensor 150 mm from the obstacle.
 *	3. Sweep 20 -> 160 deg in 10 deg steps, then walk 1 deg at a time out
 *	   from each coarse edge until clear space is confirmed.
 *	4. Print the width, where the obstacle sits and which side to pass it
 *	   on (and publish them).
 *	5. Drive round it on that side and back onto the line (detour.c), then
 *	   hand the wheels back to the line follower.  If anything about the
 *	   detour is uncertain, the robot stays stopped instead.
 *
 *	The work is split: obstacle_scan.c and detour_plan.c (the algorithms),
 *	detour.c (the detour's moves), ultrasonic.c and servo.c (the hardware).
 *	This file is the task and its motion rules.
 */

#include "subsystem5.h"
#include "obstacle_scan.h"
#include "ultrasonic.h"
#include "servo.h"
#include "detour.h"
#include "detour_plan.h"
#include "robot.h"
#include "motion_hal.h"
#include "../subsystem1/comms.h"
#include "crash_log.h"
#include <tm/tmonitor.h>
#include <math.h>

#if defined(ROBO_PRODUCTION_BUILD) && !defined(ROBO_IR_MONITOR) && ROBO_SUBSYSTEM5

#define S5_STOP_LATENCY_S	0.010	/* stop latch until the next 10 ms motion tick */
#define S5_BRAKE_MM		3.0	/* travel while braking from line speed; tune */
#define S5_PING_PERIOD_S	((double)S5_US_CYCLE_MS / 1000.0)
#define S5_HISTORY		3U	/* front distance: median of the last 3 pings */
#define S5_HISTORY_AGE_S	0.25	/* older pings no longer count */
#define S5_WARN_PERIOD_S	5.0
#define S5_PUBLISH_TRIES	10U	/* state changes the line follower must see */

typedef struct {
	double		range_mm[S5_HISTORY];
	double		time_s[S5_HISTORY];
	bool		b_echo[S5_HISTORY];
	unsigned	next;
} s5_history_t;

static s5_scan_t	s5_scan;	/* ~10 KB: kept off the task stack */
static double		s5_front_mm = INFINITY;
static double		s5_width_m;
static double		s5_offset_m;
static bool		s5_sensor_ok;
static bool		s5_blocked;
static bool		s5_decided;	/* s5_avoid holds the held obstacle's decision */
static s5_avoid_t	s5_avoid;	/* read by other tasks: s5_get_avoidance() */

typedef enum {
	S5_PHASE_IDLE,		/* watching, or held at an obstacle */
	S5_PHASE_AVOIDING,	/* driving round it: the line follower keeps off */
	S5_PHASE_FAILED		/* gave up on the detour; held */
} s5_phase_t;

static s5_phase_t	s5_phase;

static int s5_round(double value)
{
	return (int)((value < 0.0) ? (value - 0.5) : (value + 0.5));
}

static int s5_magnitude(int value)
{
	return (value < 0) ? -value : value;
}

static void history_clear(s5_history_t *p_history)
{
	unsigned i;

	for (i = 0U; i < S5_HISTORY; i++) {
		p_history->b_echo[i] = false;
		p_history->range_mm[i] = 0.0;
		p_history->time_s[i] = 0.0;
	}
	p_history->next = 0U;
}

static void history_add(s5_history_t *p_history, bool b_echo, double range_mm, double time_s)
{
	p_history->b_echo[p_history->next] = b_echo;
	p_history->range_mm[p_history->next] = range_mm;
	p_history->time_s[p_history->next] = time_s;
	p_history->next = (p_history->next + 1U) % S5_HISTORY;
}

/* Median of the last pings, each moved forward to "now" by the distance
   driven since it was taken.  A missed or stale ping votes for "clear", so
   one stray short echo cannot stop the robot but two can. */
static double history_front(const s5_history_t *p_history, double now, double speed_mm_s)
{
	double		value[S5_HISTORY];
	double		swap;
	unsigned	i;

	for (i = 0U; i < S5_HISTORY; i++) {
		double age = now - p_history->time_s[i];
		value[i] = (p_history->b_echo[i] && (age <= S5_HISTORY_AGE_S)) ?
			   p_history->range_mm[i] - speed_mm_s * fmax(0.0, age) : INFINITY;
	}
	if (value[0] > value[1]) { swap = value[0]; value[0] = value[1]; value[1] = swap; }
	if (value[1] > value[2]) { swap = value[1]; value[1] = value[2]; value[2] = swap; }
	if (value[0] > value[1]) { swap = value[0]; value[0] = value[1]; value[1] = swap; }
	return value[1];
}

/* Stop on the ping nearest to S5_STOP_MM: the next one comes a ping period
   (~5 mm at line speed) later, so stop now if that is closer to target. */
static bool should_stop(double front_mm, double speed_mm_s)
{
	double speed = fmax(0.0, speed_mm_s);
	double predicted = front_mm - speed * S5_STOP_LATENCY_S -
			   ((speed > 1.0) ? S5_BRAKE_MM : 0.0);

	return isfinite(predicted) &&
	       (predicted - S5_STOP_MM <= 0.5 * speed * S5_PING_PERIOD_S);
}

static void note_sensor(s5_us_status_t result)
{
	s5_sensor_ok = (result == S5_US_OK) || (result == S5_US_NO_ECHO);
}

static bool publish_status(double now)
{
	subsystem_status_t status;

	status.available = s5_sensor_ok;
	status.timestamp = now;
	status.values[0] = (isfinite(s5_front_mm) ? s5_front_mm : S5_US_MAX_MM) / 1000.0;
	status.values[1] = s5_offset_m;
	status.values[2] = s5_width_m;
	status.values[3] = (s5_phase == S5_PHASE_AVOIDING) ? S5_STATE_AVOIDING :
			   !s5_blocked ? S5_STATE_CLEAR :
			   (s5_phase == S5_PHASE_FAILED) ? S5_STATE_AVOID_FAILED :
			   !s5_decided ? S5_STATE_HELD :
			   (s5_avoid.side == S5_AVOID_LEFT) ? S5_STATE_PASS_LEFT :
			   (s5_avoid.side == S5_AVOID_RIGHT) ? S5_STATE_PASS_RIGHT : S5_STATE_NO_WAY;
	return robot_update_subsystem(SOURCE_OBSTACLE, &status);
}

/* The line follower reads values[3] to keep off the wheels: a change of
   phase must reach the shared status before the robot moves. */
static bool publish_verified(void)
{
	unsigned attempt;

	for (attempt = 0U; attempt < S5_PUBLISH_TRIES; attempt++) {
		if (publish_status(motion_hal_time())) {
			return true;
		}
		(void)tk_dly_tsk(2U);
	}
	return false;
}

bool s5_get_avoidance(s5_avoid_t *p_out)
{
	UINT	intsts;
	bool	b_ok;

	DI(intsts);
	b_ok = s5_blocked && s5_decided;
	*p_out = s5_avoid;
	EI(intsts);
	return b_ok;
}

static void set_decision(bool b_decided, const s5_avoid_t *p_avoid)
{
	UINT intsts;

	DI(intsts);
	s5_decided = b_decided;
	if (p_avoid != NULL) {
		s5_avoid = *p_avoid;
	}
	EI(intsts);
}

/* ---- scan callbacks ---- */

static bool scan_ping(void *p_context, double *p_range_mm)
{
	double		time_s;
	bool		*p_failed = p_context;
	s5_us_status_t	result;

	if (*p_failed) { return false; }
	result = s5_us_ping(p_range_mm, &time_s);
	note_sensor(result);
	*p_failed = !s5_sensor_ok;
	return result == S5_US_OK;
}

/* The scan needs the robot still.  If something released the motors
   meanwhile (e.g. an MQTT "start"), stop again: the obstacle is right there. */
static bool hold_still(void)
{
	robot_status_t status;

	if (s5_status(&status) && status.motion.hold && status.motion.stationary &&
	    (status.motion.state == STOPPED) && (status.motion.fault == F_NONE)) {
		return true;
	}
	motion_stop();
	tm_printf((const UB *)"[OBSTACLE] hold/still confirmation lost; scan aborted\n");
	return false;
}

static bool scan_measure(void *p_context, int angle_deg, s5_estimate_t *p_estimate)
{
	bool failed = false;
	(void)p_context;
	if (!hold_still()) { return false; }
	s5_servo_move(angle_deg);
	if (!hold_still()) { return false; }
	s5_sample_point(scan_ping, &failed, p_estimate);
	(void)publish_status(motion_hal_time());
	if (failed) {
		motion_stop();
		tm_printf((const UB *)"[OBSTACLE] ultrasonic failed during scan; holding\n");
		return false;
	}
	return hold_still();
}

/* ---- report ---- */

static void print_point(const char *p_phase, const s5_point_t *p_point)
{
	const s5_estimate_t *p_estimate = &p_point->estimate;

	if (!s5_estimate_usable(p_estimate)) {
		tm_printf((const UB *)"[OBSTACLE] %s %3d deg: %s (%u/%u pings echoed)          -> clear\n",
			  p_phase, p_point->angle_deg,
			  (p_estimate->status == S5_ECHO_NONE) ? "no echo     " : "no agreement",
			  p_estimate->valid, p_estimate->total);
		return;
	}
	tm_printf((const UB *)"[OBSTACLE] %s %3d deg: %4d mm, forward %4d mm, %2u/%2u agree -> %s\n",
		  p_phase, p_point->angle_deg, s5_round(p_estimate->range_mm),
		  s5_round(s5_forward_mm(p_estimate->range_mm, (double)p_point->angle_deg)),
		  p_estimate->support, p_estimate->total,
		  p_point->b_object ? "OBSTACLE" : "clear");
}

static void print_edge(const char *p_name, const s5_edge_t *p_edge, int limit_deg)
{
	int tenths = s5_round(p_edge->angle_deg * 10.0);
	int out = s5_round(p_edge->outward_deg * 10.0);
	int in = s5_round(p_edge->inward_deg * 10.0);
	int lateral = s5_round(p_edge->lateral_mm);

	if (p_edge->b_found) {
		tm_printf((const UB *)"[OBSTACLE] %s edge: outward %d.%d deg, inward %d.%d deg%s -> %d.%d deg, "
			  "%d mm away, %d mm %s of centre\n",
			  p_name, out / 10, out % 10, in / 10, in % 10,
			  p_edge->b_inward ? "" : " (not refound)",
			  tenths / 10, tenths % 10, s5_round(p_edge->range_mm),
			  s5_magnitude(lateral), (lateral >= 0) ? "left" : "right");
	} else {
		tm_printf((const UB *)"[OBSTACLE] %s edge: NOT FOUND, obstacle continues past the %d deg "
			  "limit (%d mm %s of centre)\n",
			  p_name, limit_deg, s5_magnitude(lateral), (lateral >= 0) ? "left" : "right");
	}
}

static const char *avoid_name(s5_avoid_side_t side)
{
	return (side == S5_AVOID_LEFT) ? "left" : ((side == S5_AVOID_RIGHT) ? "right" : "none");
}

/* "123 mm" or "none" for a side with nothing beside the obstacle */
static void print_side(const char *p_name, unsigned side, const s5_scan_t *p_scan)
{
	const s5_avoid_t *p_avoid = &p_scan->avoid;

	if (isfinite(p_avoid->clear_mm[side])) {
		tm_printf((const UB *)"[OBSTACLE]   %s: move %s%d mm, nearest thing beside it %d mm out -> %s\n",
			  p_name, p_scan->edge[side].b_found ? "" : ">= ",
			  s5_round(p_avoid->shift_mm[side]), s5_round(p_avoid->clear_mm[side]),
			  p_avoid->b_room[side] ? "room" : "NO ROOM");
	} else {
		tm_printf((const UB *)"[OBSTACLE]   %s: move %s%d mm, nothing beside it -> %s\n",
			  p_name, p_scan->edge[side].b_found ? "" : ">= ",
			  s5_round(p_avoid->shift_mm[side]),
			  p_avoid->b_room[side] ? "room" : "NO ROOM (edge beyond the sweep)");
	}
}

static void print_avoid(const s5_scan_t *p_scan)
{
	const char *p_why;

	switch (p_scan->avoid.side) {
	case S5_AVOID_LEFT:
	case S5_AVOID_RIGHT:
		p_why = (p_scan->placement.position == S5_POSITION_LEFT) ||
			(p_scan->placement.position == S5_POSITION_RIGHT) ?
			"obstacle reaches less far that way" : "centred: default side";
		if (((p_scan->placement.position == S5_POSITION_LEFT) &&
		     (p_scan->avoid.side == S5_AVOID_LEFT)) ||
		    ((p_scan->placement.position == S5_POSITION_RIGHT) &&
		     (p_scan->avoid.side == S5_AVOID_RIGHT)) ||
		    ((p_scan->placement.position != S5_POSITION_LEFT) &&
		     (p_scan->placement.position != S5_POSITION_RIGHT) &&
		     (p_scan->avoid.side != S5_DEFAULT_AVOID))) {
			p_why = "the other side has no room";
		}
		tm_printf((const UB *)"[OBSTACLE] ===== PASS ON THE %s (%s) =====\n",
			  (p_scan->avoid.side == S5_AVOID_LEFT) ? "LEFT" : "RIGHT", p_why);
		break;
	default:
		tm_printf((const UB *)"[OBSTACLE] ===== NO WAY AROUND: neither side has room =====\n");
		break;
	}
	print_side("left ", S5_LEFT, p_scan);
	print_side("right", S5_RIGHT, p_scan);
}

static const char *position_name(s5_position_t position)
{
	switch (position) {
	case S5_POSITION_CENTRE:	return "centre";
	case S5_POSITION_LEFT:		return "left";
	case S5_POSITION_RIGHT:		return "right";
	default:			return "unknown";
	}
}

/* Degrees as "<whole>.<tenth>" of the magnitude; the sign is in the words */
static void tenths_of(double degrees, int *p_whole, int *p_tenth)
{
	int tenths = s5_magnitude(s5_round(degrees * 10.0));

	*p_whole = tenths / 10;
	*p_tenth = tenths % 10;
}

static void print_placement(const s5_placement_t *p_placement)
{
	int	diff_whole, diff_tenth, left_whole, left_tenth, right_whole, right_tenth, tol_whole, tol_tenth;

	tenths_of(p_placement->difference_deg, &diff_whole, &diff_tenth);
	tenths_of(p_placement->left_deg, &left_whole, &left_tenth);
	tenths_of(p_placement->right_deg, &right_whole, &right_tenth);
	tenths_of(S5_CENTRE_TOLERANCE_DEG, &tol_whole, &tol_tenth);

	switch (p_placement->position) {
	case S5_POSITION_LEFT:
	case S5_POSITION_RIGHT:
		tm_printf((const UB *)"[OBSTACLE] position: %s by %d.%d deg (left edge %d.%d deg, right edge %d.%d deg, "
			  "tolerance %d.%d deg)%s\n",
			  (p_placement->position == S5_POSITION_LEFT) ? "LEFT" : "RIGHT",
			  diff_whole, diff_tenth, left_whole, left_tenth, right_whole, right_tenth,
			  tol_whole, tol_tenth,
			  p_placement->b_bounded ? ", one edge beyond the sweep" : "");
		break;
	case S5_POSITION_CENTRE:
		tm_printf((const UB *)"[OBSTACLE] position: CENTRE (left edge %d.%d deg, right edge %d.%d deg, "
			  "within %d.%d deg)\n",
			  left_whole, left_tenth, right_whole, right_tenth, tol_whole, tol_tenth);
		break;
	default:
		tm_printf((const UB *)"[OBSTACLE] position: UNKNOWN (%s)\n",
			  p_placement->b_bounded ? "edges beyond the sweep, cannot compare" : "no obstacle");
		break;
	}
}

static void report_scan(const s5_scan_t *p_scan)
{
	unsigned	i;
	int		width;
	int		distance;
	int		offset;
	bool		b_complete;

	if (!p_scan->b_valid) {
		tm_printf((const UB *)"[OBSTACLE] no echo straight ahead when the scan began; no width measured\n");
		return;
	}
	distance = s5_round(p_scan->centre.estimate.range_mm);
	tm_printf((const UB *)"[OBSTACLE] stopped %d mm from the obstacle (target %d mm), %u/%u pings agree\n",
		  distance, s5_round(S5_STOP_MM),
		  p_scan->centre.estimate.support, p_scan->centre.estimate.total);

	tm_printf((const UB *)"[OBSTACLE] coarse sweep, %d deg steps (obstacle = forward distance within %d mm of 90 deg)\n",
		  S5_COARSE_STEP_DEG, s5_round(S5_EDGE_THRESHOLD_FRACTION * p_scan->centre_forward_mm));
	for (i = 0U; i < p_scan->coarse_count; i++) {
		print_point("coarse", &p_scan->coarse[i]);
	}
	tm_printf((const UB *)"[OBSTACLE] fine scan, left edge, %d deg steps (out, then back in)\n",
		  S5_FINE_STEP_DEG);
	for (i = 0U; i < p_scan->fine_count[S5_LEFT]; i++) {
		print_point((i < p_scan->fine_outward[S5_LEFT]) ? "out   " : "in    ",
			    &p_scan->fine[S5_LEFT][i]);
	}
	tm_printf((const UB *)"[OBSTACLE] fine scan, right edge, %d deg steps (out, then back in)\n",
		  S5_FINE_STEP_DEG);
	for (i = 0U; i < p_scan->fine_count[S5_RIGHT]; i++) {
		print_point((i < p_scan->fine_outward[S5_RIGHT]) ? "out   " : "in    ",
			    &p_scan->fine[S5_RIGHT][i]);
	}
	print_edge("left ", &p_scan->edge[S5_LEFT], S5_ANGLE_LEFT_DEG);
	print_edge("right", &p_scan->edge[S5_RIGHT], S5_ANGLE_RIGHT_DEG);

	width = s5_round(p_scan->width_mm);
	offset = s5_round(p_scan->offset_mm);
	b_complete = p_scan->edge[S5_LEFT].b_found && p_scan->edge[S5_RIGHT].b_found;
	tm_printf((const UB *)"[OBSTACLE] ===== OBSTACLE WIDTH %s%d mm (%d.%d cm), %d mm ahead, "
		  "centre %d mm %s =====\n",
		  b_complete ? "" : ">= ", width, width / 10, width % 10, distance,
		  s5_magnitude(offset), (offset >= 0) ? "left" : "right");
	print_placement(&p_scan->placement);
	print_avoid(p_scan);
	comms_latch_event(TF_EV_OBSTACLE | TF_EV_SCAN_DONE);

	if (comms_is_online()) {
		char payload[COMMS_PAYLOAD_MAX];
		/* The combined report exceeds the queue's 96-byte buffer even for
		   ordinary readings. Bound each write, and send two text diagnostics. */
		INT length = tm_snprintf((UB *)payload, (INT)sizeof(payload),
				(const UB *)"obstacle width_mm=%d dist_mm=%d offset_mm=%d complete=%d position=%s",
				width, distance, offset, b_complete ? 1 : 0,
				position_name(p_scan->placement.position));
		if ((length > 0) && ((size_t)length < sizeof(payload))) {
			(void)comms_send_telemetry("obstacle", payload);
		} else {
			tm_printf((const UB *)"[OBSTACLE] measurement event too long (%d bytes); skipped\n", length);
		}
		length = tm_snprintf((UB *)payload, (INT)sizeof(payload),
				(const UB *)"obstacle avoid=%s shift_l_mm=%d shift_r_mm=%d",
				avoid_name(p_scan->avoid.side),
				s5_round(p_scan->avoid.shift_mm[S5_LEFT]),
				s5_round(p_scan->avoid.shift_mm[S5_RIGHT]));
		if ((length > 0) && ((size_t)length < sizeof(payload))) {
			(void)comms_send_telemetry("obstacle", payload);
		} else {
			tm_printf((const UB *)"[OBSTACLE] avoidance event too long (%d bytes); skipped\n", length);
		}
	}
}

/* ---- detour ---- */

#if S5_AVOID_DRIVE

/* Drive round the held obstacle and back onto the line.  True: back on
   the line and released, so the line follower drives on.  False: the
   robot is stopped and held (or was never released). */
static bool drive_round(void)
{
	s5_detour_plan_t	plan;
	robot_status_t		status;

	if (!s5_decided || (s5_avoid.side == S5_AVOID_NONE)) {
		return false;
	}
	if (!s5_detour_plan(&s5_scan, &plan)) {
		tm_printf((const UB *)"[OBSTACLE] no safe detour (it would need over %d mm of reversing); holding\n",
			  s5_round(S5_BACKUP_MAX_MM));
		return false;
	}
	if (!s5_status(&status) || !s5_line_reporting(&status, motion_hal_time())) {
		tm_printf((const UB *)"[OBSTACLE] line sensors not reporting, could not find the way back; holding\n");
		return false;
	}
	s5_phase = S5_PHASE_AVOIDING;
	if (!publish_verified()) {
		s5_phase = S5_PHASE_IDLE;
		tm_printf((const UB *)"[OBSTACLE] could not tell the line follower to keep off; holding\n");
		return false;
	}
	crash_mark(CRASH_MARK_OBSTACLE, CRASH_OBSTACLE_DETOUR);
	if (!s5_detour_run(&plan)) {
		motion_stop();
		s5_servo_move(S5_ANGLE_CENTRE_DEG);
		s5_phase = S5_PHASE_FAILED;
		s5_blocked = true;
		(void)publish_verified();
		tm_printf((const UB *)"[OBSTACLE] ===== DETOUR ABANDONED: stopped and holding =====\n");
		return false;
	}
	s5_servo_move(S5_ANGLE_CENTRE_DEG);
	s5_phase = S5_PHASE_IDLE;
	s5_blocked = false;
	set_decision(false, NULL);	/* this obstacle is behind us */
	(void)publish_verified();	/* CLEAR: the line follower takes over */
	return true;
}
#endif

/* ---- task ---- */

void obstacle_task(INT code, void *p_context)
{
	s5_history_t	history;
	bool		b_scanned = false;	/* this obstacle has been measured */
	bool		b_announced = false;	/* approach zone already reported */
	double		next_warning = 0.0;

	(void)code; (void)p_context;
#ifdef S5_DIAG
	/* Diagnostic builds (-DS5_DIAG=n) to find which part of this task the
	   watchdog freeze needs: 0 nothing, 1 servo only, 2 ultrasonic pings only,
	   3 everything except the ultrasonic, 4 servo PWM slice without its pin,
	   6 servo only on GP14, 9 freeze-catcher self-test. */
	tm_printf((const UB *)"[OBSTACLE] S5_DIAG=%d\n", S5_DIAG);
#if (S5_DIAG == 1) || (S5_DIAG == 4) || (S5_DIAG == 6)
	s5_servo_init(S5_ANGLE_CENTRE_DEG);
#elif S5_DIAG == 2
	(void)s5_us_init();
	for (;;) {
		double	diag_range_mm;
		double	diag_time_s;

		(void)s5_us_ping(&diag_range_mm, &diag_time_s);
	}
#elif S5_DIAG == 9
	/* Starve Motion Control from task code: the catcher must report it */
	(void)tk_dly_tsk(5000U);
	(void)tk_chg_pri(TSK_SELF, 1);
	for (;;) {
	}
#endif
#if S5_DIAG != 3
	for (;;) {
		(void)tk_slp_tsk(TMO_FEVR);
	}
#endif
#endif
	history_clear(&history);
	s5_servo_init(S5_ANGLE_CENTRE_DEG);
#if !defined(S5_DIAG) || (S5_DIAG != 3)
	(void)s5_us_init();
#endif
	tm_printf((const UB *)"[OBSTACLE] ready: servo GP%u at 90 deg, ultrasonic TRIG GP16 / ECHO GP17, "
		  "slow zone %d mm, stop at %d mm\n", (unsigned)S5_SERVO_PIN, s5_round(S5_SLOW_MM), s5_round(S5_STOP_MM));

	for (;;) {
		robot_status_t	status;
		double		range_mm;
		double		time_s;
		double		now;
		double		speed_mm_s = 0.0;
		bool		b_may_move = false;
		s5_us_status_t	result;

		crash_mark(CRASH_MARK_OBSTACLE, CRASH_OBSTACLE_PING);
#if defined(S5_DIAG) && (S5_DIAG == 3)
		(void)tk_dly_tsk(S5_US_CYCLE_MS);		/* ultrasonic left off */
		range_mm = 0.0;
		time_s = 0.0;
		result = S5_US_NO_ECHO;
#else
		result = s5_us_ping(&range_mm, &time_s);	/* paces the loop, 60 ms */
#endif
		crash_mark(CRASH_MARK_OBSTACLE, CRASH_OBSTACLE_DECIDE);
		note_sensor(result);
		history_add(&history, result == S5_US_OK, range_mm, time_s);
		now = motion_hal_time();
		if (s5_status(&status)) {
			speed_mm_s = 500.0 * (status.motion.speed[0] + status.motion.speed[1]);
			b_may_move = !status.motion.hold &&
				     ((status.motion.state == READY) || (status.motion.state == DRIVING));
			s5_blocked = s5_blocked && status.motion.hold;
			if (!s5_blocked) {
				set_decision(false, NULL);	/* released: the decision is spent */
				s5_phase = S5_PHASE_IDLE;
			}
		}
		s5_front_mm = history_front(&history, now, speed_mm_s);
		if (!s5_sensor_ok && b_may_move) {
			motion_stop();
			s5_blocked = true;
			s5_phase = S5_PHASE_FAILED;
			b_may_move = false;
		}

		if (s5_front_mm > S5_CLEAR_MM) {
			b_scanned = false;		/* path clear: arm for the next obstacle */
			b_announced = false;
		}
		if (b_may_move && !b_announced && (s5_front_mm <= S5_SLOW_MM)) {
			b_announced = true;
			tm_printf((const UB *)"[OBSTACLE] approaching: %d mm ahead at %d mm/s\n",
				  s5_round(s5_front_mm), s5_round(speed_mm_s));
		}

		if (b_may_move && should_stop(s5_front_mm, speed_mm_s)) {
			crash_mark(CRASH_MARK_OBSTACLE, CRASH_OBSTACLE_STOP);
			motion_stop();
			s5_blocked = true;
			tm_printf((const UB *)"[OBSTACLE] STOP: %d mm ahead at %d mm/s\n",
				  s5_round(s5_front_mm), s5_round(speed_mm_s));
			crash_mark(CRASH_MARK_OBSTACLE, CRASH_OBSTACLE_SETTLE);
			if (!s5_wait_still()) {
				s5_phase = S5_PHASE_FAILED;
				set_decision(false, NULL);
				(void)publish_verified();
				history_clear(&history);
				tm_printf((const UB *)"[OBSTACLE] robot not confirmed still; scan cancelled, holding\n");
				continue;
			}
			if (!b_scanned) {
				tm_printf((const UB *)"[OBSTACLE] scanning %d..%d deg\n",
					  S5_ANGLE_LEFT_DEG, S5_ANGLE_RIGHT_DEG);
				crash_mark(CRASH_MARK_OBSTACLE, CRASH_OBSTACLE_SCAN);
				set_decision(false, NULL);
				if (s5_scan_run(&s5_scan, scan_measure, NULL)) {
					set_decision(true, &s5_scan.avoid);
				} else {
					motion_stop();
					s5_phase = S5_PHASE_FAILED;
				}
				s5_servo_move(S5_ANGLE_CENTRE_DEG);
				if (s5_scan.b_valid) {
					s5_width_m = s5_scan.width_mm / 1000.0;
					s5_offset_m = s5_scan.offset_mm / 1000.0;
				}
				crash_mark(CRASH_MARK_OBSTACLE, CRASH_OBSTACLE_REPORT);
				report_scan(&s5_scan);
				b_scanned = true;
				b_announced = true;
#if S5_AVOID_DRIVE
				if (drive_round()) {
					b_scanned = false;	/* passed: arm for the next obstacle */
					b_announced = false;
				}
#endif
			} else {
				tm_printf((const UB *)"[OBSTACLE] the measured obstacle is still ahead; holding\n");
			}
			history_clear(&history);	/* pings from the sweep are not "ahead" */
		} else if (!s5_sensor_ok && (now >= next_warning)) {
			next_warning = now + S5_WARN_PERIOD_S;
			tm_printf((const UB *)"[OBSTACLE] ultrasonic not answering (%s); check GP16/GP17 wiring and 3.3 V supply\n",
				  (result == S5_US_BUSY) ? "ECHO stuck high" : "no ECHO pulse");
		}
		(void)publish_status(motion_hal_time());
	}
}

#else /* disabled, calibration profiles and the IR monitor: leave the pins alone */

void obstacle_task(INT code, void *p_context)
{
	(void)code; (void)p_context;
	for (;;) {
		(void)tk_slp_tsk(TMO_FEVR);
	}
}

#endif
