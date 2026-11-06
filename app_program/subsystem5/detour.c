/*
 *	detour.c
 *	Subsystem 5 -- drive round a scanned obstacle and back onto the line.
 *
 *	1. Release the hold obstacle_task latched to scan.
 *	2. Back up if needed, turn out, drive the diagonal, turn back parallel.
 *	3. Drive forward without stopping, the sensor alternating between sideways
 *	   and ahead, until the obstacle has ended and the rear clears it.
 *	4. Turn in and drive back slowly until Buddy 3's sensors find the line.
 *
 *	From S5_REJOIN_AFTER_MM into step 3, once parallel again after the diagonal,
 *	and during step 4, Buddy 3's centre and right sensors both seeing black
 *	(line_rejoin_seen) interrupts the detour: the robot stops, makes sure the
 *	line is under the sensors and hands straight back to the line follower.
 *	It does not line itself up first: the line need not run the way the robot
 *	was heading when it stopped, and the follower's search finds it from here.
 *
 *	Positions are tracked from the encoders (detour_plan.h frame).  Every
 *	step has a limit; anything unexpected returns false and the caller
 *	stops the robot.
 */

#include "robot_rtos.h"	/* first: kernel types with the C library's size_t */
#include "detour.h"
#include "steering.h"
#include "motion_hal.h"
#include "servo.h"
#include "ultrasonic.h"
#include "crash_log.h"
#include "../subsystem3/line_rejoin.h"
#include <tm/tmonitor.h>
#include <math.h>

#define S5_DEG_TO_RAD		(0.017453292519943295)
#define S5_POLL_MS		20U
#define S5_SETTLE_TIMEOUT_S	4.0	/* > Motion Control's 3 s stop timeout */
#define S5_RELEASE_TIMEOUT_S	1.0
#define S5_ACCEPT_TIMEOUT_S	0.5	/* a move not started by then was rejected */
#define S5_MOVE_SLACK_S		3.0	/* on top of distance / speed */
#define S5_TURN_TIMEOUT_S	4.0
#define S5_LINE_FRESH_S		0.2	/* line readings older than this are stale */
#define S5_LINE_CONFIRM_S	0.3	/* wait this long for the line after aligning */
#define S5_MIN_MOVE_MM		3.0	/* shorter moves are skipped (1 mm command resolution) */
#define S5_SQUARE_SETTLE_MS	500U	/* rocking after squaring up would abort the next scan */
#define S5_MIN_TURN_DEG		1.5	/* smaller turns are skipped (1 deg command resolution) */
#define S5_REJOIN_AFTER_MM	20.0	/* driven parallel before the line may end the detour */
#define S5_ID_FIRST		5100U
#define S5_ID_LAST		5999U

bool s5_status(robot_status_t *p_status)
{
	unsigned attempt;

	for (attempt = 0U; attempt < 3U; attempt++) {
		if (motion_get_status(p_status)) {
			return true;
		}
		(void)tk_dly_tsk(1U);
	}
	return false;
}

#define S5_STILL_MOVE_MM	5.0

static double	s5_still_travel[2];

void s5_still_mark(void)
{
	robot_status_t status;

	if (s5_status(&status)) {
		s5_still_travel[0] = status.motion.travel[0];
		s5_still_travel[1] = status.motion.travel[1];
	}
}

bool s5_still_moved(const robot_status_t *p_status)
{
	return (fabs(p_status->motion.travel[0] - s5_still_travel[0]) * 1000.0 > S5_STILL_MOVE_MM) ||
	       (fabs(p_status->motion.travel[1] - s5_still_travel[1]) * 1000.0 > S5_STILL_MOVE_MM);
}

bool s5_wait_still(void)
{
	double		start = motion_hal_time();
	robot_status_t	status;

	while (motion_hal_time() - start < S5_SETTLE_TIMEOUT_S) {
		if (s5_status(&status) && status.motion.hold && status.motion.stationary &&
		    (status.motion.state != DRIVING) && (status.motion.state != STOPPING)) {
			return true;
		}
		(void)tk_dly_tsk(S5_POLL_MS);
	}
	return false;
}

bool s5_line_reporting(const robot_status_t *p_status, double now)
{
	return p_status->line.available && (now - p_status->line.timestamp <= S5_LINE_FRESH_S);
}

#if defined(ROBO_PRODUCTION_BUILD) && !defined(ROBO_IR_MONITOR) && ROBO_SUBSYSTEM5

typedef enum { S5_MOVE_DONE, S5_MOVE_LINE, S5_MOVE_REJOIN, S5_MOVE_FAILED, S5_MOVE_RUNNING,
	       S5_MOVE_STALLED } s5_move_t;

encoder_sample_t s5_log_encoders(const char *phase, uint32_t id)
{
	encoder_sample_t sample = motion_hal_sample();
	tm_printf((const UB *)"[OBSTACLE] enc phase=%s id=%u L=%lu R=%lu invalid=%lu,%lu reliable=%u\n",
		  phase, (unsigned)id, (unsigned long)sample.count[0], (unsigned long)sample.count[1],
		  (unsigned long)sample.invalid[0], (unsigned long)sample.invalid[1], (unsigned)sample.reliable);
	return sample;
}

static void log_encoder_after(uint32_t id, const encoder_sample_t *before, s5_move_t result)
{
	encoder_sample_t after = s5_log_encoders("move-after", id);
	/* encoder_delta returns the signed 32-bit modular difference. Use the
	   monitor's supported long format (it does not support long long). */
	tm_printf((const UB *)"[DETOUR] enc id=%u delta_L=%ld delta_R=%ld result=%s\n",
		  (unsigned)id, (long)encoder_delta(after.count[0], before->count[0]),
		  (long)encoder_delta(after.count[1], before->count[1]),
		  (result == S5_MOVE_DONE) ? "done" : (result == S5_MOVE_LINE) ? "line" :
		  (result == S5_MOVE_REJOIN) ? "rejoin" : (result == S5_MOVE_STALLED) ? "stalled" : "failed");
}

static uint32_t	s5_move_id = S5_ID_LAST;
static double	s5_heading0;	/* rad, Motion Control's heading at the stop */
static double	s5_x;		/* mm, the axle centre, detour_plan.h frame */
static double	s5_y;
static bool	s5_rejoin_armed;	/* pass and return: both sensors black interrupts the detour */
static bool	s5_rejoined;		/* the pass stage ended by handing back to the line */
static double	s5_rejoin_after_m;	/* armed, but no hand-back before this centre travel */
static double	s5_rejoin_centre_m;	/* centre travel to come back to if braking overshoots */
static bool	s5_line_touched;	/* return leg: one sensor has seen the line */
static double	s5_line_first_m;	/* centre travel where the sensors were first on it */
static double	s5_line_last_m;		/* and most recently */
static double	s5_stall_since = -1.0;	/* wheels first seen stopped under a target, or -1 */
static bool	s5_pass_stalled;	/* the pass ended against something: see escape_stall() */

static int s5_mm(double value)
{
	return (int)((value < 0.0) ? (value - 0.5) : (value + 0.5));
}

static void detour_stage(UW marker, const char *name)
{
	crash_mark(CRASH_MARK_OBSTACLE, marker);
	tm_printf((const UB *)"[DETOUR] stage=%s x=%d y=%d\n", name, s5_mm(s5_x), s5_mm(s5_y));
}

/* Buddy 3's values[1]: 1 while a sensor sees black (debounced) */
static bool line_seen(const robot_status_t *p_status, double now)
{
	return s5_line_reporting(p_status, now) && (p_status->line.values[1] > 0.5);
}

static double heading_deg(const robot_status_t *p_status)
{
	return (p_status->motion.heading - s5_heading0) / S5_DEG_TO_RAD;
}

static uint32_t next_id(void)
{
	s5_move_id = (s5_move_id >= S5_ID_LAST) ? S5_ID_FIRST : (s5_move_id + 1U);
	return s5_move_id;
}

/* Only the hold this task latched is released, and only once Motion
   Control reports the robot still; motion_release() checks that too. */
static bool release_hold(void)
{
	double		start;
	robot_status_t	status;

	if (!s5_wait_still()) {
		tm_printf((const UB *)"[DETOUR] robot not confirmed still; not releasing\n");
		return false;
	}
	motion_controller_release(true);
	start = motion_hal_time();
	while (motion_hal_time() - start < S5_RELEASE_TIMEOUT_S) {
		if (s5_status(&status) && !status.motion.hold && (status.motion.state == READY)) {
			return true;
		}
		(void)tk_dly_tsk(S5_POLL_MS);
	}
	tm_printf((const UB *)"[DETOUR] Motion Control did not release the hold\n");
	return false;
}

/* One check of move id.  Both sensors black, once armed, stops the robot at once.
   Watching the line, one sensor black where odometry expects the line does not:
   the sensors do not reach the line together, so it keeps going for the other one.
   If it comes off the line without both ever having been black, or the move ends,
   it stops then (*p_line_centre_m: centre travel in the middle of the black). */
static s5_move_t move_poll(uint32_t id, double start, double timeout_s, bool b_watch_line,
			   double centre0_m, double *p_line_centre_m)
{
	double		now = motion_hal_time();
	robot_status_t	status;

	if (!s5_status(&status)) {
		if (now - start > timeout_s) {
			motion_stop();
			return S5_MOVE_FAILED;
		}
		return S5_MOVE_RUNNING;
	}
	if (status.motion.hold || (status.motion.state == FAULT) ||
	    (status.motion.fault != F_NONE)) {
		tm_printf((const UB *)"[DETOUR] move %u interrupted: hold %u, state %u, fault %u\n",
			  (unsigned)id, (unsigned)status.motion.hold,
			  (unsigned)status.motion.state, (unsigned)status.motion.fault);
		return S5_MOVE_FAILED;
	}
	if ((status.last_event.id == id) && (status.last_event.type != EV_COMPLETE)) {
		tm_printf((const UB *)"[DETOUR] move %u ended with event %u, fault %u\n",
			  (unsigned)id, (unsigned)status.last_event.type,
			  (unsigned)status.last_event.fault);
		return S5_MOVE_FAILED;
	}
	if (s5_rejoin_armed && (status.motion.centre >= s5_rejoin_after_m) &&
	    line_rejoin_seen(&status, now)) {
		motion_stop();
		s5_rejoin_centre_m = status.motion.centre;
		return S5_MOVE_REJOIN;
	}
	if (b_watch_line) {
		double heading = heading_deg(&status);
		double y = s5_y + (status.motion.centre - centre0_m) * 1000.0 *
			   sin(heading * S5_DEG_TO_RAD);
		bool b_seen = line_seen(&status, now);

		if (b_seen && (s5_line_touched || s5_line_expected(y, heading))) {
			s5_line_first_m = s5_line_touched ? s5_line_first_m : status.motion.centre;
			s5_line_last_m = status.motion.centre;
			s5_line_touched = true;
		}
		/* A fresh line detection wins if it arrives with move completion.
		 * Failure/cancellation events above still take priority over it. */
		if (s5_line_touched && (!b_seen || (status.last_event.id == id))) {
			motion_stop();
			*p_line_centre_m = 0.5 * (s5_line_first_m + s5_line_last_m);
			return S5_MOVE_LINE;
		}
	}
	if ((status.last_event.id != id) && (status.motion.state == DRIVING) &&
	    (fabs(status.motion.speed[0]) < S5_STALL_SPEED_M_S) &&
	    (fabs(status.motion.speed[1]) < S5_STALL_SPEED_M_S) &&
	    (fabs(status.motion.target[0]) + fabs(status.motion.target[1]) > S5_STALL_TARGET_M_S)) {
		if (s5_stall_since < 0.0) {
			s5_stall_since = now;
		} else if (now - s5_stall_since >= S5_STALL_S) {
			motion_stop();
			tm_printf((const UB *)"[DETOUR] move %u stalled: wheels not turning for %d ms with %d mm/s asked\n",
				  (unsigned)id, s5_mm((now - s5_stall_since) * 1000.0),
				  s5_mm(500.0 * (fabs(status.motion.target[0]) + fabs(status.motion.target[1]))));
			return S5_MOVE_STALLED;
		}
	} else {
		s5_stall_since = -1.0;
	}
	if (status.last_event.id == id) { return S5_MOVE_DONE; }
	if ((now - start > S5_ACCEPT_TIMEOUT_S) && (status.motion.id != id)) {
		tm_printf((const UB *)"[DETOUR] move %u was not accepted (state %u, owner %u)\n",
			  (unsigned)id, (unsigned)status.motion.state, (unsigned)status.owner);
		return S5_MOVE_FAILED;
	}
	if (now - start > timeout_s) {
		motion_stop();
		tm_printf((const UB *)"[DETOUR] move %u timed out\n", (unsigned)id);
		return S5_MOVE_FAILED;
	}
	return S5_MOVE_RUNNING;
}

static s5_move_t wait_move(uint32_t id, double timeout_s, bool b_watch_line,
			   double centre0_m, double *p_line_centre_m)
{
	double		start = motion_hal_time();
	s5_move_t	move;

	s5_line_touched = false;
	s5_stall_since = -1.0;
	do {
		(void)tk_dly_tsk(S5_POLL_MS);
		move = move_poll(id, start, timeout_s, b_watch_line, centre0_m, p_line_centre_m);
	} while (move == S5_MOVE_RUNNING);
	return move;
}

/* R_BUSY means the nonblocking command-bus lock was unavailable; no command
 * was queued. Retry only that case, with a deadline, and never across a stop,
 * reset, fault, or another command starting. R_OK is submitted only once. */
static bool retry_busy(double start, uint32_t generation)
{
	robot_status_t status;
	if (motion_hal_time() - start >= S5_ACCEPT_TIMEOUT_S) { return false; }
	(void)tk_dly_tsk(S5_POLL_MS);
	return (motion_hal_time() - start < S5_ACCEPT_TIMEOUT_S) && s5_status(&status) &&
	       !status.motion.hold && (status.motion.state == READY) &&
	       (status.motion.fault == F_NONE) && (status.motion.generation == generation);
}

/* Straight at S5_DRIVE_SPEED_MM_S (S5_LINE_SEARCH_SPEED_MM_S when watching
   for the line), negative = reverse.  The pose follows
   what the encoders measured, not what was asked for. */
static s5_move_t straight(double mm, bool b_watch_line, double *p_line_centre_m)
{
	robot_status_t	before;
	robot_status_t	after;
	encoder_sample_t counts_before;
	uint32_t	id;
	motion_result_t	result;
	s5_move_t	move;
	double		submit_start;
	double		speed_mm_s = b_watch_line ? S5_LINE_SEARCH_SPEED_MM_S : S5_DRIVE_SPEED_MM_S;

	if (fabs(mm) < S5_MIN_MOVE_MM) {
		return S5_MOVE_DONE;
	}
	if (!s5_status(&before)) {
		return S5_MOVE_FAILED;
	}
	id = next_id();
	submit_start = motion_hal_time();
	counts_before = s5_log_encoders("move-before", id);
	do {
		result = motion_steer_distance(SOURCE_OBSTACLE, id, mm / 1000.0, 0.0,
					       speed_mm_s / 1000.0);
	} while ((result == R_BUSY) && retry_busy(submit_start, before.motion.generation));
	if (result != R_OK) {
		tm_printf((const UB *)"[DETOUR] drive %d mm refused (%u)\n", s5_mm(mm), (unsigned)result);
		log_encoder_after(id, &counts_before, S5_MOVE_FAILED);
		return S5_MOVE_FAILED;
	}
	tm_printf((const UB *)"[DETOUR] move %u queued: straight %d mm at %d mm/s\n",
		  (unsigned)id, s5_mm(mm), s5_mm(speed_mm_s));
	move = wait_move(id, fabs(mm) / speed_mm_s + S5_MOVE_SLACK_S,
			 b_watch_line, before.motion.centre, p_line_centre_m);
	if ((move == S5_MOVE_LINE) || (move == S5_MOVE_REJOIN) || (move == S5_MOVE_STALLED)) {
		if (!s5_wait_still()) {
			tm_printf((const UB *)"[DETOUR] move %u could not confirm line stop\n", (unsigned)id);
			log_encoder_after(id, &counts_before, S5_MOVE_FAILED);
			return S5_MOVE_FAILED;
		}
	}
	log_encoder_after(id, &counts_before, move);
	if (!s5_status(&after)) {
		motion_stop();
		tm_printf((const UB *)"[DETOUR] move %u lost final position; holding\n", (unsigned)id);
		return S5_MOVE_FAILED;
	}
	{
		double travelled = (after.motion.centre - before.motion.centre) * 1000.0;
		double heading = 0.5 * (heading_deg(&before) + heading_deg(&after)) * S5_DEG_TO_RAD;

		s5_x += travelled * cos(heading);
		s5_y += travelled * sin(heading);
	}
	if (move == S5_MOVE_DONE) {
		tm_printf((const UB *)"[DETOUR] move %u complete: straight, drive took %d ms\n",
			  (unsigned)id, s5_mm((motion_hal_time() - submit_start) * 1000.0));
	}
	return move;
}

/* Turn in place to a heading relative to the one at the stop, so turn
   errors do not add up over the detour. */
static s5_move_t turn_to(double target_deg)
{
	robot_status_t	status;
	encoder_sample_t counts_before;
	uint32_t	id;
	motion_result_t	result;
	s5_move_t	move;
	double		turn;
	double		submit_start;

	if (!s5_status(&status)) {
		return S5_MOVE_FAILED;
	}
	turn = target_deg - heading_deg(&status);
	if (fabs(turn) < S5_MIN_TURN_DEG) {
		return S5_MOVE_DONE;
	}
	id = next_id();
	submit_start = motion_hal_time();
	counts_before = s5_log_encoders("move-before", id);
	do {
		result = motion_turn_degrees(SOURCE_OBSTACLE, id, turn);
	} while ((result == R_BUSY) && retry_busy(submit_start, status.motion.generation));
	if (result != R_OK) {
		tm_printf((const UB *)"[DETOUR] turn %d deg refused (%u)\n", s5_mm(turn), (unsigned)result);
		log_encoder_after(id, &counts_before, S5_MOVE_FAILED);
		return S5_MOVE_FAILED;
	}
	tm_printf((const UB *)"[DETOUR] move %u queued: turn %d deg to heading %d deg\n",
		  (unsigned)id, s5_mm(turn), s5_mm(target_deg));
	move = wait_move(id, S5_TURN_TIMEOUT_S, false, 0.0, NULL);
	log_encoder_after(id, &counts_before, move);
	if (move == S5_MOVE_DONE) {
		tm_printf((const UB *)"[DETOUR] move %u complete: turn\n", (unsigned)id);
	}
	return move;
}

static bool detour_ping(void *p_context, double *p_range_mm)
{
	double time_s;
	bool *p_failed = p_context;
	s5_us_status_t result;

	if (*p_failed) { return false; }
	result = s5_us_ping(p_range_mm, &time_s);
	*p_failed = (result == S5_US_NO_PULSE) || (result == S5_US_BUSY);
	return result == S5_US_OK;
}

typedef enum { S5_LOOK_ECHO, S5_LOOK_CLEAR, S5_LOOK_FAILED, S5_LOOK_UNSURE } s5_look_t;

/* Measurements are only useful while still at the same command generation.
   In particular, never resume a detour across a user stop/reset during a sweep.
   READY with no move active means the motors are off: see s5_still_moved(). */
static bool look_still(uint32_t generation)
{
	robot_status_t status;
	return s5_status(&status) && !status.motion.hold && !s5_still_moved(&status) &&
	       (status.motion.state == READY) && (status.motion.fault == F_NONE) &&
	       (status.motion.generation == generation);
}

static s5_look_t look_failed(int angle_deg, const char *p_why, const s5_estimate_t *p_estimate)
{
	robot_status_t now;
	bool b_now = s5_status(&now);

	tm_printf((const UB *)"[DETOUR] look at %d deg failed: %s (hold %u, state %u, fault %u, moved %u, "
		  "pings %u of %u, %u agree)\n", angle_deg, p_why,
		  b_now ? (unsigned)now.motion.hold : 9U, b_now ? (unsigned)now.motion.state : 9U,
		  b_now ? (unsigned)now.motion.fault : 9U, (b_now && s5_still_moved(&now)) ? 1U : 0U,
		  (p_estimate != NULL) ? p_estimate->valid : 0U, (p_estimate != NULL) ? p_estimate->total : 0U,
		  (p_estimate != NULL) ? p_estimate->support : 0U);
	return S5_LOOK_FAILED;
}

/* A silent but responding sensor differs from a disconnected/stuck one.
   Do not turn sensor failures or conflicting readings into permission to drive. */
static s5_look_t look(int angle_deg, double *p_range_mm)
{
	s5_estimate_t estimate;
	bool failed = false;
	robot_status_t status;

	s5_still_mark();
	if (!s5_status(&status) || !look_still(status.motion.generation)) {
		return look_failed(angle_deg, "not still before the look", NULL);
	}
	s5_servo_move(angle_deg);
	if (!look_still(status.motion.generation)) {
		return look_failed(angle_deg, "not still after the servo", NULL);
	}
	s5_sample_point(detour_ping, &failed, &estimate);
	*p_range_mm = estimate.range_mm;
	if (failed) { return look_failed(angle_deg, "ultrasonic fault", &estimate); }
	if (!look_still(status.motion.generation)) {
		return look_failed(angle_deg, "not still after the pings", &estimate);
	}
	if (s5_estimate_usable(&estimate)) { return S5_LOOK_ECHO; }
	if (estimate.status == S5_ECHO_NONE) { return S5_LOOK_CLEAR; }
	return look_failed(angle_deg, "pings disagree", &estimate);
}

/* Nothing ahead within the move plus the front overhang and the margin */
static bool path_clear(double distance_mm)
{
	double range_mm;
	double needed_mm = distance_mm + (S5_FRONT_MM - S5_SERVO_AHEAD_MM) + S5_PASS_MARGIN_MM;
	s5_look_t result = look(S5_ANGLE_CENTRE_DEG, &range_mm);

	if (result == S5_LOOK_FAILED) {
		tm_printf((const UB *)"[DETOUR] front measurement failed; not moving\n");
		return false;
	}
	if ((result == S5_LOOK_ECHO) &&
	    (s5_forward_mm(range_mm, S5_ANGLE_CENTRE_DEG) <= needed_mm)) {
		tm_printf((const UB *)"[DETOUR] something %d mm ahead, need %d mm\n",
			  s5_mm(s5_forward_mm(range_mm, S5_ANGLE_CENTRE_DEG)), s5_mm(needed_mm));
		return false;
	}
	return true;
}

/* The five bearings cover a corridor, not just the centre ray. Widen it for
   the beam's uncertain echo bearing; normal no-echo permits progress, while
   an unusable sensor or a close echo refuses the move. Distances include the
   sensor's offset from the servo axis, as in the original scan. */
static bool	s5_sweep_blocked;	/* the last clearance_sweep failed on an echo, not a fault */

static bool clearance_sweep(double distance_mm, const char *phase)
{
	double needed = distance_mm + S5_FRONT_MM - S5_SERVO_AHEAD_MM + S5_PASS_MARGIN_MM;
	double half_width = S5_ROBOT_HALF_WIDTH_MM + S5_PASS_MARGIN_MM;
	robot_status_t status;
	double start_s = motion_hal_time();
	int angle;

	s5_sweep_blocked = false;
	s5_still_mark();
	if (!s5_status(&status) || !look_still(status.motion.generation)) { return false; }
	tm_printf((const UB *)"[DETOUR] clearance=%s sweep +/- %d deg, travel %d mm\n",
		  phase, S5_GUARD_SWEEP_DEG, s5_mm(distance_mm));
	for (angle = S5_ANGLE_CENTRE_DEG - S5_GUARD_SWEEP_DEG;
	     angle <= S5_ANGLE_CENTRE_DEG + S5_GUARD_SWEEP_DEG;
	     angle += S5_GUARD_SWEEP_STEP_DEG) {
		double range;
		s5_look_t result;
		if (!look_still(status.motion.generation)) { return false; }
		result = look(angle, &range);

		if ((result == S5_LOOK_FAILED) || !look_still(status.motion.generation)) {
			robot_status_t now;
			bool b_now = s5_status(&now);

			tm_printf((const UB *)"[DETOUR] clearance=%s measurement failed at %d deg; holding "
				  "(look %u, hold %u, state %u, fault %u, moved %u)\n",
				  phase, angle, (unsigned)result,
				  b_now ? (unsigned)now.motion.hold : 9U, b_now ? (unsigned)now.motion.state : 9U,
				  b_now ? (unsigned)now.motion.fault : 9U, (b_now && s5_still_moved(&now)) ? 1U : 0U);
			return false;
		}
		if (result == S5_LOOK_ECHO) {
			double axis_range = range + S5_SENSOR_OFFSET_MM;
			double bearing = fabs((double)(angle - S5_ANGLE_CENTRE_DEG));
			double near_bearing = fmax(0.0, bearing - S5_GUARD_BEAM_HALF_DEG);
			double far_bearing = fmin(90.0, bearing + S5_GUARD_BEAM_HALF_DEG);
			double min_forward = axis_range * cos(far_bearing * S5_DEG_TO_RAD);
			double min_lateral = axis_range * sin(near_bearing * S5_DEG_TO_RAD);

			if ((min_forward <= needed) && (min_lateral <= half_width)) {
				tm_printf((const UB *)"[DETOUR] clearance=%s blocked at %d deg, range %d mm\n",
					  phase, angle, s5_mm(range));
				s5_sweep_blocked = true;
				return false;
			}
		}
	}
	/* The centre bearing of the sweep already covered the path ahead (its
	   near bearing is 0 deg, so any echo within the move is refused), so no
	   second centre look is needed. */
	if (!look_still(status.motion.generation)) { return false; }
	tm_printf((const UB *)"[DETOUR] clearance=%s clear, sweep took %d ms\n",
		  phase, s5_mm((motion_hal_time() - start_s) * 1000.0));
	return true;
}

static int obstacle_look_angle(const s5_detour_plan_t *p_plan)
{
	return (p_plan->side == S5_AVOID_LEFT) ? S5_ANGLE_RIGHT_DEG : S5_ANGLE_LEFT_DEG;
}

static bool passing_clearance(const s5_detour_plan_t *p_plan)
{
	double range;
	s5_look_t result;

	if (!clearance_sweep(S5_PASS_STEP_MM, "parallel")) { return false; }
	result = look(obstacle_look_angle(p_plan), &range);
	if (result == S5_LOOK_FAILED) {
		tm_printf((const UB *)"[DETOUR] obstacle-side clearance measurement failed; holding\n");
		return false;
	}
	if (result == S5_LOOK_ECHO) {
		double gap = fabs(s5_lateral_mm(range, (double)obstacle_look_angle(p_plan)));
		double needed = S5_ROBOT_HALF_WIDTH_MM + S5_PASS_MARGIN_MM;
		if (gap < needed) {
			tm_printf((const UB *)"[DETOUR] insufficient sideways clearance: %d mm, need %d mm; holding\n",
				  s5_mm(gap), s5_mm(needed));
			return false;
		}
	}
	return true;
}

static bool go_out(const s5_detour_plan_t *p_plan)
{
	double remaining = p_plan->diagonal_mm + S5_DIAGONAL_EXTRA_MM;
	double heading_deg = p_plan->turn_deg;
	double heading = heading_deg * S5_DEG_TO_RAD;
	double sign = (p_plan->turn_deg < 0.0) ? -1.0 : 1.0;
	detour_stage(CRASH_OBSTACLE_OUT, "out");
	tm_printf((const UB *)"[DETOUR] out: back %d mm, turn %d deg, diagonal %d mm -> %d mm %s of the line\n",
		  s5_mm(p_plan->backup_mm), s5_mm(p_plan->turn_deg), s5_mm(p_plan->diagonal_mm),
		  s5_mm(fabs(p_plan->shift_mm)), (p_plan->shift_mm >= 0.0) ? "left" : "right");
	if ((straight(-p_plan->backup_mm, false, NULL) != S5_MOVE_DONE) ||
	    (turn_to(p_plan->turn_deg) != S5_MOVE_DONE)) { return false; }
	/* The diagonal is the commanded distance: no compensation for a move
	   that finishes a few mm short.  A remainder below S5_MIN_DIAGONAL_STEP_MM
	   is too short for the wheels to move at all, so it is dropped. */
	while (remaining >= S5_MIN_DIAGONAL_STEP_MM) {
		double step = fmin(remaining, S5_DIAGONAL_STEP_MM);
		double before_x = s5_x;
		double before_y = s5_y;
		double progress;
		/* Do not leave an undrivable tail: share the last two steps. */
		if ((remaining > S5_DIAGONAL_STEP_MM) &&
		    (remaining - S5_DIAGONAL_STEP_MM < S5_MIN_DIAGONAL_STEP_MM)) {
			step = 0.5 * remaining;
		}
		while (!clearance_sweep(step, "diagonal")) {
			/* Blocked: turn further out and look again.  A fault is not blocked. */
			double steeper = heading_deg + sign * S5_OUT_RETRY_STEP_DEG;

			if (!s5_sweep_blocked || (fabs(steeper) > S5_OUT_RETRY_MAX_DEG + 1e-9)) {
				tm_printf((const UB *)"[DETOUR] diagonal not clear at %d deg; holding\n", s5_mm(heading_deg));
				return false;
			}
			tm_printf((const UB *)"[DETOUR] turning further out: %d deg\n", s5_mm(steeper));
			if (turn_to(steeper) != S5_MOVE_DONE) { return false; }
			/* Same sideways shift in a shorter diagonal */
			remaining *= sin(fabs(heading_deg) * S5_DEG_TO_RAD) / sin(fabs(steeper) * S5_DEG_TO_RAD);
			heading_deg = steeper;
			heading = heading_deg * S5_DEG_TO_RAD;
			step = fmin(remaining, S5_DIAGONAL_STEP_MM);
		}
		if (straight(step, false, NULL) != S5_MOVE_DONE) { return false; }
		progress = (s5_x - before_x) * cos(heading) + (s5_y - before_y) * sin(heading);
		if (progress < 0.5) {
			tm_printf((const UB *)"[DETOUR] diagonal made no forward progress; holding\n");
			return false;
		}
		remaining -= step;
	}
	return (turn_to(0.0) == S5_MOVE_DONE) && passing_clearance(p_plan);
}

/* Wait for a sensor on the line, debounced, while stationary */
static bool line_confirm(void)
{
	double		start = motion_hal_time();
	robot_status_t	status;

	while (motion_hal_time() - start < S5_LINE_CONFIRM_S) {
		if (s5_status(&status) && line_seen(&status, motion_hal_time())) {
			return true;
		}
		(void)tk_dly_tsk(S5_POLL_MS);
	}
	return false;
}

/* Already standing on the line, not held: nothing to stop or back out of */
/* A move was stopped on the line and left the hold latched.  Braking can carry
   the sensors past it, and the line follower does not search for a line it
   never owned, so reverse onto it if it is no longer there. */
static bool rejoin_after_stop(void)
{
	robot_status_t	status;
	double		overshoot_mm;

	s5_rejoin_armed = false;
	if (!release_hold() || !s5_status(&status)) {
		return false;
	}
	overshoot_mm = (status.motion.centre - s5_rejoin_centre_m) * 1000.0;
	tm_printf((const UB *)"[DETOUR] line at x %d y %d, heading %d deg, stopped %d mm on; interrupting the detour\n",
		  s5_mm(s5_x), s5_mm(s5_y), s5_mm(heading_deg(&status)), s5_mm(overshoot_mm));
	if (!line_confirm() &&
	    ((straight(-fmax(overshoot_mm, S5_MIN_DIAGONAL_STEP_MM), false, NULL) != S5_MOVE_DONE) ||
	     !line_confirm())) {
		tm_printf((const UB *)"[DETOUR] line not under the sensors after stopping; holding\n");
		return false;
	}
	return true;
}

/* ---- the pass: one long move, measured while it runs ---- */

typedef struct {
	uint32_t	id;
	double		start;
	double		timeout_s;
	double		centre_m;	/* s5_x and s5_y are up to date for this travel */
	double		heading_deg;	/* and this heading */
	encoder_sample_t counts;
	s5_move_t	result;		/* S5_MOVE_RUNNING until the move is over */
	bool		b_sensor_failed;
} s5_drive_t;

static void drive_track(s5_drive_t *p_drive, const robot_status_t *p_status)
{
	double heading = heading_deg(p_status);
	double middle = 0.5 * (p_drive->heading_deg + heading) * S5_DEG_TO_RAD;
	double travelled = (p_status->motion.centre - p_drive->centre_m) * 1000.0;

	s5_x += travelled * cos(middle);
	s5_y += travelled * sin(middle);
	p_drive->centre_m = p_status->motion.centre;
	p_drive->heading_deg = heading;
}

static bool drive_start(s5_drive_t *p_drive, double mm)
{
	robot_status_t	before;
	motion_result_t	result;
	double		submit_start;

	if (!s5_status(&before)) {
		return false;
	}
	p_drive->id = next_id();
	submit_start = motion_hal_time();
	p_drive->counts = s5_log_encoders("move-before", p_drive->id);
	do {
		result = motion_steer_distance(SOURCE_OBSTACLE, p_drive->id, mm / 1000.0, 0.0,
					       S5_DRIVE_SPEED_MM_S / 1000.0);
	} while ((result == R_BUSY) && retry_busy(submit_start, before.motion.generation));
	if (result != R_OK) {
		tm_printf((const UB *)"[DETOUR] drive %d mm refused (%u)\n", s5_mm(mm), (unsigned)result);
		log_encoder_after(p_drive->id, &p_drive->counts, S5_MOVE_FAILED);
		return false;
	}
	tm_printf((const UB *)"[DETOUR] move %u queued: pass, up to %d mm at %d mm/s\n",
		  (unsigned)p_drive->id, s5_mm(mm), s5_mm(S5_DRIVE_SPEED_MM_S));
	p_drive->start = motion_hal_time();
	p_drive->timeout_s = mm / S5_DRIVE_SPEED_MM_S + S5_MOVE_SLACK_S;
	p_drive->centre_m = before.motion.centre;
	p_drive->heading_deg = heading_deg(&before);
	p_drive->result = S5_MOVE_RUNNING;
	p_drive->b_sensor_failed = false;
	s5_stall_since = -1.0;
	return true;
}

static s5_move_t drive_poll(s5_drive_t *p_drive)
{
	robot_status_t status;

	if (p_drive->result == S5_MOVE_RUNNING) {
		p_drive->result = move_poll(p_drive->id, p_drive->start, p_drive->timeout_s,
					    false, 0.0, NULL);
		if (s5_status(&status)) {
			drive_track(p_drive, &status);
		}
	}
	return p_drive->result;
}

static void drive_wait(s5_drive_t *p_drive, unsigned ms)
{
	double end = motion_hal_time() + (double)ms / 1000.0;

	while ((motion_hal_time() < end) && (drive_poll(p_drive) == S5_MOVE_RUNNING)) {
		(void)tk_dly_tsk(S5_POLL_MS);
	}
}

/* Between pings, so that a fault or the line is seen within one ping (60 ms) */
static bool drive_ping(void *p_context, double *p_range_mm)
{
	s5_drive_t *p_drive = p_context;
	double time_s;
	s5_us_status_t result;

	if ((drive_poll(p_drive) != S5_MOVE_RUNNING) || p_drive->b_sensor_failed) {
		return false;
	}
	result = s5_us_ping(p_range_mm, &time_s);
	p_drive->b_sensor_failed = (result == S5_US_NO_PULSE) || (result == S5_US_BUSY);
	return result == S5_US_OK;
}

/* *p_x: where the robot was, halfway through the pings.  Pings that disagree
   are S5_LOOK_UNSURE, normal while passing an edge: stopping for them would
   abandon detours, and the next look is about 100 mm on. */
static s5_look_t drive_look(s5_drive_t *p_drive, int angle_deg, double *p_range_mm, double *p_x)
{
	s5_estimate_t	estimate;
	double		first_x;

	drive_wait(p_drive, s5_servo_aim(angle_deg));
	first_x = s5_x;
	s5_sample_point(drive_ping, p_drive, &estimate);
	*p_x = 0.5 * (first_x + s5_x);
	*p_range_mm = estimate.range_mm;
	if ((p_drive->result != S5_MOVE_RUNNING) || p_drive->b_sensor_failed) {
		return S5_LOOK_FAILED;
	}
	if (s5_estimate_usable(&estimate)) { return S5_LOOK_ECHO; }
	return (estimate.status == S5_ECHO_NONE) ? S5_LOOK_CLEAR : S5_LOOK_UNSURE;
}

/* Wait for the stop, then log and bring the pose up to date.  Leaves it held. */
static bool drive_settle(s5_drive_t *p_drive)
{
	robot_status_t status;

	if (!s5_wait_still() || !s5_status(&status)) {
		tm_printf((const UB *)"[DETOUR] move %u could not confirm the stop\n", (unsigned)p_drive->id);
		return false;
	}
	drive_track(p_drive, &status);
	log_encoder_after(p_drive->id, &p_drive->counts, p_drive->result);
	return true;
}

/* The move ended by itself: the line interrupted it, or something went wrong */
static bool drive_ended(s5_drive_t *p_drive)
{
	if (p_drive->result == S5_MOVE_REJOIN) {
		s5_rejoined = drive_settle(p_drive) && rejoin_after_stop();
		return s5_rejoined;
	}
	if (p_drive->result == S5_MOVE_STALLED) {
		s5_pass_stalled = drive_settle(p_drive);
		return false;
	}
	if (p_drive->result == S5_MOVE_DONE) {
		tm_printf((const UB *)"[DETOUR] drove the whole pass without the obstacle ending\n");
	}
	return false;
}

/* Sensor failed, or something ahead: the caller stops too, but this is moving */
static bool drive_refuse(const char *p_reason, int value_mm)
{
	motion_stop();
	tm_printf((const UB *)"[DETOUR] %s (%d mm) while passing; holding\n", p_reason, value_mm);
	return false;
}

/* True: go on.  An unsure look ahead counts if its nearest agreeing echo is close. */
static bool drive_look_ahead(s5_drive_t *p_drive)
{
	double		range_mm;
	double		look_x;
	double		needed = S5_PASS_STEP_MM + S5_FRONT_MM - S5_SERVO_AHEAD_MM + S5_PASS_MARGIN_MM;
	s5_look_t	result = drive_look(p_drive, S5_ANGLE_CENTRE_DEG, &range_mm, &look_x);

	if (p_drive->result != S5_MOVE_RUNNING) {
		return false;
	}
	if (result == S5_LOOK_FAILED) {
		return drive_refuse("front measurement failed", 0);
	}
	if ((result != S5_LOOK_CLEAR) &&
	    (s5_forward_mm(range_mm, S5_ANGLE_CENTRE_DEG) <= needed)) {
		return drive_refuse("something ahead", s5_mm(s5_forward_mm(range_mm, S5_ANGLE_CENTRE_DEG)));
	}
	return true;
}

static bool pass_steps(const s5_detour_plan_t *p_plan)
{
	int		side_deg = obstacle_look_angle(p_plan);
	double		start_x = s5_x;
	double		clear_x;
	double		range_mm;
	double		look_x;
	s5_look_t	result;
	s5_pass_t	pass;
	s5_drive_t	drive;

	detour_stage(CRASH_OBSTACLE_PASS, "pass");
	s5_pass_init(&pass, p_plan);
	if (!drive_start(&drive, S5_PASS_MAX_MM + hypot(S5_REAR_MM, S5_ROBOT_HALF_WIDTH_MM) +
			 S5_PASS_MARGIN_MM + S5_PASS_STEP_MM)) {
		return false;
	}
	for (;;) {
		result = drive_look(&drive, side_deg, &range_mm, &look_x);
		if (drive.result != S5_MOVE_RUNNING) {
			return drive_ended(&drive);
		}
		if (result == S5_LOOK_FAILED) {
			return drive_refuse("side measurement failed", 0);
		}
		if (result == S5_LOOK_UNSURE) {
			tm_printf((const UB *)"[DETOUR] pass x %d y %d: side pings disagree, ignored\n",
				  s5_mm(look_x), s5_mm(s5_y));
		} else {
			(void)s5_pass_observe(&pass, p_plan, look_x, s5_y, result == S5_LOOK_ECHO, range_mm);
			tm_printf((const UB *)"[DETOUR] pass x %d y %d: side %d mm -> obstacle to x %d, clear %u/%u\n",
				  s5_mm(look_x), s5_mm(s5_y), (result == S5_LOOK_ECHO) ? s5_mm(range_mm) : 0,
				  s5_mm(pass.rear_mm), pass.clear_run, S5_PASS_CONFIRM);
		}
		if (pass.b_ended) {
			break;
		}
		if (s5_x - start_x >= S5_PASS_MAX_MM) {
			return drive_refuse("obstacle has not ended", s5_mm(S5_PASS_MAX_MM));
		}
		if (!drive_look_ahead(&drive)) {
			return (drive.result != S5_MOVE_RUNNING) ? drive_ended(&drive) : false;
		}
	}
	clear_x = s5_pass_clear_x(&pass);
	detour_stage(CRASH_OBSTACLE_REAR_CLEAR, "rear-clearance");
	tm_printf((const UB *)"[DETOUR] obstacle ends by x %d (%u echoes); rear clears it at x %d\n",
		  s5_mm(fmax(pass.rear_mm, pass.end_mm)), pass.seen, s5_mm(clear_x));
	/* A look ahead costs about 50 mm of travel: only worth it for a long way */
	if ((clear_x - s5_x > S5_PASS_STEP_MM) && !drive_look_ahead(&drive)) {
		return (drive.result != S5_MOVE_RUNNING) ? drive_ended(&drive) : false;
	}
	while (s5_x < clear_x) {
		if (drive_poll(&drive) != S5_MOVE_RUNNING) {
			return drive_ended(&drive);
		}
		(void)tk_dly_tsk(S5_POLL_MS);
	}
	motion_stop();
	drive.result = S5_MOVE_DONE;	/* stopped on purpose: the move's job is done */
	return drive_settle(&drive) && release_hold();
}

/* The pass stopped against something (the held robot is still).  Tilt away from
   the obstacle, a step more each time, look that way, back out along it, and
   turn parallel again so that the pass can start over from further out.
   True: ready to resume. */
static bool escape_stall(const s5_detour_plan_t *p_plan, unsigned attempt)
{
	double		sign = (p_plan->side == S5_AVOID_LEFT) ? 1.0 : -1.0;
	unsigned	step;
	bool		b_held = true;

	for (step = attempt + 1U; step <= S5_ESCAPE_TRIES; step++) {
		double		tilt = sign * S5_ESCAPE_STEP_DEG * (double)step;
		s5_move_t	move;

		tm_printf((const UB *)"[DETOUR] stalled at x %d y %d: tilting %s to %d deg\n", s5_mm(s5_x), s5_mm(s5_y),
			  (sign > 0.0) ? "left" : "right", s5_mm(tilt));
		if (b_held && !release_hold()) { return false; }
		b_held = false;
		if (turn_to(tilt) != S5_MOVE_DONE) { return false; }
		if (!clearance_sweep(S5_ESCAPE_MM, "escape")) {
			if (!s5_sweep_blocked) { return false; }
			continue;		/* blocked that way: tilt further */
		}
		move = straight(S5_ESCAPE_MM, false, NULL);
		if (move == S5_MOVE_STALLED) {
			b_held = true;		/* stuck again: tilt further */
			continue;
		}
		return (move == S5_MOVE_DONE) && (turn_to(0.0) == S5_MOVE_DONE);
	}
	tm_printf((const UB *)"[DETOUR] no way out of the stall; holding\n");
	motion_stop();
	return false;
}

/* The line can interrupt this and the return leg, but not the way out, nor a
   stall escape: only once parallel again and S5_REJOIN_AFTER_MM further on. */
static bool pass_obstacle(const s5_detour_plan_t *p_plan)
{
	bool		b_ok;
	unsigned	attempt = 0U;

	s5_rejoined = false;
	for (;;) {
		robot_status_t status;

		if (!s5_status(&status)) {
			return false;
		}
		s5_rejoin_after_m = status.motion.centre + S5_REJOIN_AFTER_MM / 1000.0;
		s5_rejoin_armed = true;
		s5_pass_stalled = false;
		b_ok = pass_steps(p_plan);
		s5_rejoin_armed = false;
		if (b_ok || !s5_pass_stalled || (attempt >= S5_ESCAPE_TRIES) ||
		    !escape_stall(p_plan, attempt)) {
			break;
		}
		attempt++;
	}
	return b_ok;
}

static bool return_to_line(const s5_detour_plan_t *p_plan)
{
	double		line_centre_m = 0.0;
	s5_move_t	move;

	detour_stage(CRASH_OBSTACLE_RETURN_TURN, "return-turn");
	s5_servo_move(S5_ANGLE_CENTRE_DEG);
	tm_printf((const UB *)"[DETOUR] back: turn %d deg, up to %d mm to the line\n",
		  s5_mm(-S5_RETURN_DEG * ((p_plan->side == S5_AVOID_LEFT) ? 1.0 : -1.0)),
		  s5_mm(s5_return_length(s5_y)));
	if (turn_to((p_plan->side == S5_AVOID_LEFT) ? -S5_RETURN_DEG : S5_RETURN_DEG) != S5_MOVE_DONE) {
		return false;
	}
	detour_stage(CRASH_OBSTACLE_RETURN_LOOK, "return-path-check");
	if (!path_clear(s5_return_length(s5_y))) {
		return false;
	}
	detour_stage(CRASH_OBSTACLE_RETURN_DRIVE, "return-to-line");
	s5_rejoin_after_m = -INFINITY;
	s5_rejoin_armed = true;
	move = straight(s5_return_length(s5_y), true, &line_centre_m);
	s5_rejoin_armed = false;
	switch (move) {
	case S5_MOVE_REJOIN:
		break;
	case S5_MOVE_LINE:
		s5_rejoin_centre_m = line_centre_m;
		break;
	case S5_MOVE_DONE:
		tm_printf((const UB *)"[DETOUR] line not found where expected (now %d mm off it)\n", s5_mm(s5_y));
		return false;
	default:
		return false;
	}
	detour_stage(CRASH_OBSTACLE_ALIGN, "line-handover");
	return rejoin_after_stop();
}

/* Held, still and fully STOPPED, the state the scan requires.  s5_wait_still()
   also accepts READY, which Motion Control reports for a few ms after
   motion_stop() on a robot that was already still. */
static bool wait_stopped(void)
{
	double		start = motion_hal_time();
	robot_status_t	status;

	while (motion_hal_time() - start < S5_SETTLE_TIMEOUT_S) {
		if (s5_status(&status) && status.motion.hold && status.motion.stationary &&
		    (status.motion.state == STOPPED) && (status.motion.fault == F_NONE)) {
			return true;
		}
		(void)tk_dly_tsk(S5_POLL_MS);
	}
	return false;
}

bool s5_back_off(double mm)
{
	robot_status_t status;

	if (!release_hold() || !s5_status(&status)) {
		return false;
	}
	s5_heading0 = status.motion.heading;
	s5_x = 0.0;
	s5_y = 0.0;
	if (straight(-mm, false, NULL) != S5_MOVE_DONE) {
		motion_stop();
		return false;
	}
	motion_stop();
	return wait_stopped();
}

bool s5_square_up(double degrees, double *p_turned_deg)
{
	robot_status_t	before;
	robot_status_t	after;
	s5_move_t	move;

	if (!release_hold() || !s5_status(&before)) {
		return false;
	}
	s5_heading0 = before.motion.heading;
	move = turn_to(degrees);
	motion_stop();
	if (((move != S5_MOVE_DONE) && (move != S5_MOVE_STALLED)) || !wait_stopped()) {
		return false;
	}
	(void)tk_dly_tsk(S5_SQUARE_SETTLE_MS);
	if (!wait_stopped() || !s5_status(&after)) {
		return false;
	}
	*p_turned_deg = (after.motion.heading - before.motion.heading) / S5_DEG_TO_RAD;
	return true;
}

bool s5_detour_run(const s5_detour_plan_t *p_plan)
{
	robot_status_t status;

	if (!p_plan->b_ok || !release_hold() || !s5_status(&status)) {
		return false;
	}
	/* Heading 0 is square to the face: every turn from here is relative to it */
	s5_heading0 = status.motion.heading + p_plan->tilt_deg * S5_DEG_TO_RAD;
	s5_x = 0.0;
	s5_y = p_plan->start_y_mm;
	if (!go_out(p_plan) || !pass_obstacle(p_plan) || (!s5_rejoined && !return_to_line(p_plan))) {
		tm_printf((const UB *)"[DETOUR] gave up at x %d y %d, heading %d deg\n",
			  s5_mm(s5_x), s5_mm(s5_y), s5_status(&status) ? s5_mm(heading_deg(&status)) : 0);
		return false;
	}
	tm_printf((const UB *)"[DETOUR] back on the line at x %d%s; line following resumes\n", s5_mm(s5_x),
		  s5_rejoined ? " (interrupted mid-pass)" : "");
	return true;
}

#endif
