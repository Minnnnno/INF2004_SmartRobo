/*
 *	detour.c
 *	Subsystem 5 -- drive round a scanned obstacle and back onto the line.
 *
 *	1. Release the hold obstacle_task latched to scan.
 *	2. Back up if needed, turn out, drive the diagonal, turn back parallel.
 *	3. Step forward, looking sideways (and ahead before every step), until
 *	   the obstacle has ended; drive on until the rear clears it.
 *	4. Turn in and drive back slowly, stopping when Buddy 3's sensors see
 *	   the line; drive the axle onto it and turn back to the original
 *	   heading, then check the line is under the sensors.
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
#define S5_MIN_TURN_DEG		1.5	/* smaller turns are skipped (1 deg command resolution) */
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

typedef enum { S5_MOVE_DONE, S5_MOVE_LINE, S5_MOVE_FAILED } s5_move_t;

static uint32_t	s5_move_id = S5_ID_LAST;
static double	s5_heading0;	/* rad, Motion Control's heading at the stop */
static double	s5_x;		/* mm, the axle centre, detour_plan.h frame */
static double	s5_y;

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

/* Wait for move id.  Watching the line, black where odometry expects the
   line stops the robot at once (*p_line_centre_m: centre travel then). */
static s5_move_t wait_move(uint32_t id, double timeout_s, bool b_watch_line,
			   double centre0_m, double *p_line_centre_m)
{
	double		start = motion_hal_time();
	robot_status_t	status;

	for (;;) {
		double now;

		(void)tk_dly_tsk(S5_POLL_MS);
		now = motion_hal_time();
		if (!s5_status(&status)) {
			if (now - start > timeout_s) {
				motion_stop();
				return S5_MOVE_FAILED;
			}
			continue;
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
		if (b_watch_line && line_seen(&status, now)) {
			double heading = heading_deg(&status);
			double y = s5_y + (status.motion.centre - centre0_m) * 1000.0 *
				   sin(heading * S5_DEG_TO_RAD);

			if (s5_line_expected(y, heading)) {
				motion_stop();
				*p_line_centre_m = status.motion.centre;
				return S5_MOVE_LINE;
			}
		}
		/* A fresh line detection wins if it arrives with move completion.
		 * Failure/cancellation events above still take priority over it. */
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
	}
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

/* Straight at S5_DRIVE_SPEED_MM_S, negative = reverse.  The pose follows
   what the encoders measured, not what was asked for. */
static s5_move_t straight(double mm, bool b_watch_line, double *p_line_centre_m)
{
	robot_status_t	before;
	robot_status_t	after;
	uint32_t	id;
	motion_result_t	result;
	s5_move_t	move;
	double		submit_start;

	if (fabs(mm) < S5_MIN_MOVE_MM) {
		return S5_MOVE_DONE;
	}
	if (!s5_status(&before)) {
		return S5_MOVE_FAILED;
	}
	id = next_id();
	submit_start = motion_hal_time();
	do {
		result = motion_steer_distance(SOURCE_OBSTACLE, id, mm / 1000.0, 0.0,
					       S5_DRIVE_SPEED_MM_S / 1000.0);
	} while ((result == R_BUSY) && retry_busy(submit_start, before.motion.generation));
	if (result != R_OK) {
		tm_printf((const UB *)"[DETOUR] drive %d mm refused (%u)\n", s5_mm(mm), (unsigned)result);
		return S5_MOVE_FAILED;
	}
	tm_printf((const UB *)"[DETOUR] move %u queued: straight %d mm at %d mm/s\n",
		  (unsigned)id, s5_mm(mm), s5_mm(S5_DRIVE_SPEED_MM_S));
	move = wait_move(id, fabs(mm) / S5_DRIVE_SPEED_MM_S + S5_MOVE_SLACK_S,
			 b_watch_line, before.motion.centre, p_line_centre_m);
	if (move == S5_MOVE_LINE) {
		if (!s5_wait_still()) {
			tm_printf((const UB *)"[DETOUR] move %u could not confirm line stop\n", (unsigned)id);
			return S5_MOVE_FAILED;
		}
	}
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
		tm_printf((const UB *)"[DETOUR] move %u complete: straight\n", (unsigned)id);
	}
	return move;
}

/* Turn in place to a heading relative to the one at the stop, so turn
   errors do not add up over the detour. */
static s5_move_t turn_to(double target_deg)
{
	robot_status_t	status;
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
	do {
		result = motion_turn_degrees(SOURCE_OBSTACLE, id, turn);
	} while ((result == R_BUSY) && retry_busy(submit_start, status.motion.generation));
	if (result != R_OK) {
		tm_printf((const UB *)"[DETOUR] turn %d deg refused (%u)\n", s5_mm(turn), (unsigned)result);
		return S5_MOVE_FAILED;
	}
	tm_printf((const UB *)"[DETOUR] move %u queued: turn %d deg to heading %d deg\n",
		  (unsigned)id, s5_mm(turn), s5_mm(target_deg));
	move = wait_move(id, S5_TURN_TIMEOUT_S, false, 0.0, NULL);
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

typedef enum { S5_LOOK_ECHO, S5_LOOK_CLEAR, S5_LOOK_FAILED } s5_look_t;

/* A silent but responding sensor differs from a disconnected/stuck one.
   Do not turn sensor failures or conflicting readings into permission to drive. */
static s5_look_t look(int angle_deg, double *p_range_mm)
{
	s5_estimate_t estimate;
	bool failed = false;

	s5_servo_move(angle_deg);
	s5_sample_point(detour_ping, &failed, &estimate);
	*p_range_mm = estimate.range_mm;
	if (failed) { return S5_LOOK_FAILED; }
	if (s5_estimate_usable(&estimate)) { return S5_LOOK_ECHO; }
	return (estimate.status == S5_ECHO_NONE) ? S5_LOOK_CLEAR : S5_LOOK_FAILED;
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

static bool go_out(const s5_detour_plan_t *p_plan)
{
	detour_stage(CRASH_OBSTACLE_OUT, "out");
	tm_printf((const UB *)"[DETOUR] out: back %d mm, turn %d deg, diagonal %d mm -> %d mm %s of the line\n",
		  s5_mm(p_plan->backup_mm), s5_mm(p_plan->turn_deg), s5_mm(p_plan->diagonal_mm),
		  s5_mm(fabs(p_plan->shift_mm)), (p_plan->shift_mm >= 0.0) ? "left" : "right");
	return (straight(-p_plan->backup_mm, false, NULL) == S5_MOVE_DONE) &&
	       (turn_to(p_plan->turn_deg) == S5_MOVE_DONE) &&
	       (straight(p_plan->diagonal_mm, false, NULL) == S5_MOVE_DONE) &&
	       (turn_to(0.0) == S5_MOVE_DONE);
}

static bool pass_obstacle(const s5_detour_plan_t *p_plan)
{
	int		side_deg = (p_plan->side == S5_AVOID_LEFT) ? S5_ANGLE_RIGHT_DEG : S5_ANGLE_LEFT_DEG;
	double		start_x = s5_x;
	double		clear_x;
	double		range_mm;
	bool		b_echo;
	s5_pass_t	pass;

	detour_stage(CRASH_OBSTACLE_PASS, "pass");
	s5_pass_init(&pass, p_plan);
	for (;;) {
		s5_look_t result = look(side_deg, &range_mm);
		if (result == S5_LOOK_FAILED) {
			tm_printf((const UB *)"[DETOUR] side measurement failed; not moving\n");
			return false;
		}
		b_echo = (result == S5_LOOK_ECHO);
		(void)s5_pass_observe(&pass, p_plan, s5_x, s5_y, b_echo, range_mm);
		if (b_echo) {
			tm_printf((const UB *)"[DETOUR] pass x %d y %d: side %d mm -> obstacle to x %d, clear %u/%u\n",
				  s5_mm(s5_x), s5_mm(s5_y), s5_mm(range_mm),
				  s5_mm(pass.rear_mm), pass.clear_run, S5_PASS_CONFIRM);
		} else {
			tm_printf((const UB *)"[DETOUR] pass x %d y %d: side no echo -> obstacle to x %d, clear %u/%u\n",
				  s5_mm(s5_x), s5_mm(s5_y), s5_mm(pass.rear_mm), pass.clear_run, S5_PASS_CONFIRM);
		}
		if (pass.b_ended) {
			break;
		}
		if (s5_x - start_x >= S5_PASS_MAX_MM) {
			tm_printf((const UB *)"[DETOUR] obstacle has not ended after %d mm\n", s5_mm(S5_PASS_MAX_MM));
			return false;
		}
		if (!path_clear(S5_PASS_STEP_MM) ||
		    (straight(S5_PASS_STEP_MM, false, NULL) != S5_MOVE_DONE)) {
			return false;
		}
	}
	clear_x = s5_pass_clear_x(&pass);
	detour_stage(CRASH_OBSTACLE_REAR_CLEAR, "rear-clearance");
	tm_printf((const UB *)"[DETOUR] obstacle ends by x %d (%u echoes); rear clears it at x %d\n",
		  s5_mm(fmax(pass.rear_mm, pass.end_mm)), pass.seen, s5_mm(clear_x));
	if (clear_x - s5_x >= S5_MIN_MOVE_MM) {
		return path_clear(clear_x - s5_x) &&
		       (straight(clear_x - s5_x, false, NULL) == S5_MOVE_DONE);
	}
	return true;
}

static bool return_to_line(const s5_detour_plan_t *p_plan)
{
	double		line_centre_m = 0.0;
	double		start;
	double		overshoot_mm;
	robot_status_t	status;

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
	switch (straight(s5_return_length(s5_y), true, &line_centre_m)) {
	case S5_MOVE_LINE:
		break;
	case S5_MOVE_DONE:
		tm_printf((const UB *)"[DETOUR] line not found where expected (now %d mm off it)\n", s5_mm(s5_y));
		return false;
	default:
		return false;
	}
	if (!release_hold() || !s5_status(&status)) {
		return false;
	}
	detour_stage(CRASH_OBSTACLE_ALIGN, "line-alignment");
	/* The sensors were on the line at line_centre_m: drive the axle there,
	   then turning back parallel leaves the sensors over the line. */
	overshoot_mm = (status.motion.centre - line_centre_m) * 1000.0;
	tm_printf((const UB *)"[DETOUR] line seen at y %d, stopped %d mm on; aligning\n",
		  s5_mm(s5_y - overshoot_mm * sin(heading_deg(&status) * S5_DEG_TO_RAD)), s5_mm(overshoot_mm));
	if ((straight(S5_IR_AHEAD_MM - overshoot_mm, false, NULL) != S5_MOVE_DONE) ||
	    (turn_to(0.0) != S5_MOVE_DONE)) {
		return false;
	}
	start = motion_hal_time();
	while (motion_hal_time() - start < S5_LINE_CONFIRM_S) {
		if (s5_status(&status) && line_seen(&status, motion_hal_time())) {
			return true;
		}
		(void)tk_dly_tsk(S5_POLL_MS);
	}
	tm_printf((const UB *)"[DETOUR] aligned, but the line is not under the sensors\n");
	return false;
}

bool s5_detour_run(const s5_detour_plan_t *p_plan)
{
	robot_status_t status;

	if (!p_plan->b_ok || !release_hold() || !s5_status(&status)) {
		return false;
	}
	s5_heading0 = status.motion.heading;
	s5_x = 0.0;
	s5_y = 0.0;
	if (!go_out(p_plan) || !pass_obstacle(p_plan) || !return_to_line(p_plan)) {
		tm_printf((const UB *)"[DETOUR] gave up at x %d y %d, heading %d deg\n",
			  s5_mm(s5_x), s5_mm(s5_y), s5_status(&status) ? s5_mm(heading_deg(&status)) : 0);
		return false;
	}
	tm_printf((const UB *)"[DETOUR] back on the line at x %d; line following resumes\n", s5_mm(s5_x));
	return true;
}

#endif
