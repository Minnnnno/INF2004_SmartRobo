/*
 *	detour.h
 *	Subsystem 5 -- drive a detour_plan.h plan with Buddy 2's finite moves.
 *	Task context only.  Owner: obstacle_task.
 *
 *	Each move is submitted once as SOURCE_OBSTACLE and waited for until its
 *	EV_COMPLETE.  Between moves the robot is READY, so the line follower
 *	must already be told to keep off the wheels (S5_STATE_AVOIDING in the
 *	shared status) before s5_detour_run() is called.
 */

#ifndef S5_DETOUR_H
#define S5_DETOUR_H

#include "robot.h"
#include "detour_plan.h"

/* motion_get_status(), retried: the shared status lock is only polled */
bool	s5_status(robot_status_t *p_status);
/* Wait until Motion Control has the robot held and still (or 4 s) */
bool	s5_wait_still(void);
/* Has the robot really moved?  Motion Control's "stationary" clears for 0.3 s on
   a single encoder count (0.08 mm in one 10 ms sample reads over stopped_speed),
   which rocking after a turn or the servo's swing can cause.  With no move
   active the motors are off, so only s5_still_moved() can tell a push from a
   wobble: more than S5_STILL_MOVE_MM on either wheel since s5_still_mark(). */
void	s5_still_mark(void);
bool	s5_still_moved(const robot_status_t *p_status);
/* Buddy 3 is publishing fresh line readings */
bool	s5_line_reporting(const robot_status_t *p_status, double now);

/* Task-context diagnostic: coherent live left/right encoder counts. id=0 for
   stop/scan boundaries; finite moves use their command ID. Production only. */
encoder_sample_t s5_log_encoders(const char *phase, uint32_t id);

/* Reverse straight back mm, from a held robot, then hold it still again.
   The line follower must already be told to keep off the wheels.
   True: moved back and held still. */
bool	s5_back_off(double mm);

/* Turn on the spot by degrees (+ left), from a held robot, then hold it still
   and let it settle.  The line follower must already be told to keep off the
   wheels.  *p_turned_deg: what the encoders measured, which a stall cuts short.
   True: held and still.  False: the caller must stop it. */
bool	s5_square_up(double degrees, double *p_turned_deg);

/* Release the obstacle hold, drive round the obstacle and back onto the
   line.  If Buddy 3's centre and right sensors both see black while passing
   the obstacle or coming back, stop there and skip the rest.  The robot is not
   aligned with the line, and when interrupted while passing it may not have
   cleared the obstacle.  True: the robot is on the line, READY and not held.
   False: it gave up somewhere; the caller must stop it. */
bool	s5_detour_run(const s5_detour_plan_t *p_plan);

#endif /* S5_DETOUR_H */
