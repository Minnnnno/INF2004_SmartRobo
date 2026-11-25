/*
 *  detour.h
 *  Subsystem 5 -- drive a detour_plan.h plan with Buddy 2's finite moves.
 *  Task context only.  Owner: obstacle_task.
 *
 *  Each move is submitted once as SOURCE_OBSTACLE and waited for until its
 *  EV_COMPLETE.  Between moves the robot is READY, so the line follower
 *  must already be told to keep off the wheels (S5_STATE_AVOIDING in the
 *  shared status) before s5_detour_run() is called.
 */

#ifndef S5_DETOUR_H
#define S5_DETOUR_H

#include "../../include/float_types.h"
#include "robot.h"
#include "detour_plan.h"

/* motion_get_status(), retried: the shared status lock is only polled */
bool s5_status(robot_status_t * p_status);
/* Wait until Motion Control has the robot held and still (or 4 s) */
bool s5_wait_still(void);
/* Has the robot really moved?  Motion Control's "stationary" clears for 0.3 s
   on a single encoder count (0.08 mm in one 10 ms sample reads over
   stopped_speed), which rocking after a turn or the servo's swing can cause.
   With no move active the motors are off, so only s5_still_moved() can tell a
   push from a wobble: more than S5_STILL_MOVE_MM on either wheel since
   s5_still_mark(). */
void s5_still_mark(void);
bool s5_still_moved(const robot_status_t * p_status);
/* Buddy 3 is publishing fresh line readings */
bool s5_line_reporting(const robot_status_t * p_status, float64_t now);

/* Task-context diagnostic: coherent live left/right encoder counts. id=0 for
   stop/scan boundaries; finite moves use their command ID. Production only. */
encoder_sample_t s5_log_encoders(const char * p_phase, uint32_t command_id);

/* Reverse straight back mm, from a held robot, then hold it still again.
   The line follower must already be told to keep off the wheels.
   True: moved back and held still. */
bool s5_back_off(float64_t distance_mm);

/* Turn on the spot by degrees (+ left), from a held robot, then hold it still
   and let it settle.  The line follower must already be told to keep off the
   wheels.  *p_turned_deg: what the encoders measured, which a stall cuts short.
   True: held and still.  False: the caller must stop it. */
bool s5_square_up(float64_t degrees, float64_t * p_turned_deg);

/* Release the obstacle hold, drive round the obstacle and back onto the
   line.  If Buddy 3's centre and right sensors both see black while passing
   the obstacle or coming back, stop there and skip the rest.  The robot is not
   aligned with the line, and when interrupted while passing it may not have
   cleared the obstacle.  True: the robot is on the line, READY and not held.
   False: it gave up somewhere; the caller must stop it, unless
   s5_detour_retryable() says the way out was blocked before it left and the
   robot is back, held and still, where it stopped: another plan may be tried.
   While it runs, an echo the plan's obstacle explains does not block a move
   (s5_echo_on_obstacle). */
bool s5_detour_run(const s5_detour_plan_t * p_plan);
bool s5_detour_retryable(void);

/* Why a scan or detour stopped (README.md "Why a detour stopped"): a 6-bit
   index kept in one byte, with one table of names for the console and MQTT. */
#define S5_WHY_CODES(X)        \
    X(ahead_blocked)           \
    X(diagonal_blocked)        \
    X(lifted)                  \
    X(line_lost_at_stop)       \
    X(line_not_found)          \
    X(line_not_reporting)      \
    X(look_moved)              \
    X(look_unstable)           \
    X(motion_fault)            \
    X(move_event)              \
    X(moved)                   \
    X(moved_while_scanning)    \
    X(no_progress)             \
    X(no_release)              \
    X(no_safe_detour)          \
    X(no_status)               \
    X(no_way_around)           \
    X(not_accepted)            \
    X(not_still)               \
    X(noted_line_blocked)      \
    X(noted_line_not_found)    \
    X(obstacle_too_long)       \
    X(parallel_blocked)        \
    X(pass_ahead_blocked)      \
    X(pass_look_failed)        \
    X(publish_failed)          \
    X(refused)                 \
    X(released_while_scanning) \
    X(repeated_spurious)       \
    X(scan_failed)             \
    X(scan_setup_failed)       \
    X(sensor_failed)           \
    X(sensor_fault)            \
    X(side_too_close)          \
    X(stall_no_escape)         \
    X(stalled) X(stopped) X(timeout) X(ultrasonic_failed) X(unknown)
#define S5_WHY_ENUM(name) S5_WHY_##name,
typedef enum
{
    S5_WHY_none,
    S5_WHY_CODES(S5_WHY_ENUM) S5_WHY_COUNT
} s5_why_t;

const char * s5_why_name(s5_why_t why);

/* Why the last s5_detour_run() (or anything since the last clear) gave up,
   or S5_WHY_none when nothing failed.
   *p_a and *p_b: two numbers whose meaning depends on the code. */
void     s5_detour_reason_clear(void);
s5_why_t s5_detour_reason(int * p_a, int * p_b);
/* Where the detour has the axle now (detour_plan.h frame), mm */
void s5_detour_pose(int * p_x_mm, int * p_y_mm);

/* Release the hold this task latched, once the robot is still (as the detour
   does before each move).  True: released, READY. */
bool s5_release(void);

#endif /* S5_DETOUR_H */

/*** end of file ***/
