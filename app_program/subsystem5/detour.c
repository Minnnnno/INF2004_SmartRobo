/*
 *  detour.c
 *  Subsystem 5 -- drive round a scanned obstacle and back onto the line.
 *
 *  1. Release the hold obstacle_task latched to scan.
 *  2. Back up if needed, turn out, drive the diagonal, turn back parallel.
 *  3. Drive forward without stopping, the sensor alternating between sideways
 *     and ahead, until the obstacle has ended and the rear clears it.
 *  4. Turn in and drive back slowly until Buddy 3's sensors find the line.
 *
 *  From S5_REJOIN_AFTER_MM into step 3, once parallel again after the diagonal,
 *  and during step 4, Buddy 3's centre and right sensors both seeing black
 *  (line_rejoin_seen) where odometry puts the straight line (within
 *  S5_LINE_WINDOW_MM) interrupts the detour: the robot stops, makes sure the
 *  line is under the sensors and hands straight back to the line follower.
 *  It does not line itself up first: the follower's search finds it from here.
 *  Both black anywhere else is a junction's branch or the line curving away:
 *  the robot notes where and carries on, so that the straight path ahead wins.
 *  Only if step 4 then finds no line does it go back to the last black it
 *  noted (return_to_noted) and hand over there.
 *
 *  Positions are tracked from the encoders (detour_plan.h frame).  Every
 *  step has a limit; anything unexpected returns false and the caller
 *  stops the robot.
 */

#include "robot_rtos.h" /* first: kernel types with the C library's size_t */
#include "../../include/float_types.h"
#include <stdint.h>
#include "detour.h"
#include "steering.h"
#include "motion_hal.h"
#include "servo.h"
#include "ultrasonic.h"
#include "crash_log.h"
#include "../subsystem3/line_rejoin.h"
#include <tm/tmonitor.h>
#include <math.h>

#define S5_DEG_TO_RAD        (0.017453292519943295)
#define S5_POLL_MS           20U
#define S5_SETTLE_TIMEOUT_S  4.0 /* > Motion Control's 3 s stop timeout */
#define S5_RELEASE_TIMEOUT_S 1.0
#define S5_ACCEPT_TIMEOUT_S  0.5 /* a move not started by then was rejected */
#define S5_MOVE_SLACK_S      3.0 /* on top of distance / speed */
#define S5_TURN_TIMEOUT_S    4.0
#define S5_LINE_FRESH_S      0.2 /* line readings older than this are stale */
/* wait this long for the line after aligning */
#define S5_LINE_CONFIRM_S 0.3
/* shorter moves are skipped (1 mm command resolution) */
#define S5_MIN_MOVE_MM 3.0
/* rocking after squaring up would abort the next scan */
#define S5_SQUARE_SETTLE_MS 500U
/* smaller turns are skipped (1 deg command resolution) */
#define S5_MIN_TURN_DEG 1.5
/* driven parallel before the line may end the detour */
#define S5_REJOIN_AFTER_MM 20.0
/* Going back to a noted line: drive the sensors this far past where they saw it
   (the line is accepted up to S5_LINE_WINDOW_MM from there) */
#define S5_NOTED_OVERSHOOT_MM 100.0
#define S5_ID_FIRST           5100U
#define S5_ID_LAST            5999U

bool
s5_status (robot_status_t * p_status)
{
    unsigned attempt;

    for (attempt = 0U; attempt < 3U; attempt++)
    {
        if (motion_get_status(p_status))
        {
            return true;
        }

        (void)tk_dly_tsk(1U);
    }

    return false;
}

#define S5_STILL_MOVE_MM 5.0

static float64_t g_s5_still_travel[2];

void
s5_still_mark (void)
{
    robot_status_t status;

    if (s5_status(&status))
    {
        g_s5_still_travel[0] = status.motion.travel[0];
        g_s5_still_travel[1] = status.motion.travel[1];
    }
}

bool
s5_still_moved (const robot_status_t * p_status)
{
    return ((fabs(p_status->motion.travel[0] - g_s5_still_travel[0]) * 1000.0)
            > S5_STILL_MOVE_MM)
           || ((fabs(p_status->motion.travel[1] - g_s5_still_travel[1])
                * 1000.0)
               > S5_STILL_MOVE_MM);
}

bool
s5_wait_still (void)
{
    float64_t      start = motion_hal_time();
    robot_status_t status;

    while ((motion_hal_time() - start) < S5_SETTLE_TIMEOUT_S)
    {
        if ((s5_status(&status)) && (status.motion.hold)
            && (status.motion.stationary) && (DRIVING != status.motion.state)
            && (STOPPING != status.motion.state))
        {
            return true;
        }

        (void)tk_dly_tsk(S5_POLL_MS);
    }

    return false;
}

bool
s5_line_reporting (const robot_status_t * p_status, float64_t now)
{
    return (p_status->line.available)
           && ((now - p_status->line.timestamp) <= S5_LINE_FRESH_S);
}

#if defined(ROBO_PRODUCTION_BUILD) && !defined(ROBO_IR_MONITOR) \
    && ROBO_SUBSYSTEM5

typedef enum
{
    S5_MOVE_DONE,
    S5_MOVE_LINE,
    S5_MOVE_REJOIN,
    S5_MOVE_FAILED,
    S5_MOVE_RUNNING,
    S5_MOVE_STALLED
} s5_move_t;

encoder_sample_t
s5_log_encoders (const char * p_phase, uint32_t command_id)
{
    encoder_sample_t sample = motion_hal_sample();

    tm_printf((const UB *)"[OBSTACLE] enc phase=%s id=%u L=%lu R=%lu "
                          "invalid=%lu,%lu reliable=%u\n",
              p_phase, (unsigned)command_id, (uint32_t)sample.count[0],
              (uint32_t)sample.count[1], (uint32_t)sample.invalid[0],
              (uint32_t)sample.invalid[1], (unsigned)sample.reliable);
    return sample;
}

static void
log_encoder_after (uint32_t command_id, const encoder_sample_t * p_before,
                   s5_move_t result)
{
    encoder_sample_t after = s5_log_encoders("move-after", command_id);

    /* encoder_delta returns the signed 32-bit modular difference. Use the
       monitor's supported long format (it does not support long long). */
    tm_printf(
        (const UB *)"[DETOUR] enc id=%u delta_L=%ld delta_R=%ld result=%s\n",
        (unsigned)command_id,
        (int32_t)encoder_delta(after.count[0], p_before->count[0]),
        (int32_t)encoder_delta(after.count[1], p_before->count[1]),
        (S5_MOVE_DONE == result)      ? "done"
        : (S5_MOVE_LINE == result)    ? "line"
        : (S5_MOVE_REJOIN == result)  ? "rejoin"
        : (S5_MOVE_STALLED == result) ? "stalled"
                                      : "failed");
}

/* One byte, obstacle_task only: changing a bit needs no lock */
static struct
{
    /* pass and return: both sensors black interrupts the detour */
    bool rejoin_armed : 1;
    /* the pass stage ended by handing back to the line */
    bool rejoined     : 1;
    bool line_touched : 1; /* return leg: one sensor has seen the line */
    /* both sensors saw black away from the straight line */
    bool line_noted : 1;
    /* the line is expected at the noted point, not at y 0 */
    bool line_goal : 1;
    /* the pass ended against something: see escape_stall() */
    bool pass_stalled : 1;
    /* the way out was blocked before any diagonal travel */
    bool out_blocked : 1;
    /* the last clearance_sweep failed on an echo, not a fault */
    bool sweep_blocked : 1;
} g_s5_leg;

_Static_assert(1U == sizeof(g_s5_leg), "eight flags in one byte");
/* the shared command ID stays 32 bits */
static uint16_t g_s5_move_id = S5_ID_LAST;
/* rad, Motion Control's heading at the stop */
static float64_t g_s5_heading0;
/* mm, the axle centre, detour_plan.h frame */
static float64_t g_s5_x;
static float64_t g_s5_y;
/* armed, but no hand-back before this centre travel */
static float64_t g_s5_rejoin_after_m;
/* centre travel to come back to if braking overshoots */
static float64_t g_s5_rejoin_centre_m;
/* centre travel where the sensors were first on it */
static float64_t g_s5_line_first_m;
static float64_t g_s5_line_last_m; /* and most recently */
/* where the line sensors were, the last time (mm) */
static float64_t g_s5_noted_x;
static float64_t g_s5_noted_y;
static float64_t g_s5_noted_centre_m; /* centre travel then */
/* noted during the return leg itself */
static bool gb_s5_noted_here;
/* wheels first seen stopped under a target, or -1 */
static float64_t g_s5_stall_since = -1.0;
/* the obstacle being driven round, or NULL */
static const s5_detour_plan_t * gp_s5_known;
/* where the last clearance sweep was blocked */
static uint8_t  g_s5_block_angle;
static uint16_t g_s5_block_range;

/* Why the detour gave up: the first failure wins, so the cause is kept rather
   than what failed because of it.  See s5_detour_reason() in detour.h. */
static uint8_t g_s5_why; /* s5_why_t */
static int16_t g_s5_why_a;
static int16_t g_s5_why_b;

/* In the order of s5_why_t. */
static const char * const g_s5_why_names[] = {
    "none",
    "ahead_blocked",
    "diagonal_blocked",
    "lifted",
    "line_lost_at_stop",
    "line_not_found",
    "line_not_reporting",
    "look_moved",
    "look_unstable",
    "motion_fault",
    "move_event",
    "moved",
    "moved_while_scanning",
    "no_progress",
    "no_release",
    "no_safe_detour",
    "no_status",
    "no_way_around",
    "not_accepted",
    "not_still",
    "noted_line_blocked",
    "noted_line_not_found",
    "obstacle_too_long",
    "parallel_blocked",
    "pass_ahead_blocked",
    "pass_look_failed",
    "publish_failed",
    "refused",
    "released_while_scanning",
    "repeated_spurious",
    "scan_failed",
    "scan_setup_failed",
    "sensor_failed",
    "sensor_fault",
    "side_too_close",
    "stall_no_escape",
    "stalled",
    "stopped",
    "timeout",
    "ultrasonic_failed",
    "unknown",
};
_Static_assert((sizeof(g_s5_why_names) / sizeof(g_s5_why_names[0]))
                   == S5_WHY_COUNT,
               "one name per code");
_Static_assert(S5_WHY_COUNT <= 64, "a code is 6 bits");

const char *
s5_why_name (s5_why_t why)
{
    return g_s5_why_names[(why < S5_WHY_COUNT) ? why : S5_WHY_UNKNOWN];
}

/* 14 bits, clamped */
static int16_t
why_number (int value)
{
    return (int16_t)((value < -8192) ? -8192 : ((value > 8191) ? 8191 : value));
}

static void
detour_why (s5_why_t why, int detail_a, int detail_b)
{
    if ((uint8_t)S5_WHY_NONE == g_s5_why)
    {
        g_s5_why   = (uint8_t)why;
        g_s5_why_a = why_number(detail_a);
        g_s5_why_b = why_number(detail_b);
    }
}

void
s5_detour_reason_clear (void)
{
    g_s5_why   = (uint8_t)S5_WHY_NONE;
    g_s5_why_a = 0;
    g_s5_why_b = 0;
}

s5_why_t
s5_detour_reason (int * p_a, int * p_b)
{
    *p_a = g_s5_why_a;
    *p_b = g_s5_why_b;
    return (s5_why_t)g_s5_why;
}

static int
s5_mm (float64_t value)
{
    return (int)((value < 0.0) ? (value - 0.5) : (value + 0.5));
}

void
s5_detour_pose (int * p_x_mm, int * p_y_mm)
{
    *p_x_mm = s5_mm(g_s5_x);
    *p_y_mm = s5_mm(g_s5_y);
}

static void
detour_stage (UW marker, const char * p_name)
{
    crash_mark(CRASH_MARK_OBSTACLE, marker);
    tm_printf((const UB *)"[DETOUR] stage=%s x=%d y=%d\n", p_name,
              s5_mm(g_s5_x), s5_mm(g_s5_y));
}

/* Buddy 3's values[1]: 1 while a sensor sees black (debounced) */
static bool
line_seen (const robot_status_t * p_status, float64_t now)
{
    return (s5_line_reporting(p_status, now))
           && (p_status->line.values[1] > 0.5);
}

static float64_t
heading_deg (const robot_status_t * p_status)
{
    return (p_status->motion.heading - g_s5_heading0) / S5_DEG_TO_RAD;
}

/* Where the axle is during a move: the pass keeps s5_x/s5_y up to date as it
   goes (drive_track); a straight move from centre travel centre0_m does not. */
static void
move_pose (const robot_status_t * p_status, bool b_straight,
           float64_t centre0_m, float64_t * p_x, float64_t * p_y)
{
    float64_t heading = heading_deg(p_status) * S5_DEG_TO_RAD;
    float64_t travelled
        = b_straight ? (p_status->motion.centre - centre0_m) * 1000.0 : 0.0;

    *p_x = g_s5_x + (travelled * cos(heading));
    *p_y = g_s5_y + (travelled * sin(heading));
}

/* Black under the line sensors, with the axle at (x_mm, y_mm), is the line
   being looked for: the straight line (y 0), or the noted point when going back
 */
static bool
line_where_expected (float64_t x_mm, float64_t y_mm, float64_t heading_now_deg)
{
    float64_t heading_rad = heading_now_deg * S5_DEG_TO_RAD;

    if (g_s5_leg.line_goal)
    {
        return hypot(x_mm + (S5_IR_AHEAD_MM * cos(heading_rad)) - g_s5_noted_x,
                     y_mm + (S5_IR_AHEAD_MM * sin(heading_rad)) - g_s5_noted_y)
               <= S5_LINE_WINDOW_MM;
    }

    return s5_line_expected(y_mm, heading_now_deg);
}

/* Both black, but not where the straight line is: remember it and carry on */
static void
note_line (float64_t x_mm, float64_t y_mm, float64_t heading_now_deg,
           float64_t centre_m)
{
    float64_t heading_rad = heading_now_deg * S5_DEG_TO_RAD;

    if (!g_s5_leg.line_noted)
    {
        tm_printf((const UB *)"[DETOUR] line under the sensors at x %d y %d, "
                              "away from the "
                              "straight path: noted, carrying on\n",
                  s5_mm(x_mm), s5_mm(y_mm));
    }

    g_s5_leg.line_noted = true;
    g_s5_noted_x        = x_mm + (S5_IR_AHEAD_MM * cos(heading_rad));
    g_s5_noted_y        = y_mm + (S5_IR_AHEAD_MM * sin(heading_rad));
    g_s5_noted_centre_m = centre_m;
    gb_s5_noted_here    = true;
}

static uint32_t
next_id (void)
{
    g_s5_move_id
        = (uint16_t)((g_s5_move_id >= S5_ID_LAST) ? S5_ID_FIRST
                                                  : (g_s5_move_id + 1U));
    return g_s5_move_id;
}

/* Only the hold this task latched is released, and only once Motion
   Control reports the robot still; motion_release() checks that too. */
static bool
release_hold (void)
{
    float64_t      start;
    robot_status_t status;

    if (!s5_wait_still())
    {
        tm_printf(
            (const UB *)"[DETOUR] robot not confirmed still; not releasing\n");
        detour_why(S5_WHY_NOT_STILL, 0, 0);
        return false;
    }

    motion_controller_release(true);
    start = motion_hal_time();

    while ((motion_hal_time() - start) < S5_RELEASE_TIMEOUT_S)
    {
        if ((s5_status(&status)) && (!status.motion.hold)
            && (READY == status.motion.state))
        {
            return true;
        }

        (void)tk_dly_tsk(S5_POLL_MS);
    }

    tm_printf((const UB *)"[DETOUR] Motion Control did not release the hold\n");
    detour_why(S5_WHY_NO_RELEASE, 0, 0);
    return false;
}

bool
s5_release (void)
{
    return release_hold();
}

/* One check of move id.  Both sensors black, once armed, stops the robot at
   once. Watching the line, one sensor black where odometry expects the line
   does not: the sensors do not reach the line together, so it keeps going for
   the other one. If it comes off the line without both ever having been black,
   or the move ends, it stops then (*p_line_centre_m: centre travel in the
   middle of the black). */
static s5_move_t
move_poll (uint32_t command_id, float64_t start, float64_t timeout_s,
           bool b_watch_line, float64_t centre0_m, float64_t * p_line_centre_m)
{
    float64_t      now = motion_hal_time();
    robot_status_t status;

    if (!s5_status(&status))
    {
        if ((now - start) > timeout_s)
        {
            motion_stop();
            return S5_MOVE_FAILED;
        }

        return S5_MOVE_RUNNING;
    }

    if ((status.motion.hold) || (FAULT == status.motion.state)
        || (F_NONE != status.motion.fault))
    {
        tm_printf((const UB *)"[DETOUR] move %u interrupted: hold %u, state "
                              "%u, fault %u\n",
                  (unsigned)command_id, (unsigned)status.motion.hold,
                  (unsigned)status.motion.state, (unsigned)status.motion.fault);

        if (F_NONE != status.motion.fault)
        {
            detour_why(S5_WHY_MOTION_FAULT, (int)status.motion.fault,
                       (int)status.motion.state);
        }
        else
        {
            /* STOP, or Buddy 4: lifted */
            detour_why(S5_WHY_STOPPED, (int)status.motion.state, 0);
        }

        return S5_MOVE_FAILED;
    }

    if ((status.last_event.id == command_id)
        && (EV_COMPLETE != status.last_event.type))
    {
        tm_printf(
            (const UB *)"[DETOUR] move %u ended with event %u, fault %u\n",
            (unsigned)command_id, (unsigned)status.last_event.type,
            (unsigned)status.last_event.fault);
        detour_why(S5_WHY_MOVE_EVENT, (int)status.last_event.type,
                   (int)status.last_event.fault);
        return S5_MOVE_FAILED;
    }

    if ((g_s5_leg.rejoin_armed) && (status.motion.centre >= g_s5_rejoin_after_m)
        && (line_rejoin_seen(&status, now)))
    {
        float64_t pos_x;
        float64_t pos_y;

        move_pose(&status, b_watch_line, centre0_m, &pos_x, &pos_y);

        if (line_where_expected(pos_x, pos_y, heading_deg(&status)))
        {
            motion_stop();
            g_s5_rejoin_centre_m = status.motion.centre;
            return S5_MOVE_REJOIN;
        }

        note_line(pos_x, pos_y, heading_deg(&status), status.motion.centre);
    }

    if (b_watch_line)
    {
        float64_t heading = heading_deg(&status);

        float64_t pos_x;
        float64_t pos_y;
        bool      b_seen = line_seen(&status, now);

        move_pose(&status, true, centre0_m, &pos_x, &pos_y);

        if (b_seen
            && ((g_s5_leg.line_touched)
                || (line_where_expected(pos_x, pos_y, heading))))
        {
            g_s5_line_first_m = g_s5_leg.line_touched ? g_s5_line_first_m
                                                      : status.motion.centre;
            g_s5_line_last_m  = status.motion.centre;
            g_s5_leg.line_touched = true;
        }

        /* A fresh line detection wins if it arrives with move completion.
         * Failure/cancellation events above still take priority over it. */
        if ((g_s5_leg.line_touched)
            && ((!b_seen) || (status.last_event.id == command_id)))
        {
            motion_stop();
            *p_line_centre_m = 0.5 * (g_s5_line_first_m + g_s5_line_last_m);
            return S5_MOVE_LINE;
        }
    }

    if ((status.last_event.id != command_id) && (DRIVING == status.motion.state)
        && (fabs(status.motion.speed[0]) < S5_STALL_SPEED_M_S)
        && (fabs(status.motion.speed[1]) < S5_STALL_SPEED_M_S)
        && ((fabs(status.motion.target[0]) + fabs(status.motion.target[1]))
            > S5_STALL_TARGET_M_S))
    {
        if (g_s5_stall_since < 0.0)
        {
            g_s5_stall_since = now;
        }
        else if ((now - g_s5_stall_since) >= S5_STALL_S)
        {
            motion_stop();
            tm_printf((const UB *)"[DETOUR] move %u stalled: wheels not "
                                  "turning for %d ms with %d mm/s asked\n",
                      (unsigned)command_id,
                      s5_mm((now - g_s5_stall_since) * 1000.0),
                      s5_mm(500.0
                            * (fabs(status.motion.target[0])
                               + fabs(status.motion.target[1]))));
            /* Cleared again if the pass escapes it (pass_obstacle) */
            detour_why(S5_WHY_STALLED, s5_mm(g_s5_x), s5_mm(g_s5_y));
            return S5_MOVE_STALLED;
        }
        else
        {
            /* Nothing to do in any other case. */
        }
    }
    else
    {
        g_s5_stall_since = -1.0;
    }

    if (status.last_event.id == command_id)
    {
        return S5_MOVE_DONE;
    }

    if (((now - start) > S5_ACCEPT_TIMEOUT_S)
        && (status.motion.id != command_id))
    {
        tm_printf((const UB *)"[DETOUR] move %u was not accepted (state %u, "
                              "owner %u)\n",
                  (unsigned)command_id, (unsigned)status.motion.state,
                  (unsigned)status.owner);
        detour_why(S5_WHY_NOT_ACCEPTED, (int)status.motion.state,
                   (int)status.owner);
        return S5_MOVE_FAILED;
    }

    if ((now - start) > timeout_s)
    {
        motion_stop();
        tm_printf((const UB *)"[DETOUR] move %u timed out\n",
                  (unsigned)command_id);
        detour_why(S5_WHY_TIMEOUT, (int)command_id, 0);
        return S5_MOVE_FAILED;
    }

    return S5_MOVE_RUNNING;
}

static s5_move_t
wait_move (uint32_t command_id, float64_t timeout_s, bool b_watch_line,
           float64_t centre0_m, float64_t * p_line_centre_m)
{
    float64_t start = motion_hal_time();
    s5_move_t move;

    g_s5_leg.line_touched = false;
    g_s5_stall_since      = -1.0;

    do
    {
        (void)tk_dly_tsk(S5_POLL_MS);
        move = move_poll(command_id, start, timeout_s, b_watch_line, centre0_m,
                         p_line_centre_m);
    } while (S5_MOVE_RUNNING == move);
    return move;
}

/* R_BUSY means the nonblocking command-bus lock was unavailable; no command
 * was queued. Retry only that case, with a deadline, and never across a stop,
 * reset, fault, or another command starting. R_OK is submitted only once. */
static bool
retry_busy (float64_t start, uint32_t generation)
{
    robot_status_t status;

    if ((motion_hal_time() - start) >= S5_ACCEPT_TIMEOUT_S)
    {
        return false;
    }

    (void)tk_dly_tsk(S5_POLL_MS);
    return ((motion_hal_time() - start) < S5_ACCEPT_TIMEOUT_S)
           && (s5_status(&status)) && (!status.motion.hold)
           && (READY == status.motion.state) && (F_NONE == status.motion.fault)
           && (status.motion.generation == generation);
}

/* Straight at S5_DRIVE_SPEED_MM_S (S5_LINE_SEARCH_SPEED_MM_S when watching
   for the line), negative = reverse.  The pose follows
   what the encoders measured, not what was asked for. */
static s5_move_t
straight (float64_t distance_mm, bool b_watch_line, float64_t * p_line_centre_m)
{
    robot_status_t   before;
    robot_status_t   after;
    encoder_sample_t counts_before;
    uint32_t         command_id;
    motion_result_t  result;
    s5_move_t        move;
    float64_t        submit_start;
    float64_t        speed_mm_s
        = b_watch_line ? S5_LINE_SEARCH_SPEED_MM_S : S5_DRIVE_SPEED_MM_S;

    if (fabs(distance_mm) < S5_MIN_MOVE_MM)
    {
        return S5_MOVE_DONE;
    }

    if (!s5_status(&before))
    {
        return S5_MOVE_FAILED;
    }

    command_id    = next_id();
    submit_start  = motion_hal_time();
    counts_before = s5_log_encoders("move-before", command_id);

    do
    {
        result = motion_steer_distance(SOURCE_OBSTACLE, command_id,
                                       distance_mm / 1000.0, 0.0,
                                       speed_mm_s / 1000.0);
    } while ((R_BUSY == result)
             && (retry_busy(submit_start, before.motion.generation)));

    if (R_OK != result)
    {
        tm_printf((const UB *)"[DETOUR] drive %d mm refused (%u)\n",
                  s5_mm(distance_mm), (unsigned)result);
        detour_why(S5_WHY_REFUSED, (int)result, s5_mm(distance_mm));
        log_encoder_after(command_id, &counts_before, S5_MOVE_FAILED);
        return S5_MOVE_FAILED;
    }

    tm_printf(
        (const UB *)"[DETOUR] move %u queued: straight %d mm at %d mm/s\n",
        (unsigned)command_id, s5_mm(distance_mm), s5_mm(speed_mm_s));
    move = wait_move(command_id,
                     (fabs(distance_mm) / speed_mm_s) + S5_MOVE_SLACK_S,
                     b_watch_line, before.motion.centre, p_line_centre_m);

    if ((S5_MOVE_LINE == move) || (S5_MOVE_REJOIN == move)
        || (S5_MOVE_STALLED == move))
    {
        if (!s5_wait_still())
        {
            tm_printf(
                (const UB *)"[DETOUR] move %u could not confirm line stop\n",
                (unsigned)command_id);
            detour_why(S5_WHY_NOT_STILL, (int)command_id, 0);
            log_encoder_after(command_id, &counts_before, S5_MOVE_FAILED);
            return S5_MOVE_FAILED;
        }
    }

    log_encoder_after(command_id, &counts_before, move);

    if (!s5_status(&after))
    {
        motion_stop();
        tm_printf((const UB *)"[DETOUR] move %u lost final position; holding\n",
                  (unsigned)command_id);
        detour_why(S5_WHY_NO_STATUS, (int)command_id, 0);
        return S5_MOVE_FAILED;
    }

    {
        float64_t travelled
            = (after.motion.centre - before.motion.centre) * 1000.0;
        float64_t heading = 0.5 * (heading_deg(&before) + heading_deg(&after))
                            * S5_DEG_TO_RAD;

        g_s5_x += travelled * cos(heading);
        g_s5_y += travelled * sin(heading);
    }

    if (S5_MOVE_DONE == move)
    {
        tm_printf((const UB *)"[DETOUR] move %u complete: straight, drive took "
                              "%d ms\n",
                  (unsigned)command_id,
                  s5_mm((motion_hal_time() - submit_start) * 1000.0));
    }

    return move;
}

/* Turn in place to a heading relative to the one at the stop, so turn
   errors do not add up over the detour. */
static s5_move_t
turn_to (float64_t target_deg)
{
    robot_status_t   status;
    encoder_sample_t counts_before;
    uint32_t         command_id;
    motion_result_t  result;
    s5_move_t        move;
    float64_t        turn;
    float64_t        submit_start;

    if (!s5_status(&status))
    {
        return S5_MOVE_FAILED;
    }

    turn = target_deg - heading_deg(&status);

    if (fabs(turn) < S5_MIN_TURN_DEG)
    {
        return S5_MOVE_DONE;
    }

    command_id    = next_id();
    submit_start  = motion_hal_time();
    counts_before = s5_log_encoders("move-before", command_id);

    do
    {
        result = motion_turn_degrees(SOURCE_OBSTACLE, command_id, turn);
    } while ((R_BUSY == result)
             && (retry_busy(submit_start, status.motion.generation)));

    if (R_OK != result)
    {
        tm_printf((const UB *)"[DETOUR] turn %d deg refused (%u)\n",
                  s5_mm(turn), (unsigned)result);
        detour_why(S5_WHY_REFUSED, (int)result, s5_mm(turn));
        log_encoder_after(command_id, &counts_before, S5_MOVE_FAILED);
        return S5_MOVE_FAILED;
    }

    tm_printf(
        (const UB *)"[DETOUR] move %u queued: turn %d deg to heading %d deg\n",
        (unsigned)command_id, s5_mm(turn), s5_mm(target_deg));
    move = wait_move(command_id, S5_TURN_TIMEOUT_S, false, 0.0, NULL);
    log_encoder_after(command_id, &counts_before, move);

    if (S5_MOVE_DONE == move)
    {
        tm_printf((const UB *)"[DETOUR] move %u complete: turn\n",
                  (unsigned)command_id);
    }

    return move;
}

static bool
detour_ping (void * p_context, float64_t * p_range_mm)
{
    float64_t      time_s;
    bool *         p_failed = p_context;
    s5_us_status_t result;

    if (*p_failed)
    {
        return false;
    }

    result    = s5_us_ping(p_range_mm, &time_s);
    *p_failed = (S5_US_NO_PULSE == result) || (S5_US_BUSY == result);
    return S5_US_OK == result;
}

typedef enum
{
    S5_LOOK_ECHO,
    S5_LOOK_CLEAR,
    S5_LOOK_FAILED,
    S5_LOOK_UNSURE
} s5_look_t;

/* Measurements are only useful while still at the same command generation.
   In particular, never resume a detour across a user stop/reset during a sweep.
   READY with no move active means the motors are off: see s5_still_moved(). */
static bool
look_still (uint32_t generation)
{
    robot_status_t status;

    return (s5_status(&status)) && (!status.motion.hold)
           && (!s5_still_moved(&status)) && (READY == status.motion.state)
           && (F_NONE == status.motion.fault)
           && (status.motion.generation == generation);
}

/* code is what detour_why() records unless STOP or a fault explains it */
static s5_look_t
look_failed (int angle_deg, s5_why_t code, const char * p_why,
             const s5_estimate_t * p_estimate)
{
    robot_status_t now;
    bool           b_now = s5_status(&now);

    tm_printf((const UB *)"[DETOUR] look at %d deg failed: %s (hold %u, state "
                          "%u, fault %u, moved %u, "
                          "pings %u of %u, %u agree)\n",
              angle_deg, p_why, b_now ? (unsigned)now.motion.hold : 9U,
              b_now ? (unsigned)now.motion.state : 9U,
              b_now ? (unsigned)now.motion.fault : 9U,
              (b_now && (s5_still_moved(&now))) ? 1U : 0U,
              (NULL != p_estimate) ? p_estimate->valid : 0U,
              (NULL != p_estimate) ? p_estimate->total : 0U,
              (NULL != p_estimate) ? p_estimate->support : 0U);

    if (b_now && ((now.motion.hold) || (F_NONE != now.motion.fault)))
    {
        detour_why((F_NONE != now.motion.fault) ? S5_WHY_MOTION_FAULT
                                                : S5_WHY_STOPPED,
                   (F_NONE != now.motion.fault) ? (int)now.motion.fault
                                                : (int)now.motion.state,
                   angle_deg);
    }
    else
    {
        detour_why(code, angle_deg,
                   (NULL != p_estimate) ? (int)p_estimate->valid : -1);
    }

    return S5_LOOK_FAILED;
}

/* A silent but responding sensor differs from a disconnected/stuck one.
   Do not turn sensor failures or conflicting readings into permission to drive.
   b_unsure: pings that disagree (an edge or a slanted face, half the beam on
   it) return S5_LOOK_UNSURE with the estimate in *p_estimate, for a caller
   that can judge every echo; otherwise they fail the look. */
static s5_look_t
look_ex (int angle_deg, float64_t * p_range_mm, bool b_unsure,
         s5_estimate_t * p_estimate)
{
    s5_estimate_t  estimate;
    bool           b_failed = false;
    robot_status_t status;

    s5_still_mark();

    if ((!s5_status(&status)) || (!look_still(status.motion.generation)))
    {
        return look_failed(angle_deg, S5_WHY_LOOK_MOVED,
                           "not still before the look", NULL);
    }

    s5_servo_move(angle_deg);

    if (!look_still(status.motion.generation))
    {
        return look_failed(angle_deg, S5_WHY_LOOK_MOVED,
                           "not still after the servo", NULL);
    }

    s5_sample_point(detour_ping, &b_failed, &estimate);
    *p_range_mm = estimate.range_mm;

    if (NULL != p_estimate)
    {
        *p_estimate = estimate;
    }

    if (b_failed)
    {
        return look_failed(angle_deg, S5_WHY_SENSOR_FAULT, "ultrasonic fault",
                           &estimate);
    }

    if (!look_still(status.motion.generation))
    {
        return look_failed(angle_deg, S5_WHY_LOOK_MOVED,
                           "not still after the pings", &estimate);
    }

    if (s5_estimate_usable(&estimate))
    {
        return S5_LOOK_ECHO;
    }

    if (S5_ECHO_NONE == estimate.status)
    {
        return S5_LOOK_CLEAR;
    }

    if (b_unsure)
    {
        return S5_LOOK_UNSURE;
    }

    return look_failed(angle_deg, S5_WHY_LOOK_UNSTABLE, "pings disagree",
                       &estimate);
}

static s5_look_t
look (int angle_deg, float64_t * p_range_mm)
{
    return look_ex(angle_deg, p_range_mm, false, NULL);
}

/* Nothing ahead within the move plus the front overhang and the margin */
static bool
path_clear (float64_t distance_mm)
{
    float64_t range_mm;
    float64_t needed_mm
        = distance_mm + (S5_FRONT_MM - S5_SERVO_AHEAD_MM) + S5_PASS_MARGIN_MM;
    s5_look_t result = look(S5_ANGLE_CENTRE_DEG, &range_mm);

    if (S5_LOOK_FAILED == result)
    {
        tm_printf(
            (const UB *)"[DETOUR] front measurement failed; not moving\n");
        return false;
    }

    if ((S5_LOOK_ECHO == result)
        && (s5_forward_mm(range_mm, S5_ANGLE_CENTRE_DEG) <= needed_mm))
    {
        tm_printf((const UB *)"[DETOUR] something %d mm ahead, need %d mm\n",
                  s5_mm(s5_forward_mm(range_mm, S5_ANGLE_CENTRE_DEG)),
                  s5_mm(needed_mm));
        detour_why(S5_WHY_AHEAD_BLOCKED,
                   s5_mm(s5_forward_mm(range_mm, S5_ANGLE_CENTRE_DEG)),
                   s5_mm(needed_mm));
        return false;
    }

    return true;
}

/* The five bearings cover a corridor, not just the centre ray. Widen it for
   the beam's uncertain echo bearing; normal no-echo permits progress, while
   an unusable sensor or a close echo refuses the move. Distances include the
   sensor's offset from the servo axis, as in the original scan. */
/* An echo at range_mm (from the transducer) seen at servo angle_deg could be in
   the corridor: needed_mm ahead of the servo axis, half_width_mm either side */
static bool
echo_in_path (float64_t range_mm, int angle_deg, float64_t needed_mm,
              float64_t half_width_mm)
{
    float64_t axis_range   = range_mm + S5_SENSOR_OFFSET_MM;
    float64_t bearing      = fabs((float64_t)(angle_deg - S5_ANGLE_CENTRE_DEG));
    float64_t near_bearing = fmax(0.0, bearing - S5_GUARD_BEAM_HALF_DEG);
    float64_t far_bearing  = fmin(90.0, bearing + S5_GUARD_BEAM_HALF_DEG);

    return ((axis_range * cos(far_bearing * S5_DEG_TO_RAD)) <= needed_mm)
           && ((axis_range * sin(near_bearing * S5_DEG_TO_RAD))
               <= half_width_mm);
}

static bool
echo_is_known (const robot_status_t * p_status, int angle_deg,
               float64_t range_mm)
{
    return (NULL != gp_s5_known)
           && (s5_echo_on_obstacle(gp_s5_known, g_s5_x, g_s5_y,
                                   heading_deg(p_status), angle_deg, range_mm));
}

static bool
clearance_sweep (float64_t distance_mm, const char * p_phase)
{
    float64_t needed
        = distance_mm + S5_FRONT_MM - S5_SERVO_AHEAD_MM + S5_PASS_MARGIN_MM;
    float64_t      half_width = S5_ROBOT_HALF_WIDTH_MM + S5_PASS_MARGIN_MM;
    robot_status_t status;
    float64_t      start_s = motion_hal_time();
    int            angle;

    g_s5_leg.sweep_blocked = false;
    s5_still_mark();

    if ((!s5_status(&status)) || (!look_still(status.motion.generation)))
    {
        robot_status_t now;
        bool           b_now = s5_status(&now);

        detour_why(!b_now                         ? S5_WHY_NO_STATUS
                   : (F_NONE != now.motion.fault) ? S5_WHY_MOTION_FAULT
                   : now.motion.hold              ? S5_WHY_STOPPED
                                                  : S5_WHY_NOT_STILL,
                   0, 0);
        return false;
    }

    tm_printf(
        (const UB *)"[DETOUR] clearance=%s sweep +/- %d deg, travel %d mm\n",
        p_phase, S5_GUARD_SWEEP_DEG, s5_mm(distance_mm));
    for (angle = S5_ANGLE_CENTRE_DEG - S5_GUARD_SWEEP_DEG;
         angle <= (S5_ANGLE_CENTRE_DEG + S5_GUARD_SWEEP_DEG);
         angle += S5_GUARD_SWEEP_STEP_DEG)
    {
        float64_t     range;
        s5_look_t     result;
        s5_estimate_t estimate;

        if (!look_still(status.motion.generation))
        {
            return false;
        }

        result = look_ex(angle, &range, true, &estimate);

        if ((S5_LOOK_FAILED == result)
            || (!look_still(status.motion.generation)))
        {
            robot_status_t now;
            bool           b_now = s5_status(&now);

            tm_printf((const UB *)"[DETOUR] clearance=%s measurement failed at "
                                  "%d deg; holding "
                                  "(look %u, hold %u, state %u, fault %u, "
                                  "moved %u)\n",
                      p_phase, angle, (unsigned)result,
                      b_now ? (unsigned)now.motion.hold : 9U,
                      b_now ? (unsigned)now.motion.state : 9U,
                      b_now ? (unsigned)now.motion.fault : 9U,
                      (b_now && (s5_still_moved(&now))) ? 1U : 0U);
            detour_why(!b_now                         ? S5_WHY_NO_STATUS
                       : (F_NONE != now.motion.fault) ? S5_WHY_MOTION_FAULT
                       : now.motion.hold              ? S5_WHY_STOPPED
                                                      : S5_WHY_MOVED,
                       angle, 0);
            return false;
        }

        if (S5_LOOK_UNSURE == result)
        {
            /* The pings disagree: some heard nothing, the rest no one range.
               The nearest decides; the obstacle being passed excuses them only
               if it explains the nearest and the furthest (and so all between).
             */
            tm_printf((const UB *)"[DETOUR] clearance=%s %d deg: pings "
                                  "disagree, %u of %u echoed, "
                                  "%d..%d mm\n",
                      p_phase, angle, estimate.valid, estimate.total,
                      s5_mm(estimate.nearest_mm), s5_mm(estimate.farthest_mm));

            if (!echo_in_path(estimate.nearest_mm, angle, needed, half_width))
            {
                continue;
            }

            if ((echo_is_known(&status, angle, estimate.nearest_mm))
                && (echo_is_known(&status, angle, estimate.farthest_mm)))
            {
                tm_printf((const UB *)"[DETOUR] clearance=%s %d deg: all of it "
                                      "is the obstacle "
                                      "being passed, ignored\n",
                          p_phase, angle);
                continue;
            }

            /* Could be something in the way: blocked, so turn further out or
             * retry */
            tm_printf((const UB *)"[DETOUR] clearance=%s blocked at %d deg, "
                                  "unsure, nearest %d mm\n",
                      p_phase, angle, s5_mm(estimate.nearest_mm));
            g_s5_leg.sweep_blocked = true;
            g_s5_block_angle       = (uint8_t)angle;
            g_s5_block_range       = (uint16_t)s5_mm(estimate.nearest_mm);
            return false;
        }

        if (S5_LOOK_ECHO == result)
        {
            if (echo_in_path(range, angle, needed, half_width))
            {
                /* The obstacle being driven round is not news: the plan keeps
                 * clear of it */
                if (echo_is_known(&status, angle, range))
                {
                    tm_printf(
                        (const UB *)"[DETOUR] clearance=%s %d deg: %d mm is "
                                    "the obstacle "
                                    "being passed (expected %d mm), ignored\n",
                        p_phase, angle, s5_mm(range),
                        s5_mm(s5_obstacle_range(gp_s5_known, g_s5_x, g_s5_y,
                                                heading_deg(&status), angle)
                              - S5_SENSOR_OFFSET_MM));
                    continue;
                }

                tm_printf((const UB *)"[DETOUR] clearance=%s blocked at %d "
                                      "deg, range %d mm\n",
                          p_phase, angle, s5_mm(range));
                g_s5_leg.sweep_blocked = true;
                g_s5_block_angle       = (uint8_t)angle;
                g_s5_block_range       = (uint16_t)s5_mm(range);
                return false;
            }
        }
    }

    /* The centre bearing of the sweep already covered the path ahead (its
       near bearing is 0 deg, so any echo within the move is refused), so no
       second centre look is needed. */
    if (!look_still(status.motion.generation))
    {
        return false;
    }

    tm_printf((const UB *)"[DETOUR] clearance=%s clear, sweep took %d ms\n",
              p_phase, s5_mm((motion_hal_time() - start_s) * 1000.0));
    return true;
}

static int
obstacle_look_angle (const s5_detour_plan_t * p_plan)
{
    return (S5_AVOID_LEFT == p_plan->side) ? S5_ANGLE_RIGHT_DEG
                                           : S5_ANGLE_LEFT_DEG;
}

static bool
passing_clearance (const s5_detour_plan_t * p_plan)
{
    float64_t range;
    s5_look_t result;

    if (!clearance_sweep(S5_PASS_STEP_MM, "parallel"))
    {
        if (g_s5_leg.sweep_blocked)
        {
            detour_why(S5_WHY_PARALLEL_BLOCKED, g_s5_block_angle,
                       g_s5_block_range);
        }

        return false;
    }

    result = look(obstacle_look_angle(p_plan), &range);

    if (S5_LOOK_FAILED == result)
    {
        tm_printf((const UB *)"[DETOUR] obstacle-side clearance measurement "
                              "failed; holding\n");
        return false;
    }

    if (S5_LOOK_ECHO == result)
    {
        float64_t gap = fabs(
            s5_lateral_mm(range, (float64_t)obstacle_look_angle(p_plan)));
        float64_t needed = S5_ROBOT_HALF_WIDTH_MM + S5_PASS_MARGIN_MM;

        if (gap < needed)
        {
            tm_printf((const UB *)"[DETOUR] insufficient sideways clearance: "
                                  "%d mm, need %d mm; holding\n",
                      s5_mm(gap), s5_mm(needed));
            detour_why(S5_WHY_SIDE_TOO_CLOSE, s5_mm(gap), s5_mm(needed));
            return false;
        }
    }

    return true;
}

static bool
go_out (const s5_detour_plan_t * p_plan)
{
    float64_t remaining   = p_plan->diagonal_mm + S5_DIAGONAL_EXTRA_MM;
    float64_t heading_deg = p_plan->turn_deg;
    float64_t heading     = heading_deg * S5_DEG_TO_RAD;
    float64_t sign        = (p_plan->turn_deg < 0.0) ? -1.0 : 1.0;
    bool      b_travelled = false; /* any of the diagonal driven yet */

    detour_stage(CRASH_OBSTACLE_OUT, "out");
    tm_printf((const UB *)"[DETOUR] out: back %d mm, turn %d deg, diagonal %d "
                          "mm -> %d mm %s of the line\n",
              s5_mm(p_plan->backup_mm), s5_mm(p_plan->turn_deg),
              s5_mm(p_plan->diagonal_mm), s5_mm(fabs(p_plan->shift_mm)),
              (p_plan->shift_mm >= 0.0) ? "left" : "right");

    if ((S5_MOVE_DONE != straight(-p_plan->backup_mm, false, NULL))
        || (S5_MOVE_DONE != turn_to(p_plan->turn_deg)))
    {
        return false;
    }

    /* The diagonal is the commanded distance: no compensation for a move
       that finishes a few mm short.  A remainder below S5_MIN_DIAGONAL_STEP_MM
       is too short for the wheels to move at all, so it is dropped. */
    while (remaining >= S5_MIN_DIAGONAL_STEP_MM)
    {
        float64_t step     = fmin(remaining, S5_DIAGONAL_STEP_MM);
        float64_t before_x = g_s5_x;
        float64_t before_y = g_s5_y;
        float64_t progress;

        /* Do not leave an undrivable tail: share the last two steps. */
        if ((remaining > S5_DIAGONAL_STEP_MM)
            && ((remaining - S5_DIAGONAL_STEP_MM) < S5_MIN_DIAGONAL_STEP_MM))
        {
            step = 0.5 * remaining;
        }

        while (!clearance_sweep(step, "diagonal"))
        {
            /* Blocked: turn further out and look again.  A fault is not
             * blocked. */
            float64_t steeper = heading_deg + (sign * S5_OUT_RETRY_STEP_DEG);

            if ((!g_s5_leg.sweep_blocked)
                || (fabs(steeper) > (S5_OUT_RETRY_MAX_DEG + 1e-9)))
            {
                tm_printf((const UB *)"[DETOUR] diagonal not clear at %d deg; "
                                      "holding\n",
                          s5_mm(heading_deg));

                if (g_s5_leg.sweep_blocked)
                {
                    detour_why(S5_WHY_DIAGONAL_BLOCKED, g_s5_block_angle,
                               g_s5_block_range);
                    /* Still where it turned out: the caller can go back and try
                     * another way */
                    g_s5_leg.out_blocked = !b_travelled;
                }

                return false;
            }

            tm_printf((const UB *)"[DETOUR] turning further out: %d deg\n",
                      s5_mm(steeper));

            if (S5_MOVE_DONE != turn_to(steeper))
            {
                return false;
            }

            /* Same sideways shift in a shorter diagonal */
            remaining *= sin(fabs(heading_deg) * S5_DEG_TO_RAD)
                         / sin(fabs(steeper) * S5_DEG_TO_RAD);
            heading_deg = steeper;
            heading     = heading_deg * S5_DEG_TO_RAD;
            step        = fmin(remaining, S5_DIAGONAL_STEP_MM);
        }

        b_travelled = true;

        if (S5_MOVE_DONE != straight(step, false, NULL))
        {
            return false;
        }

        progress = ((g_s5_x - before_x) * cos(heading))
                   + ((g_s5_y - before_y) * sin(heading));

        if (progress < 0.5)
        {
            tm_printf((const UB *)"[DETOUR] diagonal made no forward progress; "
                                  "holding\n");
            detour_why(S5_WHY_NO_PROGRESS, s5_mm(progress), 0);
            return false;
        }

        remaining -= step;
    }

    return (S5_MOVE_DONE == turn_to(0.0)) && (passing_clearance(p_plan));
}

/* Wait for a sensor on the line, debounced, while stationary */
static bool
line_confirm (void)
{
    float64_t      start = motion_hal_time();
    robot_status_t status;

    while ((motion_hal_time() - start) < S5_LINE_CONFIRM_S)
    {
        if ((s5_status(&status)) && (line_seen(&status, motion_hal_time())))
        {
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
static bool
rejoin_after_stop (void)
{
    robot_status_t status;
    float64_t      overshoot_mm;
    float64_t      back_mm;

    g_s5_leg.rejoin_armed = false;

    if ((!release_hold()) || (!s5_status(&status)))
    {
        return false;
    }

    overshoot_mm = (status.motion.centre - g_s5_rejoin_centre_m) * 1000.0;
    /* Back over the overshoot: reverse after a forward move, forward after
     * reversing */
    back_mm = -overshoot_mm;

    if (fabs(back_mm) < S5_MIN_DIAGONAL_STEP_MM)
    {
        back_mm = (overshoot_mm < 0.0) ? S5_MIN_DIAGONAL_STEP_MM
                                       : -S5_MIN_DIAGONAL_STEP_MM;
    }

    tm_printf((const UB *)"[DETOUR] line at x %d y %d, heading %d deg, stopped "
                          "%d mm on; interrupting the detour\n",
              s5_mm(g_s5_x), s5_mm(g_s5_y), s5_mm(heading_deg(&status)),
              s5_mm(overshoot_mm));

    if ((!line_confirm())
        && ((S5_MOVE_DONE != straight(back_mm, false, NULL))
            || (!line_confirm())))
    {
        tm_printf((const UB *)"[DETOUR] line not under the sensors after "
                              "stopping; holding\n");
        detour_why(S5_WHY_LINE_LOST_AT_STOP, s5_mm(g_s5_x), s5_mm(g_s5_y));
        return false;
    }

    return true;
}

/* ---- the pass: one long move, measured while it runs ---- */

typedef struct
{
    uint32_t  id;
    float64_t start;
    float64_t timeout_s;
    /* s5_x and s5_y are up to date for this travel */
    float64_t        centre_m;
    float64_t        heading_deg; /* and this heading */
    encoder_sample_t counts;
    s5_move_t        result; /* S5_MOVE_RUNNING until the move is over */
    bool             b_sensor_failed;
} s5_drive_t;

static void
drive_track (s5_drive_t * p_drive, const robot_status_t * p_status)
{
    float64_t heading = heading_deg(p_status);
    float64_t middle  = 0.5 * (p_drive->heading_deg + heading) * S5_DEG_TO_RAD;
    float64_t travelled
        = (p_status->motion.centre - p_drive->centre_m) * 1000.0;

    g_s5_x += travelled * cos(middle);
    g_s5_y += travelled * sin(middle);
    p_drive->centre_m    = p_status->motion.centre;
    p_drive->heading_deg = heading;
}

static bool
drive_start (s5_drive_t * p_drive, float64_t distance_mm)
{
    robot_status_t  before;
    motion_result_t result;
    float64_t       submit_start;

    if (!s5_status(&before))
    {
        return false;
    }

    p_drive->id     = next_id();
    submit_start    = motion_hal_time();
    p_drive->counts = s5_log_encoders("move-before", p_drive->id);

    do
    {
        result = motion_steer_distance(SOURCE_OBSTACLE, p_drive->id,
                                       distance_mm / 1000.0, 0.0,
                                       S5_DRIVE_SPEED_MM_S / 1000.0);
    } while ((R_BUSY == result)
             && (retry_busy(submit_start, before.motion.generation)));

    if (R_OK != result)
    {
        tm_printf((const UB *)"[DETOUR] drive %d mm refused (%u)\n",
                  s5_mm(distance_mm), (unsigned)result);
        log_encoder_after(p_drive->id, &p_drive->counts, S5_MOVE_FAILED);
        return false;
    }

    tm_printf(
        (const UB *)"[DETOUR] move %u queued: pass, up to %d mm at %d mm/s\n",
        (unsigned)p_drive->id, s5_mm(distance_mm), s5_mm(S5_DRIVE_SPEED_MM_S));
    p_drive->start     = motion_hal_time();
    p_drive->timeout_s = (distance_mm / S5_DRIVE_SPEED_MM_S) + S5_MOVE_SLACK_S;
    p_drive->centre_m  = before.motion.centre;
    p_drive->heading_deg     = heading_deg(&before);
    p_drive->result          = S5_MOVE_RUNNING;
    p_drive->b_sensor_failed = false;
    g_s5_stall_since         = -1.0;
    return true;
}

static s5_move_t
drive_poll (s5_drive_t * p_drive)
{
    robot_status_t status;

    if (S5_MOVE_RUNNING == p_drive->result)
    {
        p_drive->result = move_poll(p_drive->id, p_drive->start,
                                    p_drive->timeout_s, false, 0.0, NULL);

        if (s5_status(&status))
        {
            drive_track(p_drive, &status);
        }
    }

    return p_drive->result;
}

static void
drive_wait (s5_drive_t * p_drive, unsigned wait_ms)
{
    float64_t end = motion_hal_time() + ((float64_t)wait_ms / 1000.0);

    while ((motion_hal_time() < end)
           && (S5_MOVE_RUNNING == drive_poll(p_drive)))
    {
        (void)tk_dly_tsk(S5_POLL_MS);
    }
}

/* Between pings, so that a fault or the line is seen within one ping (60 ms) */
static bool
drive_ping (void * p_context, float64_t * p_range_mm)
{
    s5_drive_t *   p_drive = p_context;
    float64_t      time_s;
    s5_us_status_t result;

    if ((S5_MOVE_RUNNING != drive_poll(p_drive)) || (p_drive->b_sensor_failed))
    {
        return false;
    }

    result = s5_us_ping(p_range_mm, &time_s);
    p_drive->b_sensor_failed
        = (S5_US_NO_PULSE == result) || (S5_US_BUSY == result);
    return S5_US_OK == result;
}

/* *p_x: where the robot was, halfway through the pings.  Pings that disagree
   are S5_LOOK_UNSURE, normal while passing an edge: stopping for them would
   abandon detours, and the next look is about 100 mm on. */
static s5_look_t
drive_look (s5_drive_t * p_drive, int angle_deg, float64_t * p_range_mm,
            float64_t * p_x)
{
    s5_estimate_t estimate;
    float64_t     first_x;

    drive_wait(p_drive, s5_servo_aim(angle_deg));
    first_x = g_s5_x;
    s5_sample_point(drive_ping, p_drive, &estimate);
    *p_x        = 0.5 * (first_x + g_s5_x);
    *p_range_mm = estimate.range_mm;

    if ((S5_MOVE_RUNNING != p_drive->result) || (p_drive->b_sensor_failed))
    {
        return S5_LOOK_FAILED;
    }

    if (s5_estimate_usable(&estimate))
    {
        return S5_LOOK_ECHO;
    }

    return (S5_ECHO_NONE == estimate.status) ? S5_LOOK_CLEAR : S5_LOOK_UNSURE;
}

/* Wait for the stop, then log and bring the pose up to date.  Leaves it held.
 */
static bool
drive_settle (s5_drive_t * p_drive)
{
    robot_status_t status;

    if ((!s5_wait_still()) || (!s5_status(&status)))
    {
        tm_printf((const UB *)"[DETOUR] move %u could not confirm the stop\n",
                  (unsigned)p_drive->id);
        detour_why(S5_WHY_NOT_STILL, (int)p_drive->id, 0);
        return false;
    }

    drive_track(p_drive, &status);
    log_encoder_after(p_drive->id, &p_drive->counts, p_drive->result);
    return true;
}

/* The move ended by itself: the line interrupted it, or something went wrong */
static bool
drive_ended (s5_drive_t * p_drive)
{
    if (S5_MOVE_REJOIN == p_drive->result)
    {
        g_s5_leg.rejoined = (drive_settle(p_drive)) && (rejoin_after_stop());
        return g_s5_leg.rejoined;
    }

    if (S5_MOVE_STALLED == p_drive->result)
    {
        g_s5_leg.pass_stalled = drive_settle(p_drive);
        return false;
    }

    if (S5_MOVE_DONE == p_drive->result)
    {
        tm_printf((const UB *)"[DETOUR] drove the whole pass without the "
                              "obstacle ending\n");
        detour_why(S5_WHY_OBSTACLE_TOO_LONG, s5_mm(g_s5_x), s5_mm(g_s5_y));
    }

    return false;
}

/* Sensor failed, or something ahead: the caller stops too, but this is moving
 */
static bool
drive_refuse (s5_why_t code, const char * p_reason, int value_mm)
{
    motion_stop();
    tm_printf((const UB *)"[DETOUR] %s (%d mm) while passing; holding\n",
              p_reason, value_mm);
    detour_why(code, value_mm, s5_mm(g_s5_x));
    return false;
}

/* True: go on.  An unsure look ahead counts if its nearest agreeing echo is
 * close. */
static bool
drive_look_ahead (s5_drive_t * p_drive)
{
    float64_t range_mm;
    float64_t look_x;
    float64_t needed
        = S5_PASS_STEP_MM + S5_FRONT_MM - S5_SERVO_AHEAD_MM + S5_PASS_MARGIN_MM;
    s5_look_t result
        = drive_look(p_drive, S5_ANGLE_CENTRE_DEG, &range_mm, &look_x);

    if (S5_MOVE_RUNNING != p_drive->result)
    {
        return false;
    }

    if (S5_LOOK_FAILED == result)
    {
        return drive_refuse(S5_WHY_PASS_LOOK_FAILED, "front measurement failed",
                            0);
    }

    if ((S5_LOOK_CLEAR != result)
        && (s5_forward_mm(range_mm, S5_ANGLE_CENTRE_DEG) <= needed))
    {
        return drive_refuse(
            S5_WHY_PASS_AHEAD_BLOCKED, "something ahead",
            s5_mm(s5_forward_mm(range_mm, S5_ANGLE_CENTRE_DEG)));
    }

    return true;
}

static bool
pass_steps (const s5_detour_plan_t * p_plan)
{
    int        side_deg = obstacle_look_angle(p_plan);
    float64_t  start_x  = g_s5_x;
    float64_t  clear_x;
    float64_t  range_mm;
    float64_t  look_x;
    s5_look_t  result;
    s5_pass_t  pass;
    s5_drive_t drive;

    detour_stage(CRASH_OBSTACLE_PASS, "pass");
    s5_pass_init(&pass, p_plan);

    if (!drive_start(&drive, S5_PASS_MAX_MM
                                 + hypot(S5_REAR_MM, S5_ROBOT_HALF_WIDTH_MM)
                                 + S5_PASS_MARGIN_MM + S5_PASS_STEP_MM))
    {
        return false;
    }

    for (;;)
    {
        result = drive_look(&drive, side_deg, &range_mm, &look_x);

        if (S5_MOVE_RUNNING != drive.result)
        {
            return drive_ended(&drive);
        }

        if (S5_LOOK_FAILED == result)
        {
            return drive_refuse(S5_WHY_PASS_LOOK_FAILED,
                                "side measurement failed", 0);
        }

        if (S5_LOOK_UNSURE == result)
        {
            tm_printf((const UB *)"[DETOUR] pass x %d y %d: side pings "
                                  "disagree, ignored\n",
                      s5_mm(look_x), s5_mm(g_s5_y));
        }
        else
        {
            (void)s5_pass_observe(&pass, p_plan, look_x, g_s5_y,
                                  S5_LOOK_ECHO == result, range_mm);
            tm_printf((const UB *)"[DETOUR] pass x %d y %d: side %d mm -> "
                                  "obstacle to x %d, clear %u/%u\n",
                      s5_mm(look_x), s5_mm(g_s5_y),
                      (S5_LOOK_ECHO == result) ? s5_mm(range_mm) : 0,
                      s5_mm(pass.rear_mm), pass.clear_run, S5_PASS_CONFIRM);
        }

        if (pass.b_ended)
        {
            break;
        }

        if ((g_s5_x - start_x) >= S5_PASS_MAX_MM)
        {
            return drive_refuse(S5_WHY_OBSTACLE_TOO_LONG,
                                "obstacle has not ended",
                                s5_mm(S5_PASS_MAX_MM));
        }

        if (!drive_look_ahead(&drive))
        {
            return (S5_MOVE_RUNNING != drive.result) ? drive_ended(&drive)
                                                     : false;
        }
    }

    clear_x = s5_pass_clear_x(&pass);
    detour_stage(CRASH_OBSTACLE_REAR_CLEAR, "rear-clearance");
    tm_printf((const UB *)"[DETOUR] obstacle ends by x %d (%u echoes); rear "
                          "clears it at x %d\n",
              s5_mm(fmax(pass.rear_mm, pass.end_mm)), pass.seen,
              s5_mm(clear_x));

    /* A look ahead costs about 50 mm of travel: only worth it for a long way */
    if (((clear_x - g_s5_x) > S5_PASS_STEP_MM) && (!drive_look_ahead(&drive)))
    {
        return (S5_MOVE_RUNNING != drive.result) ? drive_ended(&drive) : false;
    }

    while (g_s5_x < clear_x)
    {
        if (S5_MOVE_RUNNING != drive_poll(&drive))
        {
            return drive_ended(&drive);
        }

        (void)tk_dly_tsk(S5_POLL_MS);
    }

    motion_stop();
    /* stopped on purpose: the move's job is done */
    drive.result = S5_MOVE_DONE;
    return (drive_settle(&drive)) && (release_hold());
}

/* The pass stopped against something (the held robot is still).  Tilt away from
   the obstacle, a step more each time, look that way, back out along it, and
   turn parallel again so that the pass can start over from further out.
   True: ready to resume. */
static bool
escape_stall (const s5_detour_plan_t * p_plan, unsigned attempt)
{
    float64_t sign = (S5_AVOID_LEFT == p_plan->side) ? 1.0 : -1.0;
    unsigned  step;
    bool      b_held = true;

    for (step = attempt + 1U; step <= S5_ESCAPE_TRIES; step++)
    {
        float64_t tilt = sign * S5_ESCAPE_STEP_DEG * (float64_t)step;
        s5_move_t move;

        tm_printf(
            (const UB *)"[DETOUR] stalled at x %d y %d: tilting %s to %d deg\n",
            s5_mm(g_s5_x), s5_mm(g_s5_y), (sign > 0.0) ? "left" : "right",
            s5_mm(tilt));

        if (b_held && (!release_hold()))
        {
            return false;
        }

        b_held = false;

        if (S5_MOVE_DONE != turn_to(tilt))
        {
            return false;
        }

        if (!clearance_sweep(S5_ESCAPE_MM, "escape"))
        {
            if (!g_s5_leg.sweep_blocked)
            {
                return false;
            }

            continue; /* blocked that way: tilt further */
        }

        move = straight(S5_ESCAPE_MM, false, NULL);

        if (S5_MOVE_STALLED == move)
        {
            b_held = true; /* stuck again: tilt further */
            continue;
        }

        return (S5_MOVE_DONE == move) && (S5_MOVE_DONE == turn_to(0.0));
    }

    tm_printf((const UB *)"[DETOUR] no way out of the stall; holding\n");
    detour_why(S5_WHY_STALL_NO_ESCAPE, s5_mm(g_s5_x), s5_mm(g_s5_y));
    motion_stop();
    return false;
}

/* The line can interrupt this and the return leg, but not the way out, nor a
   stall escape: only once parallel again and S5_REJOIN_AFTER_MM further on. */
static bool
pass_obstacle (const s5_detour_plan_t * p_plan)
{
    bool     b_ok;
    unsigned attempt = 0U;

    g_s5_leg.rejoined = false;

    for (;;)
    {
        robot_status_t status;

        if (!s5_status(&status))
        {
            return false;
        }

        g_s5_rejoin_after_m
            = status.motion.centre + (S5_REJOIN_AFTER_MM / 1000.0);
        g_s5_leg.rejoin_armed = true;
        g_s5_leg.pass_stalled = false;
        b_ok                  = pass_steps(p_plan);
        g_s5_leg.rejoin_armed = false;

        if (b_ok || (!g_s5_leg.pass_stalled) || (attempt >= S5_ESCAPE_TRIES))
        {
            break;
        }

        s5_detour_reason_clear(); /* a stall it escapes is not why it gave up */

        if (!escape_stall(p_plan, attempt))
        {
            break;
        }

        attempt++;
    }

    return b_ok;
}

/* Drive mm (negative: reverse) watching for the noted line: black under the
   sensors within S5_LINE_WINDOW_MM of the noted point ends it there.
   True: stopped on it, s5_rejoin_centre_m set, for rejoin_after_stop(). */
static bool
drive_onto_noted (float64_t distance_mm)
{
    float64_t line_centre_m = 0.0;
    s5_move_t move;

    g_s5_leg.line_goal    = true;
    g_s5_rejoin_after_m   = -INFINITY;
    g_s5_leg.rejoin_armed = true;
    move                  = straight(distance_mm, true, &line_centre_m);
    g_s5_leg.rejoin_armed = false;
    g_s5_leg.line_goal    = false;

    switch (move)
    {
        case S5_MOVE_REJOIN:
            return true;
        case S5_MOVE_LINE:
            g_s5_rejoin_centre_m = line_centre_m;
            return true;
        case S5_MOVE_DONE:
            tm_printf(
                (const UB *)"[DETOUR] the noted line is not there either\n");
            detour_why(S5_WHY_NOTED_LINE_NOT_FOUND, s5_mm(g_s5_x),
                       s5_mm(g_s5_y));
            return false;
        default:
            return false;
    }
}

/* The way back found no line where the straight path should be, so the line
   curved: drive back to where both sensors last saw it and hand over there.
   The obstacle is not excused on this leg (s5_known): it was not planned. */
static bool
return_to_noted (void)
{
    robot_status_t           status;
    const s5_detour_plan_t * p_known = gp_s5_known;
    float64_t                delta_x = g_s5_noted_x - g_s5_x;
    float64_t                delta_y = g_s5_noted_y - g_s5_y;
    float64_t                bearing = atan2(delta_y, delta_x) / S5_DEG_TO_RAD;
    float64_t                length;
    bool                     b_clear;

    if ((!g_s5_leg.line_noted) || (!s5_status(&status)))
    {
        return false;
    }

    /* The same direction, as near the present heading as possible */
    while ((bearing - heading_deg(&status)) > 180.0)
    {
        bearing -= 360.0;
    }

    while ((bearing - heading_deg(&status)) < -180.0)
    {
        bearing += 360.0;
    }

    length = fmax(0.0, hypot(delta_x, delta_y) - S5_IR_AHEAD_MM)
             + S5_NOTED_OVERSHOOT_MM;
    tm_printf((const UB *)"[DETOUR] no line on the straight path; back to the "
                          "line seen at "
                          "x %d y %d: turn to %d deg, up to %d mm\n",
              s5_mm(g_s5_noted_x), s5_mm(g_s5_noted_y), s5_mm(bearing),
              s5_mm(length));

    if (S5_MOVE_DONE != turn_to(bearing))
    {
        return false;
    }

    gp_s5_known = NULL;
    b_clear     = clearance_sweep(length, "to-noted-line");
    gp_s5_known = p_known;

    if (!b_clear)
    {
        if (g_s5_leg.sweep_blocked)
        {
            detour_why(S5_WHY_NOTED_LINE_BLOCKED, g_s5_block_angle,
                       g_s5_block_range);
        }

        return false;
    }

    return drive_onto_noted(length);
}

/* The line was crossed on the way back itself, away from the straight path,
   and there was none on the straight path: the line curved.  Reverse along the
   track just driven, over where it was crossed, rather than turning round: no
   turn, so no new heading error (a spin of 180 deg on 18 Nov missed a 2 cm
   line by more than the overshoot).  The robot has just driven this track, so
   it is not checked again; the ultrasonic only looks forward. */
static bool
reverse_to_noted (void)
{
    robot_status_t status;
    float64_t      back_mm;

    if (!s5_status(&status))
    {
        return false;
    }

    back_mm = ((status.motion.centre - g_s5_noted_centre_m) * 1000.0)
              + S5_NOTED_OVERSHOOT_MM;
    tm_printf((const UB *)"[DETOUR] no line on the straight path; reversing up "
                          "to %d mm to the "
                          "line crossed at x %d y %d\n",
              s5_mm(back_mm), s5_mm(g_s5_noted_x), s5_mm(g_s5_noted_y));
    return drive_onto_noted(-back_mm);
}

static bool
return_to_line (const s5_detour_plan_t * p_plan)
{
    float64_t line_centre_m = 0.0;
    s5_move_t move;

    detour_stage(CRASH_OBSTACLE_RETURN_TURN, "return-turn");
    s5_servo_move(S5_ANGLE_CENTRE_DEG);
    tm_printf(
        (const UB *)"[DETOUR] back: turn %d deg, up to %d mm to the line\n",
        s5_mm(-S5_RETURN_DEG * ((S5_AVOID_LEFT == p_plan->side) ? 1.0 : -1.0)),
        s5_mm(s5_return_length(g_s5_y)));

    if (S5_MOVE_DONE
        != turn_to((S5_AVOID_LEFT == p_plan->side) ? -S5_RETURN_DEG
                                                   : S5_RETURN_DEG))
    {
        return false;
    }

    detour_stage(CRASH_OBSTACLE_RETURN_LOOK, "return-path-check");

    if (!path_clear(s5_return_length(g_s5_y)))
    {
        return false;
    }

    detour_stage(CRASH_OBSTACLE_RETURN_DRIVE, "return-to-line");
    g_s5_rejoin_after_m   = -INFINITY;
    g_s5_leg.rejoin_armed = true;
    gb_s5_noted_here      = false;
    move = straight(s5_return_length(g_s5_y), true, &line_centre_m);
    g_s5_leg.rejoin_armed = false;

    switch (move)
    {
        case S5_MOVE_REJOIN:
        break;

        case S5_MOVE_LINE:
            g_s5_rejoin_centre_m = line_centre_m;
        break;

        case S5_MOVE_DONE:
            tm_printf((const UB *)"[DETOUR] line not found where expected (now "
                                  "%d mm off it)\n",
                      s5_mm(g_s5_y));

            /* Crossed on this leg: back along it.  Seen beside the obstacle:
             * turn and go there. */
            if (!(gb_s5_noted_here ? reverse_to_noted() : return_to_noted()))
            {
                detour_why(S5_WHY_LINE_NOT_FOUND, s5_mm(g_s5_y), s5_mm(g_s5_x));
                return false;
            }
        break;

        default:
            return false;
    }

    detour_stage(CRASH_OBSTACLE_ALIGN, "line-handover");
    return rejoin_after_stop();
}

/* Held, still and fully STOPPED, the state the scan requires.  s5_wait_still()
   also accepts READY, which Motion Control reports for a few ms after
   motion_stop() on a robot that was already still. */
static bool
wait_stopped (void)
{
    float64_t      start = motion_hal_time();
    robot_status_t status;

    while ((motion_hal_time() - start) < S5_SETTLE_TIMEOUT_S)
    {
        if ((s5_status(&status)) && (status.motion.hold)
            && (status.motion.stationary) && (STOPPED == status.motion.state)
            && (F_NONE == status.motion.fault))
        {
            return true;
        }

        (void)tk_dly_tsk(S5_POLL_MS);
    }

    return false;
}

bool
s5_back_off (float64_t distance_mm)
{
    robot_status_t status;

    if ((!release_hold()) || (!s5_status(&status)))
    {
        return false;
    }

    g_s5_heading0 = status.motion.heading;
    g_s5_x        = 0.0;
    g_s5_y        = 0.0;

    if (S5_MOVE_DONE != straight(-distance_mm, false, NULL))
    {
        motion_stop();
        return false;
    }

    motion_stop();
    return wait_stopped();
}

bool
s5_square_up (float64_t degrees, float64_t * p_turned_deg)
{
    robot_status_t before;
    robot_status_t after;
    s5_move_t      move;

    if ((!release_hold()) || (!s5_status(&before)))
    {
        return false;
    }

    g_s5_heading0 = before.motion.heading;
    move          = turn_to(degrees);
    motion_stop();

    if (((S5_MOVE_DONE != move) && (S5_MOVE_STALLED != move))
        || (!wait_stopped()))
    {
        return false;
    }

    (void)tk_dly_tsk(S5_SQUARE_SETTLE_MS);

    if ((!wait_stopped()) || (!s5_status(&after)))
    {
        return false;
    }

    *p_turned_deg
        = (after.motion.heading - before.motion.heading) / S5_DEG_TO_RAD;
    return true;
}

/* The way out was blocked before any of the diagonal was driven: turn back to
   the heading it stopped at and drive forward over the reversing, so that the
   next plan starts from the pose the scan was taken at.  True: held and still
   there. */
static bool
back_to_start (const s5_detour_plan_t * p_plan)
{
    float64_t back_mm = hypot(g_s5_x, g_s5_y - p_plan->start_y_mm);

    tm_printf((const UB *)"[DETOUR] way out blocked before leaving: back to "
                          "where it stopped "
                          "(%d mm, heading %d deg)\n",
              s5_mm(back_mm), s5_mm(-p_plan->tilt_deg));

    if (S5_MOVE_DONE != turn_to(-p_plan->tilt_deg))
    {
        return false;
    }

    if (back_mm >= S5_MIN_MOVE_MM)
    {
        /* Where it just was, but the obstacle could have moved closer meanwhile
         */
        if ((!path_clear(back_mm))
            || (S5_MOVE_DONE != straight(back_mm, false, NULL)))
        {
            return false;
        }
    }

    motion_stop();

    if (!wait_stopped())
    {
        detour_why(S5_WHY_NOT_STILL, 0, 0);
        return false;
    }

    return true;
}

bool
s5_detour_run (const s5_detour_plan_t * p_plan)
{
    robot_status_t status;

    g_s5_leg.out_blocked = false;
    g_s5_leg.line_noted  = false;
    g_s5_leg.line_goal   = false;

    if ((!p_plan->b_ok) || (!release_hold()) || (!s5_status(&status)))
    {
        return false;
    }

    /* Heading 0 is square to the face: every turn from here is relative to it
     */
    g_s5_heading0 = status.motion.heading + (p_plan->tilt_deg * S5_DEG_TO_RAD);
    g_s5_x        = 0.0;
    g_s5_y        = p_plan->start_y_mm;
    gp_s5_known   = p_plan;

    if (!go_out(p_plan))
    {
        gp_s5_known = NULL;

        if (g_s5_leg.out_blocked)
        {
            /* Not given up yet: the caller tries another way from the start */
            g_s5_leg.out_blocked = back_to_start(p_plan);

            if (g_s5_leg.out_blocked)
            {
                return false;
            }
        }

        tm_printf((const UB *)"[DETOUR] gave up at x %d y %d, heading %d deg\n",
                  s5_mm(g_s5_x), s5_mm(g_s5_y),
                  s5_status(&status) ? s5_mm(heading_deg(&status)) : 0);
        return false;
    }

    if ((!pass_obstacle(p_plan))
        || ((!g_s5_leg.rejoined) && (!return_to_line(p_plan))))
    {
        gp_s5_known = NULL;
        tm_printf((const UB *)"[DETOUR] gave up at x %d y %d, heading %d deg\n",
                  s5_mm(g_s5_x), s5_mm(g_s5_y),
                  s5_status(&status) ? s5_mm(heading_deg(&status)) : 0);
        return false;
    }

    gp_s5_known = NULL;
    tm_printf((const UB *)"[DETOUR] back on the line at x %d%s; line following "
                          "resumes\n",
              s5_mm(g_s5_x),
              g_s5_leg.rejoined ? " (interrupted mid-pass)" : "");
    return true;
}

bool
s5_detour_retryable (void)
{
    return g_s5_leg.out_blocked;
}

#endif

/*** end of file ***/
