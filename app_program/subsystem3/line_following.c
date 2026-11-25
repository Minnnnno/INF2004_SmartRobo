/* Only images with a working line_loop() build and create line_task
 * (ROBO_LINE_TASK in subsystems.h; host tests define the same macros). */
#if defined(ROBO_SAFE_MOTOR_TEST) || defined(ROBO_LINE_STEERING)
#ifdef ROBO_LINE_TEST_HOST
#include "../../include/float_types.h"
#include <stdint.h>
#include "line_test_fake_hal.h"
#else
#include "subsystems.h"
#include "../subsystem2/template_hal.h"
#include <tm/tmonitor.h>
#endif
#include "board_config.h"
#include "ir_io.h"
#ifndef ROBO_BARCODE_TASK
/* Host tests do not include subsystems.h: no barcode reader. */
#define ROBO_BARCODE_TASK 0
#endif

/* Each firmware profile supplies its own line_loop(); CMake keeps them
 * exclusive. */
#if defined(ROBO_SAFE_MOTOR_TEST)
#include "motor_test.h"
static volatile bool gb_sensor_paused;

static void line_loop(void);

void
line_forward_pause (void)
{
    gb_sensor_paused = true;
}

/* Either sensor HIGH drives forward; both LOW stops. A console pause lasts
 * until both read LOW. */
static void
line_loop (void)
{
    bool b_sensor_running = false;

    for (;;)
    {
        unsigned right  = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre = (unsigned)gpio_get(BOARD_LINE_C);
        bool     b_move = (1U == right) || (1U == centre);

        if (!motor_test_active())
        {
            b_sensor_running = false;
        }

        if (!b_move)
        {
            gb_sensor_paused = false;
        }

        if (b_move && (!gb_sensor_paused))
        {
            b_sensor_running = motor_test_sensor_forward();
        }
        else if (b_sensor_running)
        {
            motor_test_stop();
            b_sensor_running = false;
        }
        else
        {
            /* Nothing to do in any other case. */
        }

        /* Console labels predate the right/centre naming used by line steering.
         */
        tm_printf((const UB *)"left_digital=%u right_digital=%u "
                              "move_condition=%u sensor_forward_active=%u\n",
                  right, centre, (unsigned)b_move, (unsigned)b_sensor_running);
        (void)tk_dly_tsk(100U);
    }
}

#elif defined(ROBO_IR_MONITOR)
/* Sensor-only image: no motion requests; '\r' rewrites one console line.
 * Also shows the barcode IR (GP28) so its wiring can be checked; barcode_task
 * stays idle. */
#if defined(ROBO_IR_MONITOR_ANALOG) && ROBO_IR_MONITOR_ANALOG
/* Analog variant (-DCMAKE_C_FLAGS=-DROBO_IR_MONITOR_ANALOG=1): also reads the
 * ADC on every IR pin, to see which module output (DO or AO) each pin is wired
 * to and what the potentiometer changes. The line pins stay digital inputs (the
 * ADC still reads them); the barcode pin is set up for analog only, as
 * barcode.c does. ADC channel = pin - 26: GP26 = ADC0, GP27 = ADC1, GP28 = ADC2
 * (ir_io.h). */
#define IRMON_ADC_AVERAGE 16U

static void line_loop(void);

static void
line_loop (void)
{
    if (!ir_adc_init(BOARD_BARCODE))
    {
        tm_printf(
            (const UB *)"ir_monitor_analog: ADC not responding, a= reads 0\n");
    }

    tm_printf((const UB *)"ir_monitor_analog: ADC 0..4095 (3.3 V); d = digital "
                          "read of the same pin\n");

    for (;;)
    {
        unsigned right  = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre = (unsigned)gpio_get(BOARD_LINE_C);

        tm_printf((const UB *)"centre GP%u d=%u a=%4u | right GP%u d=%u a=%4u "
                              "| barcode GP%u a=%4u\n",
                  (unsigned)BOARD_LINE_C, centre,
                  ir_adc_read(BOARD_LINE_C, IRMON_ADC_AVERAGE),
                  (unsigned)BOARD_LINE_R, right,
                  ir_adc_read(BOARD_LINE_R, IRMON_ADC_AVERAGE),
                  (unsigned)BOARD_BARCODE,
                  ir_adc_read(BOARD_BARCODE, IRMON_ADC_AVERAGE));
        (void)tk_dly_tsk(200U);
    }
}

#else

static void line_loop(void);

static void
line_loop (void)
{
    ir_input_init(BOARD_BARCODE);

    for (;;)
    {
        unsigned right   = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre  = (unsigned)gpio_get(BOARD_LINE_C);
        unsigned barcode = (unsigned)gpio_get(BOARD_BARCODE);

        tm_printf((const UB *)"\rcentre_ir=%u right_ir=%u barcode_ir=%u",
                  centre, right, barcode);
        (void)tk_dly_tsk(50U);
    }
}

#endif

#elif defined(ROBO_LINE_STEERING)
/* Line task states (line_state_t; printed as lstate=<name> every 0.5 s, and the
 * coarse ones also as "[S3-LINE] A -> B" when they change): STARTUP line_task:
 * pin setup, settle delay, wait for the IMU's level reference. WAITING    not
 * driving the line yet: no stable reading (debounce), or stopped and the line
 * not seen. FOLLOWING  line_follow(): straight, or correct towards the side the
 * line is on. BARCODE    following while a barcode pass is read: gentler
 * steering, and a short loss holds the heading. GRACE      a short loss: the
 * last follow command is repeated for LINE_GRACE_TIME. SEARCHING
 * line_search_step(): sweep both sides, turn back, reverse, sweep again
 * (sweep=<phase>). LOST       the search gave up (or timed out): zero-speed
 * refreshes until the line is seen again. HELD       someone else has the
 * wheels: Motion Control held or faulted, or subsystem 5 avoiding. MANEUVER   a
 * route action (route.h): brake, advance to the junction, pivot. STOPPED    a
 * failed route action: stopped until power-cycle (a failed barcode read only
 * logs and carries on). CAL_WAIT   GP21 calibration finished (done or failed):
 * stopped, no route action, until GP21 is pressed again. The barcode reader has
 * its own modes (barcode.c). */
#include "steering.h"
#include "motion_hal.h"
#include "line_rejoin.h"
#include "route.h"
/* Both IR sensors read black (1) when centred on the line; GP26 sits slightly
 * right of centre. */
#define DEG_TO_RAD             (0.017453292519943295)
#define LINE_SESSION           3001U
/* Target centre speed in m/s while following the line. */
#define LINE_SPEED             0.07
/* m/s per wheel when one sensor is off the line: the same inner/outer wheel
 * ratio as 0.0375 at 0.1 m/s. */
#define LINE_CORRECTION        (LINE_SPEED * 0.375)
/* m/s outer wheel while pivoting on the stopped inner wheel. Spinning in place
 * stalled the reversing wheel. */
#define LINE_SEARCH_SPEED      0.1
/* s to keep the last command after a loss (gaps, flicker). */
#define LINE_GRACE_TIME        0.2
/* rad each side of the last heading on the line. */
#define LINE_SWEEP_ANGLE       (60.0 * DEG_TO_RAD)
/* m/s reversing back to where the line was last seen. */
#define LINE_BACKUP_SPEED      0.075
#define LINE_BACKUP_EXTRA      0.02 /* m reversed beyond the overshoot. */
/* s backstop before holding still, e.g. if a wheel slips. */
#define LINE_SEARCH_TIMEOUT    15.0
/* Barcode window: while the barcode task reads a pass (barcode_code_pass()),
 * the car steers more gently (LINE_BC_CORRECTION), and a loss first holds the
 * heading of the last on-line sample at line speed for up to LINE_BC_HOLD_LOST
 * before the normal grace and search take over. Both end as soon as the heading
 * has turned LINE_BC_MAX_TURN from the pass start (a bend), and after
 * LINE_BC_HOLD_MAX. */
/* m from the start of the pass (big code 26 cm plus margins). */
#define LINE_BC_HOLD_MAX       0.35
/* m driven while lost before the hold gives way to the search. */
#define LINE_BC_HOLD_LOST      0.05
/* rad (~8.6 deg) of heading change during the pass: a bend, steer normally.
 * Straight-line wiggle over the codes stayed under 0.10 rad in every logged
 * pass. */
#define LINE_BC_MAX_TURN       0.15
/* deg/s of turn per rad of heading error. */
#define LINE_BC_HOLD_GAIN      100.0
#define LINE_BC_HOLD_RATE      20.0 /* deg/s turn limit while holding. */
/* m/s per wheel while a pass is read: a gentler wiggle, because the barcode
 * sensor sits ~6 cm to the side (every turn speeds it up or slows it down
 * against the centre distance the decoder uses) and ~18 cm ahead (the chassis
 * rocking moves it up and down). Same window as the hold. */
#define LINE_BC_CORRECTION     (LINE_CORRECTION * 0.4)
/* Obstacle status values[3] while subsystem 5 drives round an obstacle:
 * S5_STATE_AVOIDING in subsystem5.h (not included: it needs the kernel headers
 * the host tests lack). */
#define LINE_OBSTACLE_AVOIDING 5.0

#ifdef ROBO_LINE_LOSS_TEST
/* Line-loss test image only (see .claude/notes/line-pivot-sweep.md): after a
 * loss, stop, spin on the spot towards the last side seen (up to a full turn),
 * then run the normal sweep. board_config.c keeps the 25% cap unless
 * -DROBO_LINE_LOSS_CAP says otherwise. With -DROBO_LINE_VEER_TEST=1 the image
 * also drives off the line once, LINE_TEST_FOLLOW_TIME after the line was first
 * seen. */
#define LINE_STOP_TIME      0.1 /* s at zero speed before spinning. */
/* deg/s on the spot (about 0.06 m/s per wheel); lowered from 90 for the 25%
 * cap. */
#define LINE_SPIN_RATE      60.0
#define LINE_SPIN_ANGLE     (360.0 * DEG_TO_RAD)
/* s of tape in view before a search counts as finished. */
#define LINE_REACQUIRE_TIME 0.3
/* Also -DROBO_LINE_VEER_TEST=1: drive off the line once to exercise the search.
 */
#ifdef ROBO_LINE_VEER_TEST
#define LINE_TEST_FOLLOW_TIME 5.0  /* s on the line before the test veer. */
#define LINE_TEST_VEER_TIME   1.0  /* s of forced left arc. */
#define LINE_TEST_VEER_RATE   60.0 /* deg/s left while veering. */
#endif
#endif

/* Numeric order is printed as sweep=%u in the diagnostics. STOP and SPIN come
 * last so the older values keep their numbers; they are only used in the
 * line-loss test image. */
typedef enum
{
    SEARCH_IDLE,
    SEARCH_FIRST_SIDE,
    SEARCH_OTHER_SIDE,
    SEARCH_TURN_BACK,
    SEARCH_REVERSE,
    SEARCH_HOLD,
    SEARCH_STOP,
    SEARCH_SPIN,
    SEARCH_PHASE_COUNT
} line_search_phase_t;

typedef struct
{
    line_search_phase_t phase;
    float64_t           dir; /* +1 pivot left, -1 pivot right. */
    /* Already reversed once during this loss. */
    bool backed;
    /* Heading while the line was last seen. */
    float64_t seen_heading;
    /* Centre travel (m) while the line was last seen. */
    float64_t seen_centre;
    /* Centre travel when the search started. */
    float64_t lost_centre;
    /* Centre travel when reversing started. */
    float64_t backup_from;
    /* Time since loss when the stop phase began. */
    float64_t stop_from;
    /* Heading when the on-the-spot spin began. */
    float64_t spin_from;
} line_search_t;

/* ---- Line task state (see the list at the top of this section) ---- */
typedef enum
{
    LS_STARTUP,
    LS_WAITING,
    LS_FOLLOWING,
    LS_BARCODE,
    LS_GRACE,
    LS_SEARCHING,
    LS_LOST,
    LS_HELD,
    LS_MANEUVER,
    LS_STOPPED,
    LS_CAL_WAIT,
    LS_COUNT
} line_state_t;

static void search_toward(line_search_t * p_search, line_direction_t last_side);
static motion_result_t line_hold(void);
static bool            line_motion_ok(const robot_status_t * p_status);
static motion_result_t line_pivot(float64_t dir);
static motion_result_t line_search_step(line_search_t *         p_search,
                                        const motion_status_t * p_motion,
                                        line_direction_t        last_side,
                                        float64_t               lost_for);
static motion_result_t line_follow(line_direction_t drive,
                                   float64_t        correction);
static void            line_publish(line_direction_t direction, float64_t now);
static const char *    line_state_name(line_state_t state);
static const char *    line_search_name(line_search_phase_t phase);
static bool            line_state_coarse(line_state_t state);
static void line_set_state(line_state_t * p_state, line_state_t next);
static void line_report(unsigned centre, unsigned right,
                        line_direction_t direction, line_state_t lstate,
                        line_search_phase_t phase, motion_result_t result,
                        const motion_status_t * p_motion);

static void
search_toward (line_search_t * p_search, line_direction_t last_side)
{
    p_search->phase = SEARCH_FIRST_SIDE;
    /* Left if the side is unknown. */
    p_search->dir = (LINE_RIGHT == last_side) ? -1.0 : 1.0;
}

/* Zero-speed refresh on the line session: keeps ownership without a latched
 * hold, and never lets the tracking command expire (an expiry is a latched
 * F_EXPIRED fault in production). */
static motion_result_t
line_hold (void)
{
    return motion_steer(SOURCE_LINE, LINE_SESSION, 0.0, 0.0);
}

/* Motion Control neither held nor faulted. */
static bool
line_motion_ok (const robot_status_t * p_status)
{
    return (!p_status->motion.hold) && (F_NONE == p_status->motion.fault);
}

/* Inner wheel stopped, outer wheel at LINE_SEARCH_SPEED. */
static motion_result_t
line_pivot (float64_t dir)
{
    const float64_t half = LINE_SEARCH_SPEED / 2.0;

    return (dir < 0.0)
               ? motion_steer_right(SOURCE_LINE, LINE_SESSION, half, half)
               : motion_steer_left(SOURCE_LINE, LINE_SESSION, half, half);
}

/* Pivot towards the last side seen up to the sweep angle, then the other way
 * past the original heading. If both miss, turn back to the heading on the
 * line, reverse past the overshoot and sweep once more, then hold still.
 * Zero-speed refreshes keep ownership without a latched hold, so following
 * resumes when the line is seen again. Stopping the refreshes instead would
 * raise F_EXPIRED. */
static motion_result_t
line_search_step (line_search_t * p_search, const motion_status_t * p_motion,
                  line_direction_t last_side, float64_t lost_for)
{
    float64_t turned = p_motion->heading - p_search->seen_heading;

    if (SEARCH_IDLE == p_search->phase)
    {
        search_toward(p_search, last_side);
        p_search->lost_centre = p_motion->centre;
#ifdef ROBO_LINE_LOSS_TEST
        p_search->phase     = SEARCH_STOP;
        p_search->stop_from = lost_for;
#endif
    }
#ifdef ROBO_LINE_LOSS_TEST
    if ((SEARCH_STOP == p_search->phase)
        && ((lost_for - p_search->stop_from) >= LINE_STOP_TIME))
    {
        p_search->phase     = SEARCH_SPIN;
        p_search->spin_from = p_motion->heading;
    }

    if ((SEARCH_SPIN == p_search->phase)
        && ((p_search->dir * (p_motion->heading - p_search->spin_from))
            >= LINE_SPIN_ANGLE))
    {
        /* Full turn without the line: normal sweep. */
        search_toward(p_search, last_side);
    }
#endif
    if ((p_search->phase <= SEARCH_OTHER_SIDE)
        && ((p_search->dir * turned) >= LINE_SWEEP_ANGLE))
    {
        if (SEARCH_FIRST_SIDE == p_search->phase)
        {
            p_search->phase = SEARCH_OTHER_SIDE;
            p_search->dir   = -p_search->dir;
        }
        else if (!p_search->backed)
        {
            p_search->phase = SEARCH_TURN_BACK;
            p_search->dir   = -p_search->dir;
        }
        else
        {
            p_search->phase = SEARCH_HOLD;
        }
    }

    if ((SEARCH_TURN_BACK == p_search->phase)
        && ((p_search->dir * turned) >= 0.0))
    {
        p_search->phase       = SEARCH_REVERSE;
        p_search->backup_from = p_motion->centre;
    }

    if ((SEARCH_REVERSE == p_search->phase)
        && (p_motion->centre
            <= (p_search->backup_from
                - (p_search->lost_centre - p_search->seen_centre
                   + LINE_BACKUP_EXTRA))))
    {
        search_toward(p_search, last_side);
        p_search->backed = true;
    }

    if ((SEARCH_HOLD == p_search->phase) || (lost_for >= LINE_SEARCH_TIMEOUT))
    {
        return line_hold();
    }

    if (SEARCH_REVERSE == p_search->phase)
    {
        return motion_steer(SOURCE_LINE, LINE_SESSION, -LINE_BACKUP_SPEED, 0.0);
    }
#ifdef ROBO_LINE_LOSS_TEST
    if (SEARCH_STOP == p_search->phase)
    {
        return line_hold();
    }

    if (SEARCH_SPIN == p_search->phase)
    {
        return motion_steer(SOURCE_LINE, LINE_SESSION, 0.0,
                            p_search->dir * LINE_SPIN_RATE);
    }
#endif
    return line_pivot(p_search->dir);
}

static motion_result_t
line_follow (line_direction_t drive, float64_t correction)
{
    if (LINE_LEFT == drive)
    {
        return motion_steer_left(SOURCE_LINE, LINE_SESSION, LINE_SPEED,
                                 correction);
    }

    if (LINE_RIGHT == drive)
    {
        return motion_steer_right(SOURCE_LINE, LINE_SESSION, LINE_SPEED,
                                  correction);
    }

    return motion_steer(SOURCE_LINE, LINE_SESSION, LINE_SPEED, 0.0);
}

/* Line error/confidence slots, plus both-sensors-black for line_rejoin_seen()
 * (values[3] stays 0; route_flag() sets it while a route pivots). Junctions
 * come from the barcode reader. */
static void
line_publish (line_direction_t direction, float64_t now)
{
    float64_t error     = 0.0;
    bool      b_on_line = (LINE_LOST != direction) && (LINE_WAIT != direction);

    if (LINE_LEFT == direction)
    {
        error = -1.0;
    }
    else if (LINE_RIGHT == direction)
    {
        error = 1.0;
    }
    else
    {
        /* Nothing to do in any other case. */
    }

    subsystem_status_t line_status
        = {.available = true,
           .timestamp = now,
           .values    = {error, b_on_line ? 1.0 : 0.0, 0.0, 0.0}};

    line_status.values[LINE_VALUE_BOTH_BLACK]
        = (LINE_STRAIGHT == direction) ? 1.0 : 0.0;
    (void)robot_update_subsystem(SOURCE_LINE, &line_status);
}

static const char *
line_state_name (line_state_t state)
{
    static const char * const names[]
        = {"STARTUP", "WAITING", "FOLLOWING", "BARCODE", "GRACE",   "SEARCHING",
           "LOST",    "HELD",    "MANEUVER",  "STOPPED", "CAL_WAIT"};

    _Static_assert(LS_COUNT == (sizeof names / sizeof names[0]),
                   "one name per line state");
    return names[state];
}

static const char *
line_search_name (line_search_phase_t phase)
{
    static const char * const names[]
        = {"IDLE",    "FIRST_SIDE", "OTHER_SIDE", "TURN_BACK",
           "REVERSE", "HOLD",       "STOP",       "SPIN"};

    _Static_assert(SEARCH_PHASE_COUNT == (sizeof names / sizeof names[0]),
                   "one name per search phase");
    return names[phase];
}

/* FOLLOWING, BARCODE and GRACE alternate several times a second: only the
 * line_pid report shows them. */
static bool
line_state_coarse (line_state_t state)
{
    return (LS_FOLLOWING != state) && (LS_BARCODE != state)
           && (LS_GRACE != state);
}

static void
line_set_state (line_state_t * p_state, line_state_t next)
{
    if ((next != *p_state)
        && ((line_state_coarse(*p_state)) || (line_state_coarse(next))))
    {
        tm_printf((const UB *)"[S3-LINE] %s -> %s\n",
                  (const UB *)line_state_name(*p_state),
                  (const UB *)line_state_name(next));
    }

    *p_state = next;
}

static void
line_report (unsigned centre, unsigned right, line_direction_t direction,
             line_state_t lstate, line_search_phase_t phase,
             motion_result_t result, const motion_status_t * p_motion)
{
    tm_printf(
        (const UB *)"line_pid centre_ir=%u right_ir=%u direction=%u lstate=%s "
                    "sweep=%s request=%u state=%u fault=%u speed_mm_s=%d,%d "
                    "output_permille=%d,%d target_mm_s=%d,%d\n",
        centre, right, (unsigned)direction, (const UB *)line_state_name(lstate),
        (const UB *)line_search_name(phase), (unsigned)result,
        (unsigned)p_motion->state, (unsigned)p_motion->fault,
        (int)(p_motion->speed[0] * 1000.0), (int)(p_motion->speed[1] * 1000.0),
        (int)(p_motion->output[0] * 1000.0),
        (int)(p_motion->output[1] * 1000.0),
        (int)(p_motion->target[0] * 1000.0),
        (int)(p_motion->target[1] * 1000.0));

    if (F_FEEDBACK == p_motion->fault)
    {
        /* Encoder fault details recorded by Motion Control (see motion.h
         * enums). */
        const encoder_sample_t * p_sample = &p_motion->feedback_fault_sample;

        tm_printf((const UB *)"line_fb cause=%u reliable=%u counts=%lu,%lu "
                              "invalid=%lu,%lu previous_invalid=%lu,%lu\n",
                  (unsigned)p_motion->feedback_cause,
                  (unsigned)p_sample->reliable, (uint32_t)p_sample->count[0],
                  (uint32_t)p_sample->count[1], (uint32_t)p_sample->invalid[0],
                  (uint32_t)p_sample->invalid[1],
                  (uint32_t)p_motion->feedback_previous_invalid[0],
                  (uint32_t)p_motion->feedback_previous_invalid[1]);
    }
}

#if ROBO_BARCODE_TASK
/* ---- Route actions (route.h): after a valid read (SOURCE_BARCODE values[1]
 * counts them, values[0] is the letter) or at the junction that follows it
 * (values[2], the first black after a valid read, barcode.c). Done here because
 * this task owns the motion session: the Vehicle Controller rejects any other
 * command until the session has ended. Restored from commit 9291881 (Z junction
 * pivot) and generalised. ---- */
/* The car drives this far past the junction trigger (the barcode sensor seeing
 * the black), coasting included, before turning. Line IRs are 15 cm ahead of
 * the axle and the barcode sensor 3 cm ahead of them. 0 = turn where it was
 * seen. */
#define ROUTE_ADVANCE_M       0.15
/* Junction turn: black only counts once the pivot has turned this far ... */
#define ROUTE_PIVOT_MIN       (45.0 * DEG_TO_RAD)
/* ... and the old line comes back near 220 deg: give up here. */
#define ROUTE_PIVOT_MAX       (150.0 * DEG_TO_RAD)
/* U-turn: the same line, found again behind the car ... */
#define ROUTE_UTURN_MIN       (150.0 * DEG_TO_RAD)
#define ROUTE_UTURN_MAX       (240.0 * DEG_TO_RAD) /* ... give up past this. */
/* Path not read after a U-turn: the car drives back over the code (up to 28
 * cm). The on-the-spot spin adds ~0 path (path is |centre| travel; it was 0.7
 * with the old one-wheel pivot's ~0.18 m). line_route_event() adds the way from
 * the read to the turn, twice. */
#define ROUTE_UTURN_IGNORE_M  0.52
/* m/s per wheel, opposite directions: route turns spin on the spot (~80 deg/s).
 * Too slow and a wheel can stall (latched F_FEEDBACK); the line search keeps
 * its one-wheel pivot. */
#define ROUTE_SPIN_SPEED      0.08
/* s backstop for the pivot (a stalled wheel). */
#define ROUTE_PIVOT_TIMEOUT   10.0
#define ROUTE_STILL_SPEED     0.005 /* m/s per wheel counted as stopped. */
/* s of zero-speed refreshes before giving up. */
#define ROUTE_BRAKE_TIMEOUT   1.5
#define ROUTE_MOVE_TIMEOUT    8.0 /* s for the advance. */
/* m: the pivot pauses while something is this close ahead (subsystem 5 does not
 * stop the car at 150 mm while the route pivots, see route_flag()). */
#define ROUTE_PIVOT_MIN_RANGE 0.10
/* GP21, active low: calibrate, and resume after it. */
#define LINE_CAL_BUTTON       ((unsigned)BOARD_CAL_BUTTON)

static float64_t route_abs(float64_t value);
static bool      route_brake(void);
static bool      route_advance(float64_t metres);
static void      route_flag(bool b_pivoting, bool b_black);
static bool route_pivot(float64_t dir, float64_t min_turn, float64_t max_turn);

static float64_t
route_abs (float64_t value)
{
    return (value < 0.0) ? -value : value;
}

/* Zero-speed refreshes keep the session (no expiry) until both wheels are
 * nearly still. */
static bool
route_brake (void)
{
    float64_t start = motion_hal_time();
    /* Static: the line task stack is nearly full. */
    static robot_status_t status;

    while ((motion_hal_time() - start) < ROUTE_BRAKE_TIMEOUT)
    {
        (void)line_hold();
        (void)tk_dly_tsk(20U);

        if ((!motion_get_status(&status)) || (!line_motion_ok(&status)))
        {
            return false;
        }

        if ((route_abs(status.motion.speed[0]) < ROUTE_STILL_SPEED)
            && (route_abs(status.motion.speed[1]) < ROUTE_STILL_SPEED))
        {
            return true;
        }
    }

    return false;
}

/* Drives straight on the line session (refreshed every 20 ms, so it never
 * expires) until the path has moved 'metres', then brakes. The session is never
 * given up: an expired tracking command is a latched F_EXPIRED fault in
 * production (motion.c), and only COMPLETE/CANCELLED/FAULT drop the owner. */
static bool
route_advance (float64_t metres)
{
    float64_t start = motion_hal_time();
    float64_t from;
    /* Static: the line task stack is nearly full. */
    static robot_status_t status;

    if (!motion_get_status(&status))
    {
        return false;
    }

    from = status.motion.path;

    while ((motion_hal_time() - start) < ROUTE_MOVE_TIMEOUT)
    {
        (void)motion_steer(SOURCE_LINE, LINE_SESSION, LINE_SPEED, 0.0);
        (void)tk_dly_tsk(20U);

        if ((!motion_get_status(&status)) || (!line_motion_ok(&status)))
        {
            return false;
        }

        if ((status.motion.path - from) >= metres)
        {
            return route_brake();
        }
    }

    return false;
}

/* Line status with values[3] = 1 while the route pivots: subsystem 5 then skips
 * its 150 mm stop. line_publish() writes 0 on every pass of the line loop, so
 * the flag clears itself once following resumes. */
static void
route_flag (bool b_pivoting, bool b_black)
{
    subsystem_status_t line_status
        = {.available = true,
           .timestamp = motion_hal_time(),
           .values = {0.0, b_black ? 1.0 : 0.0, 0.0, b_pivoting ? 1.0 : 0.0}};

    (void)robot_update_subsystem(SOURCE_LINE, &line_status);
}

/* Spins on the spot (dir +1 left, -1 right; both wheels driven, opposite ways,
 * session 3001) until a line IR sees black, having first left the line and
 * turned at least min_turn. Returns at once on the black, without braking, so
 * the line loop takes over the same session immediately. */
static bool
route_pivot (float64_t dir, float64_t min_turn, float64_t max_turn)
{
    static robot_status_t status;

    float64_t heading0;
    float64_t turned      = 0.0;
    float64_t start       = motion_hal_time();
    bool      b_left_line = false;
    unsigned  seen        = 0U;
    unsigned  rejected    = 0U;

    if (!motion_get_status(&status))
    {
        return false;
    }

    heading0 = status.motion.heading;

    while ((motion_hal_time() - start) < ROUTE_PIVOT_TIMEOUT)
    {
        unsigned right   = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre  = (unsigned)gpio_get(BOARD_LINE_C);
        bool     b_black = (1U == right) || (1U == centre);

        route_flag(true, b_black);

        if ((!motion_get_status(&status)) || (!line_motion_ok(&status)))
        {
            break;
        }

        turned = dir * (status.motion.heading - heading0);

        if (!b_black)
        {
            b_left_line = true;
        }

        seen
            = (b_left_line && b_black && (turned >= min_turn)) ? seen + 1U : 0U;

        if (seen >= 2U)
        /* Two samples in a row, as the follower's own debounce: one glitch is
         * not the line. */
        {
            tm_printf((const UB *)"route: line found after pivoting %d deg "
                                  "(centre_ir=%u right_ir=%u)\n",
                      (int)(turned / DEG_TO_RAD), centre, right);
            return true;
        }

        if (turned >= max_turn)
        {
            break;
        }
#if defined(ROBO_SUBSYSTEM5) && ROBO_SUBSYSTEM5
        if ((!status.obstacle.available)
            || (status.obstacle.values[0] < ROUTE_PIVOT_MIN_RANGE))
        {
#else
        if (false)
        { /* No obstacle sensor in this image. */
#endif
            /* Something right ahead: hold still (zero-speed refreshes) until it
             * moves; the timeout still ends it. */
            (void)line_hold();
            (void)tk_dly_tsk(20U);
            continue;
        }

        /* Not accepted for ~0.2 s (well under the 0.3 s command lifetime): give
         * up rather than wait. */
        rejected
            = (R_OK
               == ((dir < 0.0) ? motion_steer_right(SOURCE_LINE, LINE_SESSION,
                                                    0.0, ROUTE_SPIN_SPEED)
                               : motion_steer_left(SOURCE_LINE, LINE_SESSION,
                                                   0.0, ROUTE_SPIN_SPEED)))
                  ? 0U
                  : rejected + 1U;

        if (rejected >= 10U)
        {
            break;
        }

        (void)tk_dly_tsk(20U);
    }

    /* Zero speed first, before any printing: the last pivot refresh is about to
     * expire. */
    (void)line_hold();
    route_flag(false, false);
    tm_printf(
        (const UB *)"route: no line found while pivoting (turned %d deg)\n",
        (int)(turned / DEG_TO_RAD));
    return false;
}

/* One route action. Returns true when following can resume; *side is where the
 * line will be after it. */
static bool
route_run (route_action_t action, bool b_at_junction, float64_t path_seen,
           line_direction_t * p_side)
{
    /* Static: the line task stack is nearly full. */
    static robot_status_t status;
    /* The U-turn spins left. */
    float64_t dir = (ROUTE_RIGHT == action) ? -1.0 : 1.0;

    *p_side = LINE_STRAIGHT;

    if (!route_brake())
    {
        tm_printf((const UB *)"route: FAILED while braking\n");
        return false;
    }

    if (!motion_get_status(&status))
    {
        return false;
    }

    if (b_at_junction && (ROUTE_ADVANCE_M > 0.0))
    {
        /* Distance still to drive past the trigger, minus what the car coasted
         * (never reverses). */
        float64_t advance = ROUTE_ADVANCE_M - (status.motion.path - path_seen);

        tm_printf(
            (const UB *)"route: stopped, coasted %d mm, advancing %d mm\n",
            (int)((status.motion.path - path_seen) * 1000.0),
            (int)(advance * 1000.0));

        if ((advance > 0.005) && (!route_advance(advance)))
        {
            tm_printf((const UB *)"route: FAILED during the advance\n");
            return false;
        }
    }

    if (!((ROUTE_UTURN == action)
              ? route_pivot(dir, ROUTE_UTURN_MIN, ROUTE_UTURN_MAX)
              : route_pivot(dir, ROUTE_PIVOT_MIN, ROUTE_PIVOT_MAX)))
    {
        tm_printf((const UB *)"route: FAILED during the pivot\n");
        return false;
    }

    /* An overshoot leaves the line on the other side. */
    *p_side = (dir > 0.0) ? LINE_RIGHT : LINE_LEFT;
    return true;
}
#endif

#if defined(ROBO_SUBSYSTEM4) && ROBO_SUBSYSTEM4
/* Buddy 4 captures its tilt reference while the robot is still, about 2 s after
 * boot; driving earlier makes that capture fail. Terrain slot values[3] holds
 * validity bits | T_STATE << 3 (docs/API.md): 0 INIT, 4 RECOVERY, others mean
 * the capture is over (done, or the IMU gave up). */
/* s cap: a missing or faulty IMU never stops the robot. */
#define LINE_IMU_WAIT_MAX     10.0
#define LINE_TERRAIN_INIT     0U
#define LINE_TERRAIN_RECOVERY 4U

static bool
line_imu_settled (const robot_status_t * p_status)
{
    unsigned state;

    bool b_result = false;

    if ((!p_status->terrain.available)
        || (!(p_status->terrain.values[3] >= 0.0)))
    {
        b_result = false;
    }
    else
    {
        state = ((unsigned)p_status->terrain.values[3] >> 3) & 7U;
        b_result
            = (LINE_TERRAIN_INIT != state) && (LINE_TERRAIN_RECOVERY != state);
    }

    return b_result;
}

/* Runs once before the first motion request; nothing is driving yet.
 * With the web Start gate (ROBO_REQUIRE_START) Buddy 4 captures its reference
 * only after the dashboard START, so the wait (and its cap) starts at START:
 * the robot stays still ~2 s after START while Buddy 4 captures. */
static void
line_wait_for_imu (void)
{
    float64_t      start;
    robot_status_t status;
    bool           b_settled = false;
#ifdef ROBO_REQUIRE_START
    while ((!motion_get_status(&status)) || (status.motion.hold))
    {
        (void)tk_dly_tsk(20U);
    }
#endif
    start = motion_hal_time();

    while ((!b_settled) && ((motion_hal_time() - start) < LINE_IMU_WAIT_MAX))
    {
        b_settled = (motion_get_status(&status)) && (line_imu_settled(&status));

        if (!b_settled)
        {
            (void)tk_dly_tsk(20U);
        }
    }

    tm_printf((const UB *)"line_imu_wait settled=%u waited_ms=%d\n",
              (unsigned)b_settled, (int)((motion_hal_time() - start) * 1000.0));
}
#endif

/* Everything the line loop remembers between its 20 ms ticks. Static (not on
 * the stack): the line task's stack is nearly full. */
typedef struct
{
    line_state_t    state;
    line_steering_t filter; /* Debounce of the two IR sensors. */
    line_search_t   search; /* SEARCHING sub-state. */
    /* Raw side the line was last seen on; STRAIGHT = unknown. */
    line_direction_t last_side;
    /* Last follow command, repeated during GRACE. */
    line_direction_t follow;
    float64_t        lost_since;  /* Last time the line was seen. */
    float64_t        next_report; /* Next line_pid report. */
    /* Barcode counters read once: earlier events cannot trigger anything. */
    bool baselined;
    /* A failed route action latched the car: STOPPED, not HELD. */
    bool stopped;
    /* A route action failed: hold still for good (zero-speed refreshes). */
    bool halted;

    unsigned seen_junctions;
    unsigned seen_failures;
#if ROBO_BARCODE_TASK
    unsigned seen_reads;
    /* route_action_t read before its junction (left/right/U-turn), else NONE.
     */
    uint8_t route_pending;
    /* Path (m) at that read: sizes the U-turn's barcode ignore. */
    float64_t route_read_path;
    /* Consecutive polls with GP21 pressed (debounce). */
    unsigned cal_presses;
    /* GP21 pressed: the reader is calibrating on the next full pass. */
    bool calibrating;
    /* CAL_WAIT: calibration finished, stopped until GP21 is pressed again. */
    bool cal_wait;
    /* Path (m) where the barcode pass began, see LINE_BC_*; < 0 = none. */
    float64_t bc_hold_from;
    /* Path (m) where the line was lost during that pass; < 0 = not lost. */
    float64_t bc_lost_from;
    float64_t bc_heading_from; /* Heading (rad) at the start of that pass. */
#endif
#ifdef ROBO_LINE_RUN_LIMIT_S
    /* First follow command; the car is held still ROBO_LINE_RUN_LIMIT_S later.
     */
    float64_t run_start;
    bool      run_done;
#endif
#ifdef ROBO_LINE_LOSS_TEST
    /* Start of the current stretch with the line seen. */
    float64_t on_line_since;
    /* When the current search started; glimpses do not reset it. */
    float64_t search_began;
#ifdef ROBO_LINE_VEER_TEST
    float64_t veer_until;
    bool      veered;
#endif
#endif
} line_ctx_t;

/* Drive on owning the line session (not READY, not someone else's). */
static bool
line_driving (const robot_status_t * p_status, bool b_owns)
{
    return b_owns && (DRIVING == p_status->motion.state);
}

/* May start or continue a line command: Motion Control idle, or already driving
 * for this task. */
static bool
line_can_drive (const robot_status_t * p_status, bool b_owns)
{
    return (READY == p_status->motion.state)
           || (line_driving(p_status, b_owns));
}

#if ROBO_BARCODE_TASK
/* After something else had the wheels (a route action, the calibration stop):
 * start afresh. */
static void
line_restart (line_ctx_t * p_context)
{
    p_context->filter     = (line_steering_t){0};
    p_context->search     = (line_search_t){0};
    p_context->lost_since = motion_hal_time();
#ifdef ROBO_LINE_LOSS_TEST
    p_context->on_line_since = -1.0;
    p_context->search_began  = -1.0;
#endif
}

/* GP21, active low: true once per press (two polls = 40 ms down); holding it
 * does not repeat. */
static bool
line_cal_pressed (line_ctx_t * p_context)
{
    bool b_result = false;

    if (gpio_get(LINE_CAL_BUTTON))
    {
        p_context->cal_presses = 0U;
        b_result               = false;
    }
    else
    {
        b_result = 2U == ++p_context->cal_presses;
    }

    return b_result;
}

/* GP21 calibration: press, drive over the calibration code, stop (CAL_WAIT, no
 * route action), press again to carry on. A sensor check: the reader proves it
 * can read the code, and keeps the sensitivity that read it. Returns false when
 * this tick was used up (stopped, or just resumed). */
static bool
line_calibration_step (line_ctx_t * p_context, const robot_status_t * p_status)
{
    bool          b_pressed = line_cal_pressed(p_context);
    barcode_cal_t result;

    if (p_context->cal_wait)
    {
        /* Zero-speed refreshes: no latched hold, nothing expires. */
        (void)line_hold();

        if (!b_pressed)
        {
            return false;
        }

        p_context->cal_wait = false;
        /* Reads and junctions from the wait (robot handled) are forgotten; a
         * failed read is not: it is reported next tick, as always. The reader
         * also drops a junction armed meanwhile. */
        p_context->seen_reads     = (unsigned)p_status->barcode.values[1];
        p_context->seen_junctions = (unsigned)p_status->barcode.values[2];
        /* A turn read before the wait has lost its junction. */
        p_context->route_pending = (uint8_t)ROUTE_NONE;
        barcode_ignore_for(0.0);
        line_restart(p_context);
        tm_printf((const UB *)"calibration: GP21 pressed, carrying on\n");
        return false;
    }

    if (!p_context->calibrating)
    {
        if (b_pressed && (!p_context->stopped))
        {
            p_context->calibrating = true;
            barcode_calibrate_begin();
            tm_printf(
                (const UB *)"calibration: started; follow the line over *%c*\n",
                ROUTE_CALIBRATION_LETTER);
        }

        return true;
    }

    result = barcode_calibration_result();

    if (BARCODE_CAL_NONE == result)
    {
        return true;
    }

    p_context->calibrating = false;
    tm_printf((const UB *)"calibration %s%s\n",
              (BARCODE_CAL_DONE == result)
                  ? "done"
                  : "FAILED, the sensor could not read the code",
              p_context->stopped ? "" : " - press GP21 to carry on");

    if (p_context->stopped)
    {
        return true;
    } /* A failed route action already stopped the car: STOPPED stays. */
    p_context->cal_wait = true;
    (void)route_brake();
    (void)line_hold();
    line_set_state(&p_context->state, LS_CAL_WAIT);
    return false;
}

/* A new read or junction: run its route action if it is due now (route.h).
 * Returns false when an action ran (this tick was used up). A failed action
 * holds still for good (STOPPED). */
static bool
line_route_event (line_ctx_t * p_context, const robot_status_t * p_status,
                  bool b_owns, unsigned reads, unsigned junctions)
{
    /* A new read first; its junction comes later. */
    bool             b_at_junction = reads == p_context->seen_reads;
    char             letter = (char)(unsigned)p_status->barcode.values[0];
    route_action_t   action;
    line_direction_t side = LINE_STRAIGHT;

    tm_printf((const UB *)"route: %s after *%c*, path %d mm\n",
              b_at_junction ? "junction" : "read", letter,
              (int)(p_status->motion.path * 1000.0));

    if (b_at_junction)
    {
        p_context->seen_junctions = junctions;
        /* Decided at the read: a new route cannot switch it. */
        action                   = (route_action_t)p_context->route_pending;
        p_context->route_pending = (uint8_t)ROUTE_NONE;
    }
    else
    {
        p_context->seen_reads = reads;
        action                = route_action(route_get(), letter);

        if (ROUTE_STRAIGHT == action)
        {
            tm_printf((const UB *)"route: straight on, junction ignored\n");
        }

        p_context->route_pending
            = (uint8_t)((ROUTE_STRAIGHT == action) ? ROUTE_NONE : action);
        p_context->route_read_path = p_status->motion.path;
        return true; /* Every action waits for its junction. */
    }

    if (ROUTE_NONE == action)
    {
        return true;
    }

    if ((SEARCH_IDLE != p_context->search.phase) || (!b_owns)
        || (!line_motion_ok(p_status))
        || (LINE_OBSTACLE_AVOIDING == p_status->obstacle.values[3]))
    {
        /* A spin or sweep carries the barcode sensor in an arc, possibly back
         * over the code. */
        tm_printf((const UB *)"route: not following the line right now, action "
                              "skipped\n");
        return true;
    }

    if (ROUTE_UTURN == action)
    /* Before turning: the spin sweeps the sensor, then it drives back over the
     * code. */
    {
        barcode_ignore_for(
            ROUTE_UTURN_IGNORE_M
            + (p_status->motion.path - p_context->route_read_path)
            + (2.0 * ROUTE_ADVANCE_M));
    }

    line_set_state(&p_context->state, LS_MANEUVER);

    if (!route_run(action, b_at_junction, p_status->motion.path, &side))
    {
        /* Zero speed at once; line_loop keeps refreshing it. */
        (void)line_hold();
        p_context->stopped = true;
        /* line_loop holds still from now on (until power-cycle). */
        p_context->halted = true;
        line_set_state(&p_context->state, LS_STOPPED);
        tm_printf((const UB *)"route: holding still after the failure\n");
        return false;
    }

    line_restart(p_context);
    p_context->last_side = side;
    p_context->follow    = side;
    return false;
}
#endif

/* Calibration, failed reads, reads and junctions. Returns false when this tick
 * was used up and the line must not be driven. */
static bool
line_barcode_events (line_ctx_t * p_context, const robot_status_t * p_status,
                     bool b_owns)
{
    unsigned junctions = (unsigned)p_status->barcode.values[2];
    unsigned failures  = (unsigned)p_status->barcode.values[3];
#if ROBO_BARCODE_TASK
    unsigned reads = (unsigned)p_status->barcode.values[1];

    if (!line_calibration_step(p_context, p_status))
    {
        return false;
    }
#endif
    if (!p_context->baselined)
    {
        /* Whatever was published before now cannot trigger a turn. */
        p_context->seen_junctions = junctions;
        p_context->seen_failures  = failures;
#if ROBO_BARCODE_TASK
        p_context->seen_reads = reads;
#endif
        p_context->baselined = true;
    }
    else if (failures != p_context->seen_failures)
    {
        /* The barcode task could not decode a pass: no action, keep following
         * the line. The reader arms no junction after a failed pass, and an
         * older read's turn must not fire at this code's junction. The
         * junction's own bar is never counted as a failure (barcode.c). */
        p_context->seen_failures = failures;
#if ROBO_BARCODE_TASK
        p_context->route_pending = (uint8_t)ROUTE_NONE;
#endif
        tm_printf((const UB *)"route: barcode read failed, carrying on\n");
#if ROBO_BARCODE_TASK
    }
    else if ((reads != p_context->seen_reads)
             || (junctions != p_context->seen_junctions))
    {
        return line_route_event(p_context, p_status, b_owns, reads, junctions);
#else
    }
    else if (junctions != p_context->seen_junctions)
    {
        p_context->seen_junctions = junctions;
        tm_printf((const UB *)"route: junction after %c, path %d mm (no "
                              "action, following on)\n",
                  (char)(unsigned)p_status->barcode.values[0],
                  (int)(p_status->motion.path * 1000.0));
#endif
    }
    else
    {
        /* Nothing to do in any other case. */
    }

    (void)b_owns;
    return true;
}

/* While a barcode pass is read: the steering correction to use, and *hold =
 * hold the heading through a short loss (BARCODE state). Both end at a bend
 * (LINE_BC_MAX_TURN) or after LINE_BC_HOLD_MAX. */
static float64_t
line_barcode_window (line_ctx_t * p_context, const robot_status_t * p_status,
                     line_direction_t direction, bool * p_window, bool * p_hold)
{
#if ROBO_BARCODE_TASK
    if (barcode_code_pass())
    {
        if (p_context->bc_hold_from < 0.0)
        {
            p_context->bc_hold_from    = p_status->motion.path;
            p_context->bc_heading_from = p_status->motion.heading;
        }
    }
    else
    {
        /* Pass over, or tape / big code seen: steer normally at once. */
        p_context->bc_hold_from = -1.0;
    }

    *p_window = (p_context->bc_hold_from >= 0.0)
                && ((p_status->motion.path - p_context->bc_hold_from)
                    < LINE_BC_HOLD_MAX)
                && (SEARCH_IDLE == p_context->search.phase)
                && ((p_status->motion.heading - p_context->bc_heading_from)
                    < LINE_BC_MAX_TURN)
                && ((p_context->bc_heading_from - p_status->motion.heading)
                    < LINE_BC_MAX_TURN);

    if (LINE_LOST != direction)
    {
        p_context->bc_lost_from = -1.0;
    }
    else if ((*p_window) && (p_context->bc_lost_from < 0.0))
    {
        p_context->bc_lost_from = p_status->motion.path;
    }
    else
    {
        /* Nothing to do in any other case. */
    }

    *p_hold = (*p_window) && (LINE_LOST == direction)
              && ((p_status->motion.path - p_context->bc_lost_from)
                  < LINE_BC_HOLD_LOST);
    return *p_window ? LINE_BC_CORRECTION : LINE_CORRECTION;
#else
    (void)p_context;
    (void)p_status;
    (void)direction;
    *p_window = false;
    *p_hold   = false;
    return LINE_CORRECTION;
#endif
}

/* One tick of line following: picks the command and the state. */
static void
line_drive (line_ctx_t * p_context, const robot_status_t * p_status,
            bool b_owns, line_direction_t direction, unsigned centre,
            unsigned right, float64_t now)
{
    /* Subsystem 5 drives round obstacles itself; keep off the wheels until it
     * publishes that it is back on the line. Exact compare: the value is
     * copied. */
    bool b_avoiding = LINE_OBSTACLE_AVOIDING == p_status->obstacle.values[3];
    /* STOPPED never drives again. */
    bool b_usable
        = (line_motion_ok(p_status)) && (!b_avoiding) && (!p_context->stopped);
    motion_result_t result = R_HELD;
    line_state_t    next
        = b_usable ? LS_WAITING : (p_context->stopped ? LS_STOPPED : LS_HELD);
    line_direction_t drive      = direction;
    bool             b_grace_ok = true;
    bool             b_window;
    bool             b_bc_hold;
    float64_t correction = line_barcode_window(p_context, p_status, direction,
                                               &b_window, &b_bc_hold);
#ifdef ROBO_LINE_LOSS_TEST
    /* A glimpse of tape during a search is not "found": keep the 360 deg budget
     * and the search clock, and stand still until the tape has stayed in view
     * for LINE_REACQUIRE_TIME. No blind grace drive while searching. */
    bool b_settled
        = (p_context->on_line_since >= 0.0)
          && ((now - p_context->on_line_since) >= LINE_REACQUIRE_TIME);
    bool b_glimpse = (LINE_LOST != direction) && (LINE_WAIT != direction)
                     && (SEARCH_IDLE != p_context->search.phase)
                     && (!b_settled);

    b_grace_ok = SEARCH_IDLE == p_context->search.phase;
#else
    const bool b_glimpse = false;
#endif
    if ((LINE_LOST != direction) && (!b_glimpse))
    {
        p_context->search.seen_heading = p_status->motion.heading;
        p_context->search.seen_centre  = p_status->motion.centre;
        p_context->search.phase        = SEARCH_IDLE;
        p_context->search.backed       = false;

        if (b_usable && (LINE_WAIT != direction))
        {
            next = b_window ? LS_BARCODE : LS_FOLLOWING;
        }
    }
    else if ((LINE_LOST == direction) && b_grace_ok && (!b_bc_hold) && b_owns
             && ((now - p_context->lost_since) < LINE_GRACE_TIME))
    {
        drive = p_context->follow; /* Bridge short dropouts. */

        if (b_usable)
        {
            next = LS_GRACE;
        }
    }
    else
    {
        /* Nothing to do in any other case. */
    }
#ifdef ROBO_LINE_RUN_LIMIT_S
    /* Analysis builds: one run of ROBO_LINE_RUN_LIMIT_S seconds, then stand
     * still (reset to rerun). */
    if ((!p_context->run_done) && (p_context->run_start >= 0.0)
        && ((now - p_context->run_start) >= (float64_t)ROBO_LINE_RUN_LIMIT_S))
    {
        p_context->run_done = true;
        tm_printf((const UB *)"line_test: run limit reached, car stopped\n");
    }

    if (p_context->run_done)
    {
        if (b_usable && (line_can_drive(p_status, b_owns)))
        {
            result = line_hold();
        }

        next = LS_HELD;
    }
    else
#endif
#ifdef ROBO_LINE_LOSS_TEST
#ifdef ROBO_LINE_VEER_TEST
        if ((now < p_context->veer_until) && b_usable
            && (line_can_drive(p_status, b_owns)))
    {
        result = motion_steer(SOURCE_LINE, LINE_SESSION, LINE_SPEED,
                              LINE_TEST_VEER_RATE);
    }
    else
#endif
        if (b_glimpse && b_usable && (line_driving(p_status, b_owns)))
    {
        result = line_hold();
        next   = LS_SEARCHING;
    }
    else
#endif
        if (p_context->stopped)
    {
        /* Failed read: even if something releases the latched hold, keep
         * refreshing zero speed. */
        if ((line_motion_ok(p_status)) && (!b_avoiding)
            && (line_can_drive(p_status, b_owns)))
        {
            result = line_hold();
        }
    }
    else if (b_bc_hold && (LINE_LOST == drive) && b_usable
             && (line_driving(p_status, b_owns)))
    {
        float64_t turn
            = LINE_BC_HOLD_GAIN
              * (p_context->search.seen_heading - p_status->motion.heading);

        if (turn > LINE_BC_HOLD_RATE)
        {
            turn = LINE_BC_HOLD_RATE;
        }

        if (turn < -LINE_BC_HOLD_RATE)
        {
            turn = -LINE_BC_HOLD_RATE;
        }

        result = motion_steer(SOURCE_LINE, LINE_SESSION, LINE_SPEED, turn);
        /* Grace and search start afresh if the line is still gone after the
         * hold. */
        p_context->lost_since = now;
        next                  = LS_BARCODE;
    }
    else if ((LINE_LOST == drive) && b_usable
             && (line_driving(p_status, b_owns)))
    {
        float64_t lost_for = now - p_context->lost_since;
#ifdef ROBO_LINE_LOSS_TEST
        if (SEARCH_IDLE == p_context->search.phase)
        {
            p_context->search_began = p_context->lost_since;
        }

        /* Glimpses must not restart the clock. */
        lost_for = now - p_context->search_began;
#endif
        result = line_search_step(&p_context->search, &p_status->motion,
                                  p_context->last_side, lost_for);
        next   = ((SEARCH_HOLD == p_context->search.phase)
                  || (lost_for >= LINE_SEARCH_TIMEOUT))
                     ? LS_LOST
                     : LS_SEARCHING;
    }
    else if ((LINE_WAIT != drive) && (LINE_LOST != drive) && b_usable
             && (line_can_drive(p_status, b_owns)))
    {
        p_context->follow = drive;
        result            = line_follow(drive, correction);
#ifdef ROBO_LINE_RUN_LIMIT_S
        if (p_context->run_start < 0.0)
        {
            p_context->run_start = now;
        }
#endif
    }
    else
    {
        /* Nothing to do in any other case. */
    }

    line_set_state(&p_context->state, next);

    if (now >= p_context->next_report)
    {
        p_context->next_report = now + 0.5;
        line_report(centre, right, direction, p_context->state,
                    p_context->search.phase, result, &p_status->motion);
    }
}

static void
line_loop (void)
{
    static line_ctx_t ctx;
    line_ctx_t *      p_context = &ctx;

    p_context->state     = LS_STARTUP;
    p_context->last_side = LINE_STRAIGHT;
    p_context->follow    = LINE_STRAIGHT;
#if ROBO_BARCODE_TASK
    p_context->bc_hold_from = -1.0;
    p_context->bc_lost_from = -1.0;
#endif
#ifdef ROBO_LINE_RUN_LIMIT_S
    p_context->run_start = -1.0;
#endif
#ifdef ROBO_LINE_LOSS_TEST
    p_context->on_line_since = -1.0;
    p_context->search_began  = -1.0;
#ifdef ROBO_LINE_VEER_TEST
    p_context->veer_until = -1.0;
#endif
#endif
#if defined(ROBO_SUBSYSTEM4) && ROBO_SUBSYSTEM4
    line_wait_for_imu();
#endif
#if ROBO_BARCODE_TASK
    gpio_init(LINE_CAL_BUTTON);
    gpio_set_dir(LINE_CAL_BUTTON, false);
    gpio_pull_up(LINE_CAL_BUTTON);
    tm_printf((const UB *)"calibration: press GP21 any time to check the "
                          "barcode sensor on *%c*\n",
              ROUTE_CALIBRATION_LETTER);
#endif
    line_set_state(&p_context->state, LS_WAITING);

    for (;;)
    {
#if ROBO_BARCODE_TASK
        if (p_context->halted)
        /* Terminal STOPPED after a failed route action: zero-speed refreshes,
         * no latched hold. */
        {
            /* Every pass: a lost clear must not keep subsystem 5 off. */
            route_flag(false, false);
            (void)line_hold();
            (void)tk_dly_tsk(20U);
            continue;
        }
#endif
        unsigned right  = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre = (unsigned)gpio_get(BOARD_LINE_C);
        /* Centre only black: robot drifted right, line is to the left ->
         * LINE_LEFT. Right only black: robot drifted left, line is to the right
         * -> LINE_RIGHT. */
        line_direction_t direction = motion_line_direction(
            &p_context->filter, 1U == centre, 1U == right);
        robot_status_t status;
        float64_t      now = motion_hal_time();

        /* Raw, not debounced: a single centre-only sample before the loss still
         * counts. */
        if (centre != right)
        {
            p_context->last_side = (1U == centre) ? LINE_LEFT : LINE_RIGHT;
        }
        else if (1U == centre)
        {
            p_context->last_side = LINE_STRAIGHT;
        }
        else
        {
            /* Nothing to do in any other case. */
        }

        if (LINE_LOST != direction)
        {
            p_context->lost_since = now;
        }
#ifdef ROBO_LINE_LOSS_TEST
        if ((LINE_LOST == direction) || (LINE_WAIT == direction))
        {
            p_context->on_line_since = -1.0;
        }
        else if (p_context->on_line_since < 0.0)
        {
            p_context->on_line_since = now;
        }
#ifdef ROBO_LINE_VEER_TEST
        if ((!p_context->veered) && (p_context->on_line_since >= 0.0)
            && ((now - p_context->on_line_since) >= LINE_TEST_FOLLOW_TIME))
        {
            p_context->veered     = true;
            p_context->veer_until = now + LINE_TEST_VEER_TIME;
            tm_printf((const UB *)"line_test: veering off the line now\n");
        }
#endif
#endif
        line_publish(direction, now);

        if (motion_get_status(&status))
        {
            bool b_owns = (status.owner_valid) && (SOURCE_LINE == status.owner)
                          && (LINE_SESSION == status.motion.id);

            if (line_barcode_events(p_context, &status, b_owns))
            {
                line_drive(p_context, &status, b_owns, direction, centre, right,
                           now);
            }
        }

        (void)tk_dly_tsk(20U);
    }
}
#endif

void
line_task (INT code, void * p_context)
{
    (void)code;
    (void)p_context;
    ir_input_init(BOARD_LINE_R);
    ir_input_init(BOARD_LINE_C);
#ifdef ROBO_LINE_LOSS_TEST
    tm_printf((const UB *)"line_test: on-the-spot pivot search enabled\n");
#ifdef ROBO_LINE_VEER_TEST
    tm_printf(
        (const UB *)"line_test: veers off the line after %u s on the line\n",
        (unsigned)LINE_TEST_FOLLOW_TIME);
#endif
#endif
    (void)tk_dly_tsk(2000U);
    line_loop();
}
#endif

/*** end of file ***/
