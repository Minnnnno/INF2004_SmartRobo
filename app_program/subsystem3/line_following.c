/* Only images with a working line_loop() build and create line_task
 * (ROBO_LINE_TASK in subsystems.h; host tests define the same macros). */
#if defined(ROBO_SAFE_MOTOR_TEST) || defined(ROBO_LINE_STEERING)
#ifdef ROBO_LINE_TEST_HOST
#include "line_test_fake_hal.h"
#else
#include "subsystems.h"
#include "../subsystem2/template_hal.h"
#include <tm/tmonitor.h>
#endif
#include "board_config.h"
#include "ir_io.h"
#ifndef ROBO_BARCODE_TASK
#define ROBO_BARCODE_TASK 0 /* Host tests do not include subsystems.h: no barcode reader. */
#endif

/* Each firmware profile supplies its own line_loop(); CMake keeps them exclusive. */
#if defined(ROBO_SAFE_MOTOR_TEST)
#include "motor_test.h"
static volatile bool sensor_paused;
void line_forward_pause(void) { sensor_paused = true; }

/* Either sensor HIGH drives forward; both LOW stops. A console pause lasts until both read LOW. */
static void line_loop(void)
{
    bool sensor_running = false;
    for (;;) {
        unsigned right = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre = (unsigned)gpio_get(BOARD_LINE_C);
        bool move = (right == 1U) || (centre == 1U);
        if (!motor_test_active()) { sensor_running = false; }
        if (!move) { sensor_paused = false; }
        if (move && !sensor_paused) {
            sensor_running = motor_test_sensor_forward();
        } else if (sensor_running) {
            motor_test_stop();
            sensor_running = false;
        }
        /* Console labels predate the right/centre naming used by line steering. */
        tm_printf((const UB *)"left_digital=%u right_digital=%u move_condition=%u sensor_forward_active=%u\n",
                  right, centre, (unsigned)move, (unsigned)sensor_running);
        (void)tk_dly_tsk(100U);
    }
}

#elif defined(ROBO_IR_MONITOR)
/* Sensor-only image: no motion requests; '\r' rewrites one console line.
 * Also shows the barcode IR (GP28) so its wiring can be checked; barcode_task stays idle. */
#if defined(ROBO_IR_MONITOR_ANALOG) && ROBO_IR_MONITOR_ANALOG
/* Analog variant (-DCMAKE_C_FLAGS=-DROBO_IR_MONITOR_ANALOG=1): also reads the ADC on every IR pin, to see
 * which module output (DO or AO) each pin is wired to and what the potentiometer changes. The line pins stay
 * digital inputs (the ADC still reads them); the barcode pin is set up for analog only, as barcode.c does.
 * ADC channel = pin - 26: GP26 = ADC0, GP27 = ADC1, GP28 = ADC2 (ir_io.h). */
#define IRMON_ADC_AVERAGE 16U

static void line_loop(void)
{
    if (!ir_adc_init(BOARD_BARCODE)) { tm_printf((const UB *)"ir_monitor_analog: ADC not responding, a= reads 0\n"); }
    tm_printf((const UB *)"ir_monitor_analog: ADC 0..4095 (3.3 V); d = digital read of the same pin\n");
    for (;;) {
        unsigned right = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre = (unsigned)gpio_get(BOARD_LINE_C);
        tm_printf((const UB *)"centre GP%u d=%u a=%4u | right GP%u d=%u a=%4u | barcode GP%u a=%4u\n",
                  (unsigned)BOARD_LINE_C, centre, ir_adc_read(BOARD_LINE_C, IRMON_ADC_AVERAGE),
                  (unsigned)BOARD_LINE_R, right, ir_adc_read(BOARD_LINE_R, IRMON_ADC_AVERAGE),
                  (unsigned)BOARD_BARCODE, ir_adc_read(BOARD_BARCODE, IRMON_ADC_AVERAGE));
        (void)tk_dly_tsk(200U);
    }
}
#else
static void line_loop(void)
{
    ir_input_init(BOARD_BARCODE);
    for (;;) {
        unsigned right = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre = (unsigned)gpio_get(BOARD_LINE_C);
        unsigned barcode = (unsigned)gpio_get(BOARD_BARCODE);
        tm_printf((const UB *)"\rcentre_ir=%u right_ir=%u barcode_ir=%u", centre, right, barcode);
        (void)tk_dly_tsk(50U);
    }
}
#endif

#elif defined(ROBO_LINE_STEERING)
/* Line task states (line_state_t; printed as lstate=<name> every 0.5 s, and the coarse ones also
 * as "[S3-LINE] A -> B" when they change):
 *   STARTUP    line_task: pin setup, settle delay, wait for the IMU's level reference.
 *   WAITING    not driving the line yet: no stable reading (debounce), or stopped and the line not seen.
 *   FOLLOWING  line_follow(): straight, or correct towards the side the line is on.
 *   BARCODE    following while a barcode pass is read: gentler steering, and a short loss holds the heading.
 *   GRACE      a short loss: the last follow command is repeated for LINE_GRACE_TIME.
 *   SEARCHING  line_search_step(): sweep both sides, turn back, reverse, sweep again (sweep=<phase>).
 *   LOST       the search gave up (or timed out): zero-speed refreshes until the line is seen again.
 *   HELD       someone else has the wheels: Motion Control held or faulted, or subsystem 5 avoiding.
 *   MANEUVER   a route action (route.h): brake, advance to the junction, pivot.
 *   STOPPED    a failed barcode read or a failed route action: stopped until power-cycle.
 *   CAL_WAIT   GP21 calibration finished (done or failed): stopped, no route action, until GP21 is pressed again.
 * The barcode reader has its own modes (barcode.c). */
#include "steering.h"
#include "motion_hal.h"
#include "line_rejoin.h"
#include "route.h"
/* Both IR sensors read black (1) when centred on the line; GP26 sits slightly right of centre. */
#define DEG_TO_RAD          (0.017453292519943295)
#define LINE_SESSION        3001U
#define LINE_SPEED          0.05  /* m/s forward while following. Halved from 0.1 so the barcode sensor gets
                                   * about 7 samples (0.42 mm each) across a 3 mm bar of the small barcode. */
#define LINE_CORRECTION     (LINE_SPEED * 0.375) /* m/s per wheel when one sensor is off the line: the same
                                   * inner/outer wheel ratio as 0.0375 at 0.1 m/s. */
#define LINE_SEARCH_SPEED   0.1   /* m/s outer wheel while pivoting on the stopped inner wheel.
                                   * Spinning in place stalled the reversing wheel. */
#define LINE_GRACE_TIME     0.2   /* s to keep the last command after a loss (gaps, flicker). */
#define LINE_SWEEP_ANGLE    (60.0 * DEG_TO_RAD) /* rad each side of the last heading on the line. */
#define LINE_BACKUP_SPEED   0.075 /* m/s reversing back to where the line was last seen. */
#define LINE_BACKUP_EXTRA   0.02  /* m reversed beyond the overshoot. */
#define LINE_SEARCH_TIMEOUT 15.0  /* s backstop before holding still, e.g. if a wheel slips. */
/* Barcode window: while the barcode task reads a pass (barcode_code_pass()), the car steers more gently
 * (LINE_BC_CORRECTION), and a loss first holds the heading of the last on-line sample at line speed for up to LINE_BC_HOLD_LOST before the normal grace and
 * search take over. Both end as soon as the heading has turned LINE_BC_MAX_TURN from the pass start
 * (a bend), and after LINE_BC_HOLD_MAX. */
#define LINE_BC_HOLD_MAX    0.35  /* m from the start of the pass (big code 26 cm plus margins). */
#define LINE_BC_HOLD_LOST   0.05  /* m driven while lost before the hold gives way to the search. */
#define LINE_BC_MAX_TURN    0.15  /* rad (~8.6 deg) of heading change during the pass: a bend, steer normally.
                                   * Straight-line wiggle over the codes stayed under 0.10 rad in every logged pass. */
#define LINE_BC_HOLD_GAIN   100.0 /* deg/s of turn per rad of heading error. */
#define LINE_BC_HOLD_RATE   20.0  /* deg/s turn limit while holding. */
#define LINE_BC_CORRECTION  (LINE_CORRECTION * 0.4) /* m/s per wheel while a pass is read: a gentler wiggle,
                                   * because the barcode sensor sits ~6 cm to the side (every turn speeds it up or
                                   * slows it down against the centre distance the decoder uses) and ~18 cm ahead
                                   * (the chassis rocking moves it up and down). Same window as the hold. */
#define LINE_OBSTACLE_AVOIDING 5.0 /* Obstacle status values[3] while subsystem 5 drives round an
                                    * obstacle: S5_STATE_AVOIDING in subsystem5.h (not included:
                                    * it needs the kernel headers the host tests lack). */

#ifdef ROBO_LINE_LOSS_TEST
/* Line-loss test image only (see .claude/notes/line-pivot-sweep.md): after a loss, stop, spin on the
 * spot towards the last side seen (up to a full turn), then run the normal sweep. board_config.c
 * keeps the 25% cap unless -DROBO_LINE_LOSS_CAP says otherwise. With -DROBO_LINE_VEER_TEST=1 the
 * image also drives off the line once, LINE_TEST_FOLLOW_TIME after the line was first seen. */
#define LINE_STOP_TIME        0.1   /* s at zero speed before spinning. */
#define LINE_SPIN_RATE        60.0  /* deg/s on the spot (about 0.06 m/s per wheel); lowered from 90 for the 25% cap. */
#define LINE_SPIN_ANGLE       (360.0 * DEG_TO_RAD)
#define LINE_REACQUIRE_TIME   0.3   /* s of tape in view before a search counts as finished. */
#ifdef ROBO_LINE_VEER_TEST /* Also -DROBO_LINE_VEER_TEST=1: drive off the line once to exercise the search. */
#define LINE_TEST_FOLLOW_TIME 5.0   /* s on the line before the test veer. */
#define LINE_TEST_VEER_TIME   1.0   /* s of forced left arc. */
#define LINE_TEST_VEER_RATE   60.0  /* deg/s left while veering. */
#endif
#endif

/* Numeric order is printed as sweep=%u in the diagnostics. STOP and SPIN come last so the
 * older values keep their numbers; they are only used in the line-loss test image. */
typedef enum { SEARCH_IDLE, SEARCH_FIRST_SIDE, SEARCH_OTHER_SIDE, SEARCH_TURN_BACK,
               SEARCH_REVERSE, SEARCH_HOLD, SEARCH_STOP, SEARCH_SPIN, SEARCH_PHASE_COUNT } line_search_phase_t;
typedef struct {
    line_search_phase_t phase;
    double dir;          /* +1 pivot left, -1 pivot right. */
    bool backed;         /* Already reversed once during this loss. */
    double seen_heading; /* Heading while the line was last seen. */
    double seen_centre;  /* Centre travel (m) while the line was last seen. */
    double lost_centre;  /* Centre travel when the search started. */
    double backup_from;  /* Centre travel when reversing started. */
    double stop_from;    /* Time since loss when the stop phase began. */
    double spin_from;    /* Heading when the on-the-spot spin began. */
} line_search_t;

static void search_toward(line_search_t *s, line_direction_t last_side)
{
    s->phase = SEARCH_FIRST_SIDE;
    s->dir = (last_side == LINE_RIGHT) ? -1.0 : 1.0; /* Left if the side is unknown. */
}

/* Zero-speed refresh on the line session: keeps ownership without a latched hold, and never lets the
 * tracking command expire (an expiry is a latched F_EXPIRED fault in production). */
static motion_result_t line_hold(void) { return motion_steer(SOURCE_LINE, LINE_SESSION, 0.0, 0.0); }

/* Motion Control neither held nor faulted. */
static bool line_motion_ok(const robot_status_t *status)
{
    return !status->motion.hold && status->motion.fault == F_NONE;
}

/* Inner wheel stopped, outer wheel at LINE_SEARCH_SPEED. */
static motion_result_t line_pivot(double dir)
{
    const double half = LINE_SEARCH_SPEED / 2.0;
    return (dir < 0.0) ? motion_steer_right(SOURCE_LINE, LINE_SESSION, half, half)
                       : motion_steer_left(SOURCE_LINE, LINE_SESSION, half, half);
}

/* Pivot towards the last side seen up to the sweep angle, then the other way past
 * the original heading. If both miss, turn back to the heading on the line, reverse
 * past the overshoot and sweep once more, then hold still.
 * Zero-speed refreshes keep ownership without a latched hold, so following resumes
 * when the line is seen again. Stopping the refreshes instead would raise F_EXPIRED. */
static motion_result_t line_search_step(line_search_t *s, const motion_status_t *motion,
                                        line_direction_t last_side, double lost_for)
{
    double turned = motion->heading - s->seen_heading;
    if (s->phase == SEARCH_IDLE) {
        search_toward(s, last_side);
        s->lost_centre = motion->centre;
#ifdef ROBO_LINE_LOSS_TEST
        s->phase = SEARCH_STOP;
        s->stop_from = lost_for;
#endif
    }
#ifdef ROBO_LINE_LOSS_TEST
    if (s->phase == SEARCH_STOP && lost_for - s->stop_from >= LINE_STOP_TIME) {
        s->phase = SEARCH_SPIN;
        s->spin_from = motion->heading;
    }
    if (s->phase == SEARCH_SPIN && s->dir * (motion->heading - s->spin_from) >= LINE_SPIN_ANGLE) {
        search_toward(s, last_side); /* Full turn without the line: normal sweep. */
    }
#endif
    if (s->phase <= SEARCH_OTHER_SIDE && s->dir * turned >= LINE_SWEEP_ANGLE) {
        if (s->phase == SEARCH_FIRST_SIDE) { s->phase = SEARCH_OTHER_SIDE; s->dir = -s->dir; }
        else if (!s->backed) { s->phase = SEARCH_TURN_BACK; s->dir = -s->dir; }
        else { s->phase = SEARCH_HOLD; }
    }
    if (s->phase == SEARCH_TURN_BACK && s->dir * turned >= 0.0) {
        s->phase = SEARCH_REVERSE;
        s->backup_from = motion->centre;
    }
    if (s->phase == SEARCH_REVERSE &&
        motion->centre <= s->backup_from - (s->lost_centre - s->seen_centre + LINE_BACKUP_EXTRA)) {
        search_toward(s, last_side);
        s->backed = true;
    }
    if (s->phase == SEARCH_HOLD || lost_for >= LINE_SEARCH_TIMEOUT) { return line_hold(); }
    if (s->phase == SEARCH_REVERSE) { return motion_steer(SOURCE_LINE, LINE_SESSION, -LINE_BACKUP_SPEED, 0.0); }
#ifdef ROBO_LINE_LOSS_TEST
    if (s->phase == SEARCH_STOP) { return line_hold(); }
    if (s->phase == SEARCH_SPIN) { return motion_steer(SOURCE_LINE, LINE_SESSION, 0.0, s->dir * LINE_SPIN_RATE); }
#endif
    return line_pivot(s->dir);
}

static motion_result_t line_follow(line_direction_t drive, double correction)
{
    if (drive == LINE_LEFT) { return motion_steer_left(SOURCE_LINE, LINE_SESSION, LINE_SPEED, correction); }
    if (drive == LINE_RIGHT) { return motion_steer_right(SOURCE_LINE, LINE_SESSION, LINE_SPEED, correction); }
    return motion_steer(SOURCE_LINE, LINE_SESSION, LINE_SPEED, 0.0);
}

/* Line error/confidence slots, plus both-sensors-black for line_rejoin_seen() (values[3] stays 0;
 * route_flag() sets it while a route pivots). Junctions come from the barcode reader. */
static void line_publish(line_direction_t direction, double now)
{
    double error = 0.0;
    bool on_line = direction != LINE_LOST && direction != LINE_WAIT;
    if (direction == LINE_LEFT) { error = -1.0; }
    else if (direction == LINE_RIGHT) { error = 1.0; }
    subsystem_status_t line_status = {.available = true, .timestamp = now,
        .values = {error, on_line ? 1.0 : 0.0, 0.0, 0.0}};
    line_status.values[LINE_VALUE_BOTH_BLACK] = (direction == LINE_STRAIGHT) ? 1.0 : 0.0;
    (void)robot_update_subsystem(SOURCE_LINE, &line_status);
}

/* ---- Line task state (see the list at the top of this section) ---- */
typedef enum { LS_STARTUP, LS_WAITING, LS_FOLLOWING, LS_BARCODE, LS_GRACE, LS_SEARCHING,
               LS_LOST, LS_HELD, LS_MANEUVER, LS_STOPPED, LS_CAL_WAIT, LS_COUNT } line_state_t;

static const char *line_state_name(line_state_t s)
{
    static const char *const names[] = {"STARTUP", "WAITING", "FOLLOWING", "BARCODE", "GRACE", "SEARCHING",
                                        "LOST", "HELD", "MANEUVER", "STOPPED", "CAL_WAIT"};
    _Static_assert(sizeof names / sizeof names[0] == LS_COUNT, "one name per line state");
    return names[s];
}

static const char *line_search_name(line_search_phase_t p)
{
    static const char *const names[] = {"IDLE", "FIRST_SIDE", "OTHER_SIDE", "TURN_BACK", "REVERSE", "HOLD",
                                        "STOP", "SPIN"};
    _Static_assert(sizeof names / sizeof names[0] == SEARCH_PHASE_COUNT, "one name per search phase");
    return names[p];
}

/* FOLLOWING, BARCODE and GRACE alternate several times a second: only the line_pid report shows them. */
static bool line_state_coarse(line_state_t s)
{
    return s != LS_FOLLOWING && s != LS_BARCODE && s != LS_GRACE;
}

static void line_set_state(line_state_t *state, line_state_t next)
{
    if (next != *state && (line_state_coarse(*state) || line_state_coarse(next))) {
        tm_printf((const UB *)"[S3-LINE] %s -> %s\n", (const UB *)line_state_name(*state), (const UB *)line_state_name(next));
    }
    *state = next;
}

static void line_report(unsigned centre, unsigned right, line_direction_t direction, line_state_t lstate,
                        line_search_phase_t phase, motion_result_t result, const motion_status_t *m)
{
    tm_printf((const UB *)"line_pid centre_ir=%u right_ir=%u direction=%u lstate=%s sweep=%s request=%u state=%u fault=%u speed_mm_s=%d,%d output_permille=%d,%d target_mm_s=%d,%d\n",
        centre, right, (unsigned)direction, (const UB *)line_state_name(lstate), (const UB *)line_search_name(phase), (unsigned)result,
        (unsigned)m->state, (unsigned)m->fault,
        (int)(m->speed[0] * 1000.0), (int)(m->speed[1] * 1000.0),
        (int)(m->output[0] * 1000.0), (int)(m->output[1] * 1000.0),
        (int)(m->target[0] * 1000.0), (int)(m->target[1] * 1000.0));
    if (m->fault == F_FEEDBACK) {
        /* Encoder fault details recorded by Motion Control (see motion.h enums). */
        const encoder_sample_t *f = &m->feedback_fault_sample;
        tm_printf((const UB *)"line_fb cause=%u reliable=%u counts=%lu,%lu invalid=%lu,%lu previous_invalid=%lu,%lu\n",
            (unsigned)m->feedback_cause, (unsigned)f->reliable,
            (unsigned long)f->count[0], (unsigned long)f->count[1],
            (unsigned long)f->invalid[0], (unsigned long)f->invalid[1],
            (unsigned long)m->feedback_previous_invalid[0], (unsigned long)m->feedback_previous_invalid[1]);
    }
}

#if ROBO_BARCODE_TASK
/* ---- Route actions (route.h): after a valid read (SOURCE_BARCODE values[1] counts them, values[0] is the
 * letter) or at the junction that follows it (values[2], the first black after a valid read, barcode.c).
 * Done here because this task owns the motion session: the Vehicle Controller rejects any other command
 * until the session has ended. Restored from commit 9291881 (Z junction pivot) and generalised. ---- */
#define ROUTE_ADVANCE_M       0.15  /* The car drives this far past the junction trigger (the barcode sensor seeing the
                                     * black), coasting included, before turning. Line IRs are 15 cm ahead of the axle
                                     * and the barcode sensor 3 cm ahead of them. 0 = turn where it was seen. */
#define ROUTE_PIVOT_MIN       (45.0 * DEG_TO_RAD)  /* Junction turn: black only counts once the pivot has turned this far ... */
#define ROUTE_PIVOT_MAX       (150.0 * DEG_TO_RAD) /* ... and the old line comes back near 220 deg: give up here. */
#define ROUTE_UTURN_MIN       (150.0 * DEG_TO_RAD) /* U-turn: the same line, found again behind the car ... */
#define ROUTE_UTURN_MAX       (240.0 * DEG_TO_RAD) /* ... give up past this. */
#define ROUTE_UTURN_IGNORE_M  0.7   /* Path not read after a U-turn: the pivot itself adds ~0.18 m (path counts both
                                     * directions), then the car drives back over the code it just read (up to 28 cm). */
#define ROUTE_PIVOT_TIMEOUT   10.0  /* s backstop for the pivot (a stalled wheel). */
#define ROUTE_STILL_SPEED     0.005 /* m/s per wheel counted as stopped. */
#define ROUTE_BRAKE_TIMEOUT   1.5   /* s of zero-speed refreshes before giving up. */
#define ROUTE_MOVE_TIMEOUT    8.0   /* s for the advance. */
#define ROUTE_PIVOT_MIN_RANGE 0.10  /* m: the pivot pauses while something is this close ahead (subsystem 5 does not
                                     * stop the car at 150 mm while the route pivots, see route_flag()). */
#define LINE_CAL_BUTTON       ((unsigned)BOARD_CAL_BUTTON) /* GP21, active low: calibrate, and resume after it. */

#define ROUTE_CASE(letter_, action_, trigger_) if (letter == (letter_)) { *trigger = (trigger_); return (action_); }
static route_action_t route_action_for(char letter, route_trigger_t *trigger)
{
    ROUTE_TABLE(ROUTE_CASE)
    *trigger = ROUTE_AT_JUNCTION;
    return ROUTE_NONE;
}

static double route_abs(double x) { return (x < 0.0) ? -x : x; }

/* Zero-speed refreshes keep the session (no expiry) until both wheels are nearly still. */
static bool route_brake(void)
{
    double start = motion_hal_time();
    static robot_status_t status; /* Static: the line task stack is nearly full. */
    while (motion_hal_time() - start < ROUTE_BRAKE_TIMEOUT) {
        (void)line_hold();
        (void)tk_dly_tsk(20U);
        if (!motion_get_status(&status) || !line_motion_ok(&status)) { return false; }
        if (route_abs(status.motion.speed[0]) < ROUTE_STILL_SPEED &&
            route_abs(status.motion.speed[1]) < ROUTE_STILL_SPEED) { return true; }
    }
    return false;
}

/* Drives straight on the line session (refreshed every 20 ms, so it never expires) until the path has
 * moved 'metres', then brakes. The session is never given up: an expired tracking command is a latched
 * F_EXPIRED fault in production (motion.c), and only COMPLETE/CANCELLED/FAULT drop the owner. */
static bool route_advance(double metres)
{
    double start = motion_hal_time(), from;
    static robot_status_t status; /* Static: the line task stack is nearly full. */
    if (!motion_get_status(&status)) { return false; }
    from = status.motion.path;
    while (motion_hal_time() - start < ROUTE_MOVE_TIMEOUT) {
        (void)motion_steer(SOURCE_LINE, LINE_SESSION, LINE_SPEED, 0.0);
        (void)tk_dly_tsk(20U);
        if (!motion_get_status(&status) || !line_motion_ok(&status)) { return false; }
        if (status.motion.path - from >= metres) { return route_brake(); }
    }
    return false;
}

/* Line status with values[3] = 1 while the route pivots: subsystem 5 then skips its 150 mm stop.
 * line_publish() writes 0 on every pass of the line loop, so the flag clears itself once following resumes. */
static void route_flag(bool pivoting, bool black)
{
    subsystem_status_t line_status = {.available = true, .timestamp = motion_hal_time(),
        .values = {0.0, black ? 1.0 : 0.0, 0.0, pivoting ? 1.0 : 0.0}};
    (void)robot_update_subsystem(SOURCE_LINE, &line_status);
}

/* Pivots (dir +1 left, -1 right; the line search's own pivot on the stopped inner wheel, session 3001)
 * until a line IR sees black, having first left the line and turned at least min_turn. Returns at once
 * on the black, without braking, so the line loop takes over the same session immediately. */
static bool route_pivot(double dir, double min_turn, double max_turn)
{
    static robot_status_t status;
    double heading0, turned = 0.0, start = motion_hal_time();
    bool left_line = false;
    unsigned seen = 0U, rejected = 0U;
    if (!motion_get_status(&status)) { return false; }
    heading0 = status.motion.heading;
    while (motion_hal_time() - start < ROUTE_PIVOT_TIMEOUT) {
        unsigned right = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre = (unsigned)gpio_get(BOARD_LINE_C);
        bool black = right == 1U || centre == 1U;
        route_flag(true, black);
        if (!motion_get_status(&status) || !line_motion_ok(&status)) { break; }
        turned = dir * (status.motion.heading - heading0);
        if (!black) { left_line = true; }
        seen = (left_line && black && turned >= min_turn) ? seen + 1U : 0U;
        if (seen >= 2U) { /* Two samples in a row, as the follower's own debounce: one glitch is not the line. */
            tm_printf((const UB *)"route: line found after pivoting %d deg (centre_ir=%u right_ir=%u)\n",
                      (int)(turned / DEG_TO_RAD), centre, right);
            return true;
        }
        if (turned >= max_turn) { break; }
#if defined(ROBO_SUBSYSTEM5) && ROBO_SUBSYSTEM5
        if (!status.obstacle.available || status.obstacle.values[0] < ROUTE_PIVOT_MIN_RANGE) {
#else
        if (false) { /* No obstacle sensor in this image. */
#endif
            /* Something right ahead: hold still (zero-speed refreshes) until it moves; the timeout still ends it. */
            (void)line_hold();
            (void)tk_dly_tsk(20U);
            continue;
        }
        /* Not accepted for ~0.2 s (well under the 0.3 s command lifetime): give up rather than wait. */
        rejected = (line_pivot(dir) == R_OK) ? 0U : rejected + 1U;
        if (rejected >= 10U) { break; }
        (void)tk_dly_tsk(20U);
    }
    (void)line_hold(); /* Zero speed first, before any printing: the last pivot refresh is about to expire. */
    route_flag(false, false);
    tm_printf((const UB *)"route: no line found while pivoting (turned %d deg)\n", (int)(turned / DEG_TO_RAD));
    return false;
}

/* One route action. Returns true when following can resume; *side is where the line will be after it. */
static bool route_run(route_action_t action, bool at_junction, double path_seen, line_direction_t *side)
{
    static robot_status_t status; /* Static: the line task stack is nearly full. */
    double dir = (action == ROUTE_RIGHT) ? -1.0 : 1.0; /* The U-turn pivots left. */
    *side = LINE_STRAIGHT;
    if (action == ROUTE_STRAIGHT) { tm_printf((const UB *)"route: straight on\n"); return true; }
    if (action == ROUTE_UTURN) { barcode_ignore_for(ROUTE_UTURN_IGNORE_M); } /* Before turning: the pivot sweeps the sensor. */
    if (!route_brake()) { tm_printf((const UB *)"route: FAILED while braking\n"); return false; }
    if (!motion_get_status(&status)) { return false; }
    if (at_junction && ROUTE_ADVANCE_M > 0.0) {
        /* Distance still to drive past the trigger, minus what the car coasted (never reverses). */
        double advance = ROUTE_ADVANCE_M - (status.motion.path - path_seen);
        tm_printf((const UB *)"route: stopped, coasted %d mm, advancing %d mm\n",
                  (int)((status.motion.path - path_seen) * 1000.0), (int)(advance * 1000.0));
        if (advance > 0.005 && !route_advance(advance)) {
            tm_printf((const UB *)"route: FAILED during the advance\n");
            return false;
        }
    }
    if (!((action == ROUTE_UTURN) ? route_pivot(dir, ROUTE_UTURN_MIN, ROUTE_UTURN_MAX)
                                  : route_pivot(dir, ROUTE_PIVOT_MIN, ROUTE_PIVOT_MAX))) {
        tm_printf((const UB *)"route: FAILED during the pivot\n");
        return false;
    }
    *side = (dir > 0.0) ? LINE_RIGHT : LINE_LEFT; /* An overshoot leaves the line on the other side. */
    return true;
}
#endif

#if defined(ROBO_SUBSYSTEM4) && ROBO_SUBSYSTEM4
/* Buddy 4 captures its tilt reference while the robot is still, about 2 s after
 * boot; driving earlier makes that capture fail. Terrain slot values[3] holds
 * validity bits | T_STATE << 3 (docs/API.md): 0 INIT, 4 RECOVERY, others mean the
 * capture is over (done, or the IMU gave up). */
#define LINE_IMU_WAIT_MAX       10.0 /* s cap: a missing or faulty IMU never stops the robot. */
#define LINE_TERRAIN_INIT       0U
#define LINE_TERRAIN_RECOVERY   4U

static bool line_imu_settled(const robot_status_t *status)
{
    unsigned state;
    if (!status->terrain.available || !(status->terrain.values[3] >= 0.0)) { return false; }
    state = ((unsigned)status->terrain.values[3] >> 3) & 7U;
    return (state != LINE_TERRAIN_INIT) && (state != LINE_TERRAIN_RECOVERY);
}

/* Runs once before the first motion request; nothing is driving yet. */
static void line_wait_for_imu(void)
{
    double start = motion_hal_time();
    robot_status_t status;
    bool settled = false;
    while (!settled && ((motion_hal_time() - start) < LINE_IMU_WAIT_MAX)) {
        settled = motion_get_status(&status) && line_imu_settled(&status);
        if (!settled) { (void)tk_dly_tsk(20U); }
    }
    tm_printf((const UB *)"line_imu_wait settled=%u waited_ms=%d\n", (unsigned)settled,
              (int)((motion_hal_time() - start) * 1000.0));
}
#endif

/* Everything the line loop remembers between its 20 ms ticks. Static (not on the stack): the line
 * task's stack is nearly full. */
typedef struct {
    line_state_t state;
    line_steering_t filter;       /* Debounce of the two IR sensors. */
    line_search_t search;         /* SEARCHING sub-state. */
    line_direction_t last_side;   /* Raw side the line was last seen on; STRAIGHT = unknown. */
    line_direction_t follow;      /* Last follow command, repeated during GRACE. */
    double lost_since;            /* Last time the line was seen. */
    double next_report;           /* Next line_pid report. */
    bool baselined;               /* Barcode counters read once: earlier events cannot trigger anything. */
    bool stopped;                 /* A failed read or route latched the car: STOPPED, not HELD. */
    bool halted;                  /* A route action failed: hold still for good (zero-speed refreshes). */
    unsigned seen_junctions, seen_failures;
#if ROBO_BARCODE_TASK
    unsigned seen_reads;
    unsigned cal_presses;         /* Consecutive polls with GP21 pressed (debounce). */
    bool calibrating;             /* GP21 pressed: the reader is calibrating on the next full pass. */
    bool cal_wait;                /* CAL_WAIT: calibration finished, stopped until GP21 is pressed again. */
    double bc_hold_from;          /* Path (m) where the barcode pass began, see LINE_BC_*; < 0 = none. */
    double bc_lost_from;          /* Path (m) where the line was lost during that pass; < 0 = not lost. */
    double bc_heading_from;       /* Heading (rad) at the start of that pass. */
#endif
#ifdef ROBO_LINE_RUN_LIMIT_S
    double run_start;             /* First follow command; the car is held still ROBO_LINE_RUN_LIMIT_S later. */
    bool run_done;
#endif
#ifdef ROBO_LINE_LOSS_TEST
    double on_line_since;         /* Start of the current stretch with the line seen. */
    double search_began;          /* When the current search started; glimpses do not reset it. */
#ifdef ROBO_LINE_VEER_TEST
    double veer_until;
    bool veered;
#endif
#endif
} line_ctx_t;

/* Drive on owning the line session (not READY, not someone else's). */
static bool line_driving(const robot_status_t *status, bool owns)
{
    return owns && status->motion.state == DRIVING;
}

/* May start or continue a line command: Motion Control idle, or already driving for this task. */
static bool line_can_drive(const robot_status_t *status, bool owns)
{
    return status->motion.state == READY || line_driving(status, owns);
}

#if ROBO_BARCODE_TASK
/* After something else had the wheels (a route action, the calibration stop): start afresh. */
static void line_restart(line_ctx_t *c)
{
    c->filter = (line_steering_t){0};
    c->search = (line_search_t){0};
    c->lost_since = motion_hal_time();
#ifdef ROBO_LINE_LOSS_TEST
    c->on_line_since = -1.0;
    c->search_began = -1.0;
#endif
}

/* GP21, active low: true once per press (two polls = 40 ms down); holding it does not repeat. */
static bool line_cal_pressed(line_ctx_t *c)
{
    if (gpio_get(LINE_CAL_BUTTON)) { c->cal_presses = 0U; return false; }
    return ++c->cal_presses == 2U;
}

/* GP21 calibration: press, drive over the calibration code, stop (CAL_WAIT, no route action), press
 * again to carry on. A sensor check: the reader proves it can read the code, and keeps the sensitivity
 * that read it. Returns false when this tick was used up (stopped, or just resumed). */
static bool line_calibration_step(line_ctx_t *c, const robot_status_t *status)
{
    bool pressed = line_cal_pressed(c);
    barcode_cal_t result;
    if (c->cal_wait) {
        (void)line_hold(); /* Zero-speed refreshes: no latched hold, nothing expires. */
        if (!pressed) { return false; }
        c->cal_wait = false;
        /* Reads and junctions from the wait (robot handled) are forgotten; a failed read is not: it is
         * acted on next tick (STOPPED), as always. The reader also drops a junction armed meanwhile. */
        c->seen_reads = (unsigned)status->barcode.values[1];
        c->seen_junctions = (unsigned)status->barcode.values[2];
        barcode_ignore_for(0.0);
        line_restart(c);
        tm_printf((const UB *)"calibration: GP21 pressed, carrying on\n");
        return false;
    }
    if (!c->calibrating) {
        if (pressed && !c->stopped) {
            c->calibrating = true;
            barcode_calibrate_begin();
            tm_printf((const UB *)"calibration: started; follow the line over *%c*\n", ROUTE_CALIBRATION_LETTER);
        }
        return true;
    }
    result = barcode_calibration_result();
    if (result == BARCODE_CAL_NONE) { return true; }
    c->calibrating = false;
    tm_printf((const UB *)"calibration %s%s\n",
              (result == BARCODE_CAL_DONE) ? "done" : "FAILED, the sensor could not read the code",
              c->stopped ? "" : " - press GP21 to carry on");
    if (c->stopped) { return true; } /* A failed read already stopped the car: STOPPED stays. */
    c->cal_wait = true;
    (void)route_brake();
    (void)line_hold();
    line_set_state(&c->state, LS_CAL_WAIT);
    return false;
}

/* A new read or junction: run its route action if it is due now (route.h). Returns false when an
 * action ran (this tick was used up). A failed action holds still for good (STOPPED). */
static bool line_route_event(line_ctx_t *c, const robot_status_t *status, bool owns, unsigned reads, unsigned junctions)
{
    bool at_junction = reads == c->seen_reads; /* A new read first; its junction comes later. */
    char letter = (char)(unsigned)status->barcode.values[0];
    route_trigger_t trigger;
    route_action_t action = route_action_for(letter, &trigger);
    line_direction_t side = LINE_STRAIGHT;
    if (at_junction) { c->seen_junctions = junctions; } else { c->seen_reads = reads; }
    tm_printf((const UB *)"route: %s after *%c*, path %d mm\n", at_junction ? "junction" : "read",
              letter, (int)(status->motion.path * 1000.0));
    if (action == ROUTE_NONE || (trigger == ROUTE_AT_JUNCTION) != at_junction) {
        return true; /* Nothing to do now (no action for this letter, or it waits for the junction). */
    }
    if (c->search.phase != SEARCH_IDLE || !owns || !line_motion_ok(status) ||
        status->obstacle.values[3] == LINE_OBSTACLE_AVOIDING) {
        /* A spin or sweep carries the barcode sensor in an arc, possibly back over the code. */
        tm_printf((const UB *)"route: not following the line right now, action skipped\n");
        return true;
    }
    line_set_state(&c->state, LS_MANEUVER);
    if (!route_run(action, at_junction, status->motion.path, &side)) {
        (void)line_hold(); /* Zero speed at once; line_loop keeps refreshing it. */
        c->stopped = true;
        c->halted = true; /* line_loop holds still from now on (until power-cycle). */
        line_set_state(&c->state, LS_STOPPED);
        tm_printf((const UB *)"route: holding still after the failure\n");
        return false;
    }
    line_restart(c);
    c->last_side = side;
    c->follow = side;
    return false;
}
#endif

/* Calibration, failed reads, reads and junctions. Returns false when this tick was used up and the
 * line must not be driven. */
static bool line_barcode_events(line_ctx_t *c, const robot_status_t *status, bool owns)
{
    unsigned junctions = (unsigned)status->barcode.values[2];
    unsigned failures = (unsigned)status->barcode.values[3];
#if ROBO_BARCODE_TASK
    unsigned reads = (unsigned)status->barcode.values[1];
    if (!line_calibration_step(c, status)) { return false; }
#endif
    if (!c->baselined) {
        c->seen_junctions = junctions; /* Whatever was published before now cannot trigger a turn. */
        c->seen_failures = failures;
#if ROBO_BARCODE_TASK
        c->seen_reads = reads;
#endif
        c->baselined = true;
    } else if (failures != c->seen_failures) {
        /* The barcode task could not decode a pass: stop right there. The hold stays until power-cycle
         * (this image has no Wi-Fi to release it); the junction's own bar is never counted (barcode.c). */
        c->seen_failures = failures;
        motion_stop();
        c->stopped = true;
        line_set_state(&c->state, LS_STOPPED);
        tm_printf((const UB *)"route: barcode read failed, car stopped (power-cycle to rerun)\n");
        return false;
#if ROBO_BARCODE_TASK
    } else if (reads != c->seen_reads || junctions != c->seen_junctions) {
        return line_route_event(c, status, owns, reads, junctions);
#else
    } else if (junctions != c->seen_junctions) {
        c->seen_junctions = junctions;
        tm_printf((const UB *)"route: junction after %c, path %d mm (no action, following on)\n",
                  (char)(unsigned)status->barcode.values[0], (int)(status->motion.path * 1000.0));
#endif
    }
    (void)owns;
    return true;
}

/* While a barcode pass is read: the steering correction to use, and *hold = hold the heading through
 * a short loss (BARCODE state). Both end at a bend (LINE_BC_MAX_TURN) or after LINE_BC_HOLD_MAX. */
static double line_barcode_window(line_ctx_t *c, const robot_status_t *status, line_direction_t direction,
                                  bool *window, bool *hold)
{
#if ROBO_BARCODE_TASK
    if (barcode_code_pass()) {
        if (c->bc_hold_from < 0.0) {
            c->bc_hold_from = status->motion.path;
            c->bc_heading_from = status->motion.heading;
        }
    } else {
        c->bc_hold_from = -1.0; /* Pass over, or tape / big code seen: steer normally at once. */
    }
    *window = c->bc_hold_from >= 0.0 &&
              (status->motion.path - c->bc_hold_from) < LINE_BC_HOLD_MAX && c->search.phase == SEARCH_IDLE &&
              status->motion.heading - c->bc_heading_from < LINE_BC_MAX_TURN &&
              c->bc_heading_from - status->motion.heading < LINE_BC_MAX_TURN;
    if (direction != LINE_LOST) { c->bc_lost_from = -1.0; }
    else if (*window && c->bc_lost_from < 0.0) { c->bc_lost_from = status->motion.path; }
    *hold = *window && direction == LINE_LOST && (status->motion.path - c->bc_lost_from) < LINE_BC_HOLD_LOST;
    return *window ? LINE_BC_CORRECTION : LINE_CORRECTION;
#else
    (void)c; (void)status; (void)direction;
    *window = false;
    *hold = false;
    return LINE_CORRECTION;
#endif
}

/* One tick of line following: picks the command and the state. */
static void line_drive(line_ctx_t *c, const robot_status_t *status, bool owns, line_direction_t direction,
                       unsigned centre, unsigned right, double now)
{
    /* Subsystem 5 drives round obstacles itself; keep off the wheels until it
     * publishes that it is back on the line. Exact compare: the value is copied. */
    bool avoiding = status->obstacle.values[3] == LINE_OBSTACLE_AVOIDING;
    bool usable = line_motion_ok(status) && !avoiding && !c->stopped; /* STOPPED never drives again. */
    motion_result_t result = R_HELD;
    line_state_t next = usable ? LS_WAITING : (c->stopped ? LS_STOPPED : LS_HELD);
    line_direction_t drive = direction;
    bool grace_ok = true, window, bc_hold;
    double correction = line_barcode_window(c, status, direction, &window, &bc_hold);
#ifdef ROBO_LINE_LOSS_TEST
    /* A glimpse of tape during a search is not "found": keep the 360 deg budget and the
     * search clock, and stand still until the tape has stayed in view for
     * LINE_REACQUIRE_TIME. No blind grace drive while searching. */
    bool settled = c->on_line_since >= 0.0 && (now - c->on_line_since) >= LINE_REACQUIRE_TIME;
    bool glimpse = direction != LINE_LOST && direction != LINE_WAIT &&
                   c->search.phase != SEARCH_IDLE && !settled;
    grace_ok = c->search.phase == SEARCH_IDLE;
#else
    const bool glimpse = false;
#endif
    if (direction != LINE_LOST && !glimpse) {
        c->search.seen_heading = status->motion.heading;
        c->search.seen_centre = status->motion.centre;
        c->search.phase = SEARCH_IDLE;
        c->search.backed = false;
        if (usable && direction != LINE_WAIT) { next = window ? LS_BARCODE : LS_FOLLOWING; }
    }
    else if (direction == LINE_LOST && grace_ok && !bc_hold && owns && (now - c->lost_since) < LINE_GRACE_TIME) {
        drive = c->follow; /* Bridge short dropouts. */
        if (usable) { next = LS_GRACE; }
    }
#ifdef ROBO_LINE_RUN_LIMIT_S
    /* Analysis builds: one run of ROBO_LINE_RUN_LIMIT_S seconds, then stand still (reset to rerun). */
    if (!c->run_done && c->run_start >= 0.0 && (now - c->run_start) >= (double)ROBO_LINE_RUN_LIMIT_S) {
        c->run_done = true;
        tm_printf((const UB *)"line_test: run limit reached, car stopped\n");
    }
    if (c->run_done) {
        if (usable && line_can_drive(status, owns)) { result = line_hold(); }
        next = LS_HELD;
    } else
#endif
#ifdef ROBO_LINE_LOSS_TEST
#ifdef ROBO_LINE_VEER_TEST
    if (now < c->veer_until && usable && line_can_drive(status, owns)) {
        result = motion_steer(SOURCE_LINE, LINE_SESSION, LINE_SPEED, LINE_TEST_VEER_RATE);
    } else
#endif
    if (glimpse && usable && line_driving(status, owns)) {
        result = line_hold();
        next = LS_SEARCHING;
    } else
#endif
    if (c->stopped) {
        /* Failed read: even if something releases the latched hold, keep refreshing zero speed. */
        if (line_motion_ok(status) && !avoiding && line_can_drive(status, owns)) { result = line_hold(); }
    } else if (bc_hold && drive == LINE_LOST && usable && line_driving(status, owns)) {
        double turn = LINE_BC_HOLD_GAIN * (c->search.seen_heading - status->motion.heading);
        if (turn > LINE_BC_HOLD_RATE) { turn = LINE_BC_HOLD_RATE; }
        if (turn < -LINE_BC_HOLD_RATE) { turn = -LINE_BC_HOLD_RATE; }
        result = motion_steer(SOURCE_LINE, LINE_SESSION, LINE_SPEED, turn);
        c->lost_since = now; /* Grace and search start afresh if the line is still gone after the hold. */
        next = LS_BARCODE;
    } else if (drive == LINE_LOST && usable && line_driving(status, owns)) {
        double lost_for = now - c->lost_since;
#ifdef ROBO_LINE_LOSS_TEST
        if (c->search.phase == SEARCH_IDLE) { c->search_began = c->lost_since; }
        lost_for = now - c->search_began; /* Glimpses must not restart the clock. */
#endif
        result = line_search_step(&c->search, &status->motion, c->last_side, lost_for);
        next = (c->search.phase == SEARCH_HOLD || lost_for >= LINE_SEARCH_TIMEOUT) ? LS_LOST : LS_SEARCHING;
    } else if (drive != LINE_WAIT && drive != LINE_LOST && usable && line_can_drive(status, owns)) {
        c->follow = drive;
        result = line_follow(drive, correction);
#ifdef ROBO_LINE_RUN_LIMIT_S
        if (c->run_start < 0.0) { c->run_start = now; }
#endif
    }
    line_set_state(&c->state, next);
    if (now >= c->next_report) {
        c->next_report = now + 0.5;
        line_report(centre, right, direction, c->state, c->search.phase, result, &status->motion);
    }
}

static void line_loop(void)
{
    static line_ctx_t ctx;
    line_ctx_t *c = &ctx;
    c->state = LS_STARTUP;
    c->last_side = LINE_STRAIGHT;
    c->follow = LINE_STRAIGHT;
#if ROBO_BARCODE_TASK
    c->bc_hold_from = -1.0;
    c->bc_lost_from = -1.0;
#endif
#ifdef ROBO_LINE_RUN_LIMIT_S
    c->run_start = -1.0;
#endif
#ifdef ROBO_LINE_LOSS_TEST
    c->on_line_since = -1.0;
    c->search_began = -1.0;
#ifdef ROBO_LINE_VEER_TEST
    c->veer_until = -1.0;
#endif
#endif
#if defined(ROBO_SUBSYSTEM4) && ROBO_SUBSYSTEM4
    line_wait_for_imu();
#endif
#if ROBO_BARCODE_TASK
    gpio_init(LINE_CAL_BUTTON);
    gpio_set_dir(LINE_CAL_BUTTON, false);
    gpio_pull_up(LINE_CAL_BUTTON);
    tm_printf((const UB *)"calibration: press GP21 any time to check the barcode sensor on *%c*\n",
              ROUTE_CALIBRATION_LETTER);
#endif
    line_set_state(&c->state, LS_WAITING);
    for (;;) {
#if ROBO_BARCODE_TASK
        if (c->halted) { /* Terminal STOPPED after a failed route action: zero-speed refreshes, no latched hold. */
            route_flag(false, false); /* Every pass: a lost clear must not keep subsystem 5 off. */
            (void)line_hold();
            (void)tk_dly_tsk(20U);
            continue;
        }
#endif
        unsigned right = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre = (unsigned)gpio_get(BOARD_LINE_C);
        /* Centre only black: robot drifted right, line is to the left -> LINE_LEFT.
         * Right only black: robot drifted left, line is to the right -> LINE_RIGHT. */
        line_direction_t direction = motion_line_direction(&c->filter, centre == 1U, right == 1U);
        robot_status_t status;
        double now = motion_hal_time();
        /* Raw, not debounced: a single centre-only sample before the loss still counts. */
        if (centre != right) { c->last_side = (centre == 1U) ? LINE_LEFT : LINE_RIGHT; }
        else if (centre == 1U) { c->last_side = LINE_STRAIGHT; }
        if (direction != LINE_LOST) { c->lost_since = now; }
#ifdef ROBO_LINE_LOSS_TEST
        if (direction == LINE_LOST || direction == LINE_WAIT) { c->on_line_since = -1.0; }
        else if (c->on_line_since < 0.0) { c->on_line_since = now; }
#ifdef ROBO_LINE_VEER_TEST
        if (!c->veered && c->on_line_since >= 0.0 && (now - c->on_line_since) >= LINE_TEST_FOLLOW_TIME) {
            c->veered = true;
            c->veer_until = now + LINE_TEST_VEER_TIME;
            tm_printf((const UB *)"line_test: veering off the line now\n");
        }
#endif
#endif
        line_publish(direction, now);
        if (motion_get_status(&status)) {
            bool owns = status.owner_valid && status.owner == SOURCE_LINE && status.motion.id == LINE_SESSION;
            if (line_barcode_events(c, &status, owns)) {
                line_drive(c, &status, owns, direction, centre, right, now);
            }
        }
        (void)tk_dly_tsk(20U);
    }
}
#endif

void line_task(INT code, void *context)
{
    (void)code; (void)context;
    ir_input_init(BOARD_LINE_R);
    ir_input_init(BOARD_LINE_C);
#ifdef ROBO_LINE_LOSS_TEST
    tm_printf((const UB *)"line_test: on-the-spot pivot search enabled\n");
#ifdef ROBO_LINE_VEER_TEST
    tm_printf((const UB *)"line_test: veers off the line after %u s on the line\n", (unsigned)LINE_TEST_FOLLOW_TIME);
#endif
#endif
    (void)tk_dly_tsk(2000U);
    line_loop();
}
#endif
