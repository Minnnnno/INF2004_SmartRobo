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

/* Plain digital input, pulls off (the IR modules drive the line). */
static void line_input_init(unsigned pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, false);
    gpio_disable_pulls(pin);
}

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
 * ADC channel = pin - 26: GP26 = ADC0, GP27 = ADC1, GP28 = ADC2. */
#define IRMON_ADC_BASE    0x4004C000U
#define IRMON_ADC_CS      (IRMON_ADC_BASE + 0x00U)
#define IRMON_ADC_RESULT  (IRMON_ADC_BASE + 0x04U)
#define IRMON_ADC_EN      (1UL << 0U)
#define IRMON_ADC_START   (1UL << 2U)
#define IRMON_ADC_READY   (1UL << 8U)
#define IRMON_ADC_AVERAGE 16U

static unsigned irmon_adc(unsigned pin)
{
    uint32_t sum = 0U, input = (uint32_t)(pin - 26U) << 12U;
    for (unsigned i = 0U; i < IRMON_ADC_AVERAGE; ++i) {
        out_w(IRMON_ADC_CS, IRMON_ADC_EN | input | IRMON_ADC_START);
        while ((in_w(IRMON_ADC_CS) & IRMON_ADC_READY) == 0U) { }
        sum += in_w(IRMON_ADC_RESULT) & 0xFFFU;
    }
    return (unsigned)(sum / IRMON_ADC_AVERAGE);
}

static void line_loop(void)
{
    clr_w(RESETS_RESET, RESETS_RESET_ADC);
    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_ADC) == 0U) { }
    out_w(GPIO_CTRL(BOARD_BARCODE), GPIO_CTRL_FUNCSEL_NULL);
    out_w(GPIO(BOARD_BARCODE), GPIO_OD); /* Digital input and pulls off for analog use. */
    out_w(IRMON_ADC_CS, IRMON_ADC_EN);
    while ((in_w(IRMON_ADC_CS) & IRMON_ADC_READY) == 0U) { }
    tm_printf((const UB *)"ir_monitor_analog: ADC 0..4095 (3.3 V); d = digital read of the same pin\n");
    for (;;) {
        unsigned right = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre = (unsigned)gpio_get(BOARD_LINE_C);
        tm_printf((const UB *)"centre GP%u d=%u a=%4u | right GP%u d=%u a=%4u | barcode GP%u a=%4u\n",
                  (unsigned)BOARD_LINE_C, centre, irmon_adc(BOARD_LINE_C),
                  (unsigned)BOARD_LINE_R, right, irmon_adc(BOARD_LINE_R),
                  (unsigned)BOARD_BARCODE, irmon_adc(BOARD_BARCODE));
        (void)tk_dly_tsk(200U);
    }
}
#else
static void line_loop(void)
{
    line_input_init(BOARD_BARCODE);
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
/* Line-tracking states (project brief / Week 6 statechart):
 *   INIT    line_task: pin setup and settle delay.
 *   WAIT    debounce gives LINE_WAIT until a stable reading; no request yet.
 *   FOLLOW  line_follow(): straight, or correct towards the side the line is on.
 *   GRACE   repeat the last follow command for LINE_GRACE_TIME after a loss.
 *   SEARCH  line_search_step(): sweep both sides, turn back, reverse, sweep again.
 *   HOLD    zero-speed refreshes after the search or LINE_SEARCH_TIMEOUT.
 * Not implemented yet: junction detection, barcode decode/actions (barcode.c),
 * and acknowledged handover back from obstacle avoidance. */
#include "steering.h"
#include "motion_hal.h"
#include "line_rejoin.h"
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
               SEARCH_REVERSE, SEARCH_HOLD, SEARCH_STOP, SEARCH_SPIN } line_search_phase_t;
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
    if (s->phase == SEARCH_HOLD || lost_for >= LINE_SEARCH_TIMEOUT) {
        return motion_steer(SOURCE_LINE, LINE_SESSION, 0.0, 0.0);
    }
    if (s->phase == SEARCH_REVERSE) { return motion_steer(SOURCE_LINE, LINE_SESSION, -LINE_BACKUP_SPEED, 0.0); }
#ifdef ROBO_LINE_LOSS_TEST
    if (s->phase == SEARCH_STOP) { return motion_steer(SOURCE_LINE, LINE_SESSION, 0.0, 0.0); }
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

/* Existing line-error/confidence slots, plus both-sensors-black for line_rejoin_seen().
 * No junction detector. */
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

/* Overall line state, printed as lstate=%u in the diagnostics. */
typedef enum { LS_FOLLOWING, LS_LOST, LS_SEARCHING, LS_HOLDING } line_state_t;

static void line_report(unsigned centre, unsigned right, line_direction_t direction, line_state_t lstate,
                        line_search_phase_t phase, motion_result_t result, const motion_status_t *m)
{
    tm_printf((const UB *)"line_pid centre_ir=%u right_ir=%u direction=%u lstate=%u sweep=%u request=%u state=%u fault=%u speed_mm_s=%d,%d output_permille=%d,%d target_mm_s=%d,%d\n",
        centre, right, (unsigned)direction, (unsigned)lstate, (unsigned)phase, (unsigned)result,
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

/* ---- Junction: the barcode task reports the first black after a valid read (barcode.c,
 * SOURCE_BARCODE values[2] counts them). The line task only logs it and keeps following; the route
 * action for each letter is not implemented yet. ---- */

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

static void line_loop(void)
{
    unsigned seen_junctions = 0U, seen_failures = 0U;
    bool junction_baselined = false;
    line_steering_t filter = {0};
    line_search_t search = {0};
    line_direction_t last_side = LINE_STRAIGHT; /* Raw side the line was last seen on; STRAIGHT = unknown. */
    line_direction_t follow = LINE_STRAIGHT;    /* Last follow command, repeated during the grace time. */
    double lost_since = 0.0;
    double next_report = 0.0;
#if defined(ROBO_BARCODE_TASK) && ROBO_BARCODE_TASK
    double bc_hold_from = -1.0;                       /* Path (m) where the barcode pass began, see LINE_BC_*. */
    double bc_lost_from = -1.0;                       /* Path (m) where the line was lost during that pass. */
    double bc_heading_from = 0.0;                     /* Heading (rad) at the start of that pass. */
#endif
#ifdef ROBO_LINE_RUN_LIMIT_S
    double run_start = -1.0; /* First follow command; the car is held still ROBO_LINE_RUN_LIMIT_S later. */
    bool run_done = false;
#endif
#ifdef ROBO_LINE_LOSS_TEST
    double on_line_since = -1.0; /* Start of the current stretch with the line seen. */
    double search_began = -1.0;  /* When the current search started; glimpses do not reset it. */
#ifdef ROBO_LINE_VEER_TEST
    double veer_until = -1.0;
    bool veered = false;
#endif
#endif
#if defined(ROBO_SUBSYSTEM4) && ROBO_SUBSYSTEM4
    line_wait_for_imu();
#endif
    for (;;) {
        unsigned right = (unsigned)gpio_get(BOARD_LINE_R);
        unsigned centre = (unsigned)gpio_get(BOARD_LINE_C);
        /* Centre only black: robot drifted right, line is to the left -> LINE_LEFT.
         * Right only black: robot drifted left, line is to the right -> LINE_RIGHT. */
        line_direction_t direction = motion_line_direction(&filter, centre == 1U, right == 1U);
        robot_status_t status;
        motion_result_t result = R_HELD;
        line_state_t lstate = LS_HOLDING; /* Not driving: held, faulted or avoiding. */
        double now = motion_hal_time();
        /* Raw, not debounced: a single centre-only sample before the loss still counts. */
        if (centre != right) { last_side = (centre == 1U) ? LINE_LEFT : LINE_RIGHT; }
        else if (centre == 1U) { last_side = LINE_STRAIGHT; }
        if (direction != LINE_LOST) { lost_since = now; }
#ifdef ROBO_LINE_LOSS_TEST
        if (direction == LINE_LOST || direction == LINE_WAIT) { on_line_since = -1.0; }
        else if (on_line_since < 0.0) { on_line_since = now; }
#ifdef ROBO_LINE_VEER_TEST
        if (!veered && on_line_since >= 0.0 && (now - on_line_since) >= LINE_TEST_FOLLOW_TIME) {
            veered = true;
            veer_until = now + LINE_TEST_VEER_TIME;
            tm_printf((const UB *)"line_test: veering off the line now\n");
        }
#endif
#endif
        line_publish(direction, now);
        if (motion_get_status(&status)) {
            bool owns = status.owner_valid && status.owner == SOURCE_LINE && status.motion.id == LINE_SESSION;
            unsigned junctions = (unsigned)status.barcode.values[2];
            unsigned failures = (unsigned)status.barcode.values[3];
            if (!junction_baselined) {
                seen_junctions = junctions; /* Whatever was published before now cannot trigger a turn. */
                seen_failures = failures;
                junction_baselined = true;
            } else if (failures != seen_failures) {
                /* The barcode task could not decode a pass: stop right there. The hold stays until power-cycle
                 * (this image has no Wi-Fi to release it); the junction's own bar is never counted (barcode.c). */
                seen_failures = failures;
                motion_stop();
                tm_printf((const UB *)"route: barcode read failed, car stopped (power-cycle to rerun)\n");
                (void)tk_dly_tsk(20U);
                continue;
            } else if (junctions != seen_junctions) {
                seen_junctions = junctions;
                tm_printf((const UB *)"route: junction after %c, path %d mm (no action, following on)\n",
                          (char)(unsigned)status.barcode.values[0], (int)(status.motion.path * 1000.0));
            }
            /* Subsystem 5 drives round obstacles itself; keep off the wheels until it
             * publishes that it is back on the line. Exact compare: the value is copied. */
            bool avoiding = status.obstacle.values[3] == LINE_OBSTACLE_AVOIDING;
            bool usable = !status.motion.hold && status.motion.fault == F_NONE && !avoiding;
            line_direction_t drive = direction;
            bool grace_ok = true;
            bool bc_hold = false;
            double correction = LINE_CORRECTION;
#if defined(ROBO_BARCODE_TASK) && ROBO_BARCODE_TASK
            if (barcode_code_pass()) {
                if (bc_hold_from < 0.0) {
                    bc_hold_from = status.motion.path;
                    bc_heading_from = status.motion.heading;
                }
            } else {
                bc_hold_from = -1.0; /* Pass over, or tape / big code seen: steer normally at once. */
            }
            bc_hold = bc_hold_from >= 0.0 &&
                      (status.motion.path - bc_hold_from) < LINE_BC_HOLD_MAX && search.phase == SEARCH_IDLE &&
                      status.motion.heading - bc_heading_from < LINE_BC_MAX_TURN &&
                      bc_heading_from - status.motion.heading < LINE_BC_MAX_TURN;
            if (bc_hold) { correction = LINE_BC_CORRECTION; }
            if (direction != LINE_LOST) { bc_lost_from = -1.0; }
            else if (bc_hold && bc_lost_from < 0.0) { bc_lost_from = status.motion.path; }
            bc_hold = bc_hold && direction == LINE_LOST &&
                      (status.motion.path - bc_lost_from) < LINE_BC_HOLD_LOST;
#endif
#ifdef ROBO_LINE_LOSS_TEST
            /* A glimpse of tape during a search is not "found": keep the 360 deg budget and the
             * search clock, and stand still until the tape has stayed in view for
             * LINE_REACQUIRE_TIME. No blind grace drive while searching. */
            bool settled = on_line_since >= 0.0 && (now - on_line_since) >= LINE_REACQUIRE_TIME;
            bool glimpse = direction != LINE_LOST && direction != LINE_WAIT &&
                           search.phase != SEARCH_IDLE && !settled;
            grace_ok = search.phase == SEARCH_IDLE;
#else
            const bool glimpse = false;
#endif
            if (direction != LINE_LOST && !glimpse) {
                search.seen_heading = status.motion.heading;
                search.seen_centre = status.motion.centre;
                search.phase = SEARCH_IDLE;
                search.backed = false;
                lstate = LS_FOLLOWING;
            }
            else if (direction == LINE_LOST && grace_ok && !bc_hold && owns && (now - lost_since) < LINE_GRACE_TIME) {
                drive = follow; /* Bridge short dropouts. */
                lstate = LS_LOST;
            }
#ifdef ROBO_LINE_RUN_LIMIT_S
            /* Analysis builds: one run of ROBO_LINE_RUN_LIMIT_S seconds, then stand still (reset to rerun). */
            if (!run_done && run_start >= 0.0 && (now - run_start) >= (double)ROBO_LINE_RUN_LIMIT_S) {
                run_done = true;
                tm_printf((const UB *)"line_test: run limit reached, car stopped\n");
            }
            if (run_done) {
                if (usable && (status.motion.state == READY || (status.motion.state == DRIVING && owns))) {
                    result = motion_steer(SOURCE_LINE, LINE_SESSION, 0.0, 0.0);
                }
                lstate = LS_HOLDING;
            } else
#endif
#ifdef ROBO_LINE_LOSS_TEST
#ifdef ROBO_LINE_VEER_TEST
            if (now < veer_until && usable &&
                (status.motion.state == READY || (status.motion.state == DRIVING && owns))) {
                result = motion_steer(SOURCE_LINE, LINE_SESSION, LINE_SPEED, LINE_TEST_VEER_RATE);
            } else
#endif
            if (glimpse && owns && usable && status.motion.state == DRIVING) {
                result = motion_steer(SOURCE_LINE, LINE_SESSION, 0.0, 0.0);
                lstate = LS_SEARCHING;
            } else
#endif
            if (bc_hold && drive == LINE_LOST && owns && usable && status.motion.state == DRIVING) {
                double turn = LINE_BC_HOLD_GAIN * (search.seen_heading - status.motion.heading);
                if (turn > LINE_BC_HOLD_RATE) { turn = LINE_BC_HOLD_RATE; }
                if (turn < -LINE_BC_HOLD_RATE) { turn = -LINE_BC_HOLD_RATE; }
                result = motion_steer(SOURCE_LINE, LINE_SESSION, LINE_SPEED, turn);
                lost_since = now; /* Grace and search start afresh if the line is still gone after the hold. */
                lstate = LS_LOST;
            } else if (drive == LINE_LOST && owns && usable && status.motion.state == DRIVING) {
                double lost_for = now - lost_since;
#ifdef ROBO_LINE_LOSS_TEST
                if (search.phase == SEARCH_IDLE) { search_began = lost_since; }
                lost_for = now - search_began; /* Glimpses must not restart the clock. */
#endif
                result = line_search_step(&search, &status.motion, last_side, lost_for);
                lstate = (search.phase == SEARCH_HOLD || lost_for >= LINE_SEARCH_TIMEOUT)
                             ? LS_HOLDING : LS_SEARCHING;
            } else if (drive != LINE_WAIT && drive != LINE_LOST && usable &&
                       (status.motion.state == READY || (status.motion.state == DRIVING && owns))) {
                follow = drive;
                result = line_follow(drive, correction);
#ifdef ROBO_LINE_RUN_LIMIT_S
                if (run_start < 0.0) { run_start = now; }
#endif
            }
            if (now >= next_report) {
                next_report = now + 0.5;
                line_report(centre, right, direction, lstate, search.phase, result, &status.motion);
            }
        }
        (void)tk_dly_tsk(20U);
    }
}
#endif

void line_task(INT code, void *context)
{
    (void)code; (void)context;
    line_input_init(BOARD_LINE_R);
    line_input_init(BOARD_LINE_C);
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
