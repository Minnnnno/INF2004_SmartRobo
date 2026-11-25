/* Exercise the real task and detour code with a deterministic motor/sensor rig.
 * Including these two sources keeps the RTOS/register stubs out of firmware. */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <setjmp.h>

#define ROBO_RTOS_H
typedef int32_t INT;
typedef int32_t ER;
typedef uint32_t UW;
typedef uint32_t UINT;
typedef uint32_t RELTIM;
typedef uint8_t UB;
#define DI(state) ((state) = 0U)
#define EI(state) ((void)(state))
ER tk_dly_tsk(RELTIM delay);

#define COMMS_H
#include "../../app_program/subsystem1/comms_hooks.h"
#include "telemetry_frame.h"
bool comms_is_online(void);
void comms_latch_event(UB events);
ER comms_send_telemetry(const char *name, const char *payload);
#define ROBO_CRASH_LOG_H
#define CRASH_MARK_OBSTACLE 0U
enum { CRASH_OBSTACLE_PING, CRASH_OBSTACLE_DECIDE, CRASH_OBSTACLE_STOP,
       CRASH_OBSTACLE_SETTLE, CRASH_OBSTACLE_SCAN, CRASH_OBSTACLE_REPORT,
       CRASH_OBSTACLE_DETOUR, CRASH_OBSTACLE_OUT, CRASH_OBSTACLE_PASS,
       CRASH_OBSTACLE_REAR_CLEAR, CRASH_OBSTACLE_RETURN_TURN,
       CRASH_OBSTACLE_RETURN_LOOK, CRASH_OBSTACLE_RETURN_DRIVE, CRASH_OBSTACLE_ALIGN };
static void crash_mark(UW mark, UW value) { (void)mark; (void)value; }

#include "../../app_program/subsystem5/detour.c"
#include "../../app_program/subsystem5/subsystem5.c"

static struct {
    robot_status_t status;
    encoder_sample_t encoders;
    char encoder_log[4096];
    size_t encoder_log_length;
    double now, stop_at;
    s5_us_status_t side_result, front_result;
    bool angle_override[181];
    s5_us_status_t angle_result[181];
    double angle_range[181];
    unsigned angle_reads[181], override_after_distance, look_action;
    double largest_distance, distance_scale;
    unsigned moves, pings, servo_moves, releases, distance_calls, turn_calls;
    unsigned busy_distance_call, busy_turn_call, retry_action;
    int angle;
    bool status_missing, refuse_stop, release_during_servo, release_during_ping;
    bool online, always_busy, lose_status_after_completion, detect_line_on_next_distance;
    bool lose_line_on_stop, line_back_on_reverse;
    unsigned both_black_on_distance_call, one_black_on_distance_call;
    double clear_line_at_s;
    double drive_left, drive_speed; /* A long move runs in time, not in one call. */
    double fault_at_s;
    uint32_t drive_id;
    unsigned moving_pings, stops;
    double stall_after_m, drive_start_centre; /* a long move stops turning the wheels this far in ... */
    unsigned stalls_left;                      /* ... this many times (until motion_stop()) */
    double first_turn_deg;
    bool yaw_live;      /* the yawed face follows the robot's turns */
    double yaw_now;
    unsigned stop_at_turn; /* end the run when this turn is asked for */
    double stopped_turn_deg;
    double block_h_min, block_h_max; /* rad: echoes override only while |heading| is in this range */
    unsigned stop_lag_ticks, lag_left; /* motion_stop() reaches STOPPED only after this many delays. */
    unsigned drive_pings;  /* Clear pings while line following, before the obstacle. */
    double heading_per_ping; /* rad */
    unsigned events, oversized_formats;
    char messages[2][COMMS_PAYLOAD_MAX];
    char all_messages[2048]; /* every telemetry payload, one per line */
    UB latched_events;
    double turn_log[64];     /* every turn asked for, degrees */
    unsigned turn_log_count;
    unsigned block_turns_max; /* angle overrides apply only up to this many turns (0: always) */
    bool vanish_on_stop;     /* the front echo goes when the robot is stopped ... */
    bool reappear_on_release; /* ... and comes back when it is released */
    bool angle_scatter[181]; /* overridden pings here: every other one silent, the rest 30 mm apart */
    unsigned scatter_reads;
    unsigned black_calls;    /* bit n: both sensors black from distance call n ... */
    unsigned clear_calls;    /* ... bit n: line gone from distance call n */
    unsigned line_partial_call; /* this distance call stops on the line part way, like detect_line_on_next_distance */
    double drive_dir;        /* +1 or -1: a long move forward or in reverse */
    double last_metres;      /* the last straight move asked for */
    bool black_band;         /* both sensors black while the centre travel is in ... */
    double black_from_m, black_to_m; /* ... [from, to]: a line crossed part way through a long move */
} rig;
static jmp_buf task_end;
static unsigned checks, failures;
#define CHECK(test) do { ++checks; if (!(test)) { ++failures; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #test); } } while (0)

static void reset_rig(void)
{
    memset(&rig, 0, sizeof(rig));
    rig.status.motion.state = STOPPED;
    rig.status.motion.hold = true;
    rig.status.motion.stationary = true;
    rig.status.line.available = true;
    rig.encoders.reliable = true;
    rig.angle = S5_ANGLE_CENTRE_DEG;
    rig.front_result = S5_US_NO_ECHO;
    rig.side_result = S5_US_NO_ECHO;
    rig.distance_scale = 1.0;
    s5_task.blocked = false;
    s5_task.decided = false;
    s5_task.sensor_ok = false;
    s5_task.phase = S5_PHASE_IDLE;
    s5_known = NULL;            /* a test that longjmps out mid-detour leaves it set */
    s5_leg.out_blocked = false;
    s5_task.spurious_count = 0U;
    s5_spurious_path_m = 0.0;
    s5_leg.line_noted = false;
    s5_leg.line_goal = false;
    s5_detour_reason_clear();
}

ER tk_dly_tsk(RELTIM delay)
{
    rig.now += (double)delay / 1000.0;
    rig.status.line.timestamp = rig.now;
    if (rig.fault_at_s > 0.0 && rig.now >= rig.fault_at_s) { rig.status.motion.fault = F_FEEDBACK; }
    if (rig.clear_line_at_s > 0.0 && rig.now >= rig.clear_line_at_s) {
        rig.clear_line_at_s = 0.0;
        rig.status.line.values[1] = 0.0;
    }
    if (rig.black_band) {
        double on = (rig.status.motion.centre >= rig.black_from_m &&
                     rig.status.motion.centre <= rig.black_to_m) ? 1.0 : 0.0;
        rig.status.line.values[1] = on;
        rig.status.line.values[LINE_VALUE_BOTH_BLACK] = on;
    }
    if (rig.drive_left > 0.0 && rig.stalls_left > 0U &&
        rig.status.motion.centre - rig.drive_start_centre >= rig.stall_after_m) {
        rig.status.motion.speed[0] = rig.status.motion.speed[1] = 0.0;
        rig.status.motion.target[0] = rig.status.motion.target[1] = 0.08;
    } else if (rig.drive_left > 0.0) {
        double step = fmin(rig.drive_left, rig.drive_speed * (double)delay / 1000.0);
        rig.status.motion.centre += (rig.drive_dir < 0.0 ? -1.0 : 1.0) * step;
        rig.drive_left -= step;
        if (rig.drive_left <= 0.0) {
            rig.status.last_event = (motion_event_t){EV_COMPLETE, rig.drive_id, 0U, F_NONE};
            rig.status.motion.state = READY;
            rig.status.motion.stationary = true;
        }
    }
    if (rig.retry_action != 0U && rig.turn_calls == 1U && rig.busy_turn_call == 1U) {
        if (rig.retry_action == 1U) { rig.status.motion.hold = true; }
        if (rig.retry_action == 2U) { ++rig.status.motion.generation; }
        if (rig.retry_action == 3U) { rig.status.motion.fault = F_FEEDBACK; }
        if (rig.retry_action == 4U) { rig.status.motion.state = DRIVING; }
        rig.retry_action = 0U;
    }
    if (rig.lag_left > 0U && --rig.lag_left == 0U && rig.status.motion.hold) {
        rig.status.motion.state = STOPPED;
    }
    if (rig.stop_at > 0.0 && rig.now >= rig.stop_at) { longjmp(task_end, 1); }
    return 0;
}
double motion_hal_time(void) { return rig.now; }
encoder_sample_t motion_hal_sample(void) { return rig.encoders; }
bool motion_get_status(robot_status_t *status)
{
    if (rig.status_missing) { return false; }
    *status = rig.status;
    if (rig.lose_status_after_completion && rig.status.last_event.id != 0U &&
        rig.status.last_event.type == EV_COMPLETE) {
        rig.status_missing = true;
    }
    return true;
}
void motion_stop(void)
{
    ++rig.stops;
    if (rig.status.motion.target[0] > 0.0) {
        rig.status.motion.target[0] = rig.status.motion.target[1] = 0.0;
        if (rig.stalls_left > 0U) { --rig.stalls_left; }
    }
    rig.drive_left = 0.0;
    if (rig.vanish_on_stop) { rig.front_result = S5_US_NO_ECHO; }
    rig.status.motion.hold = true;
    rig.status.motion.stationary = !rig.refuse_stop;
    rig.status.motion.state = rig.refuse_stop ? STOPPING : STOPPED;
    if (rig.stop_lag_ticks > 0U && !rig.refuse_stop) {
        rig.status.motion.state = READY;
        rig.lag_left = rig.stop_lag_ticks;
    }
    if (rig.lose_line_on_stop) {
        rig.status.line.values[LINE_VALUE_BOTH_BLACK] = 0.0;
        rig.status.line.values[1] = 0.0;
        rig.status.motion.centre += 0.015; /* Braked past the line. */
    }
}
void motion_controller_release(bool checks_ok)
{
    ++rig.releases;
    if (checks_ok && rig.status.motion.stationary) {
        rig.status.motion.hold = false;
        rig.status.motion.state = READY;
        if (rig.reappear_on_release) { rig.front_result = S5_US_OK; }
    }
}
static motion_result_t finish_move(uint32_t id, double metres, double degrees)
{
    ++rig.moves;
    rig.status.motion.id = id;
    rig.status.motion.centre += metres * rig.distance_scale;
    rig.status.motion.heading += degrees * S5_DEG_TO_RAD;
    for (unsigned wheel = 0U; wheel < 2U; ++wheel) {
        double travel = metres * rig.distance_scale + (wheel == 0U ? -1.0 : 1.0) *
                        degrees * S5_DEG_TO_RAD * 0.115 / 2.0;
        rig.encoders.count[wheel] += (uint32_t)(int32_t)round(travel * 10000.0);
        rig.status.motion.travel[wheel] += travel;
    }
    rig.status.last_event = (motion_event_t){EV_COMPLETE, id, 0U, F_NONE};
    return R_OK;
}
motion_result_t motion_steer_distance(request_source_t source, uint32_t id,
                                     double metres, double degrees, double speed)
{
    (void)source;
    ++rig.distance_calls;
    if (fabs(metres) > rig.largest_distance) { rig.largest_distance = fabs(metres); }
    rig.last_metres = metres;
    if (rig.always_busy || rig.distance_calls == rig.busy_distance_call) { return R_BUSY; }
    if (rig.distance_calls < 32U && (rig.clear_calls & (1U << rig.distance_calls)) != 0U) {
        rig.status.line.values[1] = 0.0;
        rig.status.line.values[LINE_VALUE_BOTH_BLACK] = 0.0;
    }
    if (rig.distance_calls < 32U && (rig.black_calls & (1U << rig.distance_calls)) != 0U) {
        rig.status.line.values[1] = 1.0;
        rig.status.line.values[LINE_VALUE_BOTH_BLACK] = 1.0;
    }
    if (rig.distance_calls == rig.both_black_on_distance_call) {
        rig.status.line.values[1] = 1.0;
        rig.status.line.values[LINE_VALUE_BOTH_BLACK] = 1.0;
    }
    if (rig.distance_calls == rig.one_black_on_distance_call) { rig.status.line.values[1] = 1.0; }
    if (rig.line_back_on_reverse && metres < 0.0) { rig.status.line.values[1] = 1.0; }
    if ((fabs(metres) >= 0.9) || (rig.distance_calls == rig.one_black_on_distance_call)) {
        /* Only the pass is this long; crossing a line takes time too. */
        ++rig.moves;
        rig.drive_left = fabs(metres);
        rig.drive_dir = metres < 0.0 ? -1.0 : 1.0;
        rig.drive_start_centre = rig.status.motion.centre;
        rig.drive_speed = speed;
        rig.drive_id = id;
        rig.status.motion.id = id;
        rig.status.motion.state = DRIVING;
        rig.status.motion.stationary = false;
        return R_OK;
    }
    if (rig.detect_line_on_next_distance || rig.distance_calls == rig.line_partial_call) {
        rig.detect_line_on_next_distance = false;
        /* Starting 190 mm to the side at 45 deg, the front IR reaches the
         * line after about 188.7 mm. The requested leg has not completed. */
        (void)finish_move(id, 0.1887, 0.0);
        rig.status.last_event.id = 0U;
        rig.status.motion.state = DRIVING;
        rig.status.motion.stationary = false;
        rig.status.line.values[1] = 1.0;
        rig.status.line.values[LINE_VALUE_BOTH_BLACK] = 1.0;
        return R_OK;
    }
    return finish_move(id, metres, degrees);
}
static void yawed_face(double yaw_deg);
motion_result_t motion_turn_degrees(request_source_t source, uint32_t id, double degrees)
{
    (void)source;
    ++rig.turn_calls;
    if (rig.always_busy || rig.turn_calls == rig.busy_turn_call) { return R_BUSY; }
    if (rig.turn_log_count < sizeof(rig.turn_log) / sizeof(rig.turn_log[0])) {
        rig.turn_log[rig.turn_log_count++] = degrees;
    }
    if (rig.moves == 0U) { rig.first_turn_deg = degrees; }
    if (rig.turn_calls == rig.stop_at_turn) {
        rig.stopped_turn_deg = degrees;
        longjmp(task_end, 1);
    }
    if (rig.yaw_live && fabs(rig.yaw_now - degrees) <= 45.0) {
        rig.yaw_now -= degrees;
        yawed_face(rig.yaw_now);
    }
    return finish_move(id, 0.0, degrees);
}
bool robot_update_subsystem(request_source_t source, const subsystem_status_t *status)
{
    (void)source;
    rig.status.obstacle = *status;
    return true;
}
void s5_servo_init(int angle) { rig.angle = angle; }
void s5_servo_move(int angle)
{
    rig.angle = angle;
    ++rig.servo_moves;
    (void)tk_dly_tsk(100U);
    if (rig.release_during_servo) {
        rig.status.motion.hold = false;
        rig.status.motion.stationary = false;
        rig.status.motion.state = DRIVING;
    }
    if (rig.look_action == 1U) { motion_stop(); }
    if (rig.look_action == 2U) { ++rig.status.motion.generation; }
    if (rig.look_action == 3U) { rig.status.motion.stationary = false; rig.status.motion.travel[0] += 0.01; }
    if (rig.look_action == 5U) { rig.status.motion.stationary = false; rig.status.motion.travel[0] += 0.0001; }
    if (rig.look_action == 4U) { rig.status.motion.fault = F_FEEDBACK; }
    rig.look_action = 0U;
}
unsigned s5_servo_aim(int angle)
{
    rig.angle = angle;
    ++rig.servo_moves;
    return 380U;
}
int s5_servo_angle(void) { return rig.angle; }
bool s5_us_init(void) { return true; }
s5_us_status_t s5_us_ping(double *range, double *time)
{
    ++rig.pings;
    ++rig.angle_reads[rig.angle];
    if (rig.status.motion.state == DRIVING) { ++rig.moving_pings; }
    (void)tk_dly_tsk(S5_US_CYCLE_MS);
    *time = rig.now;
    *range = 100.0;
    if (rig.drive_pings > 0U) {
        --rig.drive_pings;
        rig.status.motion.path += 0.006;
        rig.status.motion.heading += rig.heading_per_ping;
        return S5_US_NO_ECHO;
    }
    if (rig.release_during_ping) {
        rig.status.motion.hold = false;
        rig.status.motion.stationary = false;
        rig.status.motion.state = DRIVING;
    }
    if (rig.angle_override[rig.angle] && rig.distance_calls >= rig.override_after_distance &&
        (rig.block_turns_max == 0U || rig.turn_calls <= rig.block_turns_max) &&
        (rig.block_h_max == 0.0 || (fabs(rig.status.motion.heading) > rig.block_h_min &&
                                    fabs(rig.status.motion.heading) < rig.block_h_max))) {
        if (rig.angle_scatter[rig.angle]) {
            /* 5 of 10 pings echo, at base +0, 30, 60, 90, 120 mm: no 3 agree */
            unsigned k = rig.scatter_reads++ % 10U;
            if ((k % 2U) != 0U) { return S5_US_NO_ECHO; }
            *range = rig.angle_range[rig.angle] + 30.0 * (double)(k / 2U);
            return S5_US_OK;
        }
        *range = rig.angle_range[rig.angle];
        return rig.angle_result[rig.angle];
    }
    return rig.angle == S5_ANGLE_CENTRE_DEG ? rig.front_result : rig.side_result;
}
bool comms_is_online(void) { return rig.online; }
void comms_latch_event(UB events) { rig.latched_events = (UB)(rig.latched_events | events); }
ER comms_send_telemetry(const char *name, const char *payload)
{
    CHECK(strcmp(name, "obstacle") == 0);
    CHECK(strlen(payload) < COMMS_PAYLOAD_MAX);
    if (rig.events < 2U) {
        (void)snprintf(rig.messages[rig.events], COMMS_PAYLOAD_MAX, "%s", payload);
    }
    {
        size_t used = strlen(rig.all_messages);
        (void)snprintf(rig.all_messages + used, sizeof(rig.all_messages) - used, "%s\n", payload);
    }
    ++rig.events;
    return 0;
}
INT tm_printf(const UB *format, ...)
{
    va_list args;
    int length = 0;
    if (strstr((const char *)format, " enc ") == NULL) { return 0; }
    size_t room = sizeof(rig.encoder_log) - rig.encoder_log_length;
    va_start(args, format);
    length = vsnprintf(rig.encoder_log + rig.encoder_log_length, room, (const char *)format, args);
    va_end(args);
    if (length > 0) {
        rig.encoder_log_length += (size_t)length < room ? (size_t)length : room - 1U;
    }
    return length;
}
INT tm_sprintf(UB *buffer, const UB *format, ...)
{
    int result;
    va_list args;
    va_start(args, format);
    result = vsnprintf((char *)buffer, COMMS_PAYLOAD_MAX, (const char *)format, args);
    va_end(args);
    /* Detect the real tm_sprintf's unbounded write without corrupting the rig. */
    if (result >= COMMS_PAYLOAD_MAX) { ++rig.oversized_formats; }
    return result;
}

INT tm_snprintf(UB *buffer, INT capacity, const UB *format, ...)
{
    int result;
    va_list args;
    va_start(args, format);
    result = vsnprintf((char *)buffer, (size_t)capacity, (const char *)format, args);
    va_end(args);
    return result;
}

static s5_point_t centre_point(double range_mm)
{
    s5_estimate_t e = {S5_ECHO_OK, range_mm, 5U, 5U, 5U, range_mm, range_mm};
    return s5_point_pack(S5_ANGLE_CENTRE_DEG, &e, true);
}

static void report_buffer_test(void)
{
    static s5_scan_t scan;
    reset_rig();
    rig.online = true;
    memset(&scan, 0, sizeof(scan));
    scan.b_valid = true;
    scan.width_mm = 160.0;
    scan.offset_mm = -15.0;
    scan.centre = centre_point(153.0);
    scan.edge[S5_LEFT].b_found = scan.edge[S5_RIGHT].b_found = true;
    scan.placement.position = S5_POSITION_CENTRE;
    scan.avoid.side = S5_AVOID_RIGHT;
    scan.avoid.shift_mm[S5_LEFT] = 180.0;
    scan.avoid.shift_mm[S5_RIGHT] = 210.0;
    report_scan(&scan);
    CHECK(rig.oversized_formats == 0U);
    CHECK(rig.events == 2U);
    CHECK(rig.latched_events == (TF_EV_OBSTACLE | TF_EV_SCAN_DONE));
    CHECK(strstr(rig.messages[0], "width_mm=160") != NULL);
    CHECK(strstr(rig.messages[1], "avoid=right") != NULL);

    reset_rig(); /* An offline scan still latches the binary event for reconnect. */
    report_scan(&scan);
    CHECK(rig.events == 0U);
    CHECK(rig.latched_events == (TF_EV_OBSTACLE | TF_EV_SCAN_DONE));

    reset_rig();
    scan.b_valid = false;
    report_scan(&scan);
    CHECK(rig.events == 0U && rig.latched_events == 0U);
}

static const s5_detour_plan_t plan = {
    .b_ok = true, .side = S5_AVOID_RIGHT, .shift_mm = -190.0,
    .edge_mm = -75.0, .face_mm = 240.0, .turn_deg = -60.0,
    .diagonal_mm = 220.0, .out_x_mm = 110.0
};

static void detour_sensor_tests(void)
{
    const s5_us_status_t faults[] = {S5_US_NO_PULSE, S5_US_BUSY};
    for (unsigned i = 0U; i < sizeof(faults) / sizeof(faults[0]); ++i) {
        reset_rig();
        rig.side_result = faults[i];
        CHECK(!s5_detour_run(&plan));
        CHECK(rig.moves == 1U && rig.distance_calls == 0U); /* Guard fails before diagonal travel. */

        reset_rig();
        rig.front_result = faults[i];
        CHECK(!s5_detour_run(&plan));
        CHECK(rig.moves == 1U && rig.distance_calls == 0U);
    }
    reset_rig();
    CHECK(!s5_detour_run(&plan)); /* No line in this rig: return eventually fails. */
    CHECK(rig.moves > 3U);       /* A normal no-echo still permits passing. */

    reset_rig();
    rig.status.motion.hold = false;
    rig.status.motion.state = READY;
    rig.front_result = S5_US_OK;
    /* Raw 100 mm = 130 mm from the axis. Required room is travel + 90 mm. */
    CHECK(path_clear(39.0));
    CHECK(!path_clear(40.0));
}

static void ready_rig(void)
{
    reset_rig();
    rig.status.motion.hold = false;
    rig.status.motion.state = READY;
    s5_heading0 = s5_x = s5_y = 0.0;
}

static void echo_at(int angle, double range)
{
    rig.angle_override[angle] = true;
    rig.angle_result[angle] = S5_US_OK;
    rig.angle_range[angle] = range;
}

/* The pings at this angle disagree, nearest at range */
static void scatter_at(int angle, double range)
{
    echo_at(angle, range);
    rig.angle_scatter[angle] = true;
}

static void encoder_logging_tests(void)
{
    ready_rig();
    s5_move_id = S5_ID_LAST;
    rig.encoders.count[0] = UINT32_MAX - 4U;
    rig.encoders.count[1] = 100U;
    CHECK(straight(10.0, false, NULL) == S5_MOVE_DONE);
    CHECK(strstr(rig.encoder_log, "phase=move-before id=5100 L=4294967291 R=100") != NULL);
    CHECK(strstr(rig.encoder_log, "phase=move-after id=5100 L=95 R=200") != NULL);
    CHECK(strstr(rig.encoder_log, "delta_L=100 delta_R=100 result=done") != NULL);

    ready_rig();
    rig.distance_scale = 0.0;
    CHECK(straight(5.0, false, NULL) == S5_MOVE_DONE);
    CHECK(strstr(rig.encoder_log, "delta_L=0 delta_R=0 result=done") != NULL);

    ready_rig();
    CHECK(turn_to(-45.0) == S5_MOVE_DONE);
    CHECK(strstr(rig.encoder_log, "delta_L=452 delta_R=-452 result=done") != NULL);

    ready_rig();
    rig.always_busy = true;
    CHECK(straight(50.0, false, NULL) == S5_MOVE_FAILED);
    CHECK(strstr(rig.encoder_log, "delta_L=0 delta_R=0 result=failed") != NULL);

    ready_rig();
    (void)s5_log_encoders("scan-before", 0U);
    (void)s5_log_encoders("scan-after", 0U);
    CHECK(strstr(rig.encoder_log, "phase=scan-before id=0") != NULL);
    CHECK(strstr(rig.encoder_log, "phase=scan-after id=0") != NULL);
}

/* Steps of S5_OUT_RETRY_STEP_DEG from a 60 deg turn up to S5_OUT_RETRY_MAX_DEG */
#define RETRIES ((unsigned)((S5_OUT_RETRY_MAX_DEG - 60.0) / S5_OUT_RETRY_STEP_DEG + 0.5))

static void steeper_turn_tests(void)
{
    for (unsigned side = 0U; side < 2U; ++side) {
        s5_detour_plan_t p = plan;
        double sign = side == 0U ? -1.0 : 1.0;
        p.side = side == 0U ? S5_AVOID_RIGHT : S5_AVOID_LEFT;
        p.turn_deg = sign * 60.0;
        p.shift_mm = sign * 190.0;

        /* Blocked at 60 deg, clear at 70: turns out further, same shift, carries on. */
        ready_rig();
        echo_at(60, 165.0);
        rig.block_h_min = 10.0 * S5_DEG_TO_RAD;
        rig.block_h_max = 65.0 * S5_DEG_TO_RAD;
        CHECK(go_out(&p));
        CHECK(rig.turn_calls == 3U && rig.distance_calls == 1U); /* out, further out, parallel */
        CHECK(fabs(rig.status.motion.heading) < 1e-9);
        CHECK(fabs(s5_y - sign * (p.diagonal_mm + S5_DIAGONAL_EXTRA_MM) * sin(60.0 * S5_DEG_TO_RAD)) < 0.5);

        /* Blocked all the way to straight sideways: gives up there. */
        ready_rig();
        echo_at(60, 165.0);
        rig.block_h_min = 10.0 * S5_DEG_TO_RAD;
        rig.block_h_max = 100.0 * S5_DEG_TO_RAD;
        CHECK(!go_out(&p));
        CHECK(rig.turn_calls == 1U + RETRIES && rig.distance_calls == 0U);
        CHECK(fabs(rig.status.motion.heading - sign * S5_OUT_RETRY_MAX_DEG * S5_DEG_TO_RAD) < 1e-9);

        /* A sensor fault or lost hold is not an obstruction: no further turning. */
        ready_rig();
        rig.look_action = 1U;
        CHECK(!go_out(&p));
        CHECK(rig.turn_calls == 1U && rig.distance_calls == 0U);
    }
}

static void clearance_tests(void)
{
    /* One encoder count of wobble clears "stationary" but is not a move. */
    ready_rig();
    rig.look_action = 5U;
    CHECK(clearance_sweep(50.0, "wobble"));
    CHECK(rig.pings > 0U);
    /* And the same for the scan's own check, while held. */
    reset_rig();
    rig.look_action = 5U;
    {
        s5_estimate_t estimate;
        CHECK(scan_measure(NULL, 90, &estimate));
        CHECK(rig.pings > 0U);
        reset_rig();
        rig.look_action = 3U; /* Really pushed: not trusted. */
        CHECK(!scan_measure(NULL, 90, &estimate));
        CHECK(rig.pings == 0U && rig.status.motion.hold);
    }

    ready_rig();
    CHECK(clearance_sweep(50.0, "test"));
    CHECK(rig.moves == 0U);
    CHECK(rig.angle_reads[60] > 0U && rig.angle_reads[90] > 0U && rig.angle_reads[120] > 0U);
    CHECK(rig.angle_reads[75] == 0U && rig.angle_reads[105] == 0U);

    /* Centre is clear, but either front corner can still be obstructed. */
    for (int angle = 60; angle <= 120; angle += 30) {
        ready_rig();
        echo_at(angle, 100.0);
        CHECK(!clearance_sweep(50.0, "corner"));
        CHECK(rig.moves == 0U);
    }
    ready_rig();
    echo_at(60, 1000.0); /* Distant echoes do not block the next short move. */
    CHECK(clearance_sweep(50.0, "far"));

    for (unsigned action = 1U; action <= 4U; ++action) {
        ready_rig();
        rig.look_action = action;
        CHECK(!clearance_sweep(50.0, "interrupted"));
        CHECK(rig.moves == 0U && rig.pings == 0U);
    }
    ready_rig();
    rig.release_during_ping = true;
    CHECK(!clearance_sweep(50.0, "moved"));
    CHECK(rig.moves == 0U);
    ready_rig();
    rig.status_missing = true;
    CHECK(!clearance_sweep(50.0, "missing-status"));
    CHECK(rig.pings == 0U);

    for (unsigned side = 0U; side < 2U; ++side) {
        s5_detour_plan_t p = plan;
        int obstacle_angle = side == 0U ? 20 : 160;
        p.side = side == 0U ? S5_AVOID_RIGHT : S5_AVOID_LEFT;
        p.turn_deg = side == 0U ? -60.0 : 60.0;
        p.shift_mm = side == 0U ? -190.0 : 190.0;
        ready_rig();
        CHECK(go_out(&p));
        CHECK(rig.distance_calls == 1U && rig.turn_calls == 2U); /* One leg, no stop on the diagonal. */
        CHECK(rig.largest_distance <= S5_DIAGONAL_STEP_MM / 1000.0);
        CHECK(rig.angle_reads[obstacle_angle] > 0U);
        CHECK(rig.angle_reads[180 - obstacle_angle] == 0U);
        CHECK(fabs(s5_y - (p.diagonal_mm + S5_DIAGONAL_EXTRA_MM) * sin(p.turn_deg * S5_DEG_TO_RAD)) < 0.01);

        ready_rig();
        echo_at(90, 100.0);
        CHECK(!go_out(&p)); /* Blocked at every angle: turns out as far as allowed, no travel. */
        CHECK(rig.turn_calls == 1U + RETRIES && rig.distance_calls == 0U);
        CHECK(fabs(fabs(rig.status.motion.heading) - S5_OUT_RETRY_MAX_DEG * S5_DEG_TO_RAD) < 1e-9);

        ready_rig();
        echo_at(90, 40.0);
        rig.override_after_distance = 1U;
        p.diagonal_mm = 2.0 * S5_DIAGONAL_STEP_MM; /* Long enough to need a second section. */
        CHECK(!go_out(&p)); /* A new obstruction stops the next diagonal section. */
        CHECK(rig.distance_calls == 1U && rig.turn_calls == 1U + RETRIES);
        p.diagonal_mm = plan.diagonal_mm;

        ready_rig();
        echo_at(90, 100.0);
        rig.override_after_distance = 1U;
        CHECK(!go_out(&p)); /* Parallel sweep refuses forward passing. */
        CHECK(rig.distance_calls == 1U && rig.turn_calls == 2U);

        ready_rig();
        echo_at(obstacle_angle, 70.0); /* ~94 mm sideways: less than 80 + 50 mm. */
        CHECK(!go_out(&p));
        CHECK(rig.distance_calls == 1U && rig.turn_calls == 2U);
        ready_rig();
        echo_at(obstacle_angle, 150.0); /* ~169 mm sideways: enough clearance. */
        CHECK(go_out(&p));

        ready_rig();
        rig.distance_scale = 0.9; /* Undershoot is not compensated: same steps as a perfect run. */
        CHECK(go_out(&p));

        ready_rig();
        rig.distance_scale = 0.0;
        CHECK(!go_out(&p));
        CHECK(rig.distance_calls == 1U && rig.turn_calls == 1U);
    }
}

static void scan_tests(void)
{
    s5_estimate_t estimate;
    reset_rig();
    rig.status_missing = true;
    CHECK(!scan_measure(NULL, 90, &estimate));
    CHECK(rig.pings == 0U);

    reset_rig();
    rig.status.motion.stationary = false;
    rig.status.motion.state = STOPPING; /* Still braking is not a wobble: refused on state. */
    CHECK(!scan_measure(NULL, 90, &estimate));
    CHECK(rig.pings == 0U);

    reset_rig();
    rig.release_during_servo = true;
    CHECK(!scan_measure(NULL, 90, &estimate));
    CHECK(rig.status.motion.hold);
    CHECK(rig.pings == 0U);

    reset_rig();
    rig.release_during_ping = true;
    CHECK(!scan_measure(NULL, 90, &estimate));
    CHECK(rig.status.motion.hold);

    reset_rig();
    rig.front_result = S5_US_BUSY;
    CHECK(!scan_measure(NULL, 90, &estimate));

    reset_rig();
    CHECK(scan_measure(NULL, 90, &estimate));
    CHECK(estimate.status == S5_ECHO_NONE);

    reset_rig();
    rig.status.motion.hold = false;
    rig.status.motion.state = DRIVING;
    rig.status.motion.stationary = false;
    rig.front_result = S5_US_OK;
    rig.refuse_stop = true;
    rig.stop_at = 6.0;
    if (setjmp(task_end) == 0) { obstacle_task(0, NULL); }
    CHECK(rig.servo_moves == 0U); /* A stop timeout must never start a sweep. */
    CHECK(rig.releases == 0U);

    reset_rig();
    rig.status.motion.hold = false;
    rig.status.motion.state = DRIVING;
    rig.front_result = S5_US_NO_PULSE;
    rig.stop_at = 0.5;
    if (setjmp(task_end) == 0) { obstacle_task(0, NULL); }
    CHECK(rig.status.motion.hold); /* Loss of the front sensor must hold motion. */
}

static void pass_ready(void)
{
    reset_rig();
    rig.status.motion.hold = false;
    rig.status.motion.state = READY;
    s5_heading0 = 0.0;
    s5_x = 400.0;
    s5_y = plan.shift_mm;
}

static void pass_return_transition_tests(void)
{
    for (unsigned side = 0U; side < 2U; ++side) {
        s5_detour_plan_t p = plan;
        p.side = side == 0U ? S5_AVOID_RIGHT : S5_AVOID_LEFT;
        p.shift_mm = side == 0U ? -190.0 : 190.0;
        p.edge_mm = side == 0U ? -75.0 : 75.0;
        reset_rig();
        rig.status.motion.hold = false;
        rig.status.motion.state = READY;
        s5_heading0 = 0.0;
        s5_x = 400.0;
        s5_y = p.shift_mm;
        /* The submission collides once with the status mutex. No command
         * was queued on R_BUSY. */
        rig.busy_distance_call = 1U;
        CHECK(pass_obstacle(&p));
        CHECK(rig.distance_calls == 2U && rig.moves == 1U);
        CHECK(s5_x > 400.0 + S5_PASS_STEP_MM);
        /* One stop, at the end: every look happened on the move. */
        CHECK(rig.stops == 1U && rig.releases == 1U);
        CHECK(rig.moving_pings == rig.pings && rig.pings > 0U);
        CHECK(rig.angle_reads[S5_ANGLE_CENTRE_DEG] > 0U); /* It still looks ahead. */
        /* Same transient collision when starting the inward turn. */
        rig.busy_turn_call = 1U;
        CHECK(turn_to(side == 0U ? S5_RETURN_DEG : -S5_RETURN_DEG) == S5_MOVE_DONE);
        CHECK(rig.turn_calls == 2U && rig.moves == 2U);

        /* Something appears ahead: it stops while still moving. */
        pass_ready();
        s5_y = p.shift_mm;
        echo_at(S5_ANGLE_CENTRE_DEG, 100.0);
        CHECK(!pass_obstacle(&p));
        CHECK(rig.status.motion.hold && rig.stops == 1U && rig.distance_calls == 1U);

        /* The front sensor dies mid-pass. */
        pass_ready();
        s5_y = p.shift_mm;
        rig.front_result = S5_US_NO_PULSE;
        CHECK(!pass_obstacle(&p));
        CHECK(rig.status.motion.hold);

        /* The side sensor dies mid-pass. */
        pass_ready();
        s5_y = p.shift_mm;
        rig.side_result = S5_US_BUSY;
        CHECK(!pass_obstacle(&p));
        CHECK(rig.status.motion.hold);

        /* An endless obstacle: gives up after S5_PASS_MAX_MM, stopped. */
        pass_ready();
        s5_y = p.shift_mm;
        rig.side_result = S5_US_OK;
        CHECK(!pass_obstacle(&p));
        CHECK(rig.status.motion.hold && s5_x - 400.0 >= S5_PASS_MAX_MM);
        CHECK(s5_x - 400.0 <= S5_PASS_MAX_MM + 2.0 * S5_PASS_STEP_MM);

        /* A fault while the servo swings is noticed before the next ping. */
        pass_ready();
        s5_y = p.shift_mm;
        rig.side_result = S5_US_OK;
        rig.fault_at_s = rig.now + 0.1;
        CHECK(!pass_obstacle(&p));
        CHECK(rig.pings == 0U);
    }
    reset_rig();
    rig.status.motion.hold = false;
    rig.status.motion.state = READY;
    rig.always_busy = true;
    CHECK(turn_to(45.0) == S5_MOVE_FAILED);
    CHECK(rig.moves == 0U && rig.now < 0.7); /* Retries must remain bounded. */
    for (unsigned action = 1U; action <= 4U; ++action) {
        reset_rig();
        rig.status.motion.hold = false;
        rig.status.motion.state = READY;
        rig.busy_turn_call = 1U;
        rig.retry_action = action;
        CHECK(turn_to(45.0) == S5_MOVE_FAILED);
        CHECK(rig.turn_calls == 1U && rig.moves == 0U);
    }
    reset_rig();
    rig.status.motion.hold = false;
    rig.status.motion.state = READY;
    rig.lose_status_after_completion = true;
    CHECK(straight(50.0, false, NULL) == S5_MOVE_FAILED);

    reset_rig();
    rig.status.motion.hold = false;
    rig.status.motion.state = READY;
    rig.status.motion.id = 5101U;
    rig.status.last_event = (motion_event_t){EV_COMPLETE, 5101U, 0U, F_NONE};
    rig.status.line.values[1] = 1.0;
    s5_y = 0.0;
    s5_heading0 = rig.status.motion.heading = 0.0;
    double line_centre = -1.0;
    CHECK(wait_move(5101U, 1.0, true, 0.0, &line_centre) == S5_MOVE_LINE);
    CHECK(rig.status.motion.hold && line_centre == rig.status.motion.centre);

    rig.status.motion.hold = false;
    rig.status.last_event.type = EV_REJECTED;
    line_centre = -1.0;
    CHECK(wait_move(5101U, 1.0, true, 0.0, &line_centre) == S5_MOVE_FAILED);
    CHECK(line_centre == -1.0); /* Black cannot conceal a rejected command. */

    for (unsigned side = 0U; side < 2U; ++side) {
        s5_detour_plan_t p = plan;
        p.side = side == 0U ? S5_AVOID_RIGHT : S5_AVOID_LEFT;
        reset_rig();
        rig.status.motion.hold = false;
        rig.status.motion.state = READY;
        s5_heading0 = 0.0;
        s5_x = 650.0;
        s5_y = side == 0U ? -190.0 : 190.0;
        rig.detect_line_on_next_distance = true;
        CHECK(return_to_line(&p));
        /* Turn in and drive: no drive on and no turn back once the line is found. */
        CHECK(rig.moves == 2U && rig.releases == 1U && rig.turn_calls == 1U);
        CHECK(fabs(fabs(rig.status.motion.heading) - S5_RETURN_DEG * S5_DEG_TO_RAD) < 1e-9);
        CHECK(!rig.status.motion.hold && rig.status.motion.state == READY);
    }
}

/* A flat face 180 mm from the servo axis, with the robot yawed right by yaw_deg. */
static void yawed_face(double yaw_deg)
{
    double face = 180.0 * cos(yaw_deg * S5_DEG_TO_RAD);
    for (int angle = S5_ANGLE_LEFT_DEG; angle <= S5_ANGLE_RIGHT_DEG; angle += S5_COARSE_STEP_DEG) {
        double ray = ((double)(S5_ANGLE_CENTRE_DEG - angle) - yaw_deg) * S5_DEG_TO_RAD;
        double axis = face / cos(ray);
        rig.angle_override[angle] = false;
        if (cos(ray) > 0.0 && fabs(axis * sin(ray)) <= 100.0) {
            rig.angle_override[angle] = true;
            rig.angle_result[angle] = S5_US_OK;
            rig.angle_range[angle] = axis - S5_SENSOR_OFFSET_MM;
        }
    }
}

/* Drive up to it through the real task, scan, and see which side it chose. */
static void run_to_scan(unsigned drive_pings, double heading_per_ping, double yaw_deg, unsigned stop_at_turn)
{
    reset_rig();
    rig.status.motion.hold = false;
    rig.status.motion.state = DRIVING;
    rig.status.motion.stationary = false;
    rig.status.motion.heading = 0.0;
    rig.drive_pings = drive_pings;
    rig.heading_per_ping = heading_per_ping;
    yawed_face(yaw_deg);
    rig.yaw_live = true;
    rig.yaw_now = yaw_deg;
    rig.stop_at_turn = stop_at_turn;
    rig.stop_at = drive_pings * 0.07 + 60.0;
    if (setjmp(task_end) == 0) { obstacle_task(0, NULL); }
}

/* The pass stops against something: tilt away, back out, resume. */
static void stall_escape_tests(void)
{
    for (unsigned side = 0U; side < 2U; ++side) {
        s5_detour_plan_t p = plan;
        double sign = side == 0U ? -1.0 : 1.0;
        p.side = side == 0U ? S5_AVOID_RIGHT : S5_AVOID_LEFT;
        p.shift_mm = sign * 190.0;
        p.edge_mm = sign * 75.0;

        /* One stall: tilt 20 deg away, back out 100 mm, parallel again, finish the pass. */
        pass_ready();
        s5_y = p.shift_mm;
        rig.stalls_left = 1U;
        rig.stall_after_m = 0.05;
        CHECK(pass_obstacle(&p));
        CHECK(rig.turn_calls == 2U);                 /* tilt, then parallel */
        CHECK(rig.distance_calls == 3U);             /* pass, back out, pass again */
        CHECK(fabs(rig.status.motion.heading) < 1e-9);
        CHECK(rig.stalls_left == 0U);
        CHECK(!s5_leg.pass_stalled);

        /* The tilt is blocked at 20 deg: tilts further, to 40. */
        pass_ready();
        s5_y = p.shift_mm;
        rig.stalls_left = 1U;
        rig.stall_after_m = 0.05;
        echo_at(60, 100.0);
        echo_at(120, 100.0);
        rig.block_h_min = 10.0 * S5_DEG_TO_RAD;
        rig.block_h_max = 30.0 * S5_DEG_TO_RAD;
        CHECK(pass_obstacle(&p));
        CHECK(rig.turn_calls == 3U);                 /* 20, 40, parallel */
        CHECK(fabs(rig.status.motion.heading) < 1e-9);

        /* Stalls every time: gives up after the allowed tilts. */
        pass_ready();
        s5_y = p.shift_mm;
        rig.stalls_left = 99U;
        rig.stall_after_m = 0.05;
        CHECK(!pass_obstacle(&p));
        CHECK(rig.status.motion.hold);
        CHECK(rig.turn_calls == 2U * S5_ESCAPE_TRIES);
        CHECK(!s5_leg.rejoin_armed);

        /* Blocked at every tilt: holds. */
        pass_ready();
        s5_y = p.shift_mm;
        rig.stalls_left = 1U;
        rig.stall_after_m = 0.05;
        echo_at(60, 100.0);
        echo_at(120, 100.0);
        rig.block_h_min = 10.0 * S5_DEG_TO_RAD;
        rig.block_h_max = 90.0 * S5_DEG_TO_RAD;
        CHECK(!pass_obstacle(&p));
        CHECK(rig.status.motion.hold);
    }

    /* A move that is merely slow to start is not a stall. */
    pass_ready();
    {
        double t0 = rig.now;
        rig.status.motion.state = DRIVING;
        rig.status.motion.id = 5100U;
        rig.status.motion.target[0] = rig.status.motion.target[1] = 0.08;
        CHECK(move_poll(5100U, t0, 100.0, false, 0.0, NULL) == S5_MOVE_RUNNING);
        rig.now += S5_STALL_S - 0.1;
        CHECK(move_poll(5100U, t0, 100.0, false, 0.0, NULL) == S5_MOVE_RUNNING);
        rig.status.motion.speed[0] = 0.05; /* moving again: the timer starts over */
        CHECK(move_poll(5100U, t0, 100.0, false, 0.0, NULL) == S5_MOVE_RUNNING);
        rig.status.motion.speed[0] = 0.0;
        CHECK(move_poll(5100U, t0, 100.0, false, 0.0, NULL) == S5_MOVE_RUNNING);
        rig.now += 1.0;
        CHECK(move_poll(5100U, t0, 100.0, false, 0.0, NULL) == S5_MOVE_RUNNING);
        rig.now += S5_STALL_S;
        CHECK(move_poll(5100U, t0, 100.0, false, 0.0, NULL) == S5_MOVE_STALLED);
        CHECK(rig.status.motion.hold);
    }
}

/* Stopped too close: reverse to 150 mm before scanning. */
static void back_off_tests(void)
{
    const double ranges[] = {100.0, 60.0};
    for (unsigned i = 0U; i < 2U; ++i) {
        reset_rig();
        rig.angle_override[S5_ANGLE_CENTRE_DEG] = true;
        rig.angle_result[S5_ANGLE_CENTRE_DEG] = S5_US_OK;
        rig.angle_range[S5_ANGLE_CENTRE_DEG] = ranges[i];
        rig.stop_lag_ticks = 3U; /* STOPPED arrives a few polls after motion_stop(). */
        CHECK(back_off_if_close());
        /* The mock does not move the obstacle, so every try sees it still close. */
        CHECK(rig.distance_calls == S5_BACKOFF_TRIES);
        CHECK(fabs(rig.status.motion.centre + (double)S5_BACKOFF_TRIES * (S5_STOP_MM - ranges[i]) / 1000.0) < 1e-9);
        CHECK(rig.status.motion.hold && rig.status.motion.state == STOPPED);
        CHECK(s5_task.phase == S5_PHASE_IDLE);
    }

    reset_rig(); /* Only the nearest corner of a slanted face is close: still reverses. */
    rig.angle_override[50] = true;
    rig.angle_result[50] = S5_US_OK;
    rig.angle_range[50] = 100.0;
    CHECK(back_off_if_close());
    CHECK(rig.distance_calls == S5_BACKOFF_TRIES && rig.angle == S5_ANGLE_CENTRE_DEG);

    reset_rig(); /* Within the tolerance: no move. */
    rig.angle_override[S5_ANGLE_CENTRE_DEG] = true;
    rig.angle_result[S5_ANGLE_CENTRE_DEG] = S5_US_OK;
    rig.angle_range[S5_ANGLE_CENTRE_DEG] = S5_STOP_MM - S5_BACKOFF_TOL_MM + 1.0;
    CHECK(back_off_if_close() && rig.distance_calls == 0U);

    reset_rig(); /* Further than the target: no move. */
    rig.angle_override[S5_ANGLE_CENTRE_DEG] = true;
    rig.angle_result[S5_ANGLE_CENTRE_DEG] = S5_US_OK;
    rig.angle_range[S5_ANGLE_CENTRE_DEG] = 180.0;
    CHECK(back_off_if_close() && rig.distance_calls == 0U);

    reset_rig(); /* No echo: the scan decides. */
    CHECK(back_off_if_close() && rig.distance_calls == 0U);

    reset_rig(); /* A wobble does not stop the back-off's looks. */
    rig.look_action = 5U;
    CHECK(back_off_if_close() && rig.pings > 0U);

    reset_rig(); /* Sensor fault: hold. */
    rig.front_result = S5_US_NO_PULSE;
    CHECK(!back_off_if_close() && rig.status.motion.hold);

    reset_rig(); /* The reverse is refused: hold. */
    rig.angle_override[S5_ANGLE_CENTRE_DEG] = true;
    rig.angle_result[S5_ANGLE_CENTRE_DEG] = S5_US_OK;
    rig.angle_range[S5_ANGLE_CENTRE_DEG] = 100.0;
    rig.always_busy = true;
    CHECK(!back_off_if_close() && rig.status.motion.hold && s5_task.phase == S5_PHASE_IDLE);
}

/* Through the real task: the tilt sets the side and the turns, curve or none. */
static void face_task_tests(void)
{
    for (unsigned curve = 0U; curve < 2U; ++curve) {
        double per_ping = curve == 0U ? 0.0 : 2.0 * S5_DEG_TO_RAD;
        for (unsigned side = 0U; side < 2U; ++side) {
            double yaw = side == 0U ? 15.0 : -15.0; /* + robot points right of square */
            s5_detour_plan_t p;

            run_to_scan(40U, per_ping, yaw, 2U); /* to the detour's first turn */
            CHECK(s5_scan.b_valid && s5_scan.face.b_valid);
            /* It turned to face it, then scanned again from there. */
            CHECK(fabs(rig.first_turn_deg - yaw) < 3.0);
            CHECK(fabs(s5_scan.squared_deg - rig.first_turn_deg) < 6.0);
            CHECK(fabs(s5_scan.face.tilt_deg) < 5.0);
            CHECK(fabs(s5_start_tilt_deg(&s5_scan) - yaw) < 6.0);
            /* Centred: the side that was closer when it stopped. */
            CHECK(s5_scan.avoid.side == (side == 0U ? S5_AVOID_LEFT : S5_AVOID_RIGHT));
            CHECK(s5_detour_plan(&s5_scan, &p));
            /* Still knows the axle is off the line by the tilt it stopped with. */
            CHECK(fabs(p.start_y_mm - S5_IR_AHEAD_MM * sin(s5_start_tilt_deg(&s5_scan) * S5_DEG_TO_RAD)) < 1e-9);
            /* The detour's first turn is what is left after the tilt, from square. */
            CHECK(rig.turn_calls == 2U);
            CHECK(fabs(rig.stopped_turn_deg - (p.turn_deg + p.tilt_deg)) < 1.0);
        }
    }
    CHECK(s5_curve_recent(&s5_curve)); /* Still logged. */

    /* A small tilt is not worth turning for, and the width still gets its fine scan. */
    run_to_scan(40U, 0.0, 3.0, 1U);
    CHECK(s5_scan.face.b_valid && s5_scan.squared_deg == 0.0);
    CHECK(s5_scan.fine_count[S5_LEFT] > 0U && s5_scan.fine_count[S5_RIGHT] > 0U);

    /* Up to the first pivot: only coarse angles were read. */
    run_to_scan(0U, 0.0, 20.0, 1U);
    CHECK(rig.turn_calls == 1U);
    {
        unsigned fine_reads = 0U;
        for (int angle = S5_ANGLE_LEFT_DEG; angle <= S5_ANGLE_RIGHT_DEG; ++angle) {
            if (angle % S5_COARSE_STEP_DEG != 0) { fine_reads += rig.angle_reads[angle]; }
        }
        CHECK(fine_reads == 0U);
    }

    /* The turn is refused: holds, and does not plan from the stale scan. */
    reset_rig();
    rig.always_busy = true;
    {
        double turned = 0.0;
        CHECK(!square_up(15.0, &turned));
        CHECK(rig.status.motion.hold && s5_task.phase == S5_PHASE_IDLE && turned == 0.0);
    }
    /* A good turn reports what the encoders measured, and ends held. */
    reset_rig();
    {
        double turned = 0.0;
        CHECK(square_up(15.0, &turned));
        CHECK(fabs(turned - 15.0) < 1e-9 && rig.status.motion.hold && rig.status.motion.state == STOPPED);
    }
    /* A curve long past is forgotten. */
    reset_rig();
    rig.status.motion.hold = false;
    rig.status.motion.state = DRIVING;
    rig.status.motion.stationary = false;
    rig.drive_pings = 20U;
    rig.heading_per_ping = 3.0 * S5_DEG_TO_RAD;
    rig.stop_at = 2.0;
    if (setjmp(task_end) == 0) { obstacle_task(0, NULL); }
    CHECK(s5_curve_recent(&s5_curve));
    s5_curve_note(&s5_curve, rig.status.motion.path * 1000.0 + 2.0 * ((double)S5_CURVE_WINDOW_MM + S5_CURVE_HOLD_MM),
                  rig.status.motion.heading / S5_DEG_TO_RAD);
    CHECK(!s5_curve_recent(&s5_curve));

    /* The detour starts in the face's frame: heading -tilt, sensors on the line. */
    {
        s5_detour_plan_t p = plan;
        reset_rig();
        rig.status.motion.heading = 1.0;
        p.tilt_deg = -20.0;
        p.start_y_mm = S5_IR_AHEAD_MM * sin(p.tilt_deg * S5_DEG_TO_RAD);
        (void)s5_detour_run(&p);
        CHECK(fabs(s5_heading0 - (1.0 + p.tilt_deg * S5_DEG_TO_RAD)) < 1e-9);
        CHECK(fabs(rig.first_turn_deg - (p.turn_deg + p.tilt_deg)) < 1.0);
    }
}

/* rejoin_after_stop() from a held, still robot (as a move leaves it) */
static bool rig_rejoin_after_stop(void)
{
    rig.status.motion.hold = true;
    rig.status.motion.state = STOPPED;
    rig.status.motion.stationary = true;
    return rejoin_after_stop();
}

static void line_interrupt_tests(void)
{
    /* Whole detour: the diagonal is call 1, the pass call 2, the way back call 3.
     * A junction's branch under the robot beside the obstacle is noted, not
     * taken: the straight line on the way back is where it hands over. */
    reset_rig();
    rig.black_calls = 1U << 2U;
    rig.clear_calls = 1U << 3U;
    rig.line_partial_call = 3U;
    CHECK(s5_detour_run(&plan));
    CHECK(!s5_leg.rejoined && !s5_leg.rejoin_armed && s5_leg.line_noted);
    CHECK(rig.distance_calls == 3U);
    CHECK(rig.turn_calls == 3U); /* out, parallel, in towards the line */
    CHECK(s5_line_expected(s5_y, 45.0)); /* handed over where the straight line is */
    CHECK(!rig.status.motion.hold && rig.status.motion.state == READY);

    /* The line curved: black beside the obstacle and nothing on the way back,
     * so it drives back to where it saw it and hands over there. */
    reset_rig();
    rig.black_calls = 1U << 2U;
    rig.clear_calls = (1U << 3U);
    rig.black_calls |= 1U << 4U; /* there again when it drives back */
    CHECK(s5_detour_run(&plan));
    CHECK(s5_leg.line_noted && rig.distance_calls == 4U);
    CHECK(rig.turn_calls == 4U); /* out, parallel, in, towards the noted line */
    CHECK(fabs(rig.turn_log[3]) > 90.0); /* it is behind the robot by then */
    CHECK(!rig.status.motion.hold && rig.status.motion.state == READY);

    /* Something in the way back to it: holds and says so. */
    reset_rig();
    rig.black_calls = 1U << 2U;
    rig.clear_calls = 1U << 3U;
    echo_at(S5_ANGLE_CENTRE_DEG, 100.0);
    rig.override_after_distance = 3U;
    CHECK(!s5_detour_run(&plan));
    {
        int a = 0, b = 0;
        const char *p_code = s5_why_name(s5_detour_reason(&a, &b));
        CHECK(p_code != NULL && strcmp(p_code, "noted_line_blocked") == 0);
    }
    CHECK(rig.distance_calls == 3U);

    /* Gone when it gets there: holds. */
    reset_rig();
    rig.black_calls = 1U << 2U;
    rig.clear_calls = 1U << 3U;
    CHECK(!s5_detour_run(&plan));
    {
        int a = 0, b = 0;
        const char *p_code = s5_why_name(s5_detour_reason(&a, &b));
        CHECK(p_code != NULL && strcmp(p_code, "noted_line_not_found") == 0);
    }
    CHECK(rig.distance_calls == 4U);

    /* 18 Nov: the curve crossed on the way back itself, 250 mm off the straight
     * line, and no line where the straight path should be.  It reverses along
     * the same track onto the curve: no turning round. */
    pass_ready();
    s5_x = 650.0;
    s5_y = -700.0;               /* a long way back: the rig drives it in time */
    rig.black_band = true;
    rig.black_from_m = 0.200;    /* sensors over the curve for 30 mm of the leg */
    rig.black_to_m = 0.230;
    CHECK(return_to_line(&plan));
    CHECK(s5_leg.line_noted && rig.distance_calls == 2U);
    CHECK(rig.turn_calls == 1U); /* only the turn in: no turning round */
    CHECK(rig.status.motion.centre >= 0.195 && rig.status.motion.centre <= 0.235);
    CHECK(!rig.status.motion.hold && rig.status.motion.state == READY);
    CHECK(strstr(rig.encoder_log, "result=rejoin") != NULL);

    /* Braked off it while reversing: drives forward back onto it, not further back. */
    reset_rig();
    rig.status.motion.centre = 0.285;
    s5_rejoin_centre_m = 0.300;  /* the line: 15 mm ahead after reversing past it */
    CHECK(!rig_rejoin_after_stop()); /* no line in this rig: it holds afterwards */
    CHECK(rig.distance_calls == 1U && rig.last_metres > 0.014 && rig.last_metres < 0.016);
    /* And after a forward move, it reverses as before. */
    reset_rig();
    rig.status.motion.centre = 0.315;
    s5_rejoin_centre_m = 0.300;
    CHECK(!rig_rejoin_after_stop());
    CHECK(rig.distance_calls == 1U && rig.last_metres < -0.014 && rig.last_metres > -0.016);

    /* Not armed on the way out: black on the diagonal does not interrupt it. */
    reset_rig();
    rig.status.motion.hold = false;
    rig.status.motion.state = READY;
    s5_heading0 = 0.0;
    s5_x = s5_y = 0.0;
    rig.status.line.values[LINE_VALUE_BOTH_BLACK] = 1.0;
    CHECK(go_out(&plan));
    CHECK(rig.distance_calls == 1U && rig.turn_calls == 2U && rig.releases == 0U);

    /* Armed on the way back: where odometry puts the line, hands over where it
     * is, without driving on or turning back to the original heading. */
    pass_ready();
    s5_x = 650.0;
    s5_y = plan.shift_mm;
    rig.line_partial_call = 1U;
    CHECK(return_to_line(&plan));
    CHECK(rig.distance_calls == 1U && rig.turn_calls == 1U && rig.stops == 1U && rig.releases == 1U);
    CHECK(!rig.status.motion.hold && rig.status.motion.state == READY && !s5_leg.rejoin_armed);
    CHECK(strstr(rig.encoder_log, "result=rejoin") != NULL);

    /* Far from where odometry puts the line, both black is noted, not taken:
     * it drives on and hands over only once the sensors reach the straight path. */
    pass_ready();
    s5_x = 650.0;
    s5_y = -700.0;
    rig.both_black_on_distance_call = 1U;
    CHECK(return_to_line(&plan));
    CHECK(s5_leg.line_noted && rig.distance_calls == 1U && rig.stops == 1U && rig.releases == 1U);
    CHECK(s5_line_expected(s5_y, 45.0)); /* stopped at the window's edge, about y -157 */
    CHECK(strstr(rig.encoder_log, "result=rejoin") != NULL);

    /* Braked past it on the way back: reverses onto it, then hands over. */
    pass_ready();
    s5_x = 650.0;
    s5_y = plan.shift_mm;
    rig.line_partial_call = 1U;
    rig.lose_line_on_stop = true;
    rig.line_back_on_reverse = true;
    CHECK(return_to_line(&plan));
    CHECK(rig.distance_calls == 2U && rig.releases == 1U);
    CHECK(!rig.status.motion.hold && rig.status.motion.state == READY);

    /* One sensor only is not enough to stop... */
    pass_ready();
    s5_x = 650.0;
    s5_y = 80.0 * sin(45.0 * S5_DEG_TO_RAD) * -1.0 - 30.0; /* Sensors inside the window. */
    rig.one_black_on_distance_call = 1U;
    rig.clear_line_at_s = rig.now + 1.0;
    /* ...until it comes off the line without both ever being black: it backs up. */
    rig.line_back_on_reverse = true;
    CHECK(return_to_line(&plan));
    CHECK(rig.distance_calls == 2U && rig.stops == 1U);
    CHECK(strstr(rig.encoder_log, "result=line") != NULL);

    /* ...and if the line does not come back, it holds. */
    pass_ready();
    s5_x = 650.0;
    s5_y = 80.0 * sin(45.0 * S5_DEG_TO_RAD) * -1.0 - 30.0;
    rig.one_black_on_distance_call = 1U;
    rig.clear_line_at_s = rig.now + 1.0;
    CHECK(!return_to_line(&plan));
    CHECK(rig.distance_calls == 2U && rig.status.motion.hold == false);

    /* One sensor far from where odometry puts the line is something else. */
    pass_ready();
    s5_x = 650.0;
    s5_y = -800.0;
    rig.one_black_on_distance_call = 1U;
    rig.clear_line_at_s = rig.now + 2.0;
    CHECK(!return_to_line(&plan));
    CHECK(rig.distance_calls == 1U && rig.stops == 0U);

    /* Black beside the obstacle, away from the straight line (a junction's
     * branch, or the line curving away): noted, and the pass carries on. */
    pass_ready();
    rig.status.line.values[1] = 1.0;
    rig.status.line.values[LINE_VALUE_BOTH_BLACK] = 1.0;
    CHECK(pass_obstacle(&plan));
    CHECK(!s5_leg.rejoined && s5_leg.line_noted && rig.distance_calls == 1U);
    CHECK(rig.stops == 1U); /* only the planned stop at the end of the pass */
    CHECK(fabs(s5_noted_y - plan.shift_mm) < 1.0);

    /* Black from the moment it is parallel, where the line should be: not
     * handed back until it has driven on. */
    pass_ready();
    s5_y = -20.0;
    rig.status.line.values[1] = 1.0;
    rig.status.line.values[LINE_VALUE_BOTH_BLACK] = 1.0;
    CHECK(pass_obstacle(&plan));
    CHECK(s5_leg.rejoined && rig.distance_calls == 1U);
    CHECK(rig.status.motion.centre >= 0.02 && rig.status.motion.centre < 0.03);

    /* Black during a stall escape: the tilt does not hand back; parallel again, 20 mm counts anew. */
    pass_ready();
    s5_y = -20.0;
    rig.stalls_left = 1U;
    rig.stall_after_m = 0.005;
    rig.status.line.values[1] = 1.0;
    rig.status.line.values[LINE_VALUE_BOTH_BLACK] = 1.0;
    CHECK(pass_obstacle(&plan));
    CHECK(s5_leg.rejoined && rig.turn_calls == 2U && rig.distance_calls == 3U);
    CHECK(fabs(rig.status.motion.heading) < 1e-9);

    /* A stale reading is not the line. */
    pass_ready();
    rig.status.line.values[LINE_VALUE_BOTH_BLACK] = 1.0;
    rig.status.line.timestamp = -1.0;
    CHECK(!line_rejoin_seen(&rig.status, rig.now));

    /* Publisher gone (a profile without line steering): never fires. */
    pass_ready();
    rig.status.line.available = false;
    rig.status.line.values[LINE_VALUE_BOTH_BLACK] = 1.0;
    CHECK(!line_rejoin_seen(&rig.status, rig.now));

    /* Braked past it and it stays lost: reverse once, then hold. */
    pass_ready();
    s5_y = -20.0;
    rig.both_black_on_distance_call = 1U;
    rig.lose_line_on_stop = true;
    CHECK(!pass_obstacle(&plan));
    CHECK(!s5_leg.rejoined && !s5_leg.rejoin_armed);
    CHECK(rig.distance_calls == 2U && rig.releases == 1U);
    CHECK(rig.largest_distance >= S5_PASS_STEP_MM / 1000.0);

    /* Black cannot conceal a rejected command. */
    pass_ready();
    s5_leg.rejoin_armed = true;
    rig.status.motion.id = 5101U;
    rig.status.last_event = (motion_event_t){EV_REJECTED, 5101U, 0U, F_NONE};
    rig.status.line.values[LINE_VALUE_BOTH_BLACK] = 1.0;
    CHECK(wait_move(5101U, 1.0, false, 0.0, NULL) == S5_MOVE_FAILED);
    s5_leg.rejoin_armed = false;

    /* A pass stopped on purpose is not logged as a failed move. */
    pass_ready();
    CHECK(pass_obstacle(&plan));
    CHECK(strstr(rig.encoder_log, "result=done") != NULL);
    CHECK(strstr(rig.encoder_log, "result=failed") == NULL);
}

/* A 200 mm box, centred, with the sensor 150 mm from it; no face fit, so the
   edges decide.  Its plans: right at about 70 deg with no reversing, right at
   55 deg after 79 mm of reversing, and the mirror images on the left. */
static void ladder_scan(void)
{
    memset(&s5_scan, 0, sizeof(s5_scan));
    s5_scan.b_valid = true;
    s5_scan.centre = centre_point(150.0);
    s5_scan.centre_forward_mm = 180.0;
    s5_scan.edge[S5_LEFT].b_found = s5_scan.edge[S5_RIGHT].b_found = true;
    s5_scan.edge[S5_LEFT].lateral_mm = 100.0;
    s5_scan.edge[S5_RIGHT].lateral_mm = -100.0;
    s5_scan.avoid.side = S5_AVOID_RIGHT;
    s5_scan.avoid.shift_mm[S5_LEFT] = s5_scan.avoid.shift_mm[S5_RIGHT] = 230.0;
    s5_scan.avoid.b_room[S5_LEFT] = s5_scan.avoid.b_room[S5_RIGHT] = true;
    s5_scan.avoid.clear_mm[S5_LEFT] = s5_scan.avoid.clear_mm[S5_RIGHT] = INFINITY;
    s5_task.decided = true;
    s5_task.blocked = true;
    rig.online = true;
}

static bool turned_beyond(double from_deg, double to_deg)
{
    for (unsigned i = 0U; i < rig.turn_log_count; ++i) {
        if (rig.turn_log[i] >= from_deg && rig.turn_log[i] <= to_deg) { return true; }
    }
    return false;
}

/* Turned out, the look back towards the obstacle hears its corner: blocked
   without the plan, ignored with it.  Anything nearer still blocks. */
static void known_obstacle_tests(void)
{
    s5_detour_plan_t p;
    double expected;
    int look = S5_ANGLE_CENTRE_DEG - S5_GUARD_SWEEP_DEG; /* 30 deg left: back at the box */

    reset_rig();
    ladder_scan();
    CHECK(s5_detour_plan(&s5_scan, &p) && p.side == S5_AVOID_RIGHT && p.backup_mm == 0.0);
    expected = s5_obstacle_range(&p, 0.0, p.start_y_mm, p.turn_deg, look) - S5_SENSOR_OFFSET_MM;
    CHECK(isfinite(expected));
    for (unsigned known = 0U; known < 3U; ++known) {
        ready_rig();
        rig.status.motion.heading = p.turn_deg * S5_DEG_TO_RAD;
        s5_y = p.start_y_mm;
        echo_at(look, known == 2U ? expected - 80.0 : expected);
        s5_known = known > 0U ? &p : NULL;
        CHECK(clearance_sweep(p.diagonal_mm + S5_DIAGONAL_EXTRA_MM, "diagonal") == (known == 1U));
        CHECK(s5_leg.sweep_blocked == (known != 1U));
        s5_known = NULL;
    }
}

/* 19:15 on 18 Nov: turned out, the first look (60 deg) heard the box's corner
   as 5 scattered echoes of 10 and the detour gave up without trying again.
   Scattered echoes in the way are now "blocked": turn further, then retry. */
static void unsure_look_tests(void)
{
    const char *p_code;
    int a = 0, b = 0;

    /* Near and scattered: blocked (so retried), not a failed measurement */
    ready_rig();
    scatter_at(60, 100.0);
    CHECK(!clearance_sweep(50.0, "unsure"));
    CHECK(s5_leg.sweep_blocked && s5_block_angle == 60 && s5_block_range == 100);
    CHECK(rig.angle_reads[60] == S5_SAMPLES_PER_POINT + S5_RETRY_SAMPLES);
    /* All of it well beyond the move: clear */
    ready_rig();
    scatter_at(60, 1000.0);
    CHECK(clearance_sweep(50.0, "unsure-far"));

    /* Elsewhere (the front check before a straight leg) it still refuses, saying why */
    ready_rig();
    scatter_at(90, 100.0);
    CHECK(!path_clear(50.0));
    p_code = s5_why_name(s5_detour_reason(&a, &b));
    CHECK(p_code != NULL && strcmp(p_code, "look_unstable") == 0 && a == 90 && b == 5);
    /* A sensor fault and a nudge have their own codes */
    ready_rig();
    rig.front_result = S5_US_NO_PULSE;
    CHECK(!path_clear(50.0));
    p_code = s5_why_name(s5_detour_reason(&a, &b));
    CHECK(p_code != NULL && strcmp(p_code, "sensor_fault") == 0);
    ready_rig();
    rig.look_action = 3U;
    CHECK(!path_clear(50.0));
    p_code = s5_why_name(s5_detour_reason(&a, &b));
    CHECK(p_code != NULL && strcmp(p_code, "look_moved") == 0);

    /* The 19:15 run: scattered only while turned out ~70 deg; turns further and goes */
    reset_rig();
    ladder_scan();
    scatter_at(60, 100.0);
    rig.block_h_min = 65.0 * S5_DEG_TO_RAD;
    rig.block_h_max = 75.0 * S5_DEG_TO_RAD;
    (void)drive_round(); /* no line in this rig: it fails later, on the way back */
    CHECK(strstr(rig.all_messages, "abort=look_") == NULL);
    CHECK(turned_beyond(-11.0, -9.0)); /* turned 10 deg further out (the log is relative) */
    CHECK(rig.largest_distance > 0.2);  /* drove the diagonal */
    CHECK(strstr(rig.all_messages, "tries=1 ") != NULL);

    /* Scattered at every way out: the whole ladder, then it holds */
    reset_rig();
    ladder_scan();
    scatter_at(90, 100.0);
    rig.block_h_min = 10.0 * S5_DEG_TO_RAD;
    rig.block_h_max = 100.0 * S5_DEG_TO_RAD;
    CHECK(!drive_round());
    CHECK(strstr(rig.all_messages, "abort=diagonal_blocked") != NULL);
    CHECK(strstr(rig.all_messages, "tries=6 state=failed") != NULL);
}

static void retry_ladder_tests(void)
{
    /* Blocked ahead whenever it has turned out: tries every way, comes back to
       where it stopped each time, and holds saying why. */
    reset_rig();
    ladder_scan();
    echo_at(90, 100.0);
    rig.block_h_min = 10.0 * S5_DEG_TO_RAD;
    rig.block_h_max = 100.0 * S5_DEG_TO_RAD;
    CHECK(!drive_round());
    CHECK(s5_task.phase == S5_PHASE_FAILED && s5_task.blocked && rig.status.motion.hold);
    CHECK(rig.status.obstacle.values[3] == S5_STATE_AVOID_FAILED);
    CHECK(strstr(rig.all_messages, "abort=diagonal_blocked") != NULL);
    CHECK(strstr(rig.all_messages, "tries=6 state=failed") != NULL); /* 4 plans, then 2 waits */
    CHECK(turned_beyond(-75.0, -65.0));  /* right, steep */
    CHECK(turned_beyond(-56.0, -54.0));  /* right, shallow */
    CHECK(turned_beyond(65.0, 75.0));    /* left, steep: the other side */
    CHECK(turned_beyond(54.0, 56.0));    /* left, shallow */
    CHECK(rig.largest_distance < 0.1);   /* only reversing and coming back: no diagonal */
    CHECK(fabs(rig.status.motion.heading) < 1e-9 && fabs(s5_x) < 0.5); /* back where it stopped */
    CHECK(rig.now >= 2.0 * (double)S5_RETRY_WAIT_MS / 1000.0);

    /* A hand in the way of the first try only: the second goes. */
    reset_rig();
    ladder_scan();
    echo_at(90, 100.0);
    rig.block_h_min = 10.0 * S5_DEG_TO_RAD;
    rig.block_h_max = 100.0 * S5_DEG_TO_RAD;
    rig.block_turns_max = 3U; /* out, further out, back: the next try is clear */
    (void)drive_round();      /* no line in this rig: it fails later, on the way back */
    CHECK(strstr(rig.all_messages, "abort=diagonal_blocked") == NULL);
    CHECK(strstr(rig.all_messages, "tries=2 ") != NULL);
    CHECK(rig.largest_distance > 0.2); /* drove the diagonal */

    /* Stopped from outside (the dashboard's STOP): held, but not a failure. */
    reset_rig();
    ladder_scan();
    rig.look_action = 1U;
    CHECK(!drive_round());
    CHECK(s5_task.phase == S5_PHASE_STOPPED && rig.status.obstacle.values[3] == S5_STATE_STOPPED);
    CHECK(strstr(rig.all_messages, "abort=stopped") != NULL);
    CHECK(strstr(rig.all_messages, "state=stopped") != NULL);
    CHECK(rig.turn_calls == 1U); /* no further tries after a stop */

    /* Lifted by Buddy 4: said so. */
    reset_rig();
    ladder_scan();
    rig.look_action = 1U;
    rig.status.terrain.available = true;
    rig.status.terrain.values[3] = (double)(TERRAIN_TS_LIFTED << 3U);
    CHECK(!drive_round());
    CHECK(s5_task.phase == S5_PHASE_STOPPED && strstr(rig.all_messages, "abort=lifted") != NULL);

    /* No line readings: no detour, and it says why. */
    reset_rig();
    ladder_scan();
    rig.status.line.available = false;
    CHECK(!drive_round());
    CHECK(rig.turn_calls == 0U && rig.status.motion.hold);
    CHECK(strstr(rig.all_messages, "abort=line_not_reporting") != NULL);

    /* No room either side: a result, not a failure. */
    reset_rig();
    ladder_scan();
    s5_scan.avoid.side = S5_AVOID_NONE;
    CHECK(!drive_round());
    CHECK(rig.turn_calls == 0U && s5_task.phase == S5_PHASE_IDLE);
    CHECK(strstr(rig.all_messages, "abort=no_way_around a=-1 b=-1") != NULL);
}

static void drive_into(double range_mm)
{
    reset_rig();
    rig.online = true;
    rig.status.motion.hold = false;
    rig.status.motion.state = DRIVING;
    rig.status.motion.stationary = false;
    rig.front_result = S5_US_OK;
    if (range_mm > 0.0) { echo_at(S5_ANGLE_CENTRE_DEG, range_mm); }
}

static void spurious_stop_tests(void)
{
    /* Something stops it and has gone by the scan: line following resumes. */
    drive_into(0.0);
    rig.vanish_on_stop = true;
    rig.stop_at = 40.0;
    if (setjmp(task_end) == 0) { obstacle_task(0, NULL); }
    CHECK(rig.stops == 1U && rig.releases == 1U);
    CHECK(!rig.status.motion.hold && s5_task.phase == S5_PHASE_IDLE && !s5_task.blocked);
    CHECK(strstr(rig.all_messages, "spurious_stop ahead_mm=-1 resumed=1") != NULL);
    CHECK(rig.turn_calls == 0U && rig.distance_calls == 0U); /* never drove round anything */

    /* Back as soon as it is released, in the same place: the second stop holds. */
    drive_into(0.0);
    rig.vanish_on_stop = true;
    rig.reappear_on_release = true;
    rig.stop_at = 80.0;
    if (setjmp(task_end) == 0) { obstacle_task(0, NULL); }
    CHECK(rig.releases == 1U && rig.status.motion.hold && s5_task.phase == S5_PHASE_FAILED);
    CHECK(strstr(rig.all_messages, "resumed=0") != NULL);
    CHECK(strstr(rig.all_messages, "abort=repeated_spurious") != NULL);

    /* Gone from straight ahead but still in the robot's path: does not resume. */
    drive_into(0.0);
    rig.vanish_on_stop = true;
    echo_at(70, 120.0);
    rig.stop_at = 60.0;
    if (setjmp(task_end) == 0) { obstacle_task(0, NULL); }
    CHECK(rig.status.motion.hold);
    CHECK(strstr(rig.all_messages, "spurious_stop") == NULL);
    CHECK(strstr(rig.all_messages, "abort=no_way_around") != NULL);
}

static void hump_tests(void)
{
    /* Climbing a hump, an echo at 140 mm (the floor, nose down) does not stop it ... */
    drive_into(140.0);
    rig.status.terrain.available = true;
    rig.status.terrain.values[3] = (double)(TERRAIN_TS_CLIMBING << 3U);
    rig.stop_at = 2.0;
    if (setjmp(task_end) == 0) { obstacle_task(0, NULL); }
    CHECK(rig.stops == 0U && !rig.status.motion.hold);
    /* ... but one nearer than S5_HUMP_STOP_MM still does ... */
    drive_into(90.0);
    rig.status.terrain.available = true;
    rig.status.terrain.values[3] = (double)(TERRAIN_TS_DESCENDING << 3U);
    rig.stop_at = 2.0;
    if (setjmp(task_end) == 0) { obstacle_task(0, NULL); }
    CHECK(rig.stops >= 1U);
    /* ... and on the level, 140 mm stops as always. */
    drive_into(140.0);
    rig.status.terrain.available = true;
    rig.status.terrain.values[3] = (double)(TERRAIN_TS_WAITING << 3U);
    rig.stop_at = 2.0;
    if (setjmp(task_end) == 0) { obstacle_task(0, NULL); }
    CHECK(rig.stops >= 1U);
}

int main(void)
{
    line_interrupt_tests();
    detour_sensor_tests();
    encoder_logging_tests();
    clearance_tests();
    scan_tests();
    report_buffer_test();
    pass_return_transition_tests();
    back_off_tests();
    stall_escape_tests();
    steeper_turn_tests();
    face_task_tests();
    known_obstacle_tests();
    unsure_look_tests();
    retry_ladder_tests();
    spurious_stop_tests();
    hump_tests();
    printf("%u/%u obstacle runtime checks passed\n", checks - failures, checks);
    return failures == 0U ? 0 : 1;
}
