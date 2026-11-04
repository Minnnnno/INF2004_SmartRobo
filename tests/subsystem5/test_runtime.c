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
ER comms_send_telemetry(const char *name, const char *payload);
void comms_latch_event(UB events);
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
    double now, stop_at;
    s5_us_status_t side_result, front_result;
    unsigned moves, pings, servo_moves, releases, distance_calls, turn_calls;
    unsigned busy_distance_call, busy_turn_call, retry_action;
    int angle;
    bool status_missing, refuse_stop, release_during_servo, release_during_ping;
    bool online, always_busy, lose_status_after_completion, detect_line_on_next_distance;
    unsigned events, oversized_formats;
    UB latched_events;
    char messages[2][COMMS_PAYLOAD_MAX];
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
    rig.angle = S5_ANGLE_CENTRE_DEG;
    rig.front_result = S5_US_NO_ECHO;
    rig.side_result = S5_US_NO_ECHO;
    s5_blocked = false;
    s5_decided = false;
    s5_sensor_ok = false;
    s5_phase = S5_PHASE_IDLE;
}

ER tk_dly_tsk(RELTIM delay)
{
    rig.now += (double)delay / 1000.0;
    rig.status.line.timestamp = rig.now;
    if (rig.retry_action != 0U && rig.turn_calls == 1U && rig.busy_turn_call == 1U) {
        if (rig.retry_action == 1U) { rig.status.motion.hold = true; }
        if (rig.retry_action == 2U) { ++rig.status.motion.generation; }
        if (rig.retry_action == 3U) { rig.status.motion.fault = F_FEEDBACK; }
        if (rig.retry_action == 4U) { rig.status.motion.state = DRIVING; }
        rig.retry_action = 0U;
    }
    if (rig.stop_at > 0.0 && rig.now >= rig.stop_at) { longjmp(task_end, 1); }
    return 0;
}
double motion_hal_time(void) { return rig.now; }
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
    rig.status.motion.hold = true;
    rig.status.motion.stationary = !rig.refuse_stop;
    rig.status.motion.state = rig.refuse_stop ? STOPPING : STOPPED;
}
void motion_controller_release(bool checks_ok)
{
    ++rig.releases;
    if (checks_ok && rig.status.motion.stationary) {
        rig.status.motion.hold = false;
        rig.status.motion.state = READY;
    }
}
static motion_result_t finish_move(uint32_t id, double metres, double degrees)
{
    ++rig.moves;
    rig.status.motion.id = id;
    rig.status.motion.centre += metres;
    rig.status.motion.heading += degrees * S5_DEG_TO_RAD;
    rig.status.last_event = (motion_event_t){EV_COMPLETE, id, 0U, F_NONE};
    return R_OK;
}
motion_result_t motion_steer_distance(request_source_t source, uint32_t id,
                                     double metres, double degrees, double speed)
{
    (void)source; (void)speed;
    ++rig.distance_calls;
    if (rig.always_busy || rig.distance_calls == rig.busy_distance_call) { return R_BUSY; }
    if (rig.detect_line_on_next_distance) {
        rig.detect_line_on_next_distance = false;
        /* Starting 190 mm to the side at 45 deg, the front IR reaches the
         * line after about 188.7 mm. The requested leg has not completed. */
        (void)finish_move(id, 0.1887, 0.0);
        rig.status.last_event.id = 0U;
        rig.status.motion.state = DRIVING;
        rig.status.motion.stationary = false;
        rig.status.line.values[1] = 1.0;
        return R_OK;
    }
    return finish_move(id, metres, degrees);
}
motion_result_t motion_turn_degrees(request_source_t source, uint32_t id, double degrees)
{
    (void)source;
    ++rig.turn_calls;
    if (rig.always_busy || rig.turn_calls == rig.busy_turn_call) { return R_BUSY; }
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
}
int s5_servo_angle(void) { return rig.angle; }
bool s5_us_init(void) { return true; }
s5_us_status_t s5_us_ping(double *range, double *time)
{
    ++rig.pings;
    (void)tk_dly_tsk(S5_US_CYCLE_MS);
    *time = rig.now;
    *range = 100.0;
    if (rig.release_during_ping) {
        rig.status.motion.hold = false;
        rig.status.motion.stationary = false;
        rig.status.motion.state = DRIVING;
    }
    return rig.angle == S5_ANGLE_CENTRE_DEG ? rig.front_result : rig.side_result;
}
bool comms_is_online(void) { return rig.online; }
void comms_latch_event(UB events) { rig.latched_events |= events; }
ER comms_send_telemetry(const char *name, const char *payload)
{
    CHECK(strcmp(name, "obstacle") == 0);
    CHECK(strlen(payload) < COMMS_PAYLOAD_MAX);
    if (rig.events < 2U) {
        (void)snprintf(rig.messages[rig.events], COMMS_PAYLOAD_MAX, "%s", payload);
    }
    ++rig.events;
    return 0;
}
INT tm_printf(const UB *format, ...) { (void)format; return 0; }
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

static void report_buffer_test(void)
{
    static s5_scan_t scan;
    reset_rig();
    rig.online = true;
    memset(&scan, 0, sizeof(scan));
    scan.b_valid = true;
    scan.width_mm = 160.0;
    scan.offset_mm = -15.0;
    scan.centre.estimate.range_mm = 153.0;
    scan.edge[S5_LEFT].b_found = scan.edge[S5_RIGHT].b_found = true;
    scan.placement.position = S5_POSITION_CENTRE;
    scan.avoid.side = S5_AVOID_RIGHT;
    scan.avoid.shift_mm[S5_LEFT] = 180.0;
    scan.avoid.shift_mm[S5_RIGHT] = 210.0;
    report_scan(&scan);
    CHECK(rig.oversized_formats == 0U);
    CHECK(rig.events == 2U);
    CHECK(strstr(rig.messages[0], "width_mm=160") != NULL);
    CHECK(strstr(rig.messages[1], "avoid=right") != NULL);
    CHECK(rig.latched_events == TF_EV_SCAN_DONE);

    reset_rig(); /* An offline scan still latches the binary event for reconnect. */
    report_scan(&scan);
    CHECK(rig.events == 0U);
    CHECK(rig.latched_events == TF_EV_SCAN_DONE);

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
        CHECK(rig.moves == 3U); /* No forward step after the failed side look. */

        reset_rig();
        rig.side_result = S5_US_OK;
        rig.front_result = faults[i];
        CHECK(!s5_detour_run(&plan));
        CHECK(rig.moves == 3U); /* No forward step after a failed front look. */
    }
    reset_rig();
    CHECK(!s5_detour_run(&plan)); /* No line in this rig: return eventually fails. */
    CHECK(rig.moves > 3U);       /* A normal no-echo still permits passing. */

    reset_rig();
    rig.front_result = S5_US_OK;
    /* Raw 100 mm = 130 mm from the axis. Required room is travel + 90 mm. */
    CHECK(path_clear(39.0));
    CHECK(!path_clear(40.0));
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
    rig.status.motion.state = STOPPING;
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
        /* First 50 mm step succeeds; the rear-clearance submission collides
         * once with the status mutex. No command was queued on R_BUSY. */
        rig.busy_distance_call = 2U;
        CHECK(pass_obstacle(&p));
        CHECK(rig.distance_calls == 3U && rig.moves == 2U);
        CHECK(s5_x > 600.0);
        /* Same transient collision when starting the inward turn. */
        rig.busy_turn_call = 1U;
        CHECK(turn_to(side == 0U ? S5_RETURN_DEG : -S5_RETURN_DEG) == S5_MOVE_DONE);
        CHECK(rig.turn_calls == 2U && rig.moves == 3U);
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
        CHECK(rig.moves == 4U && rig.releases == 1U);
        CHECK(fabs(s5_y) < 0.1 && fabs(rig.status.motion.heading) < 1e-9);
        CHECK(!rig.status.motion.hold && rig.status.motion.state == READY);
    }
}

int main(void)
{
    detour_sensor_tests();
    scan_tests();
    report_buffer_test();
    pass_return_transition_tests();
    printf("%u/%u obstacle runtime checks passed\n", checks - failures, checks);
    return failures == 0U ? 0 : 1;
}
