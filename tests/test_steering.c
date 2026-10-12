#include "steering.h"
#include "board_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); exit(1); } } while (0)
#define NEAR(a,b) CHECK(fabs((a)-(b)) < 1e-9)
static unsigned checks, calls;
static robot_request_t requested;
static robot_status_t snapshot;
static bool snapshot_available = true;
static motion_result_t result = R_OK;
motion_result_t robot_submit(request_source_t source, uint32_t id, motion_mode_t mode,
                             double value, double left, double right)
{
    ++calls;
    requested = (robot_request_t){.source=source, .command={.id=id, .mode=mode,
        .value=value, .speed={left,right}}};
    return result;
}
motion_result_t robot_submit_arc(request_source_t source, uint32_t id, double metres,
                                 double radians, double seconds, double left, double right)
{
    (void)robot_submit(source, id, ARC_MOVE, metres, left, right);
    requested.command.angle = radians;
    requested.command.duration = seconds;
    return result;
}
bool motion_get_status(robot_status_t *status)
{ if (!snapshot_available) { return false; } *status = snapshot; return true; }
int main(void)
{
    motion_config_t c = board_motion_config();
    const double radians = 0.17453292519943295;
    motion_feedback_t feedback;
    line_steering_t line = {0};
    CHECK(motion_steer(SOURCE_LINE, 3001U, 0.08, 0.0) == R_OK);
    CHECK(requested.source == SOURCE_LINE && requested.command.id == 3001U);
    CHECK(requested.command.mode == WHEEL_SPEED_TRACKING);
    NEAR(requested.command.speed[0], 0.08); NEAR(requested.command.speed[1], 0.08);
    CHECK(motion_steer(SOURCE_OBSTACLE, 5001U, 0.08, 10.0) == R_OK);
    CHECK(requested.source == SOURCE_OBSTACLE && requested.command.id == 5001U);
    NEAR(requested.command.speed[0], 0.08 - radians*c.track/2.0);
    NEAR(requested.command.speed[1], 0.08 + radians*c.track/2.0);
    CHECK(motion_steer(SOURCE_LINE, 3001U, 0.08, -10.0) == R_OK);
    CHECK(requested.command.speed[0] > requested.command.speed[1]);
    CHECK(motion_steer_left(SOURCE_LINE, 3001U, 0.08, 0.01) == R_OK);
    NEAR(requested.command.speed[0], 0.07); NEAR(requested.command.speed[1], 0.09);
    CHECK(motion_steer_right(SOURCE_LINE, 3001U, 0.08, 0.01) == R_OK);
    NEAR(requested.command.speed[0], 0.09); NEAR(requested.command.speed[1], 0.07);
    CHECK(motion_steer_left(SOURCE_LINE, 3001U, 0.25, 0.10) == R_OK);
    NEAR(requested.command.speed[0]/requested.command.speed[1], 0.15/0.35);
    NEAR(requested.command.speed[1], c.max_speed);
    unsigned before = calls;
    CHECK(motion_steer(SOURCE_LINE, 1U, NAN, 0.0) == R_INVALID);
    CHECK(motion_steer_left(SOURCE_LINE, 1U, 0.08, -0.01) == R_INVALID);
    CHECK(motion_steer_right(SOURCE_LINE, 1U, 0.08, INFINITY) == R_INVALID);
    CHECK(calls == before);
    result = R_HELD;
    CHECK(motion_steer(SOURCE_LINE, 1U, 0.08, 0.0) == R_HELD);
    result = R_BUSY;
    CHECK(motion_steer_distance(SOURCE_OBSTACLE, 5U, 0.20, 10.0, 0.08) == R_BUSY);
    result = R_OK;
    CHECK(motion_steer_distance(SOURCE_OBSTACLE, 5U, 0.20, 10.0, 0.08) == R_OK);
    CHECK(requested.command.mode == ARC_MOVE && requested.source == SOURCE_OBSTACLE);
    NEAR(requested.command.value, 0.20); NEAR(requested.command.angle, radians);
    NEAR(requested.command.duration, 0.0);
    NEAR(requested.command.speed[0], (0.20-radians*c.track/2.0)*0.08/0.20);
    NEAR(requested.command.speed[1], (0.20+radians*c.track/2.0)*0.08/0.20);
    CHECK(motion_steer_distance(SOURCE_LINE, 6U, -0.20, -10.0, 0.08) == R_OK);
    CHECK(requested.command.speed[0] < 0.0 && requested.command.speed[1] < 0.0);
    CHECK(motion_steer_time(SOURCE_OBSTACLE, 7U, 2.0, -10.0, 0.08) == R_OK);
    NEAR(requested.command.value, 0.16); NEAR(requested.command.duration, 2.0);
    NEAR(requested.command.angle, -radians);
    CHECK(motion_steer_time(SOURCE_LINE, 8U, 2.0, 10.0, -0.08) == R_OK);
    CHECK(requested.command.speed[0] < 0.0 && requested.command.speed[1] < 0.0);
    before = calls;
    CHECK(motion_steer_time(SOURCE_LINE, 1U, 0.1, 90.0, 0.08) == R_INVALID);
    CHECK(motion_steer_time(SOURCE_LINE, 1U, 0.0, 10.0, 0.08) == R_INVALID);
    CHECK(motion_steer_time(SOURCE_LINE, 1U, 30.0, 10.0, 0.08) == R_INVALID);
    CHECK(motion_steer_distance(SOURCE_LINE, 1U, 6.0, 10.0, 0.08) == R_INVALID);
    CHECK(motion_steer_distance(SOURCE_LINE, 1U, 0.2, 361.0, 0.08) == R_INVALID);
    CHECK(motion_steer_distance(SOURCE_LINE, 1U, 0.2, NAN, 0.08) == R_INVALID);
    CHECK(motion_steer_distance(SOURCE_LINE, 1U, 0.0, 10.0, 0.08) == R_INVALID);
    CHECK(calls == before);
    snapshot.motion = (motion_status_t){.sample_time=5.0, .speed={0.10,0.20},
        .travel={1.0,1.1}, .centre=1.05, .path=2.0, .heading=radians,
        .state=DRIVING, .feedback_trusted=true};
    CHECK(motion_get_feedback(&feedback));
    NEAR(feedback.timestamp_s, 5.0); NEAR(feedback.centre_speed_m_s, 0.15);
    NEAR(feedback.left_metres, 1.0); NEAR(feedback.right_metres, 1.1);
    NEAR(feedback.centre_metres, 1.05); NEAR(feedback.path_metres, 2.0);
    NEAR(feedback.heading_deg, 10.0);
    NEAR(feedback.turn_deg_s, 0.10/c.track/0.017453292519943295);
    CHECK(feedback.feedback_trusted && feedback.state == DRIVING);
    snapshot.motion.feedback_trusted=false; snapshot.motion.fault=F_FEEDBACK;
    CHECK(motion_get_feedback(&feedback) && !feedback.feedback_trusted && feedback.fault == F_FEEDBACK);
    CHECK(!motion_get_feedback(NULL)); snapshot_available=false;
    CHECK(!motion_get_feedback(&feedback));
    CHECK(motion_line_direction(NULL, true, true) == LINE_WAIT);
    CHECK(motion_line_direction(&line, true, true) == LINE_WAIT);
    CHECK(motion_line_direction(&line, true, true) == LINE_STRAIGHT);
    CHECK(motion_line_direction(&line, true, false) == LINE_STRAIGHT);
    CHECK(motion_line_direction(&line, true, false) == LINE_LEFT);
    CHECK(motion_line_direction(&line, false, true) == LINE_LEFT);
    CHECK(motion_line_direction(&line, false, true) == LINE_RIGHT);
    CHECK(motion_line_direction(&line, false, false) == LINE_RIGHT);
    CHECK(motion_line_direction(&line, false, true) == LINE_RIGHT); /* Single dropout filtered. */
    CHECK(motion_line_direction(&line, false, false) == LINE_RIGHT);
    CHECK(motion_line_direction(&line, false, false) == LINE_LOST);
    printf("PASS: %u steering API and line-policy checks\n", checks);
    return 0;
}
