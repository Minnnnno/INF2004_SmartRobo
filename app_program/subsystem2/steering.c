#include "steering.h"
#include "board_config.h"
#include <math.h>
#include <stddef.h>

#define DEG_TO_RAD (0.017453292519943295)

static bool geometry(const motion_config_t *c)
{
    return isfinite(c->track) && c->track > 0.0 &&
           isfinite(c->max_speed) && c->max_speed > 0.0;
}
static motion_result_t wheel_request(request_source_t source, uint32_t id,
                                     double left, double right)
{
    motion_config_t c = board_motion_config();
    double largest;
    if (!geometry(&c) || !isfinite(left) || !isfinite(right)) { return R_INVALID; }
    largest = fmax(fabs(left), fabs(right));
    if (largest > c.max_speed) {
        left *= c.max_speed / largest;
        right *= c.max_speed / largest; /* Preserve the requested curvature. */
    }
    return robot_submit(source, id, WHEEL_SPEED_TRACKING, 0.0, left, right);
}
motion_result_t motion_steer(request_source_t source, uint32_t id,
                            double speed, double turn_deg_s)
{
    motion_config_t c = board_motion_config();
    double correction;
    if (!geometry(&c) || !isfinite(speed) || !isfinite(turn_deg_s)) { return R_INVALID; }
    correction = turn_deg_s * DEG_TO_RAD * c.track / 2.0;
    return wheel_request(source, id, speed - correction, speed + correction);
}
motion_result_t motion_steer_left(request_source_t source, uint32_t id,
                                 double speed, double correction)
{
    if (!isfinite(speed) || !isfinite(correction) || correction < 0.0) { return R_INVALID; }
    return wheel_request(source, id, speed - correction, speed + correction);
}
motion_result_t motion_steer_right(request_source_t source, uint32_t id,
                                  double speed, double correction)
{
    if (!isfinite(speed) || !isfinite(correction) || correction < 0.0) { return R_INVALID; }
    return wheel_request(source, id, speed + correction, speed - correction);
}
static motion_result_t arc(request_source_t source, uint32_t id, double metres,
                            double degrees, double seconds, double speed)
{
    motion_config_t c = board_motion_config();
    double angle = degrees * DEG_TO_RAD;
    double goal[2], wheel[2], largest;
    if (!geometry(&c) || !isfinite(metres) || !isfinite(angle) || !isfinite(seconds) ||
        !isfinite(speed) || seconds < 0.0 || speed <= 0.0 || speed > c.max_speed ||
        fabs(metres) > c.max_distance || fabs(angle) > c.max_angle ||
        (metres == 0.0 && angle == 0.0)) { return R_INVALID; }
    goal[0] = metres - angle * c.track / 2.0;
    goal[1] = metres + angle * c.track / 2.0;
    if (seconds > 0.0) {
        if (seconds + c.command_lifetime >= c.movement_timeout) { return R_INVALID; }
        for (unsigned w = 0U; w < 2U; ++w) {
            wheel[w] = goal[w] / seconds;
            if (1.5 * fabs(wheel[w]) > c.max_speed ||
                6.0 * fabs(goal[w]) / (seconds * seconds) > c.acceleration) { return R_INVALID; }
        }
    } else {
        if (metres == 0.0) { return R_INVALID; }
        wheel[0] = goal[0] * speed / fabs(metres);
        wheel[1] = goal[1] * speed / fabs(metres);
        largest = fmax(fabs(wheel[0]), fabs(wheel[1]));
        if (largest > c.max_speed) {
            wheel[0] *= c.max_speed / largest;
            wheel[1] *= c.max_speed / largest;
        }
    }
    return robot_submit_arc(source, id, metres, angle, seconds, wheel[0], wheel[1]);
}
motion_result_t motion_steer_distance(request_source_t source, uint32_t id,
                                     double metres, double degrees, double speed)
{ return arc(source, id, metres, degrees, 0.0, speed); }
motion_result_t motion_steer_time(request_source_t source, uint32_t id,
                                 double seconds, double degrees, double speed)
{
    if (!isfinite(seconds) || seconds <= 0.0 || !isfinite(speed) || speed == 0.0) { return R_INVALID; }
    return arc(source, id, speed * seconds, degrees, seconds, fabs(speed));
}
bool motion_get_feedback(motion_feedback_t *feedback)
{
    robot_status_t status;
    motion_config_t c = board_motion_config();
    if (feedback == NULL || !geometry(&c) || !motion_get_status(&status)) { return false; }
    *feedback = (motion_feedback_t){
        .timestamp_s=status.motion.sample_time,
        .left_speed_m_s=status.motion.speed[0], .right_speed_m_s=status.motion.speed[1],
        .centre_speed_m_s=(status.motion.speed[0] + status.motion.speed[1]) / 2.0,
        .turn_deg_s=(status.motion.speed[1] - status.motion.speed[0]) / c.track / DEG_TO_RAD,
        .left_metres=status.motion.travel[0], .right_metres=status.motion.travel[1],
        .centre_metres=status.motion.centre, .path_metres=status.motion.path,
        .heading_deg=status.motion.heading / DEG_TO_RAD,
        .state=status.motion.state, .fault=status.motion.fault,
        .feedback_trusted=status.motion.feedback_trusted && motion_config_valid(&c)};
    return true;
}
line_direction_t motion_line_direction(line_steering_t *state, bool left, bool right)
{
    unsigned observed = (left ? 1U : 0U) | (right ? 2U : 0U);
    static const line_direction_t directions[] = {LINE_LOST, LINE_LEFT, LINE_RIGHT, LINE_STRAIGHT};
    if (state == NULL) { return LINE_WAIT; }
    if (observed != state->candidate) { state->candidate = observed; state->consecutive = 1U; }
    else if (state->consecutive < 2U) { ++state->consecutive; }
    if (state->consecutive >= 2U) { state->direction = directions[observed]; }
    return state->direction;
}
