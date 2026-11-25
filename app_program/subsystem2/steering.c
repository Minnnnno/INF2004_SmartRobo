#include "../../include/float_types.h"
#include "steering.h"
#include "board_config.h"
#include <math.h>
#include <stddef.h>

#define DEG_TO_RAD (0.017453292519943295)

static bool
geometry (const motion_config_t * p_config)
{
    return (isfinite(p_config->track)) && (p_config->track > 0.0)
           && (isfinite(p_config->max_speed)) && (p_config->max_speed > 0.0);
}

motion_result_t
motion_move_mm (request_source_t source, uint32_t command_id,
                float64_t distance_mm)
{
    motion_config_t config = board_motion_config();

    if ((!isfinite(distance_mm))
        || (fabs(distance_mm / 1000.0) > config.max_distance))
    {
        return R_INVALID;
    }

    return robot_submit(source, command_id, DISTANCE_MOVE, distance_mm / 1000.0,
                        0.0, 0.0);
}

motion_result_t
motion_turn_degrees (request_source_t source, uint32_t command_id,
                     float64_t degrees)
{
    motion_config_t config  = board_motion_config();
    float64_t       radians = degrees * DEG_TO_RAD;

    if ((!isfinite(radians)) || (fabs(radians) > config.max_angle))
    {
        return R_INVALID;
    }

    return robot_submit(source, command_id, ENCODER_TURN, radians, 0.0, 0.0);
}

static motion_result_t
wheel_request (request_source_t source, uint32_t command_id, float64_t left,
               float64_t right)
{
    motion_config_t config = board_motion_config();
    float64_t       largest;

    if ((!geometry(&config)) || (!isfinite(left)) || (!isfinite(right)))
    {
        return R_INVALID;
    }

    largest = fmax(fabs(left), fabs(right));

    if (largest > config.max_speed)
    {
        left *= config.max_speed / largest;
        /* Preserve the requested curvature. */
        right *= config.max_speed / largest;
    }

    return robot_submit(source, command_id, WHEEL_SPEED_TRACKING, 0.0, left,
                        right);
}

motion_result_t
motion_set_wheel_speeds_mm_s (request_source_t source, uint32_t command_id,
                              float64_t left, float64_t right)
{
    return wheel_request(source, command_id, left / 1000.0, right / 1000.0);
}

motion_result_t
motion_steer (request_source_t source, uint32_t command_id, float64_t speed,
              float64_t turn_deg_s)
{
    motion_config_t config = board_motion_config();
    float64_t       correction;

    if ((!geometry(&config)) || (!isfinite(speed)) || (!isfinite(turn_deg_s)))
    {
        return R_INVALID;
    }

    correction = (turn_deg_s * DEG_TO_RAD * config.track) / 2.0;
    return wheel_request(source, command_id, speed - correction,
                         speed + correction);
}

motion_result_t
motion_steer_left (request_source_t source, uint32_t command_id,
                   float64_t speed, float64_t correction)
{
    if ((!isfinite(speed)) || (!isfinite(correction)) || (correction < 0.0))
    {
        return R_INVALID;
    }

    return wheel_request(source, command_id, speed - correction,
                         speed + correction);
}

motion_result_t
motion_steer_right (request_source_t source, uint32_t command_id,
                    float64_t speed, float64_t correction)
{
    if ((!isfinite(speed)) || (!isfinite(correction)) || (correction < 0.0))
    {
        return R_INVALID;
    }

    return wheel_request(source, command_id, speed + correction,
                         speed - correction);
}

static motion_result_t
arc (request_source_t source, uint32_t command_id, float64_t metres,
     float64_t degrees, float64_t seconds, float64_t speed)
{
    motion_config_t config = board_motion_config();
    float64_t       angle  = degrees * DEG_TO_RAD;

    float64_t goal[2];
    float64_t wheel[2];
    float64_t largest;

    if ((!geometry(&config)) || (!isfinite(metres)) || (!isfinite(angle))
        || (!isfinite(seconds)) || (!isfinite(speed)) || (seconds < 0.0)
        || (speed <= 0.0) || (speed > config.max_speed)
        || (fabs(metres) > config.max_distance)
        || (fabs(angle) > config.max_angle)
        || ((0.0 == metres) && (0.0 == angle)))
    {
        return R_INVALID;
    }

    goal[0] = metres - ((angle * config.track) / 2.0);
    goal[1] = metres + ((angle * config.track) / 2.0);

    if (seconds > 0.0)
    {
        if ((seconds + config.command_lifetime) >= config.movement_timeout)
        {
            return R_INVALID;
        }

        for (unsigned wheel_index = 0U; wheel_index < 2U; ++wheel_index)
        {
            wheel[wheel_index] = goal[wheel_index] / seconds;

            if (((1.5 * fabs(wheel[wheel_index])) > config.max_speed)
                || (((6.0 * fabs(goal[wheel_index])) / (seconds * seconds))
                    > config.acceleration))
            {
                return R_INVALID;
            }
        }
    }
    else
    {
        if (0.0 == metres)
        {
            return R_INVALID;
        }

        wheel[0] = (goal[0] * speed) / fabs(metres);
        wheel[1] = (goal[1] * speed) / fabs(metres);
        largest  = fmax(fabs(wheel[0]), fabs(wheel[1]));

        if (largest > config.max_speed)
        {
            wheel[0] *= config.max_speed / largest;
            wheel[1] *= config.max_speed / largest;
        }
    }

    return robot_submit_arc(source, command_id, metres, angle, seconds,
                            wheel[0], wheel[1]);
}

motion_result_t
motion_steer_distance (request_source_t source, uint32_t command_id,
                       float64_t metres, float64_t degrees, float64_t speed)
{
    return arc(source, command_id, metres, degrees, 0.0, speed);
}

motion_result_t
motion_steer_time (request_source_t source, uint32_t command_id,
                   float64_t seconds, float64_t degrees, float64_t speed)
{
    if ((!isfinite(seconds)) || (seconds <= 0.0) || (!isfinite(speed))
        || (0.0 == speed))
    {
        return R_INVALID;
    }

    return arc(source, command_id, speed * seconds, degrees, seconds,
               fabs(speed));
}

bool
motion_get_feedback (motion_feedback_t * p_feedback)
{
    robot_status_t  status;
    motion_config_t config = board_motion_config();

    if ((NULL == p_feedback) || (!geometry(&config))
        || (!motion_get_status(&status)))
    {
        return false;
    }

    *p_feedback = (motion_feedback_t){
        .timestamp_s     = status.motion.sample_time,
        .left_speed_m_s  = status.motion.speed[0],
        .right_speed_m_s = status.motion.speed[1],
        .centre_speed_m_s
        = (status.motion.speed[0] + status.motion.speed[1]) / 2.0,
        .turn_deg_s    = (status.motion.speed[1] - status.motion.speed[0])
                         / config.track / DEG_TO_RAD,
        .left_metres   = status.motion.travel[0],
        .right_metres  = status.motion.travel[1],
        .centre_metres = status.motion.centre,
        .path_metres   = status.motion.path,
        .heading_deg   = status.motion.heading / DEG_TO_RAD,
        .state         = status.motion.state,
        .fault         = status.motion.fault,
        .feedback_trusted
        = (status.motion.feedback_trusted) && (motion_config_valid(&config))};
    return true;
}

bool
motion_get_measurements_mm (motion_measurements_mm_t * p_measurements)
{
    motion_feedback_t feedback;

    if ((NULL == p_measurements) || (!motion_get_feedback(&feedback)))
    {
        return false;
    }

    *p_measurements = (motion_measurements_mm_t){
        .timestamp_s        = feedback.timestamp_s,
        .left_speed_mm_s    = feedback.left_speed_m_s * 1000.0,
        .right_speed_mm_s   = feedback.right_speed_m_s * 1000.0,
        .centre_speed_mm_s  = feedback.centre_speed_m_s * 1000.0,
        .left_distance_mm   = feedback.left_metres * 1000.0,
        .right_distance_mm  = feedback.right_metres * 1000.0,
        .centre_distance_mm = feedback.centre_metres * 1000.0,
        .path_distance_mm   = feedback.path_metres * 1000.0,
        .heading_deg        = feedback.heading_deg,
        .state              = feedback.state,
        .fault              = feedback.fault,
        .feedback_trusted   = feedback.feedback_trusted};
    return true;
}

line_direction_t
motion_line_direction (line_steering_t * p_state, bool b_left, bool b_right)
{
    unsigned observed = (b_left ? 1U : 0U) | (b_right ? 2U : 0U);
    static const line_direction_t directions[]
        = {LINE_LOST, LINE_LEFT, LINE_RIGHT, LINE_STRAIGHT};

    if (NULL == p_state)
    {
        return LINE_WAIT;
    }

    if (observed != p_state->candidate)
    {
        p_state->candidate   = observed;
        p_state->consecutive = 1U;
    }
    else if (p_state->consecutive < 2U)
    {
        ++p_state->consecutive;
    }

    if (p_state->consecutive >= 2U)
    {
        p_state->direction = directions[observed];
    }

    return p_state->direction;
}

/*** end of file ***/
