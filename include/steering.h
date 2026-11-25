/** @file steering.h
 *
 * @brief Steering requests.
 */

#ifndef ROBO_STEERING_H
#define ROBO_STEERING_H
#include "float_types.h"
#include "robot.h"

/*! @brief Buddy task API: millimetres, mm/s, degrees. Task context only.
 * Finite moves submit once; positive distance=forward, positive angle=left.
 * Turn is an in-place robot heading change, not motor-shaft rotation.
 * Motion Control chooses/ramp-limits finite-move speed from configuration. */
motion_result_t motion_move_mm(request_source_t source, uint32_t command_id,
                               float64_t distance_mm);
motion_result_t motion_turn_degrees(request_source_t source,
                                    uint32_t command_id, float64_t degrees);
/*! @brief Refresh the same nonzero ID every 20-50 ms. Targets are scaled
 * together to the configured wheel-speed limit, preserving curvature. */
motion_result_t motion_set_wheel_speeds_mm_s(request_source_t source,
                                             uint32_t         command_id,
                                             float64_t        left_mm_s,
                                             float64_t        right_mm_s);

typedef struct
{
    float64_t timestamp_s;

    float64_t      left_speed_mm_s;
    float64_t      right_speed_mm_s;
    float64_t      centre_speed_mm_s;
    float64_t      left_distance_mm;
    float64_t      right_distance_mm;
    float64_t      centre_distance_mm;
    float64_t      path_distance_mm;
    float64_t      heading_deg;
    motion_state_t state;
    motion_fault_t fault;
    bool           feedback_trusted;
} motion_measurements_mm_t;

/*! @brief One synchronized encoder-based snapshot. Returns false if
 * unavailable. Distances accumulate since init/reset: subtract snapshots for a
 * run. Centre distance is signed; path distance accumulates forward/reverse
 * travel. Check feedback_trusted and fault before using measurements for
 * navigation. */
bool motion_get_measurements_mm(motion_measurements_mm_t * p_measurements);

/*! @brief Task-context requests through the Vehicle Controller. Angles are
 * degrees. Positive heading/turn rate means left. Use SOURCE_LINE or
 * SOURCE_OBSTACLE. Continuous commands: refresh the same nonzero ID every 20-50
 * ms. Finite commands: submit once, then wait for matching completion/fault. */
motion_result_t motion_steer(request_source_t source, uint32_t command_id,
                             float64_t speed_m_s, float64_t turn_deg_s);
motion_result_t motion_steer_left(request_source_t source, uint32_t command_id,
                                  float64_t speed_m_s,
                                  float64_t correction_m_s);
motion_result_t motion_steer_right(request_source_t source, uint32_t command_id,
                                   float64_t speed_m_s,
                                   float64_t correction_m_s);
motion_result_t motion_steer_distance(request_source_t source,
                                      uint32_t command_id, float64_t metres,
                                      float64_t heading_deg,
                                      float64_t speed_m_s);
/*! @brief speed_m_s is average centre speed; timed arcs use smooth start/end
 * ramps. Physically impossible speed/acceleration/time combinations are
 * rejected. */
motion_result_t motion_steer_time(request_source_t source, uint32_t command_id,
                                  float64_t seconds, float64_t heading_deg,
                                  float64_t speed_m_s);

typedef struct
{
    float64_t      timestamp_s;
    float64_t      left_speed_m_s;
    float64_t      right_speed_m_s;
    float64_t      centre_speed_m_s;
    float64_t      turn_deg_s;
    float64_t      left_metres;
    float64_t      right_metres;
    float64_t      centre_metres;
    float64_t      path_metres;
    float64_t      heading_deg;
    motion_state_t state;
    motion_fault_t fault;
    bool           feedback_trusted;
} motion_feedback_t;

/*! @brief Encoder estimates, not a ground-truth slip measurement. Distances are
 * cumulative since init/reset. Save a snapshot and subtract to measure a run.
 */
bool motion_get_feedback(motion_feedback_t * p_feedback);

typedef enum
{
    LINE_WAIT,
    LINE_LOST,
    LINE_STRAIGHT,
    LINE_LEFT,
    LINE_RIGHT
} line_direction_t;

typedef struct
{
    unsigned         candidate;
    unsigned         consecutive;
    line_direction_t direction;
} line_steering_t;

/*! @brief Two consecutive equal samples filter DO chatter. Pass true for black.
 * A zero-initialized state returns LINE_WAIT until a stable sample pair. */
line_direction_t motion_line_direction(line_steering_t * p_state,
                                       bool b_left_black, bool b_right_black);
#endif

/*** end of file ***/
