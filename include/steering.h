#ifndef ROBO_STEERING_H
#define ROBO_STEERING_H
#include "robot.h"

/* Task-context requests through the Vehicle Controller. Angles are degrees.
 * Positive heading/turn rate means left. Use SOURCE_LINE or SOURCE_OBSTACLE.
 * Continuous commands: refresh the same nonzero ID every 20-50 ms.
 * Finite commands: submit once, then wait for matching completion/fault. */
motion_result_t motion_steer(request_source_t source, uint32_t id,
                            double speed_m_s, double turn_deg_s);
motion_result_t motion_steer_left(request_source_t source, uint32_t id,
                                 double speed_m_s, double correction_m_s);
motion_result_t motion_steer_right(request_source_t source, uint32_t id,
                                  double speed_m_s, double correction_m_s);
motion_result_t motion_steer_distance(request_source_t source, uint32_t id,
                                     double metres, double heading_deg, double speed_m_s);
/* speed_m_s is average centre speed; timed arcs use smooth start/end ramps.
 * Physically impossible speed/acceleration/time combinations are rejected. */
motion_result_t motion_steer_time(request_source_t source, uint32_t id,
                                 double seconds, double heading_deg, double speed_m_s);

typedef struct {
    double timestamp_s, left_speed_m_s, right_speed_m_s, centre_speed_m_s;
    double turn_deg_s, left_metres, right_metres, centre_metres, path_metres, heading_deg;
    motion_state_t state;
    motion_fault_t fault;
    bool feedback_trusted;
} motion_feedback_t;
/* Encoder estimates, not a ground-truth slip measurement. Distances are
 * cumulative since init/reset. Save a snapshot and subtract to measure a run. */
bool motion_get_feedback(motion_feedback_t *feedback);

typedef enum { LINE_WAIT, LINE_LOST, LINE_STRAIGHT, LINE_LEFT, LINE_RIGHT } line_direction_t;
typedef struct { unsigned candidate, consecutive; line_direction_t direction; } line_steering_t;
/* Two consecutive equal samples filter DO chatter. Pass true for black.
 * A zero-initialized state returns LINE_WAIT until a stable sample pair. */
line_direction_t motion_line_direction(line_steering_t *state, bool left_black, bool right_black);
#endif
