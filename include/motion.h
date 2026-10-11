#ifndef ROBO_MOTION_H
#define ROBO_MOTION_H

#include <stdbool.h>
#include <stdint.h>

/* All distances metres, speeds m/s, angles radians, times seconds. */
typedef enum { INITIALISING, READY, DRIVING, STOPPING, STOPPED, FAULT, SHUTDOWN } motion_state_t;
typedef enum { DISTANCE_MOVE, ENCODER_TURN, WHEEL_SPEED_TRACKING, MANUAL_OUTPUT } motion_mode_t;
typedef enum { F_NONE, F_CONFIG, F_SETUP, F_TIMING, F_FEEDBACK, F_DEADLINE,
               F_EXPIRED, F_STOP_TIMEOUT, F_FINAL_GOAL, F_EVENT_OVERFLOW, F_SYSTEM } motion_fault_t;
typedef enum { R_OK, R_INVALID, R_BUSY, R_HELD, R_STALE, R_FULL } motion_result_t;
typedef enum { EV_COMPLETE, EV_CANCELLED, EV_FAULT, EV_STOPPED, EV_REJECTED } motion_event_type_t;
typedef struct { double kp, ki, kd, derivative_tau; } pid_config_t;
typedef struct { double integral, previous, derivative; bool seeded; } pid_t;
typedef struct {
    double counts_per_rev[2], diameter[2], track;
    int motor_sign[2], encoder_sign[2];
    bool quadrature[2]; /* x4 A/B; single channel counts A rising edges. */
    double max_speed, acceleration, max_output, period, max_dt;
    double distance_tolerance, angle_tolerance, stopped_speed, settle_time;
    double startup_grace, feedback_timeout, stop_timeout, setup_timeout;
    double command_lifetime, movement_timeout, max_distance, max_angle;
    double straight_gain, reversal_time, manual_limit, manual_timeout;
    pid_config_t pid[2];
    unsigned pwm_hz;
    bool hardware_confirmed, brake, manual_enabled;
} motion_config_t;
typedef struct {
    uint32_t id, generation;
    motion_mode_t mode;
    double value, speed[2], expires, deadline;
} motion_command_t;
typedef struct {
    uint32_t count[2], invalid[2];
    bool reliable;
} encoder_sample_t;
typedef struct {
    uint32_t previous[2];
    double travel[2], speed[2], centre, path, heading;
} odometry_t;
typedef struct {
    motion_event_type_t type;
    uint32_t id, generation;
    motion_fault_t fault;
} motion_event_t;
typedef struct {
    motion_state_t state;
    motion_mode_t mode;
    motion_fault_t fault;
    uint32_t id, generation;
    double speed[2], travel[2], output[2], target[2], centre, path, heading;
    bool stationary, feedback_trusted, inhibited, hold;
} motion_status_t;
typedef struct {
    motion_config_t config;
    motion_status_t status;
    odometry_t odom;
    pid_t pid[2];
    motion_command_t command;
    uint32_t start_count[2]; /* Raw modular counts captured on command entry. */
    double start[2], last_time, entered, last_progress[2], expected_since[2], still_since;
    uint32_t invalid[2];
    bool active, still_tracking, goal_pending, feedback_latched;
    motion_event_t events[8];
    unsigned event_head, event_count;
} motion_t;

bool motion_config_valid(const motion_config_t *config);
double pid_step(pid_t *pid, const pid_config_t *config, double target,
                double measured, double dt, double limit);
void pid_reset(pid_t *pid);
int64_t encoder_delta(uint32_t now, uint32_t before);
int encoder_quadrature(uint8_t before, uint8_t after, bool *invalid);
bool odometry_step(odometry_t *odom, const motion_config_t *config,
                   const encoder_sample_t *sample, double dt);
void motion_init(motion_t *motion, const motion_config_t *config,
                 encoder_sample_t sample, double now, bool setup_ok);
void motion_step(motion_t *motion, encoder_sample_t sample, double now);
motion_result_t motion_accept(motion_t *motion, const motion_command_t *command, double now);
void motion_cancel(motion_t *motion, double now);
void motion_raise_fault(motion_t *motion, motion_fault_t fault, double now);
bool motion_release(motion_t *motion, bool checks_ok);
bool motion_reset(motion_t *motion, bool cause_cleared, bool externally_verified_rest,
                  encoder_sample_t sample, double now);
void motion_shutdown(motion_t *motion, double now);
bool motion_next_event(motion_t *motion, motion_event_t *event);
/* Core API is single-owner: only the motion task calls these functions. */
#endif
