#ifndef ROBO_ROBOT_H
#define ROBO_ROBOT_H
#include "motion.h"

typedef enum { SOURCE_MANUAL, SOURCE_LINE, SOURCE_BARCODE, SOURCE_OBSTACLE, SOURCE_WIFI, SOURCE_TERRAIN } request_source_t;
typedef enum { CONTROL_STOP = 1U, CONTROL_RELEASE = 2U, CONTROL_RESET = 4U,
               CONTROL_FAULT = 8U, CONTROL_SHUTDOWN = 16U } robot_control_t;
typedef enum { REMOTE_START = 1, REMOTE_STOP, REMOTE_ESTOP } robot_remote_command_t;
typedef struct { request_source_t source; motion_command_t command; } robot_request_t;
typedef struct {
    bool available; /* false until owner supplies genuine data */
    double timestamp;
    double values[4]; /* subsystem-specific; see docs/API.md */
} subsystem_status_t;
typedef struct {
    motion_status_t motion;
    subsystem_status_t line, barcode, terrain, obstacle, wifi;
    uint32_t rejected_requests, dropped_events;
    motion_event_t last_event;
    request_source_t owner;
    bool owner_valid;
} robot_status_t;
typedef struct {
    robot_request_t requests[8];
    unsigned head, count;
    motion_command_t approved;
    bool approved_valid;
    uint32_t controls, generation;
    bool checks_ok, external_rest, shutdown_latched;
    robot_status_t status;
    motion_event_t events[8];
    unsigned event_head, event_count;
} robot_bus_t;

/* Bus operations require the caller's short priority-inheritance mutex lock.
 * Payloads copied by value; never keep references to caller storage. */
void robot_bus_init(robot_bus_t *bus, const motion_status_t *status);
motion_result_t robot_request(robot_bus_t *bus, const robot_request_t *request);
void robot_control(robot_bus_t *bus, robot_control_t control, bool checks_ok, bool external_rest);
void vehicle_controller_cycle(robot_bus_t *bus, double now);
/* Bridge for Buddy 1's queued start/stop/estop commands; caller holds bus mutex. */
motion_result_t vehicle_controller_remote_command(robot_bus_t *bus, robot_remote_command_t command);
/* Take controls/approved under lock; run control calculations outside lock. */
void robot_motion_inputs(robot_bus_t *bus, motion_t *motion, encoder_sample_t sample, double now);
void robot_publish(robot_bus_t *bus, motion_t *motion, double now);

/* Target task-context API. Nonblocking submission; result is acceptance into request buffer. */
motion_result_t motion_move_forward(uint32_t id, double metres);
motion_result_t motion_move_backward(uint32_t id, double metres);
motion_result_t motion_turn_left(uint32_t id, double radians);
motion_result_t motion_turn_right(uint32_t id, double radians);
motion_result_t motion_set_wheel_speeds(uint32_t id, double left, double right);
motion_result_t motion_manual_output(uint32_t id, double left, double right);
motion_result_t robot_submit(request_source_t source, uint32_t id, motion_mode_t mode,
                             double value, double left, double right);
void motion_stop(void);
void motion_controller_release(bool checks_ok);
void motion_fault_reset(bool cause_cleared, bool externally_verified_rest);
void motion_request_shutdown(void);
bool motion_get_status(robot_status_t *status);
bool robot_update_subsystem(request_source_t source, const subsystem_status_t *status);
void robot_system_fault(void);
#endif
