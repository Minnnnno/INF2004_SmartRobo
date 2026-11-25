#ifndef ROBO_ROBOT_H
#define ROBO_ROBOT_H
#include "float_types.h"
#include "motion.h"

typedef enum
{
    SOURCE_MANUAL,
    SOURCE_LINE,
    SOURCE_BARCODE,
    SOURCE_OBSTACLE,
    SOURCE_WIFI,
    SOURCE_TERRAIN
} request_source_t;

typedef enum
{
    CONTROL_STOP     = 1U,
    CONTROL_RELEASE  = 2U,
    CONTROL_RESET    = 4U,
    CONTROL_FAULT    = 8U,
    CONTROL_SHUTDOWN = 16U,
    CONTROL_ESTOP    = 32U
} robot_control_t;

typedef enum
{
    REMOTE_START = 1,
    REMOTE_STOP,
    REMOTE_ESTOP
} robot_remote_command_t;

/* SOURCE_BARCODE status values (barcode.c): [0] letter (ASCII), [1] valid read
 * counter, [2] junction counter (the first black after a valid read, line
 * images only), [3] failed-read counter (a long pass that did not decode, not
 * counting the junction's own bar; line images stop the car on it). */
typedef struct
{
    request_source_t source;
    motion_command_t command;
} robot_request_t;

typedef struct
{
    bool      available; /* false until owner supplies genuine data */
    float64_t timestamp;
    float64_t values[4]; /* subsystem-specific; see docs/API.md */
} subsystem_status_t;

typedef struct
{
    motion_status_t motion;

    subsystem_status_t line;
    subsystem_status_t barcode;
    subsystem_status_t terrain;
    subsystem_status_t obstacle;
    subsystem_status_t wifi;
    uint32_t           rejected_requests;
    uint32_t           dropped_events;
    motion_event_t     last_event;
    request_source_t   owner;
    bool               owner_valid;
} robot_status_t;

typedef struct
{
    robot_request_t requests[8];

    unsigned         head;
    unsigned         count;
    motion_command_t approved;
    bool             approved_valid;

    uint32_t       controls;
    uint32_t       generation;
    bool           checks_ok;
    bool           external_rest;
    bool           shutdown_latched;
    robot_status_t status;
    motion_event_t events[8];

    unsigned event_head;
    unsigned event_count;
    /* Web Start gate (Buddy 1): set by a remote STOP/ESTOP (and at boot when
     * ROBO_REQUIRE_START), cleared only by an accepted remote START. While set,
     * CONTROL_RELEASE from any other source is ignored, so no subsystem can
     * restart a robot the operator stopped. */
    bool remote_hold;
} robot_bus_t;

/* Bus operations require the caller's short priority-inheritance mutex lock.
 * Payloads copied by value; never keep references to caller storage. */
void robot_bus_init(robot_bus_t * p_bus, const motion_status_t * p_status);
motion_result_t robot_request(robot_bus_t *           p_bus,
                              const robot_request_t * p_request);
void            robot_control(robot_bus_t * p_bus, robot_control_t control,
                              bool b_checks_ok, bool b_external_rest);
void            vehicle_controller_cycle(robot_bus_t * p_bus, float64_t now);
/* Bridge for Buddy 1's queued start/stop/estop commands; caller holds bus
 * mutex. */
motion_result_t
vehicle_controller_remote_command(robot_bus_t *          p_bus,
                                  robot_remote_command_t command);
/* Take controls/approved under lock; run control calculations outside lock. */
void robot_motion_inputs(robot_bus_t * p_bus, motion_t * p_motion,
                         encoder_sample_t sample, float64_t now);
void robot_publish(robot_bus_t * p_bus, motion_t * p_motion, float64_t now);

/* Target task-context API. Nonblocking submission; result is acceptance into
 * request buffer. */
motion_result_t motion_move_forward(uint32_t command_id, float64_t metres);
motion_result_t motion_move_backward(uint32_t command_id, float64_t metres);
motion_result_t motion_turn_left(uint32_t command_id, float64_t radians);
motion_result_t motion_turn_right(uint32_t command_id, float64_t radians);
motion_result_t motion_set_wheel_speeds(uint32_t command_id, float64_t left,
                                        float64_t right);
motion_result_t motion_manual_output(uint32_t command_id, float64_t left,
                                     float64_t right);
motion_result_t robot_submit(request_source_t source, uint32_t command_id,
                             motion_mode_t mode, float64_t value,
                             float64_t left, float64_t right);
motion_result_t robot_submit_arc(request_source_t source, uint32_t command_id,
                                 float64_t metres, float64_t radians,
                                 float64_t seconds, float64_t left,
                                 float64_t right);
void            motion_stop(void);
void            motion_controller_release(bool b_checks_ok);
void motion_fault_reset(bool b_cause_cleared, bool b_externally_verified_rest);
void motion_request_shutdown(void);
bool motion_get_status(robot_status_t * p_status);
bool robot_update_subsystem(request_source_t           source,
                            const subsystem_status_t * p_status);
void robot_system_fault(void);
#endif

/*** end of file ***/
