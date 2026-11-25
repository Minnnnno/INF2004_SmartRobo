/* Shared request bus and task-context API; only the motion owner mutates
 * motion_t. */
#include "../../include/float_types.h"
#include "robot_runtime.h"
#include "../subsystem1/comms.h"
#include "../app.h"
#include "motion_hal.h"
#include <math.h>

#define ROBOT_COMMANDS_PER_CYCLE 4U

_Static_assert(((int)COMMS_CMD_START == (int)REMOTE_START)
                   && ((int)COMMS_CMD_STOP == (int)REMOTE_STOP)
                   && ((int)COMMS_CMD_ESTOP == (int)REMOTE_ESTOP),
               "Buddy 1 command mapping changed");

static robot_bus_t             g_bus;
static ID                      g_status_mutex;
static bool                    gb_running;
static const motion_config_t * gp_runtime_config;

static bool
lock_bus (void)
{
    return (g_status_mutex > 0)
           && (E_OK == tk_loc_mtx(g_status_mutex, TMO_POL));
}

static void
unlock_bus (void)
{
    (void)tk_unl_mtx(g_status_mutex);
}

void
robot_runtime_prepare (const motion_config_t * p_config,
                       const motion_status_t * p_initial_status)
{
    gp_runtime_config = p_config;
    robot_bus_init(&g_bus, p_initial_status);
#ifdef ROBO_REQUIRE_START
    g_bus.remote_hold = true; /* boot held until the dashboard START */
#endif
}

ID
robot_runtime_init (void)
{
    T_CMTX mutex = {.mtxatr = TA_INHERIT};

    g_status_mutex = tk_cre_mtx(&mutex);
    return g_status_mutex;
}

void
robot_runtime_enable (void)
{
    gb_running = true;
}

bool
robot_runtime_lock (void)
{
    return lock_bus();
}

bool
robot_runtime_wait (void)
{
    return E_OK == tk_loc_mtx(g_status_mutex, TMO_FEVR);
}

void
robot_runtime_unlock (void)
{
    unlock_bus();
}

void
robot_runtime_motion_inputs (motion_t * p_owner, encoder_sample_t sample,
                             float64_t now)
{
    robot_motion_inputs(&g_bus, p_owner, sample, now);
}

void
robot_runtime_publish (motion_t * p_owner, float64_t now)
{
    robot_publish(&g_bus, p_owner, now);
}

bool
robot_runtime_stop_pending (void)
{
    return 0U
           != (g_bus.controls
               & (CONTROL_STOP | CONTROL_FAULT | CONTROL_ESTOP
                  | CONTROL_SHUTDOWN));
}

static motion_result_t
submit (request_source_t source, uint32_t command_id, motion_mode_t mode,
        float64_t value, float64_t left, float64_t right, float64_t angle,
        float64_t duration)
{
    robot_request_t request;
    motion_result_t result;
    float64_t       now     = motion_hal_time();
    motion_result_t outcome = R_BUSY;

    if ((!gb_running) || (!lock_bus()))
    {
        outcome = R_BUSY;
    }
    else
    {
        request.source  = source;
        request.command = (motion_command_t){
            .id         = command_id,
            .generation = g_bus.generation,
            .mode       = mode,
            .value      = value,
            .speed      = {left, right},
            .expires    = now + gp_runtime_config->command_lifetime,
            .deadline
            = now
              + ((MANUAL_OUTPUT == mode) ? gp_runtime_config->manual_timeout
                                         : gp_runtime_config->movement_timeout),
            .angle    = angle,
            .duration = duration};
        result = robot_request(&g_bus, &request);
        unlock_bus();
        outcome = result;
    }

    return outcome;
}

motion_result_t
robot_submit (request_source_t source, uint32_t command_id, motion_mode_t mode,
              float64_t value, float64_t left, float64_t right)
{
    return submit(source, command_id, mode, value, left, right, 0.0, 0.0);
}

motion_result_t
robot_submit_arc (request_source_t source, uint32_t command_id,
                  float64_t metres, float64_t radians, float64_t seconds,
                  float64_t left, float64_t right)
{
    return submit(source, command_id, ARC_MOVE, metres, left, right, radians,
                  seconds);
}

static void
control (robot_control_t request, bool b_checks, bool b_rest)
{
    /* Priority inheritance, bounded short critical sections. Never enqueue
     * stop. */
    if (gb_running && (E_OK == tk_loc_mtx(g_status_mutex, TMO_FEVR)))
    {
        robot_control(&g_bus, request, b_checks, b_rest);
        unlock_bus();
    }
}

void
motion_stop (void)
{
    control(CONTROL_STOP, false, false);
}

void
motion_controller_release (bool b_checks)
{
    control(CONTROL_RELEASE, b_checks, false);
}

void
motion_fault_reset (bool b_cleared, bool b_rest)
{
    control(CONTROL_RESET, b_cleared, b_rest);
}

void
motion_request_shutdown (void)
{
    control(CONTROL_SHUTDOWN, false, false);
}

void
robot_system_fault (void)
{
    control(CONTROL_FAULT, false, false);
}

bool
motion_get_status (robot_status_t * p_status)
{
    bool b_result = false;

    if ((NULL == p_status) || (!gb_running) || (!lock_bus()))
    {
        b_result = false;
    }
    else
    {
        *p_status = g_bus.status;
        unlock_bus();
        b_result = true;
    }

    return b_result;
}

bool
robot_update_subsystem (request_source_t           source,
                        const subsystem_status_t * p_status)
{
    subsystem_status_t * p_destination = NULL;
    unsigned             index;

    if ((NULL == p_status) || (!isfinite(p_status->timestamp)))
    {
        return false;
    }

    for (index = 0; index < ROBOT_STATUS_VALUES; ++index)
    {
        if (!isfinite(p_status->values[index]))
        {
            return false;
        }
    }

    if ((!gb_running) || (!lock_bus()))
    {
        return false;
    }

    switch (source)
    {
        case SOURCE_LINE:
            p_destination = &g_bus.status.line;
        break;

        case SOURCE_BARCODE:
            p_destination = &g_bus.status.barcode;
        break;

        case SOURCE_TERRAIN:
            p_destination = &g_bus.status.terrain;
        break;

        case SOURCE_OBSTACLE:
            p_destination = &g_bus.status.obstacle;
        break;

        case SOURCE_WIFI:
            p_destination = &g_bus.status.wifi;
        break;

        default:
        break;
    }

    if (NULL != p_destination)
    {
        *p_destination = *p_status;
    }

    unlock_bus();
    return NULL != p_destination;
}

void
controller_task (INT code, void * p_context)
{
    (void)code;
    (void)p_context;

    for (;;)
    {
        /* Consume Buddy 1's command buffer without blocking motion scheduling.
         */
        for (unsigned budget = 0U; budget < ROBOT_COMMANDS_PER_CYCLE; ++budget)
        {
            comms_cmd_t command;
            INT received = tk_rcv_mbf(g_mbf_comms_cmd, &command, TMO_POL);

            if (received < E_OK)
            {
                break;
            }

            if (received != (INT)sizeof(command))
            {
                continue;
            }

            if (E_OK == tk_loc_mtx(g_status_mutex, TMO_FEVR))
            {
                (void)vehicle_controller_remote(
                    &g_bus, (robot_remote_command_t)command);
                unlock_bus();
            }
        }

        if (lock_bus())
        {
            vehicle_controller_cycle(&g_bus, motion_hal_time());
            unlock_bus();
        }

        (void)tk_dly_tsk(10U);
    }
}

/*** end of file ***/
