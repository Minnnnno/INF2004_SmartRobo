#include "../../include/float_types.h"
#include "robot.h"
#include <math.h>
#include <string.h>

void
robot_bus_init (robot_bus_t * p_bus, const motion_status_t * p_status)
{
    memset(p_bus, 0, sizeof(*p_bus));
    p_bus->generation    = p_status->generation;
    p_bus->status.motion = *p_status;
}

motion_result_t
robot_request (robot_bus_t * p_bus, const robot_request_t * p_request)
{
    unsigned index;

    if (((unsigned)p_request->source > (unsigned)SOURCE_TERRAIN)
        || (0U == p_request->command.id)
        || (!isfinite(p_request->command.value))
        || (!isfinite(p_request->command.speed[0]))
        || (!isfinite(p_request->command.speed[1]))
        || (!isfinite(p_request->command.angle))
        || (!isfinite(p_request->command.duration))
        || ((unsigned)p_request->command.mode > (unsigned)ARC_MOVE))
    {
        return R_INVALID;
    }

    if ((p_request->command.generation != p_bus->generation)
        || (p_bus->shutdown_latched))
    {
        return R_STALE;
    }

    if ((0U != p_bus->controls) || (p_bus->status.motion.hold)
        || ((READY != p_bus->status.motion.state)
            && (DRIVING != p_bus->status.motion.state)))
    {
        return R_HELD;
    }

    /* Latest steering target replaces pending targets only for the same
     * owner/session. */
    for (index = 0; index < p_bus->count; ++index)
    {
        robot_request_t * p_pending
            = &p_bus->requests[(p_bus->head + index) % 8U];

        if ((WHEEL_SPEED_TRACKING == p_request->command.mode)
            && (WHEEL_SPEED_TRACKING == p_pending->command.mode)
            && (p_request->source == p_pending->source)
            && (p_request->command.id == p_pending->command.id))
        {
            if (p_request->command.expires < p_pending->command.expires)
            {
                return R_STALE;
            }

            *p_pending = *p_request;
            return R_OK;
        }
    }

    if (8U == p_bus->count)
    {
        return R_FULL;
    }

    p_bus->requests[(p_bus->head + p_bus->count) % 8U] = *p_request;
    ++p_bus->count;
    return R_OK;
}

void
robot_control (robot_bus_t * p_bus, robot_control_t control, bool b_checks_ok,
               bool b_external_rest)
{
    /* Operator stop latched: only vehicle_controller_remote_command(START) may
     * release. */
    if ((CONTROL_RELEASE == control) && (p_bus->remote_hold))
    {
        return;
    }

    p_bus->controls |= (uint32_t)control;

    if ((CONTROL_STOP == control) || (CONTROL_FAULT == control)
        || (CONTROL_ESTOP == control) || (CONTROL_SHUTDOWN == control))
    {
        ++p_bus->generation;
        p_bus->count              = 0U;
        p_bus->approved_valid     = false;
        p_bus->status.owner_valid = false;
        p_bus->status.motion.hold = true;
        p_bus->controls &= ~(uint32_t)(CONTROL_RELEASE | CONTROL_RESET);
    }

    if (CONTROL_SHUTDOWN == control)
    {
        p_bus->shutdown_latched = true;
    }

    p_bus->checks_ok     = b_checks_ok;
    p_bus->external_rest = b_external_rest;
}

void
vehicle_controller_cycle (robot_bus_t * p_bus, float64_t now)
{
    robot_request_t request;
    unsigned        index;

    if (0U != p_bus->event_count)
    {
        for (index = 0; index < p_bus->event_count; ++index)
        {
            motion_event_t motion_event
                = p_bus->events[(p_bus->event_head + index) % 8U];

            p_bus->status.last_event = motion_event;

            if ((EV_FAULT == motion_event.type)
                || (EV_COMPLETE == motion_event.type)
                || (EV_CANCELLED == motion_event.type))
            {
                p_bus->status.owner_valid = false;
            }
        }

        p_bus->event_head  = (p_bus->event_head + p_bus->event_count) % 8U;
        p_bus->event_count = 0U;
    }

    if ((0U != p_bus->controls) || (p_bus->shutdown_latched)
        || (p_bus->status.motion.hold) || (FAULT == p_bus->status.motion.state))
    {
        p_bus->count = 0U;
        return;
    }

    if ((0U == p_bus->count) || (p_bus->approved_valid))
    {
        return;
    }

    request     = p_bus->requests[p_bus->head];
    p_bus->head = (p_bus->head + 1U) % 8U;
    --p_bus->count;

    if ((request.command.generation != p_bus->generation)
        || (!isfinite(request.command.expires))
        || (request.command.expires <= now)
        || ((p_bus->status.owner_valid)
            && ((p_bus->status.owner != request.source)
                || (p_bus->status.motion.id != request.command.id)
                || (WHEEL_SPEED_TRACKING != request.command.mode))))
    {
        ++p_bus->status.rejected_requests;
        return;
    }

    p_bus->status.owner       = request.source;
    p_bus->status.owner_valid = true;
    p_bus->approved           = request.command;
    p_bus->approved_valid     = true;
}

motion_result_t
vehicle_controller_remote_command (robot_bus_t *          p_bus,
                                   robot_remote_command_t command)
{
    if (REMOTE_STOP == command)
    {
        p_bus->remote_hold = true;
        robot_control(p_bus, CONTROL_STOP, false, false);
        return R_OK;
    }

    if (REMOTE_ESTOP == command)
    {
        p_bus->remote_hold = true;
        robot_control(p_bus, CONTROL_ESTOP, false, false);
        return R_OK;
    }

    if (REMOTE_START != command)
    {
        return R_INVALID;
    }

    if ((p_bus->shutdown_latched) || (0U != p_bus->controls)
        || (STOPPED != p_bus->status.motion.state)
        || (F_NONE != p_bus->status.motion.fault)
        || (!p_bus->status.motion.stationary)
        || (!p_bus->status.motion.feedback_trusted))
    {
        return R_HELD;
    }

    p_bus->remote_hold = false;
    robot_control(p_bus, CONTROL_RELEASE, true, false);
    return R_OK;
}

void
robot_motion_inputs (robot_bus_t * p_bus, motion_t * p_motion,
                     encoder_sample_t sample, float64_t now)
{
    uint32_t controls = p_bus->controls;

    p_bus->controls = 0U;

    /* Emergency stop wins a simultaneous system fault; record both causes.
     * Faults win over cancellation; shutdown is terminal and retains the cause.
     */
    if (0U != (controls & CONTROL_ESTOP))
    {
        motion_raise_fault(p_motion, F_ESTOP, now);
    }

    if (0U != (controls & CONTROL_FAULT))
    {
        motion_raise_fault(p_motion, F_SYSTEM, now);
    }

    if (p_bus->shutdown_latched)
    {
        motion_shutdown(p_motion, now);
    }
    else if (0U != (controls & CONTROL_STOP))
    {
        motion_cancel(p_motion, now);
    }
    else if (0U == (controls & (CONTROL_FAULT | CONTROL_ESTOP)))
    {
        if (0U != (controls & CONTROL_RESET))
        {
            (void)motion_reset(p_motion, p_bus->checks_ok, p_bus->external_rest,
                               sample, now);
        }
        else if (0U != (controls & CONTROL_RELEASE))
        {
            (void)motion_release(p_motion, p_bus->checks_ok);
        }
    }

    if (0U != controls)
    {
        p_bus->count          = 0U;
        p_bus->approved_valid = false;

        /* One authority for generations across multiple coalesced stop
         * requests. */
        if (p_motion->status.generation < p_bus->generation)
        {
            p_motion->status.generation = p_bus->generation;
        }

        p_bus->generation = p_motion->status.generation;
    }

    if (p_bus->approved_valid)
    {
        motion_result_t result = motion_accept(p_motion, &p_bus->approved, now);

        if (R_OK != result)
        {
            ++p_bus->status.rejected_requests;
            p_bus->status.last_event
                = (motion_event_t){EV_REJECTED, p_bus->approved.id,
                                   p_bus->approved.generation, F_NONE};

            if (!p_motion->active)
            {
                p_bus->status.owner_valid = false;
            }
        }

        p_bus->approved_valid = false;
    }
}

void
robot_publish (robot_bus_t * p_bus, motion_t * p_motion, float64_t now)
{
    motion_event_t motion_event;

    while (motion_next_event(p_motion, &motion_event))
    {
        if (8U == p_bus->event_count)
        {
            ++p_bus->status.dropped_events;
            motion_raise_fault(p_motion, F_EVENT_OVERFLOW, now);
            break;
        }

        p_bus->events[(p_bus->event_head + p_bus->event_count) % 8U]
            = motion_event;
        ++p_bus->event_count;
    }

    if (p_bus->generation != p_motion->status.generation)
    {
        p_bus->count              = 0U;
        p_bus->approved_valid     = false;
        p_bus->status.owner_valid = false;
    }

    p_bus->generation    = p_motion->status.generation;
    p_bus->status.motion = p_motion->status;
}

/*** end of file ***/
