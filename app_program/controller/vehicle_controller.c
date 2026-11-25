#include "../../include/float_types.h"
#include "robot.h"
#include <math.h>
#include <string.h>

void
robot_bus_init (robot_bus_t * b, const motion_status_t * s)
{
    memset(b, 0, sizeof(*b));
    b->generation    = s->generation;
    b->status.motion = *s;
}

motion_result_t
robot_request (robot_bus_t * b, const robot_request_t * r)
{
    unsigned i;

    if (((unsigned)r->source > (unsigned)SOURCE_TERRAIN)
        || (0U == r->command.id) || (!isfinite(r->command.value))
        || (!isfinite(r->command.speed[0])) || (!isfinite(r->command.speed[1]))
        || (!isfinite(r->command.angle)) || (!isfinite(r->command.duration))
        || ((unsigned)r->command.mode > (unsigned)ARC_MOVE))
    {
        return R_INVALID;
    }

    if ((r->command.generation != b->generation) || (b->shutdown_latched))
    {
        return R_STALE;
    }

    if ((0U != b->controls) || (b->status.motion.hold)
        || ((READY != b->status.motion.state)
            && (DRIVING != b->status.motion.state)))
    {
        return R_HELD;
    }

    /* Latest steering target replaces pending targets only for the same
     * owner/session. */
    for (i = 0; i < b->count; ++i)
    {
        robot_request_t * pending = &b->requests[(b->head + i) % 8U];

        if ((WHEEL_SPEED_TRACKING == r->command.mode)
            && (WHEEL_SPEED_TRACKING == pending->command.mode)
            && (r->source == pending->source)
            && (r->command.id == pending->command.id))
        {
            if (r->command.expires < pending->command.expires)
            {
                return R_STALE;
            }

            *pending = *r;
            return R_OK;
        }
    }

    if (8U == b->count)
    {
        return R_FULL;
    }

    b->requests[(b->head + b->count) % 8U] = *r;
    ++b->count;
    return R_OK;
}

void
robot_control (robot_bus_t * b, robot_control_t control, bool checks_ok,
               bool external_rest)
{
    /* Operator stop latched: only vehicle_controller_remote_command(START) may
     * release. */
    if ((CONTROL_RELEASE == control) && (b->remote_hold))
    {
        return;
    }

    b->controls |= (uint32_t)control;

    if ((CONTROL_STOP == control) || (CONTROL_FAULT == control)
        || (CONTROL_ESTOP == control) || (CONTROL_SHUTDOWN == control))
    {
        ++b->generation;
        b->count              = 0U;
        b->approved_valid     = false;
        b->status.owner_valid = false;
        b->status.motion.hold = true;
        b->controls &= ~(uint32_t)(CONTROL_RELEASE | CONTROL_RESET);
    }

    if (CONTROL_SHUTDOWN == control)
    {
        b->shutdown_latched = true;
    }

    b->checks_ok     = checks_ok;
    b->external_rest = external_rest;
}

void
vehicle_controller_cycle (robot_bus_t * b, float64_t now)
{
    robot_request_t r;
    unsigned        i;

    if (0U != b->event_count)
    {
        for (i = 0; i < b->event_count; ++i)
        {
            motion_event_t e = b->events[(b->event_head + i) % 8U];

            b->status.last_event = e;

            if ((EV_FAULT == e.type) || (EV_COMPLETE == e.type)
                || (EV_CANCELLED == e.type))
            {
                b->status.owner_valid = false;
            }
        }

        b->event_head  = (b->event_head + b->event_count) % 8U;
        b->event_count = 0U;
    }

    if ((0U != b->controls) || (b->shutdown_latched) || (b->status.motion.hold)
        || (FAULT == b->status.motion.state))
    {
        b->count = 0U;
        return;
    }

    if ((0U == b->count) || (b->approved_valid))
    {
        return;
    }

    r       = b->requests[b->head];
    b->head = (b->head + 1U) % 8U;
    --b->count;

    if ((r.command.generation != b->generation)
        || (!isfinite(r.command.expires)) || (r.command.expires <= now)
        || ((b->status.owner_valid)
            && ((b->status.owner != r.source)
                || (b->status.motion.id != r.command.id)
                || (WHEEL_SPEED_TRACKING != r.command.mode))))
    {
        ++b->status.rejected_requests;
        return;
    }

    b->status.owner       = r.source;
    b->status.owner_valid = true;
    b->approved           = r.command;
    b->approved_valid     = true;
}

motion_result_t
vehicle_controller_remote_command (robot_bus_t *          b,
                                   robot_remote_command_t command)
{
    if (REMOTE_STOP == command)
    {
        b->remote_hold = true;
        robot_control(b, CONTROL_STOP, false, false);
        return R_OK;
    }

    if (REMOTE_ESTOP == command)
    {
        b->remote_hold = true;
        robot_control(b, CONTROL_ESTOP, false, false);
        return R_OK;
    }

    if (REMOTE_START != command)
    {
        return R_INVALID;
    }

    if ((b->shutdown_latched) || (0U != b->controls)
        || (STOPPED != b->status.motion.state)
        || (F_NONE != b->status.motion.fault) || (!b->status.motion.stationary)
        || (!b->status.motion.feedback_trusted))
    {
        return R_HELD;
    }

    b->remote_hold = false;
    robot_control(b, CONTROL_RELEASE, true, false);
    return R_OK;
}

void
robot_motion_inputs (robot_bus_t * b, motion_t * m, encoder_sample_t sample,
                     float64_t now)
{
    uint32_t controls = b->controls;

    b->controls = 0U;

    /* Emergency stop wins a simultaneous system fault; record both causes.
     * Faults win over cancellation; shutdown is terminal and retains the cause.
     */
    if (0U != (controls & CONTROL_ESTOP))
    {
        motion_raise_fault(m, F_ESTOP, now);
    }

    if (0U != (controls & CONTROL_FAULT))
    {
        motion_raise_fault(m, F_SYSTEM, now);
    }

    if (b->shutdown_latched)
    {
        motion_shutdown(m, now);
    }
    else if (0U != (controls & CONTROL_STOP))
    {
        motion_cancel(m, now);
    }
    else if (0U == (controls & (CONTROL_FAULT | CONTROL_ESTOP)))
    {
        if (0U != (controls & CONTROL_RESET))
        {
            (void)motion_reset(m, b->checks_ok, b->external_rest, sample, now);
        }
        else if (0U != (controls & CONTROL_RELEASE))
        {
            (void)motion_release(m, b->checks_ok);
        }
    }

    if (0U != controls)
    {
        b->count          = 0U;
        b->approved_valid = false;

        /* One authority for generations across multiple coalesced stop
         * requests. */
        if (m->status.generation < b->generation)
        {
            m->status.generation = b->generation;
        }

        b->generation = m->status.generation;
    }

    if (b->approved_valid)
    {
        motion_result_t result = motion_accept(m, &b->approved, now);

        if (R_OK != result)
        {
            ++b->status.rejected_requests;
            b->status.last_event = (motion_event_t){
                EV_REJECTED, b->approved.id, b->approved.generation, F_NONE};

            if (!m->active)
            {
                b->status.owner_valid = false;
            }
        }

        b->approved_valid = false;
    }
}

void
robot_publish (robot_bus_t * b, motion_t * m, float64_t now)
{
    motion_event_t e;

    while (motion_next_event(m, &e))
    {
        if (8U == b->event_count)
        {
            ++b->status.dropped_events;
            motion_raise_fault(m, F_EVENT_OVERFLOW, now);
            break;
        }

        b->events[(b->event_head + b->event_count) % 8U] = e;
        ++b->event_count;
    }

    if (b->generation != m->status.generation)
    {
        b->count              = 0U;
        b->approved_valid     = false;
        b->status.owner_valid = false;
    }

    b->generation    = m->status.generation;
    b->status.motion = m->status;
}

/*** end of file ***/
