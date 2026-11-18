#include "robot.h"
#include "terrain_status.h"
#include <math.h>
#include <string.h>

void robot_bus_init(robot_bus_t *b, const motion_status_t *s)
{
    memset(b, 0, sizeof(*b));
    b->generation = s->generation;
    b->status.motion = *s;
    b->start_since = -1.0;
}

void robot_bus_wait_for_start(robot_bus_t *b) { b->start_gate = true; }

/* Release a pending first START once Buddy 4 has its level reference. Buddy 4
 * starts calibrating when it sees start_pending; the robot stays held (still)
 * meanwhile, so the capture is not disturbed. An IMU fault or a silent Buddy 4
 * releases too: a sensor problem never stops the run. LIFTED cancels it. */
static void start_gate_cycle(robot_bus_t *b, double now)
{
    uint8_t state;
    bool ready;
    if (!b->status.start_pending) { return; }
    if (!(b->start_since >= 0.0)) { b->start_since = now; }
    state = terrain_published_state(&b->status.terrain);
    if (state == TERRAIN_TS_LIFTED) {
        b->status.start_pending = false;
        b->start_gate = false; /* Calibrated already: the next START is a normal one. */
        return;
    }
    ready = (state == TERRAIN_TS_WAITING) || (state == TERRAIN_TS_CLIMBING) ||
            (state == TERRAIN_TS_DESCENDING) || (state == TERRAIN_TS_FAULT) ||
            ((now - b->start_since) >= ROBOT_START_CAL_TIMEOUT_S);
    if (ready) {
        b->status.start_pending = false;
        b->start_gate = false;
        robot_control(b, CONTROL_RELEASE, true, false);
    }
}

motion_result_t robot_request(robot_bus_t *b, const robot_request_t *r)
{
    unsigned i;
    if (((unsigned)r->source > (unsigned)SOURCE_TERRAIN) ||
        (r->command.id == 0U) || !isfinite(r->command.value) ||
        !isfinite(r->command.speed[0]) || !isfinite(r->command.speed[1]) ||
        !isfinite(r->command.angle) || !isfinite(r->command.duration) ||
        ((unsigned)r->command.mode > (unsigned)ARC_MOVE)) { return R_INVALID; }
    if ((r->command.generation != b->generation) || b->shutdown_latched) { return R_STALE; }
    if ((b->controls != 0U) || b->status.motion.hold ||
        ((b->status.motion.state != READY) && (b->status.motion.state != DRIVING))) { return R_HELD; }
    /* Latest steering target replaces pending targets only for the same owner/session. */
    for (i = 0; i < b->count; ++i) {
        robot_request_t *pending = &b->requests[(b->head + i) % 8U];
        if ((r->command.mode == WHEEL_SPEED_TRACKING) &&
            (pending->command.mode == WHEEL_SPEED_TRACKING) &&
            (r->source == pending->source) && (r->command.id == pending->command.id)) {
            if (r->command.expires < pending->command.expires) { return R_STALE; }
            *pending = *r;
            return R_OK;
        }
    }
    if (b->count == 8U) { return R_FULL; }
    b->requests[(b->head + b->count) % 8U] = *r;
    ++b->count;
    return R_OK;
}

void robot_control(robot_bus_t *b, robot_control_t control, bool checks_ok, bool external_rest)
{
    b->controls |= (uint32_t)control;
    if ((control == CONTROL_STOP) || (control == CONTROL_FAULT) ||
        (control == CONTROL_ESTOP) || (control == CONTROL_SHUTDOWN)) {
        ++b->generation;
        b->count = 0U;
        b->approved_valid = false;
        b->status.owner_valid = false;
        b->status.motion.hold = true;
        b->status.start_pending = false; /* A STOP cancels a START still waiting for calibration. */
        b->start_since = -1.0;
        b->controls &= ~(uint32_t)(CONTROL_RELEASE | CONTROL_RESET);
    }
    if (control == CONTROL_SHUTDOWN) { b->shutdown_latched = true; }
    b->checks_ok = checks_ok;
    b->external_rest = external_rest;
}

void vehicle_controller_cycle(robot_bus_t *b, double now)
{
    robot_request_t r;
    unsigned i;
    start_gate_cycle(b, now);
    if (b->event_count != 0U) {
        for (i = 0; i < b->event_count; ++i) {
            motion_event_t e = b->events[(b->event_head + i) % 8U];
            b->status.last_event = e;
            if ((e.type == EV_FAULT) || (e.type == EV_COMPLETE) || (e.type == EV_CANCELLED)) {
                b->status.owner_valid = false;
            }
        }
        b->event_head = (b->event_head + b->event_count) % 8U;
        b->event_count = 0U;
    }
    if ((b->controls != 0U) || b->shutdown_latched || b->status.motion.hold ||
        (b->status.motion.state == FAULT)) { b->count = 0U; return; }
    if ((b->count == 0U) || b->approved_valid) { return; }
    r = b->requests[b->head];
    b->head = (b->head + 1U) % 8U;
    --b->count;
    if ((r.command.generation != b->generation) || !isfinite(r.command.expires) ||
        (r.command.expires <= now) ||
        (b->status.owner_valid && ((b->status.owner != r.source) ||
        (b->status.motion.id != r.command.id) || (r.command.mode != WHEEL_SPEED_TRACKING)))) {
        ++b->status.rejected_requests;
        return;
    }
    b->status.owner = r.source;
    b->status.owner_valid = true;
    b->approved = r.command;
    b->approved_valid = true;
}

motion_result_t vehicle_controller_remote_command(robot_bus_t *b, robot_remote_command_t command)
{
    if (command == REMOTE_STOP) { robot_control(b, CONTROL_STOP, false, false); return R_OK; }
    if (command == REMOTE_ESTOP) { robot_control(b, CONTROL_ESTOP, false, false); return R_OK; }
    if (command != REMOTE_START) { return R_INVALID; }
    if (b->shutdown_latched || b->controls != 0U || b->status.motion.state != STOPPED ||
        b->status.motion.fault != F_NONE || !b->status.motion.stationary ||
        !b->status.motion.feedback_trusted) { return R_HELD; }
    if (b->start_gate) {
        /* First START: Buddy 4 calibrates now; start_gate_cycle() releases after. */
        if (!b->status.start_pending) { b->status.start_pending = true; b->start_since = -1.0; }
        return R_OK;
    }
    robot_control(b, CONTROL_RELEASE, true, false);
    return R_OK;
}

void robot_motion_inputs(robot_bus_t *b, motion_t *m, encoder_sample_t sample, double now)
{
    uint32_t controls = b->controls;
    b->controls = 0U;
    /* Emergency stop wins a simultaneous system fault; record both causes.
     * Faults win over cancellation; shutdown is terminal and retains the cause. */
    if ((controls & CONTROL_ESTOP) != 0U) { motion_raise_fault(m, F_ESTOP, now); }
    if ((controls & CONTROL_FAULT) != 0U) { motion_raise_fault(m, F_SYSTEM, now); }
    if (b->shutdown_latched) { motion_shutdown(m, now); }
    else if ((controls & CONTROL_STOP) != 0U) { motion_cancel(m, now); }
    else if ((controls & (CONTROL_FAULT | CONTROL_ESTOP)) == 0U) {
        if ((controls & CONTROL_RESET) != 0U) {
            (void)motion_reset(m, b->checks_ok, b->external_rest, sample, now);
        } else if ((controls & CONTROL_RELEASE) != 0U) { (void)motion_release(m, b->checks_ok); }
    }
    if (controls != 0U) {
        b->count = 0U;
        b->approved_valid = false;
        /* One authority for generations across multiple coalesced stop requests. */
        if (m->status.generation < b->generation) { m->status.generation = b->generation; }
        b->generation = m->status.generation;
    }
    if (b->approved_valid) {
        motion_result_t result = motion_accept(m, &b->approved, now);
        if (result != R_OK) {
            ++b->status.rejected_requests;
            b->status.last_event = (motion_event_t){EV_REJECTED, b->approved.id, b->approved.generation, F_NONE};
            if (!m->active) { b->status.owner_valid = false; }
        }
        b->approved_valid = false;
    }
}

void robot_publish(robot_bus_t *b, motion_t *m, double now)
{
    motion_event_t e;
    while (motion_next_event(m, &e)) {
        if (b->event_count == 8U) {
            ++b->status.dropped_events;
            motion_raise_fault(m, F_EVENT_OVERFLOW, now);
            break;
        }
        b->events[(b->event_head + b->event_count) % 8U] = e;
        ++b->event_count;
    }
    if (b->generation != m->status.generation) {
        b->count = 0U;
        b->approved_valid = false;
        b->status.owner_valid = false;
    }
    b->generation = m->status.generation;
    b->status.motion = m->status;
}
