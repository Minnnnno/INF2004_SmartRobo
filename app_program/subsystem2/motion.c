#include "motion.h"
#include "motion_compact.h"
#include <math.h>
#include <string.h>

static bool motion_flag_get(const motion_t *p_motion, uint8_t mask)
{
    return (0u != ((uint32_t)p_motion->runtime_flags & mask));
}

static void motion_flag_set(motion_t *p_motion, uint8_t mask, bool b_value)
{
    uint32_t flags = p_motion->runtime_flags;
    flags = b_value ? (flags | mask) : (flags & ~(uint32_t)mask);
    p_motion->runtime_flags = (uint8_t)flags;
}

static uint32_t motion_queue_count(const motion_t *p_motion)
{
    return ((uint32_t)p_motion->event_queue >> 3u) & 15u;
}

static uint32_t motion_queue_head(const motion_t *p_motion)
{
    return (uint32_t)p_motion->event_queue & 7u;
}

static void motion_queue_set(motion_t *p_motion, uint32_t head, uint32_t count)
{
    p_motion->event_queue = (uint8_t)(head | (count << 3u));
}

static void motion_fault_record(motion_t *p_motion, motion_fault_t fault)
{
    if (((uint32_t)fault > (uint32_t)F_NONE) && ((uint32_t)fault <= (uint32_t)F_SYSTEM))
    {
        p_motion->fault_flags = (uint16_t)((uint32_t)p_motion->fault_flags |
                               (UINT32_C(1) << ((uint32_t)fault - 1u)));
    }
}

static void zero_output(motion_t *m)
{
    unsigned w;
    for (w = 0; w < 2U; ++w) {
        m->status.output[w] = 0.0;
        m->status.target[w] = 0.0;
        pid_reset(&m->pid[w]);
    }
    m->status.inhibited = true;
}

static void event(motion_t *m, motion_event_type_t type)
{
    motion_event_t e = {type, m->status.id, m->status.generation, m->status.fault};
    if (motion_queue_count(m) == MOTION_EVENT_CAPACITY) {
        /* Local state remains authoritative even when notifications cannot be delivered. */
        zero_output(m);
        m->active = false;
        m->status.hold = true;
        m->status.state = FAULT;
        if (m->status.fault == F_NONE) { m->status.fault = F_EVENT_OVERFLOW; }
        motion_fault_record(m, F_EVENT_OVERFLOW);
        ++m->status.generation;
        return;
    }
    uint32_t index = (motion_queue_head(m) + motion_queue_count(m)) % MOTION_EVENT_CAPACITY;
    m->events.id[index] = e.id;
    m->events.generation[index] = e.generation;
    m->events.info[index] = (uint8_t)((uint32_t)e.type | ((uint32_t)e.fault << 3u));
    motion_queue_set(m, motion_queue_head(m), motion_queue_count(m) + 1u);
}

bool motion_next_event(motion_t *m, motion_event_t *e)
{
    if (motion_queue_count(m) == 0U) { return false; }
    uint32_t head = motion_queue_head(m);
    uint32_t info = m->events.info[head];
    *e = (motion_event_t){(motion_event_type_t)(info & 7u), m->events.id[head],
                         m->events.generation[head], (motion_fault_t)((info >> 3u) & 15u)};
    motion_queue_set(m, (head + 1u) % MOTION_EVENT_CAPACITY, motion_queue_count(m) - 1u);
    return true;
}

void motion_raise_fault(motion_t *m, motion_fault_t fault, double now)
{
    if (m->status.state == SHUTDOWN) { return; }
    motion_fault_record(m, fault);
    zero_output(m);
    m->status.hold = true;
    m->active = false;
    motion_flag_set(m, MOTION_GOAL_PENDING, false);
    if ((fault == F_FEEDBACK) || (fault == F_TIMING)) {
        motion_flag_set(m, MOTION_FEEDBACK_LATCHED, true);
        m->status.stationary = false;
        m->status.feedback_trusted = false;
    }
    if (m->status.state != FAULT) {
        m->status.fault = fault;
        event(m, EV_FAULT);
        ++m->status.generation;
    }
    m->status.state = FAULT;
    m->entered = now;
}

static void record_feedback(motion_t *m, encoder_sample_t sample, motion_feedback_cause_t cause)
{
    if (m->status.feedback_cause != FB_NONE) { return; }
    m->status.feedback_cause = cause;
    m->status.feedback_fault_sample = sample;
    for (unsigned w = 0U; w < 2U; ++w) {
        m->status.feedback_previous_count[w] = m->odom.previous[w];
        m->status.feedback_previous_invalid[w] = m->invalid[w];
    }
}

void motion_init(motion_t *m, const motion_config_t *c,
                 encoder_sample_t sample, double now, bool setup_ok)
{
    unsigned w;
    memset(m, 0, sizeof(*m));
    m->config = *c;
    m->status.state = INITIALISING;
    m->status.generation = 1U;
    m->last_time = now;
    m->entered = now;
    m->status.sample_time = now;
    m->status.feedback_trusted = sample.reliable;
    for (w = 0; w < 2U; ++w) {
        m->odom.previous[w] = sample.count[w];
        m->invalid[w] = sample.invalid[w];
    }
    zero_output(m);
    if (!motion_config_valid(c) || !isfinite(now)) { motion_raise_fault(m, F_CONFIG, now); }
    else if (!setup_ok) { motion_raise_fault(m, F_SETUP, now); }
    else if (!sample.reliable) {
        record_feedback(m, sample, FB_UNRELIABLE); motion_raise_fault(m, F_FEEDBACK, now);
    }
}

static void timing_fault(motion_t *m, double now, double dt)
{
    if (!m->status.timing_fault_recorded) {
        m->status.timing_fault_dt = dt;
        m->status.timing_fault_state = m->status.state;
        m->status.timing_fault_recorded = true;
    }
    motion_raise_fault(m, F_TIMING, now);
}

void motion_begin_sampling(motion_t *m, encoder_sample_t sample, double now)
{
    unsigned w;
    if (m->status.state != INITIALISING || m->active) { return; }
    if (!isfinite(now) || now < m->last_time) { timing_fault(m, now, now - m->last_time); return; }
    if (!sample.reliable || sample.invalid[0] != m->invalid[0] || sample.invalid[1] != m->invalid[1]) {
        record_feedback(m, sample, sample.reliable ? FB_INVALID_TRANSITION : FB_UNRELIABLE);
        motion_raise_fault(m, F_FEEDBACK, now); return;
    }
    for (w=0U; w<2U; ++w) { m->odom.previous[w]=sample.count[w]; }
    m->last_time=now; m->entered=now; m->status.sample_time=now;
    motion_flag_set(m, MOTION_STILL_TRACKING, false); m->status.stationary=false;
}

static void stopping(motion_t *m, double now)
{
    zero_output(m);
    m->status.state = STOPPING;
    m->entered = now;
    motion_flag_set(m, MOTION_STILL_TRACKING, false);
    m->status.stationary = false;
}

void motion_cancel(motion_t *m, double now)
{
    if (m->status.state == SHUTDOWN) { return; }
    m->status.hold = true;
    if (m->active) { event(m, EV_CANCELLED); }
    m->active = false;
    motion_flag_set(m, MOTION_GOAL_PENDING, false);
    ++m->status.generation;
    if (m->status.state == FAULT) { zero_output(m); return; }
    if (m->status.state == DRIVING) { stopping(m, now); }
    else if (m->status.state != STOPPING) {
        zero_output(m);
        m->status.state = STOPPED;
    }
}

void motion_shutdown(motion_t *m, double now)
{
    motion_cancel(m, now);
    zero_output(m);
    m->status.state = SHUTDOWN;
    m->status.hold = true;
}

bool motion_release(motion_t *m, bool checks_ok)
{
    if ((m->status.state != STOPPED) || !checks_ok || !m->status.stationary ||
        !m->status.feedback_trusted || (m->status.fault != F_NONE) ||
        !motion_config_valid(&m->config)) { return false; }
    ++m->status.generation;
    m->status.hold = false;
    m->status.state = READY;
    return true;
}

bool motion_reset(motion_t *m, bool cleared, bool external_rest,
                  encoder_sample_t sample, double now)
{
    uint32_t generation = m->status.generation;
    motion_config_t config = m->config;
    if ((m->status.state != FAULT) || !cleared || !sample.reliable ||
        !motion_config_valid(&config) ||
        (motion_flag_get(m, MOTION_FEEDBACK_LATCHED) ? !external_rest : !m->status.stationary)) { return false; }
    motion_init(m, &config, sample, now, true);
    m->status.generation = generation + 1U;
    /* Reset remains held: explicit release follows the fresh settling check. */
    m->status.hold = true;
    return true;
}

motion_result_t motion_accept(motion_t *m, const motion_command_t *c, double now)
{
    unsigned w;
    motion_storage_command_t stored;
    motion_command_t decoded;
    if (c == NULL) { return R_INVALID; }
    bool refresh = m->active && (motion_compact_mode(&m->command) == WHEEL_SPEED_TRACKING) &&
                   (c->mode == WHEEL_SPEED_TRACKING);
    if ((c->generation != m->status.generation) || !isfinite(now) ||
        !isfinite(c->expires) || (c->expires <= now)) { return R_STALE; }
    if ((c->id == 0U) || !isfinite(c->value) || !isfinite(c->speed[0]) ||
        !isfinite(c->speed[1]) || !isfinite(c->deadline) || (c->deadline <= now) ||
        (c->deadline > now + m->config.movement_timeout) ||
        (c->expires > now + m->config.command_lifetime) ||
        !isfinite(c->angle) || !isfinite(c->duration) ||
        ((unsigned)c->mode > (unsigned)ARC_MOVE)) { return R_INVALID; }
    if ((m->status.state == FAULT) || (m->status.state == SHUTDOWN) || m->status.hold ||
        (m->status.state == INITIALISING)) { return R_HELD; }
    if ((m->status.state != READY) && !refresh) { return R_BUSY; }
    if ((c->mode == DISTANCE_MOVE) && (fabs(c->value) > m->config.max_distance)) { return R_INVALID; }
    if ((c->mode == ENCODER_TURN) && (fabs(c->value) > m->config.max_angle)) { return R_INVALID; }
    if (c->mode == ARC_MOVE) {
        if (fabs(c->value) > m->config.max_distance || fabs(c->angle) > m->config.max_angle ||
            c->duration < 0.0 || (c->value == 0.0 && c->angle == 0.0) ||
            (c->duration > 0.0 && (c->duration + m->config.command_lifetime >= m->config.movement_timeout ||
                                   now + c->duration > c->deadline))) { return R_INVALID; }
        for (w = 0U; w < 2U; ++w) {
            double goal = c->value + (w == 0U ? -1.0 : 1.0) * c->angle * m->config.track / 2.0;
            if (goal * c->speed[w] < 0.0 || (goal != 0.0 && c->speed[w] == 0.0)) { return R_INVALID; }
            if (c->duration > 0.0 &&
                (fabs(c->speed[w] - goal / c->duration) > 1e-9 ||
                 1.5 * fabs(c->speed[w]) > m->config.max_speed ||
                 6.0 * fabs(goal) / (c->duration * c->duration) > m->config.acceleration)) { return R_INVALID; }
        }
    }
    for (w = 0; w < 2U; ++w) {
        double limit = (c->mode == MANUAL_OUTPUT) ? m->config.manual_limit : m->config.max_speed;
        if (fabs(c->speed[w]) > limit) { return R_INVALID; }
    }
    if ((c->mode == MANUAL_OUTPUT) && (!m->config.manual_enabled ||
        (c->deadline > now + m->config.manual_timeout))) { return R_INVALID; }
    if (refresh && (c->id != m->command.id)) { return R_BUSY; }
    if (refresh && (c->expires < m->command.expires)) { return R_STALE; }
    if (!motion_compact_encode(c, &m->config, &stored) ||
        !motion_compact_decode(&stored, &m->config, &decoded)) { return R_INVALID; }
    /* Rounded ARC goals must remain physically valid. Timed targets are
     * recomputed by the adapter, never independently rounded from goal/time. */
    if (decoded.mode == ARC_MOVE) {
        if (fabs(decoded.value) < 1e-12 && fabs(decoded.angle) < 1e-12) { return R_INVALID; }
        for (w = 0u; w < 2u; ++w) {
            double goal = decoded.value + (w == 0u ? -1.0 : 1.0) * decoded.angle * m->config.track / 2.0;
            if (goal * decoded.speed[w] < 0.0 ||
                (fabs(goal) > 1e-12 && fabs(decoded.speed[w]) < 1e-12)) { return R_INVALID; }
            if (decoded.duration > 0.0 &&
                (1.5 * fabs(decoded.speed[w]) > m->config.max_speed ||
                 6.0 * fabs(goal) / (decoded.duration * decoded.duration) > m->config.acceleration)) { return R_INVALID; }
        }
    }
    if (!refresh) {
        for (w = 0; w < 2U; ++w) {
            m->start_count[w] = m->odom.previous[w];
            m->start[w] = m->odom.travel[w];
            m->last_progress[w] = now;
            m->expected_since[w] = now;
            pid_reset(&m->pid[w]);
        }
        m->entered = now;
    }
    m->command = stored;
    m->status.id = c->id;
    m->status.mode = c->mode;
    m->status.state = DRIVING;
    m->status.stationary = false;
    m->status.inhibited = false;
    m->active = true;
    return R_OK;
}

static bool final_goal(const motion_t *m)
{
    motion_command_t command;
    if (!motion_compact_decode(&m->command, &m->config, &command)) { return false; }
    double left = m->odom.travel[0] - m->start[0];
    double right = m->odom.travel[1] - m->start[1];
    if (command.mode == ARC_MOVE) {
        return fabs((left + right) / 2.0 - command.value) <= m->config.distance_tolerance &&
               fabs((right - left) / m->config.track - command.angle) <= m->config.angle_tolerance;
    }
    if (command.mode == DISTANCE_MOVE) {
        return (fabs((left + right) / 2.0 - command.value) <= m->config.distance_tolerance) &&
               (fabs(right - left) <= 2.0 * m->config.distance_tolerance);
    }
    return (fabs((right - left) / m->config.track - command.value) <= m->config.angle_tolerance) &&
           (fabs((left + right) / 2.0) <= m->config.distance_tolerance);
}

static void drive(motion_t *m, double now, double dt, const bool progressed[2])
{
    motion_command_t command;
    if (!motion_compact_decode(&m->command, &m->config, &command)) {
        motion_raise_fault(m, F_SYSTEM, now);
        return;
    }
    double desired[2] = {command.speed[0], command.speed[1]};
    unsigned w;
    if ((command.mode == MANUAL_OUTPUT) && (now >= command.deadline)) {
        motion_cancel(m, now);
        return;
    }
    if (now >= command.deadline) { motion_raise_fault(m, F_DEADLINE, now); return; }
    if ((command.mode == WHEEL_SPEED_TRACKING) &&
        (now >= command.expires)) { motion_raise_fault(m, F_EXPIRED, now); return; }
    if (command.mode == ARC_MOVE) {
        double elapsed = now - m->entered;
        double tolerance = fmin(m->config.distance_tolerance / 2.0,
                                m->config.angle_tolerance * m->config.track / 4.0);
        bool reached = true;
        if (command.duration > 0.0 && elapsed >= command.duration) {
            motion_flag_set(m, MOTION_GOAL_PENDING, true);
            stopping(m, now); /* Final encoder distance AND heading are checked after settling. */
            return;
        }
        for (w = 0U; w < 2U; ++w) {
            double goal = command.value + (w == 0U ? -1.0 : 1.0) * command.angle * m->config.track / 2.0;
            double travelled = m->odom.travel[w] - m->start[w];
            double remaining = goal - travelled;
            if (fabs(remaining) > tolerance) { reached = false; }
            if (command.duration > 0.0) {
                double u = fmax(0.0, fmin(1.0, elapsed / command.duration));
                double progress = u * u * (3.0 - 2.0 * u);
                desired[w] = command.speed[w] * 6.0 * u * (1.0 - u) +
                             2.0 * m->config.straight_gain * (goal * progress - travelled);
            } else {
                desired[w] = fabs(remaining) <= tolerance ? 0.0 :
                    copysign(fmin(fabs(command.speed[w]),
                                  sqrt(2.0 * m->config.acceleration * fabs(remaining))), remaining);
            }
        }
        if (command.duration == 0.0 && reached) {
            motion_flag_set(m, MOTION_GOAL_PENDING, true);
            stopping(m, now);
            return;
        }
    }
    if ((command.mode == DISTANCE_MOVE) || (command.mode == ENCODER_TURN)) {
        double left = m->odom.travel[0] - m->start[0];
        double right = m->odom.travel[1] - m->start[1];
        double travelled = (command.mode == DISTANCE_MOVE) ? (left + right) / 2.0 :
                           (right - left) / m->config.track;
        double remaining = fabs(command.value) - copysign(1.0, command.value) * travelled;
        double tolerance = (command.mode == DISTANCE_MOVE) ? m->config.distance_tolerance : m->config.angle_tolerance;
        if (remaining <= tolerance * 0.5) {
            motion_flag_set(m, MOTION_GOAL_PENDING, true);
            stopping(m, now);
            return;
        }
        if (command.mode == ENCODER_TURN) { remaining *= m->config.track / 2.0; }
        desired[1] = copysign(fmin(m->config.max_speed, sqrt(2.0 * m->config.acceleration * remaining)), command.value);
        desired[0] = (command.mode == DISTANCE_MOVE) ? desired[1] : -desired[1];
        if (command.mode == DISTANCE_MOVE) {
            double correction = fmax(-m->config.max_speed / 2.0,
                                fmin(m->config.max_speed / 2.0, m->config.straight_gain * (right - left)));
            desired[0] += correction;
            desired[1] -= correction;
        }
    }
    for (w = 0; w < 2U; ++w) {
        double change;
        double quantum = 3.141592653589793 * m->config.diameter[w] / m->config.counts_per_rev[w];
        bool manual = command.mode == MANUAL_OUTPUT;
        desired[w] = fmax(-m->config.max_speed, fmin(m->config.max_speed, desired[w]));
        change = fmax(-m->config.acceleration * dt,
                 fmin(m->config.acceleration * dt, desired[w] - m->status.target[w]));
        m->status.target[w] += change;
        m->status.output[w] = manual ? command.speed[w] :
            pid_step(&m->pid[w], &m->config.pid[w], m->status.target[w], m->odom.speed[w], dt, m->config.max_output);
        /* No active reverse torque at zero target. Hardware enforces reversal dead-time. */
        if ((desired[w] == 0.0) && (fabs(m->status.target[w]) < 1e-9)) {
            m->status.output[w] = 0.0;
            pid_reset(&m->pid[w]);
        }
        if (progressed[w]) { m->last_progress[w] = now; }
        if ((fabs(m->status.target[w]) < 1e-9) ||
            (fabs(m->status.output[w]) < 1e-9)) {
            m->expected_since[w] = now;
            m->last_progress[w] = now;
        } else {
            double timeout = m->config.feedback_timeout + 2.0 * quantum / fabs(m->status.target[w]);
            if (((now - m->expected_since[w]) >= m->config.startup_grace) &&
                ((now - m->last_progress[w]) > timeout)) {
                encoder_sample_t sample = {{m->odom.previous[0], m->odom.previous[1]},
                                           {m->invalid[0], m->invalid[1]}, true};
                record_feedback(m, sample, FB_NO_PROGRESS);
                motion_raise_fault(m, F_FEEDBACK, now);
                return;
            }
        }
    }
}

void motion_step(motion_t *m, encoder_sample_t sample, double now)
{
    double dt = now - m->last_time;
    bool progressed[2];
    unsigned w;
    if (m->status.state == SHUTDOWN) { zero_output(m); return; }
    m->status.last_dt = dt;
    if (isfinite(dt) && dt > m->status.max_observed_dt) { m->status.max_observed_dt = dt; }
    if (!motion_config_valid(&m->config)) { motion_raise_fault(m, F_CONFIG, now); return; }
    if (isfinite(now) && isfinite(dt) && dt > m->config.max_dt &&
        m->status.state == INITIALISING && !m->active && m->status.inhibited) {
        /* Outputs are still disabled: qualify startup cadence/rest before READY,
         * rather than treating bring-up as a missed running control deadline.
         * Keep the original setup deadline; repeated gaps cannot postpone it. */
        zero_output(m);
        ++m->status.startup_gaps;
        m->status.max_startup_gap = fmax(m->status.max_startup_gap, dt);
        motion_flag_set(m, MOTION_STILL_TRACKING, false);
        m->status.stationary = false;
        if (now - m->entered > m->config.setup_timeout) {
            motion_raise_fault(m, F_SETUP, now); return;
        }
        if (!sample.reliable || sample.invalid[0] != m->invalid[0] || sample.invalid[1] != m->invalid[1]) {
            record_feedback(m, sample, sample.reliable ? FB_INVALID_TRANSITION : FB_UNRELIABLE);
            motion_raise_fault(m, F_FEEDBACK, now); return;
        }
        /* Rebase the pre-drive encoder/time sample. Do not feed an oversized
         * interval into odometry/PID, or count it as stationary settling. */
        for (w = 0U; w < 2U; ++w) {
            m->odom.previous[w] = sample.count[w];
            m->odom.speed[w] = m->status.speed[w] = 0.0;
        }
        m->last_time = m->status.sample_time = now;
        m->status.feedback_trusted = false;
        return;
    } else if (!isfinite(now) || !isfinite(dt) || (dt <= 0.0) || (dt > m->config.max_dt)) {
        timing_fault(m, now, dt);
        if (isfinite(now)) { m->last_time = now; }
        return;
    }
    m->last_time = now;
    m->status.sample_time = now;
    if (!sample.reliable) { record_feedback(m, sample, FB_UNRELIABLE); }
    for (w = 0; w < 2U; ++w) {
        progressed[w] = sample.count[w] != m->odom.previous[w];
        if (sample.invalid[w] != m->invalid[w]) {
            record_feedback(m, sample, FB_INVALID_TRANSITION);
            sample.reliable = false;
        }
        m->invalid[w] = sample.invalid[w];
    }
    if (!sample.reliable || !odometry_step(&m->odom, &m->config, &sample, dt)) {
        record_feedback(m, sample, FB_ODOMETRY);
        motion_raise_fault(m, F_FEEDBACK, now);
        return;
    }
    m->status.feedback_trusted = !motion_flag_get(m, MOTION_FEEDBACK_LATCHED);
    for (w = 0; w < 2U; ++w) {
        m->status.speed[w] = m->odom.speed[w];
        m->status.travel[w] = m->odom.travel[w];
    }
    m->status.centre = m->odom.centre;
    m->status.path = m->odom.path;
    m->status.heading = m->odom.heading;
    if ((fabs(m->odom.speed[0]) <= m->config.stopped_speed) &&
        (fabs(m->odom.speed[1]) <= m->config.stopped_speed) && m->status.feedback_trusted) {
        if (!motion_flag_get(m, MOTION_STILL_TRACKING)) { m->still_since = now; motion_flag_set(m, MOTION_STILL_TRACKING, true); }
        m->status.stationary = (now - m->still_since) >= m->config.settle_time;
    } else { motion_flag_set(m, MOTION_STILL_TRACKING, false); m->status.stationary = false; }
    switch (m->status.state) {
    case INITIALISING:
        if ((now - m->entered) > m->config.setup_timeout) { motion_raise_fault(m, F_SETUP, now); }
        else if (m->status.stationary) { m->status.state = m->status.hold ? STOPPED : READY; }
        break;
    case DRIVING:
        drive(m, now, dt, progressed);
        break;
    case STOPPING:
        if ((now - m->entered) > m->config.stop_timeout) { motion_raise_fault(m, F_STOP_TIMEOUT, now); }
        else if (m->status.stationary) {
            if (m->status.hold) { m->status.state = STOPPED; event(m, EV_STOPPED); }
            else if (motion_flag_get(m, MOTION_GOAL_PENDING) && final_goal(m)) {
                m->status.state = READY;
                m->active = false;
                motion_flag_set(m, MOTION_GOAL_PENDING, false);
                event(m, EV_COMPLETE);
            } else { motion_raise_fault(m, F_FINAL_GOAL, now); }
        }
        break;
    default:
        break;
    }
}
