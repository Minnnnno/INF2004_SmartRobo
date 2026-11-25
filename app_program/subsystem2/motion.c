#include "../../include/float_types.h"
#include "motion.h"
#include "motion_compact.h"
#include <math.h>
#include <string.h>

/* ROBO_MOTION_FAULTS_DISABLED (optional diagnostic override): the FAULT state
 * is never entered and status.fault stays F_NONE, so nothing latches the robot
 * stopped. Causes are still recorded in fault_flags. Time limits on a command
 * and the remote e-stop still stop the motors: see motion_fault_unlatched. */
#ifdef ROBO_MOTION_FAULTS_DISABLED
#define MOTION_FAULTS_ENABLED false
#else
#define MOTION_FAULTS_ENABLED true
#endif

static bool
motion_flag_get (const motion_t * p_motion, uint8_t mask)
{
    return (0u != ((uint32_t)p_motion->runtime_flags & mask));
}

static void
motion_flag_set (motion_t * p_motion, uint8_t mask, bool b_value)
{
    uint32_t flags = p_motion->runtime_flags;

    flags = b_value ? (flags | mask) : (flags & ~(uint32_t)mask);
    p_motion->runtime_flags = (uint8_t)flags;
}

static uint32_t
motion_queue_count (const motion_t * p_motion)
{
    return ((uint32_t)p_motion->event_queue >> 3u) & 15u;
}

static uint32_t
motion_queue_head (const motion_t * p_motion)
{
    return (uint32_t)p_motion->event_queue & 7u;
}

static void
motion_queue_set (motion_t * p_motion, uint32_t head, uint32_t count)
{
    p_motion->event_queue = (uint8_t)(head | (count << 3u));
}

static void
motion_fault_record (motion_t * p_motion, motion_fault_t fault)
{
    if (((uint32_t)fault > (uint32_t)F_NONE)
        && ((uint32_t)fault <= (uint32_t)F_SETUP_TIMEOUT))
    {
        p_motion->fault_flags
            = (uint16_t)((uint32_t)p_motion->fault_flags
                         | (UINT32_C(1) << ((uint32_t)fault - 1u)));
    }
}

static void
zero_output (motion_t * m)
{
    unsigned w;

    for (w = 0; w < 2U; ++w)
    {
        m->status.output[w] = 0.0;
        m->status.target[w] = 0.0;
        pid_reset(&m->pid[w]);
    }

    m->status.inhibited = true;
}

static int32_t
debug_scaled (float64_t value, float64_t scale)
{
    float64_t scaled = value * scale;

    if (!isfinite(scaled))
    {
        return INT32_MIN;
    }

    if (scaled >= (float64_t)INT32_MAX)
    {
        return INT32_MAX;
    }

    if (scaled <= ((float64_t)INT32_MIN + 1.0))
    {
        return INT32_MIN + 1;
    }

    return (int32_t)scaled;
}

static void
record_fault_context (motion_t * m, float64_t now)
{
    motion_fault_debug_t * d = &m->status.debug;
    motion_command_t       command;
    bool                   decoded;

    if (d->recorded)
    {
        return;
    }

    d->state   = m->status.state;
    d->mode    = m->status.mode;
    d->id      = m->status.id;
    d->time_ms = debug_scaled(now, 1000.0);
    d->dt_us   = debug_scaled(m->status.last_dt, 1000000.0);
    decoded    = (m->active) && (motion_config_valid(&m->config))
                 && (motion_compact_decode(&m->command, &m->config, &command));
    d->finite_goal
        = decoded
          && ((DISTANCE_MOVE == command.mode) || (ENCODER_TURN == command.mode)
              || (ARC_MOVE == command.mode));

    for (unsigned w = 0U; w < 2U; ++w)
    {
        float64_t target = fabs(m->status.target[w]);
        float64_t limit  = 0.0;

        d->target_mm_s[w]     = debug_scaled(m->status.target[w], 1000.0);
        d->speed_mm_s[w]      = debug_scaled(m->status.speed[w], 1000.0);
        d->output_permille[w] = debug_scaled(m->status.output[w], 1000.0);
        d->no_progress_ms[w]  = debug_scaled(now - m->last_progress[w], 1000.0);

        if ((target > 1e-9) && (m->config.counts_per_rev[w] > 0.0))
        {
            float64_t quantum = (3.141592653589793 * m->config.diameter[w])
                                / m->config.counts_per_rev[w];

            limit = m->config.feedback_timeout + ((2.0 * quantum) / target);
        }

        d->feedback_limit_ms[w] = debug_scaled(limit, 1000.0);

        if (d->finite_goal)
        {
            float64_t goal = command.value;
            float64_t sign = (0U == w) ? -1.0 : 1.0;

            if (ENCODER_TURN == command.mode)
            {
                goal = (sign * command.value * m->config.track) / 2.0;
            }

            if (ARC_MOVE == command.mode)
            {
                goal += (sign * command.angle * m->config.track) / 2.0;
            }

            d->remaining_um[w] = debug_scaled(
                goal - (m->odom.travel[w] - m->start[w]), 1000000.0);
        }
    }

    d->recorded = true;
}

const char *
motion_fault_name (motion_fault_t fault)
{
    switch (fault)
    {
        case F_NONE:
            return "NONE";
        case F_CONFIG:
            return "CONFIG";
        case F_SETUP:
            return "SETUP";
        case F_TIMING:
            return "TIMING";
        case F_FEEDBACK:
            return "FEEDBACK";
        case F_DEADLINE:
            return "DEADLINE";
        case F_EXPIRED:
            return "EXPIRED";
        case F_STOP_TIMEOUT:
            return "STOP_TIMEOUT";
        case F_FINAL_GOAL:
            return "FINAL_GOAL";
        case F_EVENT_OVERFLOW:
            return "EVENT_OVERFLOW";
        case F_SYSTEM:
            return "SYSTEM";
        case F_ESTOP:
            return "ESTOP";
        case F_SETUP_TIMEOUT:
            return "SETUP_TIMEOUT";
        default:
            return "UNKNOWN";
    }
}

const char *
motion_feedback_name (motion_feedback_cause_t cause)
{
    switch (cause)
    {
        case FB_NONE:
            return "NONE";
        case FB_UNRELIABLE:
            return "UNRELIABLE";
        case FB_INVALID_TRANSITION:
            return "INVALID_TRANSITION";
        case FB_ODOMETRY:
            return "ODOMETRY";
        case FB_NO_PROGRESS:
            return "NO_ENCODER_PROGRESS";
        case FB_RESTART_LATCH:
            return "MANUAL_RESET_REQUIRED_AFTER_RESTART";
        default:
            return "UNKNOWN";
    }
}

const char *
motion_mode_name (motion_mode_t mode)
{
    switch (mode)
    {
        case DISTANCE_MOVE:
            return "DISTANCE";
        case ENCODER_TURN:
            return "TURN";
        case WHEEL_SPEED_TRACKING:
            return "TRACKING";
        case MANUAL_OUTPUT:
            return "MANUAL";
        case ARC_MOVE:
            return "ARC";
        default:
            return "UNKNOWN";
    }
}

static void
event (motion_t * m, motion_event_type_t type)
{
    motion_event_t e
        = {type, m->status.id, m->status.generation, m->status.fault};

    if (MOTION_EVENT_CAPACITY == motion_queue_count(m))
    {
        if (!MOTION_FAULTS_ENABLED)
        {
            motion_fault_record(m, F_EVENT_OVERFLOW);
            return;
        }

        /* Local state remains authoritative even when notifications cannot be
         * delivered. */
        record_fault_context(m, m->last_time);
        zero_output(m);
        m->active       = false;
        m->status.hold  = true;
        m->status.state = FAULT;

        if (F_NONE == m->status.fault)
        {
            m->status.fault = F_EVENT_OVERFLOW;
        }

        motion_fault_record(m, F_EVENT_OVERFLOW);
        ++m->status.generation;
        return;
    }

    uint32_t index = (motion_queue_head(m) + motion_queue_count(m))
                     % MOTION_EVENT_CAPACITY;

    m->events.id[index]         = e.id;
    m->events.generation[index] = e.generation;
    m->events.info[index]
        = (uint8_t)((uint32_t)e.type | ((uint32_t)e.fault << 3u));
    motion_queue_set(m, motion_queue_head(m), motion_queue_count(m) + 1u);
}

bool
motion_next_event (motion_t * m, motion_event_t * e)
{
    if (0U == motion_queue_count(m))
    {
        return false;
    }

    uint32_t head = motion_queue_head(m);
    uint32_t info = m->events.info[head];

    *e = (motion_event_t){(motion_event_type_t)(info & 7u), m->events.id[head],
                          m->events.generation[head],
                          (motion_fault_t)((info >> 3u) & 15u)};
    motion_queue_set(m, (head + 1u) % MOTION_EVENT_CAPACITY,
                     motion_queue_count(m) - 1u);
    return true;
}

static void motion_fault_unlatched(motion_t * m, motion_fault_t fault,
                                   float64_t now);

void
motion_raise_fault (motion_t * m, motion_fault_t fault, float64_t now)
{
    if (SHUTDOWN == m->status.state)
    {
        return;
    }

    motion_fault_record(m, fault);
    record_fault_context(m, now);

    if (!MOTION_FAULTS_ENABLED)
    {
        motion_fault_unlatched(m, fault, now);
        return;
    }

    zero_output(m);
    m->status.hold = true;
    m->active      = false;
    motion_flag_set(m, MOTION_GOAL_PENDING, false);

    if ((F_FEEDBACK == fault) || (F_TIMING == fault))
    {
        motion_flag_set(m, MOTION_FEEDBACK_LATCHED, true);
        m->status.stationary       = false;
        m->status.feedback_trusted = false;
    }

    if (FAULT != m->status.state)
    {
        m->status.fault = fault;
        event(m, EV_FAULT);
        ++m->status.generation;
    }

    m->status.state = FAULT;
    m->entered      = now;
}

static void
record_feedback (motion_t * m, encoder_sample_t sample,
                 motion_feedback_cause_t cause)
{
    if (FB_NONE != m->status.feedback_cause)
    {
        return;
    }

    m->status.feedback_cause        = cause;
    m->status.feedback_fault_sample = sample;

    for (unsigned w = 0U; w < 2U; ++w)
    {
        m->status.feedback_previous_count[w]   = m->odom.previous[w];
        m->status.feedback_previous_invalid[w] = m->invalid[w];
    }
}

void
motion_restore_encoder_fault (motion_t * m, encoder_sample_t sample,
                              float64_t now)
{
    record_feedback(m, sample, FB_RESTART_LATCH);
    motion_raise_fault(m, F_FEEDBACK, now);
}

void
motion_init (motion_t * m, const motion_config_t * c, encoder_sample_t sample,
             float64_t now, bool setup_ok)
{
    unsigned w;

    memset(m, 0, sizeof(*m));
    m->config                     = *c;
    m->status.state               = INITIALISING;
    m->status.generation          = 1U;
    m->last_time                  = now;
    m->entered                    = now;
    m->status.sample_time         = now;
    m->status.encoder             = sample;
    m->status.encoder_sample_time = now;
    m->status.feedback_trusted    = sample.reliable;

    for (w = 0; w < 2U; ++w)
    {
        m->odom.previous[w] = sample.count[w];
        m->invalid[w]       = sample.invalid[w];
    }

    zero_output(m);

    if ((!motion_config_valid(c)) || (!isfinite(now)))
    {
        motion_raise_fault(m, F_CONFIG, now);
    }
    else if (!setup_ok)
    {
        motion_raise_fault(m, F_SETUP, now);
    }
    else if (!sample.reliable)
    {
        record_feedback(m, sample, FB_UNRELIABLE);
        motion_raise_fault(m, F_FEEDBACK, now);
    }
}

static void
timing_fault (motion_t * m, float64_t now, float64_t dt)
{
    if (!m->status.timing_fault_recorded)
    {
        m->status.timing_fault_dt       = dt;
        m->status.timing_fault_state    = m->status.state;
        m->status.timing_fault_recorded = true;
    }

    motion_raise_fault(m, F_TIMING, now);
}

void
motion_begin_sampling (motion_t * m, encoder_sample_t sample, float64_t now)
{
    unsigned w;

    m->status.encoder             = sample;
    m->status.encoder_sample_time = now;

    if ((INITIALISING != m->status.state) || (m->active))
    {
        return;
    }

    if ((!isfinite(now)) || (now < m->last_time))
    {
        timing_fault(m, now, now - m->last_time);
        return;
    }

    if ((!sample.reliable) || (sample.invalid[0] != m->invalid[0])
        || (sample.invalid[1] != m->invalid[1]))
    {
        record_feedback(
            m, sample, sample.reliable ? FB_INVALID_TRANSITION : FB_UNRELIABLE);
        motion_raise_fault(m, F_FEEDBACK, now);
        return;
    }

    for (w = 0U; w < 2U; ++w)
    {
        m->odom.previous[w] = sample.count[w];
    }

    m->last_time          = now;
    m->entered            = now;
    m->status.sample_time = now;
    motion_flag_set(m, MOTION_STILL_TRACKING, false);
    m->status.stationary = false;
}

static void
stopping (motion_t * m, float64_t now)
{
    zero_output(m);
    m->status.state = STOPPING;
    m->entered      = now;
    motion_flag_set(m, MOTION_STILL_TRACKING, false);
    m->status.stationary = false;
}

void
motion_cancel (motion_t * m, float64_t now)
{
    if (SHUTDOWN == m->status.state)
    {
        return;
    }

    m->status.hold = true;

    if (m->active)
    {
        event(m, EV_CANCELLED);
    }

    m->active = false;
    motion_flag_set(m, MOTION_GOAL_PENDING, false);
    ++m->status.generation;

    if (FAULT == m->status.state)
    {
        zero_output(m);
        return;
    }

    if (DRIVING == m->status.state)
    {
        stopping(m, now);
    }
    else if (STOPPING != m->status.state)
    {
        zero_output(m);
        m->status.state = STOPPED;
    }
}

void
motion_shutdown (motion_t * m, float64_t now)
{
    motion_cancel(m, now);
    zero_output(m);
    m->status.state = SHUTDOWN;
    m->status.hold  = true;
}

/* Only reached with faults disabled. Nothing here sets FAULT or status.fault.
 */
static void
motion_fault_unlatched (motion_t * m, motion_fault_t fault, float64_t now)
{
    switch (fault)
    {
        case F_DEADLINE:
        case F_EXPIRED:
            /* Time limits still end the command: without them the last PWM
             * would be held forever once a sender stops refreshing. No hold, so
             * it can restart. */
            event(m, EV_CANCELLED);
            m->active = false;
            motion_flag_set(m, MOTION_GOAL_PENDING, false);
            ++m->status.generation;
            stopping(m, now);
        break;

        case F_STOP_TIMEOUT:
        case F_FINAL_GOAL:
            /* The outputs are already off: finish the stop instead of latching.
             */
            zero_output(m);

            if (m->status.hold)
            {
                m->status.state = STOPPED;
                event(m, EV_STOPPED);
            }
            else
            {
                bool goal_pending = motion_flag_get(m, MOTION_GOAL_PENDING);

                m->status.state = READY;
                m->active       = false;
                motion_flag_set(m, MOTION_GOAL_PENDING, false);

                if (goal_pending)
                {
                    event(m, EV_COMPLETE);
                }
            }
        break;

        case F_SYSTEM:
        case F_ESTOP:
            /* System faults and remote e-stop both require a held stop. */
            motion_cancel(m, now);
        break;

        case F_CONFIG:
            /* motion_step cannot run without a valid config, so never hold
             * stale PWM. */
            zero_output(m);
            m->active = false;
            motion_flag_set(m, MOTION_GOAL_PENDING, false);
        break;

        default:
            /* Setup/timing/feedback/event failures: recorded, keep running. */
        break;
    }
}

bool
motion_release (motion_t * m, bool checks_ok)
{
    if ((STOPPED != m->status.state) || (!checks_ok) || (!m->status.stationary)
        || (!m->status.feedback_trusted) || (F_NONE != m->status.fault)
        || (!motion_config_valid(&m->config)))
    {
        return false;
    }

    ++m->status.generation;
    m->status.hold  = false;
    m->status.state = READY;
    return true;
}

bool
motion_reset (motion_t * m, bool cleared, bool external_rest,
              encoder_sample_t sample, float64_t now)
{
    uint32_t        generation = m->status.generation;
    motion_config_t config     = m->config;

    if ((FAULT != m->status.state) || (!cleared) || (!sample.reliable)
        || (!motion_config_valid(&config))
        || (motion_flag_get(m, MOTION_FEEDBACK_LATCHED)
                ? !external_rest
                : !m->status.stationary))
    {
        return false;
    }

    motion_init(m, &config, sample, now, true);
    m->status.generation = generation + 1U;
    /* Reset remains held: explicit release follows the fresh settling check. */
    m->status.hold = true;
    return true;
}

motion_result_t
motion_accept (motion_t * m, const motion_command_t * c, float64_t now)
{
    unsigned                 w;
    motion_storage_command_t stored;
    motion_command_t         decoded;

    if (NULL == c)
    {
        return R_INVALID;
    }

    bool refresh = (m->active)
                   && (WHEEL_SPEED_TRACKING == motion_compact_mode(&m->command))
                   && (WHEEL_SPEED_TRACKING == c->mode);

    if ((c->generation != m->status.generation) || (!isfinite(now))
        || (!isfinite(c->expires)) || (c->expires <= now))
    {
        return R_STALE;
    }

    if ((0U == c->id) || (!isfinite(c->value)) || (!isfinite(c->speed[0]))
        || (!isfinite(c->speed[1])) || (!isfinite(c->deadline))
        || (c->deadline <= now)
        || (c->deadline > (now + m->config.movement_timeout))
        || (c->expires > (now + m->config.command_lifetime))
        || (!isfinite(c->angle)) || (!isfinite(c->duration))
        || ((unsigned)c->mode > (unsigned)ARC_MOVE))
    {
        return R_INVALID;
    }

    if ((FAULT == m->status.state) || (SHUTDOWN == m->status.state)
        || (m->status.hold) || (INITIALISING == m->status.state))
    {
        return R_HELD;
    }

    if ((READY != m->status.state) && (!refresh))
    {
        return R_BUSY;
    }

    if ((DISTANCE_MOVE == c->mode) && (fabs(c->value) > m->config.max_distance))
    {
        return R_INVALID;
    }

    if ((ENCODER_TURN == c->mode) && (fabs(c->value) > m->config.max_angle))
    {
        return R_INVALID;
    }

    if (ARC_MOVE == c->mode)
    {
        if ((fabs(c->value) > m->config.max_distance)
            || (fabs(c->angle) > m->config.max_angle) || (c->duration < 0.0)
            || ((0.0 == c->value) && (0.0 == c->angle))
            || ((c->duration > 0.0)
                && (((c->duration + m->config.command_lifetime)
                     >= m->config.movement_timeout)
                    || ((now + c->duration) > c->deadline))))
        {
            return R_INVALID;
        }

        for (w = 0U; w < 2U; ++w)
        {
            float64_t goal
                = c->value
                  + ((((0U == w) ? -1.0 : 1.0) * c->angle * m->config.track)
                     / 2.0);

            if (((goal * c->speed[w]) < 0.0)
                || ((0.0 != goal) && (0.0 == c->speed[w])))
            {
                return R_INVALID;
            }

            if ((c->duration > 0.0)
                && ((fabs(c->speed[w] - (goal / c->duration)) > 1e-9)
                    || ((1.5 * fabs(c->speed[w])) > m->config.max_speed)
                    || (((6.0 * fabs(goal)) / (c->duration * c->duration))
                        > m->config.acceleration)))
            {
                return R_INVALID;
            }
        }
    }

    for (w = 0; w < 2U; ++w)
    {
        float64_t limit = (MANUAL_OUTPUT == c->mode) ? m->config.manual_limit
                                                     : m->config.max_speed;

        if (fabs(c->speed[w]) > limit)
        {
            return R_INVALID;
        }
    }

    if ((MANUAL_OUTPUT == c->mode)
        && ((!m->config.manual_enabled)
            || (c->deadline > (now + m->config.manual_timeout))))
    {
        return R_INVALID;
    }

    if (refresh && (c->id != m->command.id))
    {
        return R_BUSY;
    }

    if (refresh && (c->expires < m->command.expires))
    {
        return R_STALE;
    }

    if ((!motion_compact_encode(c, &m->config, &stored))
        || (!motion_compact_decode(&stored, &m->config, &decoded)))
    {
        return R_INVALID;
    }

    /* Rounded ARC goals must remain physically valid. Timed targets are
     * recomputed by the adapter, never independently rounded from goal/time. */
    if (ARC_MOVE == decoded.mode)
    {
        if ((fabs(decoded.value) < 1e-12) && (fabs(decoded.angle) < 1e-12))
        {
            return R_INVALID;
        }

        for (w = 0u; w < 2u; ++w)
        {
            float64_t goal = decoded.value
                             + ((((0u == w) ? -1.0 : 1.0) * decoded.angle
                                 * m->config.track)
                                / 2.0);

            if (((goal * decoded.speed[w]) < 0.0)
                || ((fabs(goal) > 1e-12) && (fabs(decoded.speed[w]) < 1e-12)))
            {
                return R_INVALID;
            }

            if ((decoded.duration > 0.0)
                && (((1.5 * fabs(decoded.speed[w])) > m->config.max_speed)
                    || (((6.0 * fabs(goal))
                         / (decoded.duration * decoded.duration))
                        > m->config.acceleration)))
            {
                return R_INVALID;
            }
        }
    }

    if (!refresh)
    {
        for (w = 0; w < 2U; ++w)
        {
            m->start_count[w]    = m->odom.previous[w];
            m->start[w]          = m->odom.travel[w];
            m->last_progress[w]  = now;
            m->expected_since[w] = now;
            pid_reset(&m->pid[w]);
        }

        m->entered = now;
    }

    m->command           = stored;
    m->status.id         = c->id;
    m->status.mode       = c->mode;
    m->status.state      = DRIVING;
    m->status.stationary = false;
    m->status.inhibited  = false;
    m->active            = true;
    return R_OK;
}

static bool
final_goal (const motion_t * m)
{
    motion_command_t command;

    if (!motion_compact_decode(&m->command, &m->config, &command))
    {
        return false;
    }

    float64_t left  = m->odom.travel[0] - m->start[0];
    float64_t right = m->odom.travel[1] - m->start[1];

    if (ARC_MOVE == command.mode)
    {
        return (fabs(((left + right) / 2.0) - command.value)
                <= m->config.distance_tolerance)
               && (fabs(((right - left) / m->config.track) - command.angle)
                   <= m->config.angle_tolerance);
    }

    if (DISTANCE_MOVE == command.mode)
    {
        return (fabs(((left + right) / 2.0) - command.value)
                <= m->config.distance_tolerance)
               && (fabs(right - left) <= (2.0 * m->config.distance_tolerance));
    }

    return (fabs(((right - left) / m->config.track) - command.value)
            <= m->config.angle_tolerance)
           && (fabs((left + right) / 2.0) <= m->config.distance_tolerance);
}

static void
drive (motion_t * m, float64_t now, float64_t dt, const bool progressed[2])
{
    motion_command_t command;
    uint8_t          stalled_wheels = 0U;

    if (!motion_compact_decode(&m->command, &m->config, &command))
    {
        motion_raise_fault(m, F_SYSTEM, now);
        return;
    }

    float64_t desired[2] = {command.speed[0], command.speed[1]};
    unsigned  w;

    if ((MANUAL_OUTPUT == command.mode) && (now >= command.deadline))
    {
        motion_cancel(m, now);
        return;
    }

    if (now >= command.deadline)
    {
        motion_raise_fault(m, F_DEADLINE, now);
        return;
    }

    if ((WHEEL_SPEED_TRACKING == command.mode) && (now >= command.expires))
    {
        motion_raise_fault(m, F_EXPIRED, now);
        return;
    }

    if (ARC_MOVE == command.mode)
    {
        float64_t elapsed = now - m->entered;
        float64_t left    = m->odom.travel[0] - m->start[0];
        float64_t right   = m->odom.travel[1] - m->start[1];
        float64_t tolerance
            = fmin(m->config.distance_tolerance / 2.0,
                   (m->config.angle_tolerance * m->config.track) / 4.0);
        /* Distance and heading constrain common and differential wheel error
         * separately. Requiring EACH wheel within the smaller angular-derived
         * tolerance makes a valid endpoint chase millimetres in reverse until
         * the no-progress protection trips. Keep half of each configured
         * tolerance for stopping, then verify both full limits after settling.
         */
        bool reached
            = (fabs(((left + right) / 2.0) - command.value)
               <= (m->config.distance_tolerance / 2.0))
              && (fabs(((right - left) / m->config.track) - command.angle)
                  <= (m->config.angle_tolerance / 2.0));

        if ((command.duration > 0.0) && (elapsed >= command.duration))
        {
            motion_flag_set(m, MOTION_GOAL_PENDING, true);
            /* Final encoder distance AND heading are checked after settling. */
            stopping(m, now);
            return;
        }

        for (w = 0U; w < 2U; ++w)
        {
            float64_t goal      = command.value
                                  + ((((0U == w) ? -1.0 : 1.0) * command.angle
                                      * m->config.track)
                                     / 2.0);
            float64_t travelled = m->odom.travel[w] - m->start[w];
            float64_t remaining = goal - travelled;

            if (command.duration > 0.0)
            {
                float64_t u = fmax(0.0, fmin(1.0, elapsed / command.duration));
                float64_t progress = u * u * (3.0 - (2.0 * u));

                desired[w] = (command.speed[w] * 6.0 * u * (1.0 - u))
                             + (2.0 * m->config.straight_gain
                                * ((goal * progress) - travelled));
            }
            else
            {
                desired[w]
                    = (fabs(remaining) <= tolerance)
                          ? 0.0
                          : copysign(fmin(fabs(command.speed[w]),
                                          sqrt(2.0 * m->config.acceleration
                                               * fabs(remaining))),
                                     remaining);
            }
        }

        if (0.0 == command.duration)
        {
            /* Keep the wheels matched on the way, not only at the end: a move
             * stopped early never gets an endpoint correction. Half the slower
             * wheel's speed at most, so neither wheel reverses; none once a
             * wheel is at its goal. */
            float64_t progress
                = (0.0 == command.value)
                      ? 1.0
                      : fmax(0.0,
                             fmin(1.0, (left + right) / (2.0 * command.value)));
            float64_t slack = fmin(fabs(desired[0]), fabs(desired[1])) / 2.0;
            float64_t correction = fmax(
                -slack, fmin(slack, 2.0 * m->config.straight_gain
                                        * ((right - left)
                                           - (command.angle * m->config.track
                                              * progress))));

            desired[0] += correction;
            desired[1] -= correction;
        }

        if ((0.0 == command.duration) && reached)
        {
            motion_flag_set(m, MOTION_GOAL_PENDING, true);
            stopping(m, now);
            return;
        }
    }

    if ((DISTANCE_MOVE == command.mode) || (ENCODER_TURN == command.mode))
    {
        float64_t left      = m->odom.travel[0] - m->start[0];
        float64_t right     = m->odom.travel[1] - m->start[1];
        float64_t travelled = (DISTANCE_MOVE == command.mode)
                                  ? (left + right) / 2.0
                                  : (right - left) / m->config.track;
        float64_t remaining
            = fabs(command.value) - (copysign(1.0, command.value) * travelled);
        float64_t tolerance = (DISTANCE_MOVE == command.mode)
                                  ? m->config.distance_tolerance
                                  : m->config.angle_tolerance;

        if (remaining <= (tolerance * 0.5))
        {
            motion_flag_set(m, MOTION_GOAL_PENDING, true);
            stopping(m, now);
            return;
        }

        if (ENCODER_TURN == command.mode)
        {
            remaining *= m->config.track / 2.0;
        }

        /* The ramp alone falls to ~35 mm/s in the last 2 mm, where the PID
         * output
         * (~16% PWM) is below the motors' breakaway: the wheels stop short and
         * the no-progress check faults. Hold min_goal_speed instead; stopping()
         * above cuts the output at once, so the floor does not delay the stop.
         */
        desired[1] = copysign(
            fmax(m->config.min_goal_speed,
                 fmin(m->config.max_speed,
                      sqrt(2.0 * m->config.acceleration * remaining))),
            command.value);
        desired[0] = (DISTANCE_MOVE == command.mode) ? desired[1] : -desired[1];

        if (DISTANCE_MOVE == command.mode)
        {
            float64_t correction
                = fmax(-m->config.max_speed / 2.0,
                       fmin(m->config.max_speed / 2.0,
                            m->config.straight_gain * (right - left)));

            desired[0] += correction;
            desired[1] -= correction;
        }
    }

    for (w = 0; w < 2U; ++w)
    {
        float64_t change;
        float64_t quantum = (3.141592653589793 * m->config.diameter[w])
                            / m->config.counts_per_rev[w];
        bool      manual  = MANUAL_OUTPUT == command.mode;

        desired[w]
            = fmax(-m->config.max_speed, fmin(m->config.max_speed, desired[w]));
        change = fmax(-m->config.acceleration * dt,
                      fmin(m->config.acceleration * dt,
                           desired[w] - m->status.target[w]));
        m->status.target[w] += change;
        m->status.output[w]
            = manual
                  ? command.speed[w]
                  : pid_step(&m->pid[w], &m->config.pid[w], m->status.target[w],
                             m->odom.speed[w], dt, m->config.max_output);

        /* Breakaway: from rest the PID integral took longer than the
         * no-progress limit to pass the motor's starting threshold, so a
         * stationary wheel asked to move or speed up gets at least
         * breakaway[w], doubling by the time that limit expires. Not while the
         * target is falling (braking ramp) and never for manual output; a truly
         * blocked wheel still trips the no-progress check below. */
        if ((!manual) && (!progressed[w]) && (fabs(m->status.target[w]) > 1e-9)
            && ((desired[w] * m->status.target[w]) > 0.0)
            && (fabs(desired[w]) >= fabs(m->status.target[w])))
        {
            float64_t kick = fmin(m->config.max_output,
                                  m->config.breakaway[w]
                                      * (1.0
                                         + ((now - m->last_progress[w])
                                            / m->config.feedback_timeout)));

            if (fabs(m->status.output[w]) < kick)
            {
                m->status.output[w] = copysign(kick, m->status.target[w]);
            }
        }

        /* No active reverse torque at zero target. Hardware enforces reversal
         * dead-time. */
        if ((0.0 == desired[w]) && (fabs(m->status.target[w]) < 1e-9))
        {
            m->status.output[w] = 0.0;
            pid_reset(&m->pid[w]);
        }

        if (progressed[w])
        {
            m->last_progress[w] = now;
        }

        if ((fabs(m->status.target[w]) < 1e-9)
            || (fabs(m->status.output[w]) < 1e-9))
        {
            m->expected_since[w] = now;
            m->last_progress[w]  = now;
        }
        else if (MOTION_FAULTS_ENABLED)
        {
            float64_t timeout = m->config.feedback_timeout
                                + ((2.0 * quantum) / fabs(m->status.target[w]));

            if (((now - m->expected_since[w]) >= m->config.startup_grace)
                && ((now - m->last_progress[w]) > timeout))
            {
                stalled_wheels |= (uint8_t)(1U << w);
            }
        }
    }

    /* Evaluate both wheels in the same cycle before capturing and zeroing
     * outputs. Report every wheel whose no-progress limit has expired. */
    if (0U != stalled_wheels)
    {
        encoder_sample_t sample = {{m->odom.previous[0], m->odom.previous[1]},
                                   {m->invalid[0], m->invalid[1]},
                                   true};

        record_feedback(m, sample, FB_NO_PROGRESS);
        m->status.debug.stalled_wheels = stalled_wheels;
        motion_raise_fault(m, F_FEEDBACK, now);
    }
}

void
motion_step (motion_t * m, encoder_sample_t sample, float64_t now)
{
    float64_t dt = now - m->last_time;
    bool      progressed[2];
    bool      fresh;
    unsigned  w;

    m->status.encoder             = sample;
    m->status.encoder_sample_time = now;

    if (SHUTDOWN == m->status.state)
    {
        zero_output(m);
        return;
    }

    m->status.last_dt = dt;

    if ((isfinite(dt)) && (dt > m->status.max_observed_dt))
    {
        m->status.max_observed_dt = dt;
    }

    if (!motion_config_valid(&m->config))
    {
        motion_raise_fault(m, F_CONFIG, now);
        return;
    }

    if ((isfinite(now)) && (isfinite(dt)) && (dt > m->config.max_dt)
        && (INITIALISING == m->status.state) && (!m->active)
        && (m->status.inhibited))
    {
        /* Outputs are still disabled: qualify startup cadence/rest before
         * READY, rather than treating bring-up as a missed running control
         * deadline. Keep the original setup deadline; repeated gaps cannot
         * postpone it. */
        zero_output(m);
        ++m->status.startup_gaps;
        m->status.max_startup_gap = fmax(m->status.max_startup_gap, dt);
        motion_flag_set(m, MOTION_STILL_TRACKING, false);
        m->status.stationary = false;

        if (MOTION_FAULTS_ENABLED
            && ((now - m->entered) > m->config.setup_timeout))
        {
            motion_raise_fault(m, F_SETUP_TIMEOUT, now);
            return;
        }

        if (MOTION_FAULTS_ENABLED
            && ((!sample.reliable) || (sample.invalid[0] != m->invalid[0])
                || (sample.invalid[1] != m->invalid[1])))
        {
            record_feedback(m, sample,
                            sample.reliable ? FB_INVALID_TRANSITION
                                            : FB_UNRELIABLE);
            motion_raise_fault(m, F_FEEDBACK, now);
            return;
        }

        /* Rebase the pre-drive encoder/time sample. Do not feed an oversized
         * interval into odometry/PID, or count it as stationary settling. */
        for (w = 0U; w < 2U; ++w)
        {
            m->odom.previous[w] = sample.count[w];
            m->odom.speed[w] = m->status.speed[w] = 0.0;
        }

        m->last_time = m->status.sample_time = now;
        m->status.feedback_trusted           = false;
        return;
    }
    else if ((!isfinite(now)) || (!isfinite(dt)) || (dt <= 0.0)
             || (dt > m->config.max_dt))
    {
        timing_fault(m, now, dt);

        if (isfinite(now))
        {
            m->last_time = now;
        }

        if (MOTION_FAULTS_ENABLED)
        {
            return;
        }

        if ((!isfinite(now)) || (!isfinite(dt)) || (dt <= 0.0))
        {
            /* No usable clock means no deadline checks: do not keep driving. */
            m->status.output[0] = 0.0;
            m->status.output[1] = 0.0;
            return;
        }

        /* A late cycle must still reach the deadline checks below. */
        dt = m->config.max_dt;
    }

    m->last_time          = now;
    m->status.sample_time = now;

    if (!sample.reliable)
    {
        record_feedback(m, sample, FB_UNRELIABLE);
    }

    for (w = 0; w < 2U; ++w)
    {
        progressed[w] = sample.count[w] != m->odom.previous[w];

        if (sample.invalid[w] != m->invalid[w])
        {
            record_feedback(m, sample, FB_INVALID_TRANSITION);

            if (MOTION_FAULTS_ENABLED)
            {
                sample.reliable = false;
            }
        }

        m->invalid[w] = sample.invalid[w];
    }

    fresh = (sample.reliable)
            && (odometry_step(&m->odom, &m->config, &sample, dt));

    if (!fresh)
    {
        record_feedback(m, sample, FB_ODOMETRY);
        motion_raise_fault(m, F_FEEDBACK, now);

        if (MOTION_FAULTS_ENABLED)
        {
            return;
        }

        /* Rebase, or a rejected jump would be rejected again on every later
         * sample and the state machine below (deadlines, stopping) would never
         * run. */
        if (sample.reliable)
        {
            for (w = 0U; w < 2U; ++w)
            {
                m->odom.previous[w] = sample.count[w];
            }
        }
    }

    m->status.feedback_trusted
        = fresh && (!motion_flag_get(m, MOTION_FEEDBACK_LATCHED));

    for (w = 0; w < 2U; ++w)
    {
        m->status.speed[w]  = m->odom.speed[w];
        m->status.travel[w] = m->odom.travel[w];
    }

    m->status.centre  = m->odom.centre;
    m->status.path    = m->odom.path;
    m->status.heading = m->odom.heading;

    if ((fabs(m->odom.speed[0]) <= m->config.stopped_speed)
        && (fabs(m->odom.speed[1]) <= m->config.stopped_speed)
        && (m->status.feedback_trusted))
    {
        if (!motion_flag_get(m, MOTION_STILL_TRACKING))
        {
            m->still_since = now;
            motion_flag_set(m, MOTION_STILL_TRACKING, true);
        }

        m->status.stationary = (now - m->still_since) >= m->config.settle_time;
    }
    else
    {
        motion_flag_set(m, MOTION_STILL_TRACKING, false);
        m->status.stationary = false;
    }

    switch (m->status.state)
    {
        case INITIALISING:
            if (MOTION_FAULTS_ENABLED
                && ((now - m->entered) > m->config.setup_timeout))
            {
                motion_raise_fault(m, F_SETUP_TIMEOUT, now);
            }
            else if (m->status.stationary)
            {
                m->status.state = m->status.hold ? STOPPED : READY;
            }
        break;

        case DRIVING:
            drive(m, now, dt, progressed);
        break;

        case STOPPING:
            if ((now - m->entered) > m->config.stop_timeout)
            {
                motion_raise_fault(m, F_STOP_TIMEOUT, now);
            }
            else if (m->status.stationary)
            {
                if (m->status.hold)
                {
                    m->status.state = STOPPED;
                    event(m, EV_STOPPED);
                }
                else if ((motion_flag_get(m, MOTION_GOAL_PENDING))
                         && (final_goal(m)))
                {
                    m->status.state = READY;
                    m->active       = false;
                    motion_flag_set(m, MOTION_GOAL_PENDING, false);
                    event(m, EV_COMPLETE);
                }
                else
                {
                    motion_raise_fault(m, F_FINAL_GOAL, now);
                }
            }
        break;

        default:
        break;
    }
}

/*** end of file ***/
