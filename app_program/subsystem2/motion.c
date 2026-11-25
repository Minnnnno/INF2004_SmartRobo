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

static bool motion_flag_get(const motion_t * p_motion, uint8_t mask);
static void motion_flag_set(motion_t * p_motion, uint8_t mask, bool b_value);
static uint32_t motion_queue_count(const motion_t * p_motion);
static uint32_t motion_queue_head(const motion_t * p_motion);
static void     motion_queue_set(motion_t * p_motion, uint32_t head,
                                 uint32_t count);
static void     motion_fault_record(motion_t * p_motion, motion_fault_t fault);
static void     zero_output(motion_t * p_motion);
static int32_t  debug_scaled(float64_t value, float64_t scale);
static void     record_fault_context(motion_t * p_motion, float64_t now);
static void     event(motion_t * p_motion, motion_event_type_t type);
static void     record_feedback(motion_t * p_motion, encoder_sample_t sample,
                                motion_feedback_cause_t cause);
static void timing_fault(motion_t * p_motion, float64_t now, float64_t delta_s);
static void stopping(motion_t * p_motion, float64_t now);
static void motion_fault_unlatched(motion_t * p_motion, motion_fault_t fault,
                                   float64_t now);
static bool accept_arc(const motion_t *         p_motion,
                       const motion_command_t * p_command, float64_t now);
static bool accept_rounded_arc(const motion_t *         p_motion,
                               const motion_command_t * p_decoded);
static bool final_goal(const motion_t * p_motion);
static bool drive_expired(motion_t *               p_motion,
                          const motion_command_t * p_command, float64_t now);
static void drive_arc_match(const motion_t *         p_motion,
                            const motion_command_t * p_command, float64_t left,
                            float64_t right, float64_t desired[2]);
static bool drive_arc(motion_t * p_motion, const motion_command_t * p_command,
                      float64_t now, float64_t desired[2]);
static bool drive_finite(motion_t *               p_motion,
                         const motion_command_t * p_command, float64_t now,
                         float64_t desired[2]);
static bool drive_wheel(motion_t * p_motion, const motion_command_t * p_command,
                        unsigned wheel, float64_t * p_desired,
                        bool b_progressed, float64_t now, float64_t delta_s);
static void drive(motion_t * p_motion, float64_t now, float64_t delta_s,
                  const bool progressed[2]);
static void step_startup_gap(motion_t * p_motion, encoder_sample_t sample,
                             float64_t now, float64_t delta_s);
static bool step_bad_interval(motion_t * p_motion, float64_t now,
                              float64_t * p_delta_s);
static bool step_odometry(motion_t * p_motion, encoder_sample_t sample,
                          float64_t now, float64_t delta_s, bool progressed[2],
                          bool * p_fresh);
static void step_stationary(motion_t * p_motion, float64_t now);
static void step_state(motion_t * p_motion, float64_t now, float64_t delta_s,
                       const bool progressed[2]);

const char *
motion_fault_name (motion_fault_t fault)
{
    const char * p_result = NULL;

    switch (fault)
    {
        case F_NONE:
            p_result = "NONE";
        break;

        case F_CONFIG:
            p_result = "CONFIG";
        break;

        case F_SETUP:
            p_result = "SETUP";
        break;

        case F_TIMING:
            p_result = "TIMING";
        break;

        case F_FEEDBACK:
            p_result = "FEEDBACK";
        break;

        case F_DEADLINE:
            p_result = "DEADLINE";
        break;

        case F_EXPIRED:
            p_result = "EXPIRED";
        break;

        case F_STOP_TIMEOUT:
            p_result = "STOP_TIMEOUT";
        break;

        case F_FINAL_GOAL:
            p_result = "FINAL_GOAL";
        break;

        case F_EVENT_OVERFLOW:
            p_result = "EVENT_OVERFLOW";
        break;

        case F_SYSTEM:
            p_result = "SYSTEM";
        break;

        case F_ESTOP:
            p_result = "ESTOP";
        break;

        case F_SETUP_TIMEOUT:
            p_result = "SETUP_TIMEOUT";
        break;

        default:
            p_result = "UNKNOWN";
        break;
    }

    return p_result;
}

const char *
motion_feedback_name (motion_feedback_cause_t cause)
{
    const char * p_result = NULL;

    switch (cause)
    {
        case FB_NONE:
            p_result = "NONE";
        break;

        case FB_UNRELIABLE:
            p_result = "UNRELIABLE";
        break;

        case FB_INVALID_TRANSITION:
            p_result = "INVALID_TRANSITION";
        break;

        case FB_ODOMETRY:
            p_result = "ODOMETRY";
        break;

        case FB_NO_PROGRESS:
            p_result = "NO_ENCODER_PROGRESS";
        break;

        case FB_RESTART_LATCH:
            p_result = "MANUAL_RESET_REQUIRED_AFTER_RESTART";
        break;

        default:
            p_result = "UNKNOWN";
        break;
    }

    return p_result;
}

const char *
motion_mode_name (motion_mode_t mode)
{
    const char * p_result = NULL;

    switch (mode)
    {
        case DISTANCE_MOVE:
            p_result = "DISTANCE";
        break;

        case ENCODER_TURN:
            p_result = "TURN";
        break;

        case WHEEL_SPEED_TRACKING:
            p_result = "TRACKING";
        break;

        case MANUAL_OUTPUT:
            p_result = "MANUAL";
        break;

        case ARC_MOVE:
            p_result = "ARC";
        break;

        default:
            p_result = "UNKNOWN";
        break;
    }

    return p_result;
}

bool
motion_next_event (motion_t * p_motion, motion_event_t * p_event)
{
    bool b_result = false;

    if (0U == motion_queue_count(p_motion))
    {
        b_result = false;
    }
    else
    {
        uint32_t head = motion_queue_head(p_motion);
        uint32_t info = p_motion->events.info[head];

        *p_event = (motion_event_t){(motion_event_type_t)(info & 7u),
                                    p_motion->events.id[head],
                                    p_motion->events.generation[head],
                                    (motion_fault_t)((info >> 3u) & 15u)};
        motion_queue_set(p_motion, (head + 1u) % MOTION_EVENT_CAPACITY,
                         motion_queue_count(p_motion) - 1u);
        b_result = true;
    }

    return b_result;
}

void
motion_raise_fault (motion_t * p_motion, motion_fault_t fault, float64_t now)
{
    if (SHUTDOWN == p_motion->status.state)
    {
        return;
    }

    motion_fault_record(p_motion, fault);
    record_fault_context(p_motion, now);

    if (!MOTION_FAULTS_ENABLED)
    {
        motion_fault_unlatched(p_motion, fault, now);
        return;
    }

    zero_output(p_motion);
    p_motion->status.hold = true;
    p_motion->active      = false;
    motion_flag_set(p_motion, MOTION_GOAL_PENDING, false);

    if ((F_FEEDBACK == fault) || (F_TIMING == fault))
    {
        motion_flag_set(p_motion, MOTION_FEEDBACK_LATCHED, true);
        p_motion->status.stationary       = false;
        p_motion->status.feedback_trusted = false;
    }

    if (FAULT != p_motion->status.state)
    {
        p_motion->status.fault = fault;
        event(p_motion, EV_FAULT);
        ++p_motion->status.generation;
    }

    p_motion->status.state = FAULT;
    p_motion->entered      = now;
}

void
motion_restore_encoder_fault (motion_t * p_motion, encoder_sample_t sample,
                              float64_t now)
{
    record_feedback(p_motion, sample, FB_RESTART_LATCH);
    motion_raise_fault(p_motion, F_FEEDBACK, now);
}

void
motion_init (motion_t * p_motion, const motion_config_t * p_config,
             encoder_sample_t sample, float64_t now, bool b_setup_ok)
{
    unsigned wheel;

    memset(p_motion, 0, sizeof(*p_motion));
    p_motion->config                     = *p_config;
    p_motion->status.state               = INITIALISING;
    p_motion->status.generation          = 1U;
    p_motion->last_time                  = now;
    p_motion->entered                    = now;
    p_motion->status.sample_time         = now;
    p_motion->status.encoder             = sample;
    p_motion->status.encoder_sample_time = now;
    p_motion->status.feedback_trusted    = sample.reliable;

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        p_motion->odom.previous[wheel] = sample.count[wheel];
        p_motion->invalid[wheel]       = sample.invalid[wheel];
    }

    zero_output(p_motion);

    if ((!motion_config_valid(p_config)) || (!isfinite(now)))
    {
        motion_raise_fault(p_motion, F_CONFIG, now);
    }
    else if (!b_setup_ok)
    {
        motion_raise_fault(p_motion, F_SETUP, now);
    }
    else if (!sample.reliable)
    {
        record_feedback(p_motion, sample, FB_UNRELIABLE);
        motion_raise_fault(p_motion, F_FEEDBACK, now);
    }
    else
    {
        /* Nothing to do in any other case. */
    }
}

void
motion_begin_sampling (motion_t * p_motion, encoder_sample_t sample,
                       float64_t now)
{
    unsigned wheel;

    p_motion->status.encoder             = sample;
    p_motion->status.encoder_sample_time = now;

    if ((INITIALISING != p_motion->status.state) || (p_motion->active))
    {
        /* Nothing to do. */
    }
    else if ((!isfinite(now)) || (now < p_motion->last_time))
    {
        timing_fault(p_motion, now, now - p_motion->last_time);
    }
    else if ((!sample.reliable) || (sample.invalid[0] != p_motion->invalid[0])
             || (sample.invalid[1] != p_motion->invalid[1]))
    {
        record_feedback(p_motion, sample,
                        sample.reliable ? FB_INVALID_TRANSITION
                                        : FB_UNRELIABLE);
        motion_raise_fault(p_motion, F_FEEDBACK, now);
    }
    else
    {
        for (wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
        {
            p_motion->odom.previous[wheel] = sample.count[wheel];
        }

        p_motion->last_time          = now;
        p_motion->entered            = now;
        p_motion->status.sample_time = now;
        motion_flag_set(p_motion, MOTION_STILL_TRACKING, false);
        p_motion->status.stationary = false;
    }
}

void
motion_cancel (motion_t * p_motion, float64_t now)
{
    if (SHUTDOWN == p_motion->status.state)
    {
        return;
    }

    p_motion->status.hold = true;

    if (p_motion->active)
    {
        event(p_motion, EV_CANCELLED);
    }

    p_motion->active = false;
    motion_flag_set(p_motion, MOTION_GOAL_PENDING, false);
    ++p_motion->status.generation;

    if (FAULT == p_motion->status.state)
    {
        zero_output(p_motion);
        return;
    }

    if (DRIVING == p_motion->status.state)
    {
        stopping(p_motion, now);
    }
    else if (STOPPING != p_motion->status.state)
    {
        zero_output(p_motion);
        p_motion->status.state = STOPPED;
    }
    else
    {
        /* Nothing to do in any other case. */
    }
}

void
motion_shutdown (motion_t * p_motion, float64_t now)
{
    motion_cancel(p_motion, now);
    zero_output(p_motion);
    p_motion->status.state = SHUTDOWN;
    p_motion->status.hold  = true;
}

bool
motion_release (motion_t * p_motion, bool b_checks_ok)
{
    bool b_result = false;

    if ((STOPPED != p_motion->status.state) || (!b_checks_ok)
        || (!p_motion->status.stationary)
        || (!p_motion->status.feedback_trusted)
        || (F_NONE != p_motion->status.fault)
        || (!motion_config_valid(&p_motion->config)))
    {
        b_result = false;
    }
    else
    {
        ++p_motion->status.generation;
        p_motion->status.hold  = false;
        p_motion->status.state = READY;
        b_result               = true;
    }

    return b_result;
}

bool
motion_reset (motion_t * p_motion, bool b_cleared, bool b_external_rest,
              encoder_sample_t sample, float64_t now)
{
    uint32_t        generation = p_motion->status.generation;
    motion_config_t config     = p_motion->config;

    bool b_result = false;

    if ((FAULT != p_motion->status.state) || (!b_cleared) || (!sample.reliable)
        || (!motion_config_valid(&config))
        || (motion_flag_get(p_motion, MOTION_FEEDBACK_LATCHED)
                ? !b_external_rest
                : !p_motion->status.stationary))
    {
        b_result = false;
    }
    else
    {
        motion_init(p_motion, &config, sample, now, true);
        p_motion->status.generation = generation + 1U;
        /* Reset remains held: explicit release follows the fresh settling
         * check. */
        p_motion->status.hold = true;
        b_result              = true;
    }

    return b_result;
}

motion_result_t
motion_accept (motion_t * p_motion, const motion_command_t * p_command,
               float64_t now)
{
    unsigned                 wheel;
    motion_storage_command_t stored;
    motion_command_t         decoded;

    if (NULL == p_command)
    {
        return R_INVALID;
    }

    bool b_refresh
        = (p_motion->active)
          && (WHEEL_SPEED_TRACKING == motion_compact_mode(&p_motion->command))
          && (WHEEL_SPEED_TRACKING == p_command->mode);

    if ((p_command->generation != p_motion->status.generation)
        || (!isfinite(now)) || (!isfinite(p_command->expires))
        || (p_command->expires <= now))
    {
        return R_STALE;
    }

    if ((0U == p_command->id) || (!isfinite(p_command->value))
        || (!isfinite(p_command->speed[0])) || (!isfinite(p_command->speed[1]))
        || (!isfinite(p_command->deadline)) || (p_command->deadline <= now)
        || (p_command->deadline > (now + p_motion->config.movement_timeout))
        || (p_command->expires > (now + p_motion->config.command_lifetime))
        || (!isfinite(p_command->angle)) || (!isfinite(p_command->duration))
        || ((unsigned)p_command->mode > (unsigned)ARC_MOVE))
    {
        return R_INVALID;
    }

    if ((FAULT == p_motion->status.state)
        || (SHUTDOWN == p_motion->status.state) || (p_motion->status.hold)
        || (INITIALISING == p_motion->status.state))
    {
        return R_HELD;
    }

    if ((READY != p_motion->status.state) && (!b_refresh))
    {
        return R_BUSY;
    }

    if ((DISTANCE_MOVE == p_command->mode)
        && (fabs(p_command->value) > p_motion->config.max_distance))
    {
        return R_INVALID;
    }

    if ((ENCODER_TURN == p_command->mode)
        && (fabs(p_command->value) > p_motion->config.max_angle))
    {
        return R_INVALID;
    }

    if ((ARC_MOVE == p_command->mode)
        && (!accept_arc(p_motion, p_command, now)))
    {
        return R_INVALID;
    }

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        float64_t limit = (MANUAL_OUTPUT == p_command->mode)
                              ? p_motion->config.manual_limit
                              : p_motion->config.max_speed;

        if (fabs(p_command->speed[wheel]) > limit)
        {
            return R_INVALID;
        }
    }

    if ((MANUAL_OUTPUT == p_command->mode)
        && ((!p_motion->config.manual_enabled)
            || (p_command->deadline > (now + p_motion->config.manual_timeout))))
    {
        return R_INVALID;
    }

    if (b_refresh && (p_command->id != p_motion->command.id))
    {
        return R_BUSY;
    }

    if (b_refresh && (p_command->expires < p_motion->command.expires))
    {
        return R_STALE;
    }

    if ((!motion_compact_encode(p_command, &p_motion->config, &stored))
        || (!motion_compact_decode(&stored, &p_motion->config, &decoded)))
    {
        return R_INVALID;
    }

    if ((ARC_MOVE == decoded.mode) && (!accept_rounded_arc(p_motion, &decoded)))
    {
        return R_INVALID;
    }

    if (!b_refresh)
    {
        for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
        {
            p_motion->start_count[wheel]    = p_motion->odom.previous[wheel];
            p_motion->start[wheel]          = p_motion->odom.travel[wheel];
            p_motion->last_progress[wheel]  = now;
            p_motion->expected_since[wheel] = now;
            pid_reset(&p_motion->pid[wheel]);
        }

        p_motion->entered = now;
    }

    p_motion->command           = stored;
    p_motion->status.id         = p_command->id;
    p_motion->status.mode       = p_command->mode;
    p_motion->status.state      = DRIVING;
    p_motion->status.stationary = false;
    p_motion->status.inhibited  = false;
    p_motion->active            = true;
    return R_OK;
}

void
motion_step (motion_t * p_motion, encoder_sample_t sample, float64_t now)
{
    float64_t delta_s = now - p_motion->last_time;
    bool      progressed[2];
    bool      b_fresh;
    unsigned  wheel;

    p_motion->status.encoder             = sample;
    p_motion->status.encoder_sample_time = now;

    if (SHUTDOWN == p_motion->status.state)
    {
        zero_output(p_motion);
        return;
    }

    p_motion->status.last_dt = delta_s;

    if ((isfinite(delta_s)) && (delta_s > p_motion->status.max_observed_dt))
    {
        p_motion->status.max_observed_dt = delta_s;
    }

    if (!motion_config_valid(&p_motion->config))
    {
        motion_raise_fault(p_motion, F_CONFIG, now);
        return;
    }

    if ((isfinite(now)) && (isfinite(delta_s))
        && (delta_s > p_motion->config.max_dt)
        && (INITIALISING == p_motion->status.state) && (!p_motion->active)
        && (p_motion->status.inhibited))
    {
        step_startup_gap(p_motion, sample, now, delta_s);
        return;
    }
    else if ((!isfinite(now)) || (!isfinite(delta_s)) || (delta_s <= 0.0)
             || (delta_s > p_motion->config.max_dt))
    {
        if (!step_bad_interval(p_motion, now, &delta_s))
        {
            return;
        }
    }
    else
    {
        /* Nothing to do in any other case. */
    }

    p_motion->last_time          = now;
    p_motion->status.sample_time = now;

    if (!step_odometry(p_motion, sample, now, delta_s, progressed, &b_fresh))
    {
        return;
    }

    p_motion->status.feedback_trusted
        = b_fresh && (!motion_flag_get(p_motion, MOTION_FEEDBACK_LATCHED));

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        p_motion->status.speed[wheel]  = p_motion->odom.speed[wheel];
        p_motion->status.travel[wheel] = p_motion->odom.travel[wheel];
    }

    p_motion->status.centre  = p_motion->odom.centre;
    p_motion->status.path    = p_motion->odom.path;
    p_motion->status.heading = p_motion->odom.heading;

    step_stationary(p_motion, now);
    step_state(p_motion, now, delta_s, progressed);
}

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
zero_output (motion_t * p_motion)
{
    unsigned wheel;

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        p_motion->status.output[wheel] = 0.0;
        p_motion->status.target[wheel] = 0.0;
        pid_reset(&p_motion->pid[wheel]);
    }

    p_motion->status.inhibited = true;
}

static int32_t
debug_scaled (float64_t value, float64_t scale)
{
    float64_t scaled = value * scale;
    int32_t   result = 0;

    if (!isfinite(scaled))
    {
        result = INT32_MIN;
    }
    else if (scaled >= (float64_t)INT32_MAX)
    {
        result = INT32_MAX;
    }
    else if (scaled <= ((float64_t)INT32_MIN + 1.0))
    {
        result = INT32_MIN + 1;
    }
    else
    {
        result = (int32_t)scaled;
    }

    return result;
}

static void
record_fault_context (motion_t * p_motion, float64_t now)
{
    motion_fault_debug_t * p_debug = &p_motion->status.debug;
    motion_command_t       command;
    bool                   b_decoded;

    if (p_debug->recorded)
    {
        return;
    }

    p_debug->state   = p_motion->status.state;
    p_debug->mode    = p_motion->status.mode;
    p_debug->id      = p_motion->status.id;
    p_debug->time_ms = debug_scaled(now, 1000.0);
    p_debug->dt_us   = debug_scaled(p_motion->status.last_dt, 1000000.0);
    b_decoded = (p_motion->active) && (motion_config_valid(&p_motion->config))
                && (motion_compact_decode(&p_motion->command, &p_motion->config,
                                          &command));
    p_debug->finite_goal
        = b_decoded
          && ((DISTANCE_MOVE == command.mode) || (ENCODER_TURN == command.mode)
              || (ARC_MOVE == command.mode));

    for (unsigned wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        float64_t target = fabs(p_motion->status.target[wheel]);
        float64_t limit  = 0.0;

        p_debug->target_mm_s[wheel]
            = debug_scaled(p_motion->status.target[wheel], 1000.0);
        p_debug->speed_mm_s[wheel]
            = debug_scaled(p_motion->status.speed[wheel], 1000.0);
        p_debug->output_permille[wheel]
            = debug_scaled(p_motion->status.output[wheel], 1000.0);
        p_debug->no_progress_ms[wheel]
            = debug_scaled(now - p_motion->last_progress[wheel], 1000.0);

        if ((target > 1e-9) && (p_motion->config.counts_per_rev[wheel] > 0.0))
        {
            float64_t quantum
                = (3.141592653589793 * p_motion->config.diameter[wheel])
                  / p_motion->config.counts_per_rev[wheel];

            limit = p_motion->config.feedback_timeout
                    + ((2.0 * quantum) / target);
        }

        p_debug->feedback_limit_ms[wheel] = debug_scaled(limit, 1000.0);

        if (p_debug->finite_goal)
        {
            float64_t goal = command.value;
            float64_t sign = (0U == wheel) ? -1.0 : 1.0;

            if (ENCODER_TURN == command.mode)
            {
                goal = (sign * command.value * p_motion->config.track) / 2.0;
            }

            if (ARC_MOVE == command.mode)
            {
                goal += (sign * command.angle * p_motion->config.track) / 2.0;
            }

            p_debug->remaining_um[wheel] = debug_scaled(
                goal - (p_motion->odom.travel[wheel] - p_motion->start[wheel]),
                1000000.0);
        }
    }

    p_debug->recorded = true;
}

static void
event (motion_t * p_motion, motion_event_type_t type)
{
    motion_event_t motion_event
        = {type, p_motion->status.id, p_motion->status.generation,
           p_motion->status.fault};

    if (MOTION_EVENT_CAPACITY == motion_queue_count(p_motion))
    {
        if (!MOTION_FAULTS_ENABLED)
        {
            motion_fault_record(p_motion, F_EVENT_OVERFLOW);
            return;
        }

        /* Local state remains authoritative even when notifications cannot be
         * delivered. */
        record_fault_context(p_motion, p_motion->last_time);
        zero_output(p_motion);
        p_motion->active       = false;
        p_motion->status.hold  = true;
        p_motion->status.state = FAULT;

        if (F_NONE == p_motion->status.fault)
        {
            p_motion->status.fault = F_EVENT_OVERFLOW;
        }

        motion_fault_record(p_motion, F_EVENT_OVERFLOW);
        ++p_motion->status.generation;
        return;
    }

    uint32_t index
        = (motion_queue_head(p_motion) + motion_queue_count(p_motion))
          % MOTION_EVENT_CAPACITY;

    p_motion->events.id[index]         = motion_event.id;
    p_motion->events.generation[index] = motion_event.generation;
    p_motion->events.info[index]
        = (uint8_t)((uint32_t)motion_event.type
                    | ((uint32_t)motion_event.fault << 3u));
    motion_queue_set(p_motion, motion_queue_head(p_motion),
                     motion_queue_count(p_motion) + 1u);
}

static void
record_feedback (motion_t * p_motion, encoder_sample_t sample,
                 motion_feedback_cause_t cause)
{
    if (FB_NONE != p_motion->status.feedback_cause)
    {
        return;
    }

    p_motion->status.feedback_cause        = cause;
    p_motion->status.feedback_fault_sample = sample;

    for (unsigned wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        p_motion->status.feedback_previous_count[wheel]
            = p_motion->odom.previous[wheel];
        p_motion->status.feedback_previous_invalid[wheel]
            = p_motion->invalid[wheel];
    }
}

static void
timing_fault (motion_t * p_motion, float64_t now, float64_t delta_s)
{
    if (!p_motion->status.timing_fault_recorded)
    {
        p_motion->status.timing_fault_dt       = delta_s;
        p_motion->status.timing_fault_state    = p_motion->status.state;
        p_motion->status.timing_fault_recorded = true;
    }

    motion_raise_fault(p_motion, F_TIMING, now);
}

static void
stopping (motion_t * p_motion, float64_t now)
{
    zero_output(p_motion);
    p_motion->status.state = STOPPING;
    p_motion->entered      = now;
    motion_flag_set(p_motion, MOTION_STILL_TRACKING, false);
    p_motion->status.stationary = false;
}

/* Only reached with faults disabled. Nothing here sets FAULT or status.fault.
 */
static void
motion_fault_unlatched (motion_t * p_motion, motion_fault_t fault,
                        float64_t now)
{
    switch (fault)
    {
        case F_DEADLINE:
        case F_EXPIRED:
            /* Time limits still end the command: without them the last PWM
             * would be held forever once a sender stops refreshing. No hold, so
             * it can restart. */
            event(p_motion, EV_CANCELLED);
            p_motion->active = false;
            motion_flag_set(p_motion, MOTION_GOAL_PENDING, false);
            ++p_motion->status.generation;
            stopping(p_motion, now);
        break;

        case F_STOP_TIMEOUT:
        case F_FINAL_GOAL:
            /* The outputs are already off: finish the stop instead of latching.
             */
            zero_output(p_motion);

            if (p_motion->status.hold)
            {
                p_motion->status.state = STOPPED;
                event(p_motion, EV_STOPPED);
            }
            else
            {
                bool b_goal_pending
                    = motion_flag_get(p_motion, MOTION_GOAL_PENDING);

                p_motion->status.state = READY;
                p_motion->active       = false;
                motion_flag_set(p_motion, MOTION_GOAL_PENDING, false);

                if (b_goal_pending)
                {
                    event(p_motion, EV_COMPLETE);
                }
            }
        break;

        case F_SYSTEM:
        case F_ESTOP:
            /* System faults and remote e-stop both require a held stop. */
            motion_cancel(p_motion, now);
        break;

        case F_CONFIG:
            /* motion_step cannot run without a valid config, so never hold
             * stale PWM. */
            zero_output(p_motion);
            p_motion->active = false;
            motion_flag_set(p_motion, MOTION_GOAL_PENDING, false);
        break;

        default:
            /* Setup/timing/feedback/event failures: recorded, keep running. */
        break;
    }
}

/* An arc must be possible for both wheels.  */
static bool
accept_arc (const motion_t * p_motion, const motion_command_t * p_command,
            float64_t now)
{
    unsigned wheel;

    if ((fabs(p_command->value) > p_motion->config.max_distance)
        || (fabs(p_command->angle) > p_motion->config.max_angle)
        || (p_command->duration < 0.0)
        || ((0.0 == p_command->value) && (0.0 == p_command->angle))
        || ((p_command->duration > 0.0)
            && (((p_command->duration + p_motion->config.command_lifetime)
                 >= p_motion->config.movement_timeout)
                || ((now + p_command->duration) > p_command->deadline))))
    {
        return false;
    }

    for (wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        float64_t goal = p_command->value
                         + ((((0U == wheel) ? -1.0 : 1.0) * p_command->angle
                             * p_motion->config.track)
                            / 2.0);

        if (((goal * p_command->speed[wheel]) < 0.0)
            || ((0.0 != goal) && (0.0 == p_command->speed[wheel])))
        {
            return false;
        }

        if ((p_command->duration > 0.0)
            && ((fabs(p_command->speed[wheel] - (goal / p_command->duration))
                 > 1e-9)
                || ((1.5 * fabs(p_command->speed[wheel]))
                    > p_motion->config.max_speed)
                || (((6.0 * fabs(goal))
                     / (p_command->duration * p_command->duration))
                    > p_motion->config.acceleration)))
        {
            return false;
        }
    }

    return true;
}

/* Rounded ARC goals must remain physically valid. Timed targets are recomputed
 * by the adapter, never independently rounded from goal/time. */
static bool
accept_rounded_arc (const motion_t *         p_motion,
                    const motion_command_t * p_decoded)
{
    unsigned wheel;

    if ((fabs(p_decoded->value) < 1e-12) && (fabs(p_decoded->angle) < 1e-12))
    {
        return false;
    }

    for (wheel = 0u; wheel < MOTION_WHEELS; ++wheel)
    {
        float64_t goal = p_decoded->value
                         + ((((0u == wheel) ? -1.0 : 1.0) * p_decoded->angle
                             * p_motion->config.track)
                            / 2.0);

        if (((goal * p_decoded->speed[wheel]) < 0.0)
            || ((fabs(goal) > 1e-12)
                && (fabs(p_decoded->speed[wheel]) < 1e-12)))
        {
            return false;
        }

        if ((p_decoded->duration > 0.0)
            && (((1.5 * fabs(p_decoded->speed[wheel]))
                 > p_motion->config.max_speed)
                || (((6.0 * fabs(goal))
                     / (p_decoded->duration * p_decoded->duration))
                    > p_motion->config.acceleration)))
        {
            return false;
        }
    }

    return true;
}

static bool
final_goal (const motion_t * p_motion)
{
    motion_command_t command;

    bool b_result = false;

    if (!motion_compact_decode(&p_motion->command, &p_motion->config, &command))
    {
        b_result = false;
    }
    else
    {
        float64_t left  = p_motion->odom.travel[0] - p_motion->start[0];
        float64_t right = p_motion->odom.travel[1] - p_motion->start[1];

        if (ARC_MOVE == command.mode)
        {
            b_result = (fabs(((left + right) / 2.0) - command.value)
                        <= p_motion->config.distance_tolerance)
                       && (fabs(((right - left) / p_motion->config.track)
                                - command.angle)
                           <= p_motion->config.angle_tolerance);
        }
        else if (DISTANCE_MOVE == command.mode)
        {
            b_result = (fabs(((left + right) / 2.0) - command.value)
                        <= p_motion->config.distance_tolerance)
                       && (fabs(right - left)
                           <= (2.0 * p_motion->config.distance_tolerance));
        }
        else
        {
            b_result = (fabs(((right - left) / p_motion->config.track)
                             - command.value)
                        <= p_motion->config.angle_tolerance)
                       && (fabs((left + right) / 2.0)
                           <= p_motion->config.distance_tolerance);
        }
    }

    return b_result;
}

/* Out of time: the command ends here.  True: it did. */
static bool
drive_expired (motion_t * p_motion, const motion_command_t * p_command,
               float64_t now)
{
    if ((MANUAL_OUTPUT == p_command->mode) && (now >= p_command->deadline))
    {
        motion_cancel(p_motion, now);
        return true;
    }

    if (now >= p_command->deadline)
    {
        motion_raise_fault(p_motion, F_DEADLINE, now);
        return true;
    }

    if ((WHEEL_SPEED_TRACKING == p_command->mode)
        && (now >= p_command->expires))
    {
        motion_raise_fault(p_motion, F_EXPIRED, now);
        return true;
    }

    return false;
}

/* Keep the wheels of an arc matched on the way, not only at the end: a move
 * stopped early never gets an endpoint correction. Half the slower wheel's
 * speed at most, so neither wheel reverses; none once a wheel is at its goal.
 */
static void
drive_arc_match (const motion_t * p_motion, const motion_command_t * p_command,
                 float64_t left, float64_t right, float64_t desired[2])
{
    float64_t progress
        = (0.0 == p_command->value)
              ? 1.0
              : fmax(0.0, fmin(1.0, (left + right) / (2.0 * p_command->value)));
    float64_t slack      = fmin(fabs(desired[0]), fabs(desired[1])) / 2.0;
    float64_t correction = fmax(
        -slack, fmin(slack, 2.0 * p_motion->config.straight_gain
                                * ((right - left)
                                   - (p_command->angle * p_motion->config.track
                                      * progress))));

    desired[0] += correction;
    desired[1] -= correction;
}

/* Wheel speeds for an arc.  True: it is finished (stopping). */
static bool
drive_arc (motion_t * p_motion, const motion_command_t * p_command,
           float64_t now, float64_t desired[2])
{
    unsigned wheel;

    float64_t elapsed   = now - p_motion->entered;
    float64_t left      = p_motion->odom.travel[0] - p_motion->start[0];
    float64_t right     = p_motion->odom.travel[1] - p_motion->start[1];
    float64_t tolerance = fmin(
        p_motion->config.distance_tolerance / 2.0,
        (p_motion->config.angle_tolerance * p_motion->config.track) / 4.0);
    /* Distance and heading constrain common and differential wheel error
     * separately. Requiring EACH wheel within the smaller angular-derived
     * tolerance makes a valid endpoint chase millimetres in reverse until
     * the no-progress protection trips. Keep half of each configured
     * tolerance for stopping, then verify both full limits after settling.
     */
    bool b_reached
        = (fabs(((left + right) / 2.0) - p_command->value)
           <= (p_motion->config.distance_tolerance / 2.0))
          && (fabs(((right - left) / p_motion->config.track) - p_command->angle)
              <= (p_motion->config.angle_tolerance / 2.0));

    if ((p_command->duration > 0.0) && (elapsed >= p_command->duration))
    {
        motion_flag_set(p_motion, MOTION_GOAL_PENDING, true);
        /* Final encoder distance AND heading are checked after settling. */
        stopping(p_motion, now);
        return true;
    }

    for (wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        float64_t goal = p_command->value
                         + ((((0U == wheel) ? -1.0 : 1.0) * p_command->angle
                             * p_motion->config.track)
                            / 2.0);
        float64_t travelled
            = p_motion->odom.travel[wheel] - p_motion->start[wheel];
        float64_t remaining = goal - travelled;

        if (p_command->duration > 0.0)
        {
            float64_t wheel_output
                = fmax(0.0, fmin(1.0, elapsed / p_command->duration));
            float64_t progress
                = wheel_output * wheel_output * (3.0 - (2.0 * wheel_output));

            desired[wheel] = (p_command->speed[wheel] * 6.0 * wheel_output
                              * (1.0 - wheel_output))
                             + (2.0 * p_motion->config.straight_gain
                                * ((goal * progress) - travelled));
        }
        else
        {
            desired[wheel]
                = (fabs(remaining) <= tolerance)
                      ? 0.0
                      : copysign(fmin(fabs(p_command->speed[wheel]),
                                      sqrt(2.0 * p_motion->config.acceleration
                                           * fabs(remaining))),
                                 remaining);
        }
    }

    if (0.0 == p_command->duration)
    {
        drive_arc_match(p_motion, p_command, left, right, desired);
    }

    if ((0.0 == p_command->duration) && b_reached)
    {
        motion_flag_set(p_motion, MOTION_GOAL_PENDING, true);
        stopping(p_motion, now);
        return true;
    }

    return false;
}

/* Wheel speeds for a straight move or a turn on the spot.  True: it is finished
 * (stopping). */
static bool
drive_finite (motion_t * p_motion, const motion_command_t * p_command,
              float64_t now, float64_t desired[2])
{
    float64_t left      = p_motion->odom.travel[0] - p_motion->start[0];
    float64_t right     = p_motion->odom.travel[1] - p_motion->start[1];
    float64_t travelled = (DISTANCE_MOVE == p_command->mode)
                              ? (left + right) / 2.0
                              : (right - left) / p_motion->config.track;
    float64_t remaining = fabs(p_command->value)
                          - (copysign(1.0, p_command->value) * travelled);
    float64_t tolerance = (DISTANCE_MOVE == p_command->mode)
                              ? p_motion->config.distance_tolerance
                              : p_motion->config.angle_tolerance;

    if (remaining <= (tolerance * 0.5))
    {
        motion_flag_set(p_motion, MOTION_GOAL_PENDING, true);
        stopping(p_motion, now);
        return true;
    }

    if (ENCODER_TURN == p_command->mode)
    {
        remaining *= p_motion->config.track / 2.0;
    }

    /* The ramp alone falls to ~35 mm/s in the last 2 mm, where the PID
     * output
     * (~16% PWM) is below the motors' breakaway: the wheels stop short and
     * the no-progress check faults. Hold min_goal_speed instead; stopping()
     * above cuts the output at once, so the floor does not delay the stop.
     */
    desired[1] = copysign(
        fmax(p_motion->config.min_goal_speed,
             fmin(p_motion->config.max_speed,
                  sqrt(2.0 * p_motion->config.acceleration * remaining))),
        p_command->value);
    desired[0] = (DISTANCE_MOVE == p_command->mode) ? desired[1] : -desired[1];

    if (DISTANCE_MOVE == p_command->mode)
    {
        float64_t correction
            = fmax(-p_motion->config.max_speed / 2.0,
                   fmin(p_motion->config.max_speed / 2.0,
                        p_motion->config.straight_gain * (right - left)));

        desired[0] += correction;
        desired[1] -= correction;
    }

    return false;
}

/* Target, output and no-progress check for one wheel.  True: it has stalled. */
static bool
drive_wheel (motion_t * p_motion, const motion_command_t * p_command,
             unsigned wheel, float64_t * p_desired, bool b_progressed,
             float64_t now, float64_t delta_s)
{
    bool      b_stalled = false;
    float64_t change;
    float64_t quantum  = (3.141592653589793 * p_motion->config.diameter[wheel])
                         / p_motion->config.counts_per_rev[wheel];
    bool      b_manual = MANUAL_OUTPUT == p_command->mode;

    (*p_desired) = fmax(-p_motion->config.max_speed,
                        fmin(p_motion->config.max_speed, (*p_desired)));
    change       = fmax(-p_motion->config.acceleration * delta_s,
                        fmin(p_motion->config.acceleration * delta_s,
                             (*p_desired) - p_motion->status.target[wheel]));
    p_motion->status.target[wheel] += change;
    p_motion->status.output[wheel]
        = b_manual
              ? p_command->speed[wheel]
              : pid_step(&p_motion->pid[wheel], &p_motion->config.pid[wheel],
                         p_motion->status.target[wheel],
                         p_motion->odom.speed[wheel], delta_s,
                         p_motion->config.max_output);

    /* Breakaway: from rest the PID integral took longer than the
     * no-progress limit to pass the motor's starting threshold, so a
     * stationary wheel asked to move or speed up gets at least
     * breakaway[w], doubling by the time that limit expires. Not while the
     * target is falling (braking ramp) and never for manual output; a truly
     * blocked wheel still trips the no-progress check below. */
    if ((!b_manual) && (!b_progressed)
        && (fabs(p_motion->status.target[wheel]) > 1e-9)
        && (((*p_desired) * p_motion->status.target[wheel]) > 0.0)
        && (fabs(*p_desired) >= fabs(p_motion->status.target[wheel])))
    {
        float64_t kick = fmin(p_motion->config.max_output,
                              p_motion->config.breakaway[wheel]
                                  * (1.0
                                     + ((now - p_motion->last_progress[wheel])
                                        / p_motion->config.feedback_timeout)));

        if (fabs(p_motion->status.output[wheel]) < kick)
        {
            p_motion->status.output[wheel]
                = copysign(kick, p_motion->status.target[wheel]);
        }
    }

    /* No active reverse torque at zero target. Hardware enforces reversal
     * dead-time. */
    if ((0.0 == (*p_desired)) && (fabs(p_motion->status.target[wheel]) < 1e-9))
    {
        p_motion->status.output[wheel] = 0.0;
        pid_reset(&p_motion->pid[wheel]);
    }

    if (b_progressed)
    {
        p_motion->last_progress[wheel] = now;
    }

    if ((fabs(p_motion->status.target[wheel]) < 1e-9)
        || (fabs(p_motion->status.output[wheel]) < 1e-9))
    {
        p_motion->expected_since[wheel] = now;
        p_motion->last_progress[wheel]  = now;
    }
    else if (MOTION_FAULTS_ENABLED)
    {
        float64_t timeout
            = p_motion->config.feedback_timeout
              + ((2.0 * quantum) / fabs(p_motion->status.target[wheel]));

        if (((now - p_motion->expected_since[wheel])
             >= p_motion->config.startup_grace)
            && ((now - p_motion->last_progress[wheel]) > timeout))
        {
            b_stalled = true;
        }
    }
    else
    {
        /* Nothing to do in any other case. */
    }

    return b_stalled;
}

static void
drive (motion_t * p_motion, float64_t now, float64_t delta_s,
       const bool progressed[2])
{
    motion_command_t command;
    uint8_t          stalled_wheels = 0U;

    if (!motion_compact_decode(&p_motion->command, &p_motion->config, &command))
    {
        motion_raise_fault(p_motion, F_SYSTEM, now);
        return;
    }

    float64_t desired[2] = {command.speed[0], command.speed[1]};
    unsigned  wheel;

    if (drive_expired(p_motion, &command, now))
    {
        return;
    }

    if ((ARC_MOVE == command.mode)
        && drive_arc(p_motion, &command, now, desired))
    {
        return;
    }

    if (((DISTANCE_MOVE == command.mode) || (ENCODER_TURN == command.mode))
        && drive_finite(p_motion, &command, now, desired))
    {
        return;
    }

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        if (drive_wheel(p_motion, &command, wheel, &desired[wheel],
                        progressed[wheel], now, delta_s))
        {
            stalled_wheels |= (uint8_t)(1U << wheel);
        }
    }

    /* Evaluate both wheels in the same cycle before capturing and zeroing
     * outputs. Report every wheel whose no-progress limit has expired. */
    if (0U != stalled_wheels)
    {
        encoder_sample_t sample
            = {{p_motion->odom.previous[0], p_motion->odom.previous[1]},
               {p_motion->invalid[0], p_motion->invalid[1]},
               true};

        record_feedback(p_motion, sample, FB_NO_PROGRESS);
        p_motion->status.debug.stalled_wheels = stalled_wheels;
        motion_raise_fault(p_motion, F_FEEDBACK, now);
    }
}

/* A long gap while starting up, outputs still disabled. */
static void
step_startup_gap (motion_t * p_motion, encoder_sample_t sample, float64_t now,
                  float64_t delta_s)
{
    unsigned wheel;

    /* Outputs are still disabled: qualify startup cadence/rest before
     * READY, rather than treating bring-up as a missed running control
     * deadline. Keep the original setup deadline; repeated gaps cannot
     * postpone it. */
    zero_output(p_motion);
    ++p_motion->status.startup_gaps;
    p_motion->status.max_startup_gap
        = fmax(p_motion->status.max_startup_gap, delta_s);
    motion_flag_set(p_motion, MOTION_STILL_TRACKING, false);
    p_motion->status.stationary = false;

    if (MOTION_FAULTS_ENABLED
        && ((now - p_motion->entered) > p_motion->config.setup_timeout))
    {
        motion_raise_fault(p_motion, F_SETUP_TIMEOUT, now);
        return;
    }

    if (MOTION_FAULTS_ENABLED
        && ((!sample.reliable) || (sample.invalid[0] != p_motion->invalid[0])
            || (sample.invalid[1] != p_motion->invalid[1])))
    {
        record_feedback(p_motion, sample,
                        sample.reliable ? FB_INVALID_TRANSITION
                                        : FB_UNRELIABLE);
        motion_raise_fault(p_motion, F_FEEDBACK, now);
        return;
    }

    /* Rebase the pre-drive encoder/time sample. Do not feed an oversized
     * interval into odometry/PID, or count it as stationary settling. */
    for (wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        p_motion->odom.previous[wheel] = sample.count[wheel];
        p_motion->odom.speed[wheel] = p_motion->status.speed[wheel] = 0.0;
    }

    p_motion->last_time = p_motion->status.sample_time = now;
    p_motion->status.feedback_trusted                  = false;
}

/* An unusable interval.  True: carry on with the interval given back. */
static bool
step_bad_interval (motion_t * p_motion, float64_t now, float64_t * p_delta_s)
{
    timing_fault(p_motion, now, *p_delta_s);

    if (isfinite(now))
    {
        p_motion->last_time = now;
    }

    if (MOTION_FAULTS_ENABLED)
    {
        return false;
    }

    if ((!isfinite(now)) || (!isfinite(*p_delta_s)) || (*p_delta_s <= 0.0))
    {
        /* No usable clock means no deadline checks: do not keep driving. */
        p_motion->status.output[0] = 0.0;
        p_motion->status.output[1] = 0.0;
        return false;
    }

    /* A late cycle must still reach the deadline checks below. */
    *p_delta_s = p_motion->config.max_dt;

    return true;
}

/* New encoder counts into odometry.  False: a fault, nothing more to do. */
static bool
step_odometry (motion_t * p_motion, encoder_sample_t sample, float64_t now,
               float64_t delta_s, bool progressed[2], bool * p_fresh)
{
    unsigned wheel;
    bool     b_fresh;

    if (!sample.reliable)
    {
        record_feedback(p_motion, sample, FB_UNRELIABLE);
    }

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        progressed[wheel]
            = sample.count[wheel] != p_motion->odom.previous[wheel];

        if (sample.invalid[wheel] != p_motion->invalid[wheel])
        {
            record_feedback(p_motion, sample, FB_INVALID_TRANSITION);

            if (MOTION_FAULTS_ENABLED)
            {
                sample.reliable = false;
            }
        }

        p_motion->invalid[wheel] = sample.invalid[wheel];
    }

    b_fresh = (sample.reliable)
              && (odometry_step(&p_motion->odom, &p_motion->config, &sample,
                                delta_s));

    if (!b_fresh)
    {
        record_feedback(p_motion, sample, FB_ODOMETRY);
        motion_raise_fault(p_motion, F_FEEDBACK, now);

        if (MOTION_FAULTS_ENABLED)
        {
            return false;
        }

        /* Rebase, or a rejected jump would be rejected again on every later
         * sample and the state machine below (deadlines, stopping) would never
         * run. */
        if (sample.reliable)
        {
            for (wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
            {
                p_motion->odom.previous[wheel] = sample.count[wheel];
            }
        }
    }

    *p_fresh = b_fresh;

    return true;
}

/* Standing still for long enough counts as stationary. */
static void
step_stationary (motion_t * p_motion, float64_t now)
{
    if ((fabs(p_motion->odom.speed[0]) <= p_motion->config.stopped_speed)
        && (fabs(p_motion->odom.speed[1]) <= p_motion->config.stopped_speed)
        && (p_motion->status.feedback_trusted))
    {
        if (!motion_flag_get(p_motion, MOTION_STILL_TRACKING))
        {
            p_motion->still_since = now;
            motion_flag_set(p_motion, MOTION_STILL_TRACKING, true);
        }

        p_motion->status.stationary
            = (now - p_motion->still_since) >= p_motion->config.settle_time;
    }
    else
    {
        motion_flag_set(p_motion, MOTION_STILL_TRACKING, false);
        p_motion->status.stationary = false;
    }
}

/* What this state does in one cycle. */
static void
step_state (motion_t * p_motion, float64_t now, float64_t delta_s,
            const bool progressed[2])
{
    switch (p_motion->status.state)
    {
        case INITIALISING:
            if (MOTION_FAULTS_ENABLED
                && ((now - p_motion->entered) > p_motion->config.setup_timeout))
            {
                motion_raise_fault(p_motion, F_SETUP_TIMEOUT, now);
            }
            else if (p_motion->status.stationary)
            {
                p_motion->status.state
                    = p_motion->status.hold ? STOPPED : READY;
            }
            else
            {
                /* Nothing to do in any other case. */
            }
        break;

        case DRIVING:
            drive(p_motion, now, delta_s, progressed);
        break;

        case STOPPING:
            if ((now - p_motion->entered) > p_motion->config.stop_timeout)
            {
                motion_raise_fault(p_motion, F_STOP_TIMEOUT, now);
            }
            else if (p_motion->status.stationary)
            {
                if (p_motion->status.hold)
                {
                    p_motion->status.state = STOPPED;
                    event(p_motion, EV_STOPPED);
                }
                else if ((motion_flag_get(p_motion, MOTION_GOAL_PENDING))
                         && (final_goal(p_motion)))
                {
                    p_motion->status.state = READY;
                    p_motion->active       = false;
                    motion_flag_set(p_motion, MOTION_GOAL_PENDING, false);
                    event(p_motion, EV_COMPLETE);
                }
                else
                {
                    motion_raise_fault(p_motion, F_FINAL_GOAL, now);
                }
            }
            else
            {
                /* Nothing to do in any other case. */
            }
        break;

        default:
        break;
    }
}

/*** end of file ***/
