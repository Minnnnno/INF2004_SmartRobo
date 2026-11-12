/* Buddy 4 motion classification; see motion_class.h for the fusion rationale. */
#include "motion_class.h"
#include "terrain.h"
#include <stddef.h>

#define MC_FLAG_VALID   (0x01U)
#define MC_FLAG_HOLD    (0x02U)
#define MC_FLAG_SEEDED  (0x04U)
#define MC_FLAG_TURN    (0x08U)
#define MC_FLAG_TURN_DONE (0x10U)
#define MC_RING_INDEX_MASK (0x07U)
#define MC_RING_COUNT_SHIFT (3U)
#define MC_RING_PENDING_SHIFT (6U) /* Turn start/end debounce count, 2 bits. */
#define MC_TI_COMPASS_OK   (0x01U)
#define MC_TI_LAST_COMPASS (0x02U)
#define MC_COMPASS_ALL     (0x1FU) /* All 5 ring slots valid. */
#define MC_CANDIDATE_SHIFT (3U)
#define MC_COUNT_SHIFT  (6U)
#define MC_COUNT_MASK   (0x03U)
#define MC_MDEG_PER_RAD (57296) /* 180000 / pi, rounded. */

static uint8_t mc_pack(uint8_t reported, uint8_t candidate, uint8_t count)
{
    return (uint8_t)((reported & MC_CLASS_MASK) |
                     ((candidate & MC_CLASS_MASK) << MC_CANDIDATE_SHIFT) |
                     ((count & MC_COUNT_MASK) << MC_COUNT_SHIFT));
}

static int16_t mc_clamp16(int32_t value)
{
    if (value > 32767) { return 32767; }
    if (value < -32768) { return -32768; }
    return (int16_t)value;
}

void motion_class_reset(motion_class_t *p_context)
{
    if (NULL != p_context)
    {
        *p_context = (motion_class_t){0};
    }
}

int16_t motion_class_yaw_from_speeds(int16_t left_mm_s, int16_t right_mm_s, uint16_t track_mm)
{
    int32_t difference = (int32_t)right_mm_s - (int32_t)left_mm_s;
    int32_t denominator = (int32_t)track_mm * 1000;
    int32_t scaled = 0;

    if (0U == track_mm)
    {
        return 0;
    }
    /* 64-bit product: |difference| x 57296 can exceed 2^31. Round half away from 0. */
    scaled = (int32_t)(((int64_t)difference * MC_MDEG_PER_RAD +
                        ((difference >= 0) ? (denominator / 2) : -(denominator / 2))) /
                       denominator);
    return mc_clamp16(scaled);
}

static int16_t mc_abs16(int16_t value)
{
    return (value < 0) ? mc_clamp16(-(int32_t)value) : value;
}

static uint8_t mc_ring_count(motion_class_t const *p_context)
{
    return (uint8_t)((p_context->ring >> MC_RING_COUNT_SHIFT) & 0x07U);
}

/* Start the heading window again from one sample. */
static void mc_ring_seed(motion_class_t *p_context, motion_class_input_t const *p_input,
                         uint32_t now_ms)
{
    p_context->heading_ring[0] = p_input->heading_ddeg;
    p_context->compass_ring[0] = p_input->compass_ddeg;
    p_context->compass_bits = p_input->compass_valid ? 1U : 0U;
    p_context->heading_ms = now_ms;
    p_context->ring = (uint8_t)(1U | (1U << MC_RING_COUNT_SHIFT)); /* next write 1, count 1 */
}

/* Add a heading sample every 100 ms. Reseeds after a gap, an invalid heading
 * or an impossible jump (Buddy 2 reset its odometry). Returns true with the
 * net change over the full window when 5 samples are held.
 */
static bool mc_heading_window(motion_class_t *p_context, motion_class_input_t const *p_input,
                              uint32_t now_ms, int32_t *p_net_ddeg, int32_t *p_oldest_ddeg)
{
    uint8_t count = mc_ring_count(p_context);
    uint8_t next = (uint8_t)(p_context->ring & MC_RING_INDEX_MASK);
    uint8_t newest = (uint8_t)((next + MC_HEADING_SLOTS - 1U) % MC_HEADING_SLOTS);
    int32_t jump = 0;

    if (!p_input->heading_valid)
    {
        p_context->ring = 0U;
        return false;
    }
    if ((0U == count) || ((now_ms - p_context->last_ms) > MC_MAX_GAP_MS))
    {
        mc_ring_seed(p_context, p_input, now_ms);
        return false;
    }
    jump = p_input->heading_ddeg - p_context->heading_ring[newest];
    if ((jump > MC_HEADING_JUMP_DDEG) || (jump < -MC_HEADING_JUMP_DDEG))
    {
        mc_ring_seed(p_context, p_input, now_ms);
        p_context->flags &= (uint8_t)~MC_FLAG_TURN; /* Drop a turn across a reset. */
        return false;
    }
    if ((now_ms - p_context->heading_ms) >= MC_HEADING_STEP_MS)
    {
        p_context->heading_ring[next] = p_input->heading_ddeg;
        p_context->compass_ring[next] = p_input->compass_ddeg;
        if (p_input->compass_valid) { p_context->compass_bits |= (uint8_t)(1U << next); }
        else { p_context->compass_bits &= (uint8_t)~(uint8_t)(1U << next); }
        p_context->heading_ms = now_ms;
        next = (uint8_t)((next + 1U) % MC_HEADING_SLOTS);
        if (count < MC_HEADING_SLOTS) { count++; }
        p_context->ring = (uint8_t)(next | (uint8_t)(count << MC_RING_COUNT_SHIFT) |
                                    (p_context->ring & (uint8_t)(0x03U << MC_RING_PENDING_SHIFT)));
    }
    if (count < MC_HEADING_SLOTS)
    {
        return false;
    }
    /* Full ring: the next write position holds the oldest sample. */
    *p_oldest_ddeg = p_context->heading_ring[next];
    *p_net_ddeg = p_input->heading_ddeg - *p_oldest_ddeg;
    return true;
}

/* Record the compass angle over the finished turn (same interval as the
 * encoder angle), when the compass stayed valid for the whole turn. */
static void mc_record_compass(motion_class_t *p_context, motion_class_input_t const *p_input)
{
    p_context->turn_info &= (uint8_t)~MC_TI_LAST_COMPASS;
    if ((0U == (p_context->turn_info & MC_TI_COMPASS_OK)) || !p_input->compass_valid)
    {
        return;
    }
    p_context->last_turn_compass = mc_clamp16(p_input->compass_ddeg - p_context->turn_compass_start);
    p_context->turn_info |= MC_TI_LAST_COMPASS;
}

/* Track one turn from the heading condition itself, independent of which
 * class is shown on top, so a knock or a hump during a turn does not split it.
 * Start and end use the same 3-cycle debounce as the class. The start is
 * backdated to the window start; the result is latched when the turn ends. */
static void mc_track_turn(motion_class_t *p_context, motion_class_input_t const *p_input,
                          bool turning_now, bool window_full, int32_t oldest_ddeg,
                          uint32_t now_ms)
{
    int16_t yaw = mc_abs16(p_context->yaw_deg_s);
    bool active = (0U != (p_context->flags & MC_FLAG_TURN));
    uint8_t pending = (uint8_t)((p_context->ring >> MC_RING_PENDING_SHIFT) & 0x03U);

    if (turning_now != active)
    {
        pending++;
        if ((uint8_t)(pending + 1U) >= MC_CONFIRM_SAMPLES)
        {
            pending = 0U;
            active = turning_now;
            if (!active)
            {
                int32_t angle = p_input->heading_ddeg - p_context->turn_start_ddeg;
                uint32_t duration = p_context->turn_moving_ms - p_context->turn_start_ms;

                p_context->flags &= (uint8_t)~MC_FLAG_TURN;
                p_context->last_turn_ddeg = mc_clamp16(angle);
                p_context->last_turn_ms = (uint16_t)((duration > 65535U) ? 65535U : duration);
                p_context->last_turn_peak = p_context->turn_peak_deg_s;
                p_context->flags |= MC_FLAG_TURN_DONE;
                mc_record_compass(p_context, p_input);
            }
        }
    }
    else
    {
        pending = 0U;
    }
    p_context->ring = (uint8_t)((p_context->ring & 0x3FU) | (uint8_t)(pending << MC_RING_PENDING_SHIFT));

    if (active)
    {
        if (0U == (p_context->flags & MC_FLAG_TURN))
        {
            p_context->flags |= MC_FLAG_TURN;
            p_context->turn_start_ddeg = window_full ? oldest_ddeg : p_input->heading_ddeg;
            p_context->turn_start_ms = now_ms - (window_full ? MC_TURN_WINDOW_MS : 0U);
            p_context->turn_moving_ms = now_ms;
            p_context->turn_peak_deg_s = 0;
            {
                /* Compass start from the same (oldest) ring slot as the encoder
                 * start; usable only if the whole window was valid. */
                uint8_t oldest = (uint8_t)(p_context->ring & MC_RING_INDEX_MASK);
                bool ok = window_full && p_input->compass_valid &&
                          (MC_COMPASS_ALL == (p_context->compass_bits & MC_COMPASS_ALL));
                p_context->turn_compass_start = window_full ? p_context->compass_ring[oldest]
                                                            : p_input->compass_ddeg;
                if (ok) { p_context->turn_info |= MC_TI_COMPASS_OK; }
                else { p_context->turn_info &= (uint8_t)~MC_TI_COMPASS_OK; }
            }
        }
        if (!p_input->compass_valid)
        {
            p_context->turn_info &= (uint8_t)~MC_TI_COMPASS_OK; /* n/a for this turn */
        }
        if (yaw > p_context->turn_peak_deg_s) { p_context->turn_peak_deg_s = yaw; }
        if (yaw >= MC_TURN_MOVING_DEG_S) { p_context->turn_moving_ms = now_ms; }
    }
}

/* Unconfirmed classification of this one observation, highest priority first. */
static uint8_t mc_raw_class(motion_class_t const *p_context, motion_class_input_t const *p_input,
                            int16_t centre_mm_s, bool window_full, int32_t net_ddeg)
{

    if (0U != (p_context->flags & MC_FLAG_HOLD))
    {
        return MC_IMPACT;
    }
    if (TERRAIN_CLIMBING == p_input->terrain_phase)
    {
        return MC_CLIMBING;
    }
    if (TERRAIN_DESCENDING == p_input->terrain_phase)
    {
        return MC_DESCENDING;
    }
    if (p_input->stationary)
    {
        return MC_STATIONARY;
    }
    if (window_full && ((net_ddeg >= MC_TURN_NET_DDEG) || (net_ddeg <= -MC_TURN_NET_DDEG)))
    {
        return MC_TURNING; /* Heading really changed; wiggles net close to 0. */
    }
    if (centre_mm_s <= MC_REVERSE_MM_S)
    {
        return MC_REVERSING; /* Starting/stopping a reverse also reports REVERSING. */
    }
    if ((p_context->accel_mm_s2 >= MC_ACCEL_MM_S2) || (p_context->accel_mm_s2 <= -MC_ACCEL_MM_S2))
    {
        return MC_ACCELERATING; /* Speeding up or slowing down (braking). */
    }
    return MC_MOVING;
}

/* Change of centre speed SIZE over a ~100 ms window, so reversing away is
 * speeding up (+), not braking. The last value is held between updates. */
static void mc_update_acceleration(motion_class_t *p_context, int16_t centre_mm_s, uint32_t now_ms)
{
    uint32_t elapsed = now_ms - p_context->window_ms;
    int16_t size = (centre_mm_s < 0) ? mc_clamp16(-(int32_t)centre_mm_s) : centre_mm_s;

    if ((0U == (p_context->flags & MC_FLAG_SEEDED)) ||
        ((now_ms - p_context->last_ms) > MC_MAX_GAP_MS))
    {
        p_context->window_ms = now_ms;
        p_context->window_mm_s = size;
        p_context->accel_mm_s2 = 0;
        p_context->flags |= MC_FLAG_SEEDED;
        return;
    }
    if (elapsed >= MC_ACCEL_WINDOW_MS)
    {
        int32_t change = (int32_t)size - (int32_t)p_context->window_mm_s;
        p_context->accel_mm_s2 = mc_clamp16((change * 1000) / (int32_t)elapsed);
        p_context->window_ms = now_ms;
        p_context->window_mm_s = size;
    }
}

void motion_class_step(motion_class_t *p_context, motion_class_input_t const *p_input,
                       uint32_t now_ms)
{
    uint8_t reported = 0U;
    uint8_t candidate = 0U;
    uint8_t count = 0U;
    uint8_t raw = 0U;
    int16_t centre = 0;
    int32_t net_ddeg = 0;
    int32_t oldest_ddeg = 0;
    bool window_full = false;

    if ((NULL == p_context) || (NULL == p_input))
    {
        return;
    }
    if (!p_input->valid)
    {
        p_context->flags = 0U; /* Also drops an unfinished turn (not reported). */
        p_context->ring = 0U;
        p_context->turn_info = 0U;
        p_context->packed = mc_pack(MC_STATIONARY, MC_STATIONARY, 0U);
        p_context->yaw_deg_s = 0;
        p_context->accel_mm_s2 = 0;
        return;
    }
    centre = mc_clamp16(((int32_t)p_input->speed_mm_s[0] + (int32_t)p_input->speed_mm_s[1]) / 2);
    p_context->yaw_deg_s = motion_class_yaw_from_speeds(p_input->speed_mm_s[0],
                                                        p_input->speed_mm_s[1], p_input->track_mm);
    mc_update_acceleration(p_context, centre, now_ms);
    window_full = mc_heading_window(p_context, p_input, now_ms, &net_ddeg, &oldest_ddeg);
    p_context->last_ms = now_ms;

    if (p_input->impact)
    {
        p_context->impact_since = now_ms;
        p_context->flags |= MC_FLAG_HOLD;
    }
    else if ((0U != (p_context->flags & MC_FLAG_HOLD)) &&
             ((now_ms - p_context->impact_since) >= MC_IMPACT_HOLD_MS))
    {
        p_context->flags &= (uint8_t)~MC_FLAG_HOLD;
    }

    reported = p_context->packed & MC_CLASS_MASK;
    candidate = (uint8_t)((p_context->packed >> MC_CANDIDATE_SHIFT) & MC_CLASS_MASK);
    count = (uint8_t)((p_context->packed >> MC_COUNT_SHIFT) & MC_COUNT_MASK);
    raw = mc_raw_class(p_context, p_input, centre, window_full, net_ddeg);

    if (0U == (p_context->flags & MC_FLAG_VALID))
    {
        /* First valid observation is reported directly (nothing to debounce against). */
        reported = raw; candidate = raw; count = 0U;
    }
    else if ((raw == reported) || (MC_IMPACT == raw))
    {
        reported = raw; candidate = raw; count = 0U; /* IMPACT is never delayed. */
    }
    else if (raw == candidate)
    {
        count++;
        if ((uint8_t)(count + 1U) >= MC_CONFIRM_SAMPLES)
        {
            reported = raw; count = 0U;
        }
    }
    else
    {
        candidate = raw; count = 0U;
    }
    p_context->flags |= MC_FLAG_VALID;
    p_context->packed = mc_pack(reported, candidate, count);
    mc_track_turn(p_context, p_input,
                  window_full && ((net_ddeg >= MC_TURN_NET_DDEG) || (net_ddeg <= -MC_TURN_NET_DDEG)),
                  window_full, oldest_ddeg, now_ms);
}

bool motion_class_turn_compass(motion_class_t const *p_context, int16_t *p_compass_ddeg)
{
    if ((NULL == p_context) || (0U == (p_context->turn_info & MC_TI_LAST_COMPASS)))
    {
        return false;
    }
    if (NULL != p_compass_ddeg) { *p_compass_ddeg = p_context->last_turn_compass; }
    return true;
}

bool motion_class_take_turn(motion_class_t *p_context, int16_t *p_angle_ddeg,
                            uint16_t *p_duration_ms, int16_t *p_peak_deg_s)
{
    if ((NULL == p_context) || (0U == (p_context->flags & MC_FLAG_TURN_DONE)))
    {
        return false;
    }
    p_context->flags &= (uint8_t)~MC_FLAG_TURN_DONE;
    if (NULL != p_angle_ddeg) { *p_angle_ddeg = p_context->last_turn_ddeg; }
    if (NULL != p_duration_ms) { *p_duration_ms = p_context->last_turn_ms; }
    if (NULL != p_peak_deg_s) { *p_peak_deg_s = p_context->last_turn_peak; }
    return true;
}

uint8_t motion_class_get(motion_class_t const *p_context)
{
    return (NULL == p_context) ? MC_STATIONARY : (uint8_t)(p_context->packed & MC_CLASS_MASK);
}

bool motion_class_valid(motion_class_t const *p_context)
{
    return (NULL != p_context) && (0U != (p_context->flags & MC_FLAG_VALID));
}

bool motion_class_turn_active(motion_class_t const *p_context)
{
    return (NULL != p_context) && (0U != (p_context->flags & MC_FLAG_TURN));
}

int16_t motion_class_yaw_deg_s(motion_class_t const *p_context)
{
    return (NULL == p_context) ? 0 : p_context->yaw_deg_s;
}

int16_t motion_class_accel_mm_s2(motion_class_t const *p_context)
{
    return (NULL == p_context) ? 0 : p_context->accel_mm_s2;
}

char const *motion_class_name(uint8_t motion_class)
{
    static char const *const g_names[] =
    {
        "STATIONARY", "MOVING", "ACCELERATING", "REVERSING",
        "TURNING", "CLIMBING", "DESCENDING", "IMPACT"
    };
    return g_names[motion_class & MC_CLASS_MASK];
}
