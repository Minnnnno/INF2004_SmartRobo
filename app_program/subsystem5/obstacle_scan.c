/*
 *  obstacle_scan.c
 *  Subsystem 5 -- obstacle width, position and which side to pass it on,
 *  from a servo-swept ultrasonic ranger.
 *  Pure computation; see obstacle_scan.h.
 */

#include "obstacle_scan.h"
#include <math.h>
#include <string.h>

#define S5_DEG_TO_RAD (0.017453292519943295)

static void s5_face_apply(s5_scan_t * p_scan);

_Static_assert(S5_SAMPLES_PER_POINT + S5_RETRY_SAMPLES <= S5_MAX_SAMPLES,
               "S5_MAX_SAMPLES too small for one point");
_Static_assert(S5_SAMPLES_PER_POINT >= 3U,
               "take at least three pings per angle");
_Static_assert(S5_MAX_SAMPLES <= 15U, "support and valid are 4 bits");
_Static_assert(S5_ANGLE_RIGHT_DEG - S5_ANGLE_LEFT_DEG <= 255,
               "a stored angle is 8 bits");
_Static_assert(S5_FINE_MAX_POINTS + S5_INWARD_MAX_STEPS <= 255,
               "point counts are 8 bits");
_Static_assert(S5_CURVE_SAMPLES == 16, "s5_curve_t.count is 5 bits, .next 4");
_Static_assert(sizeof(s5_point_t) == 4U, "a stored reading is one word");
_Static_assert(sizeof(s5_curve_t) == 72U, "packed curve history");

static double
s5_bearing_rad (double angle_deg)
{
    /* + left */
    return ((double)S5_ANGLE_CENTRE_DEG - angle_deg) * S5_DEG_TO_RAD;
}

double
s5_forward_mm (double range_mm, double angle_deg)
{
    return (range_mm + S5_SENSOR_OFFSET_MM) * cos(s5_bearing_rad(angle_deg));
}

double
s5_lateral_mm (double range_mm, double angle_deg)
{
    return (range_mm + S5_SENSOR_OFFSET_MM) * sin(s5_bearing_rad(angle_deg));
}

/* ---- stored readings (obstacle_scan.h) ---- */

#define S5_POINT_RANGE_MASK    0x3fffU
#define S5_POINT_ANGLE_SHIFT   14U
#define S5_POINT_ANGLE_MASK    0xffU
#define S5_POINT_SUPPORT_SHIFT 22U
#define S5_POINT_VALID_SHIFT   26U
#define S5_POINT_COUNT_MASK    0xfU
#define S5_POINT_RETRIED       (1UL << 30U)
#define S5_POINT_OBJECT        (1UL << 31U)

uint16_t
s5_range_code (double range_mm)
{
    if (!(range_mm > 0.0))
    {
        return 0U;
    }

    if (range_mm >= (double)S5_RANGE_CODE_MAX / S5_RANGE_STEPS_PER_MM)
    {
        return (uint16_t)S5_RANGE_CODE_MAX;
    }

    return (uint16_t)(range_mm * S5_RANGE_STEPS_PER_MM + 0.5);
}

double
s5_range_mm (unsigned code)
{
    return (double)code / S5_RANGE_STEPS_PER_MM;
}

static uint32_t
s5_count_bits (unsigned count)
{
    return (count > S5_POINT_COUNT_MASK) ? S5_POINT_COUNT_MASK
                                         : (uint32_t)count;
}

s5_point_t
s5_point_pack (int angle_deg, const s5_estimate_t * p_estimate, bool b_object)
{
    int        angle = angle_deg - S5_ANGLE_LEFT_DEG;
    s5_point_t point;

    angle = (angle < 0) ? 0
                        : ((angle > S5_ANGLE_RIGHT_DEG - S5_ANGLE_LEFT_DEG)
                               ? S5_ANGLE_RIGHT_DEG - S5_ANGLE_LEFT_DEG
                               : angle);
    point.bits
        = (uint32_t)s5_range_code(p_estimate->range_mm)
          | ((uint32_t)angle << S5_POINT_ANGLE_SHIFT)
          | (s5_count_bits(p_estimate->support) << S5_POINT_SUPPORT_SHIFT)
          | (s5_count_bits(p_estimate->valid) << S5_POINT_VALID_SHIFT);

    if (p_estimate->total > S5_SAMPLES_PER_POINT)
    {
        point.bits |= S5_POINT_RETRIED;
    }

    if (b_object)
    {
        point.bits |= S5_POINT_OBJECT;
    }

    return point;
}

int
s5_point_angle (s5_point_t point)
{
    return (int)((point.bits >> S5_POINT_ANGLE_SHIFT) & S5_POINT_ANGLE_MASK)
           + S5_ANGLE_LEFT_DEG;
}

double
s5_point_range (s5_point_t point)
{
    return s5_range_mm((unsigned)(point.bits & S5_POINT_RANGE_MASK));
}

unsigned
s5_point_support (s5_point_t point)
{
    return (unsigned)((point.bits >> S5_POINT_SUPPORT_SHIFT)
                      & S5_POINT_COUNT_MASK);
}

unsigned
s5_point_valid (s5_point_t point)
{
    return (unsigned)((point.bits >> S5_POINT_VALID_SHIFT)
                      & S5_POINT_COUNT_MASK);
}

unsigned
s5_point_total (s5_point_t point)
{
    return ((point.bits & S5_POINT_RETRIED) != 0U)
               ? S5_SAMPLES_PER_POINT + S5_RETRY_SAMPLES
               : S5_SAMPLES_PER_POINT;
}

bool
s5_point_object (s5_point_t point)
{
    return (point.bits & S5_POINT_OBJECT) != 0U;
}

void
s5_point_set_object (s5_point_t * p_point)
{
    p_point->bits |= S5_POINT_OBJECT;
}

/* What the pings said: the same rule for a fresh estimate and a stored one */
static s5_echo_t
s5_vote (unsigned support, unsigned valid, unsigned total)
{
    if ((total - valid) * 2U > total)
    {
        return S5_ECHO_NONE; /* mostly silent */
    }

    return ((support >= S5_MIN_AGREE) && (support * 2U >= total))
               ? S5_ECHO_OK
               : S5_ECHO_UNSTABLE;
}

static bool
s5_vote_usable (s5_echo_t status, unsigned support, unsigned valid,
                unsigned total)
{
    return (status == S5_ECHO_OK)
           || ((status == S5_ECHO_UNSTABLE) && (support > total - valid));
}

s5_echo_t
s5_point_status (s5_point_t point)
{
    return s5_vote(s5_point_support(point), s5_point_valid(point),
                   s5_point_total(point));
}

bool
s5_point_usable (s5_point_t point)
{
    return s5_vote_usable(s5_point_status(point), s5_point_support(point),
                          s5_point_valid(point), s5_point_total(point));
}

static bool
s5_echo_bit (uint16_t echo, unsigned index)
{
    return ((echo >> index) & 1U) != 0U;
}

void
s5_estimate (const double range_mm[], uint16_t echo, unsigned count,
             s5_estimate_t * p_out)
{
    unsigned i, j;
    unsigned best         = count;
    unsigned best_support = 0U;
    double   best_spread  = 0.0;
    double   sum          = 0.0;

    p_out->status      = S5_ECHO_NONE;
    p_out->range_mm    = 0.0;
    p_out->support     = 0U;
    p_out->valid       = 0U;
    p_out->total       = count;
    p_out->nearest_mm  = 0.0;
    p_out->farthest_mm = 0.0;

    for (i = 0U; i < count; i++)
    {
        unsigned support = 0U;
        double   spread  = 0.0;

        if (!s5_echo_bit(echo, i))
        {
            continue;
        }

        if ((p_out->valid == 0U) || (range_mm[i] < p_out->nearest_mm))
        {
            p_out->nearest_mm = range_mm[i];
        }

        if ((p_out->valid == 0U) || (range_mm[i] > p_out->farthest_mm))
        {
            p_out->farthest_mm = range_mm[i];
        }

        p_out->valid++;

        for (j = 0U; j < count; j++)
        {
            double gap = fabs(range_mm[j] - range_mm[i]);

            if (s5_echo_bit(echo, j) && (gap <= S5_CLUSTER_TOL_MM))
            {
                support++;
                spread += gap;
            }
        }

        /* Most agreeing pings wins, then the tightest group, then the
           nearer echo (a multipath echo only ever travels further). */
        if ((best == count) || (support > best_support)
            || ((support == best_support)
                && ((spread < best_spread)
                    || ((spread == best_spread)
                        && (range_mm[i] < range_mm[best])))))
        {
            best         = i;
            best_support = support;
            best_spread  = spread;
        }
    }

    if (best == count)
    {
        return; /* no echo at all */
    }

    for (j = 0U; j < count; j++)
    {
        if (s5_echo_bit(echo, j)
            && (fabs(range_mm[j] - range_mm[best]) <= S5_CLUSTER_TOL_MM))
        {
            sum += range_mm[j];
        }
    }

    p_out->range_mm = sum / (double)best_support;
    p_out->support  = best_support;

    p_out->status = s5_vote(best_support, p_out->valid, count);
}

bool
s5_estimate_usable (const s5_estimate_t * p_estimate)
{
    return s5_vote_usable(p_estimate->status, p_estimate->support,
                          p_estimate->valid, p_estimate->total);
}

bool
s5_point_is_object (const s5_estimate_t * p_estimate, int angle_deg,
                    double centre_forward_mm)
{
    return s5_estimate_usable(p_estimate)
           && (fabs(s5_forward_mm(p_estimate->range_mm, (double)angle_deg)
                    - centre_forward_mm)
               <= S5_EDGE_THRESHOLD_FRACTION * centre_forward_mm);
}

void
s5_sample_point (s5_ping_fn ping, void * p_context, s5_estimate_t * p_out)
{
    double   range_mm[S5_MAX_SAMPLES];
    uint16_t echo  = 0U; /* a bit per ping */
    unsigned count = 0U;

    while (count < S5_SAMPLES_PER_POINT)
    {
        range_mm[count] = 0.0;

        if (ping(p_context, &range_mm[count]))
        {
            echo = (uint16_t)(echo | (1U << count));
        }

        count++;
    }

    s5_estimate(range_mm, echo, count, p_out);

    if (p_out->status != S5_ECHO_UNSTABLE)
    {
        return;
    }

    /* Disagreement (often half the beam on the edge): vote again with more
     * pings */
    while (count < S5_SAMPLES_PER_POINT + S5_RETRY_SAMPLES)
    {
        range_mm[count] = 0.0;

        if (ping(p_context, &range_mm[count]))
        {
            echo = (uint16_t)(echo | (1U << count));
        }

        count++;
    }

    s5_estimate(range_mm, echo, count, p_out);
}

/* Same surface as the previous point on the obstacle: see S5_CONTINUITY_MM.
 *p_prev_mm / *p_have follow the last point taken as the obstacle. */
static void
s5_follow_surface (s5_point_t * p_point, double * p_prev_mm, bool * p_have)
{
    bool   b_usable = s5_point_usable(*p_point);
    double range_mm = s5_point_range(*p_point);

    if (!s5_point_object(*p_point) && *p_have && b_usable
        && (fabs(range_mm - *p_prev_mm) <= S5_CONTINUITY_MM))
    {
        s5_point_set_object(p_point);
    }

    if (s5_point_object(*p_point) && b_usable)
    {
        *p_prev_mm = range_mm;
        *p_have    = true;
    }
}

static bool
s5_measure_point (s5_point_t * p_point, int angle_deg, double centre_forward_mm,
                  s5_measure_fn measure, void * p_context)
{
    s5_estimate_t estimate;

    if (!measure(p_context, angle_deg, &estimate))
    {
        return false;
    }

    *p_point = s5_point_pack(
        angle_deg, &estimate,
        s5_point_is_object(&estimate, angle_deg, centre_forward_mm));
    return true;
}

/* Walk back inward from last_deg (clear space, where the outward walk
   left the servo) until the obstacle is seen twice in a row.  The servo now
   reaches every angle moving the other way.  No further in than straight
   ahead or S5_INWARD_MAX_STEPS points: backlash can put the edge inside
   the coarse edge angle, so that is no limit. */
static bool
s5_walk_inward (s5_scan_t * p_scan, unsigned side, int last_deg,
                s5_measure_fn measure, void * p_context)
{
    s5_edge_t * p_edge = &p_scan->edge[side];
    int         step = (side == S5_LEFT) ? S5_FINE_STEP_DEG : -S5_FINE_STEP_DEG;
    int         angle            = last_deg;
    int         clear_deg        = last_deg;
    int         first_object_deg = last_deg;
    unsigned    run              = 0U;
    unsigned    taken;
    double      prev_mm = 0.0;
    bool        b_have  = false;

    for (taken = 0U; taken < S5_INWARD_MAX_STEPS; taken++)
    {
        s5_point_t * p_point = &p_scan->fine[side][p_scan->fine_count[side]];

        angle += step;

        if ((step > 0) ? (angle > S5_ANGLE_CENTRE_DEG)
                       : (angle < S5_ANGLE_CENTRE_DEG))
        {
            break;
        }

        if (!s5_measure_point(p_point, angle, p_scan->centre_forward_mm,
                              measure, p_context))
        {
            return false;
        }

        p_scan->fine_count[side]++;
        s5_follow_surface(p_point, &prev_mm, &b_have);

        if (!s5_point_object(*p_point))
        {
            run       = 0U;
            clear_deg = angle;
        }
        else if (++run == 1U)
        {
            first_object_deg = angle;
        }
        else
        {
            p_edge->b_inward   = true;
            p_edge->inward_deg = 0.5 * (double)(clear_deg + first_object_deg);
            break;
        }
    }

    return true;
}

/* Walk outward 1 deg at a time from start_deg (the outermost coarse point
   still on the obstacle).  The edge is passed once two clear points in a
   row are seen, so one dropped ping cannot end the obstacle early.  Then
   walk back inward and average the two crossings. */
static bool
s5_find_edge (s5_scan_t * p_scan, unsigned side, int start_deg,
              double start_range_mm, s5_measure_fn measure, void * p_context)
{
    s5_edge_t * p_edge = &p_scan->edge[side];
    int         step = (side == S5_LEFT) ? -S5_FINE_STEP_DEG : S5_FINE_STEP_DEG;
    int    limit = (side == S5_LEFT) ? S5_ANGLE_LEFT_DEG : S5_ANGLE_RIGHT_DEG;
    int    angle;
    int    last_deg  = start_deg;
    int    clear_deg = start_deg;
    bool   b_pending = false;
    double edge_deg;
    double prev_mm = start_range_mm;
    bool   b_have  = true;

    p_edge->b_found         = false;
    p_edge->b_inward        = false;
    p_edge->last_object_deg = (uint8_t)start_deg;
    p_edge->range_mm        = start_range_mm;

    for (angle = start_deg + step;
         (step < 0) ? (angle >= limit) : (angle <= limit); angle += step)
    {
        s5_point_t * p_point = &p_scan->fine[side][p_scan->fine_count[side]];

        if (!s5_measure_point(p_point, angle, p_scan->centre_forward_mm,
                              measure, p_context))
        {
            return false;
        }

        p_scan->fine_count[side]++;
        s5_follow_surface(p_point, &prev_mm, &b_have);
        last_deg = angle;

        if (s5_point_object(*p_point))
        {
            p_edge->last_object_deg = (uint8_t)angle;
            p_edge->range_mm        = s5_point_range(*p_point);
            b_pending               = false;
        }
        else if (!b_pending)
        {
            b_pending = true;
            clear_deg = angle;
        }
        else
        {
            p_edge->b_found = true;
            break;
        }
    }

    if (b_pending)
    {
        p_edge->b_found = true; /* also: one clear point at the scan limit */
    }

    /* Halfway between the last obstacle and the first clear angle */
    p_edge->first_clear_deg
        = p_edge->b_found ? (uint8_t)clear_deg : p_edge->last_object_deg;
    p_edge->outward_deg
        = 0.5 * (double)(p_edge->last_object_deg + p_edge->first_clear_deg);
    p_edge->inward_deg         = p_edge->outward_deg;
    p_scan->fine_outward[side] = p_scan->fine_count[side];

    if (p_edge->b_found
        && !s5_walk_inward(p_scan, side, last_deg, measure, p_context))
    {
        return false;
    }

    /* Mean of both crossings, moved inward (towards 90 deg) by the beam
       correction */
    edge_deg = 0.5 * (p_edge->outward_deg + p_edge->inward_deg);

    if (side == S5_LEFT)
    {
        edge_deg = fmin(edge_deg + S5_BEAM_CORRECTION_DEG,
                        (double)S5_ANGLE_CENTRE_DEG);
    }
    else
    {
        edge_deg = fmax(edge_deg - S5_BEAM_CORRECTION_DEG,
                        (double)S5_ANGLE_CENTRE_DEG);
    }

    p_edge->angle_deg = edge_deg;
    /* Project along the face, not the edge's own range: near an edge the
       echo comes from the corner, off the corrected bearing, and reads long. */
    p_edge->lateral_mm
        = p_scan->centre_forward_mm * tan(s5_bearing_rad(edge_deg));
    return true;
}

/* Outermost coarse point, walking out from 90 deg, before the first clear one
 */
static void
s5_coarse_edge (const s5_scan_t * p_scan, unsigned side, int * p_angle_deg,
                double * p_range_mm)
{
    int step = (side == S5_LEFT) ? -S5_COARSE_STEP_DEG : S5_COARSE_STEP_DEG;
    int angle;

    *p_angle_deg = S5_ANGLE_CENTRE_DEG;
    *p_range_mm  = s5_point_range(p_scan->centre);
    for (angle = S5_ANGLE_CENTRE_DEG + step;
         (angle >= S5_ANGLE_LEFT_DEG) && (angle <= S5_ANGLE_RIGHT_DEG);
         angle += step)
    {
        s5_point_t point
            = p_scan->coarse[(angle - S5_ANGLE_LEFT_DEG) / S5_COARSE_STEP_DEG];

        if (!s5_point_object(point))
        {
            break;
        }

        *p_angle_deg = angle;
        *p_range_mm  = s5_point_range(point);
    }
}

/* ---- the face ---- */

#define S5_FACE_ANGLES (S5_ANGLE_RIGHT_DEG - S5_ANGLE_LEFT_DEG + 1)
#define S5_FACE_CENTRE (S5_ANGLE_CENTRE_DEG - S5_ANGLE_LEFT_DEG)

/* On the task stack: 318 bytes */
typedef struct
{
    uint16_t range[S5_FACE_ANGLES];           /* s5_range_code() */
    uint8_t  taken[(S5_FACE_ANGLES + 7) / 8]; /* a bit per angle */
    uint8_t  echo[(S5_FACE_ANGLES + 7) / 8];
} s5_profile_t;

_Static_assert(sizeof(s5_profile_t) == 318U, "packed face profile");

static bool
s5_profile_bit (const uint8_t map[], int index)
{
    return ((map[index / 8] >> (index % 8)) & 1U) != 0U;
}

static bool
s5_profile_taken (const s5_profile_t * p_profile, int index)
{
    return s5_profile_bit(p_profile->taken, index);
}

static bool
s5_profile_echo (const s5_profile_t * p_profile, int index)
{
    return s5_profile_bit(p_profile->echo, index);
}

static double
s5_profile_mm (const s5_profile_t * p_profile, int index)
{
    return s5_range_mm(p_profile->range[index]);
}

static void
s5_profile_add (s5_profile_t * p_profile, s5_point_t point)
{
    int index = s5_point_angle(point) - S5_ANGLE_LEFT_DEG;

    if ((index >= 0) && (index < S5_FACE_ANGLES))
    {
        unsigned byte = (unsigned)index / 8U;
        unsigned bit  = 1U << ((unsigned)index % 8U);

        p_profile->taken[byte] = (uint8_t)(p_profile->taken[byte] | bit);
        /* Measured twice: the later reading counts */
        p_profile->echo[byte] = (uint8_t)(s5_point_usable(point)
                                              ? (p_profile->echo[byte] | bit)
                                              : (p_profile->echo[byte] & ~bit));
        p_profile->range[index] = s5_range_code(s5_point_range(point));
    }
}

/* Out along the face from start until the sweep ends (*p_jump false) or it
   does.  Returns where it was last heard, *p_before the reading before. */
static int
s5_face_walk (const s5_profile_t * p_profile, int start, int step,
              int * p_before, bool * p_jump, unsigned * p_count)
{
    int end = start;
    int index;

    *p_before = start;
    *p_jump   = false;
    for (index = start + step; (index >= 0) && (index < S5_FACE_ANGLES);
         index += step)
    {
        if (!s5_profile_taken(p_profile, index))
        {
            continue;
        }

        if (!s5_profile_echo(p_profile, index)
            || (fabs(s5_profile_mm(p_profile, index)
                     - s5_profile_mm(p_profile, end))
                > S5_FACE_JUMP_MM))
        {
            *p_jump = true;
            break;
        }

        *p_before = end;
        end       = index;
        (*p_count)++;
    }

    return end;
}

/* A jump can still be the same plane: a flat wall's range rises fast between
   coarse angles.  Carry on while the readings stay on the plane so far. */
static int
s5_face_extend (const s5_profile_t * p_profile, int end, int step,
                double distance, double tilt, int * p_before, bool * p_jump)
{
    int index;

    for (index = end + step; (index >= 0) && (index < S5_FACE_ANGLES);
         index += step)
    {
        double from_square, axis;

        if (!s5_profile_taken(p_profile, index))
        {
            continue;
        }

        if (!s5_profile_echo(p_profile, index))
        {
            return end;
        }

        from_square = fabs((double)(S5_FACE_CENTRE - index) - tilt);
        axis        = s5_profile_mm(p_profile, index) + S5_SENSOR_OFFSET_MM;

        if ((axis
             > S5_FACE_PLANE_OVER * distance / cos(from_square * S5_DEG_TO_RAD))
            || (axis < distance
                           / cos(fmax(0.0, from_square - S5_FACE_PLANE_BEAM_DEG)
                                 * S5_DEG_TO_RAD)))
        {
            return end;
        }

        *p_before = end;
        end       = index;
    }

    *p_jump = false; /* still on the plane at the limit of the sweep */
    return end;
}

/* The corner's range: the last reading, unless it is a long grazing echo */
static double
s5_face_corner_range (const s5_profile_t * p_profile, int end, int before)
{
    return (s5_profile_mm(p_profile, end) - s5_profile_mm(p_profile, before)
            > S5_FACE_TAIL_MM)
               ? s5_profile_mm(p_profile, before)
               : s5_profile_mm(p_profile, end);
}

/* First rise through level going out from start, interpolated, as an index */
static bool
s5_face_cross (const s5_profile_t * p_profile, int start, int end, int step,
               double level, double * p_index)
{
    int previous = start;
    int index;

    for (index = start + step; (step > 0) ? (index <= end) : (index >= end);
         index += step)
    {
        if (!s5_profile_taken(p_profile, index))
        {
            continue;
        }

        if (s5_profile_mm(p_profile, index) >= level)
        {
            *p_index = (double)previous
                       + (double)(index - previous)
                             * (level - s5_profile_mm(p_profile, previous))
                             / (s5_profile_mm(p_profile, index)
                                - s5_profile_mm(p_profile, previous));
            return true;
        }

        previous = index;
    }

    return false;
}

/* At the limit of the sweep: has the range stopped rising? */
static bool
s5_face_flat_end (const s5_profile_t * p_profile, int start, int end, int step)
{
    int index = end - step * S5_FACE_FLAT_DEG;

    if ((index - start) * step < 0)
    {
        index = start;
    }

    while (!s5_profile_taken(p_profile, index))
    {
        index += step;
    }

    return fabs(s5_profile_mm(p_profile, end) - s5_profile_mm(p_profile, index))
           <= S5_FACE_FLAT_MM;
}

void
s5_face_fit (const s5_scan_t * p_scan, s5_face_t * p_out)
{
    s5_profile_t profile;

    unsigned side, i;
    int      index;
    int      nearest = -1;

    int      left, right, before_left, before_right;
    bool     b_jump_left, b_jump_right;
    unsigned count = 1U;

    double range_left, range_right, rise, level, cross_left, cross_right;
    double distance, tilt;

    memset(p_out, 0, sizeof(*p_out));

    if (!p_scan->b_valid)
    {
        return;
    }

    memset(&profile, 0, sizeof(profile));

    for (i = 0U; i < p_scan->coarse_count; i++)
    {
        s5_profile_add(&profile, p_scan->coarse[i]);
    }

    for (side = S5_LEFT; side <= S5_RIGHT; side++)
    {
        for (i = 0U; i < p_scan->fine_outward[side]; i++)
        {
            s5_profile_add(&profile, p_scan->fine[side][i]);
        }
    }

    for (index = S5_FACE_CENTRE - S5_FACE_SEARCH_DEG;
         index <= S5_FACE_CENTRE + S5_FACE_SEARCH_DEG; index++)
    {
        if (s5_profile_taken(&profile, index)
            && s5_profile_echo(&profile, index)
            && ((nearest < 0)
                || (s5_profile_mm(&profile, index)
                    < s5_profile_mm(&profile, nearest))))
        {
            nearest = index;
        }
    }

    if (nearest < 0)
    {
        return;
    }

    left  = s5_face_walk(&profile, nearest, -1, &before_left, &b_jump_left,
                         &count);
    right = s5_face_walk(&profile, nearest, 1, &before_right, &b_jump_right,
                         &count);

    if ((left > S5_FACE_CENTRE) || (right < S5_FACE_CENTRE)
        || (count < S5_FACE_MIN_POINTS))
    {
        return; /* straight ahead is something else */
    }

    range_left  = s5_face_corner_range(&profile, left, before_left);
    range_right = s5_face_corner_range(&profile, right, before_right);
    rise = fmin(range_left, range_right) - s5_profile_mm(&profile, nearest);

    if (rise < S5_FACE_MIN_RISE_MM)
    {
        return;
    }

    level = s5_profile_mm(&profile, nearest) + 0.5 * rise;

    if (!s5_face_cross(&profile, nearest, left, -1, level, &cross_left)
        || !s5_face_cross(&profile, nearest, right, 1, level, &cross_right))
    {
        return;
    }

    distance = s5_profile_mm(&profile, nearest) + S5_SENSOR_OFFSET_MM;
    tilt     = (double)S5_FACE_CENTRE - 0.5 * (cross_left + cross_right);

    if (b_jump_left)
    {
        left = s5_face_extend(&profile, left, -1, distance, tilt, &before_left,
                              &b_jump_left);
    }

    if (b_jump_right)
    {
        right = s5_face_extend(&profile, right, 1, distance, tilt,
                               &before_right, &b_jump_right);
    }

    /* Still on the plane at the limit of the sweep, the last reading is a lower
     * bound */
    range_left
        = (b_jump_left ? s5_face_corner_range(&profile, left, before_left)
                       : s5_profile_mm(&profile, left))
          + S5_SENSOR_OFFSET_MM;
    range_right
        = (b_jump_right ? s5_face_corner_range(&profile, right, before_right)
                        : s5_profile_mm(&profile, right))
          + S5_SENSOR_OFFSET_MM;
    p_out->distance_mm = distance;
    p_out->left_mm
        = sqrt(fmax(0.0, range_left * range_left - distance * distance));
    p_out->right_mm
        = sqrt(fmax(0.0, range_right * range_right - distance * distance));
    p_out->width_mm  = p_out->left_mm + p_out->right_mm;
    p_out->offset_mm = 0.5 * (p_out->left_mm - p_out->right_mm);

    /* Both ends heard: half the beam's width cancels between them */
    if (b_jump_left && b_jump_right)
    {
        double middle = (double)S5_FACE_CENTRE - 0.5 * (double)(left + right);
        double edges  = middle
                        - 0.5
                              * (atan(p_out->left_mm / distance)
                                 - atan(p_out->right_mm / distance))
                              / S5_DEG_TO_RAD;

        if (fabs(edges - tilt) <= S5_FACE_EDGE_AGREE_DEG)
        {
            tilt = 0.5 * (tilt + edges);
        }
    }

    tilt += S5_FACE_TILT_BIAS_DEG;

    if (fabs(tilt) > S5_FACE_MAX_TILT_DEG)
    {
        return;
    }

    p_out->tilt_deg = tilt;
    p_out->b_corner_left
        = b_jump_left || s5_face_flat_end(&profile, nearest, left, -1);
    p_out->b_corner_right
        = b_jump_right || s5_face_flat_end(&profile, nearest, right, 1);
    p_out->end_deg[S5_LEFT]  = (uint8_t)(left + S5_ANGLE_LEFT_DEG);
    p_out->end_deg[S5_RIGHT] = (uint8_t)(right + S5_ANGLE_LEFT_DEG);
    p_out->far_mm
        = fmax(s5_profile_mm(&profile, left), s5_profile_mm(&profile, right));
    p_out->b_valid = true;
}

/* The face knows better than the edge angles */
static void
s5_face_apply (s5_scan_t * p_scan)
{
    const s5_face_t * p_face    = &p_scan->face;
    double            tolerance = S5_FACE_CENTRE_FRACTION * p_face->distance_mm;
    s5_placement_t *  p_placement = &p_scan->placement;

    p_scan->width_mm       = p_face->width_mm;
    p_scan->offset_mm      = p_face->offset_mm;
    p_placement->b_bounded = !p_face->b_corner_left || !p_face->b_corner_right;

    if ((p_face->offset_mm > tolerance) && p_face->b_corner_right)
    {
        p_placement->position = S5_POSITION_LEFT;
    }
    else if ((p_face->offset_mm < -tolerance) && p_face->b_corner_left)
    {
        p_placement->position = S5_POSITION_RIGHT;
    }
    else
    {
        p_placement->position
            = p_placement->b_bounded ? S5_POSITION_UNKNOWN : S5_POSITION_CENTRE;
    }
}

/* The reading straight ahead, then the coarse sweep if it found something */
static bool
s5_scan_coarse (s5_scan_t * p_scan, s5_measure_fn measure, void * p_context)
{
    int           angle;
    s5_estimate_t estimate;

    memset(p_scan, 0, sizeof(*p_scan));

    /* Reference: straight ahead */
    if (!measure(p_context, S5_ANGLE_CENTRE_DEG, &estimate))
    {
        return false;
    }

    p_scan->centre = s5_point_pack(S5_ANGLE_CENTRE_DEG, &estimate,
                                   s5_estimate_usable(&estimate));

    if (!s5_point_object(p_scan->centre))
    {
        return true; /* nothing straight ahead to measure */
    }

    p_scan->centre_forward_mm
        = s5_forward_mm(estimate.range_mm, (double)S5_ANGLE_CENTRE_DEG);

    /* Coarse sweep, left to right */
    for (angle = S5_ANGLE_LEFT_DEG; angle <= S5_ANGLE_RIGHT_DEG;
         angle += S5_COARSE_STEP_DEG)
    {
        if (!s5_measure_point(&p_scan->coarse[p_scan->coarse_count], angle,
                              p_scan->centre_forward_mm, measure, p_context))
        {
            return false;
        }

        p_scan->coarse_count++;
    }

    return true;
}

bool
s5_tilt_scan (s5_scan_t * p_scan, s5_measure_fn measure, void * p_context)
{
    if (!s5_scan_coarse(p_scan, measure, p_context))
    {
        return false;
    }

    p_scan->b_valid = s5_point_object(p_scan->centre);
    s5_face_fit(p_scan, &p_scan->face);
    return true;
}

bool
s5_scan_run (s5_scan_t * p_scan, s5_measure_fn measure, void * p_context)
{
    unsigned side;

    if (!s5_scan_coarse(p_scan, measure, p_context))
    {
        return false;
    }

    if (!s5_point_object(p_scan->centre))
    {
        return true;
    }

    /* Fine scan near each coarse edge */
    for (side = S5_LEFT; side <= S5_RIGHT; side++)
    {
        int    start_deg;
        double start_range_mm;

        s5_coarse_edge(p_scan, side, &start_deg, &start_range_mm);

        if (!s5_find_edge(p_scan, side, start_deg, start_range_mm, measure,
                          p_context))
        {
            return false;
        }
    }

    p_scan->width_mm
        = p_scan->edge[S5_LEFT].lateral_mm - p_scan->edge[S5_RIGHT].lateral_mm;
    p_scan->offset_mm = 0.5
                        * (p_scan->edge[S5_LEFT].lateral_mm
                           + p_scan->edge[S5_RIGHT].lateral_mm);
    p_scan->b_valid   = true;
    s5_placement(p_scan, &p_scan->placement);
    s5_face_fit(p_scan, &p_scan->face);

    if (p_scan->face.b_valid)
    {
        s5_face_apply(p_scan);
    }

    s5_avoidance(p_scan, &p_scan->avoid);
    return true;
}

void
s5_placement (const s5_scan_t * p_scan, s5_placement_t * p_out)
{
    const s5_edge_t * p_left  = &p_scan->edge[S5_LEFT];
    const s5_edge_t * p_right = &p_scan->edge[S5_RIGHT];

    memset(p_out, 0, sizeof(*p_out));
    p_out->position = S5_POSITION_UNKNOWN;

    if (!p_scan->b_valid)
    {
        return;
    }

    p_out->left_deg       = (double)S5_ANGLE_CENTRE_DEG - p_left->angle_deg;
    p_out->right_deg      = p_right->angle_deg - (double)S5_ANGLE_CENTRE_DEG;
    p_out->difference_deg = p_out->left_deg - p_out->right_deg;
    p_out->b_bounded      = !p_left->b_found || !p_right->b_found;

    /* A side whose edge was not found is at least its angle, so it can only
       make the obstacle lean further towards that side, never less. */
    if ((p_out->difference_deg > S5_CENTRE_TOLERANCE_DEG) && p_right->b_found)
    {
        p_out->position = S5_POSITION_LEFT;
    }
    else if ((p_out->difference_deg < -S5_CENTRE_TOLERANCE_DEG)
             && p_left->b_found)
    {
        p_out->position = S5_POSITION_RIGHT;
    }
    else if (!p_out->b_bounded)
    {
        p_out->position = S5_POSITION_CENTRE;
    }
}

/* Farthest range among the obstacle's own points */
static double
s5_object_range (const s5_scan_t * p_scan)
{
    double farthest = s5_point_range(p_scan->centre);

    unsigned side, i;

    for (i = 0U; i < p_scan->coarse_count; i++)
    {
        if (s5_point_object(p_scan->coarse[i]))
        {
            farthest = fmax(farthest, s5_point_range(p_scan->coarse[i]));
        }
    }

    for (side = S5_LEFT; side <= S5_RIGHT; side++)
    {
        for (i = 0U; i < p_scan->fine_count[side]; i++)
        {
            if (s5_point_object(p_scan->fine[side][i]))
            {
                farthest
                    = fmax(farthest, s5_point_range(p_scan->fine[side][i]));
            }
        }
    }

    return farthest;
}

/* Is this point something else beside the obstacle, beyond the given edge? */
static bool
s5_beside (const s5_scan_t * p_scan, unsigned side, s5_point_t point,
           double object_range_mm)
{
    const s5_edge_t * p_edge = &p_scan->edge[side];
    bool b_outside = (side == S5_LEFT)
                         ? (s5_point_angle(point) <= p_edge->first_clear_deg)
                         : (s5_point_angle(point) >= p_edge->first_clear_deg);

    return b_outside && !s5_point_object(point) && s5_point_usable(point)
           && (s5_point_range(point) > object_range_mm + S5_SPILL_RANGE_MM)
           && (s5_forward_mm(s5_point_range(point),
                             (double)s5_point_angle(point))
               <= p_scan->centre_forward_mm + S5_PASS_LOOKAHEAD_MM);
}

/* Nearest echo beside the obstacle on one side, mm from the centre line */
static double
s5_side_clear (const s5_scan_t * p_scan, unsigned side, double object_range_mm)
{
    double   nearest = INFINITY;
    double   sign    = (side == S5_LEFT) ? 1.0 : -1.0;
    unsigned i;

    for (i = 0U; i < p_scan->coarse_count; i++)
    {
        s5_point_t point = p_scan->coarse[i];

        if (s5_beside(p_scan, side, point, object_range_mm))
        {
            nearest = fmin(nearest,
                           sign
                               * s5_lateral_mm(s5_point_range(point),
                                               (double)s5_point_angle(point)));
        }
    }

    for (i = 0U; i < p_scan->fine_count[side]; i++)
    {
        s5_point_t point = p_scan->fine[side][i];

        if (s5_beside(p_scan, side, point, object_range_mm))
        {
            nearest = fmin(nearest,
                           sign
                               * s5_lateral_mm(s5_point_range(point),
                                               (double)s5_point_angle(point)));
        }
    }

    return nearest;
}

/* Nearest echo beyond this end of the face, square to the face from the axle */
static bool
s5_face_echo_real (const s5_scan_t * p_scan, unsigned side, s5_point_t point)
{
    int      sign = (side == S5_LEFT) ? -1 : 1;
    unsigned i;
    unsigned total = (unsigned)p_scan->coarse_count + p_scan->fine_count[side];

    if (sign * (s5_point_angle(point) - p_scan->face.end_deg[side])
        > S5_FACE_EDGE_ECHO_DEG)
    {
        return true;
    }

    for (i = 0U; i < total; i++)
    {
        s5_point_t other = (i < p_scan->coarse_count)
                               ? p_scan->coarse[i]
                               : p_scan->fine[side][i - p_scan->coarse_count];
        int further = sign * (s5_point_angle(other) - s5_point_angle(point));

        if ((further > 0) && (further <= S5_FACE_EDGE_CONFIRM_DEG)
            && s5_point_usable(other)
            && (fabs(s5_point_range(other) - s5_point_range(point))
                <= S5_FACE_JUMP_MM))
        {
            return true;
        }
    }

    return false;
}

static double
s5_face_clear (const s5_scan_t * p_scan, unsigned side, double axle_mm)
{
    const s5_face_t * p_face  = &p_scan->face;
    double            sign    = (side == S5_LEFT) ? 1.0 : -1.0;
    double            nearest = INFINITY;
    unsigned          i;
    unsigned total = (unsigned)p_scan->coarse_count + p_scan->fine_count[side];

    for (i = 0U; i < total; i++)
    {
        s5_point_t point = (i < p_scan->coarse_count)
                               ? p_scan->coarse[i]
                               : p_scan->fine[side][i - p_scan->coarse_count];
        bool b_beyond = (side == S5_LEFT)
                            ? (s5_point_angle(point) < p_face->end_deg[side])
                            : (s5_point_angle(point) > p_face->end_deg[side]);

        if (b_beyond && s5_point_usable(point)
            && (s5_point_range(point) > p_face->far_mm + S5_SPILL_RANGE_MM)
            && s5_face_echo_real(p_scan, side, point))
        {
            double axis = s5_point_range(point) + S5_SENSOR_OFFSET_MM;
            double angle
                = (s5_bearing_rad((double)s5_point_angle(point)) / S5_DEG_TO_RAD
                   - p_face->tilt_deg)
                  * S5_DEG_TO_RAD;
            double along = sign * (axis * sin(angle) - axle_mm);

            if ((axis * cos(angle)
                 <= p_face->distance_mm + S5_PASS_LOOKAHEAD_MM)
                && (along > 0.0))
            {
                nearest = fmin(nearest, along);
            }
        }
    }

    return nearest;
}

double
s5_start_tilt_deg (const s5_scan_t * p_scan)
{
    return (p_scan->face.b_valid ? p_scan->face.tilt_deg : 0.0)
           + p_scan->squared_deg;
}

s5_avoid_side_t
s5_preferred_side (const s5_scan_t * p_scan)
{
    double tilt = s5_start_tilt_deg(p_scan);

    if (p_scan->placement.position == S5_POSITION_LEFT)
    {
        return S5_AVOID_RIGHT;
    }

    if (p_scan->placement.position == S5_POSITION_RIGHT)
    {
        return S5_AVOID_LEFT;
    }

    if (fabs(tilt) >= S5_FACE_TILT_MIN_DEG)
    {
        return (tilt < 0.0) ? S5_AVOID_RIGHT : S5_AVOID_LEFT;
    }

    return S5_DEFAULT_AVOID;
}

void
s5_avoidance (const s5_scan_t * p_scan, s5_avoid_t * p_out)
{
    s5_avoid_side_t order[2];

    unsigned side, i;
    double   clearance = S5_ROBOT_HALF_WIDTH_MM + S5_PASS_MARGIN_MM;

    memset(p_out, 0, sizeof(*p_out));
    p_out->side               = S5_AVOID_NONE;
    p_out->clear_mm[S5_LEFT]  = INFINITY;
    p_out->clear_mm[S5_RIGHT] = INFINITY;

    if (!p_scan->b_valid)
    {
        return;
    }

    if (p_scan->face.b_valid)
    {
        const s5_face_t * p_face = &p_scan->face;
        /* The axle, along the face from the servo.  The robot's tilt puts it
           behind the servo but also to one side: towards one end, away from
           the other. */
        double axle_mm
            = S5_SERVO_AHEAD_MM * sin(p_face->tilt_deg * S5_DEG_TO_RAD);

        p_out->shift_mm[S5_LEFT]
            = fmax(0.0, p_face->left_mm - axle_mm + clearance);
        p_out->shift_mm[S5_RIGHT]
            = fmax(0.0, p_face->right_mm + axle_mm + clearance);

        for (side = S5_LEFT; side <= S5_RIGHT; side++)
        {
            p_out->clear_mm[side] = s5_face_clear(p_scan, side, axle_mm);
            p_out->b_room[side]   = s5_face_corner(p_face, side)
                                    && (p_out->clear_mm[side]
                                        >= p_out->shift_mm[side] + clearance);
        }
    }
    else
    {
        double object_range_mm = s5_object_range(p_scan);

        p_out->shift_mm[S5_LEFT]
            = fmax(0.0, p_scan->edge[S5_LEFT].lateral_mm + clearance);
        p_out->shift_mm[S5_RIGHT]
            = fmax(0.0, -p_scan->edge[S5_RIGHT].lateral_mm + clearance);

        for (side = S5_LEFT; side <= S5_RIGHT; side++)
        {
            p_out->clear_mm[side]
                = s5_side_clear(p_scan, side, object_range_mm);
            /* After the shift, the robot's far side still keeps the margin */
            p_out->b_room[side] = p_scan->edge[side].b_found
                                  && (p_out->clear_mm[side]
                                      >= p_out->shift_mm[side] + clearance);
        }
    }

    order[0] = s5_preferred_side(p_scan);
    order[1] = (order[0] == S5_AVOID_LEFT) ? S5_AVOID_RIGHT : S5_AVOID_LEFT;

    for (i = 0U; i < 2U; i++)
    {
        if (p_out->b_room[(order[i] == S5_AVOID_LEFT) ? S5_LEFT : S5_RIGHT])
        {
            p_out->side = order[i];
            return;
        }
    }
}

void
s5_curve_reset (s5_curve_t * p_curve)
{
    memset(p_curve, 0, sizeof(*p_curve));
}

/* Whole mm modulo 2^16; a path is never negative */
static uint16_t
s5_curve_path (double path_mm)
{
    return (uint16_t)(uint32_t)fmax(0.0, path_mm + 0.5);
}

void
s5_curve_mark (s5_curve_t * p_curve, double path_mm)
{
    p_curve->marked = 1U;
    p_curve->now_mm = s5_curve_path(path_mm);
    p_curve->until_mm
        = (uint16_t)(p_curve->now_mm + (uint16_t)S5_CURVE_HOLD_MM);
}

/* Every call compares against the stored samples, so a pivot on the spot
   (no path at all) still counts.  Samples are only stored per distance, so
   standing still does not push the older ones out. */
void
s5_curve_note (s5_curve_t * p_curve, double path_mm, double heading_deg)
{
    unsigned i;
    unsigned last  = (p_curve->next + S5_CURVE_SAMPLES - 1U) % S5_CURVE_SAMPLES;
    unsigned swing = 0U;
    uint16_t path  = s5_curve_path(path_mm);
    uint16_t heading = (uint16_t)(int32_t)floor(heading_deg * 10.0 + 0.5);

    for (i = 0U; i < p_curve->count; i++)
    {
        if ((uint16_t)(path - p_curve->path_mm[i])
            <= (unsigned)S5_CURVE_WINDOW_MM)
        {
            unsigned left = (uint16_t)(heading - p_curve->heading_ddeg[i]);
            /* turned the other way */
            unsigned change = (left > 0x8000U) ? 0x10000U - left : left;

            swing = (change > swing) ? change : swing;
        }
    }

    p_curve->swing_ddeg = (uint16_t)swing;
    p_curve->now_mm     = path;

    if ((double)swing >= S5_CURVE_MIN_DEG * 10.0)
    {
        s5_curve_mark(p_curve, path_mm);
    }
    else if ((uint16_t)(p_curve->until_mm - path) > (unsigned)S5_CURVE_HOLD_MM)
    {
        /* passed: forget it before until_mm can come round again */
        p_curve->marked = 0U;
    }

    if ((p_curve->count == 0U)
        || ((uint16_t)(path - p_curve->path_mm[last])
            >= (unsigned)S5_CURVE_SPACING_MM))
    {
        p_curve->path_mm[p_curve->next]      = path;
        p_curve->heading_ddeg[p_curve->next] = heading;
        p_curve->next
            = (uint16_t)(((unsigned)p_curve->next + 1U) % S5_CURVE_SAMPLES)
              & 0xfU;

        if (p_curve->count < S5_CURVE_SAMPLES)
        {
            p_curve->count++;
        }
    }
}

bool
s5_curve_recent (const s5_curve_t * p_curve)
{
    return p_curve->marked != 0U;
}

/*** end of file ***/
