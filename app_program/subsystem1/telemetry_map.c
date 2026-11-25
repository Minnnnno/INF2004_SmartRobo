/*
 *  telemetry_map.c
 *  Subsystem 1 -- robot_status_t snapshot -> status frame values.
 *
 *  Slot meanings follow docs/API.md "Snapshot for other buddies", with the
 *  Tribunal 1 packing decisions:
 *    line     values[0] error -1/0/+1, values[1] trackable (> 0.5)
 *    barcode  values[0] letter (ASCII), [1] valid-read counter,
 *             [2] junction counter, [3] failed-read counter (robot.h)
 *    terrain  values[0] turn rate rad/s (+ left), [1] pitch rad, [2] hump m,
 *             [3] validity bits 1..2 (TF_TV_*) | terrain state << 3 |
 *             motion class << 6 | motion valid << 9 (terrain_status.h)
 *    obstacle values[0] distance m, [2] width m, [3] TF_OF_* flags
 */

#include "../../include/float_types.h"
#include <stdint.h>
#include "telemetry_map.h"
#include "terrain_status.h"
#include <math.h>

#define TM_PI 3.14159265358979323846

_Static_assert((SHUTDOWN == TF_ENC_STATE_MAX)
                   && (F_SETUP_TIMEOUT == TF_ENC_FAULT_MAX)
                   && (FB_RESTART_LATCH == TF_EF_CAUSE_MAX),
               "encoder frame ranges must match the motion enums");

static bool     tm_finite(float64_t value);
static int32_t  to_int(float64_t value);
static int32_t  saturate(uint32_t value, uint32_t max);
static int32_t  wrap_deg(float64_t rad, int32_t low);
static int32_t  bc_char_code(float64_t ascii);
static uint32_t bc_count(const subsystem_status_t * p_slot, unsigned slot);
static uint32_t bits_of(const subsystem_status_t * p_slot, unsigned slot);
static bool     line_ok(const robot_status_t * p_status);

void
tm_fill (const robot_status_t * p_status, const tm_frame_info_t * p_info,
         int32_t values[TF_FIELD_COUNT])
{
    const motion_status_t * p_motion = &p_status->motion;
    uint32_t                flags;
    bool                    b_motion_ok;

    values[TF_F_VERSION] = (int32_t)TF_LAYOUT_VERSION;
    values[TF_F_SEQ]     = (int32_t)(p_info->seq & 0xFFU);
    values[TF_F_UPTIME_DS]
        = (int32_t)((uint32_t)to_int(p_info->uptime_s * 10.0) & 0xFFFFU);

    flags                = (p_motion->stationary ? TF_MF_STATIONARY : 0U)
                           | (p_motion->feedback_trusted ? TF_MF_TRUSTED : 0U)
                           | (p_motion->inhibited ? TF_MF_INHIBITED : 0U)
                           | (p_motion->hold ? TF_MF_HOLD : 0U);
    values[TF_F_M_STATE] = (int32_t)p_motion->state;
    values[TF_F_M_MODE]  = (int32_t)p_motion->mode;
    values[TF_F_M_FAULT] = (int32_t)p_motion->fault;
    values[TF_F_M_FLAGS] = (int32_t)flags;
    values[TF_F_OWNER]
        = p_status->owner_valid ? (int32_t)p_status->owner : TF_OWNER_NONE;
    values[TF_F_CMD_ID]    = (int32_t)(p_motion->id & 0xFFU);
    values[TF_F_SPEED_L]   = to_int(p_motion->speed[0] * 1000.0);
    values[TF_F_SPEED_R]   = to_int(p_motion->speed[1] * 1000.0);
    values[TF_F_CENTRE_MM] = to_int(p_motion->centre * 1000.0);
    values[TF_F_HEADING]   = wrap_deg(p_motion->heading, -180);

    values[TF_F_LINE_AVAIL] = p_status->line.available ? 1 : 0;
    values[TF_F_LINE_ERR]
        = p_status->line.available ? to_int(p_status->line.values[0]) : 0;
    values[TF_F_LINE_TRACK]
        = ((p_status->line.available) && (p_status->line.values[1] > 0.5)) ? 1
                                                                           : 0;
    values[TF_F_BC_CHAR] = p_status->barcode.available
                               ? bc_char_code(p_status->barcode.values[0])
                               : 0;
    values[TF_F_BC_READ] = (int32_t)(bc_count(&p_status->barcode, 1U) & 1U);

    /* Buddy 4 slot word: bits 0..2 validity, 3..5 T_STATE, 6..8 motion
       class, bit 9 class + turn rate valid (include/terrain_status.h).
       Bit 0 is not used by Buddy 4; the frame's TF_TV_MOTION comes from
       bit 9 so a stray bit 0 can never mark a stale class as valid. */
    flags                = bits_of(&p_status->terrain, 3U);
    b_motion_ok          = 0U != (flags & TERRAIN_MOTION_VALID);
    values[TF_F_T_VALID] = (int32_t)((flags & (TF_TV_PITCH | TF_TV_HUMP))
                                     | (b_motion_ok ? TF_TV_MOTION : 0U));
    values[TF_F_T_STATE] = (int32_t)((flags >> 3) & 0x7U);
    values[TF_F_M_CLASS]
        = b_motion_ok ? (int32_t)((flags >> TERRAIN_MC_SHIFT) & 0x7U) : 0;
    values[TF_F_PITCH]
        = (0U != (flags & TF_TV_PITCH))
              ? to_int((p_status->terrain.values[1] * 180.0) / TM_PI)
              : 0;
    values[TF_F_TURN_RATE]
        = b_motion_ok ? to_int((p_status->terrain.values[0] * 180.0) / TM_PI)
                      : 0;
    values[TF_F_HUMP_MM] = (0U != (flags & TF_TV_HUMP))
                               ? to_int(p_status->terrain.values[2] * 1000.0)
                               : 0;

    values[TF_F_O_FLAGS] = (int32_t)(bits_of(&p_status->obstacle, 3U) & 0x1FU);
    values[TF_F_O_AVAIL] = p_status->obstacle.available ? 1 : 0;
    values[TF_F_O_DIST_CM]  = p_status->obstacle.available
                                  ? to_int(p_status->obstacle.values[0] * 100.0)
                                  : 0;
    values[TF_F_O_WIDTH_CM] = p_status->obstacle.available
                                  ? to_int(p_status->obstacle.values[2] * 100.0)
                                  : 0;

    values[TF_F_REJECTED] = saturate(p_info->rejected, 7U);
    values[TF_F_DROPPED]  = saturate(p_info->dropped, 7U);
}

void
tm_fill_encoder (const robot_status_t * p_status, uint32_t seq, float64_t now_s,
                 tf_encoder_t * p_encoder)
{
    const motion_status_t * p_motion = &p_status->motion;
    float64_t               time_ms  = p_motion->encoder_sample_time;
    float64_t               age;

    p_encoder->seq        = (uint8_t)(seq & 0xFFU);
    p_encoder->reliable   = p_motion->encoder.reliable;
    p_encoder->trusted    = p_motion->feedback_trusted;
    p_encoder->cause      = (uint8_t)p_motion->feedback_cause;
    p_encoder->state      = (uint8_t)p_motion->state;
    p_encoder->fault      = (uint8_t)p_motion->fault;
    p_encoder->count[0]   = p_motion->encoder.count[0];
    p_encoder->count[1]   = p_motion->encoder.count[1];
    p_encoder->invalid[0] = p_motion->encoder.invalid[0];
    p_encoder->invalid[1] = p_motion->encoder.invalid[1];

    p_encoder->sample_ms = 0U;
    p_encoder->age_ms    = (uint16_t)TF_ENC_AGE_UNKNOWN;
    p_encoder->fresh     = false;

    if ((!tm_finite(time_ms)) || (time_ms < 0.0) || (!tm_finite(now_s)))
    {
        /* Nothing to do. */
    }
    else
    {
        p_encoder->sample_ms = (uint32_t)fmod(time_ms * 1000.0, 4294967296.0);
        age                  = (now_s - time_ms) * 1000.0;

        if ((age < 0.0) || (age >= (float64_t)TF_ENC_AGE_UNKNOWN))
        {
            /* sample from the future, or very old */
        }
        else
        {
            /* nearest ms; 65535 still means old */
            p_encoder->age_ms = (uint16_t)to_int(age);
            p_encoder->fresh  = (p_encoder->age_ms <= TF_ENC_FRESH_MS);
        }
    }
}

uint8_t
tm_edge_events (const robot_status_t * p_prev, const robot_status_t * p_cur)
{
    uint32_t events = 0U;

    if ((F_NONE != p_cur->motion.fault) && (F_NONE == p_prev->motion.fault))
    {
        events |= TF_EV_MOTION_FAULT;
    }

    if ((line_ok(p_prev)) && (!line_ok(p_cur)))
    {
        events |= TF_EV_LINE_LOST;
    }

    /* A new decode: B3's valid-read counter changed.  Every read counts,
       including the same letter twice in a row. */
    if ((p_cur->barcode.available)
        && (bc_count(&p_cur->barcode, 1U) != bc_count(&p_prev->barcode, 1U))
        && (0U != bc_count(&p_cur->barcode, 1U)))
    {
        events |= TF_EV_BARCODE;
    }

    return (uint8_t)events;
}

/* TRUE for a finite double.  NaN fails both comparisons.  (Avoids the
   isfinite macro, which warns under -Wconversion on some host libcs.) */
static bool
tm_finite (float64_t value)
{
    return (value > -1.0e300) && (value < 1.0e300);
}

/* Round half away from zero; non-finite -> 0.  The double is limited first
   so the conversion is always defined; tf_pack() does the real clamp. */
static int32_t
to_int (float64_t value)
{
    int32_t result = 0;

    if (!tm_finite(value))
    {
        result = 0;
    }
    else
    {
        if (value > 1.0e6)
        {
            value = 1.0e6;
        }
        else if (value < -1.0e6)
        {
            value = -1.0e6;
        }
        else
        {
            /* Nothing to do in any other case. */
        }

        result = (int32_t)lround(value);
    }

    return result;
}

static int32_t
saturate (uint32_t value, uint32_t max)
{
    return (int32_t)((value > max) ? max : value);
}

/* Degrees wrapped into [lo, lo + 360). */
static int32_t
wrap_deg (float64_t rad, int32_t low)
{
    int32_t degrees;
    int32_t result = 0;

    if (!tm_finite(rad))
    {
        result = low;
    }
    else
    {
        degrees = to_int(fmod((rad * 180.0) / TM_PI, 360.0));

        while (degrees < low)
        {
            degrees += 360;
        }

        while (degrees >= (low + 360))
        {
            degrees -= 360;
        }

        result = degrees;
    }

    return result;
}

/* B3 barcode slot letter (ASCII code) -> BC_CHAR: 0 none or not a Code 39
   character, n = TF_BC_CHARS[n - 1]. */
static int32_t
bc_char_code (float64_t ascii)
{
    static const char chars[] = TF_BC_CHARS;
    int32_t           code    = to_int(ascii);
    unsigned          index;

    if ((code <= 0) || (code > 127))
    {
        return 0;
    }

    for (index = 0U; index < (sizeof(chars) - 1U); index++)
    {
        if ((int32_t)(uint8_t)chars[index] == code)
        {
            return (int32_t)index + 1;
        }
    }

    return 0;
}

/* B3 barcode slot counter (values[1] valid reads, [3] failed reads); 0 when
   the slot is unavailable.  Only differences and the low bit are used, so
   wrapping is harmless. */
static uint32_t
bc_count (const subsystem_status_t * p_slot, unsigned slot)
{
    int32_t value = to_int(p_slot->values[slot]);

    return ((p_slot->available) && (value > 0)) ? (uint32_t)value : 0U;
}

static uint32_t
bits_of (const subsystem_status_t * p_slot, unsigned slot)
{
    int32_t value = to_int(p_slot->values[slot]);

    return ((p_slot->available) && (value > 0)) ? (uint32_t)value : 0U;
}

static bool
line_ok (const robot_status_t * p_status)
{
    return (p_status->line.available) && (p_status->line.values[1] > 0.5);
}

/*** end of file ***/
