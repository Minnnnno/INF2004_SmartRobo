/** @file lift.c
 *
 * @brief Buddy 4 lift / overturn detection; see lift.h for the rules.
 */

#include "lift.h"
#include <math.h>
#include <stddef.h>

#define LIFT_STATE_MASK (0x03U)
#define LIFT_F_OVER     (0x04U)
#define LIFT_F_CLEAR    (0x10U)
#define LIFT_F_SEEN     (0x20U)
#define LIFT_F_TIMERS   (LIFT_F_OVER | LIFT_F_CLEAR)

static int16_t lift_clamp16(float32_t value);
static void    lift_set_state(lift_t * p_lift, uint8_t condition);
static bool    lift_timer(lift_t * p_lift, uint8_t flag, uint32_t * p_since,
                          bool b_condition, uint32_t now_ms, uint32_t hold_ms);
static float32_t lift_upright(float32_t up_mg, float32_t magnitude,
                              bool * p_oriented);
static uint8_t lift_decide(lift_t * p_lift, uint8_t condition, bool b_oriented,
                           float32_t still, uint32_t now_ms);

/*! @brief Start again with nothing latched. */
void
lift_reset (lift_t * p_lift)
{
    if (NULL != p_lift)
    {
        *p_lift             = (lift_t){0};
        p_lift->upright_pct = 100;
    }
}

uint8_t
lift_step (lift_t * p_lift, tilt_context_t const * p_tilt, int16_t axis_x,
           int16_t axis_y, int16_t axis_z, uint32_t now_ms, bool b_armed)
{
    float32_t accel[3]
        = {(float32_t)axis_x, (float32_t)axis_y, (float32_t)axis_z};
    float32_t magnitude  = 0.0F;
    float32_t up_mg      = 0.0F;
    float32_t upright    = 1.0F;
    bool      b_oriented = false;
    uint8_t   condition  = LIFT_NONE;

    if (NULL == p_lift)
    {
        return LIFT_NONE;
    }

    condition = (uint8_t)(p_lift->flags & LIFT_STATE_MASK);

    if ((NULL == p_tilt) || (0U == (p_tilt->progress & TILT_CALIBRATED))
        || (!(p_tilt->gravity > 0.0F)))
    {
        return condition; /* No reference yet: nothing to compare against. */
    }

    if ((0U != (p_lift->flags & LIFT_F_SEEN))
        && ((now_ms - p_lift->last_ms) > LIFT_MAX_GAP_MS))
    {
        /* Stale history: restart the timers. */
        p_lift->flags &= (uint8_t)~LIFT_F_TIMERS;
        p_lift->vertical_count = 0U;
    }

    p_lift->flags |= LIFT_F_SEEN;
    p_lift->last_ms = now_ms;

    magnitude  = sqrtf((accel[0] * accel[0]) + (accel[1] * accel[1])
                       + (accel[2] * accel[2]));
    up_mg      = (accel[0] * p_tilt->up[0]) + (accel[1] * p_tilt->up[1])
                 + (accel[2] * p_tilt->up[2]);
    b_oriented = (isfinite(magnitude))
                 && (magnitude >= (float32_t)LIFT_MIN_MAGNITUDE_MG);

    upright             = lift_upright(up_mg, magnitude, &b_oriented);
    p_lift->upright_pct = (int8_t)lroundf(upright * 100.0F);

    /* Steep tilt only pauses hump measurement (until the tilt filter has
     * settled after it); it never latches a stop. */
    if (b_oriented && (p_lift->upright_pct < LIFT_SUSPECT_UPRIGHT_PCT))
    {
        p_lift->steep_hold = (uint8_t)LIFT_SUSPECT_SAMPLES;
    }
    else if (p_lift->steep_hold > 0U)
    {
        p_lift->steep_hold--;
    }
    else
    {
        /* Nothing to do in any other case. */
    }

    p_lift->vertical_mg = lift_clamp16(up_mg - p_tilt->gravity);

    if (p_lift->vertical_mg >= LIFT_VERTICAL_MG)
    {
        if (p_lift->vertical_count < 255U)
        {
            p_lift->vertical_count++;
        }
    }
    else
    {
        p_lift->vertical_count = 0U;
    }

    if (!b_armed)
    {
        /* Nothing seen while settling (placing the robot) counts later. */
        p_lift->flags &= (uint8_t)~LIFT_F_TIMERS;
        p_lift->vertical_count = 0U;
        return condition;
    }

    condition = lift_decide(p_lift, condition, b_oriented,
                            magnitude - p_tilt->gravity, now_ms);

    lift_set_state(p_lift, condition);
    return condition;
}

/*! @brief Latched condition. */
uint8_t
lift_state (lift_t const * p_lift)
{
    return (NULL == p_lift) ? LIFT_NONE
                            : (uint8_t)(p_lift->flags & LIFT_STATE_MASK);
}

bool
lift_suspect (lift_t const * p_lift)
{
    return (NULL != p_lift)
           && ((LIFT_NONE != (p_lift->flags & LIFT_STATE_MASK))
               || (0U != (p_lift->flags & LIFT_F_OVER))
               || (0U != p_lift->steep_hold));
}

/*! @brief Vertical acceleration beyond gravity, mg. */
int16_t
lift_vertical_mg (lift_t const * p_lift)
{
    return (NULL == p_lift) ? 0 : p_lift->vertical_mg;
}

/*! @brief Vertical acceleration that latched the condition, mg. */
int16_t
lift_trigger_mg (lift_t const * p_lift)
{
    return (NULL == p_lift) ? 0 : p_lift->trigger_mg;
}

/*! @brief How upright, percent. */
int8_t
lift_upright_pct (lift_t const * p_lift)
{
    return (NULL == p_lift) ? 100 : p_lift->upright_pct;
}

/*! @brief Name of a condition. */
char const *
lift_name (uint8_t condition)
{
    const char * p_result = NULL;

    switch (condition & LIFT_STATE_MASK)
    {
        case LIFT_LIFTED:
            p_result = "LIFTED";
        break;

        case LIFT_OVERTURNED:
            p_result = "OVERTURNED";
        break;

        case LIFT_NONE:
            p_result = "NONE";
        break;

        default:
            p_result = "INVALID";
        break;
    }

    return p_result;
}

/*! @brief Limit to 16 bits. */
static int16_t
lift_clamp16 (float32_t value)
{
    int16_t result = 0;

    if (!(value > -32768.0F))
    {
        result = (value < 0.0F) ? (int16_t)-32768 : 0;
    }
    else if (value > 32767.0F)
    {
        result = 32767;
    }
    else
    {
        result = (int16_t)lroundf(value);
    }

    return result;
}

/*! @brief Store the condition in the flags. */
static void
lift_set_state (lift_t * p_lift, uint8_t condition)
{
    p_lift->flags = (uint8_t)((p_lift->flags & (uint8_t)~LIFT_STATE_MASK)
                              | (condition & LIFT_STATE_MASK));
}

/*! @brief Start or keep a timer while the condition holds; true once it held
 * hold_ms.
 */
static bool
lift_timer (lift_t * p_lift, uint8_t flag, uint32_t * p_since, bool b_condition,
            uint32_t now_ms, uint32_t hold_ms)
{
    bool b_result = false;

    if (!b_condition)
    {
        p_lift->flags &= (uint8_t)~flag;
        b_result = false;
    }
    else
    {
        if (0U == (p_lift->flags & flag))
        {
            p_lift->flags |= flag;
            *p_since = now_ms;
        }

        b_result = (now_ms - *p_since) >= hold_ms;
    }

    return b_result;
}

/*! @brief How upright the robot is, 1 to -1; false in *p_oriented when the
 * reading says nothing. */
static float32_t
lift_upright (float32_t up_mg, float32_t magnitude, bool * p_oriented)
{
    float32_t upright = 1.0F;

    if (*p_oriented)
    {
        upright = up_mg / magnitude;

        if (!isfinite(upright))
        {
            upright     = 1.0F;
            *p_oriented = false;
        }

        if (upright > 1.0F)
        {
            upright = 1.0F;
        }

        if (upright < -1.0F)
        {
            upright = -1.0F;
        }
    }

    return upright;
}

/*! @brief The next condition from this sample. */
static uint8_t
lift_decide (lift_t * p_lift, uint8_t condition, bool b_oriented,
             float32_t still, uint32_t now_ms)
{
    bool b_over = lift_timer(
        p_lift, LIFT_F_OVER, &p_lift->over_since,
        b_oriented && (p_lift->upright_pct < LIFT_OVERTURN_UPRIGHT_PCT), now_ms,
        LIFT_HOLD_MS);
    bool b_raised = (p_lift->vertical_count >= LIFT_VERTICAL_SAMPLES);

    if (LIFT_NONE == condition)
    {
        if (b_over || b_raised)
        {
            condition          = b_over ? LIFT_OVERTURNED : LIFT_LIFTED;
            p_lift->trigger_mg = p_lift->vertical_mg;
            p_lift->flags &= (uint8_t)~LIFT_F_CLEAR;
        }
    }
    else
    {
        bool b_calm = b_oriented
                      && (p_lift->upright_pct >= LIFT_CLEAR_UPRIGHT_PCT)
                      && (still < (float32_t)LIFT_CLEAR_STILL_MG)
                      && (still > -(float32_t)LIFT_CLEAR_STILL_MG);

        if (b_over)
        {
            /* A lifted robot that then tips over. */
            condition = LIFT_OVERTURNED;
        }

        if (lift_timer(p_lift, LIFT_F_CLEAR, &p_lift->clear_since, b_calm,
                       now_ms, LIFT_CLEAR_MS))
        {
            condition = LIFT_NONE;
            p_lift->flags &= (uint8_t)~LIFT_F_TIMERS;
            p_lift->vertical_count = 0U;
        }
    }

    return condition;
}

/*** end of file ***/
