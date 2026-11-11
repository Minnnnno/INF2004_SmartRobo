/* Buddy 4 lift / overturn detection; see lift.h for the rules. */
#include "lift.h"
#include <math.h>
#include <stddef.h>

#define LIFT_STATE_MASK  (0x03U)
#define LIFT_F_OVER      (0x04U)
#define LIFT_F_TILT      (0x08U)
#define LIFT_F_CLEAR     (0x10U)
#define LIFT_F_SEEN      (0x20U)
#define LIFT_F_TIMERS    (LIFT_F_OVER | LIFT_F_TILT | LIFT_F_CLEAR)

void lift_reset(lift_t *p_lift)
{
    if (NULL != p_lift)
    {
        *p_lift = (lift_t){0};
        p_lift->upright_pct = 100;
    }
}

static int16_t lift_clamp16(float32_t value)
{
    if (!(value > -32768.0F)) { return (value < 0.0F) ? (int16_t)-32768 : 0; } /* NaN -> 0 */
    if (value > 32767.0F) { return 32767; }
    return (int16_t)lroundf(value);
}

static void lift_set_state(lift_t *p_lift, uint8_t condition)
{
    p_lift->flags = (uint8_t)((p_lift->flags & (uint8_t)~LIFT_STATE_MASK) |
                              (condition & LIFT_STATE_MASK));
}

/* Start or keep a timer while the condition holds; true once it held hold_ms. */
static bool lift_timer(lift_t *p_lift, uint8_t flag, uint32_t *p_since, bool condition,
                       uint32_t now_ms, uint32_t hold_ms)
{
    if (!condition)
    {
        p_lift->flags &= (uint8_t)~flag;
        return false;
    }
    if (0U == (p_lift->flags & flag))
    {
        p_lift->flags |= flag;
        *p_since = now_ms;
    }
    return (now_ms - *p_since) >= hold_ms;
}

uint8_t lift_step(lift_t *p_lift, tilt_context_t const *p_tilt,
                  int16_t x, int16_t y, int16_t z, uint32_t now_ms, bool armed)
{
    float32_t a[3] = {(float32_t)x, (float32_t)y, (float32_t)z};
    float32_t magnitude = 0.0F;
    float32_t up_mg = 0.0F;
    float32_t upright = 1.0F;
    bool oriented = false;
    uint8_t condition = LIFT_NONE;

    if (NULL == p_lift)
    {
        return LIFT_NONE;
    }
    condition = (uint8_t)(p_lift->flags & LIFT_STATE_MASK);
    if ((NULL == p_tilt) || (0U == (p_tilt->progress & TILT_CALIBRATED)) ||
        !(p_tilt->gravity > 0.0F))
    {
        return condition; /* No reference yet: nothing to compare against. */
    }
    if ((0U != (p_lift->flags & LIFT_F_SEEN)) && ((now_ms - p_lift->last_ms) > LIFT_MAX_GAP_MS))
    {
        p_lift->flags &= (uint8_t)~LIFT_F_TIMERS; /* Stale history: restart the timers. */
        p_lift->vertical_count = 0U;
    }
    p_lift->flags |= LIFT_F_SEEN;
    p_lift->last_ms = now_ms;

    magnitude = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    up_mg = a[0]*p_tilt->up[0] + a[1]*p_tilt->up[1] + a[2]*p_tilt->up[2];
    oriented = isfinite(magnitude) && (magnitude >= (float32_t)LIFT_MIN_MAGNITUDE_MG);
    if (oriented)
    {
        upright = up_mg / magnitude;
        if (!isfinite(upright)) { upright = 1.0F; oriented = false; }
        if (upright > 1.0F) { upright = 1.0F; }
        if (upright < -1.0F) { upright = -1.0F; }
    }
    p_lift->upright_pct = (int8_t)lroundf(upright * 100.0F);
    p_lift->vertical_mg = lift_clamp16(up_mg - p_tilt->gravity);
    if (p_lift->vertical_mg >= LIFT_VERTICAL_MG)
    {
        if (p_lift->vertical_count < 255U) { p_lift->vertical_count++; }
    }
    else
    {
        p_lift->vertical_count = 0U;
    }

    if (!armed)
    {
        /* Nothing seen while settling (placing the robot) counts later. */
        p_lift->flags &= (uint8_t)~LIFT_F_TIMERS;
        p_lift->vertical_count = 0U;
        return condition;
    }

    {
        bool over = lift_timer(p_lift, LIFT_F_OVER, &p_lift->over_since,
                               oriented && (p_lift->upright_pct < LIFT_OVERTURN_UPRIGHT_PCT),
                               now_ms, LIFT_HOLD_MS);
        bool tilted = lift_timer(p_lift, LIFT_F_TILT, &p_lift->tilt_since,
                                 oriented && (p_lift->upright_pct < LIFT_TILT_UPRIGHT_PCT),
                                 now_ms, LIFT_HOLD_MS);
        bool raised = (p_lift->vertical_count >= LIFT_VERTICAL_SAMPLES);

        if (LIFT_NONE == condition)
        {
            if (over || tilted || raised)
            {
                condition = over ? LIFT_OVERTURNED : LIFT_LIFTED;
                p_lift->trigger_mg = p_lift->vertical_mg;
                p_lift->flags &= (uint8_t)~LIFT_F_CLEAR;
            }
        }
        else
        {
            float32_t still = magnitude - p_tilt->gravity;
            bool calm = oriented && (p_lift->upright_pct >= LIFT_CLEAR_UPRIGHT_PCT) &&
                        (still < (float32_t)LIFT_CLEAR_STILL_MG) &&
                        (still > -(float32_t)LIFT_CLEAR_STILL_MG);

            if (over)
            {
                condition = LIFT_OVERTURNED; /* A lifted robot that then tips over. */
            }
            if (lift_timer(p_lift, LIFT_F_CLEAR, &p_lift->clear_since, calm, now_ms, LIFT_CLEAR_MS))
            {
                condition = LIFT_NONE;
                p_lift->flags &= (uint8_t)~LIFT_F_TIMERS;
                p_lift->vertical_count = 0U;
            }
        }
    }
    lift_set_state(p_lift, condition);
    return condition;
}

uint8_t lift_state(lift_t const *p_lift)
{
    return (NULL == p_lift) ? LIFT_NONE : (uint8_t)(p_lift->flags & LIFT_STATE_MASK);
}

bool lift_suspect(lift_t const *p_lift)
{
    return (NULL != p_lift) &&
           ((LIFT_NONE != (p_lift->flags & LIFT_STATE_MASK)) ||
            (0U != (p_lift->flags & (LIFT_F_OVER | LIFT_F_TILT))));
}

int16_t lift_vertical_mg(lift_t const *p_lift)
{
    return (NULL == p_lift) ? 0 : p_lift->vertical_mg;
}

int16_t lift_trigger_mg(lift_t const *p_lift)
{
    return (NULL == p_lift) ? 0 : p_lift->trigger_mg;
}

int8_t lift_upright_pct(lift_t const *p_lift)
{
    return (NULL == p_lift) ? 100 : p_lift->upright_pct;
}

char const *lift_name(uint8_t condition)
{
    switch (condition & LIFT_STATE_MASK)
    {
        case LIFT_LIFTED:
            return "LIFTED";
        case LIFT_OVERTURNED:
            return "OVERTURNED";
        case LIFT_NONE:
            return "NONE";
        default:
            return "INVALID";
    }
}
