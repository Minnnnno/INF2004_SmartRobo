/* Gravity-based bench tilt, no gyro fusion or hump state machine. */
#include "tilt.h"
#include <math.h>
#include <stddef.h>

#define TILT_FILTER_READY (0x80U)

#define TILT_PITCH_SIGN (-1.0F)
#define TILT_ROLL_SIGN  (-1.0F)
#define TILT_RAD_TO_DEG (57.2957795F)
#define TILT_ENTER_TENTHS (50)   // 5.0°: start calling it tilted
#define TILT_EXIT_TENTHS (30)   // 3.0°: go back to level below this

/* Dot product in the sensor coordinate system. Inputs are private 3-vectors. */
//The dot product converts gravity from sensor axes into robot axes using the reference saved at startup.
static float32_t tilt_dot(float32_t const a[3], float32_t const b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/* Normalise a private vector, rejecting zero/nonfinite magnitudes. */
//normalize needed because if If the axis were, say, length 1000, every projection 
//would come out 1000 times too big, and pitch and roll would be wrong. 
static bool tilt_normalise(float32_t vector[3])
{
    //working out of the length//
    //sqrt of (x² + y² + z)
    float32_t length = sqrtf(tilt_dot(vector, vector));
    //if legnth is infiity , or NAN , return false, dont normalize
    if (!isfinite(length) || (length < 0.01F))
    {
        return false;
    }
    //else divide each part by length
    //shrinks/streches the vector without turning it
    for (uint8_t i = 0U; i < 3U; i++)
    {
        vector[i] /= length;
    }
    return true;
}

//helper function
/* Basic plausibility check, not proof of stationary motion or trustworthy tilt.
 * Reject near-freefall and large acceleration. Constant acceleration can pass.
 */
//(Flitering)
static bool tilt_plausible(float32_t const acceleration[3])
{
    //total acceleration squared.
    float32_t square = tilt_dot(acceleration, acceleration);
    //returns true only if its a real number,  total at least 700mg and at most 1300mg
    return isfinite(square) && (square >= 700.0F*700.0F) && (square <= 1300.0F*1300.0F);
}

/* Clear transient history without redefining level after a data gap. */
void tilt_discard_history(tilt_context_t *p_context)
{
    if (NULL != p_context)
    {
        p_context->progress &= TILT_CALIBRATED;
        p_context->conditions = 0U;
    }
}

/* Average a stable one-second window (50 fresh samples at nominal 20 ms).
 * Anchor comparison limits gradual movement as well as one-sample spikes.
 * A changed sample starts a new window. Keep the reference only in RAM.
 */
//for capturing base reference
bool tilt_capture(tilt_context_t *p_context, int16_t x, int16_t y, int16_t z)
{
    float32_t a[3] = {(float32_t)x, (float32_t)y, (float32_t)z};//convert int to float
    uint8_t count = 0U;
    if (NULL == p_context) { return false; } 
    if (0U != (p_context->progress & TILT_CALIBRATED)) { return true; }//if already calibrated, set to true
    if (!tilt_plausible(a)) //if robot <700 or >1300 mg (robot being moved), restart the whole capture and return false
    {
        tilt_discard_history(p_context);
        return false;
    }
    //anchor = first sample of current window.
    count = p_context->progress & TILT_COUNT_MASK;
    for (uint8_t i = 0U; i < 3U; i++)
    {
        //if any axis of the reading >80mg away from anchor , count goes back to 0
        //robot has moved
        if (fabsf(a[i] - p_context->anchor[i]) > 80.0F) { count = 0U; }
    }
    if (0U == count)
    {
        for (uint8_t i = 0U; i < 3U; i++)
        {
            p_context->sum[i] = 0.0F;
            p_context->anchor[i] = a[i];
        }
    }
    //build up running total for each axis 
    for (uint8_t i = 0U; i < 3U; i++) { p_context->sum[i] += a[i]; }
    count++;
    p_context->progress = count;
    if (count < TILT_CAPTURE_SAMPLES) { return false; }

    //divide for average reading. (sampling)
    for (uint8_t i = 0U; i < 3U; i++)
    {
        p_context->up[i] = p_context->sum[i] / (float32_t)TILT_CAPTURE_SAMPLES;
    }
    //normalize it to scale it to length 1, so only direction of gravity is kept
    if (!tilt_normalise(p_context->up))
    {
        tilt_discard_history(p_context);
        return false;
    }
    //building of the front and side direction.
    for (uint8_t i = 0U; i < 3U; i++)
    {
        /* Start from sensor +Y, remove its component along up, leaving a level
        * forward direction even if the sensor is mounted slightly tilted.
        */
        p_context->forward[i] = ((1U == i) ? 1.0F : 0.0F)
                                - p_context->up[1]*p_context->up[i];
    }
    //normalize vector and scale to 1 , if almos nothing left, calibration fail and start again
    if (!tilt_normalise(p_context->forward))
    {
        tilt_discard_history(p_context);
        return false;
    }
    /* side = forward cross up, giving approximately -X for this mounting. */
    p_context->side[0] = p_context->forward[1]*p_context->up[2] - p_context->forward[2]*p_context->up[1];
    p_context->side[1] = p_context->forward[2]*p_context->up[0] - p_context->forward[0]*p_context->up[2];
    p_context->side[2] = p_context->forward[0]*p_context->up[1] - p_context->forward[1]*p_context->up[0];
    p_context->progress = TILT_CALIBRATED;
    return true;
}

/* Schmitt-trigger classification: enter tilt at +/-5 deg, leave below 3 deg.
 * A direct strong opposite tilt switches direction. Conditions are independent.
 */
static uint8_t tilt_classify(int16_t angle, uint8_t previous)
{
    if (angle >= TILT_ENTER_TENTHS) { return 1U; }
    if (angle <= -TILT_ENTER_TENTHS) { return 2U; }
    if ((1U == previous) && (angle >= TILT_EXIT_TENTHS)) { return 1U; }
    if ((2U == previous) && (angle <= -TILT_EXIT_TENTHS)) { return 2U; }
    return 0U;
}

/* Project smoothed gravity onto the reference axes, then compute pitch/roll.
 * Filter: new = old + 0.25*(sample-old). Seed from the first valid sample;
 * do not ramp up from zero. Refuse inverted/near-vertical poses to avoid
 * ambiguous roll at pitch poles. Out-of-envelope values are unavailable.
 */
uint32_t tilt_update(tilt_context_t *p_context, int16_t x, int16_t y, int16_t z)
{
    float32_t a[3] = {(float32_t)x, (float32_t)y, (float32_t)z};
    float32_t forward, side, up, pitch, roll;
    int16_t pitch_tenths, roll_tenths;
    uint8_t pitch_condition, roll_condition;
    uint32_t result;

    //checks 
    //if total accleration isnt 700-1300 mg ,forget the fliter history and return 0
    if ((NULL == p_context) || (0U == (p_context->progress & TILT_CALIBRATED))) { return 0U; }
    if (!tilt_plausible(a)) { tilt_discard_history(p_context); return 0U; }
    
    //smoothing the reading (low-pass fliter)
    for (uint8_t i = 0U; i < 3U; i++)
    {   
        // if fliter hasnt started yet, copy the reading straight in as its starting value
        if (0U == (p_context->progress & TILT_FILTER_READY)) { p_context->filtered[i] = a[i]; }
        //Once the filter has started, each new reading moves the smoothed value 25% of the way toward that reading.
        else { p_context->filtered[i] += 0.25F*(a[i] - p_context->filtered[i]); }
    }
    p_context->progress |= TILT_FILTER_READY;

    forward = tilt_dot(p_context->filtered, p_context->forward);
    side = tilt_dot(p_context->filtered, p_context->side);
    up = tilt_dot(p_context->filtered, p_context->up);
    //working out of the angles
    //pitch angle of gravity towards the front, roll= angle of gravity towards the side
    pitch = TILT_PITCH_SIGN*atan2f(forward, sqrtf(side*side + up*up))*TILT_RAD_TO_DEG;
    roll = TILT_ROLL_SIGN*atan2f(side, up)*TILT_RAD_TO_DEG;  //turns 2 lengths into an angle

    //reject unusable poses
    // if Nan/infity , if up<= 0  robot is on its side or upside down
    // or pitch/roll +- 75 degrees 
    if (!isfinite(pitch) || !isfinite(roll) || (up <= 0.0F)
        || (fabsf(pitch) > 75.0F) || (fabsf(roll) > 75.0F))
    {
        tilt_discard_history(p_context);
        return 0U;
    }
    // round to tenths and label
    pitch_tenths = (int16_t)roundf(pitch*10.0F);
    roll_tenths = (int16_t)roundf(roll*10.0F);
    //classigy condition
    pitch_condition = tilt_classify(pitch_tenths, p_context->conditions & 3U);
    roll_condition = tilt_classify(roll_tenths, (p_context->conditions >> 2U) & 3U);
    //Pack everything into one number
    p_context->conditions = (uint8_t)(pitch_condition | (roll_condition << 2U));
    result = (uint32_t)((int32_t)pitch_tenths + 900)
           | ((uint32_t)((int32_t)roll_tenths + 900) << 11U)
           | ((uint32_t)pitch_condition << 22U)
           | ((uint32_t)roll_condition << 24U) | TILT_RESULT_VALID;
    return result;
}

/* Height of the right wheel contact above the left one, in 0.1 mm, for a
 * two-wheel axle of the given track. Uses the pure sideways component of the
 * smoothed gravity unit vector, so body pitch does not change it (roll from
 * atan2(side, up) would). Positive = right wheel higher ("LEFT SIDE LOWER").
 * Zero unless calibrated with a ready filter; call after a valid tilt_update().
 */
int16_t tilt_side_height_tenths(tilt_context_t const *p_context, uint16_t track_mm)
{
    float32_t magnitude;
    float32_t height;

    if ((NULL == p_context) ||
        ((TILT_CALIBRATED | TILT_FILTER_READY) !=
         (p_context->progress & (TILT_CALIBRATED | TILT_FILTER_READY))))
    {
        return 0;
    }
    magnitude = sqrtf(tilt_dot(p_context->filtered, p_context->filtered));
    if (!(magnitude > 0.0F))
    {
        return 0;
    }
    height = TILT_ROLL_SIGN * tilt_dot(p_context->filtered, p_context->side) / magnitude *
             (float32_t)track_mm * 10.0F;
    if (!isfinite(height))
    {
        return 0;
    }
    return (int16_t)roundf(height); /* |height| <= track * 10 <= INT16_MAX for track <= 3276 mm. */
}

/* Convert packed unsigned offset back to signed tenths. Valid result required. */
int16_t tilt_angle_tenths(uint32_t result, uint8_t axis)
{
    uint32_t code = (result >> ((0U == axis) ? 0U : 11U)) & 0x7FFU;
    return (int16_t)((int32_t)code - 900);
}

/* These labels describe orientation, not movement or hump states. */
char const *tilt_condition_name(uint32_t result, uint8_t axis)
{
    uint32_t condition;
    if (0U == (result & TILT_RESULT_VALID)) { return "UNAVAILABLE"; } //check validity

    //get the 2 bit condition at bits 22-23 and roll at bits 23-25
    condition = (result >> ((0U == axis) ? 22U : 24U)) & 3U;
    if (1U == condition) { return (0U == axis) ? "FRONT UP" : "LEFT SIDE LOWER"; }
    if (2U == condition) { return (0U == axis) ? "FRONT DOWN" : "RIGHT SIDE LOWER"; }
    return "LEVEL";
}
