/* Demonstration thresholds; tune on the assembled robot before motor use. */
#ifndef TERRAIN_H
#define TERRAIN_H

#include <stdbool.h>
#include <stdint.h>

#define TERRAIN_SLOPE_DEG       (5.0F)
/* Direction signs for the user's mounting; change only between +/-1.0F.
 * Pitch labels and roll angle signs can be reversed independently.
 */
/* User's forward/backward movement is along the reference Y axis. */
#ifndef TERRAIN_FORWARD_AXIS_Y
#define TERRAIN_FORWARD_AXIS_Y  (1)
#endif
#define TERRAIN_PITCH_SIGN      (1.0F)
#define TERRAIN_ROLL_SIGN       (1.0F)
/* +1: positive roll means left side lower. Use -1 to swap left/right.
 * Verify this on the robot; the raw sensor mounting cannot establish it.
 */
#define TERRAIN_LEFT_ROLL_SIGN  (1.0F)
#define TERRAIN_STOP_DEG        (30.0F)
#define TERRAIN_ROUGH_MG        (250.0F)
#define TERRAIN_CONFIRM_SAMPLES (3U)
#define TERRAIN_RECOVER_SAMPLES (5U)

/* Mean of 15 complete ACC readings in the user's latest screenshot.
 * Values are raw acceleration counts; negative Z is intentional.
 */
#define TERRAIN_REFERENCE_X_MG (-61.33F)
#define TERRAIN_REFERENCE_Y_MG (27.07F)
#define TERRAIN_REFERENCE_Z_MG (-1039.07F)

#define TERRAIN_CALIBRATION_SAMPLES (40U)
#define TERRAIN_CALIBRATION_MOTION  (80.0F)

typedef struct
{
    float sum_x;
    float sum_y;
    float sum_z;
    float first_x;
    float first_y;
    float first_z;
    uint32_t count;
} terrain_calibration_t;

typedef enum
{
    TERRAIN_UNKNOWN = 0,
    TERRAIN_LEVEL,
    TERRAIN_SLOPE,
    TERRAIN_ROUGH,
    TERRAIN_EXCESS_TILT,
    TERRAIN_UPHILL,
    TERRAIN_DOWNHILL,
    TERRAIN_SIDE_SLOPE,
    TERRAIN_SIDE_LEFT,
    TERRAIN_SIDE_RIGHT
} terrain_condition_t;

typedef struct
{
    uint32_t confirmation_count;
    terrain_condition_t candidate;
    terrain_condition_t condition;
} terrain_axis_state_t;

typedef struct
{
    terrain_axis_state_t pitch_axis;
    terrain_axis_state_t roll_axis;
    /* Orthonormal axes of the calibrated level reference. */
    float reference_x[3];
    float reference_y[3];
    float reference_z[3];
    bool b_calibrated;
    float gravity_x;
    float gravity_y;
    float gravity_z;
    uint32_t confirmation_count;
    terrain_condition_t candidate;
    terrain_condition_t condition;
    bool b_initialized;
} terrain_filter_t;

typedef struct
{
    float pitch_deg;
    float roll_deg;
    float tilt_deg;
    float motion_mg;
    /* Independent direction reports; UNKNOWN until confirmed/valid. */
    terrain_condition_t pitch_condition;
    terrain_condition_t roll_condition;
    terrain_condition_t condition;
    bool b_tilt_valid;
} terrain_result_t;

/* Load the fixed reference above without sampling the current orientation.
 * Returns false for NULL or invalid reference magnitude; output unchanged.
 */
bool terrain_use_fixed_reference(terrain_filter_t *p_filter);

/* Optional capture API; the standalone main uses the fixed reference.
 * Calibrate on known level ground, motionless, for 40 fresh samples.
 * Motion or invalid acceleration restarts the averaging window.
 * This is a mounting reference, not a full accelerometer bias calibration.
 */
bool terrain_calibration_add(terrain_calibration_t *p_calibration,
                             terrain_filter_t *p_filter,
                             int16_t axis_x, int16_t axis_y, int16_t axis_z);
/* Call every 50 ms. Reset clears motion history but preserves calibration.
 * Zero-initialize at boot, then load the fixed reference before updates.
 * No motor is driven. Forward/side axes follow TERRAIN_FORWARD_AXIS_Y.
 * Pitch and roll directions are confirmed and reported independently.
 * Overall condition retains dominant tilt, roughness and stop priorities.
 * Generic SLOPE remains available for callers using the older interface.
 */
void terrain_reset(terrain_filter_t *p_filter);
bool terrain_update(terrain_filter_t *p_filter, int16_t axis_x, int16_t axis_y,
                    int16_t axis_z, terrain_result_t *p_result);
char const *terrain_condition_name(terrain_condition_t condition);
char const *terrain_action_name(terrain_condition_t condition);

#endif /* TERRAIN_H */
