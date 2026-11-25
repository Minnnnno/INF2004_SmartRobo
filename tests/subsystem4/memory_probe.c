/* Buddy 4 minimum-bit RAM probe (Change Requirement 1, Week 13 evidence).
 *
 * Not part of the firmware or the host tests. measure_compact.py compiles this
 * file with the Pico's own compiler (arm-none-eabi-gcc, Cortex-M0+) and reads
 * the size of every global below with arm-none-eabi-nm -S, so each size is the
 * real ARM size including padding.
 *
 * before_* = the representation without bit packing: copied from the first
 *            integrated Buddy 4 (commit b96a929, 11 Oct 2026) where that version
 *            had the field, otherwise the plain unpacked type (one enum/bool per
 *            value, float for millimetres).
 * after_*  = the packed type used now, taken from the real headers, so the
 *            probe follows the code if a header changes.
 * whole_*  = today's complete Buddy 4 structures, for the total RAM table.
 */
#include <stdbool.h>
#include <stdint.h>
#include "gy511_read.h"
#include "tilt.h"
#include "terrain.h"
#include "lift.h"
#include "buddy4.h"
#include "motion_class.h"

/* ---- 1. Raw GY-511 sample (acceleration + magnetic, 3 axes each) ---- */
typedef struct /* b96a929 gy511_read.h: gy511_sample_t */
{
    int16_t acceleration_x;
    int16_t acceleration_y;
    int16_t acceleration_z;
    int16_t magnetic_x;
    int16_t magnetic_y;
    int16_t magnetic_z;
} before_sample_t;
typedef struct /* now: two 5-byte packed 3 x 12-bit samples (gy511_read.h) */
{
    gy511_acceleration_t acceleration;
    gy511_magnetic_t magnetic;
} after_sample_t;
before_sample_t before_sample;
after_sample_t after_sample;

/* ---- 2. Tilt result handed from tilt to the lifecycle and terrain ---- */
typedef enum /* b96a929 terrain.h: terrain_condition_t */
{
    BEFORE_TERRAIN_UNKNOWN = 0, BEFORE_TERRAIN_LEVEL, BEFORE_TERRAIN_SLOPE,
    BEFORE_TERRAIN_ROUGH, BEFORE_TERRAIN_EXCESS_TILT, BEFORE_TERRAIN_UPHILL,
    BEFORE_TERRAIN_DOWNHILL, BEFORE_TERRAIN_SIDE_SLOPE, BEFORE_TERRAIN_SIDE_LEFT,
    BEFORE_TERRAIN_SIDE_RIGHT
} before_condition_t;
typedef struct /* b96a929 terrain.h: terrain_result_t */
{
    float pitch_deg;
    float roll_deg;
    float tilt_deg;
    float motion_mg;
    before_condition_t pitch_condition;
    before_condition_t roll_condition;
    before_condition_t condition;
    bool b_tilt_valid;
} before_tilt_result_t;
before_tilt_result_t before_tilt_result;
uint32_t after_tilt_result; /* now: 27-bit packed result (tilt.h) */

/* ---- 3. Lifecycle control: state, retry attempts, sub-phase ---- */
typedef enum { BEFORE_B4_INIT, BEFORE_B4_MONITORING, BEFORE_B4_RECOVERY,
               BEFORE_B4_FAULT, BEFORE_B4_STOP } before_b4_state_t;
typedef enum { BEFORE_PHASE_0, BEFORE_PHASE_1, BEFORE_PHASE_2 } before_b4_phase_t;
typedef struct
{
    before_b4_state_t state;
    unsigned attempts;
    before_b4_phase_t phase;
} before_control_t;
before_control_t before_control;
buddy4_control_t after_control; /* now: 3 + 2 + 2 bit-field, one byte (buddy4.h) */

/* ---- 4. Hump results: last peak (mm + side) and run maximum (mm + valid) ---- */
typedef struct
{
    float last_peak_mm;
    bool last_peak_right;
    float run_max_mm;
    bool run_max_valid;
} before_hump_t;
typedef struct /* now: 7-bit mm + 1 flag bit each (terrain.h) */
{
    __typeof__(((terrain_t *)0)->last_peak) last_peak;
    __typeof__(((terrain_t *)0)->run_max) run_max;
} after_hump_t;
before_hump_t before_hump;
after_hump_t after_hump;

/* ---- 5. Hump detector phase and flags ---- */
typedef enum { BEFORE_WAITING, BEFORE_CLIMBING, BEFORE_DESCENDING } before_phase_t;
typedef struct
{
    before_phase_t phase;
    bool valid;
    bool pending;
    bool side_right;
    bool completed_event;
} before_terrain_flags_t;
before_terrain_flags_t before_terrain_flags;
__typeof__(((terrain_t *)0)->flags) after_terrain_flags; /* now: one byte (terrain.c) */

/* ---- 6. Tilt capture progress and tilt conditions ---- */
typedef struct /* b96a929: uint32_t count + calibrated/initialised flags */
{
    uint32_t capture_count;
    bool calibrated;
    bool filter_ready;
    before_condition_t pitch_condition;
    before_condition_t roll_condition;
} before_tilt_progress_t;
typedef struct /* now: progress byte (6-bit count + 2 flags) and conditions byte */
{
    __typeof__(((tilt_context_t *)0)->progress) progress;
    __typeof__(((tilt_context_t *)0)->conditions) conditions;
} after_tilt_progress_t;
before_tilt_progress_t before_tilt_progress;
after_tilt_progress_t after_tilt_progress;

/* ---- 7. Motion class debounce: reported class, candidate, repeat count ---- */
typedef enum { BEFORE_MC_STATIONARY, BEFORE_MC_MOVING, BEFORE_MC_ACCELERATING,
               BEFORE_MC_REVERSING, BEFORE_MC_TURNING, BEFORE_MC_CLIMBING,
               BEFORE_MC_DESCENDING, BEFORE_MC_OVERTURNED } before_class_t;
typedef struct
{
    before_class_t reported;
    before_class_t candidate;
    uint8_t count;
} before_class_state_t;
before_class_state_t before_class_state;
__typeof__(((motion_class_t *)0)->packed) after_class_state; /* now: 3 + 3 + 2 bits */

/* ---- 8. Compass validity of the 5 heading-window slots ---- */
bool before_compass_window[MC_HEADING_SLOTS];
__typeof__(((motion_class_t *)0)->compass_bits) after_compass_window; /* now: 5-bit mask */

/* ---- 9. Lift latch: condition + timing flags ---- */
typedef enum { BEFORE_LIFT_NONE, BEFORE_LIFT_LIFTED, BEFORE_LIFT_OVERTURNED } before_lift_condition_t;
typedef struct
{
    before_lift_condition_t condition;
    bool over_timing;
    bool clear_timing;
    bool last_valid;
} before_lift_latch_t;
before_lift_latch_t before_lift_latch;
__typeof__(((lift_t *)0)->flags) after_lift_latch; /* now: 2-bit condition + 3 flags in one byte */

/* ---- Today's whole Buddy 4 structures (for the total) ---- */
buddy4_t whole_buddy4;
motion_class_t whole_motion_class;
tilt_context_t whole_tilt;
terrain_t whole_terrain;
lift_t whole_lift;
