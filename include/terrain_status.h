/* Buddy 4 terrain slot decoder for other subsystems (owner: Buddy 4).
 * Read-only: these helpers only interpret the published Terrain snapshot;
 * they never command motion. The task that drives decides what to do.
 * Header-only, so callers link it whether or not ROBO_SUBSYSTEM4 is built:
 * without Buddy 4 the slot stays unavailable and every query says "no".
 *
 * Slot layout (docs/API.md): values[0] = turn rate rad/s; values[3] =
 * validity bits 0..2 | T_STATE << 3 | motion class << 6 | motion valid << 9.
 */
#ifndef ROBO_TERRAIN_STATUS_H
#define ROBO_TERRAIN_STATUS_H
#include "robot.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* T_STATE codes (3 bits). 7 is never published and means UNKNOWN here. */
#define TERRAIN_TS_INIT       (0U)
#define TERRAIN_TS_WAITING    (1U) /* Monitoring, level ground. */
#define TERRAIN_TS_CLIMBING   (2U) /* Monitoring, one wheel going up a hump. */
#define TERRAIN_TS_DESCENDING (3U) /* Monitoring, one wheel coming down a hump. */
#define TERRAIN_TS_RECOVERY   (4U)
#define TERRAIN_TS_FAULT      (5U)
#define TERRAIN_TS_STOP       (6U)
#define TERRAIN_TS_UNKNOWN    (7U) /* Slot unavailable or malformed. */

/* Published T_STATE, or TERRAIN_TS_UNKNOWN when the slot holds no usable data. */
static inline uint8_t terrain_published_state(subsystem_status_t const *p_terrain)
{
    double word;

    if ((NULL == p_terrain) || !p_terrain->available)
    {
        return (uint8_t)TERRAIN_TS_UNKNOWN;
    }
    word = p_terrain->values[3];
    if (!(word >= 0.0) || (word > 1023.0)) /* Also rejects NaN; 10 bits are defined. */
    {
        return (uint8_t)TERRAIN_TS_UNKNOWN;
    }
    return (uint8_t)(((unsigned)word >> 3U) & 7U);
}

/* Motion class (bits 6..8 of values[3]) and its validity (bit 9). The turn
 * rate in values[0] (rad/s, + = left) shares the same validity bit.
 * Codes match app_program/subsystem4/motion_class.h. */
#define TERRAIN_MC_STATIONARY   (0U)
#define TERRAIN_MC_MOVING       (1U)
#define TERRAIN_MC_ACCELERATING (2U)
#define TERRAIN_MC_REVERSING    (3U)
#define TERRAIN_MC_TURNING      (4U)
#define TERRAIN_MC_CLIMBING     (5U)
#define TERRAIN_MC_DESCENDING   (6U)
#define TERRAIN_MC_IMPACT       (7U)
#define TERRAIN_MC_SHIFT        (6U)
#define TERRAIN_MOTION_VALID    (0x200U)

/* True with *p_class set when the slot carries a valid motion class. */
static inline bool terrain_motion_class(subsystem_status_t const *p_terrain, uint8_t *p_class)
{
    unsigned bits;

    if ((NULL == p_class) || (TERRAIN_TS_UNKNOWN == terrain_published_state(p_terrain)))
    {
        return false;
    }
    bits = (unsigned)p_terrain->values[3];
    if (0U == (bits & TERRAIN_MOTION_VALID))
    {
        return false;
    }
    *p_class = (uint8_t)((bits >> TERRAIN_MC_SHIFT) & 7U);
    return true;
}

/* IMU fault indicator (CR1): Buddy 4 latched SENSOR_FAULT, published as
 * T_STATE 5. The robot keeps driving; only terrain data is unavailable.
 * A latched fault clears by power-up (level reference needs a level boot). */
static inline bool terrain_imu_faulted(robot_status_t const *p_status)
{
    return (NULL != p_status) &&
           (TERRAIN_TS_FAULT == terrain_published_state(&p_status->terrain));
}

#endif /* ROBO_TERRAIN_STATUS_H */
