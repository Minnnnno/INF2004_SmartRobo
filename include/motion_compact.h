/** @file motion_compact.h
 * @brief Checked SI/public-to-packed adapters, owned by Buddy 2.
 * No shared state is accessed here. The caller owns/synchronizes its storage.
 */
#ifndef MOTION_COMPACT_H
#define MOTION_COMPACT_H

#include "motion.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define MOTION_DISTANCE_LIMIT 5000
#define MOTION_SPEED_LIMIT     250
#define MOTION_ANGLE_LIMIT     360
#define MOTION_RAD_PER_DEG     0.017453292519943295

static inline bool motion_compact_limits(const motion_config_t *p_config)
{
    return (NULL != p_config) && isfinite(p_config->max_distance) &&
           (p_config->max_distance > 0.0) && (p_config->max_distance <= 5.0) &&
           isfinite(p_config->max_speed) && (p_config->max_speed > 0.0) &&
           (p_config->max_speed <= 0.25) && isfinite(p_config->max_angle) &&
           (p_config->max_angle > 0.0) &&
           (p_config->max_angle <= (360.0 * MOTION_RAD_PER_DEG));
}

/* Validate BEFORE rounding. round() specifies nearest, halfway away from zero.
 * No cast of NaN, infinity, or out-of-range floating values is permitted. */
static inline bool motion_compact_quantize(float64_t value, float64_t scale,
                                           int32_t limit, int32_t *p_result)
{
    float64_t scaled = value * scale;
    bool b_valid = (NULL != p_result) && isfinite(scaled) &&
                   (fabs(scaled) <= (float64_t)limit);
    if (b_valid)
    {
        *p_result = (int32_t)round(scaled);
    }
    return b_valid;
}

static inline void motion_compact_put(uint8_t *p_bytes, uint32_t offset,
                                      uint32_t width, int32_t value)
{
    uint32_t modulus = UINT32_C(1) << width;
    uint32_t encoded = (value < 0) ? modulus - (uint32_t)(-value) : (uint32_t)value;
    for (uint32_t bit = 0u; bit < width; ++bit)
    {
        uint32_t position = offset + bit;
        uint32_t mask = UINT32_C(1) << (position % 8u);
        uint32_t byte = p_bytes[position / 8u];
        byte = (byte & ~mask) | (((encoded >> bit) & 1u) * mask);
        p_bytes[position / 8u] = (uint8_t)byte;
    }
}

static inline int32_t motion_compact_get(const uint8_t *p_bytes, uint32_t offset,
                                        uint32_t width)
{
    uint32_t encoded = 0u;
    uint32_t modulus = UINT32_C(1) << width;
    for (uint32_t bit = 0u; bit < width; ++bit)
    {
        uint32_t position = offset + bit;
        uint32_t byte = p_bytes[position / 8u];
        encoded |= ((byte >> (position % 8u)) & 1u) << bit;
    }
    /* Subtraction within int32_t avoids right-shifting signed integers and
     * avoids implementation-defined conversion of large unsigned values. */
    return ((encoded & (modulus / 2u)) != 0u) ?
           (int32_t)encoded - (int32_t)modulus : (int32_t)encoded;
}

static inline motion_mode_t motion_compact_mode(const motion_storage_command_t *p_stored)
{
    return (motion_mode_t)(((uint32_t)p_stored->payload[5] >> 2u) & 7u);
}

static inline bool motion_compact_encode(const motion_command_t *p_public,
                                         const motion_config_t *p_config,
                                         motion_storage_command_t *p_stored)
{
    int32_t distance = 0;
    int32_t angle = 0;
    int32_t velocity[2] = {0, 0};
    motion_storage_command_t encoded = {0};
    if ((NULL == p_public) || (NULL == p_stored) || !motion_compact_limits(p_config))
    {
        return false;
    }
    if (((uint32_t)p_public->mode > (uint32_t)ARC_MOVE) ||
        !isfinite(p_public->value) || !isfinite(p_public->angle) ||
        !isfinite(p_public->duration) || !isfinite(p_public->speed[0]) ||
        !isfinite(p_public->speed[1]) || !isfinite(p_public->expires) ||
        !isfinite(p_public->deadline))
    {
        return false;
    }
    if ((DISTANCE_MOVE == p_public->mode) || (ARC_MOVE == p_public->mode))
    {
        if ((fabs(p_public->value) > p_config->max_distance) ||
            !motion_compact_quantize(p_public->value, 1000.0, MOTION_DISTANCE_LIMIT, &distance))
        {
            return false;
        }
    }
    if ((ENCODER_TURN == p_public->mode) || (ARC_MOVE == p_public->mode))
    {
        float64_t radians = (ENCODER_TURN == p_public->mode) ? p_public->value : p_public->angle;
        if ((fabs(radians) > p_config->max_angle) ||
            !motion_compact_quantize(radians, 1.0 / MOTION_RAD_PER_DEG, MOTION_ANGLE_LIMIT, &angle))
        {
            return false;
        }
    }
    for (uint32_t wheel = 0u; wheel < 2u; ++wheel)
    {
        float64_t limit = (MANUAL_OUTPUT == p_public->mode) ?
                          p_config->manual_limit : p_config->max_speed;
        if (fabs(p_public->speed[wheel]) > limit)
        {
            return false;
        }
        if ((MANUAL_OUTPUT == p_public->mode) || (ARC_MOVE == p_public->mode))
        {
            encoded.precise_speed[wheel] = p_public->speed[wheel];
        }
        else if (!motion_compact_quantize(p_public->speed[wheel], 1000.0,
                                          MOTION_SPEED_LIMIT, &velocity[wheel]))
        {
            return false;
        }
    }
    encoded.id = p_public->id;
    encoded.generation = p_public->generation;
    encoded.expires = p_public->expires;
    encoded.deadline = p_public->deadline;
    encoded.duration = (ARC_MOVE == p_public->mode) ? p_public->duration : 0.0;
    motion_compact_put(encoded.payload, 0u, 14u, distance);
    motion_compact_put(encoded.payload, 14u, 9u, velocity[0]);
    motion_compact_put(encoded.payload, 23u, 9u, velocity[1]);
    motion_compact_put(encoded.payload, 32u, 10u, angle);
    /* Modes are nonnegative unsigned flags, not signed numerical fields. */
    encoded.payload[5] = (uint8_t)((uint32_t)encoded.payload[5] |
                                  ((uint32_t)p_public->mode << 2u));
    *p_stored = encoded;
    return true;
}

static inline bool motion_compact_decode(const motion_storage_command_t *p_stored,
                                         const motion_config_t *p_config,
                                         motion_command_t *p_public)
{
    motion_command_t decoded = {0};
    if ((NULL == p_stored) || (NULL == p_public) || !motion_compact_limits(p_config))
    {
        return false;
    }
    int32_t distance = motion_compact_get(p_stored->payload, 0u, 14u);
    int32_t left = motion_compact_get(p_stored->payload, 14u, 9u);
    int32_t right = motion_compact_get(p_stored->payload, 23u, 9u);
    int32_t angle = motion_compact_get(p_stored->payload, 32u, 10u);
    decoded.mode = motion_compact_mode(p_stored);
    if (((uint32_t)decoded.mode > (uint32_t)ARC_MOVE) ||
        (0u != ((uint32_t)p_stored->payload[5] & 0xe0u)) ||
        (distance < -MOTION_DISTANCE_LIMIT) || (distance > MOTION_DISTANCE_LIMIT) ||
        (left < -MOTION_SPEED_LIMIT) || (left > MOTION_SPEED_LIMIT) ||
        (right < -MOTION_SPEED_LIMIT) || (right > MOTION_SPEED_LIMIT) ||
        (angle < -MOTION_ANGLE_LIMIT) || (angle > MOTION_ANGLE_LIMIT))
    {
        return false;
    }
    decoded.id = p_stored->id;
    decoded.generation = p_stored->generation;
    decoded.expires = p_stored->expires;
    decoded.deadline = p_stored->deadline;
    decoded.value = (ENCODER_TURN == decoded.mode) ?
                    (float64_t)angle * MOTION_RAD_PER_DEG : (float64_t)distance / 1000.0;
    decoded.angle = (ARC_MOVE == decoded.mode) ? (float64_t)angle * MOTION_RAD_PER_DEG : 0.0;
    decoded.duration = p_stored->duration;
    decoded.speed[0] = (float64_t)left / 1000.0;
    decoded.speed[1] = (float64_t)right / 1000.0;
    if ((MANUAL_OUTPUT == decoded.mode) || (ARC_MOVE == decoded.mode))
    {
        decoded.speed[0] = p_stored->precise_speed[0];
        decoded.speed[1] = p_stored->precise_speed[1];
    }
    if ((ARC_MOVE == decoded.mode) && (decoded.duration > 0.0))
    {
        /* Preserve the exact timed-arc kinematic identity after quantizing the
         * goal; rounding derived velocities independently would violate it. */
        decoded.speed[0] = (decoded.value - decoded.angle * p_config->track / 2.0) / decoded.duration;
        decoded.speed[1] = (decoded.value + decoded.angle * p_config->track / 2.0) / decoded.duration;
    }
    if (!isfinite(decoded.speed[0]) || !isfinite(decoded.speed[1]) ||
        !isfinite(decoded.duration) || !isfinite(decoded.expires) || !isfinite(decoded.deadline))
    {
        return false;
    }
    float64_t limit = (MANUAL_OUTPUT == decoded.mode) ? p_config->manual_limit : p_config->max_speed;
    if ((fabs(decoded.speed[0]) > limit) || (fabs(decoded.speed[1]) > limit) ||
        ((DISTANCE_MOVE == decoded.mode || ARC_MOVE == decoded.mode) &&
         (fabs(decoded.value) > p_config->max_distance)) ||
        ((ENCODER_TURN == decoded.mode) && (fabs(decoded.value) > p_config->max_angle)) ||
        (fabs(decoded.angle) > p_config->max_angle))
    {
        return false;
    }
    *p_public = decoded;
    return true;
}

#endif /* MOTION_COMPACT_H */
