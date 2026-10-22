/** @file motion_storage.h
 * @brief Buddy 2 private storage; public command/status layouts stay unchanged.
 */
#ifndef MOTION_STORAGE_H
#define MOTION_STORAGE_H

#include <float.h>
#include <limits.h>
#include <stdint.h>

/* BARR-C 5.4 alias: this C11 toolchain has no standard float64_t typedef. */
typedef double float64_t;

#define MOTION_PAYLOAD_BYTES     6u
#define MOTION_EVENT_CAPACITY    8u
#define MOTION_STILL_TRACKING    0x01u
#define MOTION_GOAL_PENDING      0x02u
#define MOTION_FEEDBACK_LATCHED  0x04u

/* Unsigned, explicitly ordered bit stream: distance[0:13], left[14:22],
 * right[23:31], angle[32:41], mode[42:44], reserved-zero[45:47].
 * Signed fields use explicitly encoded two's complement, not C bit-fields.
 * Times/identity are NOT shortened. ARC/manual extension retains precision. */
typedef struct
{
    uint32_t id;
    uint32_t generation;
    float64_t expires;
    float64_t deadline;
    float64_t duration;
    float64_t precise_speed[2];
    uint8_t payload[MOTION_PAYLOAD_BYTES];
} motion_storage_command_t;

/* Structure of arrays avoids three padding bytes per event. Each info byte
 * contains event type[0:2] and primary fault[3:6]; bit7 is reserved zero. */
typedef struct
{
    uint32_t id[MOTION_EVENT_CAPACITY];
    uint32_t generation[MOTION_EVENT_CAPACITY];
    uint8_t info[MOTION_EVENT_CAPACITY];
} motion_storage_events_t;

/* Private HAL configuration. Other motion_config_t members are not used by
 * the motor driver after initialization; do not keep a second full copy. */
typedef struct
{
    float64_t max_output;
    float64_t reversal_time;
    uint8_t flags; /* brake bit0; negative motor signs bits1:2 */
} motion_motor_storage_t;

_Static_assert(8 == CHAR_BIT, "Motion storage requires eight-bit bytes");
_Static_assert((53 <= DBL_MANT_DIG) && (8u == sizeof(float64_t)),
               "PID/odometry require binary64 precision");
_Static_assert(56u == sizeof(motion_storage_command_t), "Command layout changed");
_Static_assert(72u == sizeof(motion_storage_events_t), "Event storage changed");
_Static_assert(24u == sizeof(motion_motor_storage_t), "Motor storage changed");

#endif /* MOTION_STORAGE_H */
