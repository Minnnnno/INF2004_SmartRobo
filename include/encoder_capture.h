#ifndef ROBO_ENCODER_CAPTURE_H
#define ROBO_ENCODER_CAPTURE_H
#include <stdbool.h>
#include <stdint.h>

/* ISR capture for all four RP2040 GPIO IRQ status registers. No simulated IO.
 * A is bit 1 and B bit 0. Storage uses defined uint32_t modular arithmetic. */
typedef struct {
    uint32_t count[2], invalid[2], edges[2][2];
    uint8_t pins[2][2];
    uint8_t flags; /* quadrature bits0:1, negative signs2:3, inferred reverse4:5 */
    uint8_t previous_ab; /* left AB bits0:1, right AB bits2:3 */
} encoder_capture_t;

_Static_assert(40u == sizeof(encoder_capture_t), "Encoder storage layout changed");

static inline uint8_t encoder_capture_previous_ab(const volatile encoder_capture_t *p_capture,
                                                   uint32_t wheel)
{
    return (uint8_t)(((uint32_t)p_capture->previous_ab >> (wheel * 2u)) & 3u);
}

/* Caller must hold the existing interrupt exclusion when shared with an ISR. */
static inline void encoder_capture_set_direction(volatile encoder_capture_t *p_capture,
                                                  uint32_t wheel, int direction)
{
    uint32_t flags = p_capture->flags;
    uint32_t mask = UINT32_C(1) << (4u + wheel);
    flags = (direction < 0) ? (flags | mask) : (flags & ~mask);
    p_capture->flags = (uint8_t)flags;
}

void encoder_capture_init(volatile encoder_capture_t *capture, const unsigned pins[2][2],
                          const bool quadrature[2], const int sign[2], uint32_t levels);
void encoder_capture_events(volatile encoder_capture_t *capture, const uint32_t pending[4],
                            uint32_t levels);
#endif
