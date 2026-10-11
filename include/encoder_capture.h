#ifndef ROBO_ENCODER_CAPTURE_H
#define ROBO_ENCODER_CAPTURE_H
#include <stdbool.h>
#include <stdint.h>

/* ISR capture for all four RP2040 GPIO IRQ status registers. No simulated IO.
 * A is bit 1 and B bit 0. Storage uses defined uint32_t modular arithmetic. */
typedef struct {
    unsigned pins[2][2];
    bool quadrature[2];
    int sign[2], inferred_direction[2];
    uint8_t previous_ab[2];
    uint32_t count[2], invalid[2], edges[2][2];
} encoder_capture_t;

void encoder_capture_init(volatile encoder_capture_t *capture, const unsigned pins[2][2],
                          const bool quadrature[2], const int sign[2], uint32_t levels);
void encoder_capture_events(volatile encoder_capture_t *capture, const uint32_t pending[4],
                            uint32_t levels);
#endif
