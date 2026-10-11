#include "encoder_capture.h"
#include "motion.h"

static uint8_t ab_state(const volatile encoder_capture_t *capture, unsigned wheel,
                        uint32_t levels)
{
    return (uint8_t)((((levels >> capture->pins[wheel][0]) & 1U) << 1U) |
                     ((levels >> capture->pins[wheel][1]) & 1U));
}

void encoder_capture_init(volatile encoder_capture_t *capture, const unsigned pins[2][2],
                          const bool quadrature[2], const int sign[2], uint32_t levels)
{
    unsigned wheel;
    for (wheel = 0; wheel < 2U; ++wheel) {
        capture->pins[wheel][0] = pins[wheel][0];
        capture->pins[wheel][1] = pins[wheel][1];
        capture->quadrature[wheel] = quadrature[wheel];
        capture->sign[wheel] = sign[wheel];
        capture->inferred_direction[wheel] = 1;
        capture->previous_ab[wheel] = ab_state(capture, wheel, levels);
        capture->count[wheel] = 0U;
        capture->invalid[wheel] = 0U;
        capture->edges[wheel][0] = 0U;
        capture->edges[wheel][1] = 0U;
    }
}

void encoder_capture_events(volatile encoder_capture_t *capture, const uint32_t pending[4],
                            uint32_t levels)
{
    unsigned wheel;
    for (wheel = 0; wheel < 2U; ++wheel) {
        unsigned channel;
        uint32_t events[2];
        int delta;
        for (channel = 0; channel < 2U; ++channel) {
            unsigned pin = capture->pins[wheel][channel];
            events[channel] = (pending[pin / 8U] >> ((pin % 8U) * 4U)) & 12U;
            if ((channel == 1U) && !capture->quadrature[wheel]) { events[channel] = 0U; }
            /* Separate latched rising/falling flags; repeated same-type edges can coalesce. */
            capture->edges[wheel][channel] += ((events[channel] & 4U) != 0U ? 1U : 0U) +
                                              ((events[channel] & 8U) != 0U ? 1U : 0U);
        }
        if ((events[0] | events[1]) == 0U) { continue; }
        if (capture->quadrature[wheel]) {
            uint8_t ab = ab_state(capture, wheel, levels);
            bool invalid;
            delta = encoder_quadrature(capture->previous_ab[wheel], ab, &invalid) * capture->sign[wheel];
            capture->previous_ab[wheel] = ab;
            if (invalid) { ++capture->invalid[wheel]; }
        } else {
            /* Single-channel direction is inferred, never physically measured. */
            delta = capture->inferred_direction[wheel];
        }
        capture->count[wheel] += (uint32_t)delta;
    }
}
