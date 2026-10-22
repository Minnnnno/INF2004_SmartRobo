/** @file encoder_capture.c
 * @brief ISR-owned quadrature capture with packed metadata and full counters.
 */
#include "encoder_capture.h"
#include "motion.h"

#define ENCODER_WHEELS   2u
#define ENCODER_CHANNELS 2u

static uint8_t encoder_ab_read(const volatile encoder_capture_t *p_capture,
                               uint32_t wheel, uint32_t levels)
{
    uint32_t phase_a = (levels >> p_capture->pins[wheel][0]) & 1u;
    uint32_t phase_b = (levels >> p_capture->pins[wheel][1]) & 1u;
    return (uint8_t)((phase_a << 1u) | phase_b);
}

static void encoder_ab_store(volatile encoder_capture_t *p_capture,
                              uint32_t wheel, uint8_t state)
{
    uint32_t shift = wheel * 2u;
    uint32_t previous = p_capture->previous_ab;
    previous = (previous & ~(UINT32_C(3) << shift)) | ((uint32_t)state << shift);
    p_capture->previous_ab = (uint8_t)previous;
}

void encoder_capture_init(volatile encoder_capture_t *p_capture,
                          const unsigned pins[2][2], const bool quadrature[2],
                          const int sign[2], uint32_t levels)
{
    p_capture->flags = 0u;
    p_capture->previous_ab = 0u;
    for (uint32_t wheel = 0u; wheel < ENCODER_WHEELS; ++wheel)
    {
        p_capture->pins[wheel][0] = (uint8_t)pins[wheel][0];
        p_capture->pins[wheel][1] = (uint8_t)pins[wheel][1];
        uint32_t flags = p_capture->flags;
        if (quadrature[wheel])
        {
            flags |= UINT32_C(1) << wheel;
        }
        if (sign[wheel] < 0)
        {
            flags |= UINT32_C(1) << (wheel + 2u);
        }
        p_capture->flags = (uint8_t)flags;
        encoder_ab_store(p_capture, wheel, encoder_ab_read(p_capture, wheel, levels));
        p_capture->count[wheel] = 0u;
        p_capture->invalid[wheel] = 0u;
        p_capture->edges[wheel][0] = 0u;
        p_capture->edges[wheel][1] = 0u;
    }
}

void encoder_capture_events(volatile encoder_capture_t *p_capture,
                            const uint32_t pending[4], uint32_t levels)
{
    for (uint32_t wheel = 0u; wheel < ENCODER_WHEELS; ++wheel)
    {
        uint32_t events[ENCODER_CHANNELS] = {0u, 0u};
        bool b_quadrature = (0u != ((uint32_t)p_capture->flags & (UINT32_C(1) << wheel)));
        for (uint32_t channel = 0u; channel < ENCODER_CHANNELS; ++channel)
        {
            uint32_t pin = p_capture->pins[wheel][channel];
            events[channel] = (pending[pin / 8u] >> ((pin % 8u) * 4u)) & 12u;
            if ((1u == channel) && !b_quadrature)
            {
                events[channel] = 0u;
            }
            /* Count separate latched rising/falling flags. Repeated same-type
             * edges can still coalesce in the hardware, as before. */
            p_capture->edges[wheel][channel] +=
                ((0u != (events[channel] & 4u)) ? 1u : 0u) +
                ((0u != (events[channel] & 8u)) ? 1u : 0u);
        }
        if (0u == (events[0] | events[1]))
        {
            continue;
        }
        int32_t delta = 0;
        if (b_quadrature)
        {
            bool b_invalid = false;
            uint8_t state = encoder_ab_read(p_capture, wheel, levels);
            delta = encoder_quadrature(encoder_capture_previous_ab(p_capture, wheel), state, &b_invalid);
            if (0u != ((uint32_t)p_capture->flags & (UINT32_C(1) << (wheel + 2u))))
            {
                delta = -delta;
            }
            encoder_ab_store(p_capture, wheel, state);
            if (b_invalid)
            {
                ++p_capture->invalid[wheel];
            }
        }
        else
        {
            delta = (0u != ((uint32_t)p_capture->flags & (UINT32_C(1) << (wheel + 4u)))) ? -1 : 1;
        }
        /* Defined modulo-2^32 arithmetic, including reverse movement. */
        p_capture->count[wheel] += (uint32_t)delta;
    }
}
