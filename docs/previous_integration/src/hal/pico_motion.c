#include "motion_hal.h"
#include "board_config.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include <math.h>

static motion_config_t configuration;
static volatile uint32_t counts[2];
static volatile uint32_t invalid_edges[2];
static uint8_t previous_ab[2];
static int inferred_direction[2] = {1, 1};
static int last_direction[2];
static double neutral_since[2];
static bool neutral[2] = {true, true};
static bool configured;
static uint16_t wrap;

static uint8_t read_ab(unsigned w)
{
    return (uint8_t)((gpio_get(encoder_pins[w][0]) ? 2U : 0U) |
                     (gpio_get(encoder_pins[w][1]) ? 1U : 0U));
}

static void encoder_irq(void)
{
    unsigned w;
    for (w = 0; w < 2U; ++w) {
        unsigned a = encoder_pins[w][0];
        unsigned b = encoder_pins[w][1];
        uint32_t events_a = gpio_get_irq_event_mask(a);
        uint32_t events_b = configuration.quadrature[w] ? gpio_get_irq_event_mask(b) : 0U;
        if ((events_a | events_b) != 0U) {
            int delta;
            gpio_acknowledge_irq(a, events_a);
            if (configuration.quadrature[w]) {
                uint8_t ab = read_ab(w);
                bool invalid;
                gpio_acknowledge_irq(b, events_b);
                delta = encoder_quadrature(previous_ab[w], ab, &invalid) * configuration.encoder_sign[w];
                previous_ab[w] = ab;
                if (invalid) { ++invalid_edges[w]; }
            } else {
                /* Direction inferred from last drive, including coast: NOT measured. */
                delta = inferred_direction[w];
            }
            counts[w] += (uint32_t)delta;
        }
    }
}

void motion_hal_emergency_stop(void)
{
    unsigned w;
    for (w = 0; w < 2U; ++w) {
        /* Fixed physical board links, even if edited configuration is invalid. */
        unsigned a = 8U + 2U * w;
        unsigned b = a + 1U;
        /* Cytron Robo Pico truth table: LOW/LOW brake. Preload before output-enable. */
        gpio_init(a);
        gpio_init(b);
        gpio_put(a, false);
        gpio_put(b, false);
        gpio_set_dir(a, GPIO_OUT);
        gpio_set_dir(b, GPIO_OUT);
    }
    configured = false;
}

bool motion_hal_init(const motion_config_t *config)
{
    unsigned w;
    uint32_t mask = 0U;
    double divider;
    configuration = *config;
    motion_hal_emergency_stop();
    if (!board_pins_valid()) { return false; }
    /* Keep safe GPIO levels when settings have not been confirmed. */
    if ((config->pwm_hz < 1000U) || (config->pwm_hz > 20000U)) { return false; }
    wrap = 4095U;
    divider = (double)clock_get_hz(clk_sys) / ((double)config->pwm_hz * (wrap + 1U));
    divider = ceil(divider * 16.0) / 16.0; /* Never exceed the requested/max 20 kHz. */
    if ((divider < 1.0) || (divider >= 256.0)) { return false; }
    for (w = 0; w < 2U; ++w) {
        unsigned channel;
        pwm_config pwm = pwm_get_default_config();
        pwm_config_set_wrap(&pwm, wrap);
        pwm_config_set_clkdiv(&pwm, (float)divider);
        pwm_init(pwm_gpio_to_slice_num(motor_pins[w][0]), &pwm, false);
        pwm_set_both_levels(pwm_gpio_to_slice_num(motor_pins[w][0]), 0U, 0U);
        for (channel = 0; channel < 2U; ++channel) {
            unsigned pin = encoder_pins[w][channel];
            if ((channel == 1U) && !config->quadrature[w]) { continue; }
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_IN);
            gpio_disable_pulls(pin); /* External electrical interface must be confirmed. */
            mask |= 1UL << pin;
        }
        previous_ab[w] = read_ab(w);
        neutral_since[w] = motion_hal_time();
    }
    /* Raw handler owns only this mask; leaves SDK callback and other GPIOs intact. */
    gpio_add_raw_irq_handler_masked(mask, encoder_irq);
    for (w = 0; w < 2U; ++w) {
        gpio_set_irq_enabled(encoder_pins[w][0], config->quadrature[w] ?
                             GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL : GPIO_IRQ_EDGE_RISE, true);
        if (config->quadrature[w]) {
            gpio_set_irq_enabled(encoder_pins[w][1], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
        }
    }
    irq_set_enabled(IO_IRQ_BANK0, true);
    if (!config->hardware_confirmed) { return true; }
    for (w = 0; w < 2U; ++w) {
        gpio_set_function(motor_pins[w][0], GPIO_FUNC_PWM);
        gpio_set_function(motor_pins[w][1], GPIO_FUNC_PWM);
        pwm_set_enabled(pwm_gpio_to_slice_num(motor_pins[w][0]), true);
    }
    configured = true;
    return true;
}

encoder_sample_t motion_hal_sample(void)
{
    encoder_sample_t sample;
    uint32_t irq_state = save_and_disable_interrupts();
    sample.count[0] = counts[0];
    sample.count[1] = counts[1];
    sample.invalid[0] = invalid_edges[0];
    sample.invalid[1] = invalid_edges[1];
    restore_interrupts(irq_state);
    sample.reliable = true; /* No independent continuity/stall sensor exists. */
    return sample;
}

void motion_hal_write(const double output[2], bool inhibited, double now)
{
    unsigned w;
    if (!configured) { return; }
    for (w = 0; w < 2U; ++w) {
        double value = (!inhibited && isfinite(output[w])) ?
                       fmax(-configuration.max_output, fmin(configuration.max_output, output[w])) : 0.0;
        int direction = (value > 0.0) ? 1 : ((value < 0.0) ? -1 : 0);
        uint16_t a = 0U;
        uint16_t b = 0U;
        if ((direction != 0) && (last_direction[w] != 0) && (direction != last_direction[w]) &&
            (!neutral[w] || ((now - neutral_since[w]) < configuration.reversal_time))) {
            value = 0.0;
            direction = 0;
        }
        if (direction == 0) {
            if (!neutral[w]) { neutral_since[w] = now; }
            neutral[w] = true;
            if (!configuration.brake) { a = (uint16_t)(wrap + 1U); b = a; }
        } else {
            uint16_t duty = (uint16_t)(fabs(value) * (wrap + 1U));
            uint32_t irq_state = save_and_disable_interrupts();
            inferred_direction[w] = direction;
            restore_interrupts(irq_state);
            last_direction[w] = direction;
            neutral[w] = false;
            if ((direction * configuration.motor_sign[w]) > 0) { a = duty; }
            else { b = duty; }
        }
        /* Both compare values share a single slice register write. */
        pwm_set_both_levels(pwm_gpio_to_slice_num(motor_pins[w][0]), a, b);
    }
}

double motion_hal_time(void)
{
    return (double)time_us_64() / 1000000.0;
}

void motion_hal_console_init(void) { (void)stdio_init_all(); }
int motion_hal_console_read(void) { return getchar_timeout_us(0U); }
