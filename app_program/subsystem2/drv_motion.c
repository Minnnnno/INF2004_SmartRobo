#include "motion_hal.h"
#include "board_config.h"
#include "template_hal.h"
#include "encoder_capture.h"
#include "motor_test.h"
#include "angle_test.h"
#include <math.h>

static motion_config_t configuration;
static volatile encoder_capture_t capture;
static uint32_t encoder_event_mask[4];
static bool encoder_initialised, encoder_irq_registered;
static int last_direction[2];
static double neutral_since[2];
static bool neutral[2] = {true, true};
static bool configured;
static uint16_t wrap;

static void encoder_irq(void)
{
    uint32_t pending[4], any = 0U;
    unsigned bank;
    uint32_t levels;
    for (bank = 0U; bank < 4U; ++bank) {
        pending[bank] = in_w(IO_BANK0_BASE + 0x120U + bank * 4U) & encoder_event_mask[bank];
        any |= pending[bank];
        if (pending[bank] != 0U) { out_w(IO_BANK0_BASE + 0xf0U + bank * 4U, pending[bank]); }
    }
    if (any == 0U) { return; }
    /* Acknowledge only owned flags, then take one coherent A/B level snapshot. */
    levels = in_w(GPIO_IN);
    encoder_capture_events(&capture, pending, levels);
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
        /* Never drive a motor-control pin if it is assigned to an encoder. */
        gpio_set_dir(a, (a != encoder_pins[0][0]) && (a != encoder_pins[0][1]) &&
                        (a != encoder_pins[1][0]) && (a != encoder_pins[1][1]));
        gpio_set_dir(b, (b != encoder_pins[0][0]) && (b != encoder_pins[0][1]) &&
                        (b != encoder_pins[1][0]) && (b != encoder_pins[1][1]));
    }
    configured = false;
}

bool motion_hal_init(const motion_config_t *config)
{
    unsigned w;
    uint32_t mask = 0U;
    uint32_t irq_state;
    double divider;
    configuration = *config;
    motion_hal_emergency_stop();
    if (!board_pins_valid()) { return false; }
    /* Keep safe GPIO levels when settings have not been confirmed. */
    if ((config->pwm_hz < 1000U) || (config->pwm_hz > 20000U)) { return false; }
    wrap = 4095U;
    divider = 125000000.0 / ((double)config->pwm_hz * (wrap + 1U));
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
            gpio_set_dir(pin, false);
            gpio_disable_pulls(pin); /* External electrical interface must be confirmed. */
            gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
            encoder_event_mask[pin / 8U] |= (config->quadrature[w] ? 12UL : 8UL) << ((pin % 8U) * 4U);
            mask |= 1UL << pin;
        }
        neutral_since[w] = motion_hal_time();
    }
    /* Board dispatcher owns IRQ 13; each registered handler owns only its pin mask. */
    if (!board_gpio_irq_register(mask, encoder_irq)) { return false; }
    encoder_irq_registered = true;
    irq_state = save_and_disable_interrupts();
    for (w = 0U; w < 4U; ++w) {
        out_w(IO_BANK0_BASE + 0xf0U + w * 4U, encoder_event_mask[w]);
    }
    encoder_capture_init(&capture, encoder_pins, config->quadrature,
                         config->encoder_sign, in_w(GPIO_IN));
    for (w = 0; w < 2U; ++w) {
        gpio_set_irq_enabled(encoder_pins[w][0], config->quadrature[w] ?
                             GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL : GPIO_IRQ_EDGE_RISE, true);
        if (config->quadrature[w]) {
            gpio_set_irq_enabled(encoder_pins[w][1], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
        }
    }
    encoder_initialised = true;
    restore_interrupts(irq_state);
#if defined(ROBO_ENCODER_DIAGNOSTICS_ONLY) || defined(ROBO_SAFE_MOTOR_TEST) || defined(ROBO_ANGLE_TEST)
    /* This build never changes motor GPIOs to PWM, even if settings are edited. */
    return true;
#else
    if (!config->hardware_confirmed) { return true; }
    for (w = 0; w < 2U; ++w) {
        gpio_set_function(motor_pins[w][0], GPIO_FUNC_PWM);
        gpio_set_function(motor_pins[w][1], GPIO_FUNC_PWM);
        pwm_set_enabled(pwm_gpio_to_slice_num(motor_pins[w][0]), true);
    }
    configured = true;
    return true;
#endif
}

encoder_sample_t motion_hal_sample(void)
{
    encoder_sample_t sample;
    uint32_t irq_state = save_and_disable_interrupts();
    sample.count[0] = capture.count[0];
    sample.count[1] = capture.count[1];
    sample.invalid[0] = capture.invalid[0];
    sample.invalid[1] = capture.invalid[1];
    sample.reliable = encoder_initialised;
    restore_interrupts(irq_state);
    return sample;
}

void motion_hal_encoder_diagnostics(encoder_diagnostics_t *d)
{
    unsigned w, channel;
    uint32_t irq_state = save_and_disable_interrupts();
    d->levels = in_w(GPIO_IN);
    d->initialised = encoder_initialised;
    d->irq_registered = encoder_irq_registered;
    d->nvic_enabled = (in_w(0xe000e100U) & (1UL << 13U)) != 0U;
    d->motors_inhibited = !configured;
#ifdef ROBO_SAFE_MOTOR_TEST
    d->motors_inhibited = !motor_test_active();
#endif
#ifdef ROBO_ANGLE_TEST
    d->motors_inhibited = !angle_test_active();
#endif
    d->sample.reliable = encoder_initialised;
    for (w = 0; w < 2U; ++w) {
        d->sample.count[w] = capture.count[w];
        d->sample.invalid[w] = capture.invalid[w];
        d->previous_ab[w] = capture.previous_ab[w];
        for (channel = 0; channel < 2U; ++channel) {
            unsigned pin = encoder_pins[w][channel];
            d->pins[w][channel] = pin;
            d->edges[w][channel] = capture.edges[w][channel];
            d->irq_events[w][channel] = (uint8_t)((in_w(irq_reg(pin, 0x100U)) >> ((pin % 8U) * 4U)) & 15U);
            d->function[w][channel] = (uint8_t)(in_w(GPIO_CTRL(pin)) & 31U);
        }
    }
    restore_interrupts(irq_state);
}

void motion_hal_write(const double output[2], bool inhibited, double now)
{
#if defined(ROBO_ENCODER_DIAGNOSTICS_ONLY) || defined(ROBO_SAFE_MOTOR_TEST) || defined(ROBO_ANGLE_TEST)
    (void)output; (void)inhibited; (void)now;
    return; /* Startup left all motor outputs LOW; no drive path in this build. */
#else
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
            capture.inferred_direction[w] = direction;
            restore_interrupts(irq_state);
            last_direction[w] = direction;
            neutral[w] = false;
            if ((direction * configuration.motor_sign[w]) > 0) { a = duty; }
            else { b = duty; }
        }
        /* Both compare values share a single slice register write. */
        pwm_set_both_levels(pwm_gpio_to_slice_num(motor_pins[w][0]), a, b);
    }
#endif
}

double motion_hal_time(void)
{
    return (double)time_us_64() / 1000000.0;
}

void motion_hal_console_init(void) { /* Template starts the console service. */ }
int motion_hal_console_read(void)
{
#if TM_CONSOLE_USB_CDC
    extern int tm_usb_console_read(void);
    return tm_usb_console_read();
#else
    if ((in_w(0x40034018U) & (1U << 4U)) != 0U) { return -1; }
    return (int)(in_w(0x40034000U) & 255U);
#endif
}
