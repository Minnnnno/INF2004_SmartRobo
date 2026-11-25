#include "../../include/float_types.h"
#include "motion_hal.h"
#include "board_config.h"
#include "template_hal.h"
#include "encoder_capture.h"
#include "motor_test.h"
#include "angle_test.h"
#include <math.h>
#include <stddef.h>

#define GPIO_IRQ_BANKS 4U /* interrupt registers, 8 pins each */

static motion_motor_storage_t     g_motor_config;
static volatile encoder_capture_t g_capture;
static uint32_t                   g_encoder_event_mask[4];
static uint8_t                    g_driver_flags;
static float64_t                  g_neutral_since[2];
/* neutral bits0:1; last directions bits2:5 */
static uint8_t   g_wheel_flags = 3u;
static uint16_t  g_wrap;
static uint16_t  g_configured_divider_16;
static uint16_t  g_last_compare[2][2];
static float64_t g_last_requested[2];
static float64_t g_last_applied[2];
static uint8_t   g_reversal_wait_mask;
static bool      gb_last_inhibited = true;
static uint32_t  g_last_write_us;

#define DRIVER_ENCODER_READY  0x01u
#define DRIVER_IRQ_REGISTERED 0x02u
#define DRIVER_PWM_READY      0x04u
#define MOTOR_BRAKE_ENABLED   0x01u

static bool    motor_state_get(uint8_t mask);
static void    motor_state_set(uint8_t mask, bool b_value);
static bool    motor_is_neutral(uint32_t wheel);
static void    motor_neutral_set(uint32_t wheel, bool b_value);
static int32_t motor_direction_get(uint32_t wheel);
static void    motor_direction_set(uint32_t wheel, int32_t direction);
static int32_t motor_sign_get(uint32_t wheel);
static void    encoder_irq(void);
static void    init_wheel(const motion_config_t * p_config, unsigned wheel,
                          float64_t divider, uint32_t * p_mask);
static void    enable_encoders(const motion_config_t * p_config);
static int32_t diagnostic_permille(float64_t value);

void
motion_hal_emergency_stop (void)
{
    unsigned wheel;

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        /* Fixed physical board links, even if edited configuration is invalid.
         */
        unsigned level_a = 8U + (2U * wheel);
        unsigned level_b = level_a + 1U;

        /* Cytron Robo Pico truth table: LOW/LOW brake. Preload before
         * output-enable. */
        gpio_init(level_a);
        gpio_init(level_b);
        gpio_put(level_a, false);
        gpio_put(level_b, false);
        /* Never drive a motor-control pin if it is assigned to an encoder. */
        gpio_set_dir(level_a, (level_a != g_encoder_pins[0][0])
                                  && (level_a != g_encoder_pins[0][1])
                                  && (level_a != g_encoder_pins[1][0])
                                  && (level_a != g_encoder_pins[1][1]));
        gpio_set_dir(level_b, (level_b != g_encoder_pins[0][0])
                                  && (level_b != g_encoder_pins[0][1])
                                  && (level_b != g_encoder_pins[1][0])
                                  && (level_b != g_encoder_pins[1][1]));
    }

    motor_state_set(DRIVER_PWM_READY, false);
    gb_last_inhibited    = true;
    g_reversal_wait_mask = 0U;
    g_last_applied[0] = g_last_applied[1] = 0.0;
}

bool
motion_hal_init (const motion_config_t * p_config)
{
    unsigned  wheel;
    uint32_t  mask = 0U;
    float64_t divider;

    g_motor_config.max_output    = p_config->max_output;
    g_motor_config.reversal_time = p_config->reversal_time;
    g_motor_config.flags         = p_config->brake ? MOTOR_BRAKE_ENABLED : 0u;

    if (p_config->motor_sign[0] < 0)
    {
        g_motor_config.flags |= 2u;
    }

    if (p_config->motor_sign[1] < 0)
    {
        g_motor_config.flags |= 4u;
    }

    motion_hal_emergency_stop();

    if (!board_pins_valid())
    {
        return false;
    }

    /* Keep safe GPIO levels when settings have not been confirmed. */
    if ((p_config->pwm_hz < 1000U) || (p_config->pwm_hz > 20000U))
    {
        return false;
    }

    g_wrap  = 4095U;
    divider = 125000000.0 / ((float64_t)p_config->pwm_hz * (g_wrap + 1U));
    /* Never exceed the requested/max 20 kHz. */
    divider = ceil(divider * 16.0) / 16.0;

    if ((divider < 1.0) || (divider >= 256.0))
    {
        return false;
    }

    g_configured_divider_16 = (uint16_t)(divider * 16.0);

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        init_wheel(p_config, wheel, divider, &mask);
    }

    /* Board dispatcher owns IRQ 13; each registered handler owns only its pin
     * mask. */
    if (!board_gpio_irq_register(mask, encoder_irq))
    {
        return false;
    }

    enable_encoders(p_config);
#if defined(ROBO_ENCODER_DIAGNOSTICS_ONLY) || defined(ROBO_SAFE_MOTOR_TEST) \
    || defined(ROBO_ANGLE_TEST)
    /* This build never changes motor GPIOs to PWM, even if settings are edited.
     */
    return true;
#else
    if (!p_config->hardware_confirmed)
    {
        return true;
    }

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        gpio_set_function(g_motor_pins[wheel][0], GPIO_FUNC_PWM);
        gpio_set_function(g_motor_pins[wheel][1], GPIO_FUNC_PWM);
        pwm_set_enabled(pwm_gpio_to_slice_num(g_motor_pins[wheel][0]), true);
    }

    motor_state_set(DRIVER_PWM_READY, true);
    g_last_write_us = in_w(TIMER_TIMERAWL);
    return true;
#endif
}

encoder_sample_t
motion_hal_sample (void)
{
    encoder_sample_t sample;
    uint32_t         irq_state = save_and_disable_interrupts();

    sample.count[0]   = g_capture.count[0];
    sample.count[1]   = g_capture.count[1];
    sample.invalid[0] = g_capture.invalid[0];
    sample.invalid[1] = g_capture.invalid[1];
    sample.reliable   = motor_state_get(DRIVER_ENCODER_READY);
    restore_interrupts(irq_state);
    return sample;
}

void
motion_hal_encoder_diagnostics (encoder_diagnostics_t * p_diagnostics)
{
    unsigned wheel;
    unsigned channel;
    uint32_t irq_state = save_and_disable_interrupts();

    p_diagnostics->levels           = in_w(GPIO_IN);
    p_diagnostics->initialised      = motor_state_get(DRIVER_ENCODER_READY);
    p_diagnostics->irq_registered   = motor_state_get(DRIVER_IRQ_REGISTERED);
    p_diagnostics->nvic_enabled     = 0U != (in_w(0xe000e100U) & (1UL << 13U));
    p_diagnostics->motors_inhibited = !motor_state_get(DRIVER_PWM_READY);
#ifdef ROBO_SAFE_MOTOR_TEST
    p_diagnostics->motors_inhibited = !motor_test_active();
#endif
#ifdef ROBO_ANGLE_TEST
    p_diagnostics->motors_inhibited = !angle_test_active();
#endif
    p_diagnostics->sample.reliable = motor_state_get(DRIVER_ENCODER_READY);

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        p_diagnostics->sample.count[wheel]   = g_capture.count[wheel];
        p_diagnostics->sample.invalid[wheel] = g_capture.invalid[wheel];
        p_diagnostics->previous_ab[wheel]
            = encoder_capture_previous_ab(&g_capture, wheel);

        for (channel = 0; channel < ENCODER_CHANNELS; ++channel)
        {
            unsigned pin = g_encoder_pins[wheel][channel];

            p_diagnostics->pins[wheel][channel] = pin;
            p_diagnostics->edges[wheel][channel]
                = g_capture.edges[wheel][channel];
            p_diagnostics->irq_events[wheel][channel]
                = (uint8_t)((in_w(irq_reg(pin, 0x100U)) >> ((pin % 8U) * 4U))
                            & 15U);
            p_diagnostics->function[wheel][channel]
                = (uint8_t)(in_w(GPIO_CTRL(pin)) & 31U);
        }
    }

    restore_interrupts(irq_state);
}

void
motion_hal_motor_diagnostics (motor_diagnostics_t * p_diagnostics)
{
    if (NULL == p_diagnostics)
    {
        return;
    }

    p_diagnostics->ready                 = motor_state_get(DRIVER_PWM_READY);
    p_diagnostics->inhibited             = gb_last_inhibited;
    p_diagnostics->reversal_wait_mask    = g_reversal_wait_mask;
    p_diagnostics->enabled_mask          = 0U;
    p_diagnostics->mismatch_mask         = 0U;
    p_diagnostics->configured_top        = g_wrap;
    p_diagnostics->configured_divider_16 = g_configured_divider_16;
    p_diagnostics->cap_permille
        = ((isfinite(g_motor_config.max_output))
           && (g_motor_config.max_output > 0.0)
           && (g_motor_config.max_output <= 1.0))
              ? (int32_t)(g_motor_config.max_output * 1000.0)
              : 0;
    p_diagnostics->output_age_ms
        = (in_w(TIMER_TIMERAWL) - g_last_write_us) / 1000U;

    for (unsigned wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        uint32_t base
            = PWM_BASE
              + (pwm_gpio_to_slice_num(g_motor_pins[wheel][0]) * 0x14U);
        uint32_t compare = in_w(base + 12U);

        p_diagnostics->requested_permille[wheel]
            = diagnostic_permille(g_last_requested[wheel]);
        p_diagnostics->applied_permille[wheel]
            = diagnostic_permille(g_last_applied[wheel]);
        p_diagnostics->compare[wheel][0]          = (uint16_t)compare;
        p_diagnostics->compare[wheel][1]          = (uint16_t)(compare >> 16U);
        p_diagnostics->expected_compare[wheel][0] = g_last_compare[wheel][0];
        p_diagnostics->expected_compare[wheel][1] = g_last_compare[wheel][1];
        p_diagnostics->top[wheel]                 = (uint16_t)in_w(base + 16U);
        p_diagnostics->divider_16[wheel]          = (uint16_t)in_w(base + 4U);

        if (0U != (in_w(base) & 1U))
        {
            p_diagnostics->enabled_mask |= (uint8_t)(1U << wheel);
        }

        for (unsigned channel = 0U; channel < ENCODER_CHANNELS; ++channel)
        {
            p_diagnostics->pin_function[wheel][channel]
                = (uint8_t)(in_w(GPIO_CTRL(g_motor_pins[wheel][channel]))
                            & 31U);
        }

        if ((p_diagnostics->ready)
            && ((p_diagnostics->compare[wheel][0] != g_last_compare[wheel][0])
                || (p_diagnostics->compare[wheel][1]
                    != g_last_compare[wheel][1])
                || (p_diagnostics->top[wheel] != g_wrap)
                || (p_diagnostics->divider_16[wheel] != g_configured_divider_16)
                || (0U == (p_diagnostics->enabled_mask & (1U << wheel)))
                || (GPIO_FUNC_PWM != p_diagnostics->pin_function[wheel][0])
                || (GPIO_FUNC_PWM != p_diagnostics->pin_function[wheel][1])))
        {
            p_diagnostics->mismatch_mask |= (uint8_t)(1U << wheel);
        }
    }
}

void
motion_hal_write (const float64_t output[2], bool b_inhibited, float64_t now)
{
#if defined(ROBO_ENCODER_DIAGNOSTICS_ONLY) || defined(ROBO_SAFE_MOTOR_TEST) \
    || defined(ROBO_ANGLE_TEST)
    (void)output;
    (void)b_inhibited;
    (void)now;
    /* Startup left all motor outputs LOW; no drive path in this build. */
    return;
#else
    unsigned wheel;

    if (!motor_state_get(DRIVER_PWM_READY))
    {
        gb_last_inhibited = true;
        return;
    }

    gb_last_inhibited    = b_inhibited;
    g_reversal_wait_mask = 0U;

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        float64_t requested
            = isfinite(output[wheel])
                  ? fmax(-g_motor_config.max_output,
                         fmin(g_motor_config.max_output, output[wheel]))
                  : 0.0;
        float64_t value     = b_inhibited ? 0.0 : requested;
        int       direction = (value > 0.0) ? 1 : ((value < 0.0) ? -1 : 0);
        uint16_t  level_a   = 0U;
        uint16_t  level_b   = 0U;

        if ((0 != direction) && (0 != motor_direction_get(wheel))
            && (direction != motor_direction_get(wheel))
            && ((!motor_is_neutral(wheel))
                || ((now - g_neutral_since[wheel])
                    < g_motor_config.reversal_time)))
        {
            value     = 0.0;
            direction = 0;
            g_reversal_wait_mask |= (uint8_t)(1U << wheel);
        }

        if (0 == direction)
        {
            if (!motor_is_neutral(wheel))
            {
                g_neutral_since[wheel] = now;
            }

            motor_neutral_set(wheel, true);

            if ((0u == ((uint32_t)g_motor_config.flags & MOTOR_BRAKE_ENABLED)))
            {
                level_a = (uint16_t)(g_wrap + 1U);
                level_b = level_a;
            }
        }
        else
        {
            uint16_t duty      = (uint16_t)(fabs(value) * (g_wrap + 1U));
            uint32_t irq_state = save_and_disable_interrupts();

            encoder_capture_set_direction(&g_capture, wheel, direction);
            restore_interrupts(irq_state);
            motor_direction_set(wheel, direction);
            motor_neutral_set(wheel, false);

            if ((direction * motor_sign_get(wheel)) > 0)
            {
                level_a = duty;
            }
            else
            {
                level_b = duty;
            }
        }

        /* Both compare values share a single slice register write. */
        pwm_set_both_levels(pwm_gpio_to_slice_num(g_motor_pins[wheel][0]),
                            level_a, level_b);
        g_last_requested[wheel]  = requested;
        g_last_applied[wheel]    = value;
        g_last_compare[wheel][0] = level_a;
        g_last_compare[wheel][1] = level_b;
    }

    g_last_write_us = in_w(TIMER_TIMERAWL);
#endif
}

float64_t
motion_hal_time (void)
{
    return (float64_t)time_us_64() / 1000000.0;
}

void
motion_hal_console_init (void)
{ /* Template starts the console service. */
}

int
motion_hal_console_read (void)
{
#if TM_CONSOLE_USB_CDC
    extern int tm_usb_console_read(void);
    return tm_usb_console_read();
#else
    if (0U != (in_w(0x40034018U) & (1U << 4U)))
    {
        return -1;
    }

    return (int)(in_w(0x40034000U) & 255U);
#endif
}

static bool
motor_state_get (uint8_t mask)
{
    return (0u != ((uint32_t)g_driver_flags & mask));
}

static void
motor_state_set (uint8_t mask, bool b_value)
{
    uint32_t flags = g_driver_flags;

    flags          = b_value ? (flags | mask) : (flags & ~(uint32_t)mask);
    g_driver_flags = (uint8_t)flags;
}

static bool
motor_is_neutral (uint32_t wheel)
{
    return (0u != ((uint32_t)g_wheel_flags & (UINT32_C(1) << wheel)));
}

static void
motor_neutral_set (uint32_t wheel, bool b_value)
{
    uint32_t mask  = UINT32_C(1) << wheel;
    uint32_t flags = g_wheel_flags;

    flags         = b_value ? (flags | mask) : (flags & ~mask);
    g_wheel_flags = (uint8_t)flags;
}

static int32_t
motor_direction_get (uint32_t wheel)
{
    uint32_t encoded = ((uint32_t)g_wheel_flags >> (2u + (2u * wheel))) & 3u;

    return (1u == encoded) ? 1 : ((2u == encoded) ? -1 : 0);
}

static void
motor_direction_set (uint32_t wheel, int32_t direction)
{
    uint32_t shift   = 2u + (2u * wheel);
    uint32_t encoded = (direction > 0) ? 1u : ((direction < 0) ? 2u : 0u);
    uint32_t flags   = g_wheel_flags;

    g_wheel_flags
        = (uint8_t)((flags & ~(UINT32_C(3) << shift)) | (encoded << shift));
}

static int32_t
motor_sign_get (uint32_t wheel)
{
    return (0u
            != ((uint32_t)g_motor_config.flags & (UINT32_C(1) << (wheel + 1u))))
               ? -1
               : 1;
}

static void
encoder_irq (void)
{
    uint32_t pending[4];
    uint32_t any = 0U;
    unsigned bank;
    uint32_t levels;

    for (bank = 0U; bank < GPIO_IRQ_BANKS; ++bank)
    {
        pending[bank] = in_w(IO_BANK0_BASE + 0x120U + (bank * 4U))
                        & g_encoder_event_mask[bank];
        any |= pending[bank];

        if (0U != pending[bank])
        {
            out_w(IO_BANK0_BASE + 0xf0U + (bank * 4U), pending[bank]);
        }
    }

    if (0U == any)
    {
        return;
    }

    /* Acknowledge only owned flags, then take one coherent A/B level snapshot.
     */
    levels = in_w(GPIO_IN);
    encoder_capture_events(&g_capture, pending, levels);
}

/* PWM and encoder pins of one wheel, outputs off.  Adds its pins to the mask.
 */
static void
init_wheel (const motion_config_t * p_config, unsigned wheel, float64_t divider,
            uint32_t * p_mask)
{
    unsigned channel;

    pwm_config_t pwm = pwm_get_default_config();

    pwm_config_set_wrap(&pwm, g_wrap);
    pwm_config_set_clkdiv(&pwm, (float32_t)divider);
    pwm_init(pwm_gpio_to_slice_num(g_motor_pins[wheel][0]), &pwm, false);
    pwm_set_both_levels(pwm_gpio_to_slice_num(g_motor_pins[wheel][0]), 0U, 0U);
    g_last_compare[wheel][0] = g_last_compare[wheel][1] = 0U;
    g_last_requested[wheel] = g_last_applied[wheel] = 0.0;

    for (channel = 0; channel < ENCODER_CHANNELS; ++channel)
    {
        unsigned pin = g_encoder_pins[wheel][channel];

        if ((1U == channel) && (!p_config->quadrature[wheel]))
        {
            /* Nothing to do. */
        }
        else
        {
            gpio_init(pin);
            gpio_set_dir(pin, false);
            /* External electrical interface must be confirmed. */
            gpio_disable_pulls(pin);
            gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                 false);
            g_encoder_event_mask[pin / 8U]
                |= (p_config->quadrature[wheel] ? 12UL : 8UL)
                   << ((pin % 8U) * 4U);
            *p_mask |= 1UL << pin;
        }
    }

    g_neutral_since[wheel] = motion_hal_time();
}

/* Start counting encoder edges from a clean state. */
static void
enable_encoders (const motion_config_t * p_config)
{
    unsigned wheel;
    uint32_t irq_state;

    motor_state_set(DRIVER_IRQ_REGISTERED, true);
    irq_state = save_and_disable_interrupts();

    for (wheel = 0U; wheel < GPIO_IRQ_BANKS; ++wheel)
    {
        out_w(IO_BANK0_BASE + 0xf0U + (wheel * 4U),
              g_encoder_event_mask[wheel]);
    }

    encoder_capture_init(&g_capture, g_encoder_pins, p_config->quadrature,
                         p_config->encoder_sign, in_w(GPIO_IN));

    for (wheel = 0; wheel < MOTION_WHEELS; ++wheel)
    {
        gpio_set_irq_enabled(g_encoder_pins[wheel][0],
                             p_config->quadrature[wheel]
                                 ? GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL
                                 : GPIO_IRQ_EDGE_RISE,
                             true);

        if (p_config->quadrature[wheel])
        {
            gpio_set_irq_enabled(g_encoder_pins[wheel][1],
                                 GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
        }
    }

    motor_state_set(DRIVER_ENCODER_READY, true);
    restore_interrupts(irq_state);
}

static int32_t
diagnostic_permille (float64_t value)
{
    int32_t result = 0;

    if (!isfinite(value))
    {
        result = INT32_MIN;
    }
    else
    {
        result = (int32_t)(fmax(-1.0, fmin(1.0, value)) * 1000.0);
    }

    return result;
}

/*** end of file ***/
