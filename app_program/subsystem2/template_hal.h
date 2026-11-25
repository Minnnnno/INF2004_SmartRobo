#ifndef ROBO_TEMPLATE_HAL_H
#define ROBO_TEMPLATE_HAL_H
/* RP2040 register adapter using the template's genuine micro T-Kernel BSP.
 * Kernel owns startup and interrupt vectors; no Pico SDK runtime is linked. */
#include "robot_rtos.h"
#include "../../include/float_types.h"
#include <stdint.h>
#include <stdbool.h>
_Static_assert(TMCLK_MHz == 125,
               "Recalculate motion PWM divider for a different kernel clock");

typedef struct
{
    uint16_t  wrap;
    float32_t divider;
} pwm_config;

bool board_gpio_irq_register(uint32_t mask, void (*p_handler)(void));
static inline uint32_t
save_and_disable_interrupts (void)
{
    UINT state;

    DI(state);
    __asm__ volatile("" ::: "memory");
    return state;
}

static inline void
restore_interrupts (uint32_t state)
{
    __asm__ volatile("" ::: "memory");
    EI(state);
    __asm__ volatile("" ::: "memory");
}

static inline void
gpio_init (unsigned pin)
{
    out_w(GPIO_OE_CLR, 1UL << pin);
    out_w(GPIO_OUT_CLR, 1UL << pin);
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO(pin), GPIO_IE | GPIO_DRIVE_4MA | GPIO_SHEMITT);
}

static inline void
gpio_put (unsigned pin, bool b_high)
{
    out_w(b_high ? GPIO_OUT_SET : GPIO_OUT_CLR, 1UL << pin);
}

static inline void
gpio_set_dir (unsigned pin, bool b_output)
{
    out_w(b_output ? GPIO_OE_SET : GPIO_OE_CLR, 1UL << pin);
}

static inline bool
gpio_get (unsigned pin)
{
    return 0U != (in_w(GPIO_IN) & (1UL << pin));
}

static inline void
gpio_disable_pulls (unsigned pin)
{
    clr_w(GPIO(pin), GPIO_PUE | GPIO_PDE);
}

static inline void
gpio_pull_up (unsigned pin)
{
    clr_w(GPIO(pin), GPIO_PDE);
    set_w(GPIO(pin), GPIO_PUE);
}

static inline void
gpio_set_function (unsigned pin, unsigned function)
{
    out_w(GPIO_CTRL(pin), function);
}
#define GPIO_FUNC_PWM      GPIO_CTRL_FUNCSEL_PWM
#define GPIO_IRQ_EDGE_FALL 4U
#define GPIO_IRQ_EDGE_RISE 8U
static inline UW
irq_reg (unsigned pin, UW base)
{
    return IO_BANK0_BASE + base + ((pin / 8U) * 4U);
}

static inline uint32_t
gpio_get_irq_event_mask (unsigned pin)
{
    return (in_w(irq_reg(pin, 0x120U)) >> ((pin % 8U) * 4U)) & 15U;
}

static inline void
gpio_acknowledge_irq (unsigned pin, uint32_t events)
{
    out_w(irq_reg(pin, 0xf0U), events << ((pin % 8U) * 4U));
}

static inline void
gpio_set_irq_enabled (unsigned pin, uint32_t events, bool b_enable)
{
    UW bits = events << ((pin % 8U) * 4U);

    /* Caller clears old latches before seeding quadrature state. Do not discard
     * an edge that arrives between that sample and interrupt enable. */
    if (b_enable)
    {
        set_w(irq_reg(pin, 0x100U), bits);
    }
    else
    {
        clr_w(irq_reg(pin, 0x100U), bits);
    }
}

static inline unsigned
pwm_gpio_to_slice_num (unsigned pin)
{
    return (pin >> 1U) & 7U;
}

static inline pwm_config
pwm_get_default_config (void)
{
    pwm_config config = {4095U, 1.0f};
    return config;
}

static inline void
pwm_config_set_wrap (pwm_config * p_config, uint16_t wrap)
{
    p_config->wrap = wrap;
}

static inline void
pwm_config_set_clkdiv (pwm_config * p_config, float32_t divider)
{
    p_config->divider = divider;
}

static inline void
pwm_init (unsigned slice, const pwm_config * p_config, bool b_enable)
{
    UW base = PWM_BASE + (slice * 0x14U);

    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    out_w(base, 0U);
    out_w(base + 4U, (UW)(p_config->divider * 16.0f));
    out_w(base + 8U, 0U);
    out_w(base + 12U, 0U);
    out_w(base + 16U, p_config->wrap);
    out_w(base, b_enable ? 1U : 0U);
}

static inline void
pwm_set_both_levels (unsigned slice, uint16_t level_a, uint16_t level_b)
{
    out_w(PWM_BASE + (slice * 0x14U) + 12U, ((UW)level_b << 16U) | level_a);
}

static inline void
pwm_set_enabled (unsigned slice, bool b_enabled)
{
    out_w(PWM_BASE + (slice * 0x14U), b_enabled ? 1U : 0U);
}

static inline uint64_t
time_us_64 (void)
{
    UW high;
    UW low;
    UW after;

    do
    {
        high  = in_w(TIMER_TIMERAWH);
        low   = in_w(TIMER_TIMERAWL);
        after = in_w(TIMER_TIMERAWH);
    } while (high != after);
    return ((uint64_t)high << 32U) | low;
}
#endif

/*** end of file ***/
