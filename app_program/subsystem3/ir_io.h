/* Subsystem 3 IR sensor pins: shared by the line task, the barcode reader, the IR monitor and the
 * shuttle test. Self-contained: pulls in the robot HAL, or the host tests' fake HAL. */
#ifndef ROBO_IR_IO_H
#define ROBO_IR_IO_H
#include <stdbool.h>
#include <stdint.h>
#ifdef ROBO_LINE_TEST_HOST
#include "line_test_fake_hal.h"
#else
#include "../subsystem2/template_hal.h"
#endif

/* Plain digital input, pulls off (the IR modules drive the line). */
static inline void ir_input_init(unsigned pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, false);
    gpio_disable_pulls(pin);
}

#ifndef ROBO_LINE_TEST_HOST
/* RP2040 ADC (registers, not the SDK). Channel = pin - 26: GP26 = ADC0, GP27 = ADC1, GP28 = ADC2. */
#define IR_ADC_CS     (0x4004C000U + 0x00U)
#define IR_ADC_RESULT (0x4004C000U + 0x04U)
#define IR_ADC_EN     (1UL << 0U)
#define IR_ADC_START  (1UL << 2U)
#define IR_ADC_READY  (1UL << 8U)
#define IR_ADC_SPINS  100000U /* Register polls before giving up: a conversion takes ~2 us (~250 cycles). */

/* Polls 'reg' until any of 'mask' is set; false after IR_ADC_SPINS polls (a dead peripheral never hangs). */
static inline bool ir_reg_wait(uint32_t reg, uint32_t mask)
{
    for (uint32_t i = 0U; i < IR_ADC_SPINS; ++i) {
        if ((in_w(reg) & mask) != 0U) { return true; }
    }
    return false;
}

/* Takes the ADC out of reset, turns it on, and makes 'pin' an analog input (digital input and pulls off).
 * False if the ADC never became ready. */
static inline bool ir_adc_init(unsigned analog_pin)
{
    clr_w(RESETS_RESET, RESETS_RESET_ADC);
    if (!ir_reg_wait(RESETS_RESET_DONE, RESETS_RESET_ADC)) { return false; }
    out_w(GPIO_CTRL(analog_pin), GPIO_CTRL_FUNCSEL_NULL);
    out_w(GPIO(analog_pin), GPIO_OD);
    out_w(IR_ADC_CS, IR_ADC_EN);
    return ir_reg_wait(IR_ADC_CS, IR_ADC_READY);
}

/* Average of 'count' conversions (about 2 us each) on 'pin', 0..4095; a conversion that never finishes
 * ends the average early (0 if none finished). */
static inline unsigned ir_adc_read(unsigned pin, unsigned count)
{
    uint32_t sum = 0U, done = 0U, input = (uint32_t)(pin - 26U) << 12U;
    for (unsigned i = 0U; i < count; ++i) {
        out_w(IR_ADC_CS, IR_ADC_EN | input | IR_ADC_START);
        if (!ir_reg_wait(IR_ADC_CS, IR_ADC_READY)) { break; }
        sum += in_w(IR_ADC_RESULT) & 0xFFFU;
        ++done;
    }
    return (done == 0U) ? 0U : (unsigned)(sum / done);
}
#endif
#endif
