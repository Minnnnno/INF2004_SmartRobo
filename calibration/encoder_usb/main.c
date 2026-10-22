/* Passive encoder calibration only. No motor activation or RTOS in this image. */
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/structs/io_bank0.h"
#include "encoder_capture.h"

static volatile encoder_capture_t capture;
static const unsigned pins[2][2] = {{4U, 6U}, {5U, 7U}};
static const bool quadrature[2] = {true, true};
static const int signs[2] = {1, 1}; /* Preserve raw direction for calibration. */
static uint32_t encoder_mask;
static uint32_t replies;

static void encoder_irq(void)
{
    uint32_t pending[4] = {io_bank0_hw->proc0_irq_ctrl.ints[0] & encoder_mask, 0U, 0U, 0U};
    if (pending[0] == 0U) { return; }
    io_bank0_hw->intr[0] = pending[0]; /* Acknowledge only encoder-owned latches. */
    encoder_capture_events(&capture, pending, gpio_get_all());
}

static void configure_inputs(void)
{
    for (unsigned wheel = 0U; wheel < 2U; ++wheel) {
        for (unsigned channel = 0U; channel < 2U; ++channel) {
            unsigned pin = pins[wheel][channel];
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_IN);
            gpio_disable_pulls(pin);
            gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
            encoder_mask |= 12U << (pin * 4U);
        }
    }
    uint32_t state = save_and_disable_interrupts();
    io_bank0_hw->intr[0] = encoder_mask;
    encoder_capture_init(&capture, pins, quadrature, signs, gpio_get_all());
    irq_set_exclusive_handler(IO_IRQ_BANK0, encoder_irq);
    irq_set_priority(IO_IRQ_BANK0, 0x40U);
    for (unsigned wheel = 0U; wheel < 2U; ++wheel) {
        for (unsigned channel = 0U; channel < 2U; ++channel) {
            gpio_set_irq_enabled(pins[wheel][channel], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
        }
    }
    irq_set_enabled(IO_IRQ_BANK0, true);
    restore_interrupts(state);
}

static void report(bool detailed)
{
    encoder_capture_t snapshot;
    uint32_t levels;
    uint32_t state = save_and_disable_interrupts();
    snapshot = capture;
    levels = gpio_get_all();
    restore_interrupts(state);
    printf("firmware=%s reply=%lu uptime_ms=%lu read_only=1 motor_pins_low=1\n",
           CALIBRATION_BUILD_ID, (unsigned long)++replies,
           (unsigned long)(time_us_64() / 1000U));
    printf("counts=%s%lu,%s%lu raw_counts=%lu,%lu invalid=%lu,%lu\n",
           snapshot.count[0] > INT32_MAX ? "-" : "",
           (unsigned long)(snapshot.count[0] > INT32_MAX ? 0U - snapshot.count[0] : snapshot.count[0]),
           snapshot.count[1] > INT32_MAX ? "-" : "",
           (unsigned long)(snapshot.count[1] > INT32_MAX ? 0U - snapshot.count[1] : snapshot.count[1]),
           (unsigned long)snapshot.count[0], (unsigned long)snapshot.count[1],
           (unsigned long)snapshot.invalid[0], (unsigned long)snapshot.invalid[1]);
    if (detailed) {
        for (unsigned wheel = 0U; wheel < 2U; ++wheel) {
            printf("%s A=GP%u level=%lu edges=%lu B=GP%u level=%lu edges=%lu prev_AB=%u\n",
                   wheel == 0U ? "left" : "right", pins[wheel][0],
                   (unsigned long)((levels >> pins[wheel][0]) & 1U), (unsigned long)snapshot.edges[wheel][0],
                   pins[wheel][1], (unsigned long)((levels >> pins[wheel][1]) & 1U),
                   (unsigned long)snapshot.edges[wheel][1], (unsigned)encoder_capture_previous_ab(&snapshot, wheel));
        }
    }
    fflush(stdout);
}

int main(void)
{
    /* Preload LOW before enabling outputs. This image never selects motor PWM. */
    for (unsigned pin = 8U; pin <= 11U; ++pin) {
        gpio_init(pin);
        gpio_put(pin, false);
        gpio_set_dir(pin, GPIO_OUT);
    }
    configure_inputs();
    stdio_init_all();
    char line[32];
    unsigned used = 0U;
    bool discard = false;
    for (;;) {
        int input = getchar_timeout_us(0U);
        if (input == PICO_ERROR_TIMEOUT) { sleep_ms(1U); continue; }
        if (input == '\n' || input == '\r') {
            if (discard) { discard = false; used = 0U; continue; }
            line[used] = '\0';
            used = 0U;
            if (strcmp(line, "status") == 0) { report(false); }
            else if (strcmp(line, "encdiag") == 0) { report(true); }
            else if (line[0] != '\0') { puts("Read-only: status | encdiag. No movement commands."); }
        } else if (input >= 32 && input <= 126) {
            if (!discard && used + 1U < sizeof(line)) { line[used++] = (char)input; }
            else { discard = true; used = 0U; }
        }
    }
}
