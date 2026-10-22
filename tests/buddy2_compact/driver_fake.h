/** @file driver_fake.h
 * @brief Host register boundary for the real Buddy 2 motor driver.
 */
#ifndef BUDDY2_DRIVER_FAKE_H
#define BUDDY2_DRIVER_FAKE_H
#include "../motor_test_fake_hal.h"
#define IO_BANK0_BASE 0x40014000u
#define GPIO_IN 0xd0000004u
#define GPIO_CTRL(PIN) (0x40014004u + 8u * (PIN))
uint32_t in_w(uint32_t address);
uint32_t irq_reg(unsigned pin, uint32_t base);
void gpio_put(unsigned pin, bool b_high);
void gpio_disable_pulls(unsigned pin);
unsigned pwm_gpio_to_slice_num(unsigned pin);
int tm_usb_console_read(void);
#endif
