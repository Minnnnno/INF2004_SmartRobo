#ifndef MOTOR_TEST_FAKE_HAL_H
#define MOTOR_TEST_FAKE_HAL_H
#include <stdint.h>
#include <stdbool.h>
typedef unsigned UINT;
typedef void (*FP)(UINT);
typedef struct { unsigned intatr; FP inthdr; } T_DINT;
typedef struct { uint16_t wrap; float divider; } pwm_config;
#define TA_HLNG 1U
#define E_OK 0
#define GPIO_FUNC_PWM 4U
#define GPIO_IRQ_EDGE_FALL 4U
#define GPIO_IRQ_EDGE_RISE 8U
#define TIMER_ALARM3 1U
#define TIMER_ARMED 2U
#define TIMER_INTR 3U
#define TIMER_INTE 4U
uint32_t save_and_disable_interrupts(void);
void restore_interrupts(uint32_t state);
uint64_t time_us_64(void);
pwm_config pwm_get_default_config(void);
void pwm_config_set_wrap(pwm_config *config, uint16_t wrap);
void pwm_config_set_clkdiv(pwm_config *config, float divider);
void pwm_init(unsigned slice, const pwm_config *config, bool enable);
void pwm_set_both_levels(unsigned slice, uint16_t a, uint16_t b);
void pwm_set_enabled(unsigned slice, bool enabled);
void gpio_set_function(unsigned pin, unsigned function);
void gpio_init(unsigned pin);
void gpio_set_dir(unsigned pin, bool output);
void gpio_pull_up(unsigned pin);
bool gpio_get(unsigned pin);
bool board_gpio_irq_register(uint32_t mask, void (*handler)(void));
void gpio_set_irq_enabled(unsigned pin, uint32_t events, bool enable);
uint32_t gpio_get_irq_event_mask(unsigned pin);
void gpio_acknowledge_irq(unsigned pin, uint32_t events);
void out_w(unsigned address, uint32_t value);
void set_w(unsigned address, uint32_t value);
void ClearInt(unsigned number);
void EnableInt(unsigned number, int priority);
int tk_def_int(unsigned number, const T_DINT *definition);
#endif
