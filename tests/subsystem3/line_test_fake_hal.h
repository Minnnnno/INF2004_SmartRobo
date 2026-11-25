#ifndef LINE_TEST_FAKE_HAL_H
#define LINE_TEST_FAKE_HAL_H
#include <stdbool.h>
#include <stdint.h>
typedef int INT;
typedef unsigned char UB;
#define TMO_FEVR (-1)
void gpio_init(unsigned pin);
void gpio_set_dir(unsigned pin, bool output);
void gpio_disable_pulls(unsigned pin);
bool gpio_get(unsigned pin);
int tk_dly_tsk(unsigned milliseconds);
int tk_slp_tsk(int timeout);
int tm_printf(const UB *format, ...);
void line_task(INT code, void *context);
void line_forward_pause(void);
#endif
