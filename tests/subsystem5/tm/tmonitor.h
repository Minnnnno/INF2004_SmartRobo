#ifndef S5_TEST_TMONITOR_H
#define S5_TEST_TMONITOR_H
#include <stdint.h>
int32_t tm_printf(const uint8_t *format, ...);
int32_t tm_sprintf(uint8_t *buffer, const uint8_t *format, ...);
int32_t tm_snprintf(uint8_t *buffer, int32_t capacity, const uint8_t *format, ...);
#endif
