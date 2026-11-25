#include "line_test_fake_hal.h"
#include <stdarg.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); exit(1); } } while (0)
typedef struct {
    bool left, right;
    bool finish_before_read;
    bool reject, serial_stop;
    unsigned expected_starts, expected_stops;
} reading_t;
static const reading_t readings[] = {
    {false, false, false, false, false, 0U, 0U},
    {true, false, false, false, false, 1U, 0U}, /* Left alone starts forward. */
    {false, true, false, false, false, 2U, 0U}, /* Right alone keeps moving. */
    {true, true, false, false, false, 3U, 0U},
    {true, true, false, false, false, 4U, 0U},
    {true, true, true, true, false, 5U, 0U}, /* Watchdog, busy rejection. */
    {true, true, false, false, false, 6U, 0U},
    {true, true, false, false, true, 6U, 1U}, /* Serial stop pauses sensors. */
    {false, true, false, false, false, 6U, 1U}, /* One HIGH does not clear pause. */
    {true, false, false, false, false, 6U, 1U},
    {false, false, false, false, false, 6U, 1U}, /* Both LOW re-arm. */
    {true, false, false, false, false, 7U, 1U},
    {false, true, false, false, false, 8U, 1U},
    {false, false, false, false, false, 8U, 2U}, /* Both LOW stop. */
    {true, true, false, true, false, 9U, 2U}, /* Busy/disabled rejection. */
    {true, true, false, false, false, 10U, 2U},
    {false, false, false, false, false, 10U, 3U},
    {false, true, false, false, false, 11U, 3U},
    {false, false, false, false, false, 11U, 4U},
};
static unsigned checks, row, reads, starts, stops, pins, directions, pulls;
static bool active, reject_start;
static jmp_buf done;
void gpio_init(unsigned pin)
{ CHECK(pin == 26U + pins); ++pins; }
void gpio_set_dir(unsigned pin, bool output)
{ CHECK(pin == 26U + directions && !output); ++directions; }
void gpio_disable_pulls(unsigned pin)
{ CHECK(pin == 26U + pulls); ++pulls; }
bool gpio_get(unsigned pin)
{
    CHECK(pin == 26U + reads); ++reads;
    if (readings[row].finish_before_read) { active = false; }
    reject_start = readings[row].reject;
    if (pin == 26U && readings[row].serial_stop) {
        line_forward_pause();
        CHECK(active); active = false; ++stops;
    }
    return pin == 26U ? readings[row].left : readings[row].right;
}
bool motor_test_active(void) { return active; }
bool motor_test_sensor_forward(void)
{
    ++starts;
    if (reject_start) { return false; }
    active = true; return true;
}
void motor_test_stop(void) { CHECK(active); active = false; ++stops; }
int tm_printf(const UB *format, ...)
{
    va_list arguments;
    (void)format;
    va_start(arguments, format);
    CHECK(va_arg(arguments, unsigned) == readings[row].left);
    CHECK(va_arg(arguments, unsigned) == readings[row].right);
    CHECK(va_arg(arguments, unsigned) == (unsigned)(readings[row].left || readings[row].right));
    CHECK(va_arg(arguments, unsigned) == (unsigned)active);
    va_end(arguments);
    return 0;
}
int tk_dly_tsk(unsigned milliseconds)
{
    if (milliseconds == 2000U) { CHECK(pins == 2U && directions == 2U && pulls == 2U && !active); return 0; }
    CHECK(milliseconds == 100U);
    CHECK(reads == 2U); reads = 0U;
    CHECK(starts == readings[row].expected_starts);
    CHECK(stops == readings[row].expected_stops);
    ++row;
    if (row == sizeof(readings) / sizeof(readings[0])) { longjmp(done, 1); }
    return 0;
}
int tk_slp_tsk(int timeout) { (void)timeout; CHECK(false); return 0; }
int main(void)
{
    if (setjmp(done) == 0) { line_task(0, NULL); }
    CHECK(!active);
    printf("PASS: %u line-sensor task checks\n", checks);
    return 0;
}
