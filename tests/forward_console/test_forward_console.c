/* Run the real serial parser; motor primitives have separate register tests. */
#include "robot_rtos.h"
#include "robot.h"
#include "motion_hal.h"
#include "motor_test.h"
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void manual_console(void);
static jmp_buf g_done;
static char const *g_input;
static size_t g_position;
static unsigned g_idle_ticks;
static unsigned g_starts;
static unsigned g_attempts;
static unsigned g_stops;
static unsigned g_shutdowns;
static unsigned g_sensor_pauses;
static unsigned g_checks;
static int g_direction;
static unsigned g_wheel;
static bool g_active;
static bool g_init_ok;
#ifdef ROBO_BATTERY_FORWARD_TEST
static unsigned g_button_polls;
static unsigned g_button_diagnostics;
#endif

static void check(bool condition, char const *name)
{
    g_checks++;
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", name);
        exit(EXIT_FAILURE);
    }
}

void motion_hal_console_init(void) {}
int motion_hal_console_read(void)
{
    unsigned char value = (unsigned char)g_input[g_position];
    if (value == 0U) { return -1; }
    g_position++;
    return value == 1U ? -2 : (int)value;
}
ER tk_dly_tsk(UW delay)
{
    check(delay == 20U, "console yields every polling cycle");
    if (g_input[g_position] == '\0')
    {
        g_idle_ticks++;
        if (g_idle_ticks == 2U) { longjmp(g_done, 1); }
    }
    return E_OK;
}
INT tm_printf(UB const *format, ...) { (void)format; return 0; }
void tm_putstring(UB const *text) { (void)text; }
encoder_sample_t motion_hal_sample(void)
{
    encoder_sample_t sample = {0};
    sample.reliable = true;
    return sample;
}
void motion_hal_encoder_diagnostics(encoder_diagnostics_t *diagnostics)
{ memset(diagnostics, 0, sizeof(*diagnostics)); }
bool motion_get_status(robot_status_t *status)
{ memset(status, 0, sizeof(*status)); return true; }
void motion_stop(void) {}
void motion_request_shutdown(void) { g_shutdowns++; }
bool motor_test_init(void) { g_active = false; return g_init_ok; }
bool motor_test_active(void) { return g_active; }
void motor_test_poll(void) {}
bool motor_test_result(motor_test_result_t *result)
{ (void)result; return false; }
void motor_test_stop(void) { g_active = false; g_stops++; }
void line_forward_pause(void) { g_sensor_pauses++; }
void motor_test_shutdown(void) { motor_test_stop(); g_init_ok = false; }
bool motor_test_start(unsigned wheel, int direction)
{
    g_attempts++;
    if (!g_init_ok || g_active) { return false; }
    g_active = true;
    g_wheel = wheel;
    g_direction = direction;
    g_starts++;
    return true;
}
bool motor_test_start_pair(int direction)
{ return motor_test_start(2U, direction); }
#ifdef ROBO_BATTERY_FORWARD_TEST
void motor_test_poll_buttons(void) { g_button_polls++; }
bool motor_test_get_buttons(motor_test_button_diag_t *d)
{
    *d = (motor_test_button_diag_t){.start_high=true, .stop_high=true,
        .ready=g_init_ok, .active=g_active, .polls=g_button_polls};
    g_button_diagnostics++;
    return true;
}
#endif

static void run(char const *input, bool init_ok)
{
    g_input = input;
    g_position = 0U;
    g_idle_ticks = 0U;
    g_starts = 0U;
    g_attempts = 0U;
    g_stops = 0U;
    g_shutdowns = 0U;
    g_sensor_pauses = 0U;
    g_direction = 0;
    g_wheel = 0U;
    g_active = false;
    g_init_ok = init_ok;
#ifdef ROBO_BATTERY_FORWARD_TEST
    g_button_polls = 0U;
    g_button_diagnostics = 0U;
#endif
    if (setjmp(g_done) == 0) { manual_console(); }
}

int main(void)
{
#ifdef ROBO_BATTERY_FORWARD_TEST
    run("", true); /* No incoming serial bytes or host connection required. */
    check(g_starts == 0U && !g_active && g_button_polls >= 2U,
          "button profile polls buttons without serial input and never starts automatically");
    run("status\n", true);
    check(g_starts == 0U, "status never starts movement");
    check(g_button_diagnostics == 1U, "status includes raw button diagnostics");
    run("buttons\nbuttons\n", true);
    check(g_button_diagnostics == 2U && g_starts == 0U && g_stops == 0U,
          "buttons command reports input levels without changing motor state");
    run("forward\nstop\n", true);
    check(g_starts == 1U && !g_active && g_stops == 1U,
          "serial stop cancels movement without restarting it");
    run("", false);
    check(g_starts == 0U && !g_active && g_attempts == 0U,
          "failed initialization never starts movement");
#else
    char oversized[128];
    run("", true);
    check(g_starts == 0U, "startup does not drive motors");
    run("status\nencdiag\nusbdiag\nARM\n", true);
    check(g_starts == 0U, "diagnostics and ARM do not drive motors");
    run("forward\r\n", true);
    check(g_starts == 1U && g_wheel == 2U && g_direction == 1,
          "forward selects both motors once; CRLF is accepted");
    run("backward\n", true);
    check(g_starts == 1U && g_direction == -1, "backward direction");
    run("left_forward\n", true);
    check(g_starts == 1U && g_wheel == 0U && g_direction == 1,
          "left motor command");
    check(g_sensor_pauses == 1U, "manual calibration disables IR restart");
    run("right_reverse\n", true);
    check(g_starts == 1U && g_wheel == 1U && g_direction == -1,
          "right motor command");
    run("forward\nforward\n", true);
    check(g_attempts == 2U && g_starts == 1U, "busy start is not queued");
    run("forward\nstop\n", true);
    check(g_starts == 1U && g_stops == 1U && !g_active && g_sensor_pauses == 2U,
          "stop reaches the test actuator");
    run("forward\nshutdown\nforward\n", true);
    check(g_starts == 1U && g_stops == 1U && g_shutdowns == 1U && !g_active,
          "shutdown stops and blocks subsequent movement");
    run("forwardXYZ\nFORWARD\nrelease_checked\n", true);
    check(g_starts == 0U, "unknown commands cannot start a pulse");
    run("forward\n", false);
    check(g_starts == 0U, "initialisation failure blocks the actuator");
    memset(oversized, 'x', 80U);
    strcpy(&oversized[80], "forward\n");
    run(oversized, true);
    check(g_starts == 0U && g_stops == 1U,
          "oversized input discards its trailing command");
    strcpy(&oversized[80], "\nforward\n");
    run(oversized, true);
    check(g_starts == 1U, "parser recovers after a new line");
    run("\001forward\n", true);
    check(g_starts == 0U && g_stops == 1U, "RX loss discards partial command");
    run("\001\nforward\n", true);
    check(g_starts == 1U && g_stops == 1U, "RX loss recovery needs newline");
    run("forward", true);
    check(g_starts == 0U, "unterminated command never drives motors");
#endif
    printf("PASS: %u serial command checks\n", g_checks);
    return EXIT_SUCCESS;
}
