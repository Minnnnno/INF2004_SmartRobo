/* The real terrain_task() compiled as the magnetometer calibration image
 * (ROBO_MAG_CAL): fake robot whose heading follows the requested pivot, and a
 * fake magnetometer whose field turns with it (offset -123/+48, Y gain 1/1.06).
 * Checks the left-then-right pivot, the stop, the printed constants, and the
 * failure paths. Each scenario ends by long-jumping out of tk_dly_tsk().
 */
#define ROBO_MAG_CAL 1
#include "../../app_program/subsystem4/terrain_task.c"
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static jmp_buf g_finish;
static uint32_t g_clock, g_start, g_end;
static unsigned g_checks;
static double g_heading_deg;   /* Fake odometry heading, + = left. */
static double g_turn_cmd;      /* Last requested turn rate, deg/s. */
static double g_max_heading;
static bool g_stuck;           /* Wheels never turn the robot. */
static bool g_stopped;
static bool g_not_ready;       /* Motion never reports READY + stationary. */
static unsigned g_steer_calls, g_bad_source;
static unsigned g_pass_lines, g_fail_lines, g_define_lines;
static int g_printed_offset_x = 9999;

static void check(int yes, char const *why)
{
    g_checks++;
    if (!yes) { fprintf(stderr, "FAIL: %s\n", why); exit(1); }
}

ER tk_get_otm(SYSTIM *time) { time->hi = 0; time->lo = g_clock; return E_OK; }
ER tk_dly_tsk(TMO delay)
{
    g_clock += (uint32_t)delay;
    if ((g_clock - g_start) >= g_end) { longjmp(g_finish, 1); }
    return E_OK;
}
ER subsystem4_init(void) { return E_OK; }
ER gy511_read_acceleration(gy511_acceleration_t *p_sample) { (void)p_sample; return E_OK; }
ER gy511_acceleration_mg(gy511_acceleration_t const *p_sample, uint8_t axis, int16_t *p_mg)
{
    (void)p_sample; *p_mg = (2U == axis) ? (int16_t)-1000 : (int16_t)0; return E_OK; /* level */
}
ER gy511_read_magnetic(gy511_magnetic_t *p_sample) { (void)p_sample; return E_OK; }
ER gy511_magnetic_counts(gy511_magnetic_t const *p_sample, uint8_t axis, int16_t *p_counts)
{
    /* The world field stays put: in the sensor frame it turns by -heading. */
    double theta = (30.0 - g_heading_deg) * 3.14159265358979323846 / 180.0;
    (void)p_sample;
    if (0U == axis) { *p_counts = (int16_t)lround(-123.0 + 400.0 * cos(theta)); }
    else if (1U == axis) { *p_counts = (int16_t)lround(48.0 + (400.0 / 1.06) * sin(theta)); }
    else { *p_counts = 300; }
    return E_OK;
}
bool motion_get_status(robot_status_t *p_status)
{
    *p_status = (robot_status_t){0};
    if (!g_stuck && !g_stopped) { g_heading_deg += g_turn_cmd * 0.020; } /* one 20 ms cycle */
    if (g_heading_deg > g_max_heading) { g_max_heading = g_heading_deg; }
    p_status->motion.heading = g_heading_deg * 3.14159265358979323846 / 180.0;
    p_status->motion.stationary = g_stopped || (0.0 == g_turn_cmd) || g_stuck;
    p_status->motion.state = g_not_ready ? DRIVING : (g_stopped ? STOPPED : READY);
    if (g_not_ready) { p_status->motion.stationary = false; }
    return true;
}
motion_result_t motion_steer(request_source_t source, uint32_t id, double speed, double turn_deg_s)
{
    (void)id;
    if ((SOURCE_TERRAIN != source) || (0.0 != speed)) { g_bad_source++; }
    g_steer_calls++;
    g_turn_cmd = turn_deg_s;
    return R_OK;
}
void motion_stop(void) { g_stopped = true; g_turn_cmd = 0.0; }
bool robot_update_subsystem(request_source_t source, const subsystem_status_t *p_status)
{
    (void)source; (void)p_status; return true;
}
void comms_latch_event(UB events) { (void)events; }
double motion_hal_time(void) { return (double)g_clock / 1000.0; }
INT tm_printf(UB const *format, ...)
{
    char line[256];
    va_list args;
    va_start(args, format);
    (void)vsnprintf(line, sizeof(line), (char const *)format, args);
    va_end(args);
    if (0 == strncmp(line, "[MAGCAL] PASS", 13U)) { g_pass_lines++; }
    if (0 == strncmp(line, "[MAGCAL] FAIL", 13U)) { g_fail_lines++; }
    if (0 == strncmp(line, "[MAGCAL] #define", 16U)) { g_define_lines++; }
    if (0 == strncmp(line, "[MAGCAL] #define B4_MAG_OFFSET_X", 32U))
    {
        char const *p_open = strchr(line, '(');
        if (NULL != p_open) { g_printed_offset_x = atoi(p_open + 1); }
    }
    return 0;
}

static void run(uint32_t end_ms)
{
    g_clock = 1000U; g_start = g_clock; g_end = end_ms;
    g_heading_deg = 0.0; g_turn_cmd = 0.0; g_max_heading = 0.0;
    g_stopped = false; g_steer_calls = 0U; g_bad_source = 0U;
    g_pass_lines = 0U; g_fail_lines = 0U; g_define_lines = 0U; g_printed_offset_x = 9999;
    s_cal = (s4_cal_t){0};
    if (0 == setjmp(g_finish)) { terrain_task(0, NULL); }
}

int main(void)
{
    /* Normal calibration: left 360, right 360 back, stop, PASS with constants. */
    g_stuck = false; g_not_ready = false;
    run(45000U);
    check(g_steer_calls > 1000U && 0U == g_bad_source, "pivot requested through motion_steer as SOURCE_TERRAIN, no forward speed");
    check(g_max_heading >= 359.0, "turned a full 360 deg left");
    check(fabs(g_heading_deg) < 3.0, "turned back to the start (cable untwisted)");
    check(g_stopped && (0.0 == g_turn_cmd), "stopped at the end");
    check(S4_CAL_DONE == s_cal.phase && NULL == s_cal.failure && MAG_CAL_PASS == s_cal.status, "calibration PASS");
    check(abs(s_cal.result.cal.offset_x + 123) <= 4 && abs(s_cal.result.cal.offset_y - 48) <= 4, "offsets recovered");
    check(abs((int)s_cal.result.cal.scale_y_q12 - 4342) <= 44, "Y scale recovered");
    check(-1 == s_cal.result.cal.sign && 36U == s_cal.result.bins_covered, "sign and full coverage");
    check(g_pass_lines >= 1U && 6U <= g_define_lines, "PASS and the #define lines printed");
    check(abs(g_printed_offset_x + 123) <= 4, "printed offset matches");

    /* Wheels spin but the robot never turns: stop with FAIL "not turning". */
    g_stuck = true;
    run(20000U);
    check(S4_CAL_DONE == s_cal.phase && NULL != s_cal.failure && g_stopped, "stuck robot stops with FAIL");
    check(0 == strcmp("robot not turning", s_cal.failure) && g_fail_lines >= 1U, "reason printed");
    g_stuck = false;

    /* Motion never READY and still: the pivot never starts. */
    g_not_ready = true;
    run(15000U);
    check(0U == g_steer_calls && S4_CAL_WAIT == s_cal.phase, "no pivot unless motion is ready and still");
    g_not_ready = false;

    printf("PASS: %u calibration-sequence checks (pivot left/right, stop, constants, failures)\n", g_checks);
    return 0;
}
