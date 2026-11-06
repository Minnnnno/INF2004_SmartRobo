/* Run the real terrain_task() loop with the real lifecycle, tilt and terrain
 * code. Faked: clock, delay, sensor transfers, motion status, shared bus and
 * comms latch. Each scenario ends by long-jumping out of tk_dly_tsk().
 */
#include "../../../app_program/subsystem4/terrain_task.c"
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>

typedef enum { MOTION_STILL, MOTION_DRIVING, MOTION_ABSENT, MOTION_BOOTING } motion_mode_fake_t;

static jmp_buf g_finish;
static uint32_t g_clock, g_start, g_end;
static unsigned g_checks;
static int16_t g_xyz[3];
static ER g_read_status;
static motion_mode_fake_t g_motion;
static unsigned g_status_calls, g_published, g_events, g_reads;
static uint32_t g_read_time[2048];
static subsystem_status_t g_last;
static UB g_event_bits;
static void (*g_script)(uint32_t elapsed);

static void check(int yes, char const *why)
{
    g_checks++;
    if (!yes) { fprintf(stderr, "FAIL: %s\n", why); exit(1); }
}

/* ---- kernel, device and team-interface fakes ---- */
ER tk_get_otm(SYSTIM *time) { time->hi = 0; time->lo = g_clock; return E_OK; }
ER tk_dly_tsk(TMO delay)
{
    check((delay > 0) && (delay <= (TMO)S4_PERIOD_MS), "delay within one period");
    g_clock += (uint32_t)delay;
    if (NULL != g_script) { g_script(g_clock - g_start); }
    if ((g_clock - g_start) >= g_end) { longjmp(g_finish, 1); }
    return E_OK;
}
ER subsystem4_init(void) { return E_OK; }
ER gy511_read_acceleration(gy511_acceleration_t *p_sample)
{
    (void)p_sample;
    if (g_reads < (sizeof(g_read_time) / sizeof(g_read_time[0]))) { g_read_time[g_reads] = g_clock; }
    g_reads++;
    g_clock += 2U; /* Synchronous I2C transfer time. */
    return g_read_status;
}
ER gy511_acceleration_mg(gy511_acceleration_t const *p_sample, uint8_t axis, int16_t *p_mg)
{
    (void)p_sample; *p_mg = g_xyz[axis]; return E_OK;
}
bool motion_get_status(robot_status_t *p_status)
{
    g_status_calls++;
    if (MOTION_ABSENT == g_motion) { return false; }
    *p_status = (robot_status_t){0};
    p_status->motion.stationary = (MOTION_STILL == g_motion);
    p_status->motion.state = (MOTION_DRIVING == g_motion) ? DRIVING :
                             ((MOTION_BOOTING == g_motion) ? INITIALISING : READY);
    return true;
}
bool robot_update_subsystem(request_source_t source, const subsystem_status_t *p_status)
{
    check(SOURCE_TERRAIN == source, "publishes only the terrain slot");
    g_last = *p_status; g_published++;
    return true;
}
void comms_latch_event(UB events) { g_event_bits |= events; g_events++; }
double motion_hal_time(void) { return (double)g_clock / 1000.0; }
INT tm_printf(UB const *format, ...) { (void)format; return 0; }

/* ---- helpers ---- */
static unsigned code_of(void) { return ((unsigned)lround(g_last.values[3]) >> 3) & 7U; }
static unsigned valid_of(void) { return (unsigned)lround(g_last.values[3]) & 7U; }
static double pitch_deg(void) { return g_last.values[1] * 180.0 / 3.14159265358979323846; }
static void level(void) { g_xyz[0] = 0; g_xyz[1] = 0; g_xyz[2] = -1000; }

static void run(motion_mode_fake_t motion, uint32_t end_ms, void (*script)(uint32_t))
{
    g_clock = 1000U; g_start = g_clock; g_end = end_ms;
    g_motion = motion; g_script = script; g_read_status = E_OK;
    g_status_calls = 0U; g_published = 0U; g_events = 0U; g_reads = 0U; g_event_bits = 0U;
    g_last = (subsystem_status_t){0};
    level();
    if (0 == setjmp(g_finish)) { terrain_task(0, NULL); }
}

/* Level boot, then a forward hump: up, down, level again. */
static unsigned g_step;
static unsigned g_calls_at_monitoring;
static void hump_script(uint32_t t)
{
    if ((0U == g_step) && (t >= 8000U))
    {
        check(g_last.available && (S4_TS_WAITING == code_of()) && (TF_TV_PITCH == valid_of()),
              "monitoring publishes WAITING with valid pitch");
        check(fabs(pitch_deg()) < 0.5, "level pitch near zero");
        check(0.0 == g_last.values[0] && 0.0 == g_last.values[2], "heading and hump height slots stay invalid");
        check(fabs(g_last.timestamp - motion_hal_time()) < 0.1, "timestamp is the fresh sample time");
        g_calls_at_monitoring = g_status_calls;
        /* Right wheel onto a ~30 mm hump: 15 degrees sideways, ~5.7 degrees nose up. */
        g_xyz[0] = 259; g_xyz[1] = -100; g_xyz[2] = -961;
        g_step = 1U;
    }
    else if ((1U == g_step) && (t >= 10000U))
    {
        check(S4_TS_CLIMBING == code_of(), "one-wheel hump publishes CLIMBING");
        check(fabs(pitch_deg() - 5.7) < 0.5, "pitch published in radians, + nose up");
        check((TF_TV_PITCH | TF_TV_HUMP) == valid_of(), "hump height valid once climbing");
        check(fabs(g_last.values[2] - 0.030) < 0.0005, "live peak 30 mm published in metres");
        check(0U == g_events, "no hump event while climbing");
        g_xyz[0] = 130; g_xyz[1] = 0; g_xyz[2] = -991; /* ~15 mm: coming down */
        g_step = 2U;
    }
    else if ((2U == g_step) && (t >= 12000U))
    {
        check(S4_TS_DESCENDING == code_of(), "lower wheel publishes DESCENDING");
        level();
        g_step = 3U;
    }
    else if ((3U == g_step) && (t >= 14000U))
    {
        check(S4_TS_WAITING == code_of(), "level again publishes WAITING");
        check((1U == g_events) && (TF_EV_HUMP_PEAK == g_event_bits), "one hump event latched");
        check(fabs(g_last.values[2] - 0.030) < 0.0005 && (0U != (valid_of() & TF_TV_HUMP)),
              "run maximum kept after the hump");
        check(g_calls_at_monitoring == g_status_calls, "motion status read only during INIT");
        g_step = 4U;
    }
}

static void fail_script(uint32_t t)
{
    if (t >= 8000U) { g_read_status = E_IO; }
}

int main(void)
{
    /* Normal run with a hump and a release-grid check. */
    g_step = 0U;
    run(MOTION_STILL, 14100U, hump_script);
    check(4U == g_step, "all hump checkpoints reached");
    for (unsigned i = 1U; i < g_reads; i++)
    {
        check(20U == (g_read_time[i] - g_read_time[i - 1U]), "one sample per 20 ms release");
    }
    check(g_published > 600U, "snapshot published every cycle");

    /* Wheels reported moving: no level permission, so capture times out. */
    run(MOTION_DRIVING, 16500U, NULL);
    check(0U == g_reads, "no capture reads without level permission");
    check(g_last.available && (S4_TS_FAULT == code_of()) && (0U == valid_of()),
          "capture timeout publishes FAULT with pitch invalid");

    /* Motion task not running: capture is allowed. */
    run(MOTION_ABSENT, 7500U, NULL);
    check((S4_TS_WAITING == code_of()) && (TF_TV_PITCH == valid_of()), "capture allowed without motion task");

    /* Motion still initialising (outputs held stopped): capture is allowed. */
    run(MOTION_BOOTING, 7500U, NULL);
    check(S4_TS_WAITING == code_of(), "capture allowed while motion initialises");

    /* Sensor failure after monitoring starts: RECOVERY, pitch invalid. */
    run(MOTION_STILL, 8100U, fail_script);
    check((S4_TS_RECOVERY == code_of()) && (0U == valid_of()), "I2C failure publishes RECOVERY");

    printf("PASS: %u terrain task checks (snapshot, state codes, hump event, level gating, timing)\n",
           g_checks);
    return 0;
}
