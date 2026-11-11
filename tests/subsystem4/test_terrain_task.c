/* Run the real terrain_task() loop with the real lifecycle, tilt and terrain
 * code. Faked: clock, delay, sensor transfers, motion status, shared bus and
 * comms latch. Each scenario ends by long-jumping out of tk_dly_tsk().
 */
#include "../../app_program/subsystem4/terrain_task.c"
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { MOTION_STILL, MOTION_DRIVING, MOTION_ABSENT, MOTION_BOOTING, MOTION_SHUTDOWN } motion_mode_fake_t;

static jmp_buf g_finish;
static uint32_t g_clock, g_start, g_end;
static unsigned g_checks;
static int16_t g_xyz[3];
static ER g_read_status;
static motion_mode_fake_t g_motion;
static double g_speed[2]; /* m/s, reported while DRIVING */
static double g_heading_rad; /* Fake odometry: integrates the speed difference per call. */
static bool g_hold;      /* Controller hold as Buddy 2 reports it. */
static unsigned g_stops; /* motion_stop() calls from the lift detector. */
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
static int16_t g_mag[3] = {200, 0, 300};
static ER g_mag_status = E_OK;
ER gy511_read_magnetic(gy511_magnetic_t *p_sample) { (void)p_sample; return g_mag_status; }
ER gy511_magnetic_counts(gy511_magnetic_t const *p_sample, uint8_t axis, int16_t *p_counts)
{
    (void)p_sample; *p_counts = g_mag[axis]; return E_OK;
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
    if (MOTION_DRIVING == g_motion)
    {
        p_status->motion.speed[0] = g_speed[0]; p_status->motion.speed[1] = g_speed[1];
        g_heading_rad += (g_speed[1] - g_speed[0]) / 0.115 * 0.020; /* one 20 ms cycle */
    }
    p_status->motion.heading = g_heading_rad;
    p_status->motion.hold = g_hold;
    p_status->motion.state = (MOTION_DRIVING == g_motion) ? DRIVING :
                             ((MOTION_BOOTING == g_motion) ? INITIALISING :
                             ((MOTION_SHUTDOWN == g_motion) ? SHUTDOWN : READY));
    return true;
}
bool robot_update_subsystem(request_source_t source, const subsystem_status_t *p_status)
{
    check(SOURCE_TERRAIN == source, "publishes only the terrain slot");
    g_last = *p_status; g_published++;
    return true;
}
/* Controlled STOP: the controller holds the robot until START. */
void motion_stop(void) { g_stops++; g_hold = true; }
void comms_latch_event(UB events) { g_event_bits |= events; g_events++; }
double motion_hal_time(void) { return (double)g_clock / 1000.0; }
static unsigned g_turn_lines;
static char g_turn_reason[16]; /* Last "%s" passed with a "compass n/a (%s)" [TURN] line. */
INT tm_printf(UB const *format, ...)
{
    if ((0 == strncmp((char const *)format, "[TURN]", 6U)) &&
        (NULL != strstr((char const *)format, "compass n/a (%s)")))
    {
        va_list args;
        char const *p_reason = NULL;
        va_start(args, format);
        /* Arguments: direction, size/10, size%10, s, ds, peak, then the reason. */
        (void)va_arg(args, char const *);
        for (unsigned i = 0U; i < 5U; i++) { (void)va_arg(args, INT); }
        p_reason = va_arg(args, char const *);
        va_end(args);
        (void)snprintf(g_turn_reason, sizeof(g_turn_reason), "%s", p_reason);
    }
    if (0 == strncmp((char const *)format, "[TURN]", 6U)) { g_turn_lines++; }
    return 0;
}

/* ---- helpers ---- */
static unsigned code_of(void) { return ((unsigned)lround(g_last.values[3]) >> 3) & 7U; }
static unsigned valid_of(void) { return (unsigned)lround(g_last.values[3]) & 7U; }
static double pitch_deg(void) { return g_last.values[1] * 180.0 / 3.14159265358979323846; }
/* What another subsystem sees through include/terrain_status.h. */
static bool faulted_of(void)
{
    robot_status_t status = {0};
    status.terrain = g_last;
    check(code_of() == terrain_published_state(&g_last), "decoder agrees with the published code");
    return terrain_imu_faulted(&status);
}
static void level(void) { g_xyz[0] = 0; g_xyz[1] = 0; g_xyz[2] = -1000; }

static void run(motion_mode_fake_t motion, uint32_t end_ms, void (*script)(uint32_t))
{
    g_clock = 1000U; g_start = g_clock; g_end = end_ms;
    g_motion = motion; g_script = script; g_read_status = E_OK;
    g_speed[0] = 0.0; g_speed[1] = 0.0; g_hold = false; g_stops = 0U; g_heading_rad = 0.0;
    g_mag_status = E_OK; g_mag[0] = 200; g_mag[1] = 0; g_turn_reason[0] = '\0';
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
        check(!faulted_of(), "level ground is not a fault");
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
        check(!faulted_of(), "climbing is not a fault");
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
        check(!faulted_of(), "descending is not a fault");
        level();
        g_step = 3U;
    }
    else if ((3U == g_step) && (t >= 14000U))
    {
        check(S4_TS_WAITING == code_of(), "level again publishes WAITING");
        check(!faulted_of(), "no fault after the hump");
        check((1U == g_events) && (TF_EV_HUMP_PEAK == g_event_bits), "one hump event latched");
        check(fabs(g_last.values[2] - 0.030) < 0.0005 && (0U != (valid_of() & TF_TV_HUMP)),
              "run maximum kept after the hump");
        check(g_status_calls > g_calls_at_monitoring, "motion status watched every cycle for shutdown");
        g_step = 4U;
    }
}

/* The hump run, then Buddy 2 shuts down: Buddy 4 must stop and keep the peak. */
static void shutdown_script(uint32_t t)
{
    hump_script(t);
    if ((4U == g_step) && (MOTION_SHUTDOWN != g_motion)) { g_motion = MOTION_SHUTDOWN; }
}

/* Motion class as other subsystems decode it (terrain_status.h). */
static int class_of(void)
{
    uint8_t mc = 0U;
    return terrain_motion_class(&g_last, &mc) ? (int)mc : -1;
}
static double yaw_deg(void) { return g_last.values[0] * 180.0 / 3.14159265358979323846; }

/* Level boot, then drive: turning, then straighten so the turn completes. */
static unsigned g_mstep;
static uint32_t g_turn_at;
static void motion_script(uint32_t t)
{
    if ((0U == g_mstep) && (t >= 4000U))
    {
        check(S4_TS_WAITING == code_of(), "monitoring before the motion checks");
        check(TERRAIN_MC_STATIONARY == class_of(), "still robot: STATIONARY, valid");
        check(0.0 == g_last.values[0], "zero turn rate at rest");
        g_motion = MOTION_DRIVING; g_speed[0] = 0.0; g_speed[1] = 0.100; /* 50 deg/s left pivot */
        g_mstep = 1U;
    }
    else if ((1U == g_mstep) && (t >= 4500U))
    {
        check(TERRAIN_MC_TURNING == class_of(), "a real heading change publishes TURNING");
        check(fabs(yaw_deg() - 50.0) < 0.6, "turn rate 50 deg/s in values[0], + = left");
        check(TF_TV_PITCH == valid_of(), "frame validity bits unchanged (heading bit stays clear)");
        g_turn_at = t;
        g_mstep = 4U;
    }
    else if ((4U == g_mstep) && (t >= (g_turn_at + 500U)))
    {
        check(TERRAIN_MC_TURNING == class_of(), "still TURNING half a second later");
        check(0U == g_stops, "driving never triggers the lift stop");
        g_speed[0] = 0.100; g_speed[1] = 0.100; /* straighten: the turn completes */
        g_mstep = 5U;
    }
}

/* Lift/overturn stop: drive straight, overturn, START pressed in the air,
 * set down, then lift straight up. */
static unsigned g_lstep;
static uint32_t g_lift_at;
static void lift_script(uint32_t t)
{
    if ((0U == g_lstep) && (t >= 4000U))
    {
        check(S4_TS_WAITING == code_of() && 0U == g_stops, "monitoring, nothing stopped yet");
        g_motion = MOTION_DRIVING; g_speed[0] = 0.100; g_speed[1] = 0.100;
        g_lstep = 1U;
    }
    else if ((1U == g_lstep) && (t >= 5000U))
    {
        check(TERRAIN_MC_MOVING == class_of() && 0U == g_stops, "driving straight: MOVING, no stop");
        g_xyz[0] = 0; g_xyz[1] = 0; g_xyz[2] = 1000; /* rolled upside down */
        g_lift_at = t;
        g_lstep = 2U;
    }
    else if ((2U == g_lstep) && (t >= (g_lift_at + 160U)))
    {
        check(0U == g_stops, "not before 200 ms overturned");
        g_lstep = 3U;
    }
    else if ((3U == g_lstep) && (t >= (g_lift_at + 260U)))
    {
        check(1U == g_stops && g_hold, "overturned: one motion_stop()");
        check(TERRAIN_MC_OVERTURNED == class_of(), "overturned publishes motion class 7");
        check(S4_TS_LIFTED != code_of() && !faulted_of(), "overturned is not T_STATE LIFTED and not a fault");
        g_hold = false; /* START pressed while still upside down */
        g_lstep = 4U;
    }
    else if ((4U == g_lstep) && (t >= (g_lift_at + 320U)))
    {
        check(2U == g_stops && g_hold, "released while overturned: stopped again");
        level();
        g_lift_at = t;
        g_lstep = 5U;
    }
    else if ((5U == g_lstep) && (t >= (g_lift_at + 700U)))
    {
        check(2U == g_stops && g_hold, "set down: latch clears, no new stop, still held");
        check(TERRAIN_MC_OVERTURNED != class_of(), "class leaves OVERTURNED");
        g_hold = false; /* START: driving again */
        g_xyz[2] = -1300; /* picked straight up: +300 mg */
        g_lift_at = t;
        g_lstep = 6U;
    }
    else if ((6U == g_lstep) && (t >= (g_lift_at + 100U)))
    {
        check(3U == g_stops && g_hold, "picked up: stopped within ~80 ms");
        check(S4_TS_LIFTED == code_of() && TERRAIN_TS_LIFTED == terrain_published_state(&g_last),
              "lifted publishes T_STATE 7");
        check(TERRAIN_MC_OVERTURNED != class_of(), "lifted is not OVERTURNED");
        g_xyz[2] = -1000; /* held still and level in the air or set down */
        g_lstep = 7U;
    }
}

/* Tilt to one side while driving (S4-H06): 45 deg for 2 s is neither a lift
 * nor an overturn, stops nothing and records no hump. */
static unsigned g_tstep;
static void tilt_script(uint32_t t)
{
    if ((0U == g_tstep) && (t >= 4000U))
    {
        g_motion = MOTION_DRIVING; g_speed[0] = 0.100; g_speed[1] = 0.100;
        g_tstep = 1U;
    }
    else if ((1U == g_tstep) && (t >= 5000U))
    {
        g_xyz[0] = -707; g_xyz[1] = 0; g_xyz[2] = -707; /* 45 deg to one side */
        g_tstep = 2U;
    }
    else if ((2U == g_tstep) && (t >= 7000U))
    {
        check(0U == g_stops && !g_hold, "45 deg tilt for 2 s: no motion_stop()");
        check(S4_TS_LIFTED != code_of(), "45 deg tilt is not T_STATE LIFTED");
        check(TERRAIN_MC_OVERTURNED != class_of(), "45 deg tilt is not OVERTURNED");
        level();
        g_tstep = 3U;
    }
    else if ((3U == g_tstep) && (t >= 9000U))
    {
        check(0U == g_stops, "still no stop after setting it level");
        check(0U == g_events, "no hump event from the tilt");
        check(0U == (valid_of() & TF_TV_HUMP), "no hump height recorded from the tilt");
        check(S4_TS_WAITING == code_of() && TERRAIN_MC_MOVING == class_of(), "back to WAITING and MOVING");
        g_tstep = 4U;
    }
}

/* Compass n/a reason: turn 1 has a good compass; then, driving straight, the
 * field goes bad for a while ("field") and recovers; during turn 2 the
 * magnetometer reads out of range ("no data"). Turn 2 must say "no data":
 * the stale "field" from between the turns must not label it. */
static void reason_script(uint32_t t)
{
    g_mag[0] = ((t >= 6000U) && (t < 6500U)) ? (int16_t)2000 : (int16_t)200;
    g_mag_status = ((t >= 7600U) && (t < 7800U)) ? E_IO : E_OK;
    if ((t >= 4000U) && (t < 4020U)) { g_motion = MOTION_DRIVING; g_speed[0] = 0.0; g_speed[1] = 0.100; }
    if ((t >= 5200U) && (t < 5220U)) { g_speed[0] = 0.100; g_speed[1] = 0.100; } /* turn 1 ends */
    if ((t >= 7000U) && (t < 7020U)) { g_speed[0] = 0.0; g_speed[1] = 0.100; }   /* turn 2 */
    if ((t >= 8200U) && (t < 8220U)) { g_speed[0] = 0.100; g_speed[1] = 0.100; } /* turn 2 ends */
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

    /* Motion class and turn rate published in the terrain slot. */
    g_mstep = 0U;
    g_turn_lines = 0U;
    run(MOTION_STILL, 6500U, motion_script);
    check(5U == g_mstep, "all motion checkpoints reached");

    /* Lift / overturn: motors stopped, re-stopped if released in the air. */
    g_lstep = 0U;
    run(MOTION_STILL, 7000U, lift_script);
    check(7U == g_lstep, "all lift checkpoints reached");
    check(3U == g_stops, "exactly three stops (overturn, release in the air, lift)");

    /* Tilting to one side never stops the robot (35 deg rule removed). */
    g_tstep = 0U;
    run(MOTION_STILL, 9100U, tilt_script);
    check(4U == g_tstep, "all tilt checkpoints reached");
    check(0U == g_stops, "no stop during the whole tilt run");
    check(1U == g_turn_lines, "one [TURN] line printed when the pivot ends");

    /* n/a reason comes from the turn itself, not a stale earlier reason. */
    g_turn_lines = 0U;
    run(MOTION_STILL, 9500U, reason_script);
    check(2U == g_turn_lines, "both pivots reported");
    check(0 == strcmp("no data", g_turn_reason), "turn 2 reason is the problem seen during it, not the stale 'field'");

    /* Motion shutdown after a hump: B4 STOP (T_STATE 6), run maximum kept. */
    g_step = 0U;
    run(MOTION_STILL, 14300U, shutdown_script);
    check(4U == g_step, "shutdown run reached the end of the hump");
    check(g_last.available && (S4_TS_STOP == code_of()), "motion SHUTDOWN publishes STOP");
    check((0U != (valid_of() & TF_TV_HUMP)) && (fabs(g_last.values[2] - 0.030) < 0.0005),
          "STOP keeps the run maximum");
    check(!faulted_of(), "STOP is not a fault");

    /* Wheels reported moving: no level permission, so capture times out. */
    run(MOTION_DRIVING, 16500U, NULL);
    check(0U == g_reads, "no capture reads without level permission");
    check(g_last.available && (S4_TS_FAULT == code_of()) && (0U == valid_of()),
          "capture timeout publishes FAULT with pitch invalid");
    check(faulted_of(), "capture timeout reports the IMU fault");

    /* Motion task not running: capture is allowed. */
    run(MOTION_ABSENT, 7500U, NULL);
    check((S4_TS_WAITING == code_of()) && (TF_TV_PITCH == valid_of()), "capture allowed without motion task");

    /* Motion still initialising (outputs held stopped): capture is allowed. */
    run(MOTION_BOOTING, 7500U, NULL);
    check(S4_TS_WAITING == code_of(), "capture allowed while motion initialises");

    /* Sensor failure after monitoring starts: RECOVERY, pitch invalid. */
    run(MOTION_STILL, 8100U, fail_script);
    check((S4_TS_RECOVERY == code_of()) && (0U == valid_of()), "I2C failure publishes RECOVERY");
    check(!faulted_of(), "RECOVERY is not yet a latched fault");

    /* Decoder edge cases: missing, malformed or absent data means normal speed. */
    {
        robot_status_t status = {0};
        subsystem_status_t slot = {0};
        check(TERRAIN_TS_UNKNOWN == terrain_published_state(NULL), "NULL slot is UNKNOWN");
        check(TERRAIN_TS_UNKNOWN == terrain_published_state(&slot), "unavailable slot is UNKNOWN");
        check(!terrain_imu_faulted(NULL) && !terrain_imu_faulted(&status),
              "no Buddy 4 (slot never published): no fault reported");
        slot.available = true;
        slot.values[3] = -16.0;
        check(TERRAIN_TS_UNKNOWN == terrain_published_state(&slot), "negative word is UNKNOWN");
        slot.values[3] = 1024.0;
        check(TERRAIN_TS_UNKNOWN == terrain_published_state(&slot), "out-of-range word is UNKNOWN");
        slot.values[3] = NAN;
        check(TERRAIN_TS_UNKNOWN == terrain_published_state(&slot), "NaN word is UNKNOWN");
        status.terrain.available = true;
        for (unsigned code = 0U; code < 8U; code++)
        {
            status.terrain.values[3] = (double)((code << 3U) | TF_TV_PITCH | TF_TV_HUMP);
            check(code == terrain_published_state(&status.terrain), "every code decodes");
            check(terrain_imu_faulted(&status) == (TERRAIN_TS_FAULT == code),
                  "fault only for T_STATE 5 (LIFTED 7 is not a fault)");
        }
    }

    /* Motion class decoder edge cases. */
    {
        subsystem_status_t slot = {0};
        uint8_t mc = 9U;
        check(!terrain_motion_class(&slot, &mc) && (9U == mc), "unavailable slot: no class");
        slot.available = true;
        slot.values[3] = (double)((TERRAIN_TS_WAITING << 3U) | (TERRAIN_MC_REVERSING << TERRAIN_MC_SHIFT));
        check(!terrain_motion_class(&slot, &mc), "class without its valid bit is ignored");
        slot.values[3] += (double)TERRAIN_MOTION_VALID;
        check(terrain_motion_class(&slot, &mc) && (TERRAIN_MC_REVERSING == mc), "valid class decodes");
        check(TERRAIN_TS_WAITING == terrain_published_state(&slot), "state still decodes with class bits set");
        check(!terrain_motion_class(&slot, NULL) && !terrain_motion_class(NULL, &mc), "NULL safe");
    }

    /* A missing Buddy 4 is never reported as an IMU fault. */
    {
        robot_status_t status = {0};
        status.terrain.values[3] = (double)(TERRAIN_TS_FAULT << 3U);
        check(!terrain_imu_faulted(&status), "unavailable slot is not a fault");
    }

    printf("PASS: %u terrain task checks (snapshot, state codes, hump event, level gating, timing, lift stop)\n",
           g_checks);
    return 0;
}
