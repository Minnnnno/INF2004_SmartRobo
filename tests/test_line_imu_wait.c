/* Line task start-up wait for Buddy 4's tilt reference (ROBO_SUBSYSTEM4=1).
 * Includes the real line_following.c so its static wait can be called directly. */
#include "../app_program/subsystem3/line_following.c"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); exit(1); } } while (0)

static unsigned checks, delays, status_calls;
static double clock_s;
static double settle_at;       /* Time Buddy 4 leaves INIT; < 0 = never. */
static unsigned settled_state; /* T_STATE published once settled. */
static bool status_ok;
static bool terrain_available;

void gpio_init(unsigned pin) { (void)pin; }
void gpio_set_dir(unsigned pin, bool output) { (void)pin; (void)output; }
void gpio_disable_pulls(unsigned pin) { (void)pin; }
bool gpio_get(unsigned pin) { (void)pin; return false; }
double motion_hal_time(void) { return clock_s; }
bool robot_update_subsystem(request_source_t source, const subsystem_status_t *s)
{ (void)source; (void)s; CHECK(false); return false; }
bool motion_get_status(robot_status_t *out)
{
    unsigned state = ((settle_at >= 0.0) && (clock_s >= settle_at)) ? settled_state : LINE_TERRAIN_INIT;
    ++status_calls;
    if (!status_ok) { return false; }
    *out = (robot_status_t){0};
    out->terrain.available = terrain_available;
    out->terrain.values[3] = (double)((state << 3) | ((state == 1U) ? 2U : 0U));
    return true;
}
motion_result_t robot_submit(request_source_t source, uint32_t id, motion_mode_t mode,
                             double value, double left, double right)
{ (void)source; (void)id; (void)mode; (void)value; (void)left; (void)right; CHECK(false); return R_INVALID; }
motion_result_t robot_submit_arc(request_source_t source, uint32_t id, double metres,
                                 double angle, double duration, double left, double right)
{ (void)source; (void)id; (void)metres; (void)angle; (void)duration; (void)left; (void)right; CHECK(false); return R_INVALID; }
void motion_stop(void) { CHECK(false); }
int tm_printf(const UB *format, ...) { (void)format; return 0; }
int tk_dly_tsk(unsigned delay) { CHECK(delay == 20U); ++delays; clock_s += 0.02; return 0; }
int tk_slp_tsk(int timeout) { (void)timeout; CHECK(false); return 0; }

static double wait_once(bool ok, bool available, double settle, unsigned state)
{
    clock_s = 2.0; delays = 0U; status_calls = 0U;
    status_ok = ok; terrain_available = available; settle_at = settle; settled_state = state;
    line_wait_for_imu();
    return clock_s - 2.0;
}

int main(void)
{
    double waited;

    /* Typical boot: Buddy 4 reaches MONITORING (WAITING) 0.5 s into the wait. */
    waited = wait_once(true, true, 2.5, 1U);
    CHECK(waited >= 0.5 && waited < 0.52);

    /* Already settled: no delay at all. */
    CHECK(wait_once(true, true, 0.0, 1U) == 0.0 && delays == 0U && status_calls == 1U);

    /* Capture over in any non-INIT/RECOVERY state: climbing, fault, stop. */
    CHECK(wait_once(true, true, 0.0, 2U) == 0.0);
    CHECK(wait_once(true, true, 0.0, 5U) == 0.0); /* IMU gave up: drive without it. */
    CHECK(wait_once(true, true, 0.0, 6U) == 0.0);

    /* RECOVERY keeps waiting (the capture may still follow), but only up to the cap. */
    waited = wait_once(true, true, 0.0, LINE_TERRAIN_RECOVERY);
    CHECK(waited >= LINE_IMU_WAIT_MAX && waited < LINE_IMU_WAIT_MAX + 0.021);

    /* IMU task absent or never leaves INIT: the cap releases the robot. */
    waited = wait_once(true, false, 0.0, 1U);
    CHECK(waited >= LINE_IMU_WAIT_MAX && waited < LINE_IMU_WAIT_MAX + 0.021);
    waited = wait_once(true, true, -1.0, 1U);
    CHECK(waited >= LINE_IMU_WAIT_MAX && waited < LINE_IMU_WAIT_MAX + 0.021);

    /* Bus not running yet: keep waiting, still capped. */
    waited = wait_once(false, true, 0.0, 1U);
    CHECK(waited >= LINE_IMU_WAIT_MAX && waited < LINE_IMU_WAIT_MAX + 0.021);

    /* Decoder: unavailable or negative slots never count as settled. */
    {
        robot_status_t s = {0};
        s.terrain.available = false; s.terrain.values[3] = 10.0;
        CHECK(!line_imu_settled(&s));
        s.terrain.available = true; s.terrain.values[3] = -8.0;
        CHECK(!line_imu_settled(&s));
        s.terrain.values[3] = 10.0; /* WAITING (1 << 3) | pitch valid (2) */
        CHECK(line_imu_settled(&s));
    }
    printf("PASS: %u line start-up IMU wait checks\n", checks);
    return 0;
}
