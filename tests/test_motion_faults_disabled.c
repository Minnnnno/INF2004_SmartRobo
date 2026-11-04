/* Motion behaviour with ROBO_MOTION_FAULTS_DISABLED: the fault-state tests
 * themselves stay in test_motion.c, which builds without the option. */
#include "motion.h"
#include "robot.h"
#include "board_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)

typedef struct { motion_t m; encoder_sample_t sample; double now; } rig_t;

static void tick(rig_t *r, unsigned counts)
{
    r->sample.count[0] += counts;
    r->sample.count[1] += counts;
    r->now += 0.01;
    motion_step(&r->m, r->sample, r->now);
}

static void run_quiet(rig_t *r, unsigned ticks)
{
    for (unsigned i = 0U; i < ticks; ++i) { tick(r, 0U); }
}

static void init(rig_t *r)
{
    motion_config_t c = board_motion_config();
    memset(r, 0, sizeof(*r));
    r->sample.reliable = true;
    motion_init(&r->m, &c, r->sample, r->now, true);
    run_quiet(r, 40U);
    CHECK(r->m.status.state == READY);
}

static motion_command_t tracking(const rig_t *r, double expires, double deadline)
{
    motion_command_t c = {1U, r->m.status.generation, WHEEL_SPEED_TRACKING, 0.0, {0.1, 0.1},
                          r->now + expires, r->now + deadline, 0.0, 0.0};
    return c;
}

static void check_never_faulted(const rig_t *r)
{
    CHECK(r->m.status.state != FAULT);
    CHECK(r->m.status.fault == F_NONE);
}

static void stalled_wheels_keep_driving(void)
{
    rig_t r;
    init(&r);
    motion_command_t c = tracking(&r, 0.25, 20.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (unsigned i = 0U; i < 200U; ++i) {
        tick(&r, 0U); /* No encoder progress at all. */
        c.expires = r.now + 0.25;
        CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    }
    check_never_faulted(&r);
    CHECK(r.m.status.state == DRIVING && !r.m.status.inhibited);
    CHECK(r.m.status.output[0] > 0.0 && r.m.status.output[1] > 0.0);
}

static void expired_command_stops_then_restarts(void)
{
    rig_t r;
    motion_event_t e;
    init(&r);
    motion_command_t c = tracking(&r, 0.25, 20.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    run_quiet(&r, 30U); /* Past expiry with no refresh. */
    check_never_faulted(&r);
    CHECK(r.m.status.state == STOPPING);
    CHECK(r.m.status.output[0] == 0.0 && r.m.status.inhibited && !r.m.status.hold);
    CHECK(motion_next_event(&r.m, &e) && e.type == EV_CANCELLED);
    run_quiet(&r, 50U);
    check_never_faulted(&r);
    CHECK(r.m.status.state == READY && !r.m.active);
    CHECK(!motion_next_event(&r.m, &e)); /* The cancel above was the only event. */
    c = tracking(&r, 0.25, 20.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    CHECK(r.m.status.state == DRIVING);
}

static void missed_goal_completes(void)
{
    rig_t r;
    motion_event_t e;
    init(&r);
    motion_command_t c = {1U, r.m.status.generation, DISTANCE_MOVE, 0.05, {0.1, 0.1},
                          r.now + 0.25, r.now + 20.0, 0.0, 0.0};
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    tick(&r, 40U); /* About 3 mm: enough to keep going. */
    for (unsigned i = 0U; i < 12U; ++i) { tick(&r, 60U); } /* About 63 mm: past the 10 mm tolerance. */
    run_quiet(&r, 60U);
    check_never_faulted(&r);
    CHECK(r.m.status.state == READY);
    CHECK(motion_next_event(&r.m, &e) && e.type == EV_COMPLETE);
    CHECK(((uint32_t)r.m.fault_flags & (UINT32_C(1) << ((uint32_t)F_FINAL_GOAL - 1u))) != 0u);
}

static void bad_samples_cannot_block_expiry(void)
{
    rig_t r;
    init(&r);
    motion_command_t c = tracking(&r, 0.25, 20.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    tick(&r, 100000U); /* Rejected by odometry; must not poison later samples. */
    check_never_faulted(&r);
    CHECK(r.m.status.state == DRIVING);
    CHECK(!r.m.status.feedback_trusted);
    tick(&r, 0U);
    CHECK(r.m.status.feedback_trusted);
    run_quiet(&r, 30U);
    CHECK(r.m.status.state == STOPPING && r.m.status.output[0] == 0.0);
    check_never_faulted(&r);
}

static void late_cycles_cannot_block_expiry(void)
{
    rig_t r;
    init(&r);
    motion_command_t c = tracking(&r, 0.25, 20.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (unsigned i = 0U; i < 5U; ++i) { /* Every cycle later than max_dt. */
        r.now += 0.1;
        motion_step(&r.m, r.sample, r.now);
    }
    check_never_faulted(&r);
    CHECK(r.m.status.state == STOPPING && r.m.status.output[0] == 0.0);
    CHECK(((uint32_t)r.m.fault_flags & (UINT32_C(1) << ((uint32_t)F_TIMING - 1u))) != 0u);
}

static void stuck_clock_cuts_output(void)
{
    rig_t r;
    init(&r);
    motion_command_t c = tracking(&r, 0.25, 20.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    tick(&r, 0U);
    tick(&r, 0U);
    CHECK(r.m.status.output[0] > 0.0);
    motion_step(&r.m, r.sample, r.now); /* dt == 0 */
    check_never_faulted(&r);
    CHECK(r.m.status.output[0] == 0.0 && r.m.status.output[1] == 0.0);
}

static void estop_still_holds(void)
{
    rig_t r;
    robot_bus_t b;
    init(&r);
    robot_bus_init(&b, &r.m.status);
    motion_command_t c = tracking(&r, 0.25, 20.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    tick(&r, 1U);
    CHECK(vehicle_controller_remote_command(&b, REMOTE_ESTOP) == R_OK);
    robot_motion_inputs(&b, &r.m, r.sample, r.now);
    check_never_faulted(&r);
    CHECK(r.m.status.state == STOPPING && r.m.status.hold);
    CHECK(r.m.status.output[0] == 0.0 && r.m.status.inhibited);
    run_quiet(&r, 50U);
    CHECK(r.m.status.state == STOPPED && r.m.status.hold);
    robot_publish(&b, &r.m, r.now);
    CHECK(vehicle_controller_remote_command(&b, REMOTE_START) == R_OK);
    robot_motion_inputs(&b, &r.m, r.sample, r.now);
    CHECK(r.m.status.state == READY && !r.m.status.hold);
}

static void startup_problems_do_not_latch(void)
{
    rig_t r;
    motion_config_t c = board_motion_config();
    memset(&r, 0, sizeof(r));
    r.sample.reliable = true;
    /* Wheels moving past the setup deadline, then settling: reaches READY. */
    motion_init(&r.m, &c, r.sample, r.now, true);
    for (unsigned i = 0U; i < 300U; ++i) { tick(&r, 5U); }
    check_never_faulted(&r);
    CHECK(r.m.status.state == INITIALISING);
    run_quiet(&r, 40U);
    CHECK(r.m.status.state == READY);
    /* An invalid config never faults and never accepts a command. */
    c.track = NAN;
    memset(&r, 0, sizeof(r));
    r.sample.reliable = true;
    motion_init(&r.m, &c, r.sample, r.now, true);
    run_quiet(&r, 40U);
    check_never_faulted(&r);
    CHECK(r.m.status.state == INITIALISING && r.m.status.inhibited);
    /* Encoder driver unavailable: stays inhibited rather than driving blind. */
    c = board_motion_config();
    memset(&r, 0, sizeof(r));
    motion_init(&r.m, &c, r.sample, r.now, true);
    run_quiet(&r, 300U);
    check_never_faulted(&r);
    CHECK(r.m.status.state == INITIALISING && r.m.status.inhibited);
}

int main(void)
{
    stalled_wheels_keep_driving();
    expired_command_stops_then_restarts();
    missed_goal_completes();
    bad_samples_cannot_block_expiry();
    late_cycles_cannot_block_expiry();
    stuck_clock_cuts_output();
    estop_still_holds();
    startup_problems_do_not_latch();
    puts("motion_faults_disabled_checks passed");
    return 0;
}
