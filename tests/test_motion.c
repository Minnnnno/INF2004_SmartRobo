#include "motion.h"
#include "robot.h"
#include "board_config.h"
#include "encoder_capture.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define NEAR(a,b,t) CHECK(fabs((a)-(b)) <= (t))

static motion_config_t simulation_config(void)
{
    motion_config_t c = board_motion_config();
    c.hardware_confirmed = true;
    c.counts_per_rev[0] = 2000.0;
    c.counts_per_rev[1] = 2000.0;
    c.diameter[0] = 0.065;
    c.diameter[1] = 0.065;
    c.track = 0.14;
    return c;
}

typedef struct { motion_t m; encoder_sample_t sample; double now, fractional[2], velocity[2]; } rig_t;

static void tick(rig_t *r, bool plant)
{
    unsigned w;
    const double dt = 0.01;
    if (plant) {
        for (w = 0; w < 2U; ++w) {
            double output = r->m.status.output[w];
            double quantum = 3.141592653589793 * r->m.config.diameter[w] / r->m.config.counts_per_rev[w];
            int32_t count;
            /* Deliberately unequal simulated motor gains, first-order inertia. */
            r->velocity[w] += (output * (w == 0U ? 0.9 : 0.8) - r->velocity[w]) * dt / 0.04;
            r->fractional[w] += r->velocity[w] * dt / quantum;
            count = (int32_t)r->fractional[w];
            r->fractional[w] -= count;
            r->sample.count[w] += (uint32_t)count;
        }
    }
    r->now += dt;
    motion_step(&r->m, r->sample, r->now);
}

static void init(rig_t *r)
{
    motion_config_t c = simulation_config();
    unsigned i;
    memset(r, 0, sizeof(*r));
    r->sample.reliable = true;
    motion_init(&r->m, &c, r->sample, r->now, true);
    CHECK(r->m.status.state == INITIALISING);
    for (i = 0; i < 40U; ++i) { tick(r, false); }
    CHECK(r->m.status.state == READY);
}

static motion_command_t command(rig_t *r, motion_mode_t mode, double value)
{
    motion_command_t c = {1U, r->m.status.generation, mode, value, {0.12, 0.12},
                          r->now + 0.25, r->now + 20.0};
    return c;
}

static void math_tests(void)
{
    motion_config_t c = simulation_config();
    odometry_t o = {0};
    encoder_sample_t s = {{100U, 200U}, {0U, 0U}, true};
    pid_t p = {0};
    pid_config_t gains = {2.0, 1.0, 0.1, 0.05};
    bool bad;
    unsigned i;
    CHECK(encoder_delta(2U, UINT32_MAX - 2U) == 5);
    CHECK(encoder_delta(UINT32_MAX - 2U, 2U) == -5);
    CHECK(encoder_delta(0x80000000U, 0U) == INT64_C(-2147483648));
    CHECK(encoder_quadrature(0, 1, &bad) == 1 && !bad);
    CHECK(encoder_quadrature(1, 3, &bad) == 1 && !bad);
    CHECK(encoder_quadrature(3, 2, &bad) == 1 && !bad);
    CHECK(encoder_quadrature(2, 0, &bad) == 1 && !bad);
    CHECK(encoder_quadrature(1, 0, &bad) == -1 && !bad);
    CHECK(encoder_quadrature(0, 3, &bad) == 0 && bad);
    CHECK(odometry_step(&o, &c, &s, 0.05));
    NEAR(o.travel[0], 100.0 * 3.141592653589793 * 0.065 / 2000.0, 1e-10);
    NEAR(o.speed[0], o.travel[0] / 0.05, 1e-10);
    NEAR(o.heading, (o.travel[1] - o.travel[0]) / 0.14, 1e-10);
    CHECK(!odometry_step(&o, &c, &s, 0.0));
    CHECK(!odometry_step(&o, &c, &s, 1.0));
    for (i = 0; i < 1000U; ++i) { CHECK(fabs(pid_step(&p, &gains, 10.0, 0.0, 0.01, 0.3)) <= 0.3); }
    NEAR(p.integral, 0.0, 1e-10);
    CHECK(pid_step(&p, &gains, -0.1, 0.0, 0.01, 0.3) < 0.0);
    pid_reset(&p);
    CHECK(!p.seeded);
    c = board_motion_config();
    CHECK(motion_config_valid(&c));
    c.hardware_confirmed = false;
    CHECK(!motion_config_valid(&c));
    c = simulation_config(); c.track = NAN;
    CHECK(!motion_config_valid(&c));
}

static void movement_tests(void)
{
    const double values[] = {0.20, -0.20, 1.57079632679, -1.57079632679, 3.14159265359, 0.0};
    unsigned test;
    for (test = 0; test < 6U; ++test) {
        rig_t r;
        motion_command_t c;
        motion_event_t e;
        unsigned i;
        init(&r);
        c = command(&r, (test < 2U || test == 5U) ? DISTANCE_MOVE : ENCODER_TURN, values[test]);
        CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
        for (i = 0; i < 2100U && (r.m.status.state == DRIVING || r.m.status.state == STOPPING); ++i) { tick(&r, true); }
        if (r.m.status.state != READY) { fprintf(stderr, "movement %u state %d fault %d travel %f %f\n", test, r.m.status.state, r.m.status.fault, r.m.odom.travel[0], r.m.odom.travel[1]); }
        CHECK(r.m.status.state == READY);
        CHECK(motion_next_event(&r.m, &e)); CHECK(e.type == EV_COMPLETE && e.id == 1U);
    }
}

static void safety_tests(void)
{
    rig_t r;
    motion_command_t c;
    unsigned i;
    init(&r); c = command(&r, DISTANCE_MOVE, 1.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i = 0; i < 50U; ++i) { tick(&r, true); }
    motion_cancel(&r.m, r.now);
    CHECK(r.m.status.state == STOPPING && r.m.status.output[0] == 0.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_STALE);
    for (i = 0; i < 80U; ++i) { tick(&r, true); }
    CHECK(r.m.status.state == STOPPED);
    CHECK(!motion_release(&r.m, false)); CHECK(motion_release(&r.m, true));
    CHECK(motion_accept(&r.m, &c, r.now) == R_STALE);
    init(&r); c = command(&r, DISTANCE_MOVE, 1.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i = 0; i < 30U; ++i) { tick(&r, false); }
    CHECK(r.m.status.state == DRIVING);
    for (i = 0; i < 150U; ++i) { tick(&r, false); }
    CHECK(r.m.status.fault == F_FEEDBACK && !r.m.status.stationary);
    CHECK(!motion_reset(&r.m, true, false, r.sample, r.now));
    CHECK(!motion_reset(&r.m, false, true, r.sample, r.now));
    CHECK(motion_reset(&r.m, true, true, r.sample, r.now));
    CHECK(motion_accept(&r.m, &c, r.now) == R_STALE);
    for (i = 0; i < 40U; ++i) { tick(&r, false); }
    CHECK(r.m.status.state == STOPPED);
    CHECK(motion_release(&r.m, true));
    motion_shutdown(&r.m, r.now);
    CHECK(!motion_release(&r.m, true)); CHECK(!motion_reset(&r.m, true, true, r.sample, r.now));
    c = command(&r, DISTANCE_MOVE, 0.1);
    CHECK(motion_accept(&r.m, &c, r.now) == R_HELD);
    init(&r); c = command(&r, WHEEL_SPEED_TRACKING, 0.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i = 0; i < 30U; ++i) { tick(&r, true); }
    CHECK(r.m.status.fault == F_EXPIRED);
    init(&r); c = command(&r, DISTANCE_MOVE, 1.0); c.deadline = r.now + 0.1;
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i = 0; i < 20U; ++i) { tick(&r, true); }
    CHECK(r.m.status.fault == F_DEADLINE);
    init(&r); r.now += 1.0; motion_step(&r.m, r.sample, r.now);
    CHECK(r.m.status.fault == F_TIMING);
    init(&r); r.sample.invalid[0] = 1U; tick(&r, false);
    CHECK(r.m.status.fault == F_FEEDBACK);
}

static void correction_and_final_tests(void)
{
    rig_t r;
    motion_command_t c;
    unsigned i;
    init(&r); c = command(&r, DISTANCE_MOVE, 1.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i = 0; i < 100U; ++i) { tick(&r, true); }
    r.m.odom.travel[1] += 0.02;
    tick(&r, true);
    CHECK(r.m.status.target[0] > r.m.status.target[1]);
    init(&r); c = command(&r, WHEEL_SPEED_TRACKING, 0.0); c.speed[0] = 0.05; c.speed[1] = 0.15;
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i = 0; i < 60U; ++i) {
        c.expires = r.now + 0.25; c.deadline = r.now + 20.0;
        CHECK(motion_accept(&r.m, &c, r.now) == R_OK); tick(&r, true);
    }
    NEAR(r.m.status.target[0], 0.05, 1e-9); NEAR(r.m.status.target[1], 0.15, 1e-9);
    CHECK(r.m.pid[0].seeded && r.m.pid[0].integral != 0.0);
    init(&r); c = command(&r, DISTANCE_MOVE, 0.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK); tick(&r, false);
    CHECK(r.m.status.state == STOPPING);
    for (i = 0; i < 10U; ++i) { r.sample.count[0] += 25U; r.sample.count[1] += 25U; tick(&r, false); }
    for (i = 0; i < 40U; ++i) { tick(&r, false); }
    CHECK(r.m.status.fault == F_FINAL_GOAL);
    init(&r); c = command(&r, DISTANCE_MOVE, 1.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK); motion_cancel(&r.m, r.now);
    for (i = 0; i < 310U; ++i) { r.sample.count[0] += 2U; tick(&r, false); }
    CHECK(r.m.status.fault == F_STOP_TIMEOUT);
}

static void bus_tests(void)
{
    rig_t r;
    robot_bus_t b;
    robot_request_t request;
    unsigned i;
    init(&r); robot_bus_init(&b, &r.m.status);
    request.source = SOURCE_LINE; request.command = command(&r, WHEEL_SPEED_TRACKING, 0.0);
    for (i = 0; i < 20U; ++i) { CHECK(robot_request(&b, &request) == R_OK); }
    CHECK(b.count == 1U);
    vehicle_controller_cycle(&b, r.now); CHECK(b.approved_valid);
    robot_control(&b, CONTROL_STOP, false, false);
    CHECK(!b.approved_valid && b.count == 0U);
    robot_motion_inputs(&b, &r.m, r.sample, r.now);
    CHECK(r.m.status.state == STOPPED);
    CHECK(robot_request(&b, &request) == R_STALE);
    init(&r); robot_bus_init(&b, &r.m.status);
    request.command = command(&r, DISTANCE_MOVE, 1.0);
    for (i = 0; i < 8U; ++i) { request.command.id = i + 1U; CHECK(robot_request(&b, &request) == R_OK); }
    CHECK(robot_request(&b, &request) == R_FULL);
    robot_control(&b, CONTROL_STOP, false, false);
    robot_control(&b, CONTROL_FAULT, false, false);
    robot_motion_inputs(&b, &r.m, r.sample, r.now);
    CHECK(r.m.status.state == FAULT && r.m.status.fault == F_SYSTEM);
    init(&r); robot_bus_init(&b, &r.m.status); b.event_count = 8U;
    motion_raise_fault(&r.m, F_FEEDBACK, r.now); robot_publish(&b, &r.m, r.now);
    CHECK(b.status.dropped_events == 1U && r.m.status.inhibited && r.m.status.fault == F_FEEDBACK);
    init(&r); r.m.event_count = 8U;
    motion_raise_fault(&r.m, F_SYSTEM, r.now);
    CHECK(r.m.status.state == FAULT && r.m.status.inhibited);
}

static void validation_tests(void)
{
    rig_t r;
    motion_command_t c;
    motion_config_t config = simulation_config();
    robot_bus_t b;
    robot_request_t request;
    unsigned i;
    CHECK(board_pins_valid());
    config.pwm_hz = 20001U; CHECK(!motion_config_valid(&config));
    config = simulation_config(); config.motor_sign[0] = 0; CHECK(!motion_config_valid(&config));
    config = simulation_config(); config.pid[1].ki = INFINITY; CHECK(!motion_config_valid(&config));
    init(&r);
    c = command(&r, DISTANCE_MOVE, NAN); CHECK(motion_accept(&r.m, &c, r.now) == R_INVALID);
    c = command(&r, (motion_mode_t)-1, 0.0); CHECK(motion_accept(&r.m, &c, r.now) == R_INVALID);
    c = command(&r, DISTANCE_MOVE, 6.0); CHECK(motion_accept(&r.m, &c, r.now) == R_INVALID);
    c = command(&r, WHEEL_SPEED_TRACKING, 0.0); c.speed[0] = INFINITY;
    CHECK(motion_accept(&r.m, &c, r.now) == R_INVALID);
    c = command(&r, WHEEL_SPEED_TRACKING, 0.0); c.speed[0] = 0.26;
    CHECK(motion_accept(&r.m, &c, r.now) == R_INVALID);
    c = command(&r, DISTANCE_MOVE, 0.2); c.generation--;
    CHECK(motion_accept(&r.m, &c, r.now) == R_STALE);
    c = command(&r, DISTANCE_MOVE, 0.2); c.expires = r.now;
    CHECK(motion_accept(&r.m, &c, r.now) == R_STALE);
    c = command(&r, MANUAL_OUTPUT, 0.0); c.deadline = r.now + 0.4;
    CHECK(motion_accept(&r.m, &c, r.now) == R_INVALID);
    r.m.config.manual_enabled = true; c.speed[0] = 0.1; c.speed[1] = 0.0;
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i = 0; i < 90U; ++i) { tick(&r, true); }
    CHECK(r.m.status.state == STOPPED && r.m.status.hold && r.m.status.output[1] == 0.0);
    init(&r); c = command(&r, WHEEL_SPEED_TRACKING, 0.0); c.speed[0] = 0.0; c.speed[1] = 0.0;
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i = 0; i < 200U; ++i) {
        c.expires = r.now + 0.25; c.deadline = r.now + 20.0;
        CHECK(motion_accept(&r.m, &c, r.now) == R_OK); tick(&r, false);
    }
    CHECK(r.m.status.state == DRIVING && r.m.status.fault == F_NONE);
    c.expires -= 0.01; CHECK(motion_accept(&r.m, &c, r.now) == R_STALE);
    c = command(&r, DISTANCE_MOVE, 0.2); CHECK(motion_accept(&r.m, &c, r.now) == R_BUSY);
    init(&r); robot_bus_init(&b, &r.m.status);
    request.source = SOURCE_LINE; request.command = command(&r, WHEEL_SPEED_TRACKING, 0.0);
    CHECK(robot_request(&b, &request) == R_OK);
    request.command.expires -= 0.01; CHECK(robot_request(&b, &request) == R_STALE);
    vehicle_controller_cycle(&b, r.now); robot_motion_inputs(&b, &r.m, r.sample, r.now);
    robot_publish(&b, &r.m, r.now);
    request.source = SOURCE_OBSTACLE; request.command.id = 2U;
    CHECK(robot_request(&b, &request) == R_OK); vehicle_controller_cycle(&b, r.now);
    CHECK(!b.approved_valid && b.status.rejected_requests == 1U);
    robot_control(&b, CONTROL_SHUTDOWN, false, false);
    robot_control(&b, CONTROL_RESET, true, true);
    robot_motion_inputs(&b, &r.m, r.sample, r.now);
    CHECK(r.m.status.state == SHUTDOWN);
    init(&r); config = simulation_config();
    motion_init(&r.m, &config, r.sample, r.now, false); CHECK(r.m.status.fault == F_SETUP);
    motion_init(&r.m, &config, r.sample, r.now, true);
    for (i = 0; i < 210U; ++i) { r.sample.count[0] += 2U; tick(&r, false); }
    CHECK(r.m.status.fault == F_SETUP);
    init(&r); c = command(&r, ENCODER_TURN, 0.0);
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i = 0; i < 50U; ++i) { tick(&r, false); }
    CHECK(r.m.status.state == READY);
}

static void low_speed_feedback_tests(void)
{
    rig_t r;
    motion_command_t c;
    unsigned i;
    init(&r); c = command(&r, WHEEL_SPEED_TRACKING, 0.0);
    c.speed[0] = 0.001; c.speed[1] = 0.0; /* Nonzero below the stationary threshold. */
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i = 0; i < 100U && r.m.status.state == DRIVING; ++i) {
        c.expires = r.now + 0.25; c.deadline = r.now + 20.0;
        CHECK(motion_accept(&r.m, &c, r.now) == R_OK); tick(&r, false);
    }
    CHECK(r.m.status.fault == F_FEEDBACK);
    init(&r); r.m.config.counts_per_rev[0] = 20.0;
    c = command(&r, WHEEL_SPEED_TRACKING, 0.0); c.speed[0] = 0.01; c.speed[1] = 0.0;
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i = 0; i < 150U; ++i) {
        c.expires = r.now + 0.25; c.deadline = r.now + 20.0;
        CHECK(motion_accept(&r.m, &c, r.now) == R_OK); tick(&r, false);
    }
    CHECK(r.m.status.state == DRIVING); /* Coarse encoder legitimately needs more time. */
    for (i = 0; i < 150U && r.m.status.state == DRIVING; ++i) {
        c.expires = r.now + 0.25; c.deadline = r.now + 20.0;
        CHECK(motion_accept(&r.m, &c, r.now) == R_OK); tick(&r, false);
    }
    CHECK(r.m.status.fault == F_FEEDBACK);
}

static void capture_bank0_event(encoder_capture_t *capture, uint32_t event, uint32_t levels)
{
    const uint32_t pending[4] = {event, 0U, 0U, 0U};
    encoder_capture_events(capture, pending, levels);
}

/* Exercise the actual ISR capture path with RP2040 GPIO event bits. */
static void encoder_capture_tests(void)
{
    const unsigned pins[2][2] = {{4U, 6U}, {5U, 7U}};
    const bool quadrature[2] = {true, true};
    const int sign[2] = {1, 1};
    const unsigned forward[5] = {0U, 1U, 3U, 2U, 0U};
    const unsigned reverse[5] = {0U, 2U, 3U, 1U, 0U};
    encoder_capture_t capture;
    unsigned wheel, direction, step;
    {
        const unsigned moved[2][2] = {{4U, 6U}, {BOARD_ENC_R_A, 7U}};
        for (direction = 0U; direction < 2U; ++direction) {
            const unsigned *sequence = direction == 0U ? forward : reverse;
            encoder_capture_init(&capture, moved, quadrature, sign, 0U);
            for (step = 1U; step < 5U; ++step) {
                unsigned channel = (sequence[step] ^ sequence[step - 1U]) == 2U ? 0U : 1U;
                unsigned pin = moved[1][channel];
                uint32_t pending[4] = {0U, 0U, 0U, 0U};
                uint32_t levels = ((sequence[step] >> 1U) << BOARD_ENC_R_A) | ((sequence[step] & 1U) << 7U);
                pending[pin / 8U] = ((levels & (1U << pin)) != 0U ? 8U : 4U) << ((pin % 8U) * 4U);
                encoder_capture_events(&capture, pending, levels);
                CHECK(capture.count[0] == 0U && capture.invalid[0] == 0U);
                CHECK(capture.edges[0][0] == 0U && capture.edges[0][1] == 0U);
            }
            CHECK(capture.count[1] == (direction == 0U ? 4U : UINT32_MAX - 3U));
            CHECK(capture.invalid[1] == 0U);
            CHECK(capture.edges[1][0] == 2U && capture.edges[1][1] == 2U);
        }
    }
    for (wheel = 0; wheel < 2U; ++wheel) {
        for (direction = 0; direction < 2U; ++direction) {
            const unsigned *sequence = direction == 0U ? forward : reverse;
            encoder_capture_init(&capture, pins, quadrature, sign, 0U);
            for (step = 1; step < 5U; ++step) {
                unsigned changed = sequence[step] ^ sequence[step - 1U];
                unsigned channel = changed == 2U ? 0U : 1U;
                unsigned pin = pins[wheel][channel];
                uint32_t levels = ((sequence[step] >> 1U) << pins[wheel][0]) |
                                  ((sequence[step] & 1U) << pins[wheel][1]);
                uint32_t event = ((levels & (1U << pin)) != 0U ? 8U : 4U) << (pin * 4U);
                capture_bank0_event(&capture, event, levels);
                CHECK(capture.count[1U - wheel] == 0U);
                CHECK(capture.invalid[1U - wheel] == 0U);
                CHECK(capture.previous_ab[1U - wheel] == 0U);
                CHECK(capture.edges[1U - wheel][0] == 0U && capture.edges[1U - wheel][1] == 0U);
            }
            CHECK(capture.count[wheel] == (direction == 0U ? 4U : UINT32_MAX - 3U));
            CHECK(capture.invalid[wheel] == 0U);
            CHECK(capture.edges[wheel][0] == 2U && capture.edges[wheel][1] == 2U);
        }
    }
    encoder_capture_init(&capture, pins, quadrature, sign, (1U << 4U) | (1U << 7U));
    CHECK(capture.previous_ab[0] == 2U && capture.previous_ab[1] == 1U);
    capture_bank0_event(&capture, 4U << (7U * 4U), 1U << 4U);
    CHECK(capture.count[1] == UINT32_MAX && capture.count[0] == 0U);
    encoder_capture_init(&capture, pins, quadrature, sign, 0U);
    capture_bank0_event(&capture, (8U << (5U * 4U)) | (8U << (7U * 4U)),
                           (1U << 5U) | (1U << 7U));
    CHECK(capture.invalid[1] == 1U && capture.invalid[0] == 0U);
    CHECK(capture.count[0] == 0U && capture.count[1] == 0U);
    CHECK(capture.previous_ab[1] == 3U);
    encoder_capture_init(&capture, pins, quadrature, sign, 0U);
    capture_bank0_event(&capture, (8U << (6U * 4U)) | (8U << (7U * 4U)),
                           (1U << 6U) | (1U << 7U));
    CHECK(capture.count[0] == 1U && capture.count[1] == 1U);
    CHECK(capture.invalid[0] == 0U && capture.invalid[1] == 0U);
    capture.count[1] = UINT32_MAX;
    capture_bank0_event(&capture, 8U << (5U * 4U), (1U << 5U) | (1U << 7U));
    CHECK(capture.count[1] == 0U && capture.count[0] == 1U);
    capture_bank0_event(&capture, 8U, 0U); /* Unowned GPIO0; leave both states intact. */
    CHECK(capture.previous_ab[0] == 1U && capture.previous_ab[1] == 3U);
    {
        const int inverted[2] = {1, -1};
        const bool single[2] = {true, false};
        encoder_capture_init(&capture, pins, quadrature, inverted, 0U);
        capture_bank0_event(&capture, 8U << (7U * 4U), 1U << 7U);
        CHECK(capture.count[1] == UINT32_MAX);
        encoder_capture_init(&capture, pins, single, sign, 0U);
        capture.inferred_direction[1] = -1;
        capture_bank0_event(&capture, 8U << (7U * 4U), 1U << 7U);
        CHECK(capture.count[1] == 0U && capture.edges[1][1] == 0U);
        capture_bank0_event(&capture, 8U << (5U * 4U), 1U << 5U);
        CHECK(capture.count[1] == UINT32_MAX && capture.count[0] == 0U);
    }
}

static void production_line_integration_tests(void)
{
    rig_t r;
    robot_bus_t bus;
    robot_request_t request;
    motion_command_t manual;
    unsigned i;
    memset(&r, 0, sizeof(r));
    motion_config_t config = board_motion_config();
    CHECK(config.hardware_confirmed && !config.manual_enabled);
    CHECK(config.max_output == 0.25 && motion_config_valid(&config));
    r.sample.reliable = true;
    motion_init(&r.m, &config, r.sample, 0.0, true);
    CHECK(r.m.status.state == INITIALISING && r.m.status.inhibited);
    for (i = 0U; i < 40U; ++i) {
        tick(&r, false);
        CHECK(r.m.status.output[0] == 0.0 && r.m.status.output[1] == 0.0);
    }
    CHECK(r.m.status.state == READY && r.m.status.fault == F_NONE);
    manual = command(&r, MANUAL_OUTPUT, 0.0);
    manual.deadline = r.now + config.manual_timeout;
    manual.speed[0] = 0.1; manual.speed[1] = 0.1;
    CHECK(motion_accept(&r.m, &manual, r.now) == R_INVALID);
    robot_bus_init(&bus, &r.m.status);
    request.source = SOURCE_LINE;
    request.command = command(&r, WHEEL_SPEED_TRACKING, 0.0);
    request.command.id = 3001U;
    request.command.speed[0] = 0.08; request.command.speed[1] = 0.06;
    for (i = 0U; i < 50U; ++i) {
        request.command.expires = r.now + config.command_lifetime;
        request.command.deadline = r.now + config.movement_timeout;
        CHECK(robot_request(&bus, &request) == R_OK);
        vehicle_controller_cycle(&bus, r.now);
        robot_motion_inputs(&bus, &r.m, r.sample, r.now);
        tick(&r, true);
        robot_publish(&bus, &r.m, r.now);
        CHECK(r.m.status.state == DRIVING && r.m.status.fault == F_NONE);
        CHECK(fabs(r.m.status.output[0]) <= 0.25 && fabs(r.m.status.output[1]) <= 0.25);
        CHECK(bus.status.owner_valid && bus.status.owner == SOURCE_LINE);
    }
    robot_control(&bus, CONTROL_STOP, false, false);
    robot_motion_inputs(&bus, &r.m, r.sample, r.now);
    CHECK(r.m.status.inhibited && r.m.status.output[0] == 0.0 && r.m.status.output[1] == 0.0);
    CHECK(robot_request(&bus, &request) != R_OK);
    for (i = 0U; i < 80U; ++i) { tick(&r, true); }
    CHECK(r.m.status.state == STOPPED && r.m.status.hold);
    config.hardware_confirmed = false;
    motion_init(&r.m, &config, r.sample, r.now, true);
    CHECK(r.m.status.state == FAULT && r.m.status.fault == F_CONFIG && r.m.status.inhibited);
}

int main(void)
{
    math_tests(); movement_tests(); safety_tests(); correction_and_final_tests(); bus_tests(); validation_tests();
    low_speed_feedback_tests();
    encoder_capture_tests();
    production_line_integration_tests();
    printf("PASS: %u checks (synthetic host model; no hardware accuracy claim)\n", checks);
    return 0;
}

