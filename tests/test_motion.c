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
                          r->now + 0.25, r->now + 20.0, 0.0, 0.0};
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
    init(&r); r.m.event_queue = (uint8_t)(8u << 3u);
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
                CHECK(encoder_capture_previous_ab(&capture, 1U - wheel) == 0U);
                CHECK(capture.edges[1U - wheel][0] == 0U && capture.edges[1U - wheel][1] == 0U);
            }
            CHECK(capture.count[wheel] == (direction == 0U ? 4U : UINT32_MAX - 3U));
            CHECK(capture.invalid[wheel] == 0U);
            CHECK(capture.edges[wheel][0] == 2U && capture.edges[wheel][1] == 2U);
        }
    }
    encoder_capture_init(&capture, pins, quadrature, sign, (1U << 4U) | (1U << 7U));
    CHECK(encoder_capture_previous_ab(&capture, 0) == 2U && encoder_capture_previous_ab(&capture, 1) == 1U);
    capture_bank0_event(&capture, 4U << (7U * 4U), 1U << 4U);
    CHECK(capture.count[1] == UINT32_MAX && capture.count[0] == 0U);
    encoder_capture_init(&capture, pins, quadrature, sign, 0U);
    capture_bank0_event(&capture, (8U << (5U * 4U)) | (8U << (7U * 4U)),
                           (1U << 5U) | (1U << 7U));
    CHECK(capture.invalid[1] == 1U && capture.invalid[0] == 0U);
    CHECK(capture.count[0] == 0U && capture.count[1] == 0U);
    CHECK(encoder_capture_previous_ab(&capture, 1) == 3U);
    encoder_capture_init(&capture, pins, quadrature, sign, 0U);
    capture_bank0_event(&capture, (8U << (6U * 4U)) | (8U << (7U * 4U)),
                           (1U << 6U) | (1U << 7U));
    CHECK(capture.count[0] == 1U && capture.count[1] == 1U);
    CHECK(capture.invalid[0] == 0U && capture.invalid[1] == 0U);
    capture.count[1] = UINT32_MAX;
    capture_bank0_event(&capture, 8U << (5U * 4U), (1U << 5U) | (1U << 7U));
    CHECK(capture.count[1] == 0U && capture.count[0] == 1U);
    capture_bank0_event(&capture, 8U, 0U); /* Unowned GPIO0; leave both states intact. */
    CHECK(encoder_capture_previous_ab(&capture, 0) == 1U && encoder_capture_previous_ab(&capture, 1) == 3U);
    {
        const int inverted[2] = {1, -1};
        const bool single[2] = {true, false};
        encoder_capture_init(&capture, pins, quadrature, inverted, 0U);
        capture_bank0_event(&capture, 8U << (7U * 4U), 1U << 7U);
        CHECK(capture.count[1] == UINT32_MAX);
        encoder_capture_init(&capture, pins, single, sign, 0U);
        encoder_capture_set_direction(&capture, 1u, -1);
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
    robot_publish(&bus, &r.m, r.now);
    vehicle_controller_cycle(&bus, r.now);
    CHECK(vehicle_controller_remote_command(&bus, (robot_remote_command_t)99) == R_INVALID);
    CHECK(vehicle_controller_remote_command(&bus, REMOTE_START) == R_OK);
    robot_motion_inputs(&bus, &r.m, r.sample, r.now);
    CHECK(r.m.status.state == READY && !r.m.status.hold);
    robot_publish(&bus, &r.m, r.now);
    CHECK(vehicle_controller_remote_command(&bus, REMOTE_STOP) == R_OK);
    CHECK(vehicle_controller_remote_command(&bus, REMOTE_START) == R_HELD);
    CHECK(vehicle_controller_remote_command(&bus, REMOTE_ESTOP) == R_OK);
    robot_motion_inputs(&bus, &r.m, r.sample, r.now);
    CHECK(r.m.status.state == FAULT && r.m.status.fault == F_SYSTEM && r.m.status.inhibited);
    robot_publish(&bus, &r.m, r.now);
    CHECK(vehicle_controller_remote_command(&bus, REMOTE_START) == R_HELD);
    config.hardware_confirmed = false;
    motion_init(&r.m, &config, r.sample, r.now, true);
    CHECK(r.m.status.state == FAULT && r.m.status.fault == F_CONFIG && r.m.status.inhibited);
}

static void arc_tests(void)
{
    rig_t r;
    motion_command_t c;
    unsigned i, direction;
    for (direction = 0U; direction < 3U; ++direction) {
        init(&r);
        c = command(&r, ARC_MOVE, direction == 1U ? -0.12 : 0.20);
        c.angle = direction == 2U ? 0.0 : (direction == 1U ? -0.1745329252 : 0.1745329252);
        for (i = 0U; i < 2U; ++i) {
            double goal = c.value + (i == 0U ? -1.0 : 1.0) * c.angle * r.m.config.track / 2.0;
            c.speed[i] = goal * 0.08 / fabs(c.value);
        }
        CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
        for (i = 0U; i < 1900U && (r.m.status.state == DRIVING || r.m.status.state == STOPPING); ++i) {
            tick(&r, true);
            CHECK(fabs(r.m.status.output[0]) <= r.m.config.max_output);
            CHECK(fabs(r.m.status.output[1]) <= r.m.config.max_output);
        }
        CHECK(r.m.status.state == READY && r.m.status.fault == F_NONE);
        NEAR(r.m.status.centre, c.value, r.m.config.distance_tolerance);
        NEAR(r.m.status.heading, c.angle, r.m.config.angle_tolerance);
        CHECK(r.m.status.sample_time == r.now);
    }
    init(&r); c = command(&r, ARC_MOVE, 0.12);
    c.angle = 0.1745329252; c.duration = 3.0;
    for (i = 0U; i < 2U; ++i) {
        c.speed[i] = (c.value + (i == 0U ? -1.0 : 1.0) * c.angle * r.m.config.track/2.0) / c.duration;
    }
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    /* Encoder samples follow a known smooth trajectory; no accuracy claim for a real plant. */
    for (i = 1U; i <= 350U; ++i) {
        double u = fmin(1.0, (double)i * 0.01 / c.duration);
        double progress = u*u*(3.0-2.0*u);
        for (unsigned w = 0U; w < 2U; ++w) {
            double quantum = 3.141592653589793 * r.m.config.diameter[w] / r.m.config.counts_per_rev[w];
            double goal = c.value + (w == 0U ? -1.0 : 1.0) * c.angle * r.m.config.track/2.0;
            r.sample.count[w] = (uint32_t)(int32_t)llround(goal * progress / quantum);
        }
        tick(&r, false);
        if (i < 300U) { CHECK(r.m.status.state == DRIVING); }
    }
    CHECK(r.m.status.state == READY && r.m.status.fault == F_NONE);
    NEAR(r.m.status.centre, c.value, r.m.config.distance_tolerance);
    NEAR(r.m.status.heading, c.angle, r.m.config.angle_tolerance);
    /* A timed command must not report completion if encoder goals were missed. */
    init(&r); c.generation = r.m.status.generation; c.expires=r.now+0.25; c.deadline=r.now+20.0;
    r.m.config.feedback_timeout=5.0;
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i = 0U; i < 350U; ++i) { tick(&r, false); }
    CHECK(r.m.status.state == FAULT && r.m.status.fault == F_FINAL_GOAL && r.m.status.inhibited);
    init(&r); c = command(&r, ARC_MOVE, 0.2); c.angle=0.1;
    c.speed[0]=-0.08;
    CHECK(motion_accept(&r.m, &c, r.now) == R_INVALID);
    c.speed[0]=0.08; c.angle=NAN;
    CHECK(motion_accept(&r.m, &c, r.now) == R_INVALID);
    c.angle=0.1; c.duration=-1.0;
    CHECK(motion_accept(&r.m, &c, r.now) == R_INVALID);
    c.duration=0.1;
    CHECK(motion_accept(&r.m, &c, r.now) == R_INVALID);
    c.duration=0.0;
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    motion_cancel(&r.m, r.now);
    CHECK(r.m.status.hold && r.m.status.inhibited);
    /* The new mode uses normal source ownership; line steering cannot replace an obstacle arc. */
    robot_bus_t b; robot_request_t request;
    init(&r); robot_bus_init(&b, &r.m.status);
    request=(robot_request_t){.source=SOURCE_OBSTACLE, .command=command(&r, ARC_MOVE, 0.2)};
    CHECK(robot_request(&b, &request) == R_OK);
    vehicle_controller_cycle(&b, r.now); robot_motion_inputs(&b, &r.m, r.sample, r.now);
    robot_publish(&b, &r.m, r.now);
    CHECK(b.status.owner_valid && b.status.owner == SOURCE_OBSTACLE);
    request.source=SOURCE_LINE; request.command=command(&r, WHEEL_SPEED_TRACKING, 0.0);
    CHECK(robot_request(&b, &request) == R_OK);
    vehicle_controller_cycle(&b, r.now);
    CHECK(!b.approved_valid && b.status.rejected_requests > 0U);
    /* Unequal motor gains: equal wheel-speed requests converge through PID.
     * A steering refresh preserves PID history and changes targets gradually. */
    init(&r); c=command(&r, WHEEL_SPEED_TRACKING, 0.0); c.speed[0]=0.08; c.speed[1]=0.08;
    CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
    for (i=0U; i<800U; ++i) {
        double old_target[2]={r.m.status.target[0],r.m.status.target[1]};
        c.expires=r.now+0.25; c.deadline=r.now+20.0;
        CHECK(motion_accept(&r.m, &c, r.now) == R_OK);
        tick(&r, true);
        CHECK(r.m.status.state == DRIVING && r.m.status.fault == F_NONE);
        CHECK(fabs(r.m.status.target[0]-old_target[0]) <= r.m.config.acceleration*0.01+1e-12);
        CHECK(fabs(r.m.status.target[1]-old_target[1]) <= r.m.config.acceleration*0.01+1e-12);
    }
    NEAR(r.m.status.speed[0],0.08,0.015); NEAR(r.m.status.speed[1],0.08,0.015);
    NEAR(r.m.status.speed[0],r.m.status.speed[1],0.01);
    double integral=r.m.pid[0].integral;
    c.speed[0]=0.07; c.speed[1]=0.09; c.expires=r.now+0.25; c.deadline=r.now+20.0;
    CHECK(motion_accept(&r.m,&c,r.now) == R_OK && r.m.pid[0].integral == integral);
    double left_before=r.m.status.target[0], right_before=r.m.status.target[1];
    tick(&r,true);
    CHECK(r.m.status.target[0] < left_before && r.m.status.target[1] > right_before);
    CHECK(fabs(r.m.status.target[0]-left_before) <= r.m.config.acceleration*0.01+1e-12);
    CHECK(fabs(r.m.status.target[1]-right_before) <= r.m.config.acceleration*0.01+1e-12);
}

static void startup_sampling_tests(void)
{
    rig_t r={0};
    motion_config_t c=simulation_config();
    r.sample.reliable=true;
    motion_init(&r.m,&c,r.sample,0.0,true);
    r.now=0.15; r.sample.count[0]=100U; r.sample.count[1]=200U;
    motion_begin_sampling(&r.m,r.sample,r.now);
    CHECK(r.m.status.state==INITIALISING && r.m.status.fault==F_NONE && r.m.status.inhibited);
    CHECK(r.m.last_time==r.now && r.m.odom.previous[0]==100U && r.m.odom.previous[1]==200U);
    for (unsigned i=0U;i<40U;++i) { tick(&r,false); }
    CHECK(r.m.status.state==READY && r.m.status.stationary);
    NEAR(r.m.status.centre,0.0,1e-12);
    motion_command_t command_value=command(&r,WHEEL_SPEED_TRACKING,0.0);
    CHECK(motion_accept(&r.m,&command_value,r.now)==R_OK);
    double last_time=r.m.last_time;
    motion_begin_sampling(&r.m,r.sample,r.now+1.0);
    CHECK(r.m.last_time==last_time); /* Cannot rebase an active command. */
    r.now+=0.06; motion_step(&r.m,r.sample,r.now);
    CHECK(r.m.status.state==FAULT && r.m.status.fault==F_TIMING && r.m.status.inhibited);
    CHECK(r.m.status.timing_fault_recorded && r.m.status.timing_fault_state==DRIVING);
    NEAR(r.m.status.timing_fault_dt,0.06,1e-12);
    NEAR(r.m.status.max_observed_dt,0.06,1e-12);
    r.now+=0.12; motion_step(&r.m,r.sample,r.now);
    NEAR(r.m.status.timing_fault_dt,0.06,1e-12); /* Retain first failure after later gaps. */
    NEAR(r.m.status.last_dt,0.12,1e-12);
    NEAR(r.m.status.max_observed_dt,0.12,1e-12);
    motion_init(&r.m,&c,r.sample,0.0,true);
    CHECK(!r.m.status.timing_fault_recorded);
    motion_step(&r.m,r.sample,0.0);
    CHECK(r.m.status.fault==F_TIMING && r.m.status.timing_fault_recorded);
    CHECK(r.m.status.timing_fault_dt==0.0 && r.m.status.timing_fault_state==INITIALISING);
    c.hardware_confirmed=false;
    motion_init(&r.m,&c,r.sample,0.0,true);
    motion_begin_sampling(&r.m,r.sample,0.15);
    CHECK(r.m.status.state==FAULT && r.m.status.fault==F_CONFIG);
    c.hardware_confirmed=true;
    motion_init(&r.m,&c,r.sample,0.0,false);
    motion_begin_sampling(&r.m,r.sample,0.15);
    CHECK(r.m.status.state==FAULT && r.m.status.fault==F_SETUP);
    motion_init(&r.m,&c,r.sample,0.0,true);
    ++r.sample.invalid[1];
    motion_begin_sampling(&r.m,r.sample,0.15);
    CHECK(r.m.status.state==FAULT && r.m.status.fault==F_FEEDBACK);
    motion_init(&r.m,&c,r.sample,0.1,true);
    motion_begin_sampling(&r.m,r.sample,0.0);
    CHECK(r.m.status.state==FAULT && r.m.status.fault==F_TIMING);
    CHECK(r.m.status.timing_fault_recorded && r.m.status.timing_fault_state==INITIALISING);
    NEAR(r.m.status.timing_fault_dt,-0.1,1e-12);
}

static void startup_gap_tests(void)
{
    rig_t r={0};
    motion_config_t c=simulation_config();
    r.sample.reliable=true;
    motion_init(&r.m,&c,r.sample,0.0,true);
    r.now=0.142231; motion_step(&r.m,r.sample,r.now); /* Captured robot startup gap. */
    CHECK(r.m.status.state==INITIALISING && r.m.status.fault==F_NONE);
    CHECK(r.m.status.inhibited && !r.m.status.stationary && !r.m.status.feedback_trusted);
    CHECK(r.m.status.output[0]==0.0 && r.m.status.output[1]==0.0);
    CHECK(r.m.status.startup_gaps==1U && !r.m.status.timing_fault_recorded);
    NEAR(r.m.status.max_startup_gap,0.142231,1e-12);
    CHECK(r.m.entered==0.0); /* Startup deadline is not extended. */
    motion_command_t move=command(&r,WHEEL_SPEED_TRACKING,0.0);
    CHECK(motion_accept(&r.m,&move,r.now)==R_HELD);
    for(unsigned i=0U;i<20U;++i) { tick(&r,false); }
    CHECK(r.m.status.state==INITIALISING && r.m.status.inhibited);
    r.now+=0.10; motion_step(&r.m,r.sample,r.now);
    CHECK(r.m.status.startup_gaps==2U && !r.m.status.stationary);
    for(unsigned i=0U;i<20U;++i) { tick(&r,false); }
    CHECK(r.m.status.state==INITIALISING); /* Rest settling restarts after each gap. */
    for(unsigned i=0U;i<20U;++i) { tick(&r,false); }
    CHECK(r.m.status.state==READY && r.m.status.feedback_trusted && r.m.status.stationary);
    r.now+=0.06; motion_step(&r.m,r.sample,r.now);
    CHECK(r.m.status.fault==F_TIMING && r.m.status.timing_fault_state==READY);
    CHECK(r.m.status.inhibited && r.m.status.hold);

    motion_init(&r.m,&c,r.sample,0.0,true);
    for(unsigned i=1U;i<=15U;++i) { r.now=(double)i*0.15; motion_step(&r.m,r.sample,r.now); }
    CHECK(r.m.status.fault==F_SETUP && r.m.status.inhibited);
    CHECK(r.m.entered>=2.0); /* Fault time; no indefinite initialization grace. */

    motion_init(&r.m,&c,r.sample,0.0,true);
    ++r.sample.invalid[0]; motion_step(&r.m,r.sample,0.142231);
    CHECK(r.m.status.fault==F_FEEDBACK && r.m.status.inhibited);
    --r.sample.invalid[0];
    motion_init(&r.m,&c,r.sample,0.0,true);
    r.sample.reliable=false; motion_step(&r.m,r.sample,0.142231);
    CHECK(r.m.status.fault==F_FEEDBACK);
    r.sample.reliable=true;
    motion_init(&r.m,&c,r.sample,0.1,true);
    motion_step(&r.m,r.sample,0.0);
    CHECK(r.m.status.fault==F_TIMING); /* Backwards clocks never qualify as startup gaps. */
}

static void feedback_diagnostic_tests(void)
{
    rig_t r;
    init(&r);
    r.sample.invalid[0]=1U;
    tick(&r,false);
    CHECK(r.m.status.fault==F_FEEDBACK && r.m.status.feedback_cause==FB_INVALID_TRANSITION);
    CHECK(r.m.status.feedback_fault_sample.invalid[0]==1U && r.m.status.feedback_fault_sample.invalid[1]==0U);
    CHECK(r.m.status.feedback_previous_invalid[0]==0U && r.m.status.feedback_previous_invalid[1]==0U);
    r.sample.invalid[1]=5U; tick(&r,false);
    CHECK(r.m.status.feedback_fault_sample.invalid[1]==0U); /* Preserve the triggering sample. */
    CHECK(r.m.status.output[0]==0.0 && r.m.status.output[1]==0.0 && r.m.status.hold);

    init(&r); CHECK(r.m.status.feedback_cause==FB_NONE);
    r.sample.count[1]=100000U; tick(&r,false);
    CHECK(r.m.status.fault==F_FEEDBACK && r.m.status.feedback_cause==FB_ODOMETRY);
    CHECK(r.m.status.feedback_fault_sample.count[1]==100000U && r.m.status.feedback_previous_count[1]==0U);
    CHECK(r.m.status.feedback_fault_sample.reliable && r.m.status.feedback_fault_sample.invalid[1]==0U);

    init(&r); r.sample.reliable=false; tick(&r,false);
    CHECK(r.m.status.fault==F_FEEDBACK && r.m.status.feedback_cause==FB_UNRELIABLE);
    CHECK(!r.m.status.feedback_fault_sample.reliable && r.m.status.inhibited);
}

int main(void)
{
    math_tests(); movement_tests(); safety_tests(); correction_and_final_tests(); bus_tests(); validation_tests();
    low_speed_feedback_tests();
    encoder_capture_tests();
    production_line_integration_tests();
    arc_tests();
    startup_sampling_tests();
    startup_gap_tests();
    feedback_diagnostic_tests();
    printf("PASS: %u checks (synthetic host model; no hardware accuracy claim)\n", checks);
    return 0;
}

