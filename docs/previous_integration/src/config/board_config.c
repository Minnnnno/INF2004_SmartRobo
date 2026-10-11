#include "motion.h"
#include "board_config.h"
#include <math.h>

const unsigned motor_pins[2][2] = {{8U, 9U}, {10U, 11U}};
const unsigned encoder_pins[2][2] = {{4U, 6U}, {5U, 7U}};

bool board_pins_valid(void)
{
    unsigned i;
    unsigned j;
    const unsigned enc[] = {encoder_pins[0][0], encoder_pins[0][1],
                            encoder_pins[1][0], encoder_pins[1][1]};
    if ((motor_pins[0][0] != 8U) || (motor_pins[0][1] != 9U) ||
        (motor_pins[1][0] != 10U) || (motor_pins[1][1] != 11U)) { return false; }
    for (i = 0; i < 4U; ++i) {
        if ((enc[i] < 4U) || (enc[i] > 7U)) { return false; }
        for (j = 0; j < i; ++j) { if (enc[i] == enc[j]) { return false; } }
    }
    return true;
}

motion_config_t board_motion_config(void)
{
    motion_config_t config = {
        /* Mandatory measurements: zero deliberately inhibits normal motion. */
        .counts_per_rev = {0.0, 0.0}, .diameter = {0.0, 0.0}, .track = 0.0,
        .motor_sign = {1, 1}, .encoder_sign = {1, 1}, .quadrature = {true, true},
        /* Provisional tuning only; validate on the actual robot. */
        .max_speed = 0.25, .acceleration = 0.3, .max_output = 0.35,
        .period = 0.01, .max_dt = 0.05,
        .distance_tolerance = 0.01, .angle_tolerance = 0.05,
        .stopped_speed = 0.005, .settle_time = 0.3,
        .startup_grace = 0.4, .feedback_timeout = 0.4,
        .stop_timeout = 3.0, .setup_timeout = 2.0,
        .command_lifetime = 0.3, .movement_timeout = 30.0,
        .max_distance = 5.0, .max_angle = 6.283185307179586,
        .straight_gain = 1.0, .reversal_time = 0.1,
        .manual_limit = 0.15, .manual_timeout = 0.5,
        .pid = {{1.0, 0.5, 0.0, 0.05}, {1.0, 0.5, 0.0, 0.05}},
        .pwm_hz = 20000U, .hardware_confirmed = false,
        .brake = true, .manual_enabled = false
    };
    return config;
}

bool motion_config_valid(const motion_config_t *c)
{
    unsigned wheel;
    const double positive[] = {c->track, c->max_speed, c->acceleration, c->max_output,
        c->period, c->max_dt, c->distance_tolerance, c->angle_tolerance,
        c->stopped_speed, c->settle_time, c->startup_grace, c->feedback_timeout,
        c->stop_timeout, c->setup_timeout, c->command_lifetime, c->movement_timeout,
        c->max_distance, c->max_angle, c->reversal_time, c->manual_limit, c->manual_timeout};
    for (wheel = 0; wheel < sizeof(positive) / sizeof(positive[0]); ++wheel) {
        if (!isfinite(positive[wheel]) || (positive[wheel] <= 0.0)) { return false; }
    }
    for (wheel = 0; wheel < 2U; ++wheel) {
        const pid_config_t *p = &c->pid[wheel];
        if (!isfinite(c->counts_per_rev[wheel]) || (c->counts_per_rev[wheel] < 1.0) ||
            !isfinite(c->diameter[wheel]) || (c->diameter[wheel] <= 0.0) ||
            ((c->motor_sign[wheel] != 1) && (c->motor_sign[wheel] != -1)) ||
            ((c->encoder_sign[wheel] != 1) && (c->encoder_sign[wheel] != -1)) ||
            !isfinite(p->kp) || !isfinite(p->ki) || !isfinite(p->kd) ||
            !isfinite(p->derivative_tau) || (p->kp < 0.0) || (p->ki < 0.0) ||
            (p->kd < 0.0) || (p->derivative_tau < 0.0)) { return false; }
    }
    return c->hardware_confirmed && (c->max_output <= 1.0) &&
        (c->manual_limit <= c->max_output) && (c->manual_timeout <= c->movement_timeout) &&
        (c->period <= c->max_dt) && (c->settle_time < c->stop_timeout) &&
        (c->stopped_speed < c->max_speed) && (c->pwm_hz >= 1000U) && (c->pwm_hz <= 20000U) &&
        isfinite(c->straight_gain) && (c->straight_gain >= 0.0);
}
