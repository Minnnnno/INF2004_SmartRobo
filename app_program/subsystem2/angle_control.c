#include "angle_test.h"
#include <math.h>
#include <string.h>
static const double pi = 3.141592653589793;
static void finish(angle_control_t *c, angle_state_t state)
{ c->state = state; c->output[0] = 0.0; c->output[1] = 0.0; }
void angle_control_init(angle_control_t *c, const motion_config_t *config)
{
    unsigned w;
    memset(c, 0, sizeof(*c));
    c->config = *config;
    c->ready = isfinite(config->track) && config->track > 0.0;
    for (w = 0U; w < 2U; ++w) {
        c->ready = c->ready && isfinite(config->diameter[w]) && config->diameter[w] > 0.0 &&
                   isfinite(config->counts_per_rev[w]) && config->counts_per_rev[w] > 0.0 &&
                   config->quadrature[w];
        if (c->ready) { c->goal[w] = config->counts_per_rev[w] * config->track / (4.0 * config->diameter[w]); }
    }
}
bool angle_control_start(angle_control_t *c, int direction, encoder_sample_t sample, double now)
{
    if (!c->ready || !sample.reliable || !isfinite(now) ||
        (direction != 1 && direction != -1) || c->state == ANGLE_TURNING || c->state == ANGLE_BRAKING ||
        (c->state != ANGLE_IDLE && now < c->quiet_since + 0.3)) { return false; }
    c->baseline = sample; c->previous = sample;
    c->delta[0] = 0U; c->delta[1] = 0U; c->angle = 0.0;
    c->direction = direction; c->deadline = now + 8.0;
    c->last_change[0] = now; c->last_change[1] = now; c->quiet_since = now;
    c->state = ANGLE_TURNING;
    c->output[0] = -0.25 * direction; c->output[1] = 0.25 * direction;
    return true;
}
void angle_control_step(angle_control_t *c, encoder_sample_t sample, double now)
{
    unsigned w;
    double travel[2], progress[2];
    bool changed = false;
    if (c->state != ANGLE_TURNING && c->state != ANGLE_BRAKING) { return; }
    if (!sample.reliable || !isfinite(now)) { finish(c, ANGLE_FEEDBACK); return; }
    for (w = 0U; w < 2U; ++w) {
        int expected = w == 0U ? -c->direction : c->direction;
        int64_t delta = encoder_delta(sample.count[w], c->baseline.count[w]);
        c->delta[w] = sample.count[w] - c->baseline.count[w];
        travel[w] = (double)delta * pi * c->config.diameter[w] / c->config.counts_per_rev[w];
        progress[w] = (double)delta * expected;
        if (sample.count[w] != c->previous.count[w]) { c->last_change[w] = now; changed = true; }
        if (sample.invalid[w] != c->baseline.invalid[w] || progress[w] < -4.0) {
            finish(c, ANGLE_FEEDBACK); c->quiet_since = now; return;
        }
    }
    c->angle = (travel[1] - travel[0]) / c->config.track;
    c->previous = sample;
    if (now >= c->deadline) { finish(c, ANGLE_TIMEOUT); c->quiet_since = now; return; }
    if (c->state == ANGLE_BRAKING) {
        if (changed) { c->quiet_since = now; }
        if (now - c->quiet_since >= 0.3) {
            finish(c, fabs(c->angle - c->direction * pi / 2.0) <= pi / 60.0 ? ANGLE_DONE : ANGLE_OVERSHOOT);
        } else if (now >= c->brake_deadline) { finish(c, ANGLE_TIMEOUT); }
        return;
    }
    for (w = 0U; w < 2U; ++w) {
        double remaining = fmax(0.0, 1.0 - progress[w] / c->goal[w]);
        double duty = fmin(0.25, 0.12 + 0.13 * remaining / (25.0 / 90.0));
        if (progress[w] >= c->goal[w]) { c->output[w] = 0.0; }
        else if (c->output[w] != 0.0) { c->output[w] = duty * (w == 0U ? -c->direction : c->direction); }
        /* A wheel already braked stays braked; never restart after overshoot. */
        if (c->output[w] != 0.0 && now - c->last_change[w] >= 1.0) {
            finish(c, ANGLE_FEEDBACK); c->quiet_since = now; return;
        }
    }
    if (c->output[0] == 0.0 && c->output[1] == 0.0) {
        c->state = ANGLE_BRAKING; c->quiet_since = now; c->brake_deadline = now + 1.0;
    }
}
void angle_control_stop(angle_control_t *c)
{ finish(c, ANGLE_STOPPED); }
