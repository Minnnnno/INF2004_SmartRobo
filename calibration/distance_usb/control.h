#ifndef DISTANCE_CALIBRATION_CONTROL_H
#define DISTANCE_CALIBRATION_CONTROL_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "motion.h"

typedef enum { TEST_IDLE, TEST_RUNNING, TEST_GOAL, TEST_STOP,
               TEST_TIMEOUT, TEST_FEEDBACK, TEST_STALL, TEST_SERVICE } test_state_t;
typedef struct {
    test_state_t state;
    uint32_t baseline[2], invalid[2], previous[2];
    uint64_t started, last_change[2];
    double goal, distance[2], duty[2], max_skew, turn_degrees;
} distance_test_t;

/* Mat trials 2-4: actual mean 296 mm, settled encoder mean 304.833 mm.
 * Keep the wheel ratio; this is a provisional common distance/speed scale.
 * About 4.7 mm of corrected travel occurred after switching PWM off. */
#define TEST_DISTANCE_SCALE (296.0 / (914.5 / 3.0))
#define TEST_BRAKE_ALLOWANCE_MM 4.5
#define TEST_TRACK_MM 115.0

static inline double test_mm_per_count(unsigned wheel) {
    return TEST_DISTANCE_SCALE * (wheel == 0U ? 3.141592653589793 * 67.5 / 2542.4 :
                                              3.141592653589793 * 67.9 / 2543.4);
}
static inline double test_brake_point(double goal) {
    return goal>TEST_BRAKE_ALLOWANCE_MM ? goal-TEST_BRAKE_ALLOWANCE_MM : 0.0;
}
static inline void test_stop(distance_test_t *t, test_state_t reason) {
    t->state = reason; t->duty[0] = 0.0; t->duty[1] = 0.0;
}
static inline void test_start(distance_test_t *t, const uint32_t count[2],
                              const uint32_t invalid[2], uint64_t now, double goal) {
    *t = (distance_test_t){.state=TEST_RUNNING, .started=now, .goal=goal};
    for (unsigned w=0U; w<2U; ++w) {
        t->baseline[w]=count[w]; t->previous[w]=count[w];
        t->invalid[w]=invalid[w]; t->last_change[w]=now;
        t->duty[w]=w==0U ? 0.249 : 0.25;
    }
}
/* Positive angle turns left in place; negative turns right. Straight-line
 * coasting allowance is deliberately not assumed valid for tyre scrubbing. */
static inline bool test_start_turn(distance_test_t *t, const uint32_t count[2],
                                  const uint32_t invalid[2], uint64_t now, double degrees) {
    if (!isfinite(degrees) || fabs(degrees)<15.0 || fabs(degrees)>180.0) { return false; }
    double wheel_goal=3.141592653589793*TEST_TRACK_MM*fabs(degrees)/360.0;
    test_start(t,count,invalid,now,wheel_goal);
    t->turn_degrees=degrees;
    double direction=degrees>0.0 ? 1.0 : -1.0;
    t->duty[0]*=-direction; t->duty[1]*=direction;
    return true;
}
static inline void test_step(distance_test_t *t, const uint32_t count[2],
                             const uint32_t invalid[2], uint64_t now) {
    if (t->state != TEST_RUNNING) { return; }
    if (now-t->started >= 5000000U) { test_stop(t, TEST_TIMEOUT); return; }
    for (unsigned w=0U; w<2U; ++w) {
        int64_t delta=encoder_delta(count[w],t->baseline[w]);
        t->distance[w]=(double)delta*test_mm_per_count(w);
        int expected=t->turn_degrees==0.0 ? 1 :
                     ((w==0U ? -1 : 1)*(t->turn_degrees>0.0 ? 1 : -1));
        if (invalid[w]!=t->invalid[w] || delta*expected < -4) { test_stop(t, TEST_FEEDBACK); return; }
        if (count[w]!=t->previous[w]) { t->last_change[w]=now; }
        t->previous[w]=count[w];
        if (t->duty[w]!=0.0 && now-t->last_change[w]>=700000U) { test_stop(t,TEST_STALL); return; }
    }
    double progress[2]={t->distance[0],t->distance[1]};
    if (t->turn_degrees!=0.0) {
        double direction=t->turn_degrees>0.0 ? 1.0 : -1.0;
        progress[0]*=-direction; progress[1]*=direction;
    }
    double centre=(progress[0]+progress[1])*0.5;
    double skew=progress[0]-progress[1];
    double magnitude=skew<0.0 ? -skew : skew;
    if (magnitude>t->max_skew) { t->max_skew=magnitude; }
    /* Stop the axle together: never pivot on an independently stopped wheel. */
    double stop_at=t->turn_degrees==0.0 ? test_brake_point(t->goal) : t->goal;
    if (centre>=stop_at) { test_stop(t,TEST_GOAL); return; }
    double remaining=t->goal-centre;
    /* One slowdown profile preserves the existing left/right trim. Avoid the
     * previous 18% crawl; low-duty motor responses need physical validation. */
    double slow_range=t->turn_degrees==0.0 ? 80.0 : 30.0;
    double pace=remaining>=slow_range ? 0.25 : 0.22+0.03*remaining/slow_range;
    t->duty[0]=pace*(0.249/0.25); t->duty[1]=pace;
    /* Reduce only the leading wheel. Never exceed the tested PWM limits or
     * reverse a wheel to correct heading. 1 mm lead reduces duty by 0.75 pp.
     * v5: increased from 0.5 pp/mm after repeated 2-4 mm left-wheel lead. */
    double correction=magnitude*0.0075;
    if (correction>0.04) { correction=0.04; }
    if (skew>0.0) { t->duty[0]-=correction; }
    else if (skew<0.0) { t->duty[1]-=correction; }
    if (t->turn_degrees!=0.0) {
        double direction=t->turn_degrees>0.0 ? 1.0 : -1.0;
        t->duty[0]*=-direction; t->duty[1]*=direction;
    }
}
#endif
