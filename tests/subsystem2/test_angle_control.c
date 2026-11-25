#include "angle_test.h"
#include "board_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static unsigned checks;
static void travel(angle_control_t *c, encoder_sample_t *sample, int direction, double fraction)
{
    unsigned w;
    for (w = 0U; w < 2U; ++w) {
        int64_t ticks = (int64_t)ceil(c->goal[w] * fraction);
        int expected = w == 0U ? -direction : direction;
        sample->count[w] = c->baseline.count[w] + (uint32_t)(ticks * expected);
    }
}
int main(void)
{
    angle_control_t c;
    motion_config_t config = board_motion_config();
    encoder_sample_t sample = {.count={UINT32_MAX-10U,20U}, .reliable=true};
    unsigned direction, i;
    CHECK(!config.hardware_confirmed && !config.manual_enabled);
    CHECK(config.motor_sign[0] == 1 && config.motor_sign[1] == -1);
    for (direction = 0U; direction < 2U; ++direction) {
        int sign = direction == 0U ? 1 : -1;
        angle_control_init(&c,&config);
        CHECK(c.ready && c.state == ANGLE_IDLE && c.output[0] == 0.0 && c.output[1] == 0.0);
        CHECK(fabs(c.goal[0]-1082.874074074) < 0.001 && fabs(c.goal[1]-1076.918262150) < 0.001);
        CHECK(angle_control_start(&c,sign,sample,0.0));
        CHECK(!angle_control_start(&c,-sign,sample,0.01));
        CHECK(c.deadline == 8.0);
        for (i=1U;i<100U;++i) {
            travel(&c,&sample,sign,(double)i/100.0);
            angle_control_step(&c,sample,(double)i*0.005);
            CHECK(c.state == ANGLE_TURNING);
            CHECK(c.output[0]*sign < 0.0 && c.output[1]*sign > 0.0);
            CHECK(fabs(c.output[0]) <= 0.25 && fabs(c.output[1]) <= 0.25);
            if (i>=80U) { CHECK(fabs(c.output[0]) < 0.25 && fabs(c.output[1]) < 0.25); }
        }
        travel(&c,&sample,sign,1.0); angle_control_step(&c,sample,0.5);
        CHECK(c.state == ANGLE_BRAKING && c.output[0] == 0.0 && c.output[1] == 0.0);
        CHECK(!angle_control_start(&c,sign,sample,0.6));
        angle_control_step(&c,sample,0.81); CHECK(c.state == ANGLE_DONE);
        CHECK(fabs(c.angle - sign*3.141592653589793/2.0) < 0.002);
    }
    sample=(encoder_sample_t){.reliable=true};
    angle_control_init(&c,&config); CHECK(angle_control_start(&c,1,sample,0.0));
    sample.count[0]=10U; angle_control_step(&c,sample,0.01);
    CHECK(c.state == ANGLE_FEEDBACK && c.output[0] == 0.0 && c.output[1] == 0.0);
    CHECK(!angle_control_start(&c,1,sample,0.1));
    angle_control_init(&c,&config); CHECK(angle_control_start(&c,1,sample,0.0));
    sample.invalid[1]++; angle_control_step(&c,sample,0.01); CHECK(c.state == ANGLE_FEEDBACK);
    angle_control_init(&c,&config); CHECK(angle_control_start(&c,1,sample,0.0));
    angle_control_step(&c,sample,1.01); CHECK(c.state == ANGLE_FEEDBACK);
    angle_control_init(&c,&config); CHECK(angle_control_start(&c,1,sample,0.0));
    for (i=1U;i<=16U;++i) {
        sample.count[0]--; sample.count[1]++;
        angle_control_step(&c,sample,(double)i*0.5);
    }
    CHECK(c.state == ANGLE_TIMEOUT && c.output[0] == 0.0 && c.output[1] == 0.0);
    angle_control_init(&c,&config); CHECK(angle_control_start(&c,1,sample,0.0));
    travel(&c,&sample,1,1.1); angle_control_step(&c,sample,0.1);
    angle_control_step(&c,sample,0.41); CHECK(c.state == ANGLE_OVERSHOOT);
    angle_control_init(&c,&config); CHECK(angle_control_start(&c,1,sample,0.0));
    travel(&c,&sample,1,1.0); angle_control_step(&c,sample,0.1);
    for (i=1U;i<=6U;++i) { sample.count[0]--; sample.count[1]++; angle_control_step(&c,sample,0.1+(double)i*0.2); }
    CHECK(c.state == ANGLE_TIMEOUT);
    angle_control_init(&c,&config); CHECK(angle_control_start(&c,1,sample,0.0));
    sample.count[0] = c.baseline.count[0] - (uint32_t)ceil(c.goal[0]);
    sample.count[1] = c.baseline.count[1] + 800U;
    angle_control_step(&c,sample,0.01); CHECK(c.output[0] == 0.0 && c.output[1] > 0.0);
    sample.count[0] += 5U; sample.count[1] += 10U;
    angle_control_step(&c,sample,0.02); CHECK(c.output[0] == 0.0);
    angle_control_stop(&c); CHECK(c.state == ANGLE_STOPPED && c.output[0] == 0.0 && c.output[1] == 0.0);
    CHECK(!angle_control_start(&c,0,sample,1.0));
    config.track=0.0; angle_control_init(&c,&config); CHECK(!c.ready && !angle_control_start(&c,1,sample,1.0));
    printf("PASS: %u angle-controller checks\n",checks);
    return 0;
}
