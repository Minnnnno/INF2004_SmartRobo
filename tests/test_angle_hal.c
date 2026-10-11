#include "angle_test.h"
#include "motion_hal.h"
#include "motor_test_fake_hal.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static unsigned checks, enable_calls;
static uint64_t now;
static uint32_t deadline;
static bool enabled[2], armed;
static uint16_t duty[2][2];
static FP deadline_handler;
static encoder_sample_t sample={.reliable=true};
uint32_t save_and_disable_interrupts(void) { return 0U; }
void restore_interrupts(uint32_t state) { (void)state; }
uint64_t time_us_64(void) { return now; }
double motion_hal_time(void) { return (double)now/1000000.0; }
encoder_sample_t motion_hal_sample(void) { return sample; }
void motion_hal_emergency_stop(void) { enabled[0]=false; enabled[1]=false; }
pwm_config pwm_get_default_config(void) { pwm_config c={4095U,1.0f}; return c; }
void pwm_config_set_wrap(pwm_config *c,uint16_t wrap) { c->wrap=wrap; }
void pwm_config_set_clkdiv(pwm_config *c,float divider) { c->divider=divider; }
void pwm_init(unsigned slice,const pwm_config *c,bool enable)
{ CHECK(slice==4U || slice==5U); CHECK(c->wrap==3999U && c->divider==1.5625f && !enable); }
void pwm_set_both_levels(unsigned slice,uint16_t a,uint16_t b)
{ CHECK(slice==4U || slice==5U); CHECK(a<=1000U && b<=1000U && (a==0U || b==0U)); duty[slice-4U][0]=a; duty[slice-4U][1]=b; }
void pwm_set_enabled(unsigned slice,bool enable)
{ CHECK(slice==4U || slice==5U); if(enable) { CHECK(armed); ++enable_calls; } enabled[slice-4U]=enable; }
void gpio_set_function(unsigned pin,unsigned function)
{ CHECK(pin>=8U && pin<=11U && function==GPIO_FUNC_PWM); }
void out_w(unsigned address,uint32_t value)
{ if(address==TIMER_ALARM3) { deadline=value; armed=true; } else if(address==TIMER_ARMED) { CHECK(value==8U); armed=false; } else { CHECK(address==TIMER_INTR && value==8U); } }
void set_w(unsigned address,uint32_t value) { CHECK(address==TIMER_INTE && value==8U); }
void ClearInt(unsigned number) { CHECK(number==3U); }
void EnableInt(unsigned number,int priority) { CHECK(number==3U && priority==0); }
int tk_def_int(unsigned number,const T_DINT *definition)
{ CHECK(number==3U); deadline_handler=definition->inthdr; return E_OK; }
int main(void)
{
    unsigned before;
    CHECK(angle_test_init()); CHECK(!enabled[0] && !enabled[1] && !angle_test_active());
    CHECK(angle_test_start(1)); CHECK(enabled[0] && enabled[1]);
    CHECK(duty[0][0]==0U && duty[0][1]==1000U && duty[1][0]==0U && duty[1][1]==1000U);
    CHECK(deadline==8000000U); CHECK(!angle_test_start(-1) && deadline==8000000U);
    sample.count[0]=0U-1000U; sample.count[1]=1000U; now=10000U; angle_test_poll();
    CHECK(duty[0][1]<1000U && duty[1][1]<1000U);
    sample.count[0]=0U-1125U; sample.count[1]=1125U; now=20000U; angle_test_poll();
    CHECK(!enabled[0] && !enabled[1] && angle_test_status().state==ANGLE_BRAKING);
    now=330000U; angle_test_poll(); CHECK(angle_test_status().state==ANGLE_DONE && !armed);
    now=1000000U; CHECK(angle_test_start(-1));
    CHECK(duty[0][0]==1000U && duty[0][1]==0U && duty[1][0]==1000U && duty[1][1]==0U);
    now=1100000U; angle_test_stop(); CHECK(!enabled[0] && !enabled[1] && !armed);
    CHECK(angle_test_status().state==ANGLE_STOPPED && !angle_test_start(1));
    now=1500000U; CHECK(angle_test_start(1));
    now=9500000U; deadline_handler(3U); /* Cutoff fires without task polling. */
    CHECK(!enabled[0] && !enabled[1]); before=enable_calls;
    angle_test_poll(); CHECK(angle_test_status().state==ANGLE_TIMEOUT && enable_calls==before);
    angle_test_stop(); now+=400000U; CHECK(angle_test_start(-1));
    now+=1100000U; angle_test_poll();
    CHECK(angle_test_status().state==ANGLE_FEEDBACK && !enabled[0] && !enabled[1]);
    printf("PASS: %u angle-HAL checks (mock registers)\n",checks);
    return 0;
}
