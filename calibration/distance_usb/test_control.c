#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "control.h"
#include "speed_window.h"
#include "turn_command.h"

static void simulate_turn(double degrees) {
    distance_test_t t;
    uint32_t count[2]={UINT32_MAX-10U,10U}, invalid[2]={2U,3U};
    uint32_t baseline[2]={count[0],count[1]};
    double position[2]={0.0,0.0};
    assert(test_start_turn(&t,count,invalid,0U,degrees));
    double direction=degrees>0.0 ? 1.0 : -1.0;
    assert(t.duty[0]*direction<0.0 && t.duty[1]*direction>0.0);
    for (uint64_t now=1000U;now<=5000000U && t.state==TEST_RUNNING;now+=1000U) {
        for (unsigned w=0U;w<2U;++w) {
            double velocity=fmax(0.0,(fabs(t.duty[w])-0.08)*1200.0)*(w==0U?1.0:0.92);
            position[w]+=velocity*0.001*(t.duty[w]>0.0?1.0:-1.0);
            int64_t delta=(int64_t)(position[w]/test_mm_per_count(w));
            count[w]=baseline[w]+(uint32_t)delta;
        }
        test_step(&t,count,invalid,now);
        assert(fabs(t.duty[0])<=0.249 && fabs(t.duty[1])<=0.25);
        if (t.state==TEST_RUNNING) {
            assert(t.duty[0]*direction<0.0 && t.duty[1]*direction>0.0);
        }
    }
    assert(t.state==TEST_GOAL && t.duty[0]==0.0 && t.duty[1]==0.0);
    double angle=(t.distance[1]-t.distance[0])*180.0/(3.141592653589793*115.0);
    assert(fabs(angle-degrees)<0.3); /* PWM-off estimate, not settled physical angle. */
    test_step(&t,count,invalid,4999999U); assert(t.state==TEST_GOAL);
}

static void check_turns(void) {
    const double angles[]={15.0,30.0,45.0,60.0,90.0,120.0,135.0,180.0};
    for (unsigned i=0U;i<sizeof(angles)/sizeof(angles[0]);++i) {
        simulate_turn(angles[i]); simulate_turn(-angles[i]);
    }
    double degrees=0.0;
    assert(parse_turn_command("turn_left 45",&degrees) && degrees==45.0);
    assert(parse_turn_command("turn_right 90",&degrees) && degrees==-90.0);
    assert(parse_turn_command("turn_left_90",&degrees) && degrees==90.0);
    assert(parse_turn_command("turn_right_180",&degrees) && degrees==-180.0);
    assert(parse_turn_command("turn_right 22.5",&degrees) && degrees==-22.5);
    const char *bad[]={"turn_left","turn_right ","turn_left 0","turn_left -45",
        "turn_left 181","turn_left nan","turn_right inf","turn_left 45 junk","turn_left 14.9"};
    for (unsigned i=0U;i<sizeof(bad)/sizeof(bad[0]);++i) { assert(!parse_turn_command(bad[i],&degrees)); }
    distance_test_t t={0}; uint32_t count[2]={0U,0U}, invalid[2]={0U,0U};
    assert(!test_start_turn(&t,count,invalid,0U,NAN) && t.state==TEST_IDLE);
    assert(test_start_turn(&t,count,invalid,0U,90.0));
    count[0]=5U; test_step(&t,count,invalid,1000U); assert(t.state==TEST_FEEDBACK);
    count[0]=0U; assert(test_start_turn(&t,count,invalid,0U,-90.0));
    count[1]=5U; test_step(&t,count,invalid,1000U); assert(t.state==TEST_FEEDBACK);
    assert(test_start_turn(&t,count,invalid,0U,90.0));
    ++invalid[1]; test_step(&t,count,invalid,1000U); assert(t.state==TEST_FEEDBACK);
    assert(test_start_turn(&t,count,invalid,0U,90.0));
    test_step(&t,count,invalid,700000U); assert(t.state==TEST_STALL);
    assert(test_start_turn(&t,count,invalid,0U,-180.0));
    test_step(&t,count,invalid,5000000U); assert(t.state==TEST_TIMEOUT);
    assert(test_start_turn(&t,count,invalid,0U,45.0)); test_stop(&t,TEST_STOP);
    test_step(&t,count,invalid,1000U);
    assert(t.state==TEST_STOP && t.duty[0]==0.0 && t.duty[1]==0.0);
}

static void simulate(double goal, bool weak_right) {
    distance_test_t t;
    uint32_t count[2]={0U,0U}, invalid[2]={0U,0U};
    double position[2]={0.0,0.0};
    test_start(&t,count,invalid,0U,goal);
    for (uint64_t now=1000U;now<=5000000U && t.state==TEST_RUNNING;now+=1000U) {
        for (unsigned w=0U;w<2U;++w) {
            /* Synthetic unequal motor response, with extra right friction
             * below 23% PWM. This is regression evidence, not a motor model. */
            double factor=weak_right && w==1U ? 0.9 : 1.0;
            double friction=weak_right && w==1U && t.duty[w]<0.23 ? 0.015 : 0.0;
            double velocity=fmax(0.0,(t.duty[w]-0.08-friction)*1200.0*factor);
            position[w]+=velocity*0.001;
            count[w]=(uint32_t)(position[w]/test_mm_per_count(w));
        }
        test_step(&t,count,invalid,now);
        assert(t.duty[0]>=0.0 && t.duty[0]<=0.249);
        assert(t.duty[1]>=0.0 && t.duty[1]<=0.25);
        if (t.state==TEST_RUNNING) { assert(t.duty[0]>0.0 && t.duty[1]>0.0); }
    }
    assert(t.state==TEST_GOAL && t.duty[0]==0.0 && t.duty[1]==0.0);
    /* Simulation ends at PWM-off; it does not model tyre coasting. */
    assert(fabs((position[0]+position[1])*0.5-(goal-4.5))<0.5);
    assert(t.max_skew<5.0);
}
int main(void) {
    check_turns();
    distance_test_t t;
    uint32_t count[2]={0U,0U}, invalid[2]={0U,18U};
    test_start(&t,count,invalid,0U,500.0);
    count[0]=6200U; count[1]=6200U;
    test_step(&t,count,invalid,100000U);
    assert(t.state==TEST_GOAL && t.duty[0]==0.0 && t.duty[1]==0.0);
    count[0]=0U; count[1]=0U; test_start(&t,count,invalid,0U,500.0);
    test_step(&t,count,invalid,5000000U); assert(t.state==TEST_TIMEOUT);
    test_start(&t,count,invalid,0U,500.0);
    test_step(&t,count,invalid,700000U); assert(t.state==TEST_STALL);
    test_start(&t,count,invalid,0U,500.0); ++invalid[1];
    test_step(&t,count,invalid,1000U); assert(t.state==TEST_FEEDBACK);
    test_start(&t,count,invalid,0U,500.0); count[0]=0U-5U;
    test_step(&t,count,invalid,1000U); assert(t.state==TEST_FEEDBACK);
    count[0]=UINT32_MAX-10U; count[1]=UINT32_MAX-10U;
    test_start(&t,count,invalid,0U,500.0); count[0]=9U; count[1]=9U;
    test_step(&t,count,invalid,1000U);
    assert(t.state==TEST_RUNNING && t.distance[0]>0.0 && t.distance[0]<2.0);
    count[0]=0U; count[1]=0U; test_start(&t,count,invalid,0U,500.0);
    count[0]=5600U; count[1]=5600U; test_step(&t,count,invalid,1000U);
    assert(t.duty[0]<0.249 && t.duty[1]<0.25 && t.duty[0]>0.0);
    /* A leading left wheel must be slowed, not the lagging right wheel. */
    count[0]=3000U; count[1]=2800U; test_start(&t,(uint32_t[2]){0U,0U},invalid,0U,500.0);
    test_step(&t,count,invalid,1000U); assert(t.duty[0]<0.249 && t.duty[1]==0.25);
    count[0]=2800U; count[1]=3000U;
    test_step(&t,count,invalid,2000U); assert(t.duty[0]==0.249 && t.duty[1]<0.25);
    /* One wheel reaching its own target must not trigger a one-wheel pivot. */
    count[0]=6000U; count[1]=5700U;
    test_step(&t,count,invalid,3000U);
    assert(t.state==TEST_RUNNING && t.duty[0]>0.0 && t.duty[1]>0.0);
    count[0]=6200U; count[1]=6200U; test_step(&t,count,invalid,4000U);
    assert(t.state==TEST_GOAL && t.duty[0]==0.0 && t.duty[1]==0.0);
    test_stop(&t,TEST_STOP); test_step(&t,count,invalid,4000U);
    assert(t.state==TEST_STOP && t.duty[0]==0.0 && t.duty[1]==0.0);
    /* Recorded v3 settled deltas from mat trial 4: approximately 305 mm
     * previously, approximately 296 mm after the common scale correction. */
    double recorded_centre=(3675.0*test_mm_per_count(0U)+3618.0*test_mm_per_count(1U))*0.5;
    assert(fabs(recorded_centre-296.0)<0.5);
    assert(fabs(test_brake_point(300.0)-295.5)<0.000001);
    assert(test_brake_point(2.0)==0.0);
    /* Cross the braking threshold without yet reaching the requested goal. */
    test_start(&t,(uint32_t[2]){0U,0U},invalid,0U,300.0);
    for (unsigned w=0U;w<2U;++w) { count[w]=(uint32_t)(295.0/test_mm_per_count(w)); }
    test_step(&t,count,invalid,1000U); assert(t.state==TEST_RUNNING);
    for (unsigned w=0U;w<2U;++w) { count[w]=(uint32_t)ceil(295.5/test_mm_per_count(w)); }
    test_step(&t,count,invalid,2000U);
    assert(t.state==TEST_GOAL && t.duty[0]==0.0 && t.duty[1]==0.0);
    simulate(100.0,false); simulate(300.0,false); simulate(500.0,false);
    simulate(100.0,true); simulate(300.0,true); simulate(500.0,true);
    speed_window_t window={0};
    speed_window_step(&window,(double[2]){68.0,70.0},100000U);
    assert(!window.started && !window.complete);
    speed_window_step(&window,(double[2]){71.0,69.0},200000U);
    assert(window.started && !window.complete && window.start_us==200000U);
    speed_window_step(&window,(double[2]){110.0,110.0},400000U);
    assert(!window.complete);
    speed_window_step(&window,(double[2]){171.0,169.0},800000U);
    assert(window.complete && window.end_us==800000U);
    double centre_speed=((window.end_mm[0]-window.start_mm[0])+
                         (window.end_mm[1]-window.start_mm[1]))*0.5/
                         ((double)(window.end_us-window.start_us)/1000000.0);
    assert(fabs(centre_speed-166.6666666667)<0.000001);
    speed_window_step(&window,(double[2]){301.0,299.0},2000000U);
    assert(window.end_us==800000U && window.end_mm[0]==171.0);
    window=(speed_window_t){0};
    speed_window_step(&window,(double[2]){100.0,100.0},100000U);
    assert(window.started && !window.complete); /* move_100 cannot report full gate. */
    puts("distance control: safety, correction direction, shared stopping and 100/300/500 mm unequal-motor simulations passed");
    puts("speed window: thresholds, unequal wheel travel, elapsed speed, frozen results and incomplete runs passed");
    puts("turns: 15/30/45/60/90/120/135/180 both directions, wrap, parsing, wrong direction, invalid, stall, timeout and stop passed");
}
