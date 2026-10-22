/* Isolated SDK distance test: no production controller, IR or RTOS. */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "encoder_capture.h"
#include "control.h"
#include "speed_window.h"
#include "turn_command.h"

static volatile encoder_capture_t capture;
static distance_test_t test;
static speed_window_t speed_window;
static const unsigned pins[2][2]={{4U,6U},{5U,7U}};
static const bool quadrature[2]={true,true};
static const int signs[2]={-1,1};
static volatile uint64_t serviced;
static uint64_t released_at, last_stop;
static bool release_seen;
static volatile bool armed, report_pending;
static uint32_t encoder_mask;
static double speed[2];
static uint32_t speed_previous[2];
static uint64_t speed_at;

static void apply_outputs(void) {
    for (unsigned w=0U;w<2U;++w) {
        double electrical=test.duty[w]*(w==0U ? 1.0 : -1.0);
        uint16_t duty=(uint16_t)(fabs(electrical)*4000.0);
        pwm_set_both_levels(4U+w,electrical>0.0?duty:0U,electrical<0.0?duty:0U);
    }
}

static void outputs_off(void) {
    pwm_set_both_levels(4U,0U,0U); pwm_set_both_levels(5U,0U,0U);
    pwm_set_enabled(4U,false); pwm_set_enabled(5U,false);
    for (unsigned p=8U;p<=11U;++p) {
        gpio_put(p,false); gpio_set_function(p,GPIO_FUNC_SIO); gpio_set_dir(p,GPIO_OUT);
    }
}
static void stop_now(test_state_t reason) {
    outputs_off();
    if (test.state==TEST_RUNNING) {
        test_stop(&test,reason); report_pending=true; last_stop=time_us_64();
    }
    armed=false; release_seen=false;
}
static void gpio_irq(void) {
    uint32_t pending[4]={io_bank0_hw->proc0_irq_ctrl.ints[0]&encoder_mask,0U,0U,0U};
    io_bank0_hw->intr[0]=pending[0];
    encoder_capture_events(&capture,pending,gpio_get_all());
    uint32_t stop=gpio_get_irq_event_mask(21U);
    uint32_t toggle=gpio_get_irq_event_mask(20U);
    gpio_acknowledge_irq(21U,stop); gpio_acknowledge_irq(20U,toggle);
    /* Stop in IRQ context; starts occur only in the main loop after debounce. */
    if (test.state==TEST_RUNNING && ((stop&GPIO_IRQ_EDGE_FALL)!=0U ||
        ((toggle&GPIO_IRQ_EDGE_FALL)!=0U && armed))) { stop_now(TEST_STOP); }
}
static bool control_tick(repeating_timer_t *timer) {
    (void)timer;
    uint64_t now=time_us_64();
    uint32_t state=save_and_disable_interrupts();
    uint32_t counts[2]={capture.count[0],capture.count[1]};
    uint32_t invalid[2]={capture.invalid[0],capture.invalid[1]};
    if (now-speed_at>=100000U) {
        for (unsigned w=0U;w<2U;++w) {
            speed[w]=(double)encoder_delta(counts[w],speed_previous[w])*test_mm_per_count(w)*
                     1000000.0/(double)(now-speed_at);
            speed_previous[w]=counts[w];
        }
        speed_at=now;
    }
    if (test.state==TEST_RUNNING) {
        if (!gpio_get(21U) || (!gpio_get(20U)&&armed)) { stop_now(TEST_STOP); }
        else if (now-serviced>=300000U) { stop_now(TEST_SERVICE); }
        else {
            test_step(&test,counts,invalid,now);
            if (test.turn_degrees==0.0 && (test.state==TEST_RUNNING || test.state==TEST_GOAL)) {
                speed_window_step(&speed_window,test.distance,now-test.started);
            }
            if (test.state!=TEST_RUNNING) {
                outputs_off(); report_pending=true; last_stop=now; armed=false; release_seen=false;
            } else {
                apply_outputs();
            }
        }
    }
    restore_interrupts(state);
    return true;
}
static bool start_run(double goal, bool turning) {
    uint32_t state=save_and_disable_interrupts();
    uint64_t now=time_us_64();
    if (test.state==TEST_RUNNING || !gpio_get(21U) || now-last_stop<500000U || report_pending) {
        restore_interrupts(state); return false;
    }
    uint32_t count[2]={capture.count[0],capture.count[1]};
    uint32_t invalid[2]={capture.invalid[0],capture.invalid[1]};
    if (turning) {
        if (!test_start_turn(&test,count,invalid,now,goal)) { restore_interrupts(state); return false; }
    } else { test_start(&test,count,invalid,now,goal); }
    speed_window=(speed_window_t){0};
    serviced=now; armed=false; release_seen=false;
    for (unsigned p=8U;p<=11U;++p) { gpio_set_function(p,GPIO_FUNC_PWM); }
    apply_outputs();
    pwm_set_enabled(4U,true); pwm_set_enabled(5U,true);
    restore_interrupts(state); return true;
}
static void buttons(void) {
    uint32_t state=save_and_disable_interrupts();
    uint64_t now=time_us_64();
    if (!gpio_get(21U)) { stop_now(TEST_STOP); }
    else if (gpio_get(20U)) {
        if (!release_seen) { released_at=now; release_seen=true; }
        if (now-released_at>=50000U) { armed=true; }
    } else if (armed) {
        if (test.state==TEST_RUNNING) { stop_now(TEST_STOP); }
        else { armed=false; release_seen=false; (void)start_run(300.0,false); }
    } else { release_seen=false; }
    restore_interrupts(state);
}
static void report(void) {
    encoder_capture_t snapshot;
    distance_test_t t;
    speed_window_t window;
    double velocity[2];
    uint32_t state=save_and_disable_interrupts();
    snapshot=capture; t=test; window=speed_window; velocity[0]=speed[0]; velocity[1]=speed[1];
    restore_interrupts(state);
    static const char *const names[]={"IDLE","RUNNING","GOAL","STOP","TIMEOUT","FEEDBACK","STALL","SERVICE"};
    printf("firmware=%s state=%s production_controller=absent ir=disabled max_ms=5000\n",
           CALIBRATION_BUILD_ID,names[t.state]);
    printf("counts=%lld,%lld invalid=%lu,%lu GP20=%u GP21=%u\n",
           (long long)encoder_delta(snapshot.count[0],0U),(long long)encoder_delta(snapshot.count[1],0U),
           (unsigned long)snapshot.invalid[0],(unsigned long)snapshot.invalid[1],
           (unsigned)gpio_get(20U),(unsigned)gpio_get(21U));
    double distance[2];
    for (unsigned w=0U;w<2U;++w) {
        distance[w]=(double)encoder_delta(snapshot.count[w],t.baseline[w])*test_mm_per_count(w);
    }
    printf("goal_mm=%.1f distance_mm=%.1f,%.1f speed_mm_s=%.1f,%.1f centre_mm=%.1f\n",
           t.goal,distance[0],distance[1],velocity[0],velocity[1],(distance[0]+distance[1])*0.5);
    printf("travel_difference_mm=%.1f max_difference_mm=%.1f output_percent=%.2f,%.2f\n",
           distance[0]-distance[1],t.max_skew,t.duty[0]*100.0,t.duty[1]*100.0);
    printf("distance_scale=%.6f brake_allowance_mm=%.1f brake_at_mm=%.1f\n",
           TEST_DISTANCE_SCALE,t.turn_degrees==0.0 ? TEST_BRAKE_ALLOWANCE_MM : 0.0,
           t.turn_degrees==0.0 ? test_brake_point(t.goal) : t.goal);
    if (t.turn_degrees!=0.0) {
        printf("mode=turn requested_deg=%.1f encoder_angle_deg=%.2f track_mm=%.1f wheel_goal_mm=%.2f\n",
               t.turn_degrees,(distance[1]-distance[0])*180.0/(3.141592653589793*TEST_TRACK_MM),
               TEST_TRACK_MM,t.goal);
    } else if (window.complete) {
        double seconds=(double)(window.end_us-window.start_us)/1000000.0;
        double left=(window.end_mm[0]-window.start_mm[0])/seconds;
        double right=(window.end_mm[1]-window.start_mm[1])/seconds;
        printf("speed_window complete=1 from_mm=%.1f to_mm=%.1f elapsed_s=%.6f\n",
               (window.start_mm[0]+window.start_mm[1])*0.5,
               (window.end_mm[0]+window.end_mm[1])*0.5,seconds);
        printf("window_speed_mm_s=%.1f,%.1f centre_speed_mm_s=%.1f\n",left,right,(left+right)*0.5);
    } else {
        puts("speed_window complete=0 needs_run_through_170mm=1");
    }
}
int main(void) {
    outputs_off();
    for (unsigned w=0U;w<2U;++w) for (unsigned c=0U;c<2U;++c) {
        unsigned p=pins[w][c]; gpio_init(p); gpio_set_dir(p,GPIO_IN); gpio_disable_pulls(p);
        encoder_mask|=12U<<(4U*p);
    }
    encoder_capture_init(&capture,pins,quadrature,signs,gpio_get_all());
    io_bank0_hw->intr[0]=encoder_mask;
    irq_set_exclusive_handler(IO_IRQ_BANK0,gpio_irq); irq_set_priority(IO_IRQ_BANK0,0x40U);
    for (unsigned w=0U;w<2U;++w) for (unsigned c=0U;c<2U;++c) {
        gpio_set_irq_enabled(pins[w][c],GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,true);
    }
    for (unsigned p=20U;p<=21U;++p) {
        gpio_init(p); gpio_set_dir(p,GPIO_IN); gpio_pull_up(p);
        gpio_acknowledge_irq(p,GPIO_IRQ_EDGE_FALL);
        gpio_set_irq_enabled(p,GPIO_IRQ_EDGE_FALL,true);
    }
    irq_set_enabled(IO_IRQ_BANK0,true);
    pwm_config pwm=pwm_get_default_config(); pwm_config_set_wrap(&pwm,3999U);
    pwm_config_set_clkdiv(&pwm,(float)clock_get_hz(clk_sys)/80000000.0f);
    pwm_init(4U,&pwm,false); pwm_init(5U,&pwm,false);
    stdio_init_all();
    serviced=time_us_64(); speed_at=serviced;
    repeating_timer_t timer;
    if (!add_repeating_timer_us(-1000,control_tick,NULL,&timer)) {
        outputs_off(); for (;;) { tight_loop_contents(); }
    }
    watchdog_enable(500U,true);
    char line[32]; unsigned used=0U; bool discard=false;
    for (;;) {
        uint32_t state=save_and_disable_interrupts(); serviced=time_us_64();
        bool done=report_pending; report_pending=false; restore_interrupts(state);
        watchdog_update(); buttons();
        if (done) { report(); }
        int input=getchar_timeout_us(0U);
        if (input=='\r'||input=='\n') {
            if (discard) { discard=false; used=0U; continue; }
            line[used]='\0'; used=0U;
            if (strcmp(line,"stop")==0) {
                state=save_and_disable_interrupts(); stop_now(TEST_STOP); restore_interrupts(state); report();
            } else if (strcmp(line,"move_500")==0 || strcmp(line,"move_300")==0 || strcmp(line,"move_100")==0) {
                double goal=strcmp(line,"move_100")==0 ? 100.0 :
                            (strcmp(line,"move_300")==0 ? 300.0 : 500.0);
                puts(start_run(goal,false)?"started":"start_rejected");
            } else if (strncmp(line,"turn_",5U)==0) {
                double degrees;
                if (!parse_turn_command(line,&degrees)) { puts("Use turn_left 15..180 | turn_right 15..180"); }
                else { puts(start_run(degrees,true)?"started":"start_rejected"); }
            } else if (strcmp(line,"status")==0 || strcmp(line,"encdiag")==0) { report(); }
            else if (line[0]!='\0') { puts("status | encdiag | move_100 | move_300 | move_500 | turn_left DEGREES | turn_right DEGREES | stop"); }
        } else if (input>=32 && input<=126) {
            if (!discard && used+1U<sizeof(line)) { line[used++]=(char)input; }
            else { discard=true; used=0U; }
        }
        sleep_ms(1U);
    }
}
