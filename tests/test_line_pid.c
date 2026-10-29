#include "line_test_fake_hal.h"
#include "steering.h"
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); exit(1); } } while (0)
typedef enum { NONE, STRAIGHT, LEFT, RIGHT, PIVOT_LEFT, PIVOT_RIGHT, HOLD, BACK } action_t;
/* centre=GP27, right=GP26 (offset right). extra = seconds added after the row.
 * heading = encoder heading in degrees (+ = left); travel = centre travel in mm.
 * scenario: 0=normal ownership, 1=obstacle owns, 2=fault, 3=controller release. */
typedef struct { bool centre,right; action_t request; double extra,heading,travel; unsigned scenario; } input_t;
static const input_t inputs[]={
    {true,true,NONE,0.0,0.0,0.0,0U}, {true,true,STRAIGHT,0.0,0.0,0.0,0U},
    {true,false,STRAIGHT,0.0,0.0,0.0,0U},        /* Debounce holds; raw last side is already left. */
    {true,false,LEFT,0.0,0.0,0.0,0U},            /* Centre only: line is to the left. */
    {false,false,LEFT,0.0,0.0,0.0,0U},           /* Last row on the line: travel 0 mm. */
    {false,false,LEFT,0.2,0.0,10.0,0U},          /* Lost: grace repeats the last command. */
    {false,false,PIVOT_LEFT,0.0,0.0,20.0,0U},    /* Sweep 1 towards the side last seen; overshoot 20 mm. */
    {false,false,PIVOT_RIGHT,0.0,61.0,40.0,0U},  /* Sweep limit reached: sweep 2 the other way. */
    {false,false,PIVOT_RIGHT,0.0,0.0,60.0,0U},
    {false,false,PIVOT_LEFT,0.0,-61.0,80.0,0U},  /* Both sides missed: turn back to the line heading. */
    {false,false,BACK,0.0,0.0,100.0,0U},         /* Heading restored: reverse 20 + 20 mm. */
    {false,false,BACK,0.0,0.0,80.0,0U},
    {false,false,PIVOT_LEFT,0.0,0.0,55.0,0U},    /* Reversed far enough: sweep again. */
    {false,false,PIVOT_RIGHT,0.0,61.0,70.0,0U},
    {false,false,HOLD,0.0,-61.0,80.0,0U},        /* Second sweeps missed too: zero-speed hold. */
    {false,false,HOLD,0.0,-61.0,80.0,0U},
    {true,false,HOLD,0.0,-61.0,80.0,0U},
    {true,false,LEFT,0.0,-61.0,80.0,0U},         /* Line seen again: resumes, no reboot. */
    {true,true,LEFT,0.0,-61.0,200.0,0U}, {true,true,STRAIGHT,0.0,-61.0,200.0,0U},
    {false,false,STRAIGHT,0.0,-61.0,200.0,0U},
    {false,false,STRAIGHT,0.2,-61.0,210.0,0U},   /* Both black then both white: side unknown, grace. */
    {false,false,PIVOT_LEFT,0.0,-61.0,220.0,0U}, /* Unknown side sweeps left first. */
    {false,false,PIVOT_RIGHT,0.0,0.0,240.0,0U},
    {false,false,PIVOT_LEFT,0.0,-122.0,280.0,0U},
    {false,false,BACK,0.0,-61.0,300.0,0U},
    {false,true,BACK,0.0,-61.0,290.0,0U},
    {false,true,RIGHT,0.0,-61.0,280.0,0U},       /* Line returns while reversing: following resumes. */
    {true,true,NONE,0.0,-61.0,0.0,2U}, {false,false,NONE,0.0,-61.0,0.0,1U},
    {false,false,NONE,0.0,-61.0,0.0,1U},         /* Cannot cancel obstacle. */
    {true,true,NONE,0.0,-61.0,0.0,3U},           /* READY and lost: never starts a search. */
    {true,true,STRAIGHT,0.0,-61.0,0.0,0U},
};
static unsigned checks,row,reads,pins,directions,pulls,requests;
static double clock_s=2.0;
static robot_status_t status;
static jmp_buf done;
void gpio_init(unsigned pin) { CHECK(pin==26U+pins); ++pins; }
void gpio_set_dir(unsigned pin,bool output) { CHECK(pin==26U+directions && !output); ++directions; }
void gpio_disable_pulls(unsigned pin) { CHECK(pin==26U+pulls); ++pulls; }
bool gpio_get(unsigned pin) { CHECK(pin==26U+reads); ++reads; return pin==26U ? inputs[row].right : inputs[row].centre; }
double motion_hal_time(void) { return clock_s; }
bool robot_update_subsystem(request_source_t source,const subsystem_status_t *line)
{
    CHECK(source==SOURCE_LINE && line->available && line->timestamp==motion_hal_time());
    CHECK(line->values[0]>=-1.0 && line->values[0]<=1.0);
    CHECK(line->values[1]==0.0 || line->values[1]==1.0);
    CHECK(line->values[2]==0.0 && line->values[3]==0.0);
    return true;
}
bool motion_get_status(robot_status_t *out)
{
    if (inputs[row].scenario==1U) {
        status.motion.state=DRIVING; status.motion.fault=F_NONE; status.motion.hold=false;
        status.owner_valid=true; status.owner=SOURCE_OBSTACLE; status.motion.id=5001U;
    } else if (inputs[row].scenario==2U) {
        status.motion.state=FAULT; status.motion.fault=F_TIMING; status.motion.hold=true;
    } else if (inputs[row].scenario==3U) {
        status.motion.state=READY; status.motion.fault=F_NONE; status.motion.hold=false;
        status.owner_valid=false;
    }
    status.motion.heading=inputs[row].heading*0.017453292519943295;
    status.motion.centre=inputs[row].travel/1000.0;
    *out=status; return true;
}
motion_result_t robot_submit(request_source_t source,uint32_t id,motion_mode_t mode,
                             double value,double left,double right)
{
    CHECK(source==SOURCE_LINE && mode==WHEEL_SPEED_TRACKING && value==0.0);
    static const double wheels[][2]={{0.0,0.0},{0.08,0.08},{0.05,0.11},{0.11,0.05},{0.0,0.08},{0.08,0.0},{0.0,0.0},{-0.06,-0.06}};
    CHECK(id==3001U && inputs[row].request!=NONE);
    CHECK(fabs(left-wheels[inputs[row].request][0])<1e-9 && fabs(right-wheels[inputs[row].request][1])<1e-9);
    status.motion.state=DRIVING; status.owner_valid=true; status.owner=SOURCE_LINE; status.motion.id=id;
    ++requests; return R_OK;
}
motion_result_t robot_submit_arc(request_source_t source,uint32_t id,double metres,
                                 double angle,double duration,double left,double right)
{ (void)source;(void)id;(void)metres;(void)angle;(void)duration;(void)left;(void)right;CHECK(false);return R_INVALID; }
void motion_stop(void) { CHECK(false); } /* Line loss must not latch a hold. */
int tm_printf(const UB *format,...) { (void)format; return 0; }
int tk_dly_tsk(unsigned delay)
{
    if (delay==2000U) { CHECK(pins==2U && directions==2U && pulls==2U && requests==0U); return 0; }
    CHECK(delay==20U && reads==2U);
    CHECK(requests==(inputs[row].request==NONE ? 0U : 1U));
    clock_s+=0.02+inputs[row].extra;
    reads=requests=0U; ++row;
    if (row==sizeof(inputs)/sizeof(inputs[0])) { longjmp(done,1); }
    return 0;
}
int tk_slp_tsk(int timeout) { (void)timeout;CHECK(false);return 0; }
int main(void)
{
    status.motion.state=READY;
    if (setjmp(done)==0) { line_task(0,NULL); }
    printf("PASS: %u production line-task integration checks\n",checks); return 0;
}
