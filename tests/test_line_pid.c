#include "line_test_fake_hal.h"
#include "steering.h"
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); exit(1); } } while (0)
typedef struct { bool left,right; line_direction_t request; bool stop; unsigned scenario; } input_t;
/* scenario: 0=normal ownership, 1=obstacle owns, 2=fault, 3=controller release. */
static const input_t inputs[]={
    {true,true,LINE_WAIT,false,0U}, {true,true,LINE_STRAIGHT,false,0U},
    {true,false,LINE_STRAIGHT,false,0U}, {true,false,LINE_LEFT,false,0U},
    {false,true,LINE_LEFT,false,0U}, {false,true,LINE_RIGHT,false,0U},
    {false,false,LINE_RIGHT,false,0U}, {false,false,LINE_WAIT,true,0U},
    {true,true,LINE_WAIT,false,0U}, {true,true,LINE_WAIT,false,0U}, /* Hold blocks re-entry. */
    {true,true,LINE_WAIT,false,2U}, {true,true,LINE_WAIT,false,1U},
    {false,false,LINE_WAIT,false,1U}, {false,false,LINE_WAIT,false,1U}, /* Cannot cancel obstacle. */
    {true,true,LINE_WAIT,false,3U}, {true,true,LINE_STRAIGHT,false,0U},
};
static unsigned checks,row,reads,pins,directions,pulls,requests,stops;
static robot_status_t status;
static jmp_buf done;
void gpio_init(unsigned pin) { CHECK(pin==26U+pins); ++pins; }
void gpio_set_dir(unsigned pin,bool output) { CHECK(pin==26U+directions && !output); ++directions; }
void gpio_disable_pulls(unsigned pin) { CHECK(pin==26U+pulls); ++pulls; }
bool gpio_get(unsigned pin) { CHECK(pin==26U+reads); ++reads; return pin==26U ? inputs[row].left : inputs[row].right; }
double motion_hal_time(void) { return 2.0+(double)row*0.02; }
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
    *out=status; return true;
}
motion_result_t robot_submit(request_source_t source,uint32_t id,motion_mode_t mode,
                             double value,double left,double right)
{
    CHECK(source==SOURCE_LINE && mode==WHEEL_SPEED_TRACKING && value==0.0);
    CHECK(id==(row<8U ? 3001U : 3002U));
    if (inputs[row].request==LINE_LEFT) { CHECK(left==0.07 && right==0.09); }
    else if (inputs[row].request==LINE_RIGHT) { CHECK(left==0.09 && right==0.07); }
    else { CHECK(inputs[row].request==LINE_STRAIGHT && left==0.08 && right==0.08); }
    status.motion.state=DRIVING; status.owner_valid=true; status.owner=SOURCE_LINE; status.motion.id=id;
    ++requests; return R_OK;
}
motion_result_t robot_submit_arc(request_source_t source,uint32_t id,double metres,
                                 double angle,double duration,double left,double right)
{ (void)source;(void)id;(void)metres;(void)angle;(void)duration;(void)left;(void)right;CHECK(false);return R_INVALID; }
void motion_stop(void)
{
    CHECK(inputs[row].stop && status.owner==SOURCE_LINE);
    status.motion.hold=true; status.motion.state=STOPPED; status.owner_valid=false; ++stops;
}
int tm_printf(const UB *format,...) { (void)format; return 0; }
int tk_dly_tsk(unsigned delay)
{
    if (delay==2000U) { CHECK(pins==2U && directions==2U && pulls==2U && requests==0U); return 0; }
    CHECK(delay==20U && reads==2U);
    CHECK(requests==(inputs[row].request==LINE_WAIT ? 0U : 1U));
    CHECK(stops==(inputs[row].stop ? 1U : 0U));
    reads=requests=stops=0U; ++row;
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
