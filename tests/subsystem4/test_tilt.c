/* Validate geometry with synthetic gravity vectors, not sensor hardware. */
#include "../../app_program/subsystem4/tilt.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks;
static void require(int ok, char const *why)
{
    checks++;
    if (!ok) { fprintf(stderr,"FAIL: %s\n",why); exit(1); }
}
static void calibrate(tilt_context_t *c, int16_t x, int16_t y, int16_t z)
{
    *c=(tilt_context_t){0};
    for (unsigned i=0;i<49;i++) { require(!tilt_capture(c,x,y,z),"needs full window"); }
    require(tilt_capture(c,x,y,z),"reference captured");
}
/* Synthetic measurement in the captured coordinate frame. Pitch is forward
 * elevation; roll rotates the side/up components. Sums back into sensor XYZ. */
static uint32_t pose(tilt_context_t *c, float pitch, float roll)
{
    /* Physical directions are opposite the stored mathematical basis. */
    float p=-pitch/57.2957795F, r=-roll/57.2957795F;
    int16_t a[3];
    for(unsigned i=0;i<3;i++)
    {
        a[i]=(int16_t)roundf(1000.0F*(sinf(p)*c->forward[i]
             +cosf(p)*sinf(r)*c->side[i]+cosf(p)*cosf(r)*c->up[i]));
    }
    uint32_t result=0;
    for(unsigned i=0;i<40;i++) { result=tilt_update(c,a[0],a[1],a[2]); }
    return result;
}
/* Pose from pitch and the TRUE sideways tilt (sin = sideways gravity fraction),
 * as when one wheel of a two-wheel axle is raised. Returns the height for a
 * 115 mm track, after the filter has settled on a valid result.
 */
static int16_t raised(tilt_context_t *c, float pitch, float lateral, uint32_t *p_out)
{
    float f=sinf(-pitch/57.2957795F), s=sinf(-lateral/57.2957795F);
    float u=sqrtf(1.0F-f*f-s*s);
    int16_t a[3];
    for(unsigned i=0;i<3;i++)
    {
        a[i]=(int16_t)roundf(1000.0F*(f*c->forward[i]+s*c->side[i]+u*c->up[i]));
    }
    for(unsigned i=0;i<40;i++) { *p_out=tilt_update(c,a[0],a[1],a[2]); }
    return tilt_side_height_tenths(c,115U);
}
static void side_height_checks(void)
{
    tilt_context_t c={0}; uint32_t out=0U;
    require(tilt_side_height_tenths(NULL,115U)==0,"NULL height");
    require(tilt_side_height_tenths(&c,115U)==0,"no height before reference");
    calibrate(&c,0,0,-1000);
    require(tilt_side_height_tenths(&c,115U)==0,"no height before a filtered sample");
    require(abs(raised(&c,0,0,&out))<=1 && (out&TILT_RESULT_VALID)!=0U,"level height zero");
    for(int pitch=-15;pitch<=20;pitch+=5)
    {
        for(int lateral=-30;lateral<=30;lateral+=5)
        {
            double expected=1150.0*sin((double)lateral/57.29577951308232);
            int16_t h=raised(&c,(float)pitch,(float)lateral,&out);
            require((out&TILT_RESULT_VALID)!=0U,"valid raised pose");
            require(fabs((double)h-expected)<=2.0,"height = track x sideways fraction, any pitch");
        }
    }
    /* A 30 mm hump under the right wheel tilts the axle asin(30/115). */
    int16_t h=raised(&c,8,(float)(asin(30.0/115.0)*57.29577951308232),&out);
    require(abs(h-300)<=2,"30 mm under the right wheel reads 30.0 mm");
    require(strcmp(tilt_condition_name(out,1),"LEFT SIDE LOWER")==0,"positive height = right wheel up = left side lower");
    h=raised(&c,8,-(float)(asin(30.0/115.0)*57.29577951308232),&out);
    require(abs(h+300)<=2,"left wheel hump is negative");
    /* atan2(side, up) roll drifts with pitch; the height must not. */
    int16_t flat=raised(&c,0,15,&out); int16_t roll_flat=tilt_angle_tenths(out,1);
    int16_t pitched=raised(&c,20,15,&out); int16_t roll_pitched=tilt_angle_tenths(out,1);
    require(abs(flat-pitched)<=2 && roll_pitched-roll_flat>=5,"height pitch-independent where roll is not");
}
int main(void)
{
    tilt_context_t c={0}; uint32_t out;
    side_height_checks();
    require(tilt_update(&c,0,0,-1000)==0U,"no angle before reference");
    require(!tilt_capture(NULL,0,0,-1000),"NULL capture");
    require(tilt_update(NULL,0,0,-1000)==0U,"NULL update");
    calibrate(&c,0,0,-1000);
    out=pose(&c,0,0);
    require((out&TILT_RESULT_VALID)!=0U && tilt_angle_tenths(out,0)==0 && tilt_angle_tenths(out,1)==0,"level zero");
    require(strcmp(tilt_condition_name(out,0),"LEVEL")==0,"level label");
    for(int pitch=-60;pitch<=60;pitch+=15)
    {
        for(int roll=-60;roll<=60;roll+=15)
        {
            out=pose(&c,(float)pitch,(float)roll);
            require((out&TILT_RESULT_VALID)!=0U,"valid combined pose");
            require(abs((int)tilt_angle_tenths(out,0)-pitch*10)<=2,"pitch accuracy synthetic");
            require(abs((int)tilt_angle_tenths(out,1)-roll*10)<=2,"roll accuracy synthetic");
        }
    }
    calibrate(&c,0,0,-1000);
    out=tilt_update(&c,0,-707,-707);
    require(strcmp(tilt_condition_name(out,0),"FRONT UP")==0,"user front-up sign");
    tilt_discard_history(&c); out=tilt_update(&c,707,0,-707);
    require(strcmp(tilt_condition_name(out,1),"LEFT SIDE LOWER")==0,"user left-lower sign");
    tilt_discard_history(&c); out=tilt_update(&c,-707,300,-640);
    require(strcmp(tilt_condition_name(out,0),"FRONT DOWN")==0 && strcmp(tilt_condition_name(out,1),"RIGHT SIDE LOWER")==0,"opposite labels");
    out=pose(&c,6,0); require(strcmp(tilt_condition_name(out,0),"FRONT UP")==0,"enter at five");
    out=pose(&c,4,0); require(strcmp(tilt_condition_name(out,0),"FRONT UP")==0,"retain within hysteresis");
    out=pose(&c,2,0); require(strcmp(tilt_condition_name(out,0),"LEVEL")==0,"leave below three");
    out=pose(&c,4,0); require(strcmp(tilt_condition_name(out,0),"LEVEL")==0,"stay level below five");
    require(tilt_update(&c,0,0,0)==0U,"freefall rejected");
    require(tilt_update(&c,0,0,-1800)==0U,"large acceleration rejected");
    tilt_discard_history(&c); require(tilt_update(&c,0,0,1000)==0U,"inverted rejected");
    require((c.progress&TILT_CALIBRATED)!=0U,"bad input preserves reference");
    out=pose(&c,80,0); require(out==0U,"near vertical rejected");
    calibrate(&c,-18,2,-1044);
    out=tilt_update(&c,-18,2,-1044);
    require(abs(tilt_angle_tenths(out,0))<=1 && abs(tilt_angle_tenths(out,1))<=1,"negative-Z offset reference zero");
    tilt_discard_history(&c); out=pose(&c,15,-20);
    require(abs(tilt_angle_tenths(out,0)-150)<=2 && abs(tilt_angle_tenths(out,1)+200)<=2,"mounted reference rotation");
    c=(tilt_context_t){0};
    for(unsigned i=0;i<20;i++) { (void)tilt_capture(&c,0,0,-1000); }
    (void)tilt_capture(&c,200,0,-1000);
    require((c.progress&TILT_COUNT_MASK)==1U,"motion restarts window");
    tilt_discard_history(&c); require(c.progress==0U,"gap discards pending capture");
    require(!tilt_capture(&c,0,0,0) && c.progress==0U,"invalid reference sample");
    for(unsigned i=0;i<50;i++) { (void)tilt_capture(&c,0,1000,0); }
    require((c.progress&TILT_CALIBRATED)==0U,"degenerate forward reference rejected");
    printf("PASS: %u tilt geometry, reference, validity and hysteresis checks\n",checks);
    return 0;
}

