/* Real lifecycle, tilt and terrain logic; fake only clock and device operations.
 * Stage 1 integration: no magnetic heading or calibration in buddy4.c.
 */
#include "../../../app_program/subsystem4/buddy4.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint32_t now, operation_ms;
static ER configure_status, read_status, clock_status;
static unsigned configurations, reads, checks;
static int16_t xyz[3];
static void check(int yes, char const *why)
{
    checks++; if (!yes) { fprintf(stderr,"FAIL: %s\n",why); exit(1); }
}
ER tk_get_otm(SYSTIM *time)
{
    time->lo=now; time->hi=0; return clock_status;
}
ER subsystem4_init(void) { configurations++; now+=operation_ms; return configure_status; }
ER gy511_read_acceleration(gy511_acceleration_t *sample)
{
    (void)sample; reads++; now+=operation_ms; return read_status;
}
ER gy511_acceleration_mg(gy511_acceleration_t const *sample, uint8_t axis, int16_t *value)
{
    (void)sample; *value=xyz[axis]; return E_OK;
}
static void reset(buddy4_t *c)
{
    now=0; operation_ms=0; configure_status=E_OK; read_status=E_OK;
    clock_status=E_OK; configurations=0; reads=0;
    xyz[0]=0; xyz[1]=0; xyz[2]=-1000;
    buddy4_begin(c,now);
}
static void tick(buddy4_t *c, uint32_t elapsed, uint8_t input)
{
    now+=elapsed; buddy4_step(c,input);
}
static void ready(buddy4_t *c)
{
    reset(c);
    buddy4_step(c,B4_LEVEL_ALLOWED);
    check(configurations==1U && reads==0U,"configure once");
    tick(c,B4_POSITION_MS-1U,B4_LEVEL_ALLOWED); check(reads==0U,"positioning wait");
    tick(c,1U,B4_LEVEL_ALLOWED);
    for (unsigned i=0;i<49;i++) { tick(c,20U,B4_LEVEL_ALLOWED); }
    check(buddy4_state(c)==B4_INIT,"49 capture samples insufficient");
    tick(c,20U,B4_LEVEL_ALLOWED);
    check(buddy4_state(c)==B4_MONITORING,"50 samples enter monitoring");
    tick(c,20U,0U); check((c->result&TILT_RESULT_VALID)!=0U,"fresh tilt available");
}
static void fail(buddy4_t *c)
{
    read_status=E_IO; tick(c,20U,0U);
    check(buddy4_state(c)==B4_RECOVERY && c->error==E_IO && c->result==0U,"communication failure enters recovery");
}
/* Every recovery tick below deliberately supplies zero input permissions. */
int main(void)
{
    buddy4_t c;
    float32_t reference[9];
    ready(&c);
    read_status=E_BUSY; tick(&c,20U,0U);
    check(buddy4_state(&c)==B4_MONITORING && c.result==0U,"one missed sample tolerated");
    tick(&c,100U,0U); check(buddy4_state(&c)==B4_MONITORING,"stale is not immediately a fault");
    tick(&c,380U,0U); check(buddy4_state(&c)==B4_RECOVERY && c.error==E_TMOUT,"500 ms loss recovers");
    ready(&c); xyz[2]=-1800; tick(&c,20U,0U);
    check(buddy4_state(&c)==B4_MONITORING && c.result==0U,"unsuitable tilt does not reconfigure");

    /* Recovery on a slope: no permissions, no new calibration, no old filter. */
    ready(&c);
    memcpy(reference,c.tilt.up,sizeof(c.tilt.up));
    memcpy(reference+3,c.tilt.forward,sizeof(c.tilt.forward));
    memcpy(reference+6,c.tilt.side,sizeof(c.tilt.side));
    fail(&c);
    check((c.tilt.progress & 0x80U)==0U,"old filter cleared on failure");
    read_status=E_OK; xyz[1]=-500; xyz[2]=-866;
    tick(&c,20U,0U);
    check(configurations==2U && buddy4_attempts(&c)==1U && c.result==0U,"automatic configure, still unavailable");
    tick(&c,20U,0U);
    check(buddy4_state(&c)==B4_MONITORING && c.error==E_OK,"automatic verification resumes");
    check(memcmp(reference,c.tilt.up,sizeof(c.tilt.up))==0,"up reference unchanged");
    check(memcmp(reference+3,c.tilt.forward,sizeof(c.tilt.forward))==0,"forward reference unchanged");
    check(memcmp(reference+6,c.tilt.side,sizeof(c.tilt.side))==0,"side reference unchanged");
    check(tilt_angle_tenths(c.result,0)>295 && tilt_angle_tenths(c.result,0)<305,"fresh slope seeds filter, not zeroed");
    fail(&c); tick(&c,20U,0U);
    check(configurations==3U && buddy4_attempts(&c)==1U,"new failure starts its own bounded episode");

    /* Persistent configuration failures: three attempts, separated by backoff. */
    ready(&c); fail(&c); configure_status=E_IO;
    tick(&c,20U,0U); check(buddy4_attempts(&c)==1U,"first retry counted");
    tick(&c,B4_RETRY_MS-1U,0U); check(configurations==2U,"no retry before backoff");
    tick(&c,1U,0U); check(buddy4_attempts(&c)==2U,"second retry at boundary");
    tick(&c,B4_RETRY_MS,0U);
    check(buddy4_state(&c)==B4_FAULT && configurations==4U && buddy4_attempts(&c)==3U,"third failure latches fault");
    unsigned previous_reads=reads;
    configure_status=E_OK; read_status=E_OK;
    for(unsigned i=0;i<10;i++) { tick(&c,10000U,0U); }
    check(configurations==4U && reads==previous_reads && c.result==0U,"replug after budget exhausted does not silently restart");
    tick(&c,20U,B4_CMD_RESET); check(buddy4_state(&c)==B4_FAULT,"reset still needs stopped confirmation");
    tick(&c,20U,B4_CMD_RESET|B4_MOTORS_STOPPED); check(buddy4_state(&c)==B4_FAULT,"reset still needs cause-cleared");
    tick(&c,20U,B4_CMD_RESET|B4_MOTORS_STOPPED|B4_CAUSE_CLEARED);
    check(buddy4_state(&c)==B4_INIT,"guarded reset");
    tick(&c,20U,0U); check(buddy4_state(&c)==B4_MONITORING,"warm reset preserves reference");

    /* Sensor returns on each permitted attempt, including the final attempt. */
    for(unsigned succeed_on=1U;succeed_on<=3U;succeed_on++)
    {
        ready(&c); fail(&c);
        for(unsigned attempt=1U;attempt<=succeed_on;attempt++)
        {
            configure_status=(attempt==succeed_on) ? E_OK:E_IO;
            tick(&c,(attempt==1U) ? 20U:B4_RETRY_MS,0U);
            check(buddy4_state(&c)==B4_RECOVERY && buddy4_attempts(&c)==attempt,"bounded retry/reconnection");
        }
        read_status=E_OK; tick(&c,20U,0U);
        check(buddy4_state(&c)==B4_MONITORING,"reconnection within budget recovers");
    }

    /* Reconfiguration alone is insufficient: each attempt needs fresh data. */
    ready(&c); fail(&c); read_status=E_BUSY;
    tick(&c,20U,0U);
    tick(&c,499U,0U);
    check(buddy4_state(&c)==B4_RECOVERY && c.result==0U,"no-ready stays verifying before deadline");
    tick(&c,1U,0U); check(c.error==E_TMOUT && buddy4_attempts(&c)==1U,"verification timeout boundary");
    tick(&c,B4_RETRY_MS-1U,0U); check(configurations==2U,"verify timeout uses backoff");
    tick(&c,1U,0U); tick(&c,500U,0U);
    tick(&c,B4_RETRY_MS,0U); tick(&c,B4_VERIFY_MS,0U);
    check(buddy4_state(&c)==B4_FAULT && configurations==4U,"three no-data verification attempts exhaust");

    ready(&c); fail(&c); tick(&c,20U,0U); /* configuration succeeds */
    tick(&c,20U,0U); /* read still E_IO */
    check(c.error==E_IO && buddy4_state(&c)==B4_RECOVERY,"verify read failure enters backoff");
    tick(&c,B4_RETRY_MS,B4_MOTORS_STOPPED);
    read_status=E_OK; tick(&c,20U,0U);
    check(buddy4_state(&c)==B4_MONITORING,"motor-status changes do not gate recovery");

    ready(&c); fail(&c); tick(&c,20U,0U);
    read_status=E_OK; xyz[2]=-1800; tick(&c,20U,0U);
    check(buddy4_state(&c)==B4_MONITORING && c.result==0U,"communication recovered but tilt still invalid");

    /* STOP prevents the next operation in every state and recovery phase. */
    for(uint8_t state=0U;state<=B4_STOP;state++)
    {
        reset(&c); c.control.state=state & 7U;
        buddy4_step(&c,B4_CMD_STOP|B4_CMD_RESET|B4_MOTORS_STOPPED|B4_CAUSE_CLEARED);
        check(buddy4_state(&c)==B4_STOP && configurations==0U && reads==0U,"STOP precedence in each state");
        tick(&c,1000U,0U); check(buddy4_state(&c)==B4_STOP,"STOP latched");
    }
    for(uint8_t phase=0U;phase<3U;phase++)
    {
        ready(&c); fail(&c); c.control.phase=phase & 3U;
        unsigned before_config=configurations, before_read=reads;
        tick(&c,20U,B4_CMD_STOP);
        check(configurations==before_config && reads==before_read && buddy4_state(&c)==B4_STOP,"STOP in configure/backoff/verify");
        check((c.tilt.progress&TILT_CALIBRATED)!=0U,"STOP retains reference");
    }

    /* If no reference was ever captured, recovery does not invent one. */
    reset(&c); configure_status=E_IO; buddy4_step(&c,0U);
    check(buddy4_state(&c)==B4_RECOVERY,"startup failure recoverable");
    configure_status=E_OK; tick(&c,20U,0U); tick(&c,20U,0U);
    check(buddy4_state(&c)==B4_INIT && (c.tilt.progress&TILT_CALIBRATED)==0U,"no reference requires INIT capture");
    unsigned before=reads;
    tick(&c,B4_POSITION_MS-1U,0U); check(reads==before,"positioning allowance after startup recovery");
    tick(&c,1U,0U); tick(&c,9999U,0U);
    check(reads==before,"no capture without level permission");
    tick(&c,1U,0U);
    check(buddy4_state(&c)==B4_FAULT,"capture timeout is not an endless automatic recovery loop");
    unsigned before_config=configurations;
    tick(&c,100000U,B4_LEVEL_ALLOWED); check(configurations==before_config,"capture fault stays latched");

    reset(&c); buddy4_step(&c,B4_LEVEL_ALLOWED); tick(&c,B4_POSITION_MS,B4_LEVEL_ALLOWED);
    for(unsigned i=0;i<500;i++)
    {
        xyz[0]=(i%2U==0U) ? 200:-200;
        tick(&c,20U,B4_LEVEL_ALLOWED);
    }
    check(buddy4_state(&c)==B4_FAULT && configurations==1U,"continuous motion cannot trigger repeated auto calibration");

    /* Wrap and blocking-time checks in both monitor and verification. */
    ready(&c); now=UINT32_MAX-50U; c.last_fresh=now;
    read_status=E_BUSY; tick(&c,100U,0U); check(buddy4_state(&c)==B4_MONITORING,"clock wrap tolerated");
    tick(&c,400U,0U); check(buddy4_state(&c)==B4_RECOVERY,"loss deadline across wrap");
    configure_status=E_IO; tick(&c,20U,0U);
    tick(&c,B4_RETRY_MS-1U,0U); check(buddy4_attempts(&c)==1U,"backoff after wrapped time");
    tick(&c,1U,0U); check(buddy4_attempts(&c)==2U,"retry across wrap");
    ready(&c); operation_ms=600U; tick(&c,20U,0U);
    check(buddy4_state(&c)==B4_RECOVERY,"blocking read time included");
    operation_ms=0U; tick(&c,20U,0U);
    operation_ms=500U; tick(&c,20U,0U);
    check(buddy4_state(&c)==B4_RECOVERY && c.error==E_TMOUT && c.result==0U,"late successful verification not accepted");
    ready(&c); clock_status=E_IO; tick(&c,20U,0U);
    check(buddy4_state(&c)==B4_FAULT,"clock failure latched");
    /* Terrain is fed from the first fresh tilt; nothing gates it. */
    /* Right wheel on a ~30 mm hump: 15 degrees sideways (X) plus some pitch (Y). */
    ready(&c);
    xyz[0]=259; xyz[1]=-100; xyz[2]=-961;
    for(unsigned i=0;i<50;i++) { tick(&c,20U,0U); }
    check((terrain_status(&c.terrain)&TERRAIN_PHASE_MASK)==TERRAIN_CLIMBING &&
          (terrain_status(&c.terrain)&TERRAIN_SLOW)!=0U,"one-wheel hump integrated with tilt");
    check(terrain_run_max(&c.terrain)==(TERRAIN_RUN_VALID|30U),"live peak 30 mm from sideways tilt");
    fail(&c);
    check((terrain_status(&c.terrain)&TERRAIN_VALID)==0U &&
          (terrain_status(&c.terrain)&TERRAIN_SLOW)!=0U,"recovery retains slow request and marks unknown");
    read_status=E_OK; tick(&c,20U,0U); tick(&c,20U,0U);
    xyz[0]=130; xyz[1]=0; xyz[2]=-991; /* ~15 mm: coming down */
    for(unsigned i=0;i<50;i++) { tick(&c,20U,0U); }
    check((terrain_status(&c.terrain)&TERRAIN_PHASE_MASK)==TERRAIN_DESCENDING,"terrain resumes phase after recovery");
    tick(&c,20U,B4_CMD_STOP);
    check((terrain_status(&c.terrain)&TERRAIN_VALID)==0U &&
          (terrain_status(&c.terrain)&TERRAIN_SLOW)!=0U,"STOP preserves slow and clears validity");
    /* Complete hump: up on the left wheel, back to level. Pitch alone never triggers. */
    ready(&c);
    xyz[0]=0; xyz[1]=-500; xyz[2]=-866; /* 30 degrees nose up, both wheels level */
    for(unsigned i=0;i<50;i++) { tick(&c,20U,0U); }
    check((terrain_status(&c.terrain)&TERRAIN_PHASE_MASK)==TERRAIN_WAITING &&
          terrain_run_max(&c.terrain)==0U,"pitch without sideways tilt is not a one-wheel hump");
    xyz[0]=-174; xyz[1]=0; xyz[2]=-985; /* left wheel ~20 mm */
    for(unsigned i=0;i<50;i++) { tick(&c,20U,0U); }
    xyz[0]=0; xyz[2]=-1000;
    for(unsigned i=0;i<50;i++) { tick(&c,20U,0U); }
    check(terrain_take_completed(&c.terrain) &&
          terrain_last_peak(&c.terrain)==20U,"left-wheel 20 mm hump completes with its peak");
    buddy4_step(NULL,0U); buddy4_begin(NULL,0U);
    printf("PASS: %u autonomous recovery, reference, deadline, STOP, reset and terrain checks\n",checks);
    return 0;
}
