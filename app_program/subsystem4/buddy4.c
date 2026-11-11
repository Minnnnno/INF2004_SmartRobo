/* Explicit lifecycle around the existing sensor/tilt functions. */
#include "buddy4.h"
/* NULL and kernel types are supplied by tkernel.h; avoid libc size_t clash. */

/* The two-bit phase is interpreted within its current state. */
#define B4_INIT_CONFIGURE (0U)// INIT step 1: set up the sensor
#define B4_INIT_POSITION  (1U)// INIT step 2: settle (B4_POSITION_MS) before capture
#define B4_INIT_CAPTURE   (2U)// INIT step 3: collect 50 stable samples
#define B4_RECOVERY_CONFIGURE (0U)// RECOVERY step 1: try configuring again
#define B4_RECOVERY_BACKOFF         (1U)// RECOVERY step 2: wait before the next try
#define B4_RECOVERY_VERIFY          (2U)// RECOVERY step 3: confirm fresh data arrives

/* Read this file starting with buddy4_step(): it chooses one handler. */
static bool buddy4_clock(buddy4_t *p_context, uint32_t *p_now_ms);
static void buddy4_enter(buddy4_t *p_context, uint8_t state,
                         uint32_t now_ms, ER error);
static void buddy4_initialise(buddy4_t *p_context, uint8_t inputs, uint32_t now_ms);
static void buddy4_monitor(buddy4_t *p_context, uint32_t now_ms);
static void buddy4_recover(buddy4_t *p_context, uint32_t now_ms);
static void buddy4_poll_magnetic(buddy4_t *p_context, uint32_t now_ms);

/* One bounded logical step; hardware transaction latency is documented.
 * STOP takes precedence over RESET and autonomous recovery. Commands are ignored in states
 * where they do not apply, so an old command must not be queued/replayed.
 */
void buddy4_step(buddy4_t *p_context, uint8_t inputs)
{
    uint32_t now_ms = 0U; //hold current time
    uint8_t state = B4_INIT; //current state

    //safety check , if no conext was passed in , it leaves instead of crashing
    if (NULL == p_context)
    {
        return;
    }
    //get the time, ask kernel for current ms and store them in now_ms
    if (!buddy4_clock(p_context, &now_ms))//if kernel clock fail has already moved to fault just reuturns
    {
        return;
    }
    p_context->result = 0U;//clear old result
    state = buddy4_state(p_context); //read the 3 bit state from control byte

    //Stop has the Highest Priority
    if (0U != (inputs & B4_CMD_STOP))
    {
        buddy4_enter(p_context, B4_STOP, now_ms, p_context->error);
        return;
    }
    if ((B4_STOP == state) || (B4_FAULT == state))
    {
        uint8_t required = B4_CMD_RESET | B4_MOTORS_STOPPED | B4_CAUSE_CLEARED;
        if (required == (inputs & required))
        {
            buddy4_enter(p_context, B4_INIT, now_ms, E_OK);
        }
        return;
    }
    //run current state handler
    switch (state)
    {
        case B4_INIT:
            buddy4_initialise(p_context, inputs, now_ms);//configures and calibrates
            break;
        case B4_MONITORING:
            buddy4_monitor(p_context, now_ms);// reads and calculates tilt
            break;
        case B4_RECOVERY:  //retries the sensor
            buddy4_recover(p_context, now_ms);
            break;
        default:
            buddy4_enter(p_context, B4_FAULT, now_ms, E_PAR);
            break;
    }
    if (B4_MONITORING == buddy4_state(p_context))
    {
        /* Recheck time after synchronous operations; never feed old tilt. */
        if (!buddy4_clock(p_context, &now_ms))
        {
            return;
        }
        /* Hump height = right-minus-left wheel contact height (pitch-independent).
         * Suspended while lifted or steeply tilted, so lifting the robot by one
         * end does not record a fake hump. */
        bool fresh = (0U != (p_context->result & TILT_RESULT_VALID)) &&
                     ((now_ms - p_context->last_fresh) < B4_STALE_MS) &&
                     !lift_suspect(&p_context->lift);
        terrain_step(&p_context->terrain, fresh,
                     fresh ? tilt_side_height_tenths(&p_context->tilt, B4_TRACK_MM) : 0,
                     now_ms);
    }
}
/* Extract the three-bit state. NULL is treated as a fault for diagnostics. */
uint8_t buddy4_state(buddy4_t const *p_context)
{
    return (NULL == p_context) ? B4_FAULT : p_context->control.state;
}
/* Number of reconfiguration attempts in this recovery episode, 0..3. */
uint8_t buddy4_attempts(buddy4_t const *p_context)
{
    return (NULL == p_context) ? 0U : p_context->control.attempts;
}
/* Phase is private: INIT=config/wait/capture; RECOVERY=configure/backoff/verify. */
static uint8_t buddy4_phase(buddy4_t const *p_context)
{
    return p_context->control.phase;
}
/* Change subphase without changing state, retry count. */
static void buddy4_set_phase(buddy4_t *p_context, uint8_t phase, uint32_t now_ms)
{
    p_context->control.phase = phase & 3U; /* Limit assignment to two bits. */
    p_context->phase_started = now_ms;
}
/* Transition invalidates the current reading and transient filter/capture
 * history, retaining accepted calibration. A new episode starts at attempt zero.
 */
static void buddy4_enter(buddy4_t *p_context, uint8_t state, uint32_t now_ms, ER error)
{
    p_context->control = (buddy4_control_t){0};
    p_context->control.state = state & 7U; /* Limit assignment to three bits. */
    p_context->phase_started = now_ms;
    p_context->result = 0U;
    terrain_step(&p_context->terrain, false, 0, now_ms);
    p_context->error = error;
    tilt_discard_history(&p_context->tilt);
    lift_reset(&p_context->lift);
}
/* Monotonic time is sampled again after blocking device operations so timeout
 * decisions and sample timestamps do not use the pre-transfer time.
 */
static bool buddy4_clock(buddy4_t *p_context, uint32_t *p_now_ms)
{
    SYSTIM time = {0};
    ER status = tk_get_otm(&time);
    if (E_OK != status)
    {
        buddy4_enter(p_context, B4_FAULT, 0U, status);
        return false;
    }
    *p_now_ms = time.lo;
    return true;
}
/* Initialise private state with no accepted reference. */
void buddy4_begin(buddy4_t *p_context, uint32_t now_ms)
{
    if (NULL != p_context)
    {
        *p_context = (buddy4_t){0};
        buddy4_enter(p_context, B4_INIT, now_ms, E_OK);
        p_context->last_fresh = now_ms;
    }
}
/* Acquire/convert one fresh sample. No value is used after a failed read. */
static ER buddy4_read(int16_t xyz[3])
{
    gy511_acceleration_t sample = {{0}};
    ER status = gy511_read_acceleration(&sample);
    for (uint8_t axis = 0U; (E_OK == status) && (axis < 3U); axis++)
    {
        status = gy511_acceleration_mg(&sample, axis, &xyz[axis]);
    }
    return status;
}
/* Complete a failed retry: bound attempts, retain diagnostic and reference. */
static void buddy4_retry_failed(buddy4_t *p_context, uint32_t now_ms, ER error)
{
    p_context->error = error;
    if (buddy4_attempts(p_context) >= B4_MAX_ATTEMPTS)
    {
        buddy4_enter(p_context, B4_FAULT, now_ms, error);
        p_context->control.attempts = B4_MAX_ATTEMPTS;
    }
    else
    {
        buddy4_set_phase(p_context, B4_RECOVERY_BACKOFF, now_ms);
    }
}
/* Configure once, allow positioning, then capture one sample per update.
 * A retained reference bypasses capture after explicit reset/reinitialisation.
 */

 //INIT state handler
static void buddy4_initialise(buddy4_t *p_context, uint8_t inputs, uint32_t now_ms)
{
    ER status = E_OK;
    int16_t xyz[3] = {0};
    //configure (set up the senso)
    if (B4_INIT_CONFIGURE == buddy4_phase(p_context))
    {
        status = subsystem4_init();
        if (!buddy4_clock(p_context, &now_ms))
        {
            return;
        }
        if (E_OK != status)//if setup failed goes to recovery
        {
            buddy4_enter(p_context, B4_RECOVERY, now_ms, status);
            return;
        }
        p_context->last_fresh = now_ms; //reset last good data timer
        if (0U != (p_context->tilt.progress & TILT_CALIBRATED))// if already calibrated skip calibration
        {
            //go straight to monitorring
            buddy4_enter(p_context, B4_MONITORING, now_ms, E_OK);
        }
        else
        {
            //if not calibrated move on to position
            buddy4_set_phase(p_context, B4_INIT_POSITION, now_ms);
        }
        return;
    }
    //(POSITION , give you time to place the board)
    if (B4_INIT_POSITION == buddy4_phase(p_context))
    {
        //waits 5 second to so ask to set the robot down and level it
        if ((now_ms - p_context->phase_started) < B4_POSITION_MS)
        {
            return;
        }
        //after 5 second move to CAPTURE
        buddy4_set_phase(p_context, B4_INIT_CAPTURE, now_ms);
        p_context->last_fresh = now_ms;//reset timer.
        return;
    }
    // if 10s pass without completing calibration goes to fault.
    if ((now_ms - p_context->phase_started) >= B4_CAPTURE_MS) 
    {
        /* Reconfiguring cannot fix missing level permission or unstable pose. */
        buddy4_enter(p_context, B4_FAULT, now_ms, E_TMOUT);
        return;
    }
    if (0U == (inputs & B4_LEVEL_ALLOWED))
    {
        tilt_discard_history(&p_context->tilt);
        return;
    }
    //read one x,y,z sample 
    status = buddy4_read(xyz);
    if (!buddy4_clock(p_context, &now_ms))
    {
        return;
    }
    if ((now_ms - p_context->last_fresh) >= B4_STALE_MS)
    {
        tilt_discard_history(&p_context->tilt);
    }
    if ((now_ms - p_context->phase_started) >= B4_CAPTURE_MS)
    {
        /* Reconfiguring cannot fix missing level permission or unstable pose. */
        buddy4_enter(p_context, B4_FAULT, now_ms, E_TMOUT);
    }
    else if (E_OK == status)
    {
        p_context->last_fresh = now_ms;
        if (tilt_capture(&p_context->tilt, xyz[0], xyz[1], xyz[2]))
        {
            buddy4_enter(p_context, B4_MONITORING, now_ms, E_OK);
        }
    }
    else if (E_BUSY != status)
    {
        buddy4_enter(p_context, B4_RECOVERY, now_ms, status);
    }
    else if ((now_ms - p_context->last_fresh) >= B4_LOSS_MS)
    {
        buddy4_enter(p_context, B4_RECOVERY, now_ms, E_TMOUT);
    }
}
/* Not-ready is tolerated until prolonged loss. Unsuitable tilt stays in
 * MONITORING with result=0; it is not a sensor communication failure.
 */
//MONITORING STATE
static void buddy4_monitor(buddy4_t *p_context, uint32_t now_ms)
{
    int16_t xyz[3] = {0};
    ER status = E_OK;
    //check last good samele that arrived, if > 500ms sensor stopped giving data
    //goes to RECOVERY
    if ((now_ms - p_context->last_fresh) >= B4_LOSS_MS)
    {
        buddy4_enter(p_context, B4_RECOVERY, now_ms, E_TMOUT);
        return;
    }
    //read one x y z sample
    status = buddy4_read(xyz);
    if (!buddy4_clock(p_context, &now_ms))
    {
        return;
    }
    if ((now_ms - p_context->last_fresh) >= B4_STALE_MS)
    {
        tilt_discard_history(&p_context->tilt);
    }
    //Enter RECOVERY if bus or device failure
    if ((E_OK != status) && (E_BUSY != status))
    {
        buddy4_enter(p_context, B4_RECOVERY, now_ms, status);
    }
    //Busy , but too long without data, Goes to recovery
    else if ((now_ms - p_context->last_fresh) >= B4_LOSS_MS)
    {
        buddy4_enter(p_context, B4_RECOVERY, now_ms, E_TMOUT);
    }
    else
    {
        if (E_OK == status)
        {
            p_context->last_fresh = now_ms;
            p_context->error = E_OK;
            p_context->result = tilt_update(&p_context->tilt, xyz[0], xyz[1], xyz[2]);
            p_context->forward_mg = (0U != (p_context->result & TILT_RESULT_VALID)) ?
                tilt_forward_dynamic_mg(&p_context->tilt, xyz[0], xyz[1], xyz[2]) : 0;
            /* Raw sample, not the tilt result: tilt rejects these very poses. */
            (void)lift_step(&p_context->lift, &p_context->tilt, xyz[0], xyz[1], xyz[2], now_ms,
                            (now_ms - p_context->phase_started) >= B4_LIFT_SETTLE_MS);
        }
        else
        {
            p_context->error = E_BUSY;
        }
        buddy4_poll_magnetic(p_context, now_ms);
    }
}
/* Read the magnetometer (30 Hz) every MONITORING cycle for the compass
 * cross-check. Not ready (E_BUSY) keeps the last sample; an out-of-range
 * reading (E_IO, e.g. a magnet nearby) only invalidates the compass. Other
 * driver errors are bus failures and start RECOVERY like the accelerometer.
 */
static void buddy4_poll_magnetic(buddy4_t *p_context, uint32_t now_ms)
{
    gy511_magnetic_t sample = {{0}};
    ER status = gy511_read_magnetic(&sample);

    if (E_OK == status)
    {
        for (uint8_t axis = 0U; (E_OK == status) && (axis < 3U); axis++)
        {
            status = gy511_magnetic_counts(&sample, axis, &p_context->mag[axis]);
        }
        if (E_OK == status)
        {
            p_context->mag_ms = now_ms;
            p_context->mag_flags |= B4_MAG_HAVE;
        }
    }
    else if (E_IO == status)
    {
        p_context->mag_flags &= (uint8_t)~B4_MAG_HAVE;
    }
    else if (E_BUSY != status)
    {
        buddy4_enter(p_context, B4_RECOVERY, now_ms, status);
    }
}
bool buddy4_magnetic(buddy4_t const *p_context, uint32_t now_ms, uint32_t max_age_ms,
                     int16_t *p_x, int16_t *p_y)
{
    if ((NULL == p_context) || (NULL == p_x) || (NULL == p_y) ||
        (B4_MONITORING != buddy4_state(p_context)) ||
        (0U == (p_context->mag_flags & B4_MAG_HAVE)) ||
        ((now_ms - p_context->mag_ms) > max_age_ms))
    {
        return false;
    }
    *p_x = p_context->mag[0];
    *p_y = p_context->mag[1];
    return true;
}
uint8_t buddy4_lift(buddy4_t const *p_context)
{
    return ((NULL == p_context) || (B4_MONITORING != buddy4_state(p_context))) ?
           LIFT_NONE : lift_state(&p_context->lift);
}
/* Recover the owned sensor automatically, even without motor-status inputs.
 * Never capture a new reference here. Configuration plus a fresh sample proves
 * communication resumed, not that tilt is trustworthy while accelerating.
 * Attempts/backoff/verification are bounded; STOP is checked by buddy4_step().
 */

 //RECOVERY STATE HANDLER
static void buddy4_recover(buddy4_t *p_context, uint32_t now_ms)
{
    ER status = E_OK;
    int16_t xyz[3] = {0};
    //if 500ms pass with no sample , attempt failed
    if (B4_RECOVERY_VERIFY == buddy4_phase(p_context))
    {
        if ((now_ms - p_context->phase_started) >= B4_VERIFY_MS)
        {
            buddy4_retry_failed(p_context, now_ms, E_TMOUT);
            return;
        }
        //reads a sample, reread the time and check the 500 ms limit again in the case the read was slow
        status = buddy4_read(xyz);
        if (!buddy4_clock(p_context, &now_ms))
        {
            return;
        }
        if ((now_ms - p_context->phase_started) >= B4_VERIFY_MS)
        {
            buddy4_retry_failed(p_context, now_ms, E_TMOUT);
        }
        //if fresh data arrive, if sensor already calibrated , goes to MONITORING
        // if never calibrated go back to INIT(POSITION phase)
        else if (E_OK == status)
        {
            p_context->last_fresh = now_ms;
            if (0U != (p_context->tilt.progress & TILT_CALIBRATED))
            {
                buddy4_enter(p_context, B4_MONITORING, now_ms, E_OK);
                p_context->result = tilt_update(&p_context->tilt, xyz[0], xyz[1], xyz[2]);
            }
            else
            {
                buddy4_enter(p_context, B4_INIT, now_ms, E_OK);
                buddy4_set_phase(p_context, B4_INIT_POSITION, now_ms);
            }
        }
        else if (E_BUSY != status)
        {
            buddy4_retry_failed(p_context, now_ms, status);
        }
        return;
    }
    //wait before trying again
    if ((B4_RECOVERY_BACKOFF == buddy4_phase(p_context)) && ((now_ms - p_context->phase_started) < B4_RETRY_MS))
    {
        return;
    }
    //if 3 attempt has already been used, it goes to fault with the last error.
    if (buddy4_attempts(p_context) >= B4_MAX_ATTEMPTS)
    {
        buddy4_enter(p_context, B4_FAULT, now_ms, p_context->error);
        return;
    }
    p_context->control.attempts++; /* The maximum was checked above. */
    status = subsystem4_init();
    if (!buddy4_clock(p_context, &now_ms))
    {
        return;
    }
    if (E_OK == status)
    {
        buddy4_set_phase(p_context, B4_RECOVERY_VERIFY, now_ms);
    }
    else
    {
        buddy4_retry_failed(p_context, now_ms, status);
    }
}
/* Constant names for diagnostics. No state transition occurs here. */
char const *buddy4_state_name(uint8_t state)
{
    switch (state)
    {
        case B4_INIT:
            return "INIT";
        case B4_MONITORING:
            return "MONITORING";
        case B4_RECOVERY:
            return "SENSOR_RECOVERY";
        case B4_FAULT:
            return "SENSOR_FAULT";
        case B4_STOP:
            return "STOP";
        default:
            return "INVALID";
    }
}

