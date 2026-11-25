/* Barcode shuttle test (analysis image only, -DROBO_BARCODE_SHUTTLE=1 on the
 * line steering profile). Drives the car straight forward and back over the
 * barcode so the reader can be scored again and again, in both directions. The
 * line task is off in this image; nothing here ships in production. */
#include "../../include/float_types.h"
#include "subsystems.h"
#if ROBO_SHUTTLE_TASK
#include "../subsystem2/template_hal.h"
#include <tm/tmonitor.h>
#include "board_config.h"
#include "motion_hal.h"
#include "robot.h"
#include "steering.h"
/* Plain digital inputs: the stream's ir= column shows where the car is on the
 * tape. */
#include "ir_io.h"

#define SHUTTLE_TRIPS 20U /* Forward-and-back round trips. */
/* One way: the 26 cm code plus a few cm of floor each side. */
#define SHUTTLE_DISTANCE_M 0.40
#define SHUTTLE_SPEED      0.05 /* m/s during barcode shuttle tests. */
/* Standing still after each leg: ends the reader's pass. */
#define SHUTTLE_PAUSE_MS    1500U
#define SHUTTLE_LEG_TIMEOUT 15.0 /* s. */
/* Abort if the car has not moved 5 mm this long after the command. */
#define SHUTTLE_IDLE_S 2.0

static bool shuttle_leg(uint32_t command_id, float64_t metres);

void
shuttle_task (INT code, void * p_context)
{
    uint32_t command_id = 4000U;

    (void)code;
    (void)p_context;
    ir_input_init(BOARD_LINE_R);
    ir_input_init(BOARD_LINE_C);
    tm_printf((const UB *)"shuttle: %u round trips of %u mm at %u mm/s, "
                          "starting in 3 s\n",
              SHUTTLE_TRIPS, (unsigned)(SHUTTLE_DISTANCE_M * 1000.0),
              (unsigned)(SHUTTLE_SPEED * 1000.0));
    (void)tk_dly_tsk(3000U);

    for (unsigned trip = 1U; trip <= SHUTTLE_TRIPS; ++trip)
    {
        for (unsigned back = 0U; back < 2U; ++back)
        {
            tm_printf((const UB *)"shuttle: trip %u %s\n", trip,
                      back ? "backward" : "forward");

            if (!shuttle_leg(++command_id,
                             back ? -SHUTTLE_DISTANCE_M : SHUTTLE_DISTANCE_M))
            {
                motion_stop();
                tm_printf((const UB *)"shuttle: leg failed, car stopped\n");

                for (;;)
                {
                    (void)tk_dly_tsk(1000U);
                } /* A task must never return (pc=0 hard fault). */
            }

            (void)tk_dly_tsk(SHUTTLE_PAUSE_MS);
        }
    }

    tm_printf((const UB *)"shuttle: done\n");

    for (;;)
    {
        (void)tk_dly_tsk(1000U);
    } /* A task must never return (pc=0 hard fault). */
}

/* One finite straight move (negative = backward). True when it completed. */
static bool
shuttle_leg (uint32_t command_id, float64_t metres)
{
    robot_status_t status;
    float64_t      start = motion_hal_time();
    float64_t      path  = 0.0;

    if (motion_get_status(&status))
    {
        path = status.motion.path;
    }

    if (R_OK
        != motion_steer_distance(SOURCE_LINE, command_id, metres, 0.0,
                                 SHUTTLE_SPEED))
    {
        tm_printf((const UB *)"shuttle: move was not accepted\n");
        return false;
    }

    for (;;)
    {
        float64_t elapsed;

        (void)tk_dly_tsk(20U);
        elapsed = motion_hal_time() - start;

        if (elapsed > SHUTTLE_LEG_TIMEOUT)
        {
            return false;
        }

        if (!motion_get_status(&status))
        {
            /* Nothing to do. */
        }
        else
        {
            if ((status.motion.hold) || (F_NONE != status.motion.fault))
            {
                return false;
            }

            if (status.last_event.id == command_id)
            /* With motion faults disabled a missed goal still completes, so
             * check the distance too. */
            {
                return (EV_COMPLETE == status.last_event.type)
                       && (F_NONE == status.last_event.fault)
                       && ((status.motion.path - path)
                           >= (0.9 * SHUTTLE_DISTANCE_M));
            }

            if ((elapsed > SHUTTLE_IDLE_S)
                && ((status.motion.path - path) < 0.005))
            {
                return false;
            }
        }
    }
}

#endif

/*** end of file ***/
