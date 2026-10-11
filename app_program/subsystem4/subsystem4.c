/* Background GY-511 owner. The Controller owns all motor decisions. */
#include "subsystem4.h"
#include "gy511_read.h"
#include "terrain.h"
#include "robot.h"
#include "motion_hal.h"
#include "../subsystem1/comms.h"
#include <tm/tmonitor.h>

#define S4_SAMPLE_MS (50U)
#define S4_MAX_AGE_S (0.20)
#define S4_REPORT_S (0.25)
#define S4_DEG_TO_RAD (0.017453292519943295)
#define S4_VALID_PITCH (2U)

static void subsystem4_report(gy511_sample_t const *p_sample,
                              terrain_result_t const *p_result);

void terrain_task(INT code, void *p_context)
{
    terrain_filter_t filter = {0};
    terrain_result_t result = {0};
    gy511_sample_t sample = {0};
    subsystem_status_t status = {0};
    double last_sample = -1.0;
    double next_report = 0.0;
    ER configuration_status = subsystem4_gy511_configure();
    bool b_ready = terrain_use_fixed_reference(&filter);

    (void)code;
    (void)p_context;
    b_ready = b_ready && (configuration_status == E_OK);
    tm_printf((UB const *)"[IMU] I2C1 GP2/GP3 init=%d\n",
              configuration_status);
    for (;;)
    {
        ER read_status = E_NOEXS;
        double now = motion_hal_time();

        if (b_ready)
        {
            read_status = gy511_read_sample(&sample);
        }
        if (read_status == E_OK)
        {
            last_sample = now;
            (void)terrain_update(&filter, sample.acceleration_x,
                                 sample.acceleration_y,
                                 sample.acceleration_z, &result);
        }
        else if ((read_status != E_BUSY) || (last_sample < 0.0) ||
                 ((now - last_sample) >= S4_MAX_AGE_S))
        {
            terrain_reset(&filter);
            result = (terrain_result_t){0};
            last_sample = -1.0;
        }

        status.available = last_sample >= 0.0;
        status.timestamp = status.available ? last_sample : now;
        /* Preserve the shared API: heading, pitch, height, validity mask.
         * Heading and height remain invalid until their owners supply them.
         */
        status.values[0] = 0.0;
        status.values[1] = (double)result.pitch_deg * S4_DEG_TO_RAD;
        status.values[2] = 0.0;
        status.values[3] = result.b_tilt_valid ? S4_VALID_PITCH : 0U;
        (void)robot_update_subsystem(SOURCE_TERRAIN, &status);
        if (now >= next_report)
        {
            next_report = now + S4_REPORT_S;
            if ((read_status == E_OK) && status.available)
            {
                subsystem4_report(&sample, &result);
            }
            else if (!status.available)
            {
                tm_printf((UB const *)
                          "[TERRAIN] Unavailable (error=%d)\n", read_status);
            }
        }
        (void)tk_dly_tsk(b_ready ? S4_SAMPLE_MS : 1000U);
    }
}

static void subsystem4_report(gy511_sample_t const *p_sample,
                              terrain_result_t const *p_result)
{
    /* Holds even eight full signed 32-bit decimal values plus JSON keys.
     * The queue's smaller limit is checked before submitting each message.
     */
    char payload[160];
    int length = 0;

    tm_printf((UB const *)
              "[IMU] ACC x=%d y=%d z=%d | MAG x=%d y=%d z=%d\n",
              (INT)p_sample->acceleration_x,
              (INT)p_sample->acceleration_y,
              (INT)p_sample->acceleration_z,
              (INT)p_sample->magnetic_x, (INT)p_sample->magnetic_y,
              (INT)p_sample->magnetic_z);
    tm_printf((UB const *)
              "[TERRAIN] pitch=%d roll=%d tilt=%d deg motion=%d mg "
              "| forward=%s side=%s | %s -> %s (suggested)\n",
              (INT)p_result->pitch_deg, (INT)p_result->roll_deg,
              (INT)p_result->tilt_deg, (INT)p_result->motion_mg,
              terrain_condition_name(p_result->pitch_condition),
              terrain_condition_name(p_result->roll_condition),
              terrain_condition_name(p_result->condition),
              terrain_action_name(p_result->condition));
    if (!comms_is_online())
    {
        return;
    }
    length = tm_sprintf((UB *)payload,
                       (UB const *)"{\"x\":%d,\"y\":%d,\"z\":%d}",
                      (int)p_sample->acceleration_x,
                      (int)p_sample->acceleration_y,
                      (int)p_sample->acceleration_z);
    if ((length >= 0) && (length < COMMS_PAYLOAD_MAX))
    {
        (void)comms_send_telemetry("imu_acc", payload);
    }
    length = tm_sprintf((UB *)payload,
                       (UB const *)"{\"x\":%d,\"y\":%d,\"z\":%d}",
                      (int)p_sample->magnetic_x, (int)p_sample->magnetic_y,
                      (int)p_sample->magnetic_z);
    if ((length >= 0) && (length < COMMS_PAYLOAD_MAX))
    {
        (void)comms_send_telemetry("imu_mag", payload);
    }
    /* Integer degrees; direction/overall values use terrain_condition_t.
     * v is tilt validity. Raw messages are one reporting batch, not atomic.
     */
    length = tm_sprintf((UB *)payload,
                      (UB const *)"{\"p\":%d,\"r\":%d,\"t\":%d,\"m\":%d,"
                      "\"f\":%d,\"s\":%d,\"c\":%d,\"v\":%d}",
                      (int)p_result->pitch_deg, (int)p_result->roll_deg,
                      (int)p_result->tilt_deg, (int)p_result->motion_mg,
                      (int)p_result->pitch_condition,
                      (int)p_result->roll_condition,
                      (int)p_result->condition,
                      p_result->b_tilt_valid ? 1 : 0);
    if ((length >= 0) && (length < COMMS_PAYLOAD_MAX))
    {
        (void)comms_send_telemetry("terrain", payload);
    }
}
