# Buddy 2 → Buddy 1: motor and encoder telemetry

Buddy 2 exposes live motor and encoder status through the existing shared
`motion_get_status()` API. Buddy 1 owns MQTT transmission and dashboard display.

## Read the shared status

Reuse the snapshot already taken by the communications task every 100 ms.
If `motion_get_status()` returns false, skip that update. Do not read encoder
hardware directly or access the motion task's private state.

```c
#include "robot.h"

robot_status_t status;

if (motion_get_status(&status)) {
    /* Actual wheel speeds, m/s */
    double left_speed  = status.motion.speed[0];
    double right_speed = status.motion.speed[1];

    /* Raw encoder counts, uint32_t */
    uint32_t left_count  = status.motion.encoder.count[0];
    uint32_t right_count = status.motion.encoder.count[1];

    /* Cumulative invalid quadrature-transition totals */
    uint32_t left_invalid  = status.motion.encoder.invalid[0];
    uint32_t right_invalid = status.motion.encoder.invalid[1];

    bool reliable = status.motion.encoder.reliable;
    bool feedback_trusted = status.motion.feedback_trusted;
    double sample_time = status.motion.encoder_sample_time;

    /* Map these copied values into your telemetry message here. */
}
```

This is an interface example; reuse the existing communications snapshot rather
than adding another polling task. Network operations must remain outside the
motion task and outside the shared-status lock.

## MQTT and dashboard work

1. Keep displaying left/right measured wheel speeds. The current telemetry
   mapper already converts m/s to mm/s.
2. Add live left/right encoder counts to MQTT and the dashboard. They are
   available in the local snapshot but are **not encoded in the existing
   21-byte status frame**. Use an additional documented message, or coordinate
   a versioned frame extension and update both the encoder and decoder.
3. Preserve all 32 count bits. Do not clamp counts into the speed or distance
   fields. Document the message format, topic, units and update rate.
4. Preferably display invalid-transition totals, feedback health, motion state
   and fault reason alongside the counts.
5. Mark disconnected or stale data clearly. Check timestamp freshness and
   finite values on the robot's clock; do not compare robot uptime directly
   with the browser's wall clock. Track message age on the dashboard too.

## Interpret the readings

| Field | Meaning |
|---|---|
| `encoder.count[0]` / `[1]` | Left/right raw encoder counts, modulo 2^32 |
| `encoder.invalid[0]` / `[1]` | Left/right cumulative invalid quadrature-transition totals |
| `encoder.reliable` | Acquisition reliability flag |
| `feedback_trusted` | Controller-qualified feedback health |
| `encoder_sample_time` | Encoder sampling timestamp, seconds on the robot's clock |

- Counts can decrease during reverse movement and wrap at 32 bits. Use modular
  deltas when calculating changes, not unsigned greater-than comparisons.
- An unchanged count is normal when stationary. During commanded movement,
  it may indicate a stalled wheel or an encoder problem; counts alone cannot
  distinguish those causes.
- Acquisition reliability alone does not prove that an encoder works. Also
  inspect feedback health and motion faults.
- Live counts continue updating while faulted. They are separate from the
  preserved `feedback_fault_sample` used for fault diagnosis.
- Reboot/reinitialization can restart the raw-count baseline. These counts are
  not reset-relative distance accumulators or persistent lifetime counters.

## Start, stop and fault handling

The current intended behavior is autonomous startup after flashing/booting.
Buddy 1 will handle any web Start gate during integration.

Route web Start/Stop/ESTOP through the existing communications command queue and
vehicle-controller interface. Do not write motor PWM from Wi-Fi code.

- **START:** releases an eligible safely stopped robot; it does not launch the
  RTOS tasks or clear a fault.
- **STOP:** requests a controlled stop.
- **ESTOP:** latches the separate emergency-stop fault.

Encoder faults require manual checks and the existing checked reset/release
procedure. Web Start must not bypass this procedure. The current MQTT command
acknowledgement means that the command was queued, not that the controller
successfully executed it; use subsequent status to show the actual outcome.

## References and validation

- [Shared API documentation](API.md)
- [Shared robot interface](../include/robot.h)
- [Motion status fields](../include/motion.h)
- [Existing MQTT frame definition](../include/telemetry_frame.h)
- [Existing telemetry mapper](../app_program/subsystem1/telemetry_map.c)

The encoder snapshot addition passed all 16 host test suites and the integrated
firmware build. It has not been flashed or verified on hardware. MQTT encoding
and dashboard display of the new encoder fields remain Buddy 1's integration
work.
