# Buddy 2 movement functions

Include `steering.h` from your subsystem C file. These are task-context calls
through the Vehicle Controller; only Buddy 2 writes motor outputs.
Buddy 3 uses `SOURCE_LINE`; Buddy 5 uses `SOURCE_OBSTACLE`.

| Function | Purpose |
|---|---|
| `motion_move_mm(source, id, distance_mm)` | Encoder-based straight move; positive forward, negative backward. |
| `motion_turn_degrees(source, id, degrees)` | Encoder-based turn in place; positive left, negative right. Robot heading, not shaft angle. |
| `motion_set_wheel_speeds_mm_s(source, id, left_mm_s, right_mm_s)` | Continuous forward/backward or small steering corrections; positive wheel speed is forward. |
| `motion_get_measurements_mm(&measurements)` | Snapshot of measured wheel/centre speeds in mm/s, distances in mm, heading in degrees and validity/state/fault. |
| `motion_steer(source, id, speed_m_s, turn_deg_s)` | Continuous centre speed and heading rate; positive rate turns left. |
| `motion_steer_left(source, id, speed_m_s, correction_m_s)` | Gentle left adjustment: left=speed-correction, right=speed+correction. |
| `motion_steer_right(source, id, speed_m_s, correction_m_s)` | Gentle right adjustment: left=speed+correction, right=speed-correction. |
| `motion_steer_distance(source, id, metres, heading_deg, speed_m_s)` | Change heading by an angle while moving the specified distance. |
| `motion_steer_time(source, id, seconds, heading_deg, speed_m_s)` | Change heading while moving for the specified duration; checks trajectory feasibility. |
| `motion_get_feedback(&feedback)` | Same synchronized feedback in metres/m/s, plus heading/rate in degrees/degrees per second. |
| `motion_get_status(&status)` | Full shared status snapshot, primary fault and latest event for completion checks. |
| `motion_stop()` | Cancel and stop; latches a hold until the controller authorizes release. |

```c
#include "steering.h"

/* Alternatives: submit ONE finite movement at a time. */
motion_result_t result = motion_move_mm(SOURCE_LINE, 3001U, 200.0); // Forward 200 mm
// result = motion_move_mm(SOURCE_LINE, 3002U, -200.0);            // Backward 200 mm
// result = motion_turn_degrees(SOURCE_OBSTACLE, 5001U, 90.0);    // Left 90 degrees
// result = motion_turn_degrees(SOURCE_OBSTACLE, 5002U, -90.0);   // Right 90 degrees

/* Continuous alternatives: refresh every 20-50 ms with one session ID. */
// result = motion_set_wheel_speeds_mm_s(SOURCE_LINE, 3010U, 80.0, 80.0); // Straight
// result = motion_set_wheel_speeds_mm_s(SOURCE_LINE, 3010U, 70.0, 90.0); // Gentle left
// result = motion_set_wheel_speeds_mm_s(SOURCE_LINE, 3010U, 90.0, 70.0); // Gentle right
```

`R_OK` means queued. It does not mean movement has finished or been approved.
Handle `R_BUSY`, `R_HELD`, `R_FULL`, `R_STALE` and `R_INVALID`; do not blindly retry.
Submit finite moves once with a fresh nonzero ID, then observe `motion_get_status()`
for matching completion/fault events (including generation). `last_event` is a
latest-event snapshot, not an event log. Wait for completion/READY before starting
the next move. Ownership means Buddy 3 and Buddy 5 cannot drive simultaneously;
the controller arbitrates. Never clear faults or holds from a sensor task.

Finite moves use the controller's configured speed limit and acceleration ramp,
encoder stopping and final tolerance checks. Continuous targets are scaled together
if needed to the configured maximum speed (currently 250 mm/s), preserving curvature.
They expire after 300 ms without refresh. For a heading change while moving forward,
the existing `motion_steer_distance()` / `motion_steer_time()` helpers use metres,
degrees and m/s; see [STEERING.md](STEERING.md).

To measure a journey, save a baseline before starting, then subtract a later
snapshot. Read the baseline successfully before submitting the command:

```c
motion_measurements_mm_t before, now;
if (motion_get_measurements_mm(&before) && before.feedback_trusted && before.fault == F_NONE) {
    /* Save 'before' in your task state, submit a movement, then on a later cycle: */
    if (motion_get_measurements_mm(&now) && now.feedback_trusted && now.fault == F_NONE) {
        double travelled_mm = now.centre_distance_mm - before.centre_distance_mm;
        double total_path_mm = now.path_distance_mm - before.path_distance_mm;
        double speed_mm_s = now.centre_speed_mm_s;
        (void)travelled_mm; (void)total_path_mm; (void)speed_mm_s;
    }
}
```

Distances accumulate since motion initialization/reset. Centre distance and speed
are signed; path accumulates forward and reverse translation. In-place turning
can leave centre/path distance near zero while wheel distances change. Encoder
estimates cannot measure wheel slip on the mat. A failed read leaves caller data
unchanged; check the return value, timestamp and `feedback_trusted`.

Build your combined application with calibration profiles OFF. The existing
`build-battery-forward/buddy2_button_forward_2s.uf2` is an isolated button test:
its production controller remains inhibited, so these APIs cannot drive it.
The 24.9%/25% open-loop button calibration is separate from production PID speed
control; this API change does not change either tuning or production confirmation.

```powershell
cmake -S . -B build-production -G Ninja -DROBO_BATTERY_FORWARD_TEST=OFF -DROBO_SAFE_MOTOR_TEST=OFF -DROBO_MANUAL_CONSOLE=OFF -DROBO_ANGLE_TEST=OFF -DROBO_ENCODER_DIAGNOSTICS_ONLY=OFF -DROBO_LINE_STEERING=OFF -DSMP=OFF -DWIFI=none -DNET=none -DCONSOLE=usb_cdc -DPython3_EXECUTABLE=C:/Users/mervy/.pico-sdk/python/3.13.7/python.exe
cmake --build build-production -j 8
```

This produces `build-production/buddy2_production.uf2`. Merge your sensing calls
and rebuild before flashing; the functions alone do not add a new sensor trigger.

## Compact storage update (22 October 2026)

The functions and shared public structures above remain source/layout compatible.
The production motion core now quantizes accepted finite distances to1mm, turn
angles to1degree and independent wheel-speed targets to1mm/s. Halfway values round
away from zero. Limits remain +/-5000mm, +/-360degrees and +/-250mm/s per wheel;
smaller configured limits also apply. New invalid/out-of-range compact inputs
are rejected; they are not truncated or wrapped. Existing steering helpers still
perform their documented curvature-preserving speed scaling before submission.
Timed ARC adapters preserve duration and derive consistent wheel speeds from
rounded goals. Manual PWM fractions and untimed ARC wheel proportions retain
full precision. Encoder measurements still retain individual counts and sub-mm
resolution; the1mm command resolution is not the feedback resolution.

Minimum teammate usage:
```c
#include "steering.h"

/* Choose ONE finite command, submit once and wait for its matching event. */
/* Forward300mm: motion_move_mm(SOURCE_LINE, 3001u, 300.0); */
/* Back100mm:    motion_move_mm(SOURCE_OBSTACLE, 5001u, -100.0); */
/* Left90deg:   motion_turn_degrees(SOURCE_OBSTACLE, 5002u, 90.0); */
/* Right45deg:  motion_turn_degrees(SOURCE_OBSTACLE, 5003u, -45.0); */

/* Continuous examples: refresh ONE choice every20-50ms with the same ID. */
/* Straight80mm/s: motion_set_wheel_speeds_mm_s(SOURCE_LINE, 3010u, 80.0, 80.0); */
/* Gentle left:    motion_set_wheel_speeds_mm_s(SOURCE_LINE, 3010u, 70.0, 90.0); */
/* Gentle right:   motion_set_wheel_speeds_mm_s(SOURCE_LINE, 3010u, 90.0, 70.0); */
```

Read `motion_get_measurements_mm()` into `motion_measurements_mm_t` and use
`centre_speed_mm_s` for signed speed, `centre_distance_mm` for signed cumulative
displacement, `path_distance_mm` for total translational path, and
`left_distance_mm`/`right_distance_mm` for individual wheel distances. Subtract
before/after snapshots for a journey. Require a successful read, recent timestamp,
`feedback_trusted` and no fault before navigation decisions. Refresh expiry is
300ms; normal zero-speed wheel commands are different from the latched global
`motion_stop()`. Teammates must not clear controller holds/faults themselves.

The latest representation build is
`build-buddy2-compact-after/buddy2_production.uf2`. The earlier standalone mat-test
UF2 uses serial commands and has no production controller; it does not execute
these teammate request APIs. Its empirically tuned mat scaling/braking policy has
not been transferred into production PID by the representation work. Production
distance/speed/turn performance still needs validation in the combined robot.

See [Buddy2 representation audit](BUDDY2_COMPACT_REPRESENTATION.md) for the bit
layout, retained precision, measured RAM/flash costs, BARR-C notes, public-schema
changes deferred to the team, and remaining hardware checks.
