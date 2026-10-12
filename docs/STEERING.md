# Buddy 2 steering for Buddy 3 and Buddy 5

Include `steering.h`. Calls enqueue requests through the Vehicle Controller;
only Motion Control writes motor GPIO/PWM. Keep `SOURCE_LINE` for Buddy 3 and
`SOURCE_OBSTACLE` for Buddy 5. `R_OK` means queued, not completed. Handle
`R_BUSY`, `R_HELD`, `R_FULL`, `R_STALE` and `R_INVALID`. Observe READY/DRIVING,
ownership, the command ID and generation in `motion_get_status()`.

The differential-drive robot has no steering servo angle. These helpers use
different wheel speeds. Positive angle/turn rate means a left heading change;
negative means right. The new helpers take degrees, while the older core turn
API still takes radians.

```c
#include "steering.h"

// Continuous steering: refresh every 20-50 ms with one stable session ID.
motion_steer(SOURCE_LINE, 3001U, 0.08, 0.0);             // Straight, 0.08 m/s
motion_steer_left(SOURCE_LINE, 3001U, 0.08, 0.01);      // L=0.07, R=0.09 m/s
motion_steer_right(SOURCE_LINE, 3001U, 0.08, 0.01);     // L=0.09, R=0.07 m/s
motion_steer(SOURCE_OBSTACLE, 5001U, 0.08, 10.0);       // Left at 10 degrees/s

// Finite requests: submit once, then wait for the matching completion/fault.
motion_steer_distance(SOURCE_OBSTACLE, 5002U, 0.20, 10.0, 0.08);
// Centre travels forward 20 cm while heading changes left by 10 degrees.
motion_steer_time(SOURCE_OBSTACLE, 5003U, 2.0, -10.0, 0.08);
// Right 10 degrees over 2 s of motion; mean centre speed 0.08 m/s (16 cm).

motion_feedback_t before, after;
if (motion_get_feedback(&before) && before.feedback_trusted) {
    // Later, outside this block or in a subsequent task iteration:
    if (motion_get_feedback(&after) && after.feedback_trusted &&
        after.timestamp_s > before.timestamp_s) {
        double distance_m = after.centre_metres - before.centre_metres;
        double average_speed_m_s = distance_m / (after.timestamp_s - before.timestamp_s);
        // Store/use these values; do not block the motion task with printing.
        (void)average_speed_m_s;
    }
}
```

The continuous wheel-speed targets retain PID history when refreshed and use
the existing acceleration limit. Wheel limits scale continuous pairs together
to preserve their ratio. Continuous requests expire after 300 ms without a
refresh. A different subsystem cannot replace the current owner. Buddy 5
must obtain a controller-approved handover from line following before its arc;
after completion the Vehicle Controller can select the line mode again.

Finite arcs use left/right travel goals `distance +/- angle_rad * track/2`.
Distance arcs brake near the encoder goals. Timed arcs use a smooth start/end
trajectory; impossible peak speed/acceleration/time combinations are rejected.
Both stop and settle, then verify centre distance and heading. If the measured
goals were missed, the result is FINAL_GOAL rather than a false completion.
Elapsed motion time starts on controller acceptance, not queue submission;
braking/settling can take additional time. Encoder heading assumes no wheel slip.

Feedback includes each wheel's m/s and cumulative metres, centre speed,
signed centre travel, absolute centre path travel, heading degrees, turn rate,
sample timestamp, state/fault and trust. Save baselines to measure a run, and
discard them after an odometry reset. Counts become distance through
`delta_counts * pi * diameter / counts_per_rev`; speed divides by actual sample
time. Do not infer ground travel from wheel encoders when the wheels are slipping.

## Why the previous line test drifts and jerks

The old motor-test image applies fixed 25% PWM to both motors, with an OR go/stop
rule and no steering correction. Equal PWM need not produce equal wheel speed.
Digital sensor chatter and a loss/re-entry stop cycle can produce jerks.
The old isolated motor-test image remains available for direction/encoder tests.

The new production line profile reads DO on GP26/GP27 every 20 ms, requires two
equal consecutive observations, and requests gentle encoder/PID steering:

| Left black (1) | Right black (1) | Action |
| --- | --- | --- |
| 1 | 1 | Both wheel targets 0.08 m/s |
| 1 | 0 | Left correction: 0.07 / 0.09 m/s |
| 0 | 1 | Right correction: 0.09 / 0.07 m/s |
| 0 | 0 | Stop the owned line session and hold |

This retains the existing HIGH=black interpretation. Verify actual sensor
polarity, mounting height and placement on black/white before motor testing.
Digital inputs give coarse left/right information; they cannot measure a
continuous lateral line error. The shared line status keeps its existing
error/confidence slots: -1 left, +1 right, 0 centred/lost; confidence 1 only for
a stable detected line. Junction/reserved slots remain zero.

Stop/line loss creates a hold. The sensor task never clears faults or releases
holds itself; the Vehicle Controller must authorize release. Both wheels can
stay stopped after line loss/reacquisition until that authorization or reboot.
The line task does not cancel an obstacle-owned movement. The 25% output cap,
feedback checks and 50 ms runtime timing limit remain in effect. Startup sampling
now establishes its origin after task creation, without clearing setup/config/
feedback faults; runtime gaps, backwards time and repeated timestamps still fault.

## Build and robot verification

```powershell
cmake -S . -B build-line-steering -G Ninja -DROBO_LINE_STEERING=ON -DROBO_MANUAL_CONSOLE=OFF -DROBO_SAFE_MOTOR_TEST=OFF -DROBO_ANGLE_TEST=OFF -DROBO_ENCODER_DIAGNOSTICS_ONLY=OFF -DSMP=OFF -DWIFI=none -DNET=none -DCONSOLE=usb_cdc -DPython3_EXECUTABLE=C:/Users/mervy/.pico-sdk/python/3.13.7/python.exe
cmake --build build-line-steering -j 8
```

Flash `build-line-steering/buddy2_line_steering.uf2`. This is the PID production
line profile; `buddy2_safe_motor_test.uf2` remains an open-loop test. The new
image has no calibration serial command parser. It reports `line_pid` every
500 ms, including measured wheel speeds and PWM output. The build directory's
`line-steering-build-manifest.json` records the build ID and SHA256.

1. With motor power disconnected, verify black=1 and white=0 at the final
   mounted height. Fix loose wiring, tilt and sensitivity before changing gains.
2. Use the bounded motor-test profile with wheels raised to verify physical
   forward rotation and positive encoder changes for each wheel. Correct only
   the confirmed encoder polarity if counts have the wrong sign.
3. Test the new PID profile with wheels raised, then on a clear short track.
   Check straight, left-only and right-only conditions. Expect the matching
   target speeds and no controller faults. Missing movement/FEEDBACK means
   tuning, encoder feedback or hardware needs investigation; do not bypass it.
4. First tune each wheel's speed response at a low equal target. Compare
   measured m/s and output, adjust the per-wheel PID gains in board_config.c in
   small steps, and keep the 25% cap. Start with proportional control, then add
   enough integral action for steady speed error; reduce gains if it oscillates.
   PWM-to-speed measurements can support a later per-wheel feedforward fit;
   no unmeasured feedforward or PWM trim was inserted here.
5. Measure a straight ground run with a ruler and a before/after feedback
   snapshot. Check effective wheel circumference/calibration and mechanical
   alignment if measured wheel speeds agree but the robot still veers.
6. Tune the line correction (currently 0.01 m/s) only after straight wheel-speed
   control is stable. Increase gently if correction is too weak; decrease it or
   base speed if the robot hunts from side to side. These defaults are initial
   test values, not verified robot tuning.

Physical testing, motor friction, encoder polarity under powered forward
motion, wheel slip and PID accuracy cannot be established by host tests.

Validation: all seven registered host suites passed with strict compiler
warnings and undefined-behavior instrumentation (10,956 checks), plus the
53-check serial-command regression. Tests cover arc direction/completion,
timed-goal failure, startup/runtime timing, command refresh without PID reset,
line filtering, lost-line holds and obstacle ownership. Pico W builds passed
for the line-steering, production, safe motor-test and angle-test profiles.
