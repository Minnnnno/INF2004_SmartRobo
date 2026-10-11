# Controlled bring-up and calibration

For the current encoder counting investigation, first use the dedicated
[read-only firmware and verification procedure](ENCODER_INVESTIGATION.md).
Keep motor power disconnected and all configuration gates inhibited. Complete
independent one-wheel checks before attempting the later calibration steps.

No measurements or successful hardware tests are claimed. Keep a physical way to
remove motor power. Support the chassis with wheels clear for first tests; board
motor-test buttons drive full output and are not this software's limited test mode.

## First passive checks

1. Confirm physical board revision against HARDWARE.md, motor supply/current limits,
   encoder output levels, shared ground, and the proposed wiring. Keep motor power
   disconnected while flashing and checking startup.
2. Build a separate `build-template-manual` with `SMP=OFF` and
   `ROBO_MANUAL_CONSOLE=ON`, using the SDK/toolchain arguments in BUILD.md. Leave
   `hardware_confirmed=false`. Flash `build-template-manual/mtkernel_3.uf2` via BOOTSEL.
3. Open USB serial (e.g. 115200; CDC ignores baud). Send `status`. Expect FAULT /
   F_CONFIG with drive inhibited. Opening serial must never move a motor. With a
   meter/oscilloscope confirm GP8/9/10/11 stay LOW through startup. Account for
   board reset/boot-ROM time: software cannot guarantee pin behaviour before it runs.
4. Send `status`, rotate one wheel by hand through 10 complete revolutions, then
   send `status` again. Record modular count difference / 10. Repeat both directions
   and wheels. Configure `quadrature[]` for the real wiring first. x4 already counts
   all four A/B transitions; enter **software counts per wheel revolution**, including
   gearing once. Do not multiply by gearing or x4 a second time. Invalid counts
   should not grow. Single channel uses rising edges only and cannot measure direction.
5. Measure loaded rolling circumference for each wheel and divide by pi. Measure
   initial wheel-centre track width. Enter these genuine measurements in
   `app_program/subsystem2/board_config.c`, plus measured counts, encoder channel types and signs.
   For quadrature, hand-rotated robot-forward travel must increment both signed counts.
   Rebuild as needed. Do not fill missing measurements with the host-test fixture.

## First controlled motor pulse

6. After verifying board/electrical configuration, set `hardware_confirmed=true`
   and `manual_enabled=true`; retain a low `manual_limit` (default 0.15) and finite
   `manual_timeout` (default 0.5 s). Rebuild/flash the manual firmware. The normal
   firmware and the mere presence of USB never launch a pulse.
7. With wheels supported and motor power available, send `status`. Require READY
   (state 1), no fault (0), and rest=true. Send `ARM`, then within five seconds
   send `left`. This submits **only the left motor at +10% duty**, bounded by the
   configured half-second deadline. Right stays stopped. Inspect motion and signs;
   if it does not move at this output, do not interpret that as successful feedback.
   A fault may indicate insufficient torque, stall or missing encoder feedback.
8. Send `stop` at any time. Wait for STOPPED (state 4) and rest=true. To test the
   other wheel, explicitly verify checks, send `ARM`, `release_checked`, wait for
   READY, then `ARM`, `right`. Each ARM is single-use and expires after five seconds.
   `left_reverse`/`right_reverse` exercise opposite signs with the same limits.
   `status` prints signed interpretations in `counts` and modulo values in
   `raw_counts`; negative travel can appear near UINT32_MAX in the latter.
9. If signs are wrong, remove motor power and correct `motor_sign[]` and/or
   quadrature `encoder_sign[]`, then rebuild. Motor polarity and encoder polarity
   are separate. Single-channel sign follows the command and is not independent
   evidence of correct physical direction.
10. A pulse ends in a hold. On a fault, inspect the cause, repair it and independently
    verify rest. Only then send `ARM`, `reset_cause_cleared_rest_verified`. Wait
    for STOPPED after fresh settling, then `ARM`, `release_checked`. Never use this
    assertion to conceal disconnected or unreliable feedback. `shutdown` is terminal;
    reboot is required afterward.

## Closed-loop calibration

* Begin with conservative output/speed/acceleration. Tune each wheel with continuous
  speed requests from an explicit test task through `motion_set_wheel_speeds`,
  refreshing the same ID faster than command_lifetime. Start Ki/Kd at zero and
  increase Kp for stable response, add Ki to reduce steady error, then consider
  filtered derivative only if measurements justify it. Record sample time, target,
  measured speed and output. The supplied gains are provisional, not tuned.
* The manual console also supports armed `forward`/`backward` (0.10 m) and
  `turn_left`/`turn_right` (pi/2). Use these only after PID and signs have been checked.
  A U-turn uses the public angle API with pi; no separate timed-turn implementation.
* For each wheel, compare measured ground travel with estimated travel; update
  `diameter_new = diameter_old * measured_travel / estimated_travel`. Repeat under
  representative load. Encoder odometry cannot distinguish slip from calibration error.
* Execute measured turns in both directions. Adjust effective track width with
  `track_new = track_old * requested_angle / measured_angle` for runs whose encoder
  goal was reached. Repeat several angles/speeds; wheel mismatch/slip may require
  fixing wheel calibration before changing track.
* Tune straight_gain only on distance moves. Check that unequal continuous targets
  still produce steering. Confirm final angle/centre and left/right drift tolerances.
* Measure speed quantisation: one count corresponds to pi*diameter/counts_per_rev.
  Set stopped_speed and settle_time so slow creep is observed; choose feedback
  grace/timeouts for acceleration, reversal dead time and the lowest commanded speed.
  Default estimates are unfiltered differences, while PID derivative is filtered.
* Measure the actual stop distance at each operating speed for both brake/coast.
  Choose stopping deadline and deceleration accordingly. Reversal is forced through
  a configurable neutral interval; verify it is long enough for the real drivetrain.
* Test cancellation during motion and STOPPING, target expiry, unplugged encoder
  feedback, jam/no-progress under safely limited conditions, and command injection
  during hold/fault/shutdown. A stopped output is not proof of stationary wheels.
* Measure 10 ms loop timing/jitter, ISR load, stack headroom and worst-case stop
  latency with all eventual tasks active. Host simulation does not validate these.

Record every result in the report templates. Before normal mission firmware, set
`manual_enabled=false`, compile without `ROBO_MANUAL_CONSOLE`, and integrate genuine
navigation requests through the Vehicle Controller.
