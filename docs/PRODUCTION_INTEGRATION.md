# Production motion and Buddy 3 integration

These changes are published on `buddy2/production-motion`. Shared `main` has
new Buddy 1 changes that replace the task startup and build setup. Merge this
branch with that work while preserving both Buddy 1 communications and the
`robot_submit()` implementation, Vehicle Controller and Motion Control tasks.
The UF2 described here builds the Buddy 2 workspace; it does not contain those
new remote Buddy 1 changes or a real line-sensing implementation.

Production motion is enabled by the user's explicit hardware confirmation.
Normal `board_motion_config()` now returns `hardware_confirmed=true`, with
`manual_enabled=false` and maximum output 25%. Verified geometry, counts and
polarity remain unchanged. Calibration profiles (including the manual console)
override confirmation to false and retain inhibited production controllers.

Motors stay stopped at startup. Motion reaches READY after valid setup, feedback
and stationary settling, then accepts controller-approved requests. Stop/hold,
feedback, command expiry, ownership and deadline checks remain active.

## Production UF2

Flash `build-production/buddy2_production.uf2` from the project directory.
`build-production/production-build-manifest.json` records the exact absolute
path, size, SHA256 and build ID. The USB startup banner reports the build ID,
hardware/manual flags and output cap. Normal builds also produce the same bytes
as `mtkernel_3.uf2`. There is no calibration serial command parser in this image.

```powershell
cmake -S . -B build-production -G Ninja -DROBO_ANGLE_TEST=OFF -DROBO_SAFE_MOTOR_TEST=OFF -DROBO_ENCODER_DIAGNOSTICS_ONLY=OFF -DROBO_MANUAL_CONSOLE=OFF -DSMP=OFF -DWIFI=none -DNET=none -DCONSOLE=usb_cdc -DPython3_EXECUTABLE=C:/Users/mervy/.pico-sdk/python/3.13.7/python.exe
cmake --build build-production -j 8
```

The current line task is an inactive placeholder. Your friend must merge his
real sensing code into this project and rebuild one combined production image.
Flashing the current image alone does not implement line following. Existing
PID gains and physical line-tracking performance still require robot testing.

## Function to call

Include `robot.h` and call `robot_submit()` from task context:

```c
motion_result_t result = robot_submit(
    SOURCE_LINE, 3001U, WHEEL_SPEED_TRACKING, 0.0, 0.08, 0.08);
```

This requests 0.08 m/s on each wheel. Use speeds computed from the real line
measurements for steering. Refresh valid targets every 50 ms with the same
source and ID, before the configured 300 ms expiry. A stable nonzero ID belongs
to one continuous session; use a new ID when the controller starts a new session.

Finite movement alternatives (submit once per event, not together):

```c
robot_submit(SOURCE_LINE, 3002U, DISTANCE_MOVE, 0.20, 0.0, 0.0); // Forward 20 cm
robot_submit(SOURCE_LINE, 3003U, DISTANCE_MOVE, -0.20, 0.0, 0.0); // Backward 20 cm
robot_submit(SOURCE_LINE, 3004U, ENCODER_TURN, 1.57079632679, 0.0, 0.0); // Left 90 degrees
robot_submit(SOURCE_LINE, 3005U, ENCODER_TURN, -1.57079632679, 0.0, 0.0); // Right 90 degrees
```

`R_OK` means queued, not approval/completion. Read `motion_get_status()` and
handle busy/held/full/stale/invalid returns. For finite movements, match command
IDs to completion/fault events before continuing. `last_event` is the latest
event snapshot, not a complete log. Check READY/DRIVING, ownership and faults.

Call `motion_stop()` on a condition requiring a stop, such as a transition to
lost-line status. Stop creates a hold; the Vehicle Controller must authorize
release after its checks. Do not repeatedly reset/release from the sensor task.
Expired continuous requests also stop and hold motion. Never write motor GPIO
or PWM directly and never use calibration actuators in the sensor task.

## Prompt for your teammate's Codex

```text
Integrate our real line-sensor task in app_program/subsystem3/subsystem3.c.
Read include/robot.h, docs/API.md and docs/PRODUCTION_INTEGRATION.md.
Use robot_submit(SOURCE_LINE, session_id, WHEEL_SPEED_TRACKING, 0.0,
left_speed_m_s, right_speed_m_s) to steer. Refresh every 50 ms with one stable
nonzero ID per continuous session. Handle return codes, ownership, holds,
faults and startup readiness through motion_get_status(). Submit finite
distance/turn requests once per event and wait for matching completion.
Use motion_stop() when sensing requires stopping; the Vehicle Controller
handles release. Do not access motor GPIO/PWM or calibration actuators.
Production confirmation is enabled by the owner. Keep the verified values,
25% output cap, manual_enabled=false and safety checks unchanged.
Implement actual sensor readings, not fabricated values. Build a combined
production image with all calibration profiles disabled and report its UF2.
```

Validation: the production build passed. Sanitized host motion/encoder/controller
tests passed 2129 checks, including actual production configuration, stopped
startup, READY transition, SOURCE_LINE acceptance/refresh, the output cap,
manual PWM rejection, stop/hold and disabled-confirmation faults. Calibration
regressions passed 673 angle-controller, 184 angle-HAL and 358 motor-test checks.
No hardware line-following or PID accuracy test was performed remotely.
