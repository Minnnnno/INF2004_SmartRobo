# Integrated motor, IR, IMU and obstacle firmware

Startup initializes Buddy 1's queues and starts its status task alongside the
Vehicle Controller, Motion Control, IR line-following, Buddy 4 IMU/terrain and
ultrasonic obstacle tasks. The normal CMake profile and VS Code build enable all
three sensor subsystems. Imported producer/consumer/monitor demos are excluded from
the robot build and preserved in docs/template_original/buddy1_demos.

Production motion is enabled by the user's explicit hardware confirmation.
Normal `board_motion_config()` now returns `hardware_confirmed=true`, with
`manual_enabled=false` and maximum output 100%. The user removed the previous
25% PWM cap on 4 November 2026. Verified geometry, counts and
polarity remain unchanged. Calibration profiles (including the manual console)
override confirmation to false and retain inhibited production controllers.

Motors stay stopped at startup. Motion reaches READY after valid setup, feedback
and stationary settling, then accepts controller-approved requests. Stop/hold,
feedback, command expiry, ownership and deadline checks remain active.

## Production UF2

The standard VS Code build produces `build/smartrobo_integrated.uf2`, with the
same bytes as `build/mtkernel_3.uf2`. The USB startup banner reports the build ID,
enabled line/IMU/obstacle tasks, hardware/manual flags and output cap. There is no
calibration serial command parser in this image. See [integrated firmware](INTEGRATED_FIRMWARE.md).

```powershell
cmake -S . -B build-production -G Ninja -DROBO_LINE_STEERING=ON -DROBO_SUBSYSTEM4=ON -DROBO_SUBSYSTEM5=ON -DROBO_IR_MONITOR=OFF -DROBO_MAG_CAL=OFF -DROBO_MOTION_FAULTS_DISABLED=OFF -DROBO_ANGLE_TEST=OFF -DROBO_SAFE_MOTOR_TEST=OFF -DROBO_ENCODER_DIAGNOSTICS_ONLY=OFF -DROBO_MANUAL_CONSOLE=OFF -DSMP=OFF -DWIFI=none -DNET=none -DCONSOLE=usb_cdc -DPython3_EXECUTABLE=C:/Users/mervy/.pico-sdk/python/3.13.7/python.exe
cmake --build build-production -j 8
```

The line task reads GP26/GP27, waits for the IMU's startup reference (with the
existing bounded fallback), and submits steering through the controller. It
publishes line status during obstacle detours and yields motor ownership to
Buddy 5. Existing PID gains and physical tracking/avoidance still require robot testing.

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
Update from origin/main, preserving any local work. Read TEAM.md first.
Integrate our real line-sensor task in app_program/subsystem3/line_following.c.
Read include/robot.h, docs/API.md and docs/PRODUCTION_INTEGRATION.md.
Use robot_submit(SOURCE_LINE, session_id, WHEEL_SPEED_TRACKING, 0.0,
left_speed_m_s, right_speed_m_s) to steer. Refresh every 50 ms with one stable
nonzero ID per continuous session. Handle return codes, ownership, holds,
faults and startup readiness through motion_get_status(). Submit finite
distance/turn requests once per event and wait for matching completion.
Use motion_stop() when sensing requires stopping; the Vehicle Controller
handles release. Do not access motor GPIO/PWM or calibration actuators.
Keep Buddy 1 communications, app_start_task(), the motion/Vehicle Controller
startup and existing task entry points. Do not restore the producer/consumer
demo startup or enable SMP. Production confirmation is enabled by the owner.
Keep the verified values,
100% production output range, manual_enabled=false and safety checks unchanged.
Implement actual sensor readings, not fabricated values. Build a combined
production image with all calibration profiles disabled and report its UF2.
```

Validation: the production build passed. Sanitized host motion/encoder/controller
tests passed 2137 checks, including actual production configuration, stopped
startup, READY transition, SOURCE_LINE acceptance/refresh, the output cap,
manual PWM rejection, stop/hold and disabled-confirmation faults. Calibration
regressions passed 673 angle-controller, 184 angle-HAL and 358 motor-test checks.
No hardware line-following or PID accuracy test was performed remotely.

The merged controller also consumes Buddy 1's start/stop/estop command queue.
Stop requests hold motion; estop latches a fault. Start requests only release
STOPPED motion with trusted stationary feedback and no fault, shutdown or
pending control. Start never clears a fault or initiates a movement itself.
Buddy 1's MQTT acknowledgement means the command was queued, not physically
completed or released. The default production UF2 uses WIFI=none/NET=none.
WiFi radio and full MQTT builds were also compile-checked; MQTT used only the
example placeholder credentials, and the temporary secrets file was removed.
Actual network connection requires local credentials and broker configuration.
