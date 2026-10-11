# Subsystem 4: GY-511 background monitoring

The existing `terrain_task` runs at priority 8 with a 4096-byte stack.
Motion Control runs at priority 2. Subsystem 4 configures the GY-511 once,
then samples every 50 ms while the robot is moving or stationary.
Console reports occur about every 250 ms. No new motor command is issued.
I2C transfers wait on kernel events with a 20 ms per-transfer timeout.

## Wiring and mounting

I2C1: SDA = GP2, SCL = GP3 (GPIO numbers, not physical header numbers).
Keep the already tested 3.3 V supply and common ground. I2C0 stays inactive;
the driver's old default pins overlap motors/encoders. UART and ADC drivers
remain disabled as in the original production build.
Fixed level reference: ACC (-61.33, 27.07, -1039.07) counts.
The mounted forward axis is Y; positive pitch = uphill. Side direction uses
the existing left/right sign. Verify those labels with the installed sensor.

## Reporting

Forward and side directions are independent, including simultaneous slopes.
The overall condition still reports roughness/excess tilt. NORMAL/SLOW/STOP
are suggestions, not motor actions. Acceleration during starts, stops and
turns can affect gravity-based tilt and roughness.

`robot_update_subsystem(SOURCE_TERRAIN, ...)` preserves docs/API.md:
heading radians, pitch radians, hump height metres, validity mask. Only
pitch is implemented; bit 1 (value 2) marks valid pitch. Heading and height
are zero with their validity bits clear. `available` marks fresh raw data.
An occasional E_BUSY keeps the last sample until 200 ms old, without changing
its timestamp. Stale data or I2C errors clear validity. A configuration failure
leaves monitoring unavailable until restart; it does not prevent motor tasks
from running once the kernel starts. Failure to initialise the I2C driver
itself remains a kernel startup error, as for other kernel drivers.

When Buddy 1's MQTT connection is online, nonblocking queues receive:

- robot/telemetry/imu_acc: x, y, z raw acceleration counts (about mg/count).
- robot/telemetry/imu_mag: x, y, z raw magnetic register counts.
- robot/telemetry/terrain: p=pitch, r=roll, t=total tilt (integer degrees),
  m=motion residual (mg), f=forward condition, s=side condition,
  c=overall condition, v=tilt validity (0/1).

Condition numbers follow terrain_condition_t in terrain.h. Packets are a
reporting batch; delivery is not atomic, and full queues can drop messages.
The existing production build has WIFI=none, NET=none and MQTT disabled;
these settings were not changed. Serial and shared status still work.
Hump height using Buddy 2's encoder distance is future work.

## How forward motion currently starts

`motion_move_forward(id, metres)` submits a distance request to the Controller;
the existing Motion Control task drives the motors. The optional manual
console's armed `forward` command requests 0.10 m. The current production
build has that console disabled and no caller that starts forward motion.
MQTT `start` releases a stopped robot; it does not request a forward move.
This integration does not add automatic movement.

## Checks

Run `python testing/subsystem4/host/run_tests.py` with native gcc on PATH.
Tests cover configuration/read errors, decoding, mounting reference,
independent slope directions and the actual background task with mocked
scheduler ticks, stale samples, sensor failure and full MQTT queues.
Firmware is built separately in build-imu-integration; no hardware was flashed.
Legacy subsystem4_test.c/task_monitor.c are excluded by the existing CMake
rules and do not test this implementation.

For a code-triggered 10 cm forward move, include robot.h and call
`motion_move_forward(command_id, 0.10)` once from an application task after
the motion state is READY. Check the submission result, then the shared
motion status/completion event; acceptance alone does not mean completion.
Use a new command ID for the next move. The IMU task runs automatically.

The current board_config.c forces hardware_confirmed=false whenever
ROBO_MANUAL_CONSOLE is enabled. Therefore enabling the console alone will
not allow its distance commands to run. Its armed `forward` handler exists,
but an explicitly configured manual motion build is needed to use it.

For an immediate serial-triggered movement test with IMU monitoring, see
[the forward-test procedure](../../docs/IMU_FORWARD_TEST.md). It uses the
separate one-second motor-test profile, with its own explicit `forward` command.
