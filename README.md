# INF2004 SmartRobo ? integrated project template

This repository now uses the supplied CMake build and the
[sirfonzie project template](https://github.com/sirfonzie/INF2004_ProjectTemplate)
at commit c3b09544cb1fd6dac6ad4b8f523be0af59b44482.

Buddy 2's motion control is integrated into the template's application structure.
Open INF2004_SmartRobo.code-workspace in VS Code to use the Raspberry Pi Pico
extension. Compile with its Compile button or Ctrl+Shift+B; firmware is generated
at build/mtkernel_3.uf2.
The template kernel owns boot, vectors, scheduling and USB console; the earlier
Pico SDK startup adapter is archived and is not linked.

| Owner | Active source |
|---|---|
| Startup / task creation | app_program/app_main.c |
| Vehicle Controller | app_program/controller/vehicle_controller.c |
| Command bus, submission APIs and controller task | app_program/controller/robot_runtime.c |
| Buddy 1 WiFi/MQTT queues and hooks | app_program/subsystem1/comms.c |
| Buddy 2 motion, PID, encoders, configuration | app_program/subsystem2/ |
| Buddy 2 initialization, motion task and motor diagnostics | app_program/subsystem2/motion_runtime.c |
| Watchdog, retained crash reports and system supervision | app_program/system/system_runtime.c |
| Buddy 3 line and barcode | app_program/subsystem3/line_following.c, barcode.c |
| Buddy 4 heading/terrain | app_program/subsystem4/subsystem4.c |
| Buddy 5 obstacle avoidance | app_program/subsystem5/subsystem5.c |

Shared interfaces remain in include/robot.h and include/motion.h. Requests pass
through the Vehicle Controller into Motion Control. Stop/fault/shutdown bypass
ordinary requests and invalidate stale commands. Buddy 1's queues and status task
are initialized alongside motion. Normal firmware also starts IR line following,
Buddy 4 IMU/terrain monitoring and ultrasonic obstacle detection/servo scanning.
The controller consumes Buddy 1 start/stop/estop messages. WiFi/MQTT radio setup
still requires an explicit network build and local credentials.

`app_main.c` composes the application: it initializes components, creates/starts
the existing task table and calls periodic supervision. Live motion state stays
private to Buddy 2; the request bus and its mutex stay private to the controller.
The excluded `subsystem2.c`/`subsystem2.h` producer examples are legacy template
files; the production Buddy 2 entry points are declared in `motion_runtime.h`.

The standard VS Code build explicitly enables `ROBO_LINE_STEERING`,
`ROBO_SUBSYSTEM4` and `ROBO_SUBSYSTEM5`, with motion fault latching enabled.
Use `build/smartrobo_integrated.uf2` (the same bytes as `build/mtkernel_3.uf2`).
The startup banner reports `line_task_enabled=1 subsystem4_enabled=1 subsystem5_enabled=1`.
See [integrated startup, pins and validation](docs/INTEGRATED_FIRMWARE.md).

**Production motion is enabled after the user's hardware confirmation.**
Startup stays stopped, manual PWM remains disabled, and production output can
use the full 100% PWM range. Calibration profiles retain inhibited production control.
See [production firmware and subsystem integration](docs/PRODUCTION_INTEGRATION.md).
Motion faults latch by default and print the captured error and wheel/driver
evidence every two seconds; see [motion fault diagnostics](docs/MOTION_FAULT_DIAGNOSTICS.md).
Measured encoder counts/revolution are 2542.4 left and 2543.4 right.
For the separate one-second, 25% motor-test firmware and serial commands, see
[safe motor testing](docs/SAFE_MOTOR_TEST.md).
Current provisional effective wheel diameters are 67.5/67.9 mm and track width
is 115 mm; validate them under load. For the separate
encoder-based 90-degree firmware, commands and results, see
[angle testing](docs/ANGLE_TEST.md).
SMP is explicitly disabled for this motion integration. WiFi and networking are
off by default; imported driver code is available for Buddy 1.

See [build instructions](docs/BUILD.md), [integration changes](docs/TEMPLATE_INTEGRATION.md),
[API](docs/API.md), [calibration](docs/CALIBRATION.md), [hardware assumptions](docs/HARDWARE.md)
and [verification](docs/VERIFICATION.md). PID and accuracy report templates are in docs/.
No firmware has been flashed or physically tested.

The original template demo and README are preserved in docs/template_original/.
The previous SDK-based integration is archived in docs/previous_integration/.
These reference files are excluded from the application source glob.
