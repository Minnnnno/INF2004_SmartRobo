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
| Buddy 1 WiFi/MQTT queues and hooks | app_program/subsystem1/comms.c |
| Buddy 2 motion, PID, encoders, configuration | app_program/subsystem2/ |
| Buddy 3 line and barcode | app_program/subsystem3/subsystem3.c |
| Buddy 4 heading/terrain | app_program/subsystem4/subsystem4.c |
| Buddy 5 obstacle avoidance | app_program/subsystem5/subsystem5.c |

Shared interfaces remain in include/robot.h and include/motion.h. Requests pass
through the Vehicle Controller into Motion Control. Stop/fault/shutdown bypass
ordinary requests and invalidate stale commands. Buddy 1's queues and status task
are initialized alongside motion; other sensor buddies sleep until implemented.
The controller consumes Buddy 1 start/stop/estop messages. WiFi/MQTT radio setup
still requires an explicit network build and local credentials.

**Production motion is enabled after the user's hardware confirmation.**
Startup stays stopped, manual PWM remains disabled, and production output is
capped at 25%. Calibration profiles retain inhibited production control.
See [production firmware and subsystem integration](docs/PRODUCTION_INTEGRATION.md).
Measured encoder counts/revolution are 2542.4 left and 2543.4 right.
For the separate one-second, 25% motor-test firmware and serial commands, see
[safe motor testing](docs/SAFE_MOTOR_TEST.md).
Verified wheel diameters are 65 mm and track width is 115 mm. For the separate
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
