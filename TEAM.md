# SmartRobo team ownership

| Buddy | Directory | Responsibility |
|---|---|---|
| 1 | app_program/subsystem1 | WiFi/MQTT task; snapshot telemetry |
| 2 | app_program/subsystem2 | Motion, motor driver, encoder IRQs, odometry, PID, calibration |
| 3 | app_program/subsystem3 | Line task and barcode task |
| 4 | app_program/subsystem4 | Heading/terrain task; confirm hardware capability |
| 5 | app_program/subsystem5 | Obstacle scan/avoidance task |

Vehicle Controller: app_program/controller. Startup and seven application tasks:
app_program/app_main.c. Kernel USB service is an additional template service task.
Buddy 1's `comms.c` adds its periodic status task (priority 20), telemetry/event
queues and the command buffer consumed by the Vehicle Controller. Legacy
producer/consumer/monitor demos are excluded from the robot build; their originals
are preserved in docs/template_original/buddy1_demos.

Requests: include/robot.h. Sources submit to the controller, never motor GPIO.
Status is copied under a micro T-Kernel inheritance mutex. Stop/fault/shutdown are
separate control latches. Motion priority 2, controller/obstacle 4, line/barcode/terrain
8, WiFi 12. Proposed motion period 10 ms with a 1 ms kernel tick; measure timing.

Pins: motors 8/9 and 10/11; encoders 4/6 and 5/7; I2C1 2/3; servo 12;
ultrasonic 16/17; line 26/27; barcode 28. GP23/24/25/29 belong to Pico W radio.
UART console reserves GP0/GP1 only when enabled.

GPIO IRQ 13 has one shared dispatcher in subsystem2/gpio_dispatch.c. Register
nonoverlapping masks via board_gpio_irq_register from core-0 initialization;
handlers acknowledge only their owned pins. Do not overwrite tk_def_int(13).
The template USB compatibility irq_add_shared_handler is USB-specific, not a
general GPIO dispatcher.

Hardware measurements, safety policy and interfaces: docs/HARDWARE.md,
docs/CALIBRATION.md and docs/API.md. Subsystem 4 supplies real IMU/tilt readings; other sensor placeholders remain inactive.

Subsystem 4 now owns I2C1 GP2 SDA / GP3 SCL and runs the GY-511
background monitor at the existing priority 8. I2C0 is not initialised.
See app_program/subsystem4/README.md for telemetry and validity semantics.
