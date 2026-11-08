# Integrated motor, IR, IMU and ultrasonic firmware

The normal firmware enables these together:

| Component | Task / owner | Hardware |
| --- | --- | --- |
| Buddy 2 motion | motion_task, priority 2, 4096-byte stack | Motors GP8/GP9 and GP10/GP11; encoders GP2..GP5 |
| Vehicle controller | controller_task, priority 4, 4096-byte stack | Shared request/ownership bus |
| Buddy 3 IR line following | line_task, priority 8, 4096-byte stack | Right IR GP26; centre IR GP27 |
| Buddy 4 IMU/terrain | terrain_task, priority 8, 4096-byte stack | GY-511 on I2C1 SDA GP6 / SCL GP7, device iicb |
| Buddy 5 obstacle handling | obstacle_task, priority 4, 8192-byte stack | Ultrasonic TRIG GP16 / ECHO GP17; scanning servo GP15 |

`app_main.c` already composes these tasks behind their feature flags. The controller
bus and mutex exist before tasks start. Each task initializes and owns its sensors;
main does not repeatedly initialize them. The kernel initializes I2C1 before main.
I2C0 stays disabled so its default GP8/GP9 mapping cannot take the left motor pins.
Motor PWM uses slices 4/5, the servo uses slice 7, and ultrasonic pulse timing uses
slice 0; these configurations do not overwrite one another.

## Build

VS Code's Configure/Compile task explicitly passes the combined settings. For an
existing CMake build directory, configure them explicitly to replace cached OFF values:

```powershell
cmake -S . -B build -DROBO_LINE_STEERING=ON -DROBO_SUBSYSTEM4=ON -DROBO_SUBSYSTEM5=ON -DROBO_IR_MONITOR=OFF -DROBO_MAG_CAL=OFF -DROBO_MOTION_FAULTS_DISABLED=OFF -DROBO_MANUAL_CONSOLE=OFF -DROBO_SAFE_MOTOR_TEST=OFF -DROBO_ANGLE_TEST=OFF -DROBO_ENCODER_DIAGNOSTICS_ONLY=OFF
cmake --build build -j 8
```

Use `build/smartrobo_integrated.uf2`, which has identical bytes to
`build/mtkernel_3.uf2`. The build ID begins `smartrobo-integrated-v1-`; the startup
banner must show `line_task_enabled=1 subsystem4_enabled=1 subsystem5_enabled=1`
and `[MOTION] faults_enabled=1`. Calibration and IR-monitor images remain separate
profiles. Fresh calibration builds default line steering off; old caches may need
an explicit `-DROBO_LINE_STEERING=OFF` when switching to a calibration profile.

## Existing startup and handover behavior

- Motors begin inhibited and motion must qualify reliable feedback and rest.
- The line task initializes the two IR inputs and waits two seconds. With Buddy 4
  enabled it then waits for tilt-reference startup to finish. The existing wait
  ends when Buddy 4 leaves INIT/RECOVERY, or after ten seconds; an IMU fault does
  not itself latch a motor fault. Boot level and still for a valid reference.
- IR steering refreshes the same SOURCE_LINE command every 20 ms through Buddy 2.
- Buddy 4 publishes tilt/terrain/motion-class data to the shared status bus every
  20 ms; it reads motion status for rest qualification and speed/heading estimates.
- Buddy 5 paces ultrasonic triggers at least 60 ms apart, stops for obstacles,
  scans and performs its existing detour. While avoidance owns motion, the line
  task continues publishing IR status but does not overwrite motor commands.
  Fresh both-black IR status supports the return-to-line handover.
- Encoder faults and watchdog restart behavior retain their checked recovery
  policy; see [motion fault diagnostics](MOTION_FAULT_DIAGNOSTICS.md).

The barcode/hand-scan task is a separate opt-in test and is not enabled by this
line-following profile. WiFi/MQTT options are unchanged.

## Verification limits

Firmware compilation and host suites validate the software paths, simulated
sensor failures and ownership/handover rules. They do not establish electrical
operation, physical accuracy or combined worst-case timing on the robot. The
integrated firmware must be flashed and tested with the actual sensors and motors.
