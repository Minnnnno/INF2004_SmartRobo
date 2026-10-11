# Buddy 2 isolated motor test

Production configuration now uses `counts_per_rev={2542.4,2543.4}`.
Encoder left A/B is GP4/GP6; right A/B is GP5/GP7. The temporary GP12 mapping
and its validation exception are removed. Motor left A/B is GP8/GP9 and right
A/B is GP10/GP11. `hardware_confirmed=false`, `manual_enabled=false`, wheel
diameters and track width remain zero. Production motion is inhibited.
Based on the user's wheel-direction test, motor polarity is now `{1,-1}`:
left unchanged, right reversed. Both the production driver and isolated test
use this configuration. Encoder polarity stays `{1,1}` pending signed-count
verification with the corrected forward direction.

## Flash and commands

Flash this file using BOOTSEL:

`C:\Y2 Tri 1\Embedded Systems\Assignment\INF2004_SmartRobo\build-safe-motor-test\buddy2_safe_motor_test.uf2`

The delivered firmware ID is `b2-motor-test-v1-20261011T110944Z`.
SHA256: `362FE4CB2EE8E26E75F96B3E74E2B041E0B0E3C52535C9FEE4EC229177EFCD14`.
File size is 160768 bytes. Metadata is in
`build-safe-motor-test/encoder-build-manifest.json`.

1. With motor power disconnected, flash the UF2 and open USB serial at 115200
   with newline enabled. Send `encdiag` and `status`. Confirm the firmware ID,
   left GP4/GP6 and right GP5/GP7, encoder init/IRQ status, and motors stopped.
   An expected configuration fault remains visible because geometry is unset.
2. This image reads digital IR signals in subsystem 3: left DO -> GP26,
   right DO -> GP27. Move the signal wires from AO to DO with power disconnected.
   DO must be 3.3 V compatible, and the sensors share ground with the Pico.
   After a two-second startup delay it reports every 100 ms as
   `left_digital=0/1 right_digital=0/1`. These are actual GPIO levels, with no
   ADC conversion or numerical light threshold. Using the existing HIGH (1)
   detection level, while at least one reading is 1,
   both motors run
   continuously at 25%, with no one-second pauses or periodic cooldown.
   Both readings 0 stop sensor-driven movement on the next sample.
   Each valid either-HIGH sample refreshes a 300 ms hardware watchdog without
   resetting PWM; if the task stops updating, the watchdog stops the motors.
   Serial `stop` pauses automatic forward until both sensors
   become 0; `shutdown` also prevents all new sensor and
   serial tests until reboot. There is no steering logic or production enabling.
3. Disconnect power while reconnecting the motor wiring and the robot's proper
   motor supply. Support the robot with both wheels clear of the ground. On
   reconnecting power, motors start stopped; either sensor HIGH (1) can start
   forward automatically after the two-second sensing startup delay.
4. Send each command separately. Wait for `motor_test_done` and at least another
   0.3 seconds before sending the next movement command:

```text
left_forward
left_reverse
right_forward
right_reverse
forward
backward
stop
status
```

The four left/right commands apply exactly 25% PWM to the selected motor.
`forward` and `backward` apply 25% to both motors together, using the corrected
right motor polarity. A hardware timer
interrupt stops both motors at the one-second deadline. `stop` ends the test
early when parsed. There is no ARM step in this profile. Starting another test
while active, while a result is pending, or during cooldown is rejected and
never extends the deadline. `shutdown` stops and disables subsequent tests
until a reboot. `encdiag` and `usbdiag` remain available.

Results have this form:

```text
motor_test_done wheel=left direction=forward reason=timeout delta_counts=123,0 invalid_delta=0,0 motors_stopped=1
```

`delta_counts` is the signed change for left,right, captured at cutoff/stop;
subsequent coasting is not included. Only the tested wheel should change.
Forward and reverse should produce opposite count changes, with invalid totals
unchanged. Paired commands report `wheel=both` with both signed count changes.
The command names select opposite electrical drive directions; check
actual forward wheel rotation before finalizing motor/encoder polarity. At 25%
output a motor may not overcome static friction; lack of rotation alone does
not identify a software fault.

## Implementation and validation

`ROBO_SAFE_MOTOR_TEST` is a separate compile-time profile. The production HAL
write function compiles to a return instruction and controller release/reset
and ordinary movement commands are excluded from this console. This profile
does not set either production enabling flag or bypass production configuration
faults. Startup and completion use LOW/LOW outputs with PWM disabled.

The actuator initializes PWM slices 4/5 at 20 kHz: wrap 3999, divider 1.5625,
compare value 1000 gives exactly 25% duty at the kernel's asserted 125 MHz clock.
Both motors are disabled before each test. Individual commands enable only the
selected slice; paired commands enable slices 4 and 5 under one shared cutoff.
The sensor entry point uses the same PWM and direction setup but refreshes its
watchdog while running. Serial tests keep their one-second limit.
Alarm3/IRQ3 is reserved by this profile; source inspection found no
other user of it. The cutoff is armed before PWM starts and runs at the highest
interrupt priority, independently of console task scheduling. No ISR printing
is performed. Completion data is copied with interrupts masked, then printed
by the console task. Hardware cutoff timing has not been measured on the robot.

The safe motor-test Pico W build with IR triggering and the normal build with
the trigger disabled both passed. Sanitized host tests
passed 202 actual line-task checks and 721 motor-actuator checks.
The unchanged motion/encoder tests previously passed 1879 checks. Actuator
tests use the actual motor-test implementation with mock registers. These
cover stopped startup, failed initialization, both wheels/directions, exact PWM
duty, arm-before-drive ordering, individual motor isolation, paired motor
directions/stop/cutoff, busy rejection without
deadline extension, timeout, stop, cooldown, pending results, signed count
rollover, invalid deltas, shutdown blocking both entry points and timer low-word
rollover. Continuous-mode tests keep motors enabled for five seconds of sensor
updates and verify no PWM resets or stop/restart writes, then check stopping,
missed-update watchdog cutoff and shutdown. Line-task tests cover
GPIO input initialization, all four digital input combinations, displayed
0/1 values, continuous refresh, serial-stop pause/re-arm and rejected
triggers.
No firmware was flashed
or physically tested by the agent.

Changes are in `CMakeLists.txt`, `app_program/subsystem2/board_config.c`,
`drv_motion.c`, `manual_console.c`, new `motor_test.c`, `include/motor_test.h`,
and the motor-test host tests. Final encoder definitions remain in
`include/board_config.h`.

## Rebuild

```powershell
cmake -S . -B build-safe-motor-test -G Ninja -DROBO_SAFE_MOTOR_TEST=ON -DROBO_MANUAL_CONSOLE=ON -DROBO_ENCODER_DIAGNOSTICS_ONLY=OFF -DSMP=OFF -DWIFI=none -DNET=none -DCONSOLE=usb_cdc -DPython3_EXECUTABLE=C:/Users/mervy/.pico-sdk/python/3.13.7/python.exe
cmake --build build-safe-motor-test -j 8
```

A reconfiguration generates a new firmware ID. Use the resulting UF2 and
`encoder-build-id.txt` together; recompute the file hash after rebuilding.

Passing the motor tests establishes direction and encoder response. Autonomous
driving still requires measured wheel diameters/track width, confirmed polarity,
PID tuning, braking/stop checks and integration with the other robot subsystems.
Those enabling changes are intentionally left for subsequent validation.



