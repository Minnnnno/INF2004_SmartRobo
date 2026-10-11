# Forward movement with background IMU monitoring

Use `build-imu-forward-test/buddy2_safe_motor_test.uf2` for this test.
This is the existing isolated motor-test profile, now built with the subsystem 4
IMU monitor. No application motor-control code was changed to enable this test.
The normal production UF2 does not expose these serial test commands.

## Run on the robot

1. Disconnect motor power. Hold BOOTSEL while connecting the Pico W over USB,
   then copy the test UF2 to the RPI-RP2 drive.
2. Wire IMU SDA to GP2 and SCL to GP3 with the tested supply and common ground.
   These are GPIO labels, not physical header pin numbers.
3. For the first motor check, support the chassis with both wheels clear.
   Connect the normal motor supply. Open the Pico USB serial port (COM6 if
   Windows still assigns that port), at 115200 with newline enabled.
4. Check for `safe_motor_test_ready=1` and `[IMU] I2C1 GP2/GP3 init=0`.
   Send `encdiag` to verify the test firmware ID if the startup banner was missed.
   A general `status` configuration fault is expected in this isolated profile:
   the normal controller is inhibited; the test actuator has its own readiness.
5. Send `forward` followed by Enter. Both motors receive 25% PWM and stop at
   the one-second hardware-alarm cutoff. This mode has no ARM step.
   Send `stop` to end a pulse early. `shutdown` blocks further tests until reboot.
6. Wait for `motor_test_done ... motors_stopped=1`, then wait at least another
   0.3 seconds before sending another movement command. The command's forward
   direction uses the existing motor signs; confirm physical wheel direction.
7. After confirming direction, a floor test can use the same one-second pulse
   with a clear path. Check the concurrent `[TERRAIN]` forward and side labels.

Expected motor messages:

```text
motor_test_started output_percent=25 cutoff_ms=1000
motor_test_done wheel=both direction=forward reason=timeout ... motors_stopped=1
```

The IMU samples approximately every 50 ms and prints approximately every 250 ms,
while stationary and during the pulse. It reports forward and side tilt
independently. NORMAL/SLOW/STOP are suggestions only: IMU tilt does not alter
this test pulse. Initial acceleration and braking can affect the tilt estimate.

`forward` here is a timed PWM pulse, not the production 10 cm distance command.
The production `motion_move_forward(id, metres)` API remains available for future
controller-triggered distance moves. A low PWM duty may not overcome motor load.

## Software verification

- Pico W test firmware compiled successfully with the IMU code and GP2/GP3.
- 339 IMU, terrain and background-task checks passed.
- 53 checks exercised the real serial parser: no startup movement, forward and
  reverse selection, CRLF, missing newline, repeated requests, stop, shutdown,
  rejected startup, long input and receive-overflow recovery. Actuator calls
  were mocked; the register-level actuator suite below checks their effects.
- 358 motor-actuator checks passed: both wheel directions, PWM limits,
  cutoff-before-enable ordering, timer cutoff, early stop, repeat rejection,
  cooldown and timer rollover.
- Existing motion/controller (2137), angle controller (673), and angle HAL
  (184) checks also passed.
- Disassembly confirms `motion_hal_write` is a return-only function in this
  profile, so the normal controller cannot compete with the test actuator.
  The linked image contains the console, IMU task and hardware-cutoff handler.

These are host tests and build checks. No firmware was flashed and no physical
motor direction, distance, current, cutoff timing or scheduling jitter was measured.

Repeat checks with native gcc/clang on PATH:

```text
python testing/subsystem4/host/run_tests.py
python tests/forward_console/run_tests.py
ctest --test-dir build-host-imu-clang --output-on-failure
```

The serial parser test leaves one existing warning disabled: the shared parser
sets an ARM variable that this profile does not consume. Other enabled warnings
are treated as errors.

## Build and artifact

The build uses SMP=OFF, WIFI=none, NET=none, MQTT=OFF,
ROBO_MANUAL_CONSOLE=ON, ROBO_SAFE_MOTOR_TEST=ON,
ROBO_ANGLE_TEST=OFF and ROBO_ENCODER_DIAGNOSTICS_ONLY=OFF.
Rebuild the configured directory with:

```text
cmake --build build-imu-forward-test --parallel
```

Firmware ID: b2-motor-test-v1-20261011T101955Z

SHA256: c5d1b73b47b9f226c5fc8c0a1cd190f12931b142c8c6888dc47237327dbab25b

Full build metadata is in `build-imu-forward-test/imu-forward-test-manifest.json`.

## Right motor direction correction

The user's wheel test showed left forward and right backward with signs {1,1}.
Commit 5825f0c1fad758d473d47d98810758e351317261 used {1,-1}; the motor-test
actuator itself is identical to that commit. The shared board configuration
now restores {1,-1}, reversing only the right motor's electrical drive.
This applies to production, motor-test and angle-test builds. Encoder signs,
geometry, IMU settings, pulse duration and output limits are unchanged.
Rebuild and reflash the relevant image to receive this correction. Physical
wheel direction still needs confirmation with the corrected image.
