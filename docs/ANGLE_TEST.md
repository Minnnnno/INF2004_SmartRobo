# Buddy 2 encoder-based 90-degree tests

The user's verified settings are installed in `board_config.c`:

```c
.counts_per_rev = {2542.4, 2543.4},
.diameter = {0.065, 0.065},
.track = 0.115,
.motor_sign = {1, 1},
.encoder_sign = {1, 1},
```

Lengths are in metres. These motor signs intentionally replace the earlier
right-motor reversal, following the user's latest verified values.
Encoders remain left GP4/GP6, right GP5/GP7. Motors remain left GP8/GP9,
right GP10/GP11. `hardware_confirmed=false` and `manual_enabled=false` remain
unchanged in this angle-test profile. The production controller stays inhibited even though geometry is
now populated. No temporary GP12 encoder allocation exists.

## Firmware

Flash this UF2 via BOOTSEL with motor power disconnected:

`C:\Y2 Tri 1\Embedded Systems\Assignment\INF2004_SmartRobo\build-angle-test\buddy2_angle_test.uf2`

- Firmware ID: `b2-angle-test-v1-20261011T070827Z`
- Size: 159744 bytes
- SHA256: `940AFC74C0EA76CD9CD1A43B50E39DAB3917FAE2A3F621D2FD97548827A65946`
- Manifest: `build-angle-test/angle-build-manifest.json`

This is a separate build profile; the earlier one-second motor-test UF2 does
not provide angle commands. All outputs are stopped at startup. Explicit
`turn_left_90` or `turn_right_90` commands are the only movement commands in
this profile. `status` and `stop` are available, plus read-only `encdiag` and
`usbdiag`. `shutdown` stops and disallows further turns until reboot.

## Exact test procedure

1. Open USB serial with newline enabled (115200 is suitable). Send `status` and
   `encdiag`. Verify the firmware ID, correct encoder pins, successful encoder
   initialization, `angle_state=idle`, and `production_inhibited=1`. A production
   configuration fault is expected because hardware confirmation is false.
2. Support the wheels off the ground for the initial direction/stop check.
   With power off, reconnect the motor wiring and correct motor supply. After
   restoring power, motors remain stopped. Send `turn_left_90`: left should
   reverse and right should go forward. Send `stop` if either direction is wrong.
3. For angle measurement, place the robot on a level, nonslip surface with
   clearance to turn. Mark the chassis starting heading and a perpendicular
   90-degree reference with tape. Reset its physical starting heading before
   each trial.
4. Send **one command**, then wait for completion:

```text
turn_left_90
```

5. Wait for `angle_state=done` and `motors_stopped=1`, then send:

```text
status
```

6. Record the settled physical angle with a protractor, `angle_mdeg`, both
   signed encoder changes and invalid totals. Positive `angle_mdeg` means left;
   90000 means 90 degrees. Left delta should be negative and right positive.
7. Return the robot to its marked starting heading and repeat three times.
   Then test the other direction independently:

```text
turn_right_90
```

8. Again wait for completion and send `status`. Right-turn target is
   `target_mdeg=-90000`, with positive left counts and negative right counts.
   `stop` brakes both motors early at any point after the command is received.
   Wait at least 0.3 seconds after a manual stop or fault before another test.

Do not paste both turn commands together: a command received during turning or
braking is rejected and never extends the deadline. A fault or timeout stops
the test; it does not automatically retry.

## Control and result interpretation

Each in-place 90-degree turn requires wheel travel magnitude
`pi * track / 4 = 0.0903208 m`. Target counts are
`counts_per_rev * track / (4 * diameter)`: 1124.523 left and 1124.965 right,
so both wheels stop at 1125 observed counts from their individual baselines.
Counter subtraction handles signed changes and uint32 rollover.

Maximum output is 25%. During the last approximately 25 degrees of each wheel's
target travel, output reduces toward 12%. Each wheel brakes independently on
reaching its goal; a braked wheel cannot restart during the same turn.
Both outputs use LOW/LOW braking. Completion requires 0.3 seconds with no
encoder count changes. Failure to settle within one second is a timeout.
The final encoder-estimated angle must be within 3 degrees of the target to
report `done`; otherwise the result is `overshoot`.

Wrong-signed travel beyond four counts, any new invalid quadrature transition,
unreliable feedback, or one second without progress on an actively driven wheel
produces `feedback_fault`. An eight-second overall deadline stops both motors.
Alarm3/IRQ3 supplies a backup cutoff independent of task polling, at the highest
interrupt priority. The task also checks the deadline every nominal 10 ms.
Hardware cutoff latency and physical stopping distance have not been measured.

Status prints `angle_state`, signed `angle_mdeg`, signed target, commanded
output in permille, and per-wheel target counts. Raw/signed encoder counts and
invalid totals remain available. State-change reports include signed
`delta_counts=left,right` and `motors_stopped`. `done` means encoder-based
success; wheel slip can still make the physical angle differ.

Keep counts/revolution fixed for these trials. Compare repeated physical angles
with encoder-estimated angles before adjusting geometry or slowdown. Encoder
overshoot suggests braking/slowdown needs attention; correct encoder angle with
wrong physical angle suggests geometry or slip. A stall at low output is
reported as a fault rather than increasing power automatically.

## Implementation, build and verification

The separate `ROBO_ANGLE_TEST` profile cannot be combined with the motor-test or
read-only encoder profile. It requires USB CDC, single-core operation and no
WiFi/networking. The production HAL write function compiles to a return
instruction; ordinary movement, ARM, reset and release commands are absent.
`angle_control.c` implements the portable turn logic, `angle_test.c` owns the
isolated actuator/timer, and `manual_console.c` exposes commands and status.
No calibration values or production faults are bypassed to enable the controller.
No printing or floating-point angle calculations take place in the timer ISR.

```powershell
cmake -S . -B build-angle-test -G Ninja -DROBO_ANGLE_TEST=ON -DROBO_SAFE_MOTOR_TEST=OFF -DROBO_MANUAL_CONSOLE=ON -DROBO_ENCODER_DIAGNOSTICS_ONLY=OFF -DSMP=OFF -DWIFI=none -DNET=none -DCONSOLE=usb_cdc -DPython3_EXECUTABLE=C:/Users/mervy/.pico-sdk/python/3.13.7/python.exe
cmake --build build-angle-test -j 8
```

Pico W angle-test and production builds passed. Host tests passed 673 turn
controller checks, 184 actuator checks with mock registers, 1879 existing
motion/encoder checks, and 358 existing motor-test checks, using strict warnings
and undefined-behaviour sanitization. Tests cover both turn directions, target
math, slowdown, independent wheel stopping, no restart after reaching a goal,
settling, overshoot, stalls, invalid/wrong-sign feedback, deadlines, manual stop,
and backup ISR cutoff without task polling. No hardware was flashed or tested
by the agent.
