# Buddy 4 host verification

Run `python testing/subsystem4/host/run_tests.py` from the project root.
Requires Python and GCC. All suites use warnings as errors.

- test_gy511.c: configuration success and fault injection (27 scenarios).
- test_acceleration.c: status/burst/close failures, unchanged output on error,
  parameter validation, five-byte size, all 4096 values on each of three axes.
- test_polling.c: real application loop/lifecycle with fake clock and sensor;
  normal releases, overrun skipping, wrap, stale/unavailable output and continued
  reporting after recovery attempts are exhausted.
- test_tilt.c: 416 reference/geometry/validity/hysteresis checks, including
  synthetic combined angles and the user's negative-Z mounting.
- test_buddy4.c: 159 checks for autonomous recovery, preserved reference, retry
  limits, configuration/verification failures, STOP/reset guards, capture
  deadlines, clock wrap, slow operations, clock failures and null parameters.

These compile the production logic against mocked hardware/kernel interfaces.
They do not run real hardware or establish physical accuracy or worst-case timing.
See docs/buddy4_states.md and docs/tilt.md for hardware checks and limitations.

- test_magnetic.c: 4096 XYZ vectors (12288 axis conversions), signed boundaries,
  output range rejection, lock draining, I2C errors and unchanged failed output.
- Lifecycle tests additionally verify magnetic availability, error independence,
  transition invalidation and stale tilt after a slow magnetic operation.
- Application tests check magnetic freshness boundaries and clock wrap.

- test_i2c_receive.py: compiles the production RP2040 receive switch arm against
  fake register access; checks byte delivery and final STOP for 1/2/6/16 bytes.
  This is not a complete driver or electrical bus simulation.

- test_heading.c: capture checks and synthetic slope sweeps including all 360 clockwise angles, capture, wrap,
  validity, timeout and reference retention. Lifecycle tests also cover heading.

- Heading mark: test_heading.c covers mark acceptance/rejection, wrap and retention;
  test_buddy4.c covers B4_CMD_MARK in each state; test_polling.c covers button debounce.

- test_mag_cal.c: one level turn completes at 330-360 degrees and recovers a known synthetic
  X/Y offset/scale distortion; calibrated heading within 2 degrees, Z nominal gain, partial-arc
  rejection, weak fields, point-ring wrap, external finish, spike guard, timeout and wrap.
  Lifecycle tests check buddy4_mag_cal_status() (TURNING/DONE/FAILED), tilted samples ignored,
  heading waits for calibration and calibration survives recovery/STOP.

- test_terrain.c: 51 checks for repeating hump sequences, flat crest, hysteresis,
  persistence, noise, invalid data, gap reset, clock wrap, completion consumption and explicit reset.
- Lifecycle integration also checks terrain phases, recovery and STOP behaviour.
