# Buddy 4 host verification

Run `python tests/subsystem4/run_tests.py` from the project root (it also works
from any other folder). Requires Python and a native C compiler (GCC by
default). Set `CC` to select another command such as `clang` or `zig cc`; quote
compiler paths containing spaces. The I2C receive test uses the same selection.
All suites build with warnings as errors; binaries go to `build/gy511-host/`.

Each suite compiles the production code in `app_program/subsystem4/` against
mocked kernel and I2C interfaces (`include/`). They do not run real hardware or
establish physical accuracy or worst-case timing: those are the S4-H tests in
`testing/subsystem4/TEST_LOG.md`, which is also where results are recorded.

| Suite | Code under test | Covers |
|---|---|---|
| `test_gy511.c` | `subsystem4.c` | GY-511 identity, register configuration with readback, failure at every step |
| `test_acceleration.c` | `gy511_read.c` | status/burst/close failures, unchanged output on error, all 4096 values per axis |
| `test_magnetic.c` | `gy511_read.c` | magnetometer lock/range/I2C failures, all 4096 values per axis |
| `test_motion_class.c` | `motion_class.c` | class rules and priority, debounce, OVERTURNED, yaw rate, turn angle, compass per turn, gaps |
| `test_mag_heading.c` | `mag_heading.c` | compass calibration, heading, field validity, unwrap |
| `test_tilt.c` | `tilt.c` | reference capture, geometry, validity envelope, hysteresis, wheel height |
| `test_lift.c` | `lift.c`, `tilt.c` | lift/overturn thresholds, 200 ms hold, clearing, hump immunity, sensor scale, gaps |
| `test_terrain.c` | `terrain.c`, `tilt.c` | hump phases, peak and side, run maximum, sway, gaps, wrap |
| `test_buddy4.c` | `buddy4.c` + tilt/terrain/lift | lifecycle, bounded recovery, retained reference, STOP/reset, lift latch |
| `test_terrain_task.c` | `terrain_task.c` (real task loop) | published snapshot and T_STATE codes, hump event, level gating, 20 ms grid, lift stop |
| `test_mag_cal_task.c` | `terrain_task.c` (`ROBO_MAG_CAL`) | calibration pivot left/right, stop, printed constants, failures |
| `test_i2c_receive.py` | `device/i2c/sysdepend/rp2040/i2c_rp2040.c` | receive switch arm: 1/2/6/16-byte completion and final STOP (not an electrical simulation) |
