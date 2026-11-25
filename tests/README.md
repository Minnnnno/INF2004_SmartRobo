# Host tests

Laptop tests that run the real firmware code with the hardware faked. Each folder
belongs to the subsystem whose code it tests; results are recorded in the matching
`testing/<folder>/TEST_LOG.md` (see [`testing/README.md`](../testing/README.md)).

| Folder | Owner | Tests | How to run |
|---|---|---|---|
| `subsystem1/` | Buddy 1 | `test_frame.c`: status/encoder frame packing and mapping; `web_dashboard_check.py`: web dashboard in demo mode | team suite (`buddy1_frame_checks`); `python tests/subsystem1/web_dashboard_check.py` |
| `subsystem2/` | Buddy 2 | `test_motion.c`, `test_motion_faults_disabled.c`, `test_motor_test.c`, `test_angle_control.c`, `test_angle_hal.c`, `test_steering.c`; fake HAL `motor_test_fake_hal.h` | team suite |
| `subsystem2/compact/` | Buddy 2 | compact motion storage and motor driver (own CMake project, UB sanitizer) | `cmake -S tests/subsystem2/compact -B build-s2-compact -G Ninja`, build, `ctest` |
| `subsystem2/forward_console/` | Buddy 2 | serial forward/calibration console (`manual_console.c`) | `python tests/subsystem2/forward_console/run_tests.py` (clang) |
| `subsystem3/` | Buddy 3 | `test_line_sensor.c`, `test_line_pid.c`, `test_barcode_decode.c` (+ `barcode/replay.txt` fixture), `test_route.c`; fake HAL `line_test_fake_hal.h` | team suite |
| `subsystem4/` | Buddy 4 | GY-511, tilt, humps, lift, motion class, compass, terrain task; `memory_probe.c` / `measure_compact.py` for the minimum-bit evidence | `python tests/subsystem4/run_tests.py` (see its README) |
| `subsystem5/` | Buddy 5 | obstacle runtime and servo | team suite |
| `integration/` | owners on both sides | `test_line_imu_wait.c`: Buddy 3's line task waits for Buddy 4's level reference (INT-05) | team suite (`line_imu_wait_checks`, `line_imu_wait_start_gate_checks`) |
| `shared/` | team | `test_crash_restart.c`: crash log and motion fault recovery (S2-U08); `tmonitor/`: kernel `tm_printf` formatting and the kernel stand-in headers it shares | team suite |

**Team suite** (every test registered in the root `CMakeLists.txt`):

```sh
cmake -S . -B build-host-cmake -G Ninja -DROBO_HOST_TESTS=ON
cmake --build build-host-cmake
ctest --test-dir build-host-cmake --output-on-failure
```

On Windows with MSYS2, add `-DCMAKE_C_COMPILER=clang`: MinGW `gcc` reports false
float-conversion errors under the `-Werror` flags.

**Adding a test:** put it in the folder of the subsystem whose code it tests; a test
of an interface between two subsystems goes in `integration/`. Register it in the
root `CMakeLists.txt` host-test block (or the folder's own runner) and give it the
test ID from its `TEST_LOG.md`.
