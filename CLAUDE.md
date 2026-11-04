# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for a 5-person team robot car (INF2004) on a **Raspberry Pi Pico W (RP2040)**, running **micro T-Kernel 3.0** (not FreeRTOS, not the bare Pico SDK runtime). The kernel owns boot2, vectors, `Reset_Handler`, `main` and the USB console; the application entry point is `usermain()` in [app_program/app_main.c](app_program/app_main.c). `pico_stdlib` is deliberately **not linked** — only Pico SDK headers (and TinyUSB / CYW43 / lwIP sources when enabled) are used. Hardware access in app code goes through T-Kernel BSP register helpers (`out_w`/`in_w`, see [app_program/subsystem2/template_hal.h](app_program/subsystem2/template_hal.h)), not SDK `hardware_*` libraries.

## Build

CMake is the only supported route (the `build_make/` GNU Make files and the `make TEST=1` instructions in `testing/README.md` are legacy/unverified; `TEST=ON` is rejected by CMake). Toolchain: Pico SDK 2.3.0, Arm GCC 15_2_Rel1, board `pico_w`.

- VS Code: Ctrl+Shift+B ("Compile Project") configures and builds into `build/` → `build/mtkernel_3.uf2`.
- Terminal (production image):

  ```sh
  cmake -S . -B build-production -G Ninja -DSMP=OFF -DWIFI=none -DNET=none -DCONSOLE=usb_cdc \
        -DROBO_MANUAL_CONSOLE=OFF -DROBO_SAFE_MOTOR_TEST=OFF -DROBO_ANGLE_TEST=OFF -DROBO_ENCODER_DIAGNOSTICS_ONLY=OFF
  cmake --build build-production -j 8
  ```

  If the toolchain isn't found, pass `-DPICO_SDK_PATH`, `-DPICO_TOOLCHAIN_PATH`, `-DPython3_EXECUTABLE`, `-Dpicotool_DIR` pointing into `~/.pico-sdk/`.
- Use a separate build directory per firmware profile (`build-<profile>`); cached options otherwise leak between profiles.
- Don't edit the `# == DO NOT EDIT ... ==` block in CMakeLists.txt; the Raspberry Pi Pico VS Code extension rewrites it. To change SDK or toolchain versions, use the extension.

### Firmware profiles (mutually exclusive CMake options, validated in [CMakeLists.txt](CMakeLists.txt))

| Option | Purpose | Extra UF2 |
|---|---|---|
| (none) | Production: `ROBO_PRODUCTION_BUILD` defined, motion enabled, output capped at 25% | `buddy2_production.uf2` |
| `ROBO_LINE_STEERING=ON` | Production line following via PID steering | `buddy2_line_steering.uf2` |
| `ROBO_IR_MONITOR=ON` (+ line steering) | Prints IR readings only, never moves | — |
| `ROBO_MANUAL_CONSOLE=ON` | Armed serial calibration console | — |
| `ROBO_SAFE_MOTOR_TEST=ON` (+ console) | 1 s, 25% motor tests ([docs/SAFE_MOTOR_TEST.md](docs/SAFE_MOTOR_TEST.md)) | `buddy2_safe_motor_test.uf2` |
| `ROBO_ANGLE_TEST=ON` (+ console) | Encoder 90° turns ([docs/ANGLE_TEST.md](docs/ANGLE_TEST.md)) | `buddy2_angle_test.uf2` |
| `ROBO_ENCODER_DIAGNOSTICS_ONLY=ON` (+ console) | Read-only encoders, motors compiled out | `buddy2_encoder_diag.uf2` |

Barcode hand-scan test is not a CMake option: build the plain production profile with `-DCMAKE_C_FLAGS=-DROBO_BARCODE_TEST=1` (e.g. in `build-barcode-test/`). It only activates `barcode_task` (ADC on GP28, Code 39 decode) when production is on and line steering is off; every other image keeps the barcode task idle.

Subsystem/behaviour switches (independent of the profile):

- `ROBO_SUBSYSTEM5` (default **ON**): obstacle detection + servo scan. OFF excludes every `app_program/subsystem5/*.c` from the build and creates no obstacle task.
- `ROBO_SUBSYSTEM4` (default **OFF**): IMU/terrain task and I2C1 startup. OFF excludes every `app_program/subsystem4/*.c`.
- `ROBO_MOTION_FAULTS_DISABLED` (default **ON** for firmware): the motion controller never enters the latched `FAULT` state; causes are still recorded in `fault_flags`. Expired commands still stop the motors and then return to READY; the remote e-stop still stops and holds. The fault code is kept, and host `motion_tests` build it with faults on, so both paths must stay correct.
- Disabled subsystems append `-no-s4` / `-no-s5` to the build ID, and the production banner prints `subsystem4_enabled` / `subsystem5_enabled`.

Other options: `SMP` must stay OFF (encoder snapshot/IRQ ownership is single-core). `WIFI=cyw43`, `NET=lwip`, `MQTT=ON` enable Buddy 1 networking; `NET=lwip` requires a local untracked `config/wifi_secrets.h` (copy `wifi_secrets.example.h`). `CONSOLE=uart` uses GP0/GP1 and forces `CRASH_REPORT`.

Each build writes a build ID (`firmware-build-id.txt`) that the USB banner prints at startup.

## Host tests

Portable logic is unit-tested natively: motion and controller (with faults both on and disabled), steering, line sensor/PID, motor/angle test state machines, the motor driver diagnostics, Buddy 1 telemetry frames, obstacle scan/detour geometry, and the `libtm` printf formatter. Test names are the `add_test` entries in the `ROBO_HOST_TESTS` block of CMakeLists.txt.

```sh
cmake -S . -B build-host-cmake -G Ninja -DROBO_HOST_TESTS=ON
cmake --build build-host-cmake
ctest --test-dir build-host-cmake --output-on-failure
ctest --test-dir build-host-cmake -R steering_checks --output-on-failure   # single test
```

`ROBO_HOST_TESTS=ON` builds only the host test project (no Pico SDK), so it needs its own build directory and a native C compiler. Host tests compile with `-Wall -Wextra -Werror -Wconversion -Wshadow` (non-MSVC); firmware code shared with them must stay warning-clean under those flags. Hardware is faked via `ROBO_*_HOST` defines and fake HAL headers in [tests/](tests/) (e.g. `motor_test_fake_hal.h`, `line_test_fake_hal.h`). Additional standalone suites: [tests/buddy2_compact/CMakeLists.txt](tests/buddy2_compact/CMakeLists.txt) (its `test_driver.c` also runs in the main host project as `motor_driver_diagnostics_checks`) and `python tests/forward_console/run_tests.py` (uses clang). Test records/logs for hardware tests live under [testing/](testing/) with IDs like `S2-U01`, `INT-01`, `SYS-01` that are never reused.

## Architecture

**Source globbing:** CMake globs `app_program/*.c` and `app_program/*/*.c` — active app sources must live at the root or exactly one subdirectory deep. Legacy demos (`task_producer|consumer|monitor.c`, `subsystem[2-5]_test.c`, `subsystem2/subsystem2.c`) are excluded by regex in CMakeLists.txt.

**Ownership** ([TEAM.md](TEAM.md)): subsystem1 = WiFi/MQTT (Buddy 1), subsystem2 = motion/motors/encoders/PID/odometry (Buddy 2), subsystem3 = line + barcode, subsystem4 = IMU heading/terrain (GY-511 on I2C1 GP6/GP7), subsystem5 = obstacle avoidance. `app_program/controller/` is the Vehicle Controller. Subsystem 3 is `line_following.c` (`line_task`) + `barcode.c` (`barcode_task`); there is no `subsystem3.c` any more, so a stray local copy causes duplicate-symbol link errors through the source glob. Subsystem 5 is split into `ultrasonic.c` (GP17 echo timed by PWM slice 0), `servo.c`, `obstacle_scan.c` and `detour_plan.c` (portable, host-tested), and `detour.c`/`subsystem5.c` (task + motion). Its behaviour and tuning are documented in [app_program/subsystem5/README.md](app_program/subsystem5/README.md).

**Telemetry:** Buddy 1 publishes a binary status frame over MQTT (`robot/frame`, `robot/event`, `robot/status`; commands on `robot/cmd`, acks on `robot/cmd/ack`). The field layout is the `TF_FIELDS` X-macro in [include/telemetry_frame.h](include/telemetry_frame.h). [tools/dashboard.py](tools/dashboard.py) parses that header at runtime, so changing a field only means editing the X-macro (`python tools/dashboard.py selftest`; `listen`/`send` need `paho-mqtt`). `subsystem1/telemetry_map.c` maps robot snapshots into frame values.

**Request flow:** sensor subsystems never touch motor GPIO. They call `robot_submit(...)` / `motion_*` from [include/robot.h](include/robot.h) (non-blocking; `R_OK` means queued, not approved or completed). The Vehicle Controller arbitrates sources into one approved `motion_command_t`; the Motion Control task (subsystem2 `motion.c` + HAL `drv_motion.c`) executes it. Stop/fault/shutdown are separate control latches that bypass the request queue and invalidate stale commands. Continuous commands (e.g. `WHEEL_SPEED_TRACKING`) must be refreshed with the same source+ID before expiry (≈300 ms). See [docs/API.md](docs/API.md) and [docs/PRODUCTION_INTEGRATION.md](docs/PRODUCTION_INTEGRATION.md).

**Shared state:** the `robot_bus_t` is protected by a T-Kernel `TA_INHERIT` mutex created in `usermain`; bus operations assume the caller holds it, copy payloads by value, and heavy computation happens outside the lock.

**Tasks / priorities** (lower number = higher priority): motion 2; controller and obstacle 4; line, barcode, terrain 8; WiFi 12; Buddy 1 status task 20. Motion period 10 ms on a 1 ms kernel tick. Optional tasks are only compiled and created when their image uses them: obstacle/terrain via `ROBO_SUBSYSTEM5/4`, line via `ROBO_LINE_TASK` (safe motor test or line steering), barcode via `ROBO_BARCODE_TASK` (barcode test). The last two are derived in [include/subsystems.h](include/subsystems.h); keep the `usermain` task table in sync with them. `motion_hal.h` is the boundary between portable motion logic and RP2040 hardware.

**GPIO IRQ 13** has a single shared dispatcher in [app_program/subsystem2/gpio_dispatch.c](app_program/subsystem2/gpio_dispatch.c). Register non-overlapping pin masks via `board_gpio_irq_register` during core-0 init; never call `tk_def_int(13)` directly.

**Pins:** motors 8/9 and 10/11; encoders left A/B 2/3, right A/B 4/5; I2C1 SDA/SCL 6/7 (subsystem 4, off by default); servo 15 (PWM slice 7B; diagnostic `S5_DIAG=6` uses GP14); ultrasonic trig/echo 16/17; line right/centre 26/27; barcode 28 (ADC2, only active in the barcode test image); calibration buttons 20/21 (optional). GP23/24/25/29 belong to the Pico W radio; GP0/GP1 are reserved when the UART console is on. Encoder pins live in `include/board_config.h`; I2C1 pins in `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h`. `board_pins_valid()` and `motor_test_init()` also hardcode the encoder pins, so a remap must update those and the host tests.

**Calibration constants** live in [app_program/subsystem2/board_config.c](app_program/subsystem2/board_config.c) (wheel diameter 65 mm, track 115 mm, ~2542/2543 counts/rev). `board_motion_config()` reports `hardware_confirmed=true` only in production; calibration profiles force it false.

**Crash handling:** after a hard fault or watchdog reset, `usermain` loops printing `LAST CRASH: ...` with motors held until power-cycled.

## Safety conventions

- Startup always leaves motors stopped; manual PWM is disabled in production; output cap is 25%. Latched motion faults are currently disabled in firmware (see `ROBO_MOTION_FAULTS_DISABLED`); don't remove the remaining stops (expiry, e-stop) on that basis. Don't relax these limits or the CMake profile guards without explicit user direction.
- Flash with motor power disconnected (hold BOOTSEL, copy UF2 to `RPI-RP2`).
- Kernel/driver code under `kernel/`, `lib/`, `device/` is imported template code with its own licenses; `docs/template_original/` and `docs/previous_integration/` are archived references, not built.
