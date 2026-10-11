# Building and flashing

The initial repository contained only a title in README.md. No AGENTS.md, source,
RTOS, SDK version, board selection, or Barr C rule document was supplied. The
project briefing describes subsystem deliverables but does not name an RTOS.
The user's explicit micro T-Kernel requirement is therefore used; no FreeRTOS
integration was replaced. Application C uses fixed-width counters, explicit
ownership, braces, header guards, static internal functions and no allocation in
control loops. This is not a claim of audited compliance with an unspecified Barr edition.

## Dependencies

* CMake >= 3.20, Ninja or another generator.
* Host C11 compiler (GCC, Clang, or Zig's `cc`). No SDK/RTOS needed for host tests.
* Target: Arm GNU toolchain, Pico C SDK 2.3.1, board `pico_w`, RP2040.
* Real TRON Forum BSP `pico_rp2040`, commit
  `15ed232c08f2d89e54e514db79de150e224307e3`.

Fetch dependencies explicitly; downloaded code is ignored, not vendored or staged:

```sh
git clone --branch pico_rp2040 https://github.com/tron-forum/mtk3_bsp.git third_party/mtk3_bsp
git -C third_party/mtk3_bsp checkout 15ed232c08f2d89e54e514db79de150e224307e3
```

Alternatively pass `-DMTKERNEL_PATH=/path/to/that/checkout`. The upstream kernel
files retain their T-License notices. No upstream file is modified by this project.
CMake generates a private configuration disabling sample drivers/T-Monitor and
using a 1 ms kernel tick. Kernel object allocation happens at startup, from a
dedicated 96 KiB pool; application loops use fixed storage.

## Host

```sh
cmake -S . -B build-host-cmake -G Ninja
cmake --build build-host-cmake
ctest --test-dir build-host-cmake --output-on-failure
```

For Zig without a compiler wrapper, direct compilation is also supported:

```sh
zig cc -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -Iinclude \
  src/control/math.c src/control/motion.c src/config/board_config.c \
  src/app/vehicle_controller.c tests/test_motion.c -o build-host/motion_tests
./build-host/motion_tests
```

Create `build-host` first; on Windows the output is `motion_tests.exe`. The synthetic
plant lives in `tests/test_motion.c`; dimensions and gains there are test fixtures,
not the robot's measured calibration. Tests retain their checks in Release builds.

## Pico W

```sh
cmake -S . -B build-pico -G Ninja -DROBO_TARGET=ON \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -DPICO_TOOLCHAIN_PATH=/path/to/arm-toolchain
cmake --build build-pico
```

On this Windows workstation the SDK is `C:/Users/mervy/.pico-sdk/sdk/2.3.1`, the
toolchain is `C:/Users/mervy/.pico-sdk/toolchain/15_2_Rel1`, and the optional
`-Dpicotool_DIR=C:/Users/mervy/.pico-sdk/picotool/2.3.1/picotool` prevents an
unnecessary SDK picotool download. Set
`-DPython3_EXECUTABLE=C:/Users/mervy/.pico-sdk/python/3.13.7/python.exe` if Python
is absent from PATH. PowerShell uses backticks, not backslashes, for line continuation;
put each command on one line when copying the examples there.

Outputs are `build-pico/smart_robo.elf`, `.bin`, `.hex` and `.uf2`.
With motor power disconnected, hold BOOTSEL while connecting USB and copy the UF2
to RPI-RP2. No flash or hardware test is performed by these build commands.

For the deliberate bring-up console use a separate directory and add
`-DROBO_MANUAL_CONSOLE=ON`. It occupies the otherwise inactive WiFi task slot,
enables USB CDC, and never activates motion merely on connection. See CALIBRATION.md.
Normal firmware has no automatic demo, network connection, or serial command sequence.

## RTOS / SDK integration boundaries

SDK owns boot2, CRT, clocks, RAM vectors and GPIO IRQ registration. The startup
adapter installs the real BSP PendSV and SysTick handlers, establishes its heap,
and calls the BSP startup function (renamed only at compilation). The BSP owns
task scheduling, mutexes and delays. The clock is fixed at the BSP's 125 MHz.
Use core 0 only; this BSP is not an SMP port. Use SDK raw/shared GPIO handlers,
not `tk_def_int`, because SDK owns the active vector table. ISRs must not call
task-context APIs. The encoder interrupt only acknowledges its own GPIO events.

The BSP does not context-switch RP2040 hardware-divider state. SDK division is
therefore configured to mask interrupts during the short hardware division sequence.
SDK TLS is explicitly global: no claim of per-task newlib/errno/TLS support. Keep
stdio/network library calls in their owner task and add a proper RTOS integration
before introducing libraries requiring per-task C runtime state. Do not use SDK
sleep calls for periodic task blocking; use `tk_dly_tsk`.

The Pico SDK and RTOS integration needs on-board scheduling, stack-watermark,
interrupt-load and timing validation. A linked UF2 proves a build, not that the
board has booted or that a control deadline is met.
