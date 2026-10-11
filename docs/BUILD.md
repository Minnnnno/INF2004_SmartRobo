# Build the integrated template

## Raspberry Pi Pico VS Code project

Open INF2004_SmartRobo.code-workspace, or open this repository folder in VS Code.
The Raspberry Pi Pico extension recognises the existing CMake Pico header and the
checked-in .vscode configuration. Reload the window if it was open before import.
The selected board is Pico W (RP2040), SDK 2.3.0 and toolchain 15_2_Rel1.

Use the Pico extension's Compile button or Ctrl+Shift+B (Compile Project). Both
use the standard build directory. The output is build/mtkernel_3.uf2 and the
compile database is build/compile_commands.json. The task configures before building.
The optional debugger configuration requires an SWD debug probe; it stops at usermain.

The explicit build-template commands below remain available for terminal builds.

Use CMake for this integration. The supplied CMakeLists.txt is the basis of the
root build, including the Pico VS Code SDK/toolchain selection and kernel-owned
linker/startup. SDK 2.3.0 and Arm GCC 15.2.Rel1 were used for verification.
Do not reuse the earlier build-pico or build-manual CMake caches.

## Firmware

In a Pico-configured terminal:

~~~sh
cmake -S . -B build-template -G Ninja -DSMP=OFF -DWIFI=none -DNET=none
cmake --build build-template
~~~

If your toolchain is not automatically found, pass:
- PICO_SDK_PATH=C:/Users/mervy/.pico-sdk/sdk/2.3.0
- PICO_TOOLCHAIN_PATH=C:/Users/mervy/.pico-sdk/toolchain/15_2_Rel1
- Python3_EXECUTABLE=C:/Users/mervy/.pico-sdk/python/3.13.7/python.exe
- picotool_DIR=C:/Users/mervy/.pico-sdk/picotool/2.3.0/picotool

Each is a CMake argument prefixed with -D. Use your own paths on another computer.
The output is **build-template/mtkernel_3.uf2** (also ELF, BIN and HEX).

For the deliberately armed calibration console use build-template-manual and add
-DROBO_MANUAL_CONSOLE=ON. USB connection alone never starts a motor. CONSOLE=usb_cdc
is the default. CONSOLE=uart uses GP0/GP1 instead.

Keep motor power disconnected while flashing. Hold BOOTSEL while connecting USB,
then copy the chosen UF2 to RPI-RP2. Complete CALIBRATION.md before enabling motion.

## Host tests

With a native C compiler:

~~~sh
cmake -S . -B build-host-cmake -G Ninja -DROBO_HOST_TESTS=ON
cmake --build build-host-cmake
ctest --test-dir build-host-cmake --output-on-failure
~~~

Direct Zig/GCC/Clang compilation is also supported; create build-host first:

~~~sh
zig cc -std=c11 -O2 -Wall -Wextra -Werror -Wconversion -Wshadow -fsanitize=undefined -Iinclude app_program/subsystem2/math.c app_program/subsystem2/motion.c app_program/subsystem2/board_config.c app_program/controller/vehicle_controller.c tests/test_motion.c -o build-host/motion_tests.exe
~~~

Run build-host/motion_tests.exe. Synthetic values are test fixtures, not measured
robot calibration.

## Options and boundaries

SMP=ON is rejected: the current encoder snapshot and IRQ ownership are single-core.
TEST=ON is rejected: imported startup demo tests must not drive this application.
Use host tests or deliberate manual mode instead. The imported GNU Make files are
retained for reference but were not verified for the merged application; CMake is
the supported build route.

WiFi/network source is imported, but WIFI=none / NET=none / MQTT=OFF is the safe
default. Optional networking requires Buddy 1 integration and real credentials;
config/wifi_secrets.h is ignored. Do not put credentials in tracked files.

All active application .c files live at app_program root or one subdirectory,
matching the supplied CMake glob. Kernel and driver licenses remain intact.
