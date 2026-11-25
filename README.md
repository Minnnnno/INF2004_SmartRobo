# INF2004 SmartRobo

Integrated robot firmware for the Raspberry Pi Pico W (RP2040) on the Robo Pico
motor board, using micro T-Kernel 3.0. The application combines encoder/PID motion
control, IR line following, Code 39 barcode navigation, IMU terrain and hump
measurement, lift/overturn stops, ultrasonic obstacle avoidance, and a Wi-Fi/MQTT
web dashboard.

The CMake build is based on the
[sirfonzie project template](https://github.com/sirfonzie/INF2004_ProjectTemplate)
at commit `c3b09544cb1fd6dac6ad4b8f523be0af59b44482`. The kernel owns boot,
vectors, scheduling and the USB console. Original demos and documentation are
archived in [docs/template_original/](docs/template_original/) and excluded from
the active application build.

## Build and flash

1. Install VS Code and the Raspberry Pi Pico extension. This project selects
   **Pico W**, **Pico SDK 2.3.0** and **Arm toolchain 15_2_Rel1**.
2. Open [INF2004_SmartRobo.code-workspace](INF2004_SmartRobo.code-workspace).
3. For dashboard control, copy
   [config/wifi_secrets.example.h](config/wifi_secrets.example.h) to
   `config/wifi_secrets.h`. Set `WIFI_SSID`, `WIFI_PASSWORD` and `MQTT_BROKER_IP`
   for your network. Use a **2.4 GHz** Wi-Fi network. The broker address must be
   the laptop's address reachable by the robot; replace the example
   `192.168.1.10`. A Windows hotspot commonly uses `192.168.137.1`; check with
   `ipconfig`. The secrets file is ignored by Git; keep credentials out of
   tracked files.
4. Press **Ctrl+Shift+B** and select **Compile Project**. The checked-in task
   configures the integrated profile before building into `build/`.
5. Check the configure output before flashing:

   | Output | Resulting behavior |
   | --- | --- |
   | `ROBO_NETWORK=AUTO: WIFI=cyw43 NET=lwip MQTT=ON` | Wi-Fi/MQTT enabled; the default web START gate holds the robot after boot. |
   | Warning that `config/wifi_secrets.h` is missing | No Wi-Fi/MQTT; production line following starts automatically after startup qualification and the IMU wait. |

   For dashboard operation, also confirm `Web Start gate ON` in the output.
   **A build without MQTT does not wait for dashboard START.**
6. Disconnect motor power. Hold **BOOTSEL** while connecting USB, then copy
   **`build/mtkernel_3.uf2`** to the `RPI-RP2` drive. The Pico reboots after copying.
   `build/smartrobo_integrated.uf2` contains the same bytes for this profile.

The normal VS Code profile explicitly enables `ROBO_LINE_STEERING`,
`ROBO_SUBSYSTEM4` and `ROBO_SUBSYSTEM5`, enables motion fault latching, disables
calibration/IR-monitor profiles, and selects `ROBO_NETWORK=AUTO` and `SMP=OFF`.
Barcode acquisition runs alongside line following in this production profile.
The startup banner should contain
`line_task_enabled=1 subsystem4_enabled=1 subsystem5_enabled=1` and
`[MOTION] faults_enabled=1`; the build ID starts `smartrobo-integrated-v1-`.

For terminal builds, toolchain paths and separate build directories, see
[build instructions](docs/BUILD.md). CMake is the supported build route;
`SMP=ON` and template startup `TEST=ON` are rejected.

## Run with the dashboard

Install Mosquitto on the laptop and the Python dependency once:

```powershell
python -m pip install paho-mqtt
```

At each session, turn on the hotspot/network used in the firmware, then run the
broker from the repository root (adjust the executable path to your installation):

```powershell
& "C:\Program Files\mosquitto\mosquitto.exe" -c mosquitto.conf -v
```

Allow Mosquitto through Windows Firewall for the robot's network. The checked-in
[mosquitto.conf](mosquitto.conf) listens on IPv4 port 1883 with anonymous access;
use it on the team's local test network. In a second terminal, start the dashboard
with the same broker address configured in the firmware:

```powershell
python tools/web_dashboard.py --host 192.168.137.1
```

Open **http://localhost:8080**. Power the robot and wait for **broker connected**,
**robot online**, **telemetry live**, and Motion **STOPPED (held)**. The Link card
should show frame layout **v3**, with approximately 10 status frames per second.
Build the dashboard and firmware from the same checkout.

Place the robot level on the course before pressing **START**, and keep it still
for about **two seconds after START**. In MQTT production builds, the IMU captures
its level reference after the first START; line following waits for that capture.
The IMU wait is capped at ten seconds, so an unavailable IMU does not indefinitely
block driving. Later STOP/START cycles keep the reference.

| Control | Behavior |
| --- | --- |
| **START** | Releases an eligible stopped, stationary, fault-free robot with trusted encoder feedback. It does not clear a fault. |
| **STOP** | Requests a controlled stop and holds the robot until another dashboard START; subsystem release requests cannot override this hold. |
| **E-STOP** | Latches an emergency fault. START cannot clear it; power-cycle the robot with battery and USB disconnected. |

A command acknowledgement means **queued**, not executed; check the subsequent
motion status. Encoder faults require the checks and reset/release procedure in
[motion fault diagnostics](docs/MOTION_FAULT_DIAGNOSTICS.md).

For run evidence, add `--log logs/run.log --csv logs/frames.csv` and optionally
`--encoder-csv logs/encoders.csv` to the dashboard command. To preview the dashboard
without a robot or broker, run `python tools/web_dashboard.py --demo`.
See [dashboard setup and troubleshooting](docs/DASHBOARD_SETUP.md) for full details.

## Barcode route and sensor behavior

In the dashboard's **Line & barcode** card, fill in **Left**, **Right**,
**Straight** and **U-turn**, then select **Send route**. Supply four distinct
letters; invalid messages leave the previous route intact. The MQTT route is
retained by the broker and delivered again when the robot reconnects. Until a
route arrives, [route.h](app_program/subsystem3/route.h) defines the fallback:
**Z = left**, **A = U-turn**.

Left, right and U-turn actions happen at the junction after the barcode; straight
ignores the junction. Turns spin on the spot using both wheels. If a barcode
cannot be decoded, the robot reports the failed read and continues following the
line without acting on that read.

Buddy 4 reports pitch, terrain state, motion class, turn rate and the highest
completed hump peak in the run. Lift/overturn detection stops the motors. Hump
recording pauses during turns on the spot and briefly afterward to avoid false
humps from turn acceleration. See the
[IMU/terrain guide](app_program/subsystem4/README.md) for mounting, calibration
and validity rules.

Buddy 5 stops for an obstacle, scans its width, chooses a detour and returns
control to line following when the line is reacquired. While avoidance owns
motion, line following continues publishing sensor status without replacing its
commands. See the [obstacle guide](app_program/subsystem5/README.md).

## Source layout and wiring

| Owner | Active source / responsibility |
| --- | --- |
| Application startup | [app_program/app_main.c](app_program/app_main.c): initialization, task creation and supervision |
| Vehicle Controller | [app_program/controller/](app_program/controller/): command bus, ownership, submission APIs and remote controls |
| Buddy 1 | [app_program/subsystem1/](app_program/subsystem1/): Wi-Fi/MQTT, queues, telemetry and command handling |
| Buddy 2 | [app_program/subsystem2/](app_program/subsystem2/): motor driver, encoders, odometry, PID and motion runtime |
| Buddy 3 | [app_program/subsystem3/](app_program/subsystem3/): IR steering, barcode decoding and routes |
| Buddy 4 | [app_program/subsystem4/](app_program/subsystem4/): IMU, terrain, hump measurement and lift/overturn stops |
| Buddy 5 | [app_program/subsystem5/](app_program/subsystem5/): ultrasonic scanning and obstacle avoidance |
| System | [app_program/system/](app_program/system/): watchdog, retained crash reports and supervision |

Shared requests and status are defined in [include/robot.h](include/robot.h) and
[include/motion.h](include/motion.h). Subsystems submit requests through the
Vehicle Controller; motion owns the motor hardware. Stop/fault/shutdown controls
bypass ordinary requests and invalidate stale commands. The binary MQTT formats
are defined in [include/telemetry_frame.h](include/telemetry_frame.h): a 21-byte
status frame (layout v3) and a separate 26-byte encoder frame, published at 10 Hz.

| Signal | Pico GPIO |
| --- | --- |
| Left / right motors | GP8/GP9 / GP10/GP11 |
| Left / right encoder A/B | GP2/GP3 / GP4/GP5 |
| GY-511 SDA / SCL | GP6 / GP7, I2C1 |
| Scanning servo | GP15 |
| Ultrasonic TRIG / ECHO | GP16 / GP17 |
| Right / centre IR | GP26 / GP27 |
| Barcode ADC | GP28 |
| Barcode calibration button (optional, active low) | GP21 |
| Optional UART0 console TX / RX | GP0 / GP1 |

GP23/GP24/GP25/GP29 are reserved for Pico W board/radio use. I2C0 stays disabled
to avoid the motor pins. Wiring details and electrical assumptions are in
[hardware documentation](docs/HARDWARE.md).

Current [motion configuration](app_program/subsystem2/board_config.c) uses measured
encoder counts/revolution of **2542.4 left / 2543.4 right**, provisional effective
wheel diameters of **67.5 / 67.9 mm**, and **115 mm** track width. Normal production
motion is enabled with up to 100% PWM; manual PWM is disabled. Validate provisional
calibration under load using the [calibration guide](docs/CALIBRATION.md).

## Host checks and hardware evidence

With CMake, Ninja and a native C compiler available, run from the repository root:

```powershell
cmake -S . -B build-host-cmake -G Ninja -DROBO_HOST_TESTS=ON
cmake --build build-host-cmake
ctest --test-dir build-host-cmake --output-on-failure
python tools/dashboard.py selftest
```

Buddy 4 has additional host suites:

```powershell
python tests/subsystem4/run_tests.py
```

See [Buddy 4 host verification](tests/subsystem4/README.md) for compiler selection.
On Windows with MSYS2, MinGW GCC (seen with 15 and 16) reports false
float-conversion errors under the host tests' `-Werror` flags; configure with
`-DCMAKE_C_COMPILER=clang` instead. Buddy 3's earlier workaround is in the
[Buddy 3 script-test instructions](testing/subsystem3/TEST_LOG.md#script-tests-laptop-no-robot).
Host checks exercise software with simulated inputs; physical accuracy, timing
and complete mission performance require robot tests. The
[testing guide](testing/README.md) explains how tests are run and recorded.

Record test IDs, tested commit, results, limitations and saved logs in the relevant
[subsystem test log](testing/). Cross-subsystem checks belong in the
[integration log](testing/integration/TEST_LOG.md), and whole-course checks in the
[system log](testing/system/TEST_LOG.md). Share hardware videos as unlisted YouTube
links and register them in the [video index](videos/README.md).

The latest Buddy 2 hardware entries record single-trial functional passes for
startup/START, independent encoders, wheel-speed control and STOP/hold/restart;
they do not establish numerical speed accuracy, stop latency or repeatability.
See the [Buddy 2 test log](testing/subsystem2/TEST_LOG.md) for those limits.
The [Buddy 3 log](testing/subsystem3/TEST_LOG.md) records that the changed
failed-barcode-read behavior still needs a fresh hardware run.

## Further instructions

- [Coding standard (BARR-C:2018) and its tools](docs/CODING_STANDARD.md)
- [Build options and toolchain](docs/BUILD.md), [integrated firmware](docs/INTEGRATED_FIRMWARE.md)
  and [production motion integration](docs/PRODUCTION_INTEGRATION.md)
- [Shared API](docs/API.md), [motion API for buddies](docs/BUDDY_MOTION_API.md)
  and [team ownership](TEAM.md)
- Separate calibration profiles: [safe motor test](docs/SAFE_MOTOR_TEST.md),
  [encoder-based angle test](docs/ANGLE_TEST.md) and [steering](docs/STEERING.md)

Keep calibration images in separate build directories. `ROBO_NETWORK=AUTO`
disables networking for calibration/test profiles; `ON` requires credentials and
forces networking, while `OFF` disables it. Terminal builds that leave
`ROBO_NETWORK` empty use the explicit `WIFI`/`NET`/`MQTT` options (fresh defaults:
`none`/`none`/`OFF`). MQTT production builds use `ROBO_REQUIRE_START=ON` by default;
setting it `OFF` removes the web startup hold.
