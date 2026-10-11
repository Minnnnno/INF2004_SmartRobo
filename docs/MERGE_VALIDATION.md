# Buddy 1 / Buddy 2 merged production validation

Merged Buddy 1 main commits through 60abb4c with Buddy 2 production commit
264abb3. Startup retains the real robot motion API, Vehicle Controller and
sensor task entry points, and initializes/starts Buddy 1's communications owner.
The controller drains the command queue without blocking for messages and maps
start/stop/estop to the existing release/stop/fault controls. Release requires
trusted stationary STOPPED motion with no pending control, fault or shutdown.

Retained motion protections: core-0 encoder synchronization (SMP=OFF), 1 ms
kernel tick, the microsecond hardware timer, 4 KiB initial-task stack, motor
stop hooks on startup/shutdown/hard fault, owner-controlled GPIO configuration,
and USB RX for isolated calibration consoles. Sample peripheral drivers remain
disabled so they cannot claim the line/encoder GPIOs. VS Code SDK paths match
the project's tested SDK 2.3.0; standard build tasks explicitly disable all
calibration profiles.

Legacy mailbox producer/consumer/monitor examples are archived and excluded
from the active CMake target. Buddy 1's comms implementation, MQTT hooks, tests,
broker configuration and network source changes are preserved.

Successful Pico W builds:

- Production: `build-production/buddy2_production.uf2`, WIFI=none/NET=none.
- Angle calibration: `build-angle-test/buddy2_angle_test.uf2`.
- WiFi radio compile check: WIFI=cyw43/NET=none.
- Full network compile check: WIFI=cyw43/NET=lwip/MQTT=ON, using the example
  placeholder secrets header temporarily. It was removed afterward. This image
  is for compile verification only, not for flashing with real credentials.

Sanitized host checks: 2137 motion/encoder/controller checks, including production
line-request acceptance and remote start/stop/estop arbitration; 673 angle control,
184 angle actuator, and 358 motor-test checks. The existing calibration test
executables were rerun; the angle target was rebuilt against merged startup.

No physical motion, line sensing, WiFi connection or MQTT broker test was
performed remotely. Your teammate must implement real line sensing and merge
that source into this same project, then rebuild the production image.
