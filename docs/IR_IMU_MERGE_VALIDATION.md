# Local IR and IMU merge validation

Merged origin/main through 612cc9e with local digital IR work saved in 06b8b7d.
The CMake conflict was resolved by retaining both the line-task tests and angle
tests. The merged serial-parser fixtures now include sensor pause and actuator
shutdown hooks; their RTOS include guard matches the real application headers.

Buddy 1 communications, Buddy 4 GY-511 background monitoring on I2C1 GP2/GP3,
and the verified motor signs {1,-1} are retained. ADC and serial sample drivers
are disabled; the I2C1 driver does not claim GP26/GP27 or encoder pins.
The remote production configuration is retained; isolated motor/angle profiles
still force hardware_confirmed=false and manual_enabled=false.

Digital IR motion remains in the isolated motor-test profile. Left DO is GP26,
right DO is GP27. Either input HIGH (1) requests continuous 25% forward; both
LOW stop. This retains the existing detection polarity; confirm the actual
sensor outputs over black and white on the robot. Serial stop pauses automatic
motion until both inputs are LOW. Shutdown disables both entry points until
reboot. A 300 ms missed-update watchdog stops the motors if sensing stalls.

Pico W production, motor-test/IR, and angle-test builds passed. Sanitized host
tests compiled with the installed Zig C compiler and strict warnings passed:

| Suite | Checks or scenarios |
| --- | ---: |
| Motion/encoder/controller | 2137 |
| Motor actuator, including continuous refresh/watchdog | 721 |
| Digital IR task, all input combinations and stop pause | 202 |
| Angle controller | 673 |
| Angle actuator | 184 |
| Merged serial parser | 53 |
| GY-511 configuration | 27 |
| IMU reading | 16 |
| Terrain filtering | 94 |
| Mounting axes | 72 |
| IMU background task and shared status | 130 |

Flash build-safe-motor-test/buddy2_safe_motor_test.uf2 for the current digital
IR test with the IMU running in the background. Its build manifest records the
ID/hash. Begin with motor power disconnected and check the digital levels,
then test with wheels raised: 0/0 stops; 1/0, 0/1 and 1/1 move. Send stop,
verify no immediate automatic restart, then clear both sensors before re-entry.
Confirm IMU telemetry or an explicit unavailable status if it is disconnected.

No robot was flashed or physically tested here. This merge is local and has
not been pushed; the owner will test before publishing.
