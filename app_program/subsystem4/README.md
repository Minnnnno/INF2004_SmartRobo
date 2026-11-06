# Subsystem 4: GY-511 IMU, tilt and terrain monitoring

**Integration stage 1.** Ported from the standalone Buddy 4 project: sensor
configuration, fresh acceleration reads, the INIT/MONITORING/RECOVERY/FAULT/STOP
lifecycle, level-reference tilt, hump detection and **hump peak height**.
**Magnetic heading and its calibration are not integrated yet**; the heading
slot stays invalid. Collision detection, motion classification and turn rate
are not implemented yet.

Buddy 4 is disabled by default (`ROBO_SUBSYSTEM4=OFF`): its sources are not
built, no terrain task is created and I2C1 is not initialised. Enable it with
`-DROBO_SUBSYSTEM4=ON` in a separate build directory. The startup banner
reports `subsystem4_enabled=1` and the build ID loses its `-no-s4` suffix.

## Files

| File | Role |
|---|---|
| `terrain_task.c` | The RTOS task: 20 ms loop, input facts, bus snapshot, console, hump event |
| `buddy4.c/.h` | Lifecycle state machine and bounded automatic sensor recovery |
| `subsystem4.c/.h` | GY-511 identity check and register configuration with readback |
| `gy511_read.c/.h` | Fresh acceleration (and magnetic, unused for now) packed 12-bit reads |
| `tilt.c/.h` | Level reference capture, filtered pitch/roll, 27-bit packed result, wheel height difference |
| `terrain.c/.h` | Height-based hump phases, peak and run maximum, slow request, completion latch |

`subsystem4_test.c` and `task_monitor.c` are template demos excluded by CMake.

## Wiring and mounting

| GY-511 | Pico W | Physical pin |
|---|---|---|
| SDA | GP6 (I2C1 SDA) | 9 |
| SCL | GP7 (I2C1 SCL) | 10 |
| GND | GND | 8 |
| VCC | 3V3(OUT) | 36 |

GP2..GP5 carry the wheel encoders, so the IMU must not use the Robo Pico's
GP2/GP3 Grove port. I2C0 stays disabled (its default pins are motor pins).
Forward is the sensor's +Y axis; positive pitch = front up.

## Startup contract

**Power up with the robot level and still.** After configuration, INIT waits
1 s (`B4_POSITION_MS`), then needs 50 stable fresh samples within 10 s,
otherwise SENSOR_FAULT. Capture normally finishes about 2 s after boot.
In `ROBO_LINE_STEERING` builds the line task does not send its first motion
request until Buddy 4 has left INIT/RECOVERY (`line_imu_wait` console line),
capped at 10 s so a missing or faulty IMU never stops the robot.
Level permission comes from Buddy 2's motion status: it is withheld only when
motion reports the wheels may be moving (not stationary and not initialising).
Software cannot tell a level floor from a stationary slope. The reference is
RAM-only and kept across automatic recovery.

I2C errors or 500 ms without fresh data start automatic recovery: up to three
reconfigure + fresh-read attempts, 3 s apart. Exhausted retries or a capture
timeout latch SENSOR_FAULT. No STOP/RESET source is wired yet, so a latched
fault needs a reboot. Sensor faults never stop the motors.

## Hump height (one-wheel humps)

Course model: 2 driven wheels + centre caster, each hump under **one** wheel
side. When one wheel climbs a hump of height h, the axle tilts sideways by
asin(h / track). The body also pitches, but only by about half as much (the
axle centre rises h/2 against the caster) and it never tips nose-down coming
off, so pitch is neither a usable detector nor a height measure here.

Height = `B4_TRACK_MM` x (sideways component of the smoothed gravity unit
vector), from `tilt_side_height_tenths()`. This is exactly how far the raised
wheel's contact sits above the other one: body pitch, the caster and forward
acceleration/braking do not change it, no encoder distance is needed, and the
sign gives the side (positive = right wheel). It is *not* the displayed roll
angle, which comes from atan2(side, up) and drifts when the body pitches.

Detector (`terrain.c`, thresholds are `#define`s), on the height smoothed by
s += (h - s)/4 per 20 ms sample: |h| >= 5 mm for 100 ms -> CLIMBING (peak
tracked, slow requested); 3 mm below the peak, or half the peak if smaller ->
DESCENDING (a rise of 2 mm+ above the peak returns to CLIMBING; smaller rises
are noise); below 2.5 mm for 300 ms -> hump complete,
peak and side stored, `TF_EV_HUMP_PEAK` latched. The run maximum follows the
live peak, so a ramp that never comes down still counts. Peaks are stored as
7-bit millimetres (0..127) plus one flag bit each (CR1).

Expected error: about +/-0.5 mm noise standing still; while driving, steering
sway and vibration swing the reading by up to about 3.3 mm (measured on the
robot), which is why entry is 5 mm; about 2 mm per degree of sideways floor slope at boot,
and some underestimate on humps whose top lasts under ~0.2 s (filter lag).
**Measure `B4_TRACK_MM`** between the two wheels' ground-contact centres (1 mm
error = 0.9% height error). A hump under both wheels at once cancels out.

## Interfaces

Task: `terrain_task`, priority 8, 4096-byte stack, 20 ms period. I2C transfers
are synchronous; the lifecycle re-reads the clock after each transfer.

Shared Terrain slot, published every cycle via `robot_update_subsystem`
(see docs/API.md and `subsystem1/telemetry_map.c`):

| Slot | Content |
|---|---|
| `available` | true once the task runs (the state itself is genuine data) |
| `timestamp` | time of the last valid tilt sample, seconds |
| `values[0]` | heading rad: 0, invalid (heading not integrated) |
| `values[1]` | pitch rad, + front up, when `TF_TV_PITCH` is set |
| `values[2]` | highest hump peak this run, m (1 mm steps), when `TF_TV_HUMP` is set |
| `values[3]` | validity bits (`TF_TV_PITCH`, `TF_TV_HUMP` once a hump is confirmed) \| `T_STATE` << 3 |

`T_STATE` (3 bits, 7 codes): 0 INIT, 1 WAITING, 2 CLIMBING, 3 DESCENDING
(1..3 are MONITORING with that terrain phase), 4 RECOVERY, 5 FAULT, 6 STOP.

Hump completion latches `TF_EV_HUMP_PEAK` through `comms_latch_event`, which
is interrupt-safe. The dashboard's `HUMP_MM` shows the run maximum.
The terrain `SLOW` request is printed but not yet consumed by the Controller.
Nothing is sent as MQTT text any more; data reaches the dashboard only through
the binary status frame (Change Requirement 1).

## Console

`[B4-STATE]` and `[TERRAIN-STATE]` transitions print once, and each hump prints
`[TERRAIN] Hump completed: peak=32 mm under RIGHT wheel | run max=41 mm`.
Every 500 ms: `[B4] INIT ...` capture progress, `[TILT] pitch=.. roll=.. |
conditions | height=..mm R/L | terrain=.. slow=.. run_max=..mm`, or the fault
state. Suppressed in `ROBO_IR_MONITOR` builds.

## Checks

```sh
python testing/subsystem4/host/run_tests.py
```

Needs native gcc on PATH. Covers configuration/readback, every 12-bit axis
value, tilt geometry and pitch-independent wheel height, height-based hump
phases/peaks, lifecycle/recovery/STOP, the real
`terrain_task` loop (snapshot encoding, T_STATE codes, hump event, level
gating, 20 ms release grid) and the I2C driver's multi-byte receive STOP.
These are host results; physical accuracy and timing need robot tests.
