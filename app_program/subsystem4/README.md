# Subsystem 4: GY-511 IMU, tilt and terrain monitoring

Sensor configuration, fresh acceleration reads, the INIT/MONITORING/RECOVERY/
FAULT/STOP lifecycle, level-reference tilt, hump detection and **hump peak
height**, plus **motion classification and turn rate**, a calibrated **compass
whose angle is shown next to each turn**, and the **lift / overturn stop**:
when the robot is lifted or overturned, Buddy 4 stops the motors.

> **Change Requirement (lecturer, 11 November 2026):** IMU collision detection
> is removed from the brief (it conflicted with obstacle avoidance). Instead the
> IMU detects when the robot has been **lifted or overturned** and the motors
> stop immediately. In the demonstration, lifting the robot ends the run. The
> old impact detector (LSM303 INT1 latch, motion class 7 IMPACT, `[IMPACT]`
> lines) has been removed; see "Lift and overturn stop" below.

Buddy 4 is enabled by default (`ROBO_SUBSYSTEM4=ON`). An existing build
directory keeps its cached value, so reconfigure it with `-DROBO_SUBSYSTEM4=ON`.
With `-DROBO_SUBSYSTEM4=OFF` its sources are not built, no terrain task is
created and I2C1 is not initialised. When enabled, the startup banner reports
`subsystem4_enabled=1` and the build ID has no `-no-s4` suffix.

## Files

| File | Role |
|---|---|
| `terrain_task.c` | The RTOS task: 20 ms loop, input facts, bus snapshot, console, hump event |
| `buddy4.c/.h` | Lifecycle state machine and bounded automatic sensor recovery |
| `subsystem4.c/.h` | GY-511 identity check and register configuration with readback |
| `gy511_read.c/.h` | Fresh acceleration and magnetic packed 12-bit reads |
| `tilt.c/.h` | Level reference capture, filtered pitch/roll, 27-bit packed result, wheel height difference |
| `terrain.c/.h` | Height-based hump phases, peak and run maximum, completion latch |
| `lift.c/.h` | Lift / overturn detection from the raw sample against the level reference, latch and clear |
| `motion_class.c/.h` | Motion class (8 codes, 3 bits), debounce, overturned, yaw rate, turn angle, compass angle per turn |
| `mag_heading.c/.h` | Compass: 2D calibration from a flat spin, calibrated heading, field check, unwrap |
| `mag_cal.h` | Calibration constants pasted from the calibration build (uncalibrated by default) |

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
timeout latch SENSOR_FAULT (T_STATE 5, `terrain_imu_faulted()`). Sensor faults
never stop the motors (lift detection is then unavailable). Buddy 4 never
changes the robot's speed; its only motion command is `motion_stop()` when the
robot is lifted or overturned.

**A new run is a power-up.** The statechart's NEW_RUN (clear the run maximum)
and the reset of a latched SENSOR_FAULT both happen by rebooting, by design:
every run already needs a level, still boot for the tilt reference. When Buddy 2
reports motion `SHUTDOWN`, Buddy 4 enters STOP and keeps its completed peaks.

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
tracked); 3 mm below the peak, or half the peak if smaller ->
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

## Motion classification and turn rate

Fusion by design. The IMU supplies what only it can sense; Buddy 2's encoder
status supplies movement:

| Code | Class | Source and rule (first match wins, top to bottom) |
|---|---|---|
| 7 | OVERTURNED | IMU lift detector latched OVERTURNED (see below); reported at once, held while latched |
| 5 | CLIMBING | IMU sideways tilt (hump detector above) |
| 6 | DESCENDING | IMU sideways tilt |
| 0 | STATIONARY | Buddy 2 `stationary` (encoders) |
| 4 | TURNING | heading really changed: \|net change over the last ~0.5 s\| >= 15 deg (Buddy 2 odometry heading) |
| 3 | REVERSING | centre wheel speed <= -20 mm/s (quadrature encoders measure direction) |
| 2 | ACCELERATING | centre speed **size** changing >= 150 mm/s^2 over 100 ms (speeding up or braking) |
| 1 | MOVING | otherwise |

A new class must repeat for 3 cycles (60 ms) before it is reported, so it
does not flicker; OVERTURNED is reported at once. Thresholds are `#define`s in
`motion_class.h`.

There is no separate BRAKING class: the 3-bit field holds 8 codes, braking is
physically negative acceleration and was not one of the brief's example events,
so its code went to REVERSING. The console's `accel=` keeps the difference:
**+ speeding up, - slowing down**, in either direction. Starting or stopping a
reverse reports REVERSING (the more useful fact), a reverse with sharp steering
reports TURNING, and humps (and overturning) still win while reversing. Limitation: a
reverse slower than 20 mm/s (the margin against standstill encoder jitter)
reports MOVING or ACCELERATING.

**Turning vs line-following wiggle.** A steering correction is +/-37.5 mm/s per
wheel, about 37 deg/s, but it flips left and right, so the heading over half a
second nets close to 0: that stays MOVING. TURNING needs the heading itself to
move >= 15 deg within ~0.5 s (an average of ~30 deg/s held): barcode turns,
U-turns, line-search pivots and sharp curves; gentle curves stay MOVING. The
window is 5 heading samples, 100 ms apart, from Buddy 2's odometry heading
(integrated from raw encoder counts, unwrapped). It reseeds after a gap, an
invalid heading or an impossible jump (> 90 deg in 100 ms = odometry reset).
The instantaneous yaw rate (deg/s, from wheel speeds) is still published and
printed, but it no longer decides the class.

**Turn angle.** Each completed turn prints
`[TURN] left 88.4 deg in 1.9 s (peak 50 deg/s)` (+ = left). A turn starts and
ends on the heading condition (same 60 ms debounce), not on the class shown,
so a hump during a turn does not split it. The start is backdated
to the window start. The angle comes from the encoders, so wheel slip inflates
it; a magnetometer heading would be the independent cross-check.

Why not the IMU for everything: the GY-511 has **no gyroscope**, so turn rate
comes from the wheel speeds (wheel slip shows as turning). Getting to 0.1 m/s
takes about 0.3 s, only about 0.03 g, which sits in the accelerometer's
vibration noise, so acceleration comes from the encoder speed change. The IMU's
own forward acceleration (`imu_fwd`, high-passed, mg) is printed next to it in
`[MOTION]` lines for the terrain report comparison.

## Lift and overturn stop

Replaces impact detection (Change Requirement above). `lift.c` reads every
fresh MONITORING sample (raw, unfiltered, mg) against the level reference
captured at boot: the reference **up** axis and the **gravity size** measured
then (so sensor scale error cancels). It computes its own values because
`tilt_update()` rejects exactly these poses (> 75 deg, upside down, outside
700..1300 mg). "Upright" = sample component along up / sample size: 100 %
level, 0 % on its side, negative upside down.

| Condition | Rule (thresholds are `#define`s in `lift.h`) | Reported as |
|---|---|---|
| OVERTURNED | upright < 50 % (more than 60 deg from upright, on its side or upside down) for 200 ms | motion class 7 OVERTURNED |
| LIFTED (tilted) | upright < 82 % (more than ~35 deg; lifted by one end) for 200 ms | T_STATE 7 LIFTED |
| LIFTED (raised) | upward acceleration >= +200 mg above gravity on 4 consecutive samples (~80 ms) | T_STATE 7 LIFTED |

Margins: one-wheel humps tilt the robot about 20 deg at most (asin(40/115)),
and hump landings or knocks are shorter than 80 ms. A lifted robot that then
tips past 60 deg becomes OVERTURNED. Detection is armed 100 ms after entering
MONITORING (`B4_LIFT_SETTLE_MS`), so the hand placing the robot during capture
is ignored. Samples below 300 mg (near free fall) have no direction and never
count as overturned. While a steep tilt is being timed or the latch is set,
hump measurement is suspended, so lifting the robot by one end does not record
a fake hump (a hump rising before the 35 deg is reached may still show).

**Action.** On a new latch, `terrain_task` calls `motion_stop()` once
(controlled STOP: Buddy 2 brakes and holds; the Vehicle Controller rejects all
movement until START). If the robot is released while still latched (START
pressed in the air), it is stopped again at once, so the wheels never drive
while lifted. Path and latency: lift sample -> latch (80-220 ms of detection)
-> `CONTROL_STOP` bus latch in the same 20 ms cycle -> motion task within 10 ms.

**Clearing.** The latch clears after the robot is upright (>= 95 %) and still
(sample size within 100 mg of gravity) for 500 ms; held still and level in the
air also clears it, which is why nothing restarts automatically: the robot
stays stopped until the dashboard START (which needs STOPPED, stationary and
trusted encoders). No reboot is needed (the level reference is kept).

**Limits.** Lift detection runs only in MONITORING: during sensor RECOVERY or
after SENSOR_FAULT it is unavailable and any latch is cleared (sensor faults
still never stop the motors). The GY-511 has no gyroscope, so a perfectly
level lift with very gentle acceleration (under +200 mg) is only caught once
the robot tilts past 35 deg. **Tune the thresholds** with hand lifts, overturns
and hump runs (S4-H01..S4-H05): humps must never stop the robot.

The old INT1 inertial interrupt registers (CTRL_REG2_A, CTRL_REG5_A,
INT1_CFG_A) are written back to their reset value 0x00 at every configuration,
because the GY-511 is not power-cycled when the Pico is reflashed.

## Compass angle per turn

The encoder turn angle (`[TURN]`) stays the measurement. The calibrated
magnetometer measures, over the same turn (same backdated start), how far the
**body** actually rotated, and prints it alongside for comparison:

    [TURN] right 90.4 deg in 1.8 s (peak 50 deg/s) compass -88.1 deg
    [TURN] left 45.0 deg in 0.9 s (peak 50 deg/s) compass n/a (field)

A much smaller compass angle than encoder angle suggests wheel slip; this is
read by eye for now (an automatic SLIP verdict was built, tested and removed
until the course's magnetic field is better understood: on the test course the
field varied 51..682 counts, so the compass was usable only in quiet areas,
where it agreed with the encoders within 2 deg). The compass never affects
driving. `compass n/a` reasons: `uncal` (no constants yet), `no data`, `hump`
(tilt distorts the flat projection), `field` (strength outside +/-20 % of the
calibrated value: steel nearby), `window` (invalid just before the turn). Only
heading *changes* are used, so true north and declination do not matter.

### Calibration (once, beforehand)

The robot calibrates itself by pivoting on the spot:

```sh
cmake -S . -B build-mag-cal -G Ninja -DSMP=OFF -DWIFI=none -DNET=none -DCONSOLE=usb_cdc \
      -DROBO_LINE_STEERING=OFF -DROBO_SUBSYSTEM5=OFF -DROBO_MAG_CAL=ON \
      -DROBO_MANUAL_CONSOLE=OFF -DROBO_SAFE_MOTOR_TEST=OFF -DROBO_ANGLE_TEST=OFF -DROBO_ENCODER_DIAGNOSTICS_ONLY=OFF
cmake --build build-mag-cal -j 8      # flash build-mag-cal/mtkernel_3.uf2
```

1. Floor, **30 cm clear, away from steel** (table legs, cabinets, laptops).
   Power up level and still.
2. After the tilt capture it counts down 3 s, pivots **left 360 deg then right
   360 deg back** at 30 deg/s (through the Vehicle Controller; Buddy 2 drives),
   and stops. Turning back leaves the USB cable untwisted. Line following and
   obstacle avoidance are not in this image; CMake refuses unsafe combinations.
3. It prints `[MAGCAL] PASS | offset_x=.. offset_y=.. scale_y=../4096 field=..
   sign=.. | coverage 36/36 bins, fit error x.x %` and six `#define` lines
   (repeated every 5 s). Paste them into `mag_cal.h` and flash the normal image.
4. FAIL (not saved): `coverage` (did not face every direction), `fit` (not a
   clean circle, > 4 % error: steel nearby), `range`, `timeout` (40 s) or
   `robot not turning` (wheels rejected or slipping). Fix and power-cycle.

Calibration is 2D (X/Y of a flat spin): the robot only rotates about its
vertical axis, and small mounting tilt is absorbed. It corrects the robot's
own magnetism and stays valid at any location while the GY-511 mounting,
wiring and nearby metal stay the same; redo it after any such change.

**At a new venue (2 min, no recalibration):** the `[TILT]` line shows
`compass=..deg field=..`; check the field stays within +/-20 % of `B4_MAG_FIELD`
at a few spots, then do one measured 90 deg turn: encoder and compass should
agree within a few degrees. A spot with steel just gives
`compass n/a (field)` there.

**First-time checks:** turning left by hand must increase the compass heading
(the calibration sets `B4_MAG_HEADING_SIGN` from the spin); compare the
heading with motors off and running while still (Tribunal A5). If the motors
shift it more than about 5 deg, note it: compass angles taken while driving
then carry that error.

## Interfaces

Task: `terrain_task`, priority 8, 4096-byte stack, 20 ms period. I2C transfers
are synchronous; the lifecycle re-reads the clock after each transfer.

Shared Terrain slot, published every cycle via `robot_update_subsystem`
(see docs/API.md and `subsystem1/telemetry_map.c`):

| Slot | Content |
|---|---|
| `available` | true once the task runs (the state itself is genuine data) |
| `timestamp` | time of the last valid tilt sample, seconds |
| `values[0]` | turn rate rad/s, + = left (from wheel speeds), when motion-valid (bit 9) |
| `values[1]` | pitch rad, + front up, when `TF_TV_PITCH` is set |
| `values[2]` | highest hump peak this run, m (1 mm steps), when `TF_TV_HUMP` is set |
| `values[3]` | validity bits (`TF_TV_PITCH`, `TF_TV_HUMP` once a hump is confirmed) \| `T_STATE` << 3 \| motion class << 6 \| motion-valid << 9 |

`T_STATE` (3 bits, 8 codes): 0 INIT, 1 WAITING, 2 CLIMBING, 3 DESCENDING
(1..3 are MONITORING with that terrain phase), 4 RECOVERY, 5 FAULT, 6 STOP,
7 LIFTED (MONITORING, lift latch set, motors stopped). OVERTURNED is motion
class 7 instead, so until Buddy 1 adds `M_CLASS` to the binary frame (see
`docs/BUDDY4_FRAME_CHANGE_REQUEST.md`) an overturned robot shows on the
dashboard only as motion STOPPED/held with pitch invalid, and in full on the
USB console.

Hump completion latches `TF_EV_HUMP_PEAK` through `comms_latch_event`, which
is interrupt-safe. The dashboard's `HUMP_MM` shows the run maximum.
Buddy 4 does not request a slower speed over humps (tested on the robot, the
effect was small and not needed); its only motion command is the lift/overturn
`motion_stop()`. Other subsystems decode the slot
with `include/terrain_status.h` (header-only, safe with Buddy 4 disabled):
`terrain_published_state()` (returns `TERRAIN_TS_UNKNOWN` = 0xFF when the slot
is unusable, outside the 3-bit codes), `terrain_motion_class()` and
`terrain_imu_faulted()`.
Validity bit 0 (heading) stays clear, so the current frame's `T_HEADING` stays 0
until Buddy 1 adds `M_CLASS` and renames `T_HEADING` to `TURN_RATE`.
Nothing is sent as MQTT text any more; data reaches the dashboard only through
the binary status frame (Change Requirement 1).

## Console

`[B4-STATE]` and `[TERRAIN-STATE]` transitions print once, and each hump prints
`[TERRAIN] Hump completed: peak=32 mm under RIGHT wheel | run max=41 mm`.
Each class change prints `[MOTION] MOVING -> TURNING yaw=37 deg/s accel=..
mm/s2 imu_fwd=.. mg`, each lift
`[LIFT] LIFTED (upright=64% vertical=312 mg) -> motors stopped; set it down level, then START`
(or `OVERTURNED`), `[LIFT] still LIFTED: motors stopped again` when released in
the air, `[LIFT] cleared: upright and still; motors stay stopped until START`,
and each completed turn `[TURN] left 88.4 deg in 1.9 s (peak 50 deg/s) compass 87.9 deg`
(or `compass n/a (reason)`). The calibration image prints `[MAGCAL]` lines.
Every 500 ms: `[B4] INIT ...` capture progress, `[TILT] pitch=.. roll=.. |
conditions | height=..mm R/L | terrain=.. run_max=..mm | class=.. yaw=.. | compass=..deg
field=.. | lift=NONE` (or `compass n/a (reason) field=..`), while tilt is
unavailable (e.g. overturned) `[TILT] UNAVAILABLE ... | lift=OVERTURNED
upright=-98% vertical=.. mg`, or the fault state. Suppressed in `ROBO_IR_MONITOR` builds.

## Checks

```sh
python tests/subsystem4/run_tests.py
```

Needs native gcc on PATH. Covers configuration/readback, every 12-bit axis
value, tilt geometry and pitch-independent wheel height, height-based hump
phases/peaks, motion classes (priority, debounce, overturned, yaw), lift and
overturn detection (`test_lift.c`: thresholds, 200 ms hold, clearing, hump
profile immunity, sensor scale, gaps, settle window), compass
calibration/heading/validity,
the compass angle per turn, the calibration pivot sequence (`test_mag_cal_task.c`),
lifecycle/recovery/STOP, the real
`terrain_task` loop (snapshot encoding, T_STATE codes, hump event, level
gating, 20 ms release grid, lift stop and re-stop when released in the air) and the I2C driver's multi-byte receive STOP.
These are host results; physical accuracy and timing need robot tests.
