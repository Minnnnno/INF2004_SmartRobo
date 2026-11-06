# Subsystem 5: obstacle detection, width and avoidance

`obstacle_task` (priority 4, created by `usermain`) watches straight ahead
while the line follower drives. It stops the robot with the sensor 150 mm
from an obstacle, sweeps the servo-mounted ultrasonic sensor across it, and
reports the obstacle's width, whether it sits more to the left or right, and
which side to pass it on. It then drives round the obstacle on that side,
leaving the line, and comes back onto the line, where the line follower
takes over again (see [Driving round it](#driving-round-it)). Whenever the
detour is in doubt, the robot stays stopped instead. Set `S5_AVOID_DRIVE`
to 0 (subsystem5.h) to always stay stopped after the scan.

To isolate subsystem 5 during line-following diagnosis, configure with
`-DROBO_SUBSYSTEM5=OFF` and rebuild. All subsystem 5 source files are excluded
from the firmware and no obstacle task is created. GP15/GP16/GP17 are not
initialized by this subsystem and it cannot issue motion stops. The production startup
banner reports `subsystem5_enabled=0`. Restore obstacle detection by
configuring with `-DROBO_SUBSYSTEM5=ON` and rebuilding (the default is ON).

| Component | Signal | Pico GPIO |
|---|---|---|
| Ultrasonic sensor | TRIG | GP16 |
| Ultrasonic sensor | ECHO | GP17 (PWM slice 0 B, used as a hardware pulse timer) |
| Servo motor | PWM | GP15 (PWM slice 7 B, 50 Hz) |

The sensor sits on the servo: 20 deg = left, 90 deg = straight ahead,
160 deg = right (the servo is clamped to 20..160). The task only runs in production builds. Build with
`ROBO_LINE_STEERING=ON` to drive. In calibration profiles and
`ROBO_IR_MONITOR` the task sleeps and never touches these pins.

These are **logical** directions. On the confirmed mounting (4 November 2026),
the old low-pulse/20-degree position physically pointed right. The servo driver
now reverses the pulse mapping globally (`S5_SERVO_REVERSED=1`), so both the
initial scan and the detour use the same correct directions. Physical pulse
trim is applied after reversal, preserving the established straight-ahead pulse.

The sensor is the 3.3 V HC-SR04 variant, so ECHO connects to GP17 directly.
A 5 V module would need a divider or a level shifter on ECHO.

## What happens

1. **Watching.** The servo stays at 90 deg and pings every 60 ms. The front
   distance is the median of the last 3 pings. Each ping is first moved
   forward by the distance driven since it was taken (encoder speed), so the
   median does not lag behind the robot. A missed ping counts as "clear", so
   a single stray echo can't stop the robot.
2. **Approach (450 mm).** This is reported on the console only. The line
   follower (subsystem 3) owns the wheels, and the Vehicle Controller does
   not let a second source change their speed. Subsystem 5 can only stop
   them. At the line speed of 80 mm/s the robot stops within a few
   millimetres.
3. **Stop (150 mm).** `motion_stop()` is called on the ping nearest
   150 mm (`S5_STOP_MM`). It was 250 mm; closer, the obstacle spans a
   wider angle, so the beam's spread past each edge matters less. The prediction allows for the 10 ms motion tick and
   `S5_BRAKE_MM` of braking. The hold latch keeps the line follower from
   restarting. The scan waits until Motion Control reports the robot still.
4. **Coarse sweep.** The sweep runs left to right, 20 to 160 deg, every 10 deg.
5. **Fine scan.** From the outermost coarse angle still on the obstacle, the
   scan walks outward 1 deg at a time. It stops once two clear angles in a
   row are seen, or at the sweep limit. It then walks back inward until it
   sees the obstacle twice in a row (at most `S5_INWARD_MAX_STEPS`). The
   servo's gears have play: walking out, the horn lags behind and the edge
   appears too far out; walking in, too far in. The mean of the two
   crossings cancels it (the host test shows 3 deg of play cancelled).
6. **Width.** Each edge is that mean, moved inward by the beam correction.
   It is converted to a sideways position with the front face's forward
   distance. Width = left edge - right edge. The task prints the width, the
   stop distance and the obstacle's centre offset. The same values go to
   the shared status and, when MQTT is online, to `robot/telemetry/obstacle`.

7. **Position.** The task compares how far each edge is from straight ahead.
   Left angle = 90° − left edge angle; right angle = right edge angle − 90°.
   If they differ by `S5_CENTRE_TOLERANCE_DEG` (12°) or less, the obstacle is
   **centre**. Otherwise it is **left** or **right**, whichever side
   reaches further. An edge beyond the sweep is only a minimum angle. So it can
   show the obstacle sits towards that side, but it can never show it is
   centred; that case is reported as **unknown**. The beam's spread past each
   edge is about the same on both sides, so it cancels in the difference even
   though it spoils the width. 12° is set from five scans of centred objects,
   whose middle scattered by 3.6° (about 7° in the difference); at the 150 mm
   stop it is about 19 mm of centre offset. This check depends on the servo's
   90° pointing truly straight ahead (`S5_SERVO_TRIM_DEG`).
8. **Pass side.** The robot passes on the side the obstacle reaches less far
   to: obstacle **left** -> pass **right**, and the other way round. A
   **centre** obstacle is passed on `S5_DEFAULT_AVOID` (right). For each side
   the task works out how far the robot's centre must move sideways to clear
   that edge: edge position + half the robot's 160 mm width + 50 mm margin.
   A side has room when its edge was found and nothing else beside the
   obstacle is closer than that move + half the width + the margin. If the
   preferred side has no room the other is used; if neither has, the result
   is **no way around**.

   **The face.** The robot can stop tilted to the obstacle, most of all after a
   curve, since two line sensors give its position but not its angle. A flat
   face reads the same either side of its perpendicular, and the beam hears a
   corner at the corner's own range however far past it the beam points. So
   `s5_face_fit()` takes from the ranges alone: the tilt (the midpoint of where
   the range rises through one level on each side), the distance D (the shortest
   range) and each corner's reach along the face, `sqrt(range^2 - D^2)`. Width is
   the two reaches, and none of this needs the width of the beam. A corner is
   placed if the range jumps past it, or stops rising at the limit of the sweep;
   a range still rising there (a wall) leaves that side with no room.

   With a fit, the side and the shifts are measured square to the face, from the
   axle. A face more than about 20 mm (`S5_FACE_CENTRE_FRACTION` of D) off centre
   is passed on the side it reaches less; a centred one on the side that was
   closer when the robot stopped, if tilted by `S5_FACE_TILT_MIN_DEG`. The detour then works
   in the face's frame, so every turn is measured from square to it, the pass
   runs parallel to it and the way back aims for the line. This assumes the box
   sits square to the line; if it does not, the way back is off by that much. If
   the scan cannot support a fit (a thin post, too little rise, straight ahead
   being a different object from the nearest), the edge angles decide as above.

   On 200 mm box scans logged on 5 November the fit read 167 to 206 mm, median
   184. Each scan prints a `face:` line. To calibrate, scan a square-on box a few
   times and set `S5_FACE_TILT_BIAS_DEG` to the tilt read, sign reversed. The
   `curve check:` line is only logged now.

   "Something beside the obstacle" is any echo outside its edge that is not
   part of it, is more than `S5_SPILL_RANGE_MM` beyond its farthest point
   (nearer off-axis echoes are its own corners), and is no more than
   `S5_PASS_LOOKAHEAD_MM` (300 mm) further ahead than its face. Its distance
   from the centre line is taken without the beam correction, so it errs
   towards "no room".

### Reading the decision

- `s5_get_avoidance(&avoid)` (subsystem5.h), from any task: true while the
  robot is held at a scanned obstacle or driving round it. `avoid.side` is
  `S5_AVOID_LEFT`, `S5_AVOID_RIGHT` or `S5_AVOID_NONE`;
  `avoid.shift_mm[S5_LEFT/RIGHT]` the sideways move for each side;
  `avoid.b_room[]` and `avoid.clear_mm[]` why.
- Shared status (`motion_get_status()`, `.obstacle`): `values[1]` = obstacle
  centre (m, + left), `values[2]` = width (m), `values[3]` = `S5_STATE_*`:
  0 clear, 1 held/scanning, 2 pass left, 3 pass right, 4 no way around,
  5 driving round it, 6 detour abandoned (stopped and held).
- MQTT `robot/telemetry/obstacle`: two text diagnostics, each bounded to the 96-byte queue buffer:
  `obstacle width_mm=<w> dist_mm=<d> offset_mm=<o> complete=<0|1> position=<centre|left|right|unknown>`
  and `obstacle avoid=<left|right|none> shift_l_mm=<l> shift_r_mm=<r>`.
  A message that cannot fit is skipped with a console diagnostic.
  Scan completion also latches `TF_EV_OBSTACLE | TF_EV_SCAN_DONE` for the binary `robot/event`
  frame, including when the scan finishes offline.
- Console:

```
[OBSTACLE] position: LEFT by 14.0 deg (left edge 31.5 deg, right edge 17.5 deg, tolerance 12.0 deg)
[OBSTACLE] ===== PASS ON THE RIGHT (obstacle reaches less far that way) =====
[OBSTACLE]   left : move 230 mm, nothing beside it -> room
[OBSTACLE]   right: move 172 mm, nearest thing beside it 498 mm out -> room
```

## Driving round it

After the scan, if a side has room, `obstacle_task` drives the detour
itself (`detour.c`, geometry in `detour_plan.c`). Every move is one of
Buddy 2's finite moves (`motion_turn_degrees`, and `motion_steer_distance`
at 60 mm/s for straight legs) under `SOURCE_OBSTACLE`, submitted once and
waited for until it completes. Positions come from the encoders, in the
frame of the stop: x forward along the line, y to the left, measured at the
axle centre.

1. **Keep the line follower off.** Between moves the robot is READY, which
   would let the line follower drive. So `values[3]` is set to 5
   (`S5_STATE_AVOIDING`) and confirmed in the shared status *before* the
   hold is released; `line_task` (line_following.c) does not drive while it
   reads 5. The task only releases the hold it latched itself, and only
   once Motion Control reports the robot still. It also needs the line
   sensors to be reporting (line steering build), or it will not leave.
2. **Out.** The robot turns out by 45..75 deg, drives the diagonal that
   puts its centre the planned shift to the side, and turns back parallel
   to the line. The angle is chosen so that its front corners never come
   within `S5_TURN_MARGIN_MM` of the obstacle's face during any of this; if
   even 75 deg does not fit, it reverses first (at most `S5_BACKUP_MAX_MM`,
   otherwise it stays stopped).
   After turning out, it checks three bearings (30, 0, -30 deg relative to the
   chassis heading) before each diagonal section of at most 400 mm. Readings
   are tested against the next section's forward travel, front overhang, robot
   half-width and 50 mm margin. A 15-degree beam bearing allowance widens the
   guard conservatively, and with 30-degree spacing the beams just meet. The
   sensor is left at the last bearing; every later look moves it first. Each
   measurement requires stationary READY motion, no hold or fault, and an
   unchanged command generation before and after settling/sampling.

   The diagonal is the planned length plus `S5_DIAGONAL_EXTRA_MM` (50 mm), and
   it is **commanded** distance: a section that finishes a few mm short is not
   made up, so the car can end the diagonal short of the extra room. A
   remainder under `S5_MIN_DIAGONAL_STEP_MM` (10 mm) is dropped, because moves
   that short do not turn the wheels, and the last two sections share the
   distance if the last would be shorter than that. Lack of progress aborts.
   There is no automatic replan around an obstruction.

   Once parallel, it repeats the three-bearing sweep for the next
   `S5_PASS_STEP_MM` of forward passing, then looks towards the obstacle (logical 20 deg when
   avoiding right; 160 deg when avoiding left). A side echo implying less than
   130 mm of axle-centre clearance (80 mm half-width + 50 mm margin) aborts.
   Blocked checks, sensor failures, movement and interruptions all stop the
   detour and leave it held. Console messages distinguish `clearance=diagonal`,
   `clearance=parallel`, and `insufficient sideways clearance`.
   If the diagonal sweep is blocked by an echo (not a sensor fault), the robot
   first turns further out by `S5_OUT_RETRY_STEP_DEG` and sweeps again, up to
   `S5_OUT_RETRY_MAX_DEG`, with a shorter diagonal for the same sideways shift,
   and only gives up if every angle is blocked.
3. **Past it.** It drives forward in one continuous move, without stopping to
   look. The one sensor alternates between sideways at the obstacle (servo 20
   or 160 deg, 70 deg off straight ahead) and straight ahead (stops if anything
   is within `S5_PASS_STEP_MM` + overhang + margin), about 1.4 s and
   `S5_PASS_STEP_MM` of travel for each pair. Faults and the line are checked
   during the servo swing and between pings. Side pings that disagree are
   ignored; ahead, they stop the robot only if the echo is close. An echo no
   further out than the obstacle's edge + `S5_SIDE_SEE_MM` is the obstacle. A
   clear look counts only where the beam crosses the obstacle's side beyond the
   last echo, and it takes `S5_PASS_CONFIRM` (2) in a row to end it: a single
   missed echo does not. The end is taken where the first of those clear looks
   crossed, so with the beam's spread it errs long. It gives up after
   `S5_PASS_MAX_MM`.
4. **Rear clear.** It stops at that end + the rear-corner swing +
   `S5_PASS_MARGIN_MM` - `S5_PASS_TRIM_MM`. With the 150 mm trim those cancel, so
   it stops level with the estimated end, or later if the second clear look
   came later. Clearing the obstacle now relies on the sideways gap, not on
   driving past it first; `subsystem5_test.c` checks the geometry.

   **Line interrupt (from `S5_REJOIN_AFTER_MM`, 20 mm, into step 3, not during
   a stall escape, and in steps 4 and 5).** If Buddy 3's centre and right IR
   sensors both read black (`line_rejoin_seen()` in
   `subsystem3/line_rejoin.h`, debounced over two samples), the robot stops and
   goes to step 6, wherever odometry puts it. If braking carried it off the
   line it reverses onto it, and if the line still is not there it holds. The
   robot is not aligned to the line, and if this happens in step 3 or 4 it may
   still be beside the obstacle. Only the forward ultrasonic guards it from
   there.
5. **Back.** It turns in by `S5_RETURN_DEG` (45 deg), checks the way ahead
   is clear, and drives towards the line at `S5_LINE_SEARCH_SPEED_MM_S`. Both
   sensors black ends it, as above. One sensor black is not enough, since they do
   not reach the line together, but if the sensors come off the line without both
   having been black it stops and reverses to the middle of what it crossed, and
   hands over as above. One sensor only counts within `S5_LINE_WINDOW_MM` of where
   odometry puts the line. If nothing is found within `S5_RETURN_EXTRA_MM` past
   where the line should be, it stops.

6. **Hand back.** `values[3]` returns to 0 and the line follower takes over from
   wherever the robot is, typically at an angle across the line. Going straight
   loses the line, and the follower's search (up to 60 deg each way) finds it. It
   does not first drive on and turn back to its heading at the stop, as it used
   to: that assumes the line runs parallel to that heading, and on 4 November
   2026 it missed by about 13 mm and abandoned the detour. The next obstacle is
   scanned afresh.

If anything fails (a move rejected, faulted, timed out or stopped by
someone else, something ahead, the obstacle not ending, the line not
found), the robot is stopped with `motion_stop()` and held, and `values[3]`
reads 6. Releasing it (e.g. MQTT `start`) hands it back to the line
follower as after any stop.

Sensor faults (`NO_PULSE` or stuck-high `BUSY`) hold the robot; they do not
count as clear space. A normal no-echo response still does. A sweep is
cancelled if the robot cannot confirm that it is held and stationary,
including if it moves while the servo settles or readings are taken.
Side and front clearance calculations include the 30 mm distance from the
servo axis to the ultrasonic transducer, as the width scan does.

**Measure before running it:** `S5_SERVO_AHEAD_MM`, `S5_FRONT_MM`,
`S5_REAR_MM` and `S5_IR_AHEAD_MM` (detour_plan.h) are placeholders. Measure
each from the centre of the wheel axle. The return length and the one-sensor
window assume the line runs roughly straight past the obstacle.

The guards sample finite bearings with a broad ultrasonic beam; no echo is not
proof of an entirely empty corridor. The side distance is an estimate from the
beam axis. Physical dimensions, beam correction and trim still need checking
on the robot after correcting the direction mapping.

```
[DETOUR] out: back 0 mm, turn 61 deg, diagonal 188 mm -> 165 mm left of the line
[DETOUR] pass x 91 y 165: side 0 mm -> obstacle to x 240, clear 0/2
[DETOUR] pass x 203 y 165: side 122 mm -> obstacle to x 283, clear 0/2
...
[DETOUR] obstacle ends by x 443 (4 echoes); rear clears it at x 443
[DETOUR] back: turn -45 deg, up to 353 mm to the line
[DETOUR] line at x 560 y 48, heading -45 deg, stopped 6 mm on; interrupting the detour
[DETOUR] back on the line at x 560; line following resumes
```

### Each angle's reading

- The servo waits 100 ms + 4 ms per degree moved before pinging, so the
  sensor on top has stopped swinging.
- 5 pings are taken, each with a 30 ms echo window (a 5 m round trip;
  the sensor's limit is 4 m). Triggers are at least 60 ms apart, so a late
  echo from one ping can't be mistaken for the next.
- The result is a "mode with tolerance". The largest group of pings that
  agree within 10 mm wins and is averaged. At least 3 of 5 must agree.
  Missed pings vote for "nothing there". If no group wins, 5 more pings are
  taken and the vote is repeated over all 10.
- The echo pulse is timed by the PWM counter in hardware (1 us = 0.17 mm),
  so task scheduling does not affect it. The speed of sound assumes 25 °C
  (`US_AIR_TEMP_C`).

### "Within 30 %" rule

A point belongs to the obstacle when its *forward* distance (range x
cos(angle from straight ahead)) is within 30 % of the 90 deg forward
distance (~54 mm at the 150 mm stop, ~80 mm at 250 mm). Raw
range is not used: the raw range to a flat face grows with the angle. At
40 deg, 400 mm becomes 522 mm, which would wrongly mark a wide flat obstacle
as clear. Straight ahead, the two are the same.

## Tuning (top of each file)

| Constant | File | Meaning |
|---|---|---|
| `S5_STOP_MM`, `S5_SLOW_MM` | subsystem5.h | 150 / 450 mm |
| `S5_BRAKE_MM` | subsystem5.c | Travel after the stop. Raise it if the robot overshoots 150 mm |
| `S5_EDGE_THRESHOLD_FRACTION` | obstacle_scan.h | The 30 % rule |
| `S5_CENTRE_TOLERANCE_DEG` | obstacle_scan.h | Edge-angle difference still called centre (12°) |
| `S5_INWARD_MAX_STEPS` | obstacle_scan.h | Most points walked back inward per edge (20) |
| `S5_ROBOT_HALF_WIDTH_MM` | obstacle_scan.h | Half the robot's width (80 mm) |
| `S5_PASS_MARGIN_MM` | obstacle_scan.h | Space kept from the obstacle and anything beside it (50 mm) |
| `S5_PASS_LOOKAHEAD_MM` | obstacle_scan.h | How far past the face something can still block a side (300 mm) |
| `S5_SPILL_RANGE_MM` | obstacle_scan.h | Off-axis echoes nearer than the obstacle's far point + this are its corners (50 mm) |
| `S5_DEFAULT_AVOID` | obstacle_scan.h | Side for a centred obstacle (right) |
| `S5_FACE_JUMP_MM`, `S5_FACE_TAIL_MM` | obstacle_scan.h | A bigger step ends the face (45 mm); a last reading this far past the one before is a grazing echo, not the corner (20 mm) |
| `S5_FACE_CENTRE_FRACTION`, `S5_FACE_TILT_MIN_DEG` | obstacle_scan.h | Off-centre tolerance as a fraction of D (0.11, about 20 mm at the stop); tilt that decides a centred face's side (5�) |
| `S5_FACE_TILT_BIAS_DEG` | obstacle_scan.h | Added to every tilt: **measure** on a square-on box (0� until then) |
| `S5_CURVE_MIN_DEG`, `S5_CURVE_WINDOW_MM` | obstacle_scan.h | A curve, for the log only: this heading change within this travel (25�, 300 mm) |
| `S5_CURVE_HOLD_MM` | obstacle_scan.h | How much further it still counts as after a curve (300 mm) |
| `S5_SAMPLES_PER_POINT` | obstacle_scan.h | Pings per angle (5) |
| `S5_SENSOR_OFFSET_MM` | obstacle_scan.h | Servo axis to sensor along the horn arm (30 mm) |
| `S5_BEAM_CORRECTION_DEG` | obstacle_scan.h | Beam-width correction (22°), below |
| `S5_SERVO_TRIM_DEG` | servo.h | Added to every servo angle so 90° points straight ahead (1°) |
| `S5_SERVO_REVERSED` | servo.h | Set to 1 if the sweep starts on the right |
| `S5_AVOID_DRIVE` | subsystem5.h | 1: drive round the obstacle; 0: stay stopped after the scan |
| `S5_SERVO_AHEAD_MM`, `S5_FRONT_MM`, `S5_REAR_MM`, `S5_IR_AHEAD_MM` | detour_plan.h | Robot dimensions from the axle centre: **measure** |
| `S5_TURN_MARGIN_MM` | detour_plan.h | Kept from the face by the front corners while turning out (30 mm) |
| `S5_OUT_MIN_DEG`, `S5_OUT_MAX_DEG` | detour_plan.h | Outward turn range (45..75°) |
| `S5_BACKUP_MAX_MM` | detour_plan.h | Most reversing before turning out (150 mm) |
| `S5_OUT_RETRY_STEP_DEG`, `S5_OUT_RETRY_MAX_DEG` | detour_plan.h | If the diagonal sweep finds something in the way, turn this much further out (10�) and look again, up to straight sideways (90�); the diagonal is shortened to keep the same shift |
| `S5_SQUARE_MIN_DEG`, `S5_SQUARE_TRIES` | subsystem5.c | Tilted by at least this (5 deg): turn on the spot to face the obstacle, up to 2 times, checking the angle with a coarse sweep only. The full scan with the fine edges runs once, after the last turn. The tilt it stopped with is kept for the side choice and the way back to the line |
| `S5_STILL_MOVE_MM` | detour.c | A scan or stopped look only fails on real movement beyond this (5 mm): one encoder count of wobble clears Motion Control's "stationary" and is ignored |
| `S5_FACE_EDGE_ECHO_DEG`, `S5_FACE_EDGE_CONFIRM_DEG` | obstacle_scan.h | A reading within 3 deg past the face's end only blocks that side if one within the next 5 deg agrees: it can be the corner's own echo |
| `S5_DIAGONAL_STEP_MM` | detour_plan.h | Maximum diagonal section between stopped sweeps (400 mm, so normally one section) |
| `S5_DIAGONAL_EXTRA_MM`, `S5_MIN_DIAGONAL_STEP_MM` | detour_plan.h | Driven beyond the planned diagonal (50 mm); shortest diagonal section, shorter remainders are dropped (10 mm) |
| `S5_GUARD_SWEEP_DEG`, `S5_GUARD_SWEEP_STEP_DEG` | detour_plan.h | Guard bearings: +/-30 deg in 30-degree steps |
| `S5_GUARD_BEAM_HALF_DEG` | detour_plan.h | Conservative beam bearing uncertainty for corridor checks (15 deg) |
| `S5_PASS_CYCLE_S`, `S5_PASS_STEP_MM`, `S5_PASS_MAX_MM` | detour_plan.h | Side + ahead look cycle (1.4 s); travel per cycle (speed x cycle, 112 mm; 150 mm fails the "single missed echoes" geometry check); give-up length (800 mm) |
| `S5_PASS_TRIM_MM` | detour_plan.h | Taken off the drive past the obstacle's end before turning back (150 mm) |
| `S5_SIDE_SEE_MM` | detour_plan.h | Side echoes this far past the edge are still the obstacle (150 mm) |
| `S5_PASS_CONFIRM` | detour_plan.h | Clear side looks in a row that end the obstacle (2) |
| `S5_RETURN_DEG`, `S5_RETURN_EXTRA_MM` | detour_plan.h | Return angle (45°), distance past the expected line before giving up (120 mm) |
| `S5_LINE_WINDOW_MM` | detour_plan.h | Black further than this from the expected line is ignored (100 mm) |
| `S5_DRIVE_SPEED_MM_S`, `S5_LINE_SEARCH_SPEED_MM_S` | detour_plan.h | Straight legs (80 mm/s); the leg watching for the line (60 mm/s) |

### Calibrating the beam correction

The ultrasonic beam is a cone, about 15 deg wide. The sensor keeps hearing
the obstacle after its centre line has passed the edge, so the raw width
reads too large: uncorrected, a 200 mm box measured 439 mm. Each edge's
lateral position is the front face's forward distance times the tangent of
the corrected edge angle (the range read at an edge is to the corner and
reads long). To calibrate:

1. Place a box of known width `W` square-on, centred on the line.
2. Let the robot stop at 150 mm and note the printed width `M`.
3. Add `atan(M / 360) - atan(W / 360)` degrees to `S5_BEAM_CORRECTION_DEG`
   (for a centred box with the sensor at 150 mm, i.e. 180 mm from the servo
   axis: 2 x 180 = 360).
4. For the trim, scan a centred object several times (the scatter is a few
   degrees) and add half the mean printed difference to `S5_SERVO_TRIM_DEG`
   (positive when it says RIGHT). Set `S5_CENTRE_TOLERANCE_DEG` to about
   twice the spread of those differences.
5. Rebuild and repeat to check.

## Encoder diagnostics

Encoder snapshots are printed after requesting the obstacle stop and immediately
before/after the scan (`phase=stop-requested`, `scan-before`, `scan-after`, id 0).
Every submitted detour straight/turn request also prints `move-before` and
`move-after` with its command ID, followed by signed count changes:

```text
[OBSTACLE] enc phase=move-before id=5105 L=12345 R=12000 invalid=0,0 reliable=1
[DETOUR] move 5105 queued: straight 5 mm at 60 mm/s
[OBSTACLE] enc phase=move-after id=5105 L=12345 R=12000 invalid=0,0 reliable=1
[DETOUR] enc id=5105 delta_L=0 delta_R=0 result=done
```

These example numbers illustrate a completed command with no encoder travel.
`L`/`R` are coherent live encoder counts with configured polarity; they are
unsigned counters and can wrap when travelling backwards. Signed deltas handle
that wrap. `invalid` is the cumulative invalid-transition count for each wheel;
`reliable` reports whether the capture is initialized. Snapshots use the hardware
capture, separately from Motion Control's periodic odometry/status publication.
Failed/refused requests also print the after snapshot and `result=failed`.
Logging runs in the obstacle task and does not reset encoder counts, change the
move tolerances, or fix the diagonal completion/width-calibration issues.

## Host tests

`subsystem5_test.c` is excluded from the firmware by `CMakeLists.txt`. It
simulates a box face, with noise, stray echoes, a beam cone, servo backlash
and a second object beside the first, and checks the width, the position
and the pass side. It also checks the detour plan (the front corners stay
off the face, reversing when needed, refusals) and finds the end of
simulated obstacles from side looks, with missed echoes and a far wall,
checking the rear always clears the true end. Run it from the repository
root, for example in WSL:

```
gcc -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -DS5_BEAM_CORRECTION_DEG=0.0 \
    app_program/subsystem5/obstacle_scan.c app_program/subsystem5/detour_plan.c \
    app_program/subsystem5/subsystem5_test.c -lm -o s5_tests && ./s5_tests
```

`tests/test_line_pid.c` checks that the line follower does not drive while
`values[3]` reads 5 and resumes when it clears.

The CMake host build also runs `obstacle_runtime_checks` and
`obstacle_servo_checks`. They cover both avoidance directions, blocked centre
and corner bearings, sensor failures and stop/reset/movement during a check,
post-diagonal clearance, the line interrupt and where it is armed, and the
actual servo PWM direction/trim/clamping code with hardware calls replaced.
Run `ctest --test-dir build-host-review --output-on-failure` after building that
host configuration. On 4 November 2026, all 14 host suites passed, including
142 runtime, 176 geometry and 9 servo assertions; strict ARM GCC source checks
and the normal Pico W firmware build passed too. No physical test or flashing
was performed for this change.

## Example output

```
[OBSTACLE] approaching: 448 mm ahead at 80 mm/s
[OBSTACLE] STOP: 403 mm ahead at 80 mm/s
[OBSTACLE] scanning 20..160 deg
[OBSTACLE] stopped 251 mm from the obstacle (target 250 mm), 5/5 pings agree
[OBSTACLE] coarse  80 deg:  254 mm, forward  250 mm,  5/ 5 agree -> OBSTACLE
...
[OBSTACLE] left  edge: obstacle at 70 deg, clear at 69 deg -> 69.5 deg, 425 mm away, 149 mm left of centre
[OBSTACLE] ===== OBSTACLE WIDTH 298 mm (29.8 cm), 401 mm ahead, centre 0 mm left =====
[OBSTACLE] position: CENTRE (left edge 20.5 deg, right edge 20.5 deg, within 2.0 deg)
```
