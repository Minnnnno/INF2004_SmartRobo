# Subsystem 4 — test log

**Subsystem:** IMU-based motion and terrain monitoring (Buddy 4, GY-511 on I2C1)  ·  **Student ID:** 2500910  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results, and
`app_program/subsystem4/README.md` for how the subsystem works.

Two kinds of test:

- **Script tests (S4-U..)** run the Buddy 4 C code on the laptop with
  simulated sensor data. They prove the **logic**. Evidence: the saved
  console output. No video.
- **Video tests (S4-H..)** run on the robot. They prove the **requirement is
  met on the real hardware**, read from the web dashboard. Evidence: a video;
  saved dashboard and serial logs are optional extra evidence.

## Requirements

Sources: **Briefing** = the Project Briefing (mission 3 "detect and measure humps" and
Buddy 4's core work); **FR/NFR** = the team's Week 6 Design Review; **CR** = the lecturer's
Change Requirement of 11 November 2026 (lift/overturn stop replaces collision detection).

| Req | Requirement | Source | Script test | Video test |
|---|---|---|---|---|
| R4-1 | IMU integration: GY-511 configured and read on I2C1 | Briefing: Buddy 4 core work | S4-U03 | S4-H08 |
| R4-2 | Sensor calibration: level reference captured after the dashboard START (robot does not drive before it); compass calibration constants recorded and applied | Briefing: calibrate the IMU; NFR9 | S4-U03, S4-U04 | S4-H08, S4-H16 |
| R4-3 | Tilt detection: pitch and uphill/downhill state | Briefing: detect tilt | S4-U03 | S4-H10 |
| R4-4 | Detect humps along the route using IMU data, with no false humps on flat floor | Briefing mission 3 (detect and measure humps); FR7 | S4-U03 | S4-H11, S4-H15 |
| R4-5 | Estimate hump height and report the **highest hump peak** of the run | Briefing mission 3 (detect and measure humps); FR7, FR8 | S4-U03 | S4-H11, S4-H14, S4-H07, S4-H09 |
| R4-6 | Motion classification with stable events (stationary, accelerating, turning, climbing, descending) | FR9 (except sudden impact, which the CR replaced with R4-9) | S4-U03 | S4-H12 |
| R4-7 | Turn-rate monitoring | FR9 | S4-U03 | S4-H12 |
| R4-8 | Motion event reporting (console and telemetry) | Briefing: report motion events; FR16 | S4-U03 | S4-H11, S4-H12 |
| R4-9 | Lifted or overturned: stop the motors immediately | CR (lift/overturn stop) | S4-U02 | S4-H01, S4-H03 |
| R4-10 | Humps and normal tilts must not trigger the lift stop | CR (humps and tilts are not lifts) | S4-U02 | S4-H04 |
| R4-11 | Humps are recorded only during the run: frozen after an overturn, paused while stopped, resumed by START; paused while turning on the spot (a pivot's sideways jolt is not a hump) | CR (the run ends lifted and turned over); dashboard START/STOP; R4-4 (no false humps) | S4-U03 | S4-H07, S4-H09, S4-H15 |

## Script tests (laptop, no robot)

S4-U02 and S4-U03 come from one command; S4-U04 from the team host suite:

```sh
python tests/subsystem4/run_tests.py > testing/subsystem4/logs/<date>_host.txt
cmake -S . -B build-host-cmake -G Ninja -DROBO_HOST_TESTS=ON
cmake --build build-host-cmake
ctest --test-dir build-host-cmake -R "motion_tests|line_imu_wait" --output-on-failure > testing/subsystem4/logs/<date>_start_gate.txt
```

| ID | What is tested | Requirement | Expected result (in the log) |
|---|---|---|---|
| S4-U01 | **Dropped 2026-11-11** (ID not reused): message buffer max line = S4_SUMMARY_MAX (template demo test, deleted with `task_monitor.c`) | IF-03 | n/a |
| S4-U02 | Lift/overturn detection: thresholds, 200 ms hold, clearing, hump profiles never trigger it, sensor scale, gaps | R4-9, R4-10 | `PASS: <n> lift checks (overturn, lift, hold, clear, humps, scale, gaps, NULL)` |
| S4-U03 | The other Buddy 4 suites: GY-511 configuration and axis conversion, tilt geometry, hump height/peak/run max, motion classes and yaw, lifecycle/recovery, terrain task (snapshot, hump event, lift stop, hump freeze, hump pause while turning on the spot), terrain task with the web Start gate (wait, START, capture, hump, STOP/START), I2C receive | R4-1 to R4-8, R4-11 | A `PASS:` line for each suite (incl. `terrain task checks (..., hump freeze)` and `terrain task web Start gate checks`), ending `All Buddy 4 host suites passed.` |
| S4-U04 | Start gate in the team suite: Vehicle Controller boot hold and START release (`motion_tests`), line task waits for Buddy 4 after START (`line_imu_wait_checks`, `line_imu_wait_start_gate_checks`) | R4-2 | `PASS:` line for each of the 3 tests and `100% tests passed, 0 tests failed out of 3` |

## Video tests (robot + dashboard)

"Dashboard" = `tools/web_dashboard.py` in the browser: the **IMU & terrain**
card (state, Pitch, Highest hump, Motion class, Turn rate), the **Motion** card (state, wheel speed,
heading), the alerts and the event log. The last column lists only what the
dashboard cannot show yet; read those lines from the serial console, kept in
the same video frame.

| ID | What is tested | Requirement | Steps | Expected on the dashboard | Console only (not on the dashboard) |
|---|---|---|---|---|---|
| S4-H01 | Lift while driving stops the motors | R4-9 | Line-following run on the floor; pick the robot straight up, briskly, mid-run | Within ~0.3 s the wheels stop: IMU & terrain card **LIFTED (motors stopped)**, alert *Robot lifted (Buddy 4)*, Motion **STOPPED (held)**, wheel speed **0 / 0**; event log `robot LIFTED: Buddy 4 stopped the motors (end of run)` | `[LIFT] LIFTED ... -> motors stopped` |
| S4-H03 | Overturn stops the motors | R4-9 | Wheels driving on a stand; roll the robot onto its side, then upside down | Within ~0.3 s: Motion **STOPPED (held)**, wheel speed **0 / 0**; Pitch **not valid** with alert *No valid pitch from the IMU* while overturned; IMU & terrain *Motion class* **OVERTURNED (motors stopped)** | `[LIFT] OVERTURNED ... -> motors stopped`, `[MOTION] ... -> OVERTURNED` |
| S4-H04 | Humps never trigger the lift stop | R4-10 | Full run over the course humps under the **right** wheel, normal speed | Never **LIFTED**, no *Robot lifted* alert; Motion stays **DRIVING**; the card goes CLIMBING → DESCENDING → WAITING on each hump and **Highest hump** updates | No `[LIFT]` line |
| S4-H07 | Overturn freezes hump recording until START (demo end) | R4-11, R4-5 | Complete one hump and note **Highest hump**; lift the robot and turn it over; hold it level in the air until LIFTED clears; set it down at an angle and rock it side to side; press START; drive over one more hump | **Highest hump** unchanged from the overturn until START; after START the next hump is recorded and **Highest hump** updates if higher | `[TERRAIN] hump recording frozen (OVERTURNED): run max kept at N mm until START`, `[TILT] ... terrain=FROZEN`, no `Hump completed` until START; then `[TERRAIN] hump recording resumed (START)` |
| S4-H08 | Level reference captured only after the dashboard START; the robot drives only after it | R4-1, R4-2 | MQTT image, dashboard open. Power up, carry the robot around for 30 s, place it level on the course, press START, keep it still | Before START: card **INIT**, Motion **STOPPED (held)**, wheel speed **0 / 0**, no *IMU fault* alert. After START: wheels still while the card shows INIT (about 2 s), then card **WAITING** with Pitch **valid, about 0°**, then Motion **DRIVING** (line following) | `[B4] waiting for dashboard START before capturing the level reference ...`, `[B4] INIT: waiting for dashboard START ...` every 0.5 s; after START `[B4] START: capturing level reference ...`, `[B4-STATE] INIT -> MONITORING`, `line_imu_wait settled=1` |
| S4-H09 | Dashboard STOP pauses hump recording until START | R4-11, R4-5 | Drive over one hump and note **Highest hump**; press STOP; raise one side of the robot ~3 cm by hand and lower it again; press START; drive over one more hump | **Highest hump** unchanged while stopped; after START the next hump is recorded | `[TERRAIN] hump recording paused (robot stopped): run max kept at N mm until START`, `[TILT] ... terrain=PAUSED`, no `Hump completed` while stopped; on START `[TERRAIN] hump recording resumed (START)` |
| S4-H10 | Tilt: uphill/downhill state and pitch | R4-3 | Motors not driving. Moving slowly (not a lift), hold each pose still for 2 s: 1. front raised about 15° (uphill), 2. front lowered about 15° (downhill), 3. set it down level | Pose 1: **Pitch** positive, about +15°. Pose 2: **Pitch** negative, about −15°. Pose 3: **Pitch** back to about 0°. Never **LIFTED** | Uphill/downhill state in the `[TILT]` line: `FRONT UP` (pose 1), `FRONT DOWN` (pose 2), `LEVEL` (pose 3) |
| S4-H11 | Hump detected and its height measured (right wheel) | R4-4, R4-5, R4-8 | 1. Measure a hump with a ruler (e.g. 20 mm). 2. Place it beside the line so only the **right** wheel rides over it. 3. Line-following run over it; write the reading in *Notes* | Card **WAITING → CLIMBING → DESCENDING → WAITING** (event log `terrain WAITING -> CLIMBING` …); event `hump completed`; **Highest hump = ruler height ± 3 mm**; never **LIFTED** | Wheel side: `[TERRAIN] Hump completed: peak=X mm under RIGHT wheel` |
| S4-H12 | Motion events are classified and stable; turn rate reported | R4-6, R4-7, R4-8 | One line-following run from standstill: a straight, a sharp curve or barcode turn, a hump, then stop (or lift to end) | Supporting only: Motion **DRIVING**/**STOPPED**, Heading changes by the turn angle, card CLIMBING/DESCENDING on the hump. *Motion class* and *Turn rate* on the IMU & terrain card follow the run (frame layout v2+, `docs/BUDDY4_FRAME_CHANGE_REQUEST.md`) | **The result is on the console:** `[MOTION]` lines `STATIONARY -> ACCELERATING`, `-> MOVING`, `MOVING -> TURNING` on the turn (`yaw=` deg/s, + left, − right), back to `MOVING`, `-> CLIMBING`, `-> DESCENDING` on the hump, `-> STATIONARY` at the end; no `TURNING` flicker on the straight |
| S4-H14 | Highest hump of the run is kept and reported | R4-5 | Measured hump under the right wheel in a line-following run; keep driving after it to the end of the run | After the hump **Highest hump** = its height (± 3 mm) and keeps that value for the rest of the run | `Hump completed` line; `run max=` equals the peak |
| S4-H15 | No false hump on flat floor, including turns on the spot | R4-4, R4-11 | Flat course, no humps. Line-following run with a sharp curve, a barcode route turn and U-turn (spin on the spot) and a line-search pivot (lose the line on purpose); 3 runs, each from a standing START (acceleration). A dashboard STOP pauses hump recording, so braking after STOP is not what this tests | Card stays **WAITING** (never CLIMBING), also while spinning or pivoting; **Highest hump** stays *none yet*; never **LIFTED** | No `Hump completed` line; `[TILT] ... height=` stays below 5 mm while driving forward; around each spin/pivot `[TERRAIN] turning on the spot: hump recording paused` then `[TERRAIN] turn finished: hump recording resumed`, with `[TILT] ... terrain=TURNING` in between |
| S4-H16 | Calibration recorded and applied | R4-2 | 1. Note the `mag_cal.h` constants (6 Nov 2026 self-pivot, fit 0.9 %) in *Notes*. 2. Place the robot level on the course, power up, press START and keep still. 3. Repeat step 2 three times, power-cycling between them (the reference is captured once per power-up) | After each capture: card **WAITING**, **Pitch** within ± 1° of 0° | `[B4] START: capturing level reference ...`, `[B4-STATE] INIT -> MONITORING`, `[TILT] pitch=` within ± 1.0 and `compass=..deg field=..` (not `n/a (uncal)`), field within ± 20 % of 348 |

## Minimum-bit representation (Change Requirement 1, Week 13)

Evidence for the four Week 13 items, for Subsystem 4 only. All numbers are
measured, not calculated by hand: `python tests/subsystem4/measure_compact.py
<firmware build dir> <output dir>` reproduces them. Output is in
`logs/compact_representation_2026-11-25/`.

**What "before" means.** The first integrated Buddy 4 (commit `b96a929`,
11 October 2026) used unpacked types:
- six `int16_t` per sample;
- the tilt result as four `float`s, three enums and a `bool`;
- Buddy 4's data sent as three MQTT JSON text messages.

The packed representations came with the rewrite in `0b4e01f` (6 November 2026).
The two versions don't have the same features (`b96a929` had no humps, lift or
motion classes), so a whole-image before/after would not compare like with like.
The comparison is therefore made **per data structure**:
- each "before" type is copied from `b96a929`, or is the plain unpacked type for
  data `b96a929` didn't have;
- each "after" type is taken from today's headers.

### 1. Data representation table

**Sent to the dashboard (status frame, `include/telemetry_frame.h`, 10 Hz)**

| Field | Valid range | Bits | Representation |
|---|---|---:|---|
| `T_VALID` | 3 flags | 3 | bit 0 motion class + turn rate valid, bit 1 pitch valid, bit 2 hump valid |
| `T_STATE` | 0..7 | 3 | 0 INIT, 1 WAITING, 2 CLIMBING, 3 DESCENDING, 4 RECOVERY, 5 FAULT, 6 STOP, 7 LIFTED |
| `M_CLASS` | 0..7 | 3 | 0 stationary, 1 moving, 2 accelerating, 3 reversing, 4 turning, 5 climbing, 6 descending, 7 overturned |
| `PITCH` | −90..90 ° | 8 | whole degrees, + nose up |
| `TURN_RATE` | −256..255 deg/s | 9 | + left. Fastest possible pivot ≈ 249 deg/s (both wheels at 250 mm/s, track 115 mm), so 8 bits (±127) would clip |
| `HUMP_MM` | 0..127 mm | 7 | highest completed hump this run, 1 mm steps |
| **Buddy 4 total** | | **33** | 4.125 bytes of the shared 21-byte frame |

**Shared with other tasks (terrain slot `values[3]`, `include/terrain_status.h`)**

| Bits | Content | Valid range |
|---|---|---|
| 0..2 | validity flags (as `T_VALID`, bit 0 unused here) | 3 flags |
| 3..5 | `T_STATE` | 0..7 |
| 6..8 | motion class | 0..7 |
| 9 | motion class + turn rate valid | flag |

The slot itself (`subsystem_status_t`, four `double`s) is the bus format shared by
all buddies, so it isn't Buddy 4's to shrink. Buddy 4 packs its state, class and
flags into one of the four values instead of using more.

**Inside Buddy 4**

| Data | Valid range | Bits | Representation | Where |
|---|---|---:|---|---|
| Acceleration / magnetic sample | 3 axes, −2048..2047 each | 36 per sample | 3 × 12-bit two's complement in 5 bytes. 12 bits is the sensor's own resolution, so nothing is lost | `gy511_read.h` |
| Tilt result | pitch and roll ±75 ° at 0.1 °, 2 conditions, valid | 27 | one `uint32_t`: pitch + 900 in bits 0..10, roll + 900 in 11..21, conditions 22..25, valid bit 26 | `tilt.h` |
| Lifecycle control | 5 states, 0..3 attempts, 3 phases | 7 | bit-field 3 + 2 + 2 in one byte | `buddy4.h` |
| Last hump peak, run maximum | 0..127 mm + side / valid | 8 each | 7-bit mm + 1 flag bit | `terrain.h` |
| Hump detector phase and flags | 3 phases + 4 flags | 8 | one byte | `terrain.c` |
| Tilt capture progress, conditions | 0..50 samples + 2 flags; 2 conditions of 3 values | 8 + 4 | 6-bit count + 2 flags; 2 + 2 bits | `tilt.h` |
| Motion class debounce | class 0..7, candidate 0..7, count 0..3 | 8 | 3 + 3 + 2 bits in one byte | `motion_class.h` |
| Compass validity of the 5 heading slots | 5 flags | 5 | bit mask | `motion_class.h` |
| Lift latch | none / lifted / overturned + 3 timing flags | 5 | 2-bit condition + 3 flags in one byte | `lift.h` |

**Kept wider than the minimum, on purpose:**
- **Millisecond timestamps** stay 32-bit, so they wrap safely.
- **Reference vectors and filters** stay `float`, because the tilt geometry needs
  the precision. Only the results are packed.
- **The 4096-byte task stack** is a kernel allocation, not data.

### 2. RAM usage comparison

Measured with the Pico's compiler and the firmware's own flags (`-mcpu=cortex-m0plus
-mthumb -mfloat-abi=soft -ffreestanding -O2`; enums are 1 byte on this toolchain),
padding included (`logs/compact_representation_2026-11-25/sizes.txt`):

| Data | Before (bytes) | After (bytes) | Saved |
|---|---:|---:|---:|
| Raw GY-511 sample (acceleration + magnetic) | 12 | 10 | 2 |
| Tilt result | 20 | 4 | 16 |
| Lifecycle control | 12 | 1 | 11 |
| Hump results (last peak, run maximum) | 16 | 2 | 14 |
| Hump detector phase + flags | 5 | 1 | 4 |
| Tilt capture progress + conditions | 8 | 2 | 6 |
| Motion class debounce | 3 | 1 | 2 |
| Compass validity, 5 slots | 5 | 1 | 4 |
| Lift latch | 4 | 1 | 3 |
| **Total** | **85** | **23** | **62 (73 % smaller)** |

**Buddy 4 in today's production firmware** (MQTT image, `arm-none-eabi-size` on its
object files):
- **Static RAM:** 264 bytes. All of it is `.bss` in `terrain_task.c`, holding the
  whole Buddy 4 state: lifecycle 152, motion class 92, compass 12, and 8 bytes of
  pause/Start-gate flags and turn time. Buddy 4 has no `.data`.
- **Task stack:** 4096 bytes, unchanged.
- **Whole image:** `bss 111792` for every buddy and the kernel together. Buddy 4's
  264 bytes are about 0.2 % of it.

### 3. Message size comparison

Measured from the original message formats and today's frame definition
(`logs/compact_representation_2026-11-25/messages.txt`).

**Before (`b96a929`):** three JSON text messages every 0.25 s (4 Hz):

| Topic | Payload (bytes) | MQTT packet (bytes) |
|---|---:|---:|
| `robot/telemetry/imu_acc` `{"x":..,"y":..,"z":..}` | 19..31 | 46..58 |
| `robot/telemetry/imu_mag` (same format) | 19..31 | 46..58 |
| `robot/telemetry/terrain` `{"p":..,"r":..,"t":..,"m":..,"f":..,"s":..,"c":..,"v":..}` | 49..61 | 76..88 |
| **Per update** | **87..123** | **168..204** |
| **Per second** | 348..492 (2784..3936 bits/s) | 672..816 |

**After:** Buddy 4's 33 bits inside the shared 21-byte binary frame on `robot/frame`,
10 times a second.

| | Before | After |
|---|---|---|
| One update | 87..123 bytes in 3 messages | 33 bits (4.125 bytes) in 1 shared frame |
| Buddy 4 data per second | 2784..3936 bits | 330 bits |
| Update rate | 4 Hz | 10 Hz |

Result: **8.4× to 11.9× less data per second**, while updating 2.5× as often.

**Frame step v1 → v2** (`e564fd7` → `b7f0afb`, `docs/BUDDY4_FRAME_CHANGE_REQUEST.md`):
Buddy 4 went from 31 to 33 bits while gaining the motion class and turn rate:
- `M_CLASS` added (3 bits);
- the unused `T_HEADING`'s 9 bits reused for `TURN_RATE`;
- `HUMP_MM` trimmed from 8 to 7 bits.

The frame stayed 21 bytes.

### 4. Correctness and performance

**No loss of required functionality:**
- **Frame round-trip** (`buddy1_frame_checks`, team suite, 24/24 pass in
  `team_host.txt`): every Buddy 4 field at full range, i.e. `TURN_RATE` −256, −1, 0,
  1, 255, `HUMP_MM` 127, and every motion class including OVERTURNED. Also a golden
  frame shared with the dashboard decoder.
- **Packed sample decode** (`host.txt`): all 4096 values of every axis, for both
  acceleration and magnetic (12288 conversions each).
- **Tilt result** pack and unpack: 682 checks.
- **Packed lifecycle, hump, class and lift state**: exercised by the lifecycle
  (296), terrain task (14749) and Start-gate (4973) suites, all passing
  (`host.txt`).
- **On the robot**: the S4-H rows in "Results" below, run on firmware with these
  packed representations. The 24 November re-tests (<https://youtu.be/jCLuBsej2is>)
  show:
  - hump measured within ± 3 mm;
  - no false humps;
  - the lift stop working.
- **Precision vs requirement:**
  - hump height: 1 mm steps against the ± 3 mm acceptance;
  - pitch: 1 ° in the frame, 0.1 ° inside Buddy 4;
  - turn rate: 1 deg/s;
  - acceleration: 12-bit sensor resolution kept.

**No unacceptable timing degradation:**
- **Task period:** the terrain task still runs every 20 ms. `test_terrain_task`
  checks there is exactly one sensor sample per 20 ms release.
- **Cost of packing:** only shifts and masks, with no allocation and no loops over
  buffers. Decoding happens only where a value is used.
- **Messages:** sending 33 bits in the shared frame replaces formatting three JSON
  strings with `tm_sprintf` every 0.25 s.
- **Not measured:** the terrain task's CPU time per cycle on the robot. The robot
  videos show the functional result (humps, turns, lift stop) but not cycle timing;
  a serial log of a run would show any `[B4-STATE] ... RECOVERY` or fault lines.

### Evidence

`logs/compact_representation_2026-11-25/`:
- `sizes.txt`: per-structure before/after sizes, Buddy 4's firmware object sizes and
  the whole-image size;
- `messages.txt`: JSON vs frame sizes and rates;
- `host.txt`: Buddy 4 host suites (13 pass);
- `team_host.txt`: team host suite (24/24 pass).

Tools: `tests/subsystem4/memory_probe.c` (the before/after types) and
`tests/subsystem4/measure_compact.py`. Neither is part of the firmware or the host
test build.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-19 | S4-H03 | main after `610725f` | 2500910 | PASS | <https://youtu.be/b0VxlQ2r320> | Overturn stops the motors |
| 2026-11-19 | S4-H01 | main after `610725f` | 2500910 | PASS | <https://youtu.be/b0VxlQ2r320> | Lift while driving stops the motors; same video as S4-H03 |
| 2026-11-19 | S4-H08 | main after `610725f` | 2500910 | PASS | <https://youtu.be/O-3cIuu86ew> | Start of the full run: INIT until START, calibrating, WAITING, then drives |
| 2026-11-19 | S4-H11 | main after `610725f` | 2500910 | PASS | <https://youtu.be/O-3cIuu86ew> | 10 mm hump (ruler); reported peak within ±3 mm. Run 1 of 3 |
| 2026-11-19 | S4-H04 | main after `610725f` | 2500910 | PASS | <https://youtu.be/O-3cIuu86ew> | Full run over the hump with no LIFTED stop |
| 2026-11-19 | S4-H12 | main after `610725f` | 2500910 | PASS | <https://youtu.be/O-3cIuu86ew> | Motion classes and turn rate (yaw sign) shown on the console during the full run |
| 2026-11-19 | S4-H10 | main after `610725f` | 2500910 | PASS | <https://youtu.be/OaEkDdhVMTM> | Robot held by hand at various angles: pitch and uphill/downhill state; also shows the hump states (CLIMBING/DESCENDING) from sideways tilt |
| 2026-11-19 | S4-U02 | `fa36f67` | Claude Code for 2500910 | PASS | `logs/2026-11-19_host.txt` | `PASS: 2273 lift checks ...` |
| 2026-11-19 | S4-U03 | `fa36f67` | Claude Code for 2500910 | PASS | `logs/2026-11-19_host.txt` | All 13 suites PASS incl. `terrain task web Start gate checks`; `All Buddy 4 host suites passed.` |
| 2026-11-19 | S4-U04 | `fa36f67` | Claude Code for 2500910 | PASS | `logs/2026-11-19_start_gate.txt` | `motion_tests` 9633 checks, `line_imu_wait_checks`, `line_imu_wait_start_gate_checks`; `100% tests passed` |
| 2026-11-24 | S4-U03 | `aa3cb6d` + review fixes (uncommitted) | Claude Code for 2500910 | PASS | `logs/2026-11-24_host.txt` | All 13 suites PASS; motion class 632 checks incl. new "turn dropped when the heading goes invalid mid-turn" |
| 2026-11-19 | S4-H07 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=b0VxlQ2r320> | Overturn freezes hump recording until START; same video as S4-H01/H03 |
| 2026-11-19 | S4-H14 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=O-3cIuu86ew> | Highest hump; same full-run video as S4-H08/H11 |
| 2026-11-19 | S4-H15 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=O-3cIuu86ew> | No false hump; same full-run video as S4-H08/H11 |
| 2026-11-19 | S4-H16 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=O-3cIuu86ew> | Level reference captured after START and applied; same full-run video as S4-H08 |
| 2026-11-19 | S4-H09 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=b0VxlQ2r320> | Dashboard STOP pauses hump recording until START; same video as S4-H01/H03/H07 |
| 2026-11-18 | S4-H15 | firmware of the 18 Nov run (commit not recorded) | 2500910 | FAIL | `logs/2026-11-18_run.log`, `logs/2026-11-18_frames.csv` | Found on 2026-11-24 while analysing the dashboard logs, and seen on the robot: turns were recorded as humps (14 `hump completed` events). In the frames, false humps start (T_STATE 1 → 2) in the same frame a one-wheel pivot begins (one wheel 0 → about 46 mm/s, the other 0, turn about −23 deg/s, e.g. UPTIME_DS 202, 956) and end with the pivot. Cause: no gyroscope, so the sideways jolt of a starting/stopping turn reads as tilt (about 4–6 mm, threshold 5 mm). Route turns now spin on the spot (`fd5955a`), a larger jolt, so the 2026-11-19 PASS does not cover the current turns. Fix: hump recording paused while turning on the spot (S4-U03 below); re-test S4-H15 on the robot |
| 2026-11-24 | S4-U03 | `4033ff7` | Claude Code for 2500910 | PASS | `logs/2026-11-24_host_turn_pause.txt` | All 13 suites PASS. New: lifecycle 296 checks (was 284: a turn discards the hump in progress, run max kept, lift still detected); terrain task 14749 checks (was 13527: a 6 mm jolt during a spin, a one-wheel pivot and the 0.5 s after records no hump; the same height while steering forward records one) |
| 2026-11-24 | S4-H15 | `4033ff7` | 2500910 | PASS | <https://youtu.be/jCLuBsej2is> | Re-test after the 2026-11-18 FAIL: barcode turns, U-turns and line-search pivots on flat floor recorded no hump; **Highest hump** unchanged across the turns |
| 2026-11-24 | S4-H11 | `4033ff7` | 2500910 | PASS | <https://youtu.be/jCLuBsej2is> | Regression after the turn pause: a real hump under the right wheel is still detected and measured. 10 mm hump (ruler); reported peak within ± 3 mm |
| 2026-11-24 | S4-H14 | `4033ff7` | 2500910 | PASS | <https://youtu.be/jCLuBsej2is> | Regression: Highest hump kept for the rest of the run, including across the later turns; same run as S4-H11 |
| 2026-11-24 | S4-H04 | `4033ff7` | 2500910 | PASS | <https://youtu.be/jCLuBsej2is> | Regression: driving over humps never showed LIFTED; same run as S4-H11 |
| 2026-11-25 | S4-U03 | `133289f` (branch `buddy2/minimum-bit-review`) | Claude Code for 2500910 | PASS | `logs/compact_representation_2026-11-25/host.txt`, `team_host.txt` | Week 13 minimum-bit evidence: all 13 Buddy 4 suites pass (296 / 14749 checks) and the team host suite 24/24 (clang), including `buddy1_frame_checks` and both `line_imu_wait` tests. RAM and message sizes measured in `sizes.txt` / `messages.txt` (section "Minimum-bit representation" above) |
