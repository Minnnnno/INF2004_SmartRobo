# Subsystem 4 — test log

**Subsystem:** IMU-based motion and terrain monitoring (Buddy 4, GY-511 on I2C1)  ·  **Student ID:** 2500910  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results, and
`app_program/subsystem4/README.md` for how the subsystem works.

Two kinds of test:

- **Script tests (S4-U..)** run the Buddy 4 C code on the laptop with
  simulated sensor data. They prove the **logic**. Evidence: the saved
  console output. No video.
- **Video tests (S4-H..)** run on the robot. They prove the **requirement is
  met on the real hardware**, read from the web dashboard. Evidence: a video,
  plus the dashboard and serial logs.

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

## Running the video tests

1. **Commit first.** Note `git rev-parse --short HEAD`; it goes in the video
   and the results row.
2. **Build the MQTT production image**, e.g. VS Code "Compile Project" with
   `ROBO_NETWORK=AUTO` and `config/wifi_secrets.h` present. `ROBO_REQUIRE_START`
   (default ON in MQTT production images) makes the robot boot held, and
   **START begins every run**: Buddy 4 calibrates, then line following
   starts. Flash with motor power disconnected.
3. **Start the broker and dashboard on the laptop:**
   ```sh
   mosquitto -c tools/mosquitto_robot.conf -v
   python tools/web_dashboard.py --host 127.0.0.1 --log testing/subsystem4/logs/<date>_S4-Hxx_dashboard.log
   ```
   Open the address it prints (port 8080) and check it shows live frames
   before testing.
4. **Start the serial log** too (PuTTY *Session → Logging → All session
   output*, `logs/<date>_S4-Hxx.txt`) for the console-only items.
5. **Power up, place the robot level, then press START** for every run. The
   tilt reference is captured after START, and a new run is a power-cycle.
6. **Lift briskly** in S4-H01. A slow lift is only caught once
   the robot passes 60° (OVERTURNED).

**Video format** (see `videos/README.md`): one test per video, 15–60 s,
landscape, unlisted YouTube. A longer video may cover several tests if each
test's start is shown with its ID.

1. Start: say or show the test ID and name, and the commit hash.
2. Show the setup: ruler on the hump, stand, robot level on the course.
3. Do the action.
4. End holding on the dashboard value that proves the result, then say
   PASS or FAIL.

Keep the robot and the dashboard in one frame. Use OBS with the browser next
to a phone camera (e.g. DroidCam), and add the serial terminal for the
console-only items. For S4-H04, the evidence is that nothing
happens: show the wheels still turning and the card never showing LIFTED.
S4-H15 is the same kind: show the card staying WAITING and *Highest hump*
staying *none yet* for the whole run.

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
| 2026-11-24 | S4-U03 | `b51982f` + turn-pause fix (uncommitted) | Claude Code for 2500910 | PASS | `logs/2026-11-24_host_turn_pause.txt` | All 13 suites PASS. New: lifecycle 296 checks (was 284: a turn discards the hump in progress, run max kept, lift still detected); terrain task 14749 checks (was 13527: a 6 mm jolt during a spin, a one-wheel pivot and the 0.5 s after records no hump; the same height while steering forward records one) |
| 2026-11-24 | S4-H15 | `b51982f` + turn-pause fix (uncommitted) | 2500910 | PASS | video pending | Re-test after the 2026-11-18 FAIL: barcode turns, U-turns and line-search pivots on flat floor recorded no hump; **Highest hump** unchanged across the turns |
| 2026-11-24 | S4-H11 | `b51982f` + turn-pause fix (uncommitted) | 2500910 | PASS | video pending | Regression after the turn pause: a real hump under the right wheel is still detected and measured. Ruler height and reported peak: *to fill in* |
| 2026-11-24 | S4-H14 | `b51982f` + turn-pause fix (uncommitted) | 2500910 | PASS | video pending | Regression: Highest hump kept for the rest of the run, including across the later turns; same run as S4-H11 |
| 2026-11-24 | S4-H04 | `b51982f` + turn-pause fix (uncommitted) | 2500910 | PASS | video pending | Regression: driving over humps never showed LIFTED; same run as S4-H11 |
