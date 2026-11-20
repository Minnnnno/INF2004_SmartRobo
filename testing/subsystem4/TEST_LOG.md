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

| Req | Requirement | Source | Script test | Video test |
|---|---|---|---|---|
| R4-1 | IMU integration: GY-511 configured and read on I2C1 | Buddy 4 tasks | S4-U03 | S4-H08 |
| R4-2 | Sensor calibration: level reference captured after the dashboard START (robot does not drive before it) | Buddy 4 tasks | S4-U03, S4-U04 | S4-H08 |
| R4-3 | Tilt detection: pitch and uphill/downhill state | Buddy 4 tasks | S4-U03 | S4-H10 |
| R4-4 | Detect humps along the route using IMU data | Mission req 4; Buddy 4 tasks | S4-U03 | S4-H11 |
| R4-5 | Estimate hump height and report the **highest hump peak** of the run | Mission req 5; Buddy 4 tasks | S4-U03 | S4-H11, S4-H07, S4-H09 |
| R4-6 | Motion classification with stable events (stationary, accelerating, turning, climbing, descending) | Buddy 4 tasks, performance objectives | S4-U03 | S4-H12 |
| R4-7 | Turn-rate monitoring | Buddy 4 tasks | S4-U03 | S4-H12 |
| R4-8 | Motion event reporting (console and telemetry) | Buddy 4 tasks | S4-U03 | S4-H11, S4-H12 |
| R4-9 | Lifted or overturned: stop the motors immediately | CR 2026-11-11 | S4-U02 | S4-H01, S4-H02, S4-H03, S4-H05 |
| R4-10 | Humps and normal tilts must not trigger the lift stop | CR 2026-11-11 (robustness) | S4-U02 | S4-H04, S4-H06 |
| R4-11 | Humps are recorded only during the run: frozen after an overturn, paused while stopped, resumed by START | CR 2026-11-11 (run ends lifted and turned over); dashboard START/STOP | S4-U03 | S4-H07, S4-H09 |

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
| S4-U03 | The other Buddy 4 suites: GY-511 configuration and axis conversion, tilt geometry, hump height/peak/run max, motion classes and yaw, lifecycle/recovery, terrain task (snapshot, hump event, lift stop, hump freeze), terrain task with the web Start gate (wait, START, capture, hump, STOP/START), I2C receive | R4-1 to R4-8, R4-11 | A `PASS:` line for each suite (incl. `terrain task checks (..., hump freeze)` and `terrain task web Start gate checks`), ending `All Buddy 4 host suites passed.` |
| S4-U04 | Start gate in the team suite: Vehicle Controller boot hold and START release (`motion_tests`), line task waits for Buddy 4 after START (`line_imu_wait_checks`, `line_imu_wait_start_gate_checks`) | R4-2 | `PASS:` line for each of the 3 tests and `100% tests passed, 0 tests failed out of 3` |

## Video tests (robot + dashboard)

"Dashboard" = `tools/web_dashboard.py` in the browser: the **IMU & terrain**
card (state, Pitch, Highest hump), the **Motion** card (state, wheel speed,
heading), the alerts and the event log. The last column lists only what the
dashboard cannot show yet; read those lines from the serial console, kept in
the same video frame.

| ID | What is tested | Requirement | Steps | Expected on the dashboard | Console only (not on the dashboard) |
|---|---|---|---|---|---|
| S4-H01 | Lift while driving stops the motors | R4-9 | Line-following run on the floor; pick the robot straight up, briskly, mid-run | Within ~0.3 s the wheels stop: IMU & terrain card **LIFTED (motors stopped)**, alert *Robot lifted (Buddy 4)*, Motion **STOPPED (held)**, wheel speed **0 / 0**; event log `robot LIFTED: Buddy 4 stopped the motors (end of run)` | `[LIFT] LIFTED ... -> motors stopped` |
| S4-H02 | Lift while stationary | R4-9 | After a run (START, drive), press STOP on the dashboard; pick the robot straight up, briskly | IMU & terrain **LIFTED (motors stopped)**; Motion stays **STOPPED (held)**, wheel speed 0 / 0 | `[LIFT] LIFTED` |
| S4-H03 | Overturn stops the motors | R4-9 | Wheels driving on a stand; roll the robot onto its side, then upside down | Within ~0.3 s: Motion **STOPPED (held)**, wheel speed **0 / 0**; Pitch **not valid** with alert *No valid pitch from the IMU* while overturned | `[LIFT] OVERTURNED ... -> motors stopped`, `[MOTION] ... -> OVERTURNED` (the word OVERTURNED reaches the dashboard only with frame layout v2) |
| S4-H04 | Humps never trigger the lift stop | R4-10 | Full run over the course humps under the **right** wheel, normal speed | Never **LIFTED**, no *Robot lifted* alert; Motion stays **DRIVING**; the card goes CLIMBING → DESCENDING → WAITING on each hump and **Highest hump** updates | No `[LIFT]` line |
| S4-H05 | START in the air is refused; set down, then START resumes | R4-9 | After S4-H01, hold it up **tilted more than 20°** (held level and still, LIFTED clears by design) and press START. Then set it down level and still for 1 s and press START | In the air: card stays **LIFTED**, Motion back to **STOPPED (held)** at once, wheels do not turn. Set down: card returns to **WAITING**; START → Motion **DRIVING**, wheel speeds non-zero | `[LIFT] still LIFTED: motors stopped again`, then `[LIFT] cleared ...` |
| S4-H06 | Tilting to one side does not stop the robot | R4-10 | Wheels driving on a stand; tilt the robot 40–50° to one side by hand for 2 s, then set it level | Never **LIFTED**; Motion stays **DRIVING**, wheel speeds non-zero; **Highest hump** unchanged | No `[LIFT]` line; roll angle in `[TILT] ... roll=` |
| S4-H07 | Overturn freezes hump recording until START (demo end) | R4-11, R4-5 | Complete one hump and note **Highest hump**; lift the robot and turn it over; hold it level in the air until LIFTED clears; set it down at an angle and rock it side to side; press START; drive over one more hump | **Highest hump** unchanged from the overturn until START; after START the next hump is recorded and **Highest hump** updates if higher | `[TERRAIN] hump recording frozen (OVERTURNED): run max kept at N mm until START`, `[TILT] ... terrain=FROZEN`, no `Hump completed` until START; then `[TERRAIN] hump recording resumed (START)` |
| S4-H08 | Level reference captured only after the dashboard START; the robot drives only after it | R4-1, R4-2 | MQTT image, dashboard open. Power up, carry the robot around for 30 s, place it level on the course, press START, keep it still | Before START: card **INIT**, Motion **STOPPED (held)**, wheel speed **0 / 0**, no *IMU fault* alert. After START: wheels still while the card shows INIT (~2 s), then card **WAITING** with Pitch **valid, ~0°**, then Motion **DRIVING** (line following) | `[B4] waiting for dashboard START before capturing the level reference ...`, `[B4] INIT: waiting for dashboard START ...` every 0.5 s; after START `[B4] START: capturing level reference ...`, `[B4-STATE] INIT -> MONITORING`, `line_imu_wait settled=1` |
| S4-H09 | Dashboard STOP pauses hump recording until START | R4-11, R4-5 | Drive over one hump and note **Highest hump**; press STOP; raise one side of the robot ~3 cm by hand and lower it again; press START; drive over one more hump | **Highest hump** unchanged while stopped; after START the next hump is recorded | `[TERRAIN] hump recording paused (robot stopped): run max kept at N mm until START`, `[TILT] ... terrain=PAUSED`, no `Hump completed` while stopped; on START `[TERRAIN] hump recording resumed (START)` |
| S4-H10 | Tilt: uphill/downhill state and pitch | R4-3 | Motors not driving. Moving slowly (not a lift), hold each pose still for 2 s: 1. front raised ~15° (uphill), 2. front lowered ~15° (downhill), 3. set it down level | Pose 1: **Pitch** positive, about +15°. Pose 2: **Pitch** negative, about −15°. Pose 3: **Pitch** back to about 0°. Never **LIFTED** | Uphill/downhill state in the `[TILT]` line: `FRONT UP` (pose 1), `FRONT DOWN` (pose 2), `LEVEL` (pose 3) |
| S4-H11 | Hump detected and its height measured (right wheel) | R4-4, R4-5, R4-8 | 1. Measure a hump with a ruler (e.g. 20 mm). 2. Place it beside the line so only the **right** wheel rides over it. 3. Line-following run over it. 4. Repeat 3 times, power-cycling between runs; write each reading in *Notes* | Card **WAITING → CLIMBING → DESCENDING → WAITING** (event log `terrain WAITING -> CLIMBING` …); event `hump completed`; **Highest hump = ruler height ± 3 mm**; never **LIFTED** | Wheel side: `[TERRAIN] Hump completed: peak=X mm under RIGHT wheel` |
| S4-H12 | Motion events are classified and stable; turn rate reported | R4-6, R4-7, R4-8 | One line-following run from standstill: a straight, a sharp curve or barcode turn, a hump, then stop (or lift to end) | Supporting only: Motion **DRIVING**/**STOPPED**, Heading changes by the turn angle, card CLIMBING/DESCENDING on the hump. *Motion class* and *Turn rate* show "needs frame layout v2" until Buddy 1 applies `docs/BUDDY4_FRAME_CHANGE_REQUEST.md` | **The result is on the console:** `[MOTION]` lines `STATIONARY -> ACCELERATING`, `-> MOVING`, `MOVING -> TURNING` on the turn (`yaw=` deg/s, + left, − right), back to `MOVING`, `-> CLIMBING`, `-> DESCENDING` on the hump, `-> STATIONARY` at the end; no `TURNING` flicker on the straight |

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
6. **Lift briskly** in S4-H01, H02 and H05. A slow lift is only caught once
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
console-only items. For S4-H04 and H06, the evidence is that nothing
happens: show the wheels still turning and the card never showing LIFTED.

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
