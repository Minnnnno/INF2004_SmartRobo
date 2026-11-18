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
| R4-1 | IMU integration: GY-511 configured and read on I2C1 | Buddy 4 tasks | S4-U02 | S4-H07 |
| R4-2 | Sensor calibration: level reference captured when START is pressed (robot does not drive before it) | Buddy 4 tasks | S4-U02, S4-U03 | S4-H07, S4-H11 |
| R4-3 | Tilt detection: pitch and uphill/downhill state | Buddy 4 tasks | S4-U02 | S4-H08 |
| R4-4 | Detect humps along the route using IMU data | Mission req 4; Buddy 4 tasks | S4-U02 | S4-H09 |
| R4-5 | Estimate hump height and report the **highest hump peak** of the run | Mission req 5; Buddy 4 tasks | S4-U02 | S4-H09 |
| R4-6 | Motion classification with stable events (stationary, accelerating, turning, climbing, descending) | Buddy 4 tasks, performance objectives | S4-U02 | S4-H10 |
| R4-7 | Turn-rate monitoring | Buddy 4 tasks | S4-U02 | S4-H10 |
| R4-8 | Motion event reporting (console and telemetry) | Buddy 4 tasks | S4-U02 | S4-H09, S4-H10 |
| R4-9 | Lifted or overturned: stop the motors immediately | CR 2026-11-11 | S4-U01 | S4-H01, S4-H02, S4-H03, S4-H05 |
| R4-10 | Humps and normal tilts must not trigger the lift stop | CR 2026-11-11 (robustness) | S4-U01 | S4-H04, S4-H06 |

## Script tests (laptop, no robot)

S4-U01 and S4-U02 come from one command; S4-U03 from the team host suite:

```sh
python tests/subsystem4/run_tests.py > testing/subsystem4/logs/<date>_host.txt
cmake -S . -B build-host-cmake -G Ninja -DROBO_HOST_TESTS=ON
cmake --build build-host-cmake
ctest --test-dir build-host-cmake -R motion_tests --output-on-failure > testing/subsystem4/logs/<date>_start_gate.txt
```

| ID | What is tested | Requirement | Expected result (in the log) |
|---|---|---|---|
| S4-U01 | Lift/overturn detection: thresholds, 200 ms hold, clearing, hump profiles never trigger it, sensor scale, gaps | R4-9, R4-10 | `PASS: <n> lift checks (overturn, lift, hold, clear, humps, scale, gaps, NULL)` |
| S4-U02 | The other Buddy 4 suites: GY-511 configuration and axis conversion, tilt geometry, hump height/peak/run max, motion classes and yaw, lifecycle/recovery (incl. calibration held until START), terrain task (snapshot, hump event, lift stop), the terrain task again built with `ROBO_WAIT_FOR_START` (INIT for 30 s with no capture or fault, calibrated ~2 s after one START pulse), I2C receive | R4-1 to R4-8 | A `PASS:` line for each suite (incl. `tilt geometry`, `height hump ... run max`, `motion class checks`, `terrain task checks ...` and `terrain task checks (..., calibration on START)`), ending `All Buddy 4 host suites passed.` |
| S4-U03 | Vehicle Controller start gate: robot held at boot; first START kept pending while Buddy 4 is INIT; released the cycle Buddy 4 reports WAITING; STOP cancels it; IMU FAULT or 15 s silence releases; LIFTED cancels; later START/STOP normal | R4-2 | `motion_tests` passes (`PASS: <n> checks ...`) and `100% tests passed` |

## Video tests (robot + dashboard)

"Dashboard" = `tools/web_dashboard.py` in the browser: the **IMU & terrain**
card (state, Pitch, Highest hump), the **Motion** card (state, wheel speed,
heading), the alerts and the event log. The last column lists only what the
dashboard cannot show yet; read those lines from the serial console, kept in
the same video frame.

| ID | What is tested | Requirement | Steps | Expected on the dashboard | Console only (not on the dashboard) |
|---|---|---|---|---|---|
| S4-H01 | Lift while driving stops the motors | R4-9 | Line-following run on the floor; pick the robot straight up, briskly, mid-run | Within ~0.3 s the wheels stop: IMU & terrain card **LIFTED (motors stopped)**, alert *Robot lifted (Buddy 4)*, Motion **STOPPED (held)**, wheel speed **0 / 0**; event log `robot LIFTED: Buddy 4 stopped the motors (end of run)` | `[LIFT] LIFTED ... -> motors stopped` |
| S4-H02 | Lift while stationary | R4-9 | Press STOP on the dashboard; pick the robot straight up, briskly | IMU & terrain **LIFTED (motors stopped)**; Motion stays **STOPPED (held)**, wheel speed 0 / 0 | `[LIFT] LIFTED` |
| S4-H03 | Overturn stops the motors | R4-9 | Wheels driving on a stand; roll the robot onto its side, then upside down | Within ~0.3 s: Motion **STOPPED (held)**, wheel speed **0 / 0**; Pitch **not valid** with alert *No valid pitch from the IMU* while overturned | `[LIFT] OVERTURNED ... -> motors stopped`, `[MOTION] ... -> OVERTURNED` (the word OVERTURNED reaches the dashboard only with frame layout v2) |
| S4-H04 | Humps never trigger the lift stop | R4-10 | Full run over the course humps under the **right** wheel, normal speed | Never **LIFTED**, no *Robot lifted* alert; Motion stays **DRIVING**; the card goes CLIMBING → DESCENDING → WAITING on each hump and **Highest hump** updates | No `[LIFT]` line |
| S4-H05 | START in the air is refused; set down, then START resumes | R4-9 | After S4-H01, still holding it up: press START. Then set it down level and still for 1 s and press START | In the air: card stays **LIFTED**, Motion back to **STOPPED (held)** at once, wheels do not turn. Set down: card returns to **WAITING**; START → Motion **DRIVING**, wheel speeds non-zero | `[LIFT] still LIFTED: motors stopped again`, then `[LIFT] cleared ...` |
| S4-H06 | Tilting to one side does not stop the robot | R4-10 | Wheels driving on a stand; tilt the robot 40–50° to one side by hand for 2 s, then set it level | Never **LIFTED**; Motion stays **DRIVING**, wheel speeds non-zero; **Highest hump** unchanged | No `[LIFT]` line; roll angle in `[TILT] ... roll=` |
| S4-H07 | IMU captures the level reference when START is pressed | R4-1, R4-2 | Robot on a level floor, still. Power up and wait until the dashboard is live, then press START and do not touch the robot | Before START: card **INIT**, Motion **STOPPED (held)**. After START: card **INIT → WAITING** in about 2 s; Pitch **valid, 0° ± 1°**; Highest hump **none yet**; no *IMU fault* or *No valid pitch* alert | `[B4] INIT: waiting for START before calibrating; robot held`, then `[B4] START pressed: calibrating ...`, `[B4] INIT: capturing level reference n/50`, `[B4-STATE] INIT -> MONITORING` |
| S4-H08 | Tilt: uphill/downhill state and pitch | R4-3 | Motors not driving. Moving slowly (not a lift), hold each pose still for 2 s: 1. front raised ~15° (uphill), 2. front lowered ~15° (downhill), 3. set it down level | Pose 1: **Pitch** positive, about +15°. Pose 2: **Pitch** negative, about −15°. Pose 3: **Pitch** back to about 0°. Never **LIFTED** | Uphill/downhill state in the `[TILT]` line: `FRONT UP` (pose 1), `FRONT DOWN` (pose 2), `LEVEL` (pose 3) |
| S4-H09 | Hump detected and its height measured (right wheel) | R4-4, R4-5, R4-8 | 1. Measure a hump with a ruler (e.g. 20 mm). 2. Place it beside the line so only the **right** wheel rides over it. 3. Line-following run over it. 4. Repeat 3 times, power-cycling between runs; write each reading in *Notes* | Card **WAITING → CLIMBING → DESCENDING → WAITING** (event log `terrain WAITING -> CLIMBING` …); event `hump completed`; **Highest hump = ruler height ± 3 mm**; never **LIFTED** | Wheel side: `[TERRAIN] Hump completed: peak=X mm under RIGHT wheel` |
| S4-H10 | Motion events are classified and stable; turn rate reported | R4-6, R4-7, R4-8 | One line-following run from standstill: a straight, a sharp curve or barcode turn, a hump, then stop (or lift to end) | Supporting only: Motion **DRIVING**/**STOPPED**, Heading changes by the turn angle, card CLIMBING/DESCENDING on the hump. *Motion class* and *Turn rate* show "needs frame layout v2" until Buddy 1 applies `docs/BUDDY4_FRAME_CHANGE_REQUEST.md` | **The result is on the console:** `[MOTION]` lines `STATIONARY -> ACCELERATING`, `-> MOVING`, `MOVING -> TURNING` on the turn (`yaw=` sign = direction, + left), back to `MOVING`, `-> CLIMBING`, `-> DESCENDING` on the hump, `-> STATIONARY` at the end; no `TURNING` flicker on the straight |
| S4-H11 | Stays in INIT until START; calibrates, then drives | R4-2 | Robot on the line, level. **Run A:** power up, wait 30 s, press START, do not touch it. **Run B:** power up, press START, then press STOP within 1 s; wait 5 s; press START again | Run A: for 30 s the card stays **INIT**, Motion **STOPPED (held)**, wheel speed **0 / 0** (no IMU fault). After START: still STOPPED while the card shows INIT (~2 s), then the card shows **WAITING** with a valid **Pitch**, and only then Motion **DRIVING**. Run B: after STOP the robot stays **STOPPED (held)** even when the card reaches WAITING; the second START drives at once | `[B4] INIT: waiting for START before calibrating; robot held` every 0.5 s; `[B4] START pressed: calibrating the level reference ...`; `[B4-STATE] INIT -> MONITORING` about 2 s later; the wheels do not move before it. (`line_imu_wait settled=0 waited_ms=10000` at boot is expected: the line task stops waiting for the IMU, but the robot is held) |

## Running the video tests

1. **Commit first.** Note `git rev-parse --short HEAD`; it goes in the video
   and the results row.
2. **Build the firmware** in its own build directory (e.g. `build-s4-video`).
   Use the production options from CLAUDE.md plus `-DROBO_LINE_STEERING=ON
   -DWIFI=cyw43 -DNET=lwip -DMQTT=ON` (needs `config/wifi_secrets.h`). This
   turns on `ROBO_WAIT_FOR_START`: the build ID ends in `-start`, the robot
   boots held, and **START begins every run** (calibration, then driving).
   Flash with motor power disconnected.
3. **Start the broker and dashboard on the laptop:**
   ```sh
   mosquitto -c tools/mosquitto_robot.conf -v
   python tools/web_dashboard.py --host 127.0.0.1 --log testing/subsystem4/logs/<date>_S4-Hxx_dashboard.log
   ```
   Open the address it prints (port 8080) and check it shows live frames
   before testing.
4. **Start the serial log** too (PuTTY *Session → Logging → All session
   output*, `logs/<date>_S4-Hxx.txt`) for the console-only items.
5. **Power up level and still, then press START** for every run. The tilt
   reference is captured when START is pressed, and a new run is a
   power-cycle.
6. **Lift briskly** in S4-H01, H02 and H05. A slow lift is only caught once
   the robot passes 60° (OVERTURNED).

**Video format** (see `videos/README.md`): one test per video, 15–60 s,
landscape, unlisted YouTube.

1. Start: say or show the test ID and name, and the commit hash.
2. Show the setup: ruler on the hump, inclinometer reading, stand.
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
| 2026-11-19 | S4-H03 | `0b11baf` | 2500910 | PASS | <https://youtu.be/b0VxlQ2r320> | Overturn stops the motors; recorded before the START-gate change |
| 2026-11-19 | S4-H01 | `0b11baf` | 2500910 | PASS | <https://youtu.be/b0VxlQ2r320> | Lift while driving stops the motors; same video as S4-H03 |
| 2026-11-19 | S4-H07 | `0b11baf` | 2500910 | PASS | <https://youtu.be/O-3cIuu86ew> | INIT → calibrating → WAITING before driving. Firmware before the START gate (calibrates at boot, not on START): re-run on the START-gate build |
| 2026-11-19 | S4-H09 | `0b11baf` | 2500910 | PASS | <https://youtu.be/O-3cIuu86ew> | 10 mm hump (ruler); reported peak within ±3 mm. Run 1 of 3 |
| 2026-11-19 | S4-H04 | `0b11baf` | 2500910 | PASS | <https://youtu.be/O-3cIuu86ew> | Full run over the hump with no LIFTED stop |
| 2026-11-19 | S4-H10 | `0b11baf` | 2500910 | PASS | <https://youtu.be/O-3cIuu86ew> | Motion classes and turn rate (yaw sign) shown on the console during the full run |
| 2026-11-19 | S4-H11 | `0b11baf` | 2500910 | PASS | <https://youtu.be/O-3cIuu86ew> | Start of the full run: INIT until START, calibrating, WAITING, then drives |
| 2026-11-19 | S4-H08 | `0b11baf` | 2500910 | PASS | <https://youtu.be/OaEkDdhVMTM> | Robot held by hand at various angles: pitch and uphill/downhill state; also shows the hump states (CLIMBING/DESCENDING) from sideways tilt |
| 2026-11-19 | S4-U01 | `8922e0a` | Claude Code for 2500910 | PASS | `logs/2026-11-19_host.txt` | `PASS: 2273 lift checks ...` |
| 2026-11-19 | S4-U02 | `8922e0a` | Claude Code for 2500910 | PASS | `logs/2026-11-19_host.txt` | All 13 suites PASS incl. `terrain task checks (..., calibration on START)`; `All Buddy 4 host suites passed.` |
| 2026-11-19 | S4-U03 | `8922e0a` | Claude Code for 2500910 | PASS | `logs/2026-11-19_start_gate.txt` | `motion_tests`: `PASS: 8003 checks`, `100% tests passed` |
