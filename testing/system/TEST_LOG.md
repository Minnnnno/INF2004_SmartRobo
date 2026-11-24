# System — test log

Tests of the complete robot against its requirements, run on the final hardware
set-up with the integrated firmware. Run the full set before every demo and before
hand-in. **Draft for the team (2026-11-24):** each lead buddy checks their rows,
adjusts the steps to the real course, and records results below.

See `testing/README.md` for how to run tests and record results.

## Requirements

Sources: **Mission 1–5** = the Project Briefing's mission objectives (1 FOLLOW the
line, 2 DECODE navigation barcodes, 3 MEASURE: detect and measure humps, 4 AVOID:
profile and bypass obstacles, 5 REPORT: status and telemetry); **FR/NFR** = the
team's Week 6 Design Review.

| Mission | Requirements | System tests |
|---|---|---|
| 1 FOLLOW | FR1, FR4, FR5 | SYS-02, SYS-11 |
| 2 DECODE | FR2, FR3, FR17 | SYS-03, SYS-11 |
| 3 MEASURE | FR7, FR8, FR9, NFR9 | SYS-05, SYS-06, SYS-11 |
| 4 AVOID | FR10–FR14, NFR5, NFR10, NFR11 | SYS-04, SYS-11 |
| 5 REPORT | FR15–FR18, NFR6 | SYS-01, SYS-07, SYS-08, SYS-09, SYS-11 |
| Safety | Lift/overturn stop (11 Nov 2026 change); FR6 (stop) | SYS-09, SYS-10 |

## Test cases

Define a test here before running it. Test IDs never change. **Lead** = the buddy
who runs and records the test; everyone else checks their own subsystem's part.

| ID | What is tested | Requirement | Lead | Steps | Expected result (dashboard / console) |
|---|---|---|---|---|---|
| SYS-01 | Safe start-up and remote START | FR15, FR17, NFR9 | B1 (B4 supports) | Power up on the course with the dashboard open; wait 30 s; press START; keep still ~2 s | Before START: robot shown online, Motion **STOPPED (held)**, wheels 0 / 0, IMU card **INIT**. After START: IMU card **WAITING** with Pitch ~0° (level reference captured), then Motion **DRIVING** |
| SYS-02 | Line following over the full course | Mission 1; FR1, FR5 | B3 | START; follow the course from start to end, no barcodes or obstacles | Stays on the line to the end without help; no FAULT |
| SYS-03 | Barcode navigation from a dashboard route | Mission 2; FR2, FR3, FR17 | B3 | Send a route from the dashboard (e.g. `L=B R=C S=D U=A`); place the codes before the junctions; START | Route acknowledged (`robot/route/ack`); every code read as the right letter (never a wrong letter) and its action (left / right / straight / U-turn) done at the junction after it; barcode events on the dashboard |
| SYS-04 | Obstacle detection, profiling and bypass | Mission 4; FR10–FR14, NFR5, NFR10, NFR11 | B5 | Box of measured width at a documented position on the line; START; 5 runs with different left/right offsets | Every run: stops before contact (sensor ~150 mm), reports width and side, bypasses and reacquires the line (or stays stopped if no safe path). Record per run: detected, width error (NFR11 ± 1 cm), side correct, line reacquired, bypass time. Briefing A-grade: 100 % detection, width error ≤ 10 %, 100 % direction, 100 % reacquisition, bypass < 3 s |
| SYS-05 | Hump detection and highest hump peak | Mission 3; FR7, FR8 | **B4** | A hump of ruler-measured height beside the line under the **right** wheel; full line-following run | IMU card **WAITING → CLIMBING → DESCENDING → WAITING** on the hump; **Highest hump** = the ruler height ± 3 mm and kept for the rest of the run; never **LIFTED**; console `[TERRAIN] Hump completed: peak=X mm under RIGHT wheel` |
| SYS-06 | Motion classes and turn rate on the real course | FR9 (sudden impact replaced by the lift/overturn stop) | **B4** | One run from standstill: a straight, a barcode turn or sharp curve, a hump, then STOP | *Motion class* goes STATIONARY → ACCELERATING → MOVING → TURNING (on the turn) → CLIMBING / DESCENDING (on the hump) → STATIONARY; *Turn rate* + left / − right; no TURNING flicker on the straight; console `[MOTION]` and `[TURN] left/right N deg` lines |
| SYS-07 | Telemetry content | Mission 5; FR16 | B1 (all) | During SYS-03 to SYS-05, check each card on the dashboard | Shows motor speed, encoder counts, distance, line status, barcode events, hump height (Highest hump, Pitch, IMU state, Motion class, Turn rate) and obstacle scan results; each buddy confirms their own values against the console |
| SYS-08 | Connection loss and reconnect | FR18, NFR6 | B1 | Mid-run switch the hotspot off for 10 s, then on | Robot keeps line following (no stop, no FAULT); dashboard shows offline, then reconnects by itself and data resumes |
| SYS-09 | Dashboard STOP / START mid-run | FR6, FR17 | B2 (B4 supports) | Drive over a hump; press STOP; wait 5 s; press START | Wheels stop and stay **held**; nothing moves until START; START resumes line following. Buddy 4: **Highest hump** unchanged while stopped (`terrain=PAUSED`), next hump recorded after START |
| SYS-10 | End of run: lift and overturn stop the robot | Lift/overturn stop (11 Nov 2026 change) | **B4** (B2 supports) | Mid-run lift the robot briskly, then turn it over | Within ~0.3 s wheels 0 / 0; IMU card **LIFTED (motors stopped)**, then *Motion class* **OVERTURNED (motors stopped)**; Motion **STOPPED (held)**; **Highest hump** frozen at its last value (`terrain=FROZEN`) |
| SYS-11 | Full mission (Week 10 dress rehearsal) | Missions 1–5 | All | Route sent; START; course with barcodes, an obstacle and humps; lift to end | Every SYS-01 to SYS-10 expectation met in one continuous run; final Highest hump and events on the dashboard |

*Method:* all system tests are **manual**: run on the robot, with a video of the
robot and the dashboard (and the serial console where needed) as evidence.

## Running the system tests

1. **Commit first** and note `git rev-parse --short HEAD`; every buddy's changes
   must be in that commit.
2. **Build the MQTT production image** (VS Code "Compile Project", with
   `config/wifi_secrets.h` present) and flash it with motor power disconnected.
3. **Start the broker and dashboard**, as in `testing/subsystem4/TEST_LOG.md`
   ("Running the video tests"), saving the dashboard log to
   `logs/<date>_SYS-xx_dashboard.log` and the serial log to `logs/<date>_SYS-xx.txt`.
4. **Every run:** power up, place the robot level on the line, press START and keep
   it still ~2 s while Buddy 4 captures the level reference. A new run is a
   power-cycle (this also clears the Highest hump).
5. **Video:** say the test ID and commit, show the set-up (ruler on the hump,
   obstacle size and position), do the run, end on the dashboard value that proves
   the result. Unlisted YouTube link in the results row.

## Buddy 4 checklist

What Buddy 4 brings to the system tests (details in
`testing/subsystem4/TEST_LOG.md`):

- **SYS-05, SYS-06, SYS-10 (lead):** measure the humps with a ruler before the run;
  keep the IMU & terrain card and the serial console in the video.
- **SYS-01 (support):** confirm the level reference is captured only after START
  (`[B4] START: capturing level reference`, card INIT → WAITING).
- **SYS-09 (support):** confirm hump recording pauses during STOP.
- **SYS-11:** end the run by lifting the robot (the demo ending); Highest hump must
  stay frozen.
- Already verified at subsystem level: S4-H01, H03, H04, H07 to H12, H14 to H16;
  at integration level: INT-03, INT-04, INT-05.

## Other logs in this folder

The `com4_2026-11-0*.log` files are raw serial captures from Buddy 2's
3–4 November debugging sessions, cited by `docs/DETOUR_FREEZE_2026-11-04.md`,
`docs/ENCODER_POLARITY_2026-11-04.md`, `docs/INTERMITTENT_STOP_INVESTIGATION.md`
and `docs/MEMORY_DEBUG_2026-11-04.md`. They are not system test results.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-19 | SYS-05 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=O-3cIuu86ew> | 10 mm hump (ruler), reported within ±3 mm; same full-run video as S4-H11/H14 |
| 2026-11-19 | SYS-06 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=O-3cIuu86ew> | Motion classes and turn rate during the full run; same video as S4-H12 |
| 2026-11-19 | SYS-10 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=b0VxlQ2r320> | Lift and overturn stop the motors, Highest hump frozen; same video as S4-H01/H03/H07 |
| 2026-11-19 | SYS-01 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=O-3cIuu86ew> | Start of the full run: held and INIT until START, level reference captured (WAITING), then DRIVING; same video as S4-H08. B1 sign-off pending |
