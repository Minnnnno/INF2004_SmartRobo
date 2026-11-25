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
| SYS-02 | Line following over the full course | Mission 1; FR1, FR5 | B3 | START; follow the course from start to end, no obstacles (barcodes may be on the course) | Stays on the line to the end without help; no FAULT |
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
3. **Start the broker and dashboard**, as in `docs/DASHBOARD_SETUP.md`, saving the
   dashboard log to
   `logs/<date>_SYS-xx_dashboard.log` and the serial log to `logs/<date>_SYS-xx.txt`.
4. **Every run:** power up, place the robot level on the line, press START and keep
   it still ~2 s while Buddy 4 captures the level reference. A new run is a
   power-cycle (this also clears the Highest hump).
5. **Video:** say the test ID and commit, show the set-up (ruler on the hump,
   obstacle size and position), do the run, end on the dashboard value that proves
   the result. Unlisted YouTube link in the results row.

## Buddy 2 checklist

Buddy 2's subsystem procedures are in `testing/subsystem2/TEST_LOG.md`;
cross-buddy motion interfaces are INT-06 to INT-10 in
`testing/integration/TEST_LOG.md`. This checklist adds responsibilities, not
completed results. Buddy 2 supplied S2-H01 to S2-H04 videos on 2026-11-24,
reporting one passing subsystem trial each. The links below are supporting
evidence; additional system coverage remains to be confirmed.

- **SYS-09 (lead, B1/B4 support):** show the robot driving over the hump, dashboard STOP, physical wheel rest and held state, a 5 s wait with no restart, then deliberate START and resumed line following. B4 confirms Highest hump stays unchanged while stopped and a subsequent hump can be recorded. Record any fault; START is not a fault reset.
- **SYS-01 (support B1/B4):** confirm wheels remain stopped during the specified 30 s pre-START wait and IMU reference qualification, then move only after valid requests are permitted.
- **SYS-02 and SYS-03 (support B3):** confirm motor response on straights/curves and route actions. Barcode tracking pivots demonstrate navigation; they do not by themselves prove finite `ENCODER_TURN` accuracy.
- **SYS-04 (support B5/B3):** confirm stop before contact, settled scan, detour execution and rejoin handover. B5 records the required width, time and repeated-trial results. Motion remaining held on an unsafe path must be distinguished from a successful bypass.
- **SYS-07 (support B1):** compare motion state, signed wheel speeds, encoder counts and distance/heading with the visible movement and serial feedback, allowing for frame precision/update delay. This does not establish physical odometry accuracy.
- **SYS-08 (support B1):** observe whether motion continues without a motion fault during the specified network outage/reconnect; this requires an actual outage test, not ordinary connected driving.
- **SYS-10 (support B4):** confirm both physical wheels stop and remain held after lift/overturn. Check existing evidence with B4 before signing off steps not yet physically demonstrated.
- **SYS-11 (all):** support a continuous full-course recording showing startup, barcode actions, obstacle bypass, hump reporting and lift/overturn ending. Reference separate outage and repeated-trial evidence where required; a single normal mission clip does not establish every stress/performance criterion in SYS-01 to SYS-10.

### Reusing subsystem videos

There is no need to record a new clip solely because a different log has a
different test ID. Reuse footage only when it shows the specified setup, action
and outcome; add the video link and timestamps to each applicable results row.

| Existing footage | Can support | Additional coverage needed |
|---|---|---|
| [S2-H01 startup](https://youtu.be/lzCOjuQXgig) | SYS-01 | Full 30 s wait, dashboard/IMU state and post-START qualification |
| [S2-H03 driving/speed feedback](https://youtu.be/8NpKcqakr0c) | Parts of SYS-02, SYS-06, SYS-07; INT-06/08/10 | Curves/clear turn, relevant motion-class/serial readings and feedback comparison; full-course completion for SYS-02 |
| [S2-H04 STOP/START](https://youtu.be/yS6CX1FzbrM) | INT-09 line-following case; SYS-09 | At least 5 s held after settling; SYS-09 also needs hump recording pause/resume |
| Team/B5 obstacle footage | SYS-04; INT-07 | Stop/scan/detour/rejoin visible; required measured width/time and trial count for system performance claims |
| Team full-mission footage | SYS-11 and demonstrated parts of other cases | All required course features and end-of-run behaviour visible; separate special-case tests where not demonstrated |

[S2-H02 encoder isolation](https://youtu.be/qB7hPMGxUTQ) is subsystem
evidence. It does not replace integrated mission footage. Physical
test evidence may be a video showing the robot and dashboard/serial readings;
no separate Buddy 2 physical-test log file is required for video-visible claims.
Preserve existing team results and append actual new runs with their tested
commit, testers, result and evidence. Owner sign-off is not implied by adding
this checklist.

## Buddy 1 checklist

What Buddy 1 brings to the system tests (details in
`testing/subsystem1/TEST_LOG.md`):

- **SYS-01 (lead):** show the dashboard before START (pill **robot online**,
  **telemetry live**, Link card counts rising) and the START acknowledgement in
  the log (`robot received START #n: queued for the controller`). Sign off the
  2026-11-19 SYS-01 result, or record a new run.
- **SYS-07 (lead):** keep the whole dashboard in the video; ask each buddy to
  compare their card with the console. The Link card must show **Lost / bad**
  not rising and Frame layout **v3**.
- **SYS-08 (lead):** switch the hotspot off for 10 s **mid-run** (not while
  parked). The robot keeps driving; the dashboard shows **telemetry stale**,
  then reconnects by itself (`wifi: link up`, `mqtt: connected to ...`,
  `status frames resumed`). S1-U19 is the same test on a stand.
- **SYS-03 (support B3):** send the route from the dashboard and show
  `robot route: ok ...` before START.
- **SYS-09 (support B2):** STOP and START are sent from the dashboard; show
  both acknowledgements in the log.
- Already verified at subsystem level: S1-H01 to S1-H05 (laptop, 2026-11-24);
  S1-U17 (START / STOP / E-STOP with live telemetry) and S1-U22 (every
  telemetry card updating during a run) on the robot (2026-11-25, `b51982f`);
  S1-U18 (route) from Buddy 3's video. S1-U22 is the Buddy 1 half of SYS-07.
- Still to run: INT-12 to INT-14 and the Buddy 1 rows for SYS-01, SYS-07
  and SYS-08 below.

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
- **SYS-03 and SYS-11 (support):** confirm barcode turns, U-turns and line-search
  pivots record no hump: **Highest hump** unchanged across each turn (console
  `[TERRAIN] turning on the spot: hump recording paused` / `turn finished`).
  SYS-11 reports the final Highest hump, so a false hump from a turn would spoil it.
- Already verified at subsystem level: S4-H01, H03, H04, H07 to H12, H14 to H16.
  S4-H15 (no false hump) failed on turns on the spot (2026-11-18) and passed the
  re-test with the hump pause while turning (2026-11-24, `4033ff7`, see
  `testing/subsystem4/TEST_LOG.md`);
  at integration level: INT-03, INT-04, INT-05.

## Buddy 3 checklist

What Buddy 3 brings to the system tests (details in
`testing/subsystem3/TEST_LOG.md`):

- **SYS-02 (lead):** one unbroken run over the **whole** course, no obstacles; keep the dashboard's *On the line* in the video. S3-H01 covers
  straights, curves and finding a lost line; Buddy 4's full-run video shows the
  whole course.
- **SYS-03 (lead):** send the route from the dashboard boxes and wait for
  `robot route: ok`; put a code before a junction for **each** action (left,
  right, straight, U-turn). S3-H03/H04 show left and U-turn; Buddy 4's full-run
  video shows right and straight. A failed read is never acted on: the robot
  carries on (`route: barcode read failed, carrying on`).
- **SYS-04 (support):** confirm line following resumes after the detour
  (`[S3-LINE]` back to FOLLOWING).
- **SYS-07 (support):** *On the line* and the barcode letter/action update on
  the dashboard.
- **SYS-11:** send the route before START. Only the four route letters are read;
  any other code is reported as a failed read and the robot carries on.
- Already verified at subsystem level: S3-U01 to U03, S3-H01 to H04.

## Other logs in this folder

The `com4_2026-11-0*.log` files are raw serial captures from Buddy 2's
3–4 November debugging sessions, cited by `docs/DETOUR_FREEZE_2026-11-04.md`,
`docs/ENCODER_POLARITY_2026-11-04.md`, `docs/INTERMITTENT_STOP_INVESTIGATION.md`
and `docs/MEMORY_DEBUG_2026-11-04.md`. They are not system test results.

## Results

Add a new row for every run. Never edit or delete old rows.

Buddy 2 entries below reuse the 2026-11-24 results in [the subsystem log](../subsystem2/TEST_LOG.md); they are not additional test runs. **PASS (B2 portion)** records the reported subsystem behavior, with the remaining case coverage listed in Notes. Hardware entries are single-trial user-reported results; their links identify a code reference, with the exact flashed revision unconfirmed.

| Date | Test ID | Commit / code reference | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-19 | SYS-05 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=O-3cIuu86ew> | 10 mm hump (ruler), reported within ±3 mm; same full-run video as S4-H11/H14 |
| 2026-11-19 | SYS-06 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=O-3cIuu86ew> | Motion classes and turn rate during the full run; same video as S4-H12 |
| 2026-11-19 | SYS-10 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=b0VxlQ2r320> | Lift and overturn stop the motors, Highest hump frozen; same video as S4-H01/H03/H07 |
| 2026-11-19 | SYS-01 | main after `610725f` | 2500910 | PASS | <https://www.youtube.com/watch?v=O-3cIuu86ew> | Start of the full run: held and INIT until START, level reference captured (WAITING), then DRIVING; same video as S4-H08. B1 sign-off pending |
| 2026-11-24 | SYS-01 | [75d36d0 (code reference)](https://github.com/Minnnnno/INF2004_SmartRobo/tree/75d36d04a00cc8f68c0fb45ae5f6aa335cf00229/app_program/subsystem2) | Buddy 2 | PASS (B2 portion) | [S2-H01 startup and START](https://youtu.be/lzCOjuQXgig) | Reused one-trial startup/START pass. The specified 30-second pre-START wait and dashboard/IMU reference-state sequence remain to be confirmed with B1/B4. |
| 2026-11-24 | SYS-02 | [75d36d0 (code reference)](https://github.com/Minnnnno/INF2004_SmartRobo/tree/75d36d04a00cc8f68c0fb45ae5f6aa335cf00229/app_program/subsystem2) | Buddy 2 | PASS (B2 portion) | [S2-H03 wheel-speed control](https://youtu.be/8NpKcqakr0c) | Supporting one-trial functional motor/wheel-speed pass. This does not establish complete line-course traversal, curves or an uninterrupted fault-free run; B3 full-course result pending. |
| 2026-11-24 | SYS-07 | [75d36d0 (code reference)](https://github.com/Minnnnno/INF2004_SmartRobo/tree/75d36d04a00cc8f68c0fb45ae5f6aa335cf00229/app_program/subsystem2) | Buddy 2 | PASS (B2 portion) | [S2-H02 encoders](https://youtu.be/qB7hPMGxUTQ), [S2-H03 wheel speeds](https://youtu.be/8NpKcqakr0c), [S2-H04 STOP/START](https://youtu.be/yS6CX1FzbrM) | Supporting signed-encoder, wheel-speed and stop-state subsystem passes. B1 comparison of serial feedback with dashboard counts, speeds, distance/heading and motion state remains unconfirmed; barcode, terrain and obstacle fields require their owners' results. |
| 2026-11-24 | SYS-09 | [75d36d0 (code reference)](https://github.com/Minnnnno/INF2004_SmartRobo/tree/75d36d04a00cc8f68c0fb45ae5f6aa335cf00229/app_program/subsystem2) | Buddy 2 | PASS (B2 portion) | [S2-H04 STOP, hold and restart](https://youtu.be/yS6CX1FzbrM) | Reused one-trial functional STOP/hold/restart pass. Five-second hold duration, hump-recording pause/resume, retained Highest hump and the subsequent hump were not established by the submitted result description; B4 coverage pending. |
| 2026-11-24 | SYS-02 | `fd5955a` | 2500622 | PASS | <https://youtu.be/t8ZaVSFGggE>, <https://www.youtube.com/watch?v=O-3cIuu86ew> | Subsystem video S3-H01: straights, curves, line lost and found again, no FAULT. Supporting: Buddy 4's full-run video (main after `610725f`, same as SYS-01/05/06) follows the whole course without help |
| 2026-11-24 | SYS-03 | `fd5955a` | 2500622 | PASS | <https://youtu.be/GsEvuXFR86U>, <https://www.youtube.com/watch?v=O-3cIuu86ew> | Subsystem video S3-H03/H04: route sent from the dashboard (`robot route: ok`), Z → left at the junction, A → U-turn. Supporting: Buddy 4's full-run video (main after `610725f`) shows the right and straight actions |
| 2026-11-25 | SYS-01 | `b51982f` | 2500842 | **TO RECORD** | <https://youtu.be/0tZDy3p91Oc> | Buddy 1 lead run: online and held 30 s before START, START acknowledged, then DRIVING |
| 2026-11-25 | SYS-07 | `b51982f` | 2500842 | **TO RECORD** | <https://youtu.be/0tZDy3p91Oc> | Whole dashboard during the full run; Link card lost/bad not rising, layout v3. Each buddy confirms their card |
| 2026-11-25 | SYS-08 | `b51982f` | 2500842 | **TO RECORD** | <https://youtu.be/TvSyGtZ1UQo> | Hotspot off 10 s mid-run: robot kept driving, dashboard reconnected by itself after ___ s |
