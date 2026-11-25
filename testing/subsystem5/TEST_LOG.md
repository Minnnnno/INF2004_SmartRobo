# Subsystem 5 — test log

**Subsystem:** Obstacle detection, width estimation and avoidance  ·  **Student ID:** 2500344  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Stops before an obstacle | Stop about 150 mm from the obstacle (`S5_STOP_MM`) | manual | 1. Put a box on a straight part of the line. 2. START from the dashboard. 3. Read the Motion and Obstacle cards when it stops. | Stops without touching the box; Motion STOPPED, wheels 0 / 0; Distance ahead about 15 cm |
| S5-U02 | Width estimate | Width within ± 10 cm | manual | 1. Measure the box with a ruler. 2. Run as S5-U01 and let the scan finish. 3. Read Width on the dashboard. | Reported width within ± 10 cm of the ruler |
| S5-U03 | Full run: stop, scan, go round, rejoin | Go round the obstacle and rejoin the line; hands motion back to line following (Buddy 3) | manual | 1. Centre a box on the line. 2. START. 3. Leave it alone. | Goes round without touching the box and carries on along the line; obstacle state held / scanning → driving round it → clear |
| S5-U04 | Avoids on the left | Pass on the side with more clearance | manual | 1. Place the box with more clearance on its left. 2. START. | Passes on the left without touching the box and gets back on the line |
| S5-U05 | Avoids on the right | Pass on the side with more clearance | manual | 1. Place the box with more clearance on its right. 2. START. | Passes on the right without touching the box and gets back on the line |
| S5-U06 | Reacts to a sudden obstacle after a turn | React to an obstacle that only comes into view after a turn | manual | 1. Put the box on the line just after a curve. 2. START before the curve. | Stops without touching the box, scans, goes round and gets back on the line |
| S5-U07 | Resumes after a false halt | If what stopped the robot is gone, resume line following | manual | 1. Run with nothing on the line. 2. Hold a hand in front of the sensor until it stops. 3. Take the hand away. | Stops, scans, then drives on along the line by itself |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-24 | S5-U01 | `fa36f67` + width scale (later `e4e5b3e`) | 2500344 | PASS | <https://youtu.be/I9SrSpBJh9Q> (0:06–0:28), [`logs/2026-11-24_normal_run_dashboard.log`](logs/2026-11-24_normal_run_dashboard.log), full dashboard log [`logs/2026-11-24_dashboard_full.log`](logs/2026-11-24_dashboard_full.log) | Distance ahead 15 cm, wheels 0 / 0, no contact seen. The scan itself then measured 162 mm. Built from checkout `aa3cb6d`, whose Subsystem 5 code is `fa36f67` |
| 2026-11-24 | S5-U02 | `fa36f67` + width scale (later `e4e5b3e`) | 2500344 | PASS | <https://youtu.be/I9SrSpBJh9Q> (0:30), [`logs/2026-11-24_normal_run_dashboard.log`](logs/2026-11-24_normal_run_dashboard.log), full dashboard log [`logs/2026-11-24_dashboard_full.log`](logs/2026-11-24_dashboard_full.log) | Reported 127 mm for a 150 mm box (tester's figure, no ruler in the video): 23 mm under. Unscaled 159 mm. SYS-04 quotes NFR11 as ± 1 cm: tolerance to be settled. Earlier runs on 20 Nov without the scale: 100 mm box read 188 and 166 mm, 150 mm box 215 and 190 mm (serial log [`logs/2026-11-20_width_100mm.txt`](logs/2026-11-20_width_100mm.txt)); two more 100 mm runs read 224 and 222 mm (full dashboard log, 12:12 and 12:15) |
| 2026-11-24 | S5-U03 | `fa36f67` + width scale (later `e4e5b3e`) | 2500344 | PASS | <https://youtu.be/I9SrSpBJh9Q>, [`logs/2026-11-24_normal_run_dashboard.log`](logs/2026-11-24_normal_run_dashboard.log), full dashboard log [`logs/2026-11-24_dashboard_full.log`](logs/2026-11-24_dashboard_full.log) | Passed on the right. Stop to handover to line following: 51 s. The log shows the line lost 1 s after the handover and found again 8 s later |
| 2026-11-24 | S5-U04 | `fa36f67` (not confirmed) | 2500344 | PASS | <https://youtu.be/RCM5-oiSxDs> | Box to the right of the line; passed on the left, no contact seen. No dashboard in the video |
| 2026-11-24 | S5-U05 | `fa36f67` (not confirmed) | 2500344 | PASS | <https://youtu.be/O0wmRNscU0k> | Box slightly to the left of the line; passed on the right, no contact seen. No dashboard in the video |
| 2026-11-24 | S5-U06 | `fa36f67` (not confirmed) | 2500344 | PASS | <https://youtu.be/5Ype3yBMZK4> | Stopped just after the curve, passed on the right, back on the line at the junction. No dashboard in the video |
| 2026-11-24 | S5-U07 | `fa36f67` + width scale (later `e4e5b3e`) | 2500344 | PASS | <https://youtu.be/FtWQj1oa1vA>, [`logs/2026-11-24_false_halt_dashboard.log`](logs/2026-11-24_false_halt_dashboard.log), full dashboard log [`logs/2026-11-24_dashboard_full.log`](logs/2026-11-24_dashboard_full.log) | Stopped for a hand and scanned; the clip ends at 0:41 before it drives on. The dashboard run with matching timing ended `abort=no_release`. Re-film. An earlier run on 20 Nov did resume: full dashboard log, 14:07:20, `spurious_stop ahead_mm=1224 resumed=1` |
