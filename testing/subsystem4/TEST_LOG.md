# Subsystem 4 — test log

**Subsystem:** Monitor (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | **Dropped 2026-11-11** (ID not reused): message buffer max line = S4_SUMMARY_MAX | IF-03 | auto | `subsystem4_test.c` (template demo test, deleted with `task_monitor.c`) | n/a |
| S4-U02 | lift/overturn detection, stop and re-stop (host) | CR 2026-11-11: lift/overturn stop | auto | `python tests/subsystem4/run_tests.py` | `PASS: ... lift checks` and `All Buddy 4 host suites passed.` |
| S4-H01 | lift while driving stops the motors | CR 2026-11-11 | manual | Line-following run on the floor; pick the robot straight up mid-run | Wheels stop within ~0.3 s; console `[LIFT] LIFTED ... -> motors stopped`; dashboard T_STATE LIFTED |
| S4-H02 | lift while stationary | CR 2026-11-11 | manual | Robot STOPPED (held); pick it straight up briskly | `[LIFT] LIFTED`; robot stays stopped |
| S4-H03 | overturn stops the motors | CR 2026-11-11 | manual | Wheels driving on a stand; roll the robot onto its side, then upside down | `[LIFT] OVERTURNED ... -> motors stopped` within ~0.3 s; `[MOTION] ... -> OVERTURNED` |
| S4-H04 | humps never trigger the lift stop | CR 2026-11-11 | manual | Full run over every course hump, both wheel sides, normal speed | No `[LIFT]` line, no stop; hump peaks still reported |
| S4-H06 | tilting to one side does not stop the robot | CR 2026-11-11 (clarified) | manual | While driving on a stand, tilt the robot 40-50 deg to one side by hand for 2 s, then set it level | No `[LIFT]` line, wheels keep driving; no hump recorded (`run_max` unchanged) |
| S4-H07 | overturn freezes hump recording until START (demo end) | CR 2026-11-11 (clarified: run ends lifted and turned over) | manual + auto | Host: `python tests/subsystem4/run_tests.py` (`freeze_script`, `buddy4_hump_start` checks). Robot: complete one hump and note `run_max`; lift it and turn it over; hold it level in the air until `[LIFT] cleared`; set it down at an angle and rock it side to side; press START; drive over one more hump | On the overturn: `[TERRAIN] hump recording frozen (OVERTURNED): run max kept at N mm until START`; `[TILT] ... terrain=FROZEN`; no `Hump completed` and `run_max` unchanged until START; after START: `[TERRAIN] hump recording resumed (START)` and the next hump is recorded |
| S4-H08 | level reference and hump recording start only at the dashboard START | Startup with the Buddy 1 web Start gate (`ROBO_REQUIRE_START`) | manual + auto | Host: `run_tests.py` (`test_terrain_task_start_gate`, WAIT_START checks) and `ctest -R line_imu_wait_start_gate_checks`. Robot (MQTT image, dashboard open): power up, carry the robot around for 30 s, place it level on the course, press START, keep it still | Before START: `[B4] waiting for dashboard START`, T_STATE INIT, no capture and no FAULT, wheels still. After START: `[B4] START: capturing level reference`, MONITORING within ~2 s, `line_imu_wait settled=1`, then line following begins; humps are recorded |
| S4-H09 | dashboard STOP pauses hump recording until START | Startup/stop with the Buddy 1 dashboard | manual + auto | Host: `run_tests.py` (`pause_script`, `B4_HUMP_PAUSE` checks). Robot: drive over one hump and note `run_max`; press STOP on the dashboard; raise one side of the robot ~3 cm by hand and lower it again; press START; drive over one more hump | On STOP: `[TERRAIN] hump recording paused (robot stopped): run max kept at N mm until START`, `[TILT] ... terrain=PAUSED`; no `Hump completed` and `run_max` unchanged while stopped; on START: `[TERRAIN] hump recording resumed (START)`, and the next hump is recorded |
| S4-H05 | set down then START resumes; START in the air is refused | CR 2026-11-11 | manual | After S4-H01: press START while still holding it up, then set it down level for 1 s and press START | In the air: `[LIFT] still LIFTED: motors stopped again`; set down: `[LIFT] cleared ...`, then START drives again |

Change Requirement 2026-11-11 (lecturer): IMU collision detection removed; the
IMU must detect lifted or overturned and stop the motors (lifting ends the run
in the demonstration). See `app_program/subsystem4/README.md`.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
