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
| S4-H02 | lift while stationary | CR 2026-11-11 | manual | Robot STOPPED (held); lift it by one end (> 35 deg) | `[LIFT] LIFTED`; robot stays stopped |
| S4-H03 | overturn stops the motors | CR 2026-11-11 | manual | Wheels driving on a stand; roll the robot onto its side, then upside down | `[LIFT] OVERTURNED ... -> motors stopped` within ~0.3 s; `[MOTION] ... -> OVERTURNED` |
| S4-H04 | humps never trigger the lift stop | CR 2026-11-11 | manual | Full run over every course hump, both wheel sides, normal speed | No `[LIFT]` line, no stop; hump peaks still reported |
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
