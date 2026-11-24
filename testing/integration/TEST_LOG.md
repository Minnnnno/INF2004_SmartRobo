# Integration — test log

Tests that two or more subsystems work together: one test (or more) per interface between buddies (the interfaces are described in `docs/API.md`). Each test names its owners; both agree on the test and sign off the result.

Sources: **FR/NFR** = the team's Week 6 Design Review; **CR** = the lecturer's Change Requirement of 11 November 2026 (lift/overturn stop replaces collision detection).

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | **Dropped 2026-11-24** (ID not reused): samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 (old template interfaces) | auto | `subsystem3_test.c` (template demo test, no longer built; archived in `docs/template_original/`) | n/a |
| INT-02 | **Dropped 2026-11-11** (ID not reused): summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` (template demo test, deleted) | n/a |
| INT-03 | Buddy 4 lift/overturn stop reaches Buddy 2: the motors stop and stay held until START (owners: B4, B2) | CR (lift/overturn stop); B4 `motion_stop()` → Vehicle Controller `CONTROL_STOP` → motion task | manual | MQTT production image, dashboard open. 1. START; mid-run lift the robot briskly. 2. Keep it up, press START. 3. Set it down level and still for 1 s, press START. 4. Repeat 1 with an overturn on a stand | 1: within ~0.3 s wheel speed **0 / 0**, Motion **STOPPED (held)**, IMU card **LIFTED (motors stopped)**; console `[LIFT] LIFTED ... -> motors stopped`. 2: wheels never turn; `[LIFT] still LIFTED: motors stopped again`. 3: `[LIFT] cleared ...`, START → **DRIVING**. 4: as 1 with *Motion class* **OVERTURNED (motors stopped)** |
| INT-04 | Buddy 4 terrain data reaches the dashboard unchanged (owners: B4, B1) | FR8, FR9, FR16; terrain slot → status frame (`T_STATE`, `PITCH`, `HUMP_MM`, `M_CLASS`, `TURN_RATE`) | manual | MQTT production image, dashboard and serial console side by side in one video. 1. Hold the front up ~15°. 2. Line-following run over a hump with a turn. 3. Lift to end | Dashboard IMU & terrain card matches the console at each step: **Pitch** = `[TILT] pitch=` (± 1°); state CLIMBING/DESCENDING/WAITING = `[TERRAIN-STATE]`; **Highest hump** = `run max=` from `Hump completed`; *Motion class* = last `[MOTION] ... -> X`; *Turn rate* sign = `yaw=` sign; **LIFTED** with `[LIFT] LIFTED` |
| INT-05 | Line following waits for Buddy 4's level reference after START (owners: B4, B3) | NFR9; B4 terrain slot (`terrain_published_state()`) → B3 line task `line_imu_wait` | auto (laptop) | `ctest --test-dir build-host-cmake -R line_imu_wait --output-on-failure` (S4-U04 command) | `line_imu_wait_checks` and `line_imu_wait_start_gate_checks` pass: the line task sends no motion until Buddy 4 leaves INIT/RECOVERY (at most 10 s; an IMU fault never blocks it) |

*Method:* **auto (laptop)** = a host test suite (`ctest` or `run_tests.py`); evidence is the saved console output.
**manual** = run on the robot; evidence is a video with the dashboard (and serial console where needed).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-19 | INT-03 | main after `610725f` | 2500910 | PASS | <https://youtu.be/b0VxlQ2r320> | Steps 1 and 4 PASS (same video as S4-H01/H03). Steps 2–3 not run on the robot; host only (S4-U03 `../subsystem4/logs/2026-11-24_host.txt`). B2 sign-off pending |
| 2026-11-19 | INT-04 | main after `610725f` | 2500910 | PASS | <https://youtu.be/OaEkDdhVMTM>, <https://youtu.be/O-3cIuu86ew>, <https://youtu.be/b0VxlQ2r320> | Evidence reused from S4-H10/H11/H12/H01; dashboard and console not compared side by side. Frame mapping: S1-H03 (2026-11-14). B1 sign-off pending |
| 2026-11-19 | INT-05 | `fa36f67` | Claude Code for 2500910 | PASS | `../subsystem4/logs/2026-11-19_start_gate.txt` | `line_imu_wait_checks`, `line_imu_wait_start_gate_checks` passed (same run as S4-U04). B3 sign-off pending |
