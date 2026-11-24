# Subsystem 2 - test log

**Subsystem:** Motion control, motors, encoders, wheel-speed PI/PID and odometry (Buddy 2)

**Student ID:** <student ID>

**Code:** `app_program/subsystem2/`; interfaces in `include/motion.h`, `include/robot.h` and `include/steering.h`

References: `docs/BUDDY_MOTION_API.md`, `docs/MOTION_FAULT_DIAGNOSTICS.md`, `docs/BUDDY2_COMPACT_REPRESENTATION.md` and `videos/README.md`.

- **Laptop tests (S2-U..)** execute portable C logic with synthetic inputs/fake registers. Evidence is saved console output; these do not prove physical accuracy or RTOS timing.
- **Hardware/video tests (S2-H..)** exercise the robot, with independent measurements for accuracy claims. Evidence is a video showing the robot, measurements and relevant dashboard/serial readings; separate physical-test logs are optional.

This document defines planned tests. Empty links and NOT RUN entries are not passes. Current Kd is zero: the configured wheel controller is **PI**, although PID support exists.

## Requirements mapping

R2 labels are local traceability labels derived from Buddy 2 responsibilities/current implementation, not official requirement numbers or newly approved physical performance limits.

| Ref | Requirement / target | Laptop tests | Hardware tests |
|---|---|---|---|
| R2-1 | Startup stopped; MQTT production requires dashboard START | S2-U04, S2-U10 | S2-H01 |
| R2-2 | Independent signed encoder capture and modular rollover | S2-U03, S2-U07 | S2-H02 |
| R2-3 | Independent wheel-speed regulation, acceleration limiting and bounded output | S2-U03, S2-U05 | S2-H03 |
| R2-4 | Signed finite travel and encoder-relative completion | S2-U03, S2-U05 | S2-H05 |
| R2-5 | Left/right finite encoder turns and heading estimation | S2-U03, S2-U05 | S2-H06 |
| R2-6 | STOP cancels movement, holds and requires deliberate checked release | S2-U04, S2-U10 | S2-H04 |
| R2-7 | Invalid/stale/conflicting command rejection; expiry, feedback and overflow faults | S2-U04 | Separate physical fault tests pending |
| R2-8 | Coherent feedback, polarity, inhibition and reversal policy | S2-U05, S2-U07 | S2-H02, S2-H03; instrumented reversal pending |
| R2-9 | Safe 1 mm / 1 mm/s / 1 degree command representation; preserve individual counts | S2-U06 | Fractional-command hardware tests pending |
| R2-10 | Retain known encoder faults across software restart; checked recovery | S2-U08 | Physical restart/recovery pending |
| R2-11 | Nominal 10 ms motion cycle; detect invalid/excessive running intervals | S2-U09 | Combined-load timing measurement pending |

## Laptop tests

IDs are permanent. S2-U01/U02 are retired producer-template examples and must not be reused. Several rows map to one executable because existing suites group related checks; these are coverage groups, not eight separate executables.

| ID | What is tested | Executable / source | Expected result |
|---|---|---|---|
| S2-U01 | Retired: producer samples within 0..100 | Legacy `subsystem2_test.c`, excluded from production | Not applicable |
| S2-U02 | Retired: producer pool/free-slot bounds | Legacy template example | Not applicable |
| S2-U03 | States, signed moves/turns, odometry, quadrature, rollover, PI/PID, low-speed/breakaway simulation | `motion_tests`; `tests/test_motion.c` | Exit 0 and `PASS: <n> checks` |
| S2-U04 | Validation, ownership, cancellation, expiry, hold/reset, faults, shutdown and buffers | `motion_tests`; `tests/test_motion.c` | Relevant checks pass; exit 0 |
| S2-U05 | Steering helpers, units, speed scaling and feedback adapters | `steering_checks`; `tests/test_steering.c` | CTest Passed |
| S2-U06 | Compact signed round trips, rounding, limits, corrupt inputs, ARC/manual exceptions and precision | `buddy2_compact_checks`; `tests/buddy2_compact/test_compact.c` | `Buddy 2 compact: PASS (...)`; exit 0 |
| S2-U07 | Driver with fake registers: PWM polarity, independent reversal, inhibition, coherent capture and emergency stop | `buddy2_driver_checks`; `tests/buddy2_compact/test_driver.c` | `Buddy 2 driver: PASS (...)`; exit 0 |
| S2-U08 | Crash evidence, retained encoder fault and checked recovery | `crash_restart_checks`; `tests/test_crash_restart.c` | CTest Passed |
| S2-U09 | Invalid/late intervals, startup qualification and timing faults | `motion_tests`; `tests/test_motion.c` | Timing cases pass; not proof of actual 10 ms execution |
| S2-U10 | Controller boot hold/START and line-task IMU wait with/without START gate | `motion_tests`, `line_imu_wait_checks`, `line_imu_wait_start_gate_checks` | All three tests pass |

### Commands and evidence

Run from the repository root. Use CMake host tests, not legacy `make TEST=1`. A native compiler is required; the compact project uses GCC/Clang-compatible flags and undefined-behaviour instrumentation. Supply `-DCMAKE_C_COMPILER=<path>` and compiler-specific arguments if needed.

Replace `<date>` with the actual date. Create `testing/subsystem2/logs/` first if absent. Check each configure/build exit code before running CTest.

```powershell
cmake -S . -B build-s2-host -G Ninja -DROBO_HOST_TESTS=ON
cmake --build build-s2-host
ctest --test-dir build-s2-host -R '^(motion_tests|steering_checks|crash_restart_checks|line_imu_wait_checks|line_imu_wait_start_gate_checks)$' -V > testing/subsystem2/logs/<date>_host.txt 2>&1

cmake -S tests/buddy2_compact -B build-s2-compact -G Ninja
cmake --build build-s2-compact
ctest --test-dir build-s2-compact -V > testing/subsystem2/logs/<date>_compact_driver.txt 2>&1
```

Save verbose output including actual check counts. Earlier inspection runs reported successful host checks; this new log does not backfill them as completed runs without identifiable saved evidence. Run again and append results with the tested commit.

## Hardware/video tests

**Film S2-H01 to H04 first.** H05/H06 require a confirmed finite-command interface and agreed physical acceptance limits.

| ID | Test | Setup and steps | Expected behaviour / evidence |
|---|---|---|---|
| S2-H01 | Startup and START gate | MQTT production with `ROBO_REQUIRE_START` active. Record before power-up; show no movement before START. Place level on course, press START and keep still during IMU qualification. | Physical wheels stay stopped before START. After START/qualification, valid autonomous requests drive. Show dashboard hold/state and wheels, not just command acknowledgement. |
| S2-H02 | Independent signed encoders | Motor power disconnected, chassis supported. Use raw-count viewer/read-only diagnostic firmware. Save baseline; rotate left only forward/backward, then right only; pause for readings. | Only rotated wheel's count changes; forward positive/reverse negative using modular interpretation. Other count unchanged; invalid totals do not increase. Record pins/viewer/build. |
| S2-H03 | Wheel-speed control | Production robot on straight track. Record serial requested/measured speeds and outputs, physical movement and dashboard. Repeat three runs in same conditions. | Forward targets correspond to positive movement. Show target/measured traces in the video and record tracking error/oscillation. Unequal PWM for equal requested speed is acceptable. No accuracy PASS without agreed limits. |
| S2-H04 | STOP, hold and deliberate restart | Press dashboard STOP while moving; show both wheels through settling. Wait at least 3 s after settling without START, then press START. Repeat three times. | Both wheels stop; state reaches STOPPED/held; no restart during hold. Healthy settled robot resumes after deliberate START and new movement requests. Record latency if claiming it; no hard latency limit is established here. |
| S2-H05 | Finite-distance accuracy | Confirm finite-command harness. Mark axle-centre start; request e.g. +300 mm; wait for completion/settling; measure final axle position and baseline-relative encoder displacement. Three trials each at +300 and -300 mm with clear space. | Record requested/encoder/physical distances, error, lateral drift and completion/fault. Apply physical limits agreed before testing. Identify command mode/build. |
| S2-H06 | Finite encoder-turn accuracy | Confirm production `ENCODER_TURN` harness. Mark axle midpoint/start heading and protractor/reference rays. Request +90 and -90 degrees; measure settled physical angle and encoder heading delta. Three trials each direction. | Correct wheel/heading signs; record angle error, centre drift and completion/fault. Apply pre-agreed physical limits. A barcode tracking pivot is a different command path. |

### Prerequisites and limits

- Normal production dashboard provides START/STOP/ESTOP, not arbitrary finite distance/turn commands. Confirm how H05/H06 will be submitted before filming. This document does not add a harness or change firmware.
- Standalone `calibration/distance_usb/`, motor-test and angle-test images have different output/control policies. Label their evidence separately; their accuracy does not establish production PI accuracy.
- H02 needs **raw encoder counts**, not just wheel speed. Use the selected firmware's count viewer/encoder-frame reader or diagnostic console; verify actual pins.
- Measure from the same axle-centre reference before/after. Use baseline-relative readings; cumulative distance includes earlier movement and dashboard heading wraps. Save results before manual repositioning.
- Code tolerances of 10 mm / 0.05 rad (about 2.86 degrees) are encoder-based completion settings, not proven physical accuracy limits. Until physical limits are agreed, accuracy status is **MEASURED / LIMIT PENDING**, not PASS.
- Current 10 Hz dashboard/video cannot verify 10 ms control execution, PWM frequency or electrical reversal dead time. Those require suitable target traces/instruments.
- Fault injection remains separate from these initial videos. Do not jam/hold a powered wheel by hand; host failure-logic results are not physical fault-test evidence.

## Recording procedure

1. Commit code before testing; capture `git rev-parse --short HEAD` and note any local changes. Also capture the flashed firmware build ID/profile: checkout hash alone does not identify an old UF2.
2. Flash with motor power disconnected. Identify production versus diagnostic/calibration image and enabled start/fault options.
3. For physical tests, record the robot and relevant dashboard/serial readings in the video. A video link is sufficient in the evidence column; separate serial/dashboard log files are optional.
4. Start each landscape clip with ID/name, commit and build/profile. Show setup and measurement references; keep robot and relevant feedback visible. Camera plus dashboard/terminal split-screen is suitable.
5. Perform the action without cutting away from the transition. Hold on final physical behaviour and readings. State PASS/FAIL only against defined criteria.
6. Aim for 15-60 s per simple demonstration; repeated accuracy trials may be longer or separate labelled clips. Record surface, load, battery condition, instrument/uncertainty and all trials including failures.
7. Later upload unlisted YouTube links and add them to results and `videos/README.md`. Links are deliberately blank below. Do not commit video files.

## Planned run status

This is planning, not completed execution. Video links remain blank until supplied.

| Test ID | Status | Log / video evidence | Preparation |
| --- | --- | --- | --- |
| S2-U03 | PASS (2026-11-24) | `logs/2026-11-24_motion_steering_startup_recovery.txt` | Host motion suite |
| S2-U04 | PASS (2026-11-24) | `logs/2026-11-24_motion_steering_startup_recovery.txt` | Same motion suite; safety coverage |
| S2-U05 | PASS (2026-11-24) | `logs/2026-11-24_motion_steering_startup_recovery.txt` | Steering suite |
| S2-U06 | PASS (2026-11-24) | `logs/2026-11-24_compact_representation_motor_driver.txt` | Compact suite |
| S2-U07 | PASS (2026-11-24) | `logs/2026-11-24_compact_representation_motor_driver.txt` | Driver suite |
| S2-U08 | PASS (2026-11-24) | `logs/2026-11-24_motion_steering_startup_recovery.txt` | Crash/restart suite |
| S2-U09 | PASS (2026-11-24) | `logs/2026-11-24_motion_steering_startup_recovery.txt` | Simulated timing cases only |
| S2-U10 | PASS (2026-11-24) | `logs/2026-11-24_motion_steering_startup_recovery.txt` | Motion and both IMU-wait suites |
| S2-H01 | NOT RUN |  | MQTT production/dashboard |
| S2-H02 | NOT RUN |  | Raw-count viewer; motor power disconnected |
| S2-H03 | NOT RUN |  | Serial targets/speeds/outputs; straight track |
| S2-H04 | NOT RUN |  | Dashboard STOP/START; clear track |
| S2-H05 | NOT RUN |  | Confirm harness and physical limits |
| S2-H06 | NOT RUN |  | Confirm finite-turn harness and physical limits |

## Results

Append one row per actual run. Never overwrite/delete old results; record failures and append later retests. Retired template examples had no recorded result; their old placeholder row was not an actual run.

Put readings in the **Notes / agreed criterion** column instead of separate measurement tables. For distance/angle accuracy, include requested value, encoder delta, physical settled measurement and error (physical minus requested), with units and trial number. For speed/stop tests, include target/measured speeds and the observation interval, or measured stopping time/distance when claimed. Use the combined Log / video evidence column for laptop log files or physical-test video links. Example only: `Requested +300 mm; encoder +301 mm; physical +294 mm; error -6 mm; trial 1/3`.

For startup, encoder-direction and STOP/START demonstrations, record the observed behaviour and video/log evidence. Retain measured values for any accuracy claim; do not infer accuracy from a single dashboard reading.

| Date | Test ID / trial | Exact commit | Tester | Result | Log / video evidence | Notes / agreed criterion |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-24 | S2-U03 | `27eda0c9439075e4bbcf3c60ea9e4f68a385b443` | Codex | PASS | [Host log](logs/2026-11-24_motion_steering_startup_recovery.txt) | `motion_tests`: 9633 synthetic checks; no hardware accuracy claim |
| 2026-11-24 | S2-U04 | `27eda0c9439075e4bbcf3c60ea9e4f68a385b443` | Codex | PASS | [Host log](logs/2026-11-24_motion_steering_startup_recovery.txt) | Safety/command coverage in the same motion suite; not a separate execution |
| 2026-11-24 | S2-U05 | `27eda0c9439075e4bbcf3c60ea9e4f68a385b443` | Codex | PASS | [Host log](logs/2026-11-24_motion_steering_startup_recovery.txt) | `steering_checks`: 126 checks |
| 2026-11-24 | S2-U06 | `27eda0c9439075e4bbcf3c60ea9e4f68a385b443` | Codex | PASS | [Compact/driver log](logs/2026-11-24_compact_representation_motor_driver.txt) | 85465 compact checks; individual count precision retained |
| 2026-11-24 | S2-U07 | `27eda0c9439075e4bbcf3c60ea9e4f68a385b443` | Codex | PASS | [Compact/driver log](logs/2026-11-24_compact_representation_motor_driver.txt) | Polarity, independent reversal, inhibition, coherent capture and emergency stop |
| 2026-11-24 | S2-U08 | `27eda0c9439075e4bbcf3c60ea9e4f68a385b443` | Codex | PASS | [Host log](logs/2026-11-24_motion_steering_startup_recovery.txt) | `crash_restart_checks`: retained crash/encoder evidence and checked recovery |
| 2026-11-24 | S2-U09 | `27eda0c9439075e4bbcf3c60ea9e4f68a385b443` | Codex | PASS | [Host log](logs/2026-11-24_motion_steering_startup_recovery.txt) | Timing cases in the same motion suite; actual 10 ms scheduling not measured |
| 2026-11-24 | S2-U10 | `27eda0c9439075e4bbcf3c60ea9e4f68a385b443` | Codex | PASS | [Host log](logs/2026-11-24_motion_steering_startup_recovery.txt) | Motion plus IMU wait (2041 checks) and web START-gate IMU wait (3605 checks) |

Run summary: selected host CTest batch 5/5 passed; standalone compact/driver batch 2/2 passed, using native Zig 0.16.0 / Clang 21.1.0 with undefined-behaviour instrumentation in the compact/driver batch. All configure/build/CTest exit codes were zero. The root build emitted a pointer-width warning in `lib/libtm/libtm_printf.c:230` for a template formatter compiled on the host; compilation still succeeded. No firmware was flashed and no physical tests were performed. The pre-existing untracked `2026-11-24_084250_*` logs were preserved; the linked motion/steering/startup/recovery and compact/driver logs are evidence for this run.
