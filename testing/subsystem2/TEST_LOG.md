# Subsystem 2 - test log

**Subsystem:** Motion control, motors, encoders, wheel-speed PI/PID and odometry (Buddy 2)

**Student ID:** <student ID>

**Code:** `app_program/subsystem2/`; interfaces in `include/motion.h`, `include/robot.h` and `include/steering.h`

References: `docs/BUDDY_MOTION_API.md`, `docs/MOTION_FAULT_DIAGNOSTICS.md`, `docs/BUDDY2_COMPACT_REPRESENTATION.md` and `videos/README.md`.

- **Laptop tests (S2-U..)** execute portable C logic with synthetic inputs/fake registers. Evidence is saved console output; these do not prove physical accuracy or RTOS timing.
- **Hardware/video tests (S2-H..)** exercise the robot, with independent measurements for accuracy claims. Evidence is a video showing the robot, measurements and relevant dashboard/serial readings; separate physical-test logs are optional.

This document records laptop tests and submitted hardware video results. The four hardware results are user-reported PASS, with one trial per test. Current Kd is zero: the configured wheel controller is **PI**, although PID support exists.

## Requirements mapping

R2 labels are local traceability labels derived from Buddy 2 responsibilities/current implementation, not official requirement numbers or newly approved physical performance limits.

| Ref | Requirement / target | Laptop tests | Hardware tests |
|---|---|---|---|
| R2-1 | Startup stopped; MQTT production requires dashboard START | S2-U04, S2-U10 | S2-H01 |
| R2-2 | Independent signed encoder capture and modular rollover | S2-U03, S2-U07 | S2-H02 |
| R2-3 | Independent wheel-speed regulation, acceleration limiting and bounded output | S2-U03, S2-U05 | S2-H03 |
| R2-4 | Signed finite travel and encoder-relative completion | S2-U03, S2-U05 | Not physically tested |
| R2-5 | Left/right finite encoder turns and heading estimation | S2-U03, S2-U05 | Not physically tested |
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
| S2-U01 | Retired: producer samples within 0..100 | Removed legacy producer demo | Not applicable |
| S2-U02 | Retired: producer pool/free-slot bounds | Removed legacy producer demo | Not applicable |
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

Save verbose output including actual check counts. The laptop results below link to saved evidence for the identified tested commit. Append a new result row for each later run.

## Code revision references

- Current repository revision: [75d36d04a00cc8f68c0fb45ae5f6aa335cf00229](https://github.com/Minnnnno/INF2004_SmartRobo/commit/75d36d04a00cc8f68c0fb45ae5f6aa335cf00229). Browse the [Subsystem 2 code at this revision](https://github.com/Minnnnno/INF2004_SmartRobo/tree/75d36d04a00cc8f68c0fb45ae5f6aa335cf00229/app_program/subsystem2).
- Recorded laptop-test revision: [27eda0c9439075e4bbcf3c60ea9e4f68a385b443](https://github.com/Minnnnno/INF2004_SmartRobo/commit/27eda0c9439075e4bbcf3c60ea9e4f68a385b443), identified in both saved host logs.

The current repository revision is a source reference for code review. The exact firmware revision flashed for S2-H01 to S2-H04 remains unconfirmed; the source reference does not establish which revision was used in those videos.

## Results

Hardware videos were submitted on 2026-11-24. Buddy 2 reports S2-H01 to S2-H04 passed, with ONE trial each. Test date: 2026-11-24, as supplied by Buddy 2. Flashed code commits were not supplied. Videos were not independently reviewed by the agent. Single trials do not establish repeatability or replace measured accuracy/timing evidence. Only these four hardware tests were performed; unperformed hardware cases have been removed.

Append one row per actual run. Never overwrite/delete old results; record failures and append later retests. Retired template examples had no recorded result; their old placeholder row was not an actual run.

Put readings in the **Notes / agreed criterion** column instead of separate measurement tables. For distance/angle accuracy, include requested value, encoder delta, physical settled measurement and error (physical minus requested), with units and trial number. For speed/stop tests, include target/measured speeds and the observation interval, or measured stopping time/distance when claimed. Use the combined Log / video evidence column for laptop log files or physical-test video links. Example only: `Requested +300 mm; encoder +301 mm; physical +294 mm; error -6 mm; trial 1/3`.

For startup, encoder-direction and STOP/START demonstrations, record the observed behaviour and video/log evidence. Retain measured values for any accuracy claim; do not infer accuracy from a single dashboard reading.

| Date | Test ID / trial | Exact commit | Tester | Result | Log / video evidence | Notes / agreed criterion |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-11-24 | S2-U03 | [27eda0c9439075e4bbcf3c60ea9e4f68a385b443](https://github.com/Minnnnno/INF2004_SmartRobo/commit/27eda0c9439075e4bbcf3c60ea9e4f68a385b443) | Codex | PASS | [Host log](logs/2026-11-24_motion_steering_startup_recovery.txt) | `motion_tests`: 9633 synthetic checks; no hardware accuracy claim |
| 2026-11-24 | S2-U04 | [27eda0c9439075e4bbcf3c60ea9e4f68a385b443](https://github.com/Minnnnno/INF2004_SmartRobo/commit/27eda0c9439075e4bbcf3c60ea9e4f68a385b443) | Codex | PASS | [Host log](logs/2026-11-24_motion_steering_startup_recovery.txt) | Safety/command coverage in the same motion suite; not a separate execution |
| 2026-11-24 | S2-U05 | [27eda0c9439075e4bbcf3c60ea9e4f68a385b443](https://github.com/Minnnnno/INF2004_SmartRobo/commit/27eda0c9439075e4bbcf3c60ea9e4f68a385b443) | Codex | PASS | [Host log](logs/2026-11-24_motion_steering_startup_recovery.txt) | `steering_checks`: 126 checks |
| 2026-11-24 | S2-U06 | [27eda0c9439075e4bbcf3c60ea9e4f68a385b443](https://github.com/Minnnnno/INF2004_SmartRobo/commit/27eda0c9439075e4bbcf3c60ea9e4f68a385b443) | Codex | PASS | [Compact/driver log](logs/2026-11-24_compact_representation_motor_driver.txt) | 85465 compact checks; individual count precision retained |
| 2026-11-24 | S2-U07 | [27eda0c9439075e4bbcf3c60ea9e4f68a385b443](https://github.com/Minnnnno/INF2004_SmartRobo/commit/27eda0c9439075e4bbcf3c60ea9e4f68a385b443) | Codex | PASS | [Compact/driver log](logs/2026-11-24_compact_representation_motor_driver.txt) | Polarity, independent reversal, inhibition, coherent capture and emergency stop |
| 2026-11-24 | S2-U08 | [27eda0c9439075e4bbcf3c60ea9e4f68a385b443](https://github.com/Minnnnno/INF2004_SmartRobo/commit/27eda0c9439075e4bbcf3c60ea9e4f68a385b443) | Codex | PASS | [Host log](logs/2026-11-24_motion_steering_startup_recovery.txt) | `crash_restart_checks`: retained crash/encoder evidence and checked recovery |
| 2026-11-24 | S2-U09 | [27eda0c9439075e4bbcf3c60ea9e4f68a385b443](https://github.com/Minnnnno/INF2004_SmartRobo/commit/27eda0c9439075e4bbcf3c60ea9e4f68a385b443) | Codex | PASS | [Host log](logs/2026-11-24_motion_steering_startup_recovery.txt) | Timing cases in the same motion suite; actual 10 ms scheduling not measured |
| 2026-11-24 | S2-U10 | [27eda0c9439075e4bbcf3c60ea9e4f68a385b443](https://github.com/Minnnnno/INF2004_SmartRobo/commit/27eda0c9439075e4bbcf3c60ea9e4f68a385b443) | Codex | PASS | [Host log](logs/2026-11-24_motion_steering_startup_recovery.txt) | Motion plus IMU wait (2041 checks) and web START-gate IMU wait (3605 checks) |
| 2026-11-24 | S2-H01 / trial 1 | Not supplied | Buddy 2 | PASS | [Startup and START](https://youtu.be/lzCOjuQXgig) | User-reported pass; startup and START gate; one trial only. |
| 2026-11-24 | S2-H02 / trial 1 | Not supplied | Buddy 2 | PASS | [Independent encoders](https://youtu.be/qB7hPMGxUTQ) | User-reported pass; independent signed encoder check; one trial only; does not establish rollover or maximum edge-rate performance. |
| 2026-11-24 | S2-H03 / trial 1 | Not supplied | Buddy 2 | PASS | [Wheel-speed control](https://youtu.be/8NpKcqakr0c) | User-reported functional pass; one trial only; original three-run repeatability check not completed; no numerical speed-accuracy result supplied. |
| 2026-11-24 | S2-H04 / trial 1 | Not supplied | Buddy 2 | PASS | [STOP, hold and restart](https://youtu.be/yS6CX1FzbrM) | User-reported functional pass; one trial only; original three-run repeatability check not completed; stop latency not supplied. |

Host run summary: selected host CTest batch 5/5 passed; standalone compact/driver batch 2/2 passed, using native Zig 0.16.0 / Clang 21.1.0 with undefined-behaviour instrumentation in the compact/driver batch. All configure/build/CTest exit codes were zero. The root build emitted a pointer-width warning in `lib/libtm/libtm_printf.c:230` for a template formatter compiled on the host; compilation still succeeded. No firmware was flashed and no physical tests were performed. The linked motion/steering/startup/recovery and compact/driver logs are evidence for this run. Two incomplete metadata-only logs from the earlier attempt were subsequently removed during cleanup; the completed evidence logs are retained.
