# Subsystem 1 — test log

**Subsystem:** WiFi communication, command and telemetry  ·  **Student ID:** 2500842  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | telemetry is queued while there is room | FR16, IF-06 | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | full queue drops the message without waiting, and counts it | NFR6, NFR8 | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | oversized payload is refused | NFR8 | auto | `subsystem1_test.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | valid binary command reaches `mbf_comms_cmd`; wrong length, truncated and reserved opcodes do not | FR17, IF-07 | auto | `subsystem1_test.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | status, heartbeat and 10 Hz frames reach the broker and decode | FR15, FR16 | manual | 1. Flash `NET=lwip MQTT=ON` image. 2. `python tools/dashboard.py listen --host <broker> --csv logs/<date>_frames.csv` | retained `robot/status online`, `robot/heartbeat` every 1 s, `robot/frame` decoded every 100 ms, `lost=0` |
| S1-U06 | binary commands are acknowledged | FR17 | manual | `python tools/dashboard.py send stop --host <broker>` while `listen` runs | `ACK ok opcode=2` |
| S1-U07 | reconnects after the broker is lost | FR18 | manual | 1. Stop the broker for 20 s. 2. Start it again. Watch the USB console. | retries at 1, 2, 4, 8, 10 s, then `robot/status online` again |
| S1-U08 | broker reports the robot offline | FR15 | manual | Unplug the Pico while subscribed | `robot/status offline` after the keep-alive expires (up to ~90 s) |
| S1-U09 | repeated or stale START is not forwarded; STOP always is | FR17, safety | auto | `subsystem1_test.c` | `[TEST] S1-U09 PASS` |
| S1-U10 | 21-byte frame round trip keeps negatives, clamps out-of-range | Tribunal 1 decode test | auto | `subsystem1_test.c` | `[TEST] S1-U10 PASS` |
| S1-U11 | latched events are published once and merged | Tribunal 1 events | manual | With `listen` running, trigger a motion fault or line loss twice quickly | one `EVENT` line per 100 ms period listing each event once |
| S1-U12 | packed telemetry does not disturb motion timing (NFR6) | NFR6, A1 | manual | Run normal operation, then stop/start the broker; read the `comms:` console line every 10 s and Buddy 2's motion timing (`max_observed_dt`) | `skipped` stays near 0, `copy_us_max` small vs 10 ms, no motion timing fault |
| S1-H01 | every field, clamp, malformed frame, all 65,536 command words, snapshot mapping, JSON vs packed size | Tribunal 1 decode test | auto (PC) | `cmake -S . -B build-host -DROBO_HOST_TESTS=ON`, build, `ctest -R buddy1` | `PASS: 317 checks, 0 failed` |
| S1-H02 | dashboard decoder agrees with firmware codec | Tribunal 1 decode test | auto (PC) | `python tools/dashboard.py selftest` | `PASS: dashboard selftest, 0 failures` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-11 | S1-U01 | not committed yet | 2500842 | PASS | `logs/2026-10-11_selftest.txt` | `NET=lwip MQTT=ON TEST=ON` build |
| 2026-10-11 | S1-U02 | not committed yet | 2500842 | PASS | `logs/2026-10-11_selftest.txt` | |
| 2026-10-11 | S1-U03 | not committed yet | 2500842 | PASS | `logs/2026-10-11_selftest.txt` | |
| 2026-10-11 | S1-U04 | not committed yet | 2500842 | PASS | `logs/2026-10-11_selftest.txt` | |
| 2026-11-01 | S1-H01 | not committed yet | Claude Code for 2500842 | PASS | console: `PASS: 317 checks, 0 failed`; `size: JSON 381 bytes, packed 21 bytes (161 bits used, 7 spare), 5.5% of JSON` | MinGW gcc 15, -Wall -Wextra -Werror -Wconversion |
| 2026-11-01 | S1-H02 | not committed yet | Claude Code for 2500842 | PASS | console: `PASS: dashboard selftest, 0 failures` | Python 3.14; golden frame matches the C test |
