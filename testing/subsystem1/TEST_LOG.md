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
| S1-U04 | valid command reaches `mbf_comms_cmd`, invalid one does not | FR17, IF-07 | auto | `subsystem1_test.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | status, heartbeat and telemetry reach the broker | FR15, FR16 | manual | 1. Flash `NET=lwip MQTT=1` image. 2. `mosquitto_sub -t "robot/#" -v` | retained `robot/status online`, `robot/heartbeat` every 1 s, `robot/telemetry/comms` every 1 s |
| S1-U06 | commands are acknowledged | FR17 | manual | `mosquitto_pub -t robot/cmd -m stop`, then `-m junk` | `robot/cmd/ack ok stop`, then `rejected` |
| S1-U07 | reconnects after the broker is lost | FR18 | manual | 1. Stop the broker for 20 s. 2. Start it again. Watch the USB console. | retries at 1, 2, 4, 8, 10 s, then `robot/status online` again |
| S1-U08 | broker reports the robot offline | FR15 | manual | Unplug the Pico while subscribed | `robot/status offline` after the keep-alive expires (up to ~90 s) |

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
