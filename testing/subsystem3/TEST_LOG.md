# Subsystem 3 — test log

**Subsystem:** Consumer (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | statistics self-consistent (min ≤ max, 0..100) | | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | barcode decoder: small and big Code 39, both directions, 0.05/0.10 m/s, no misreads | `barcode_decode.c` | auto (host) | `ctest -R barcode_decode_checks` (synthetic passes + shuttle runs replayed from `tests/barcode/replay.txt`) | `barcode decode checks passed` |
| S3-U03 | barcode sensor coverage across the bars (static) | GP28 AO, TCRT5000 | manual | barcode test image; car still; slide the sensor end to end along one small and one big wide bar, note `sensor level` every few mm; also white and a small narrow bar | small wide bar reaches near big-wide level somewhere along the bar |
| S3-U04 | shuttle reads, small code | `barcode_task` at 0.05 m/s | manual | `-DROBO_BARCODE_SHUTTLE=1 -DROBO_BARCODE_STREAM=1` image, 20 round trips over the small Z; score with `python tools/barcode_log.py --since …` | ≥ 95 % `barcode: *Z*`, no other letter |
| S3-U05 | shuttle reads, big code | as S3-U04 | manual | as S3-U04 over the big code | ≥ 95 % reads, no other letter |
| S3-I01 | subsystem 3 integration: line following + barcode read + junction detection after the read | `line_task`, `barcode_task` (`line_following.c`, `barcode.c`) | manual | integrated image (or `build-s3-integration`); car on the line before a barcode; log the serial output | `barcode: *X*`, then `route: junction after X, … (no action, following on)`; the car keeps following the line; a failed read prints `route: barcode read failed, car stopped` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-10 | S3-U02 | uncommitted (on `e8f5da0`) | Claude | PASS | `ctest -R barcode_decode_checks` | Synthetic 80/80 Z. Full logs: small Z 16/17 (run 145806), 10/40 (run 143512, white ≈ 680); big A/Z reads kept (27, 38, 40, 8); 0 wrong letters. Old decoder: small 0/57. |
| 2026-11-10 | S3-U04 | uncommitted | fatmosthegreat / Claude | FAIL → re-tuned | `Resources/logs_archive/run_20261110-124200.txt` | Shuttle at 0.05 m/s over the small A: firmware decoded 1/9 (narrow bars +15..20 over the dips beside them). Bar-end rule changed to a third of the fall and a lower in-pass bar threshold: replay now 9/9, 0 wrong. Needs a fresh hardware run. |
