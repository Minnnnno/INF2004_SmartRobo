# Subsystem 3 — test log

**Subsystem:** Line following and barcode navigation (Buddy 3)  ·  **Student ID:** 2500622  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

Two kinds of test:

- **Script tests** run the Buddy 3 C code on the laptop with recorded or
  simulated sensor data. They prove the **logic**. Evidence: the saved
  console output. No video.
- **Video tests** run on the robot. They prove the **requirement is met on the
  real hardware**, read from the web dashboard and the serial console.
  Evidence: a video, plus the dashboard and serial logs.

Script tests are numbered `S3-U..`, video tests `S3-H..`.

## Requirements

Sources: **Briefing** = the Project Briefing (mission 1 FOLLOW the line, mission 2
DECODE navigation barcodes); **FR** = the team's Week 6 Design Review (mission 1:
FR1, FR4, FR5; mission 2: FR2, FR3, FR17, see `testing/system/TEST_LOG.md`).

| Req | Requirement | Source | Script test | Video test |
|---|---|---|---|---|
| R3-1 | Line sensing: the two IR sensors tell the line from the floor and which side it is on | Briefing mission 1; FR1, FR4, FR5 | S3-U03 | S3-H01 |
| R3-2 | Line following on straights and curves | Briefing mission 1; FR1, FR4, FR5 | S3-U03 | S3-H01 |
| R3-3 | A lost line is searched for and found again | Briefing mission 1; FR1, FR4, FR5 | — | S3-H01 |
| R3-4 | Code 39 barcodes decoded in either direction while driving, never as a wrong letter | Briefing mission 2; FR2, FR3 | S3-U01 | S3-H02, S3-H03 |
| R3-5 | Route letters supplied from the dashboard over MQTT; `route.h` is the fallback | Briefing mission 2; FR17 | S3-U02 | S3-H04 |
| R3-6 | Each route action done at the junction after its code (left / right / U-turn), straight ignores the junction | Briefing mission 2; FR2, FR3 | — | S3-H03, S3-H04 |
| R3-7 | A barcode that cannot be decoded is never acted on: it is reported and the car keeps following the line | Safety (never act on a misread) | S3-U01 | S3-H05 (log only) |

## Script tests (laptop, no robot)

```sh
cmake -S . -B build-host-cmake -G Ninja -DROBO_HOST_TESTS=ON
cmake --build build-host-cmake
ctest --test-dir build-host-cmake -R "barcode_decode|route_checks|line_test|line_pid" --output-on-failure > testing/subsystem3/logs/<date>_host.txt
```

With gcc 15 the line tests only build with `-DCMAKE_C_FLAGS=-Wno-error=float-conversion`
(double→float warnings in Buddy 2 files they link, not in Buddy 3 code).

| ID | What is tested | Requirement | Expected result (in the log) |
|---|---|---|---|
| S3-U01 | Barcode decoder: small and big Code 39, both directions, 0.05/0.10 m/s, no misreads; synthetic passes and shuttle runs replayed from `tests/barcode/replay.txt`; letter sets AZ, A (reject), sensitivity ladder, four route letters | R3-4, R3-7 | `barcode decode checks passed`; all 6 `barcode_decode_*` tests pass |
| S3-U02 | Barcode route from MQTT text: any order/case/separator accepted; missing action, duplicate letter or key, bad character rejected and the last good route kept; `route.h` fallback until then | R3-5 | `route_checks passed (0 failures)` |
| S3-U03 | Line sensor reading and line-following steering (`line_test_checks`, `line_pid_checks`, both built from `line_following.c`) | R3-1, R3-2 | Both tests `Passed`; `100% tests passed` |

## Video tests (robot + dashboard)

"Dashboard" = `tools/web_dashboard.py` in the browser: the **Line & barcode**
card (line sensor, on the line, barcode letter and action, route boxes) and the
event log. The last column lists what only the serial console shows; keep it in
the same video frame. Three videos cover H01–H04: S3-H03 and S3-H04 share one run, filmed with
the dashboard screen recording overlaid (same session, synced), so the route being sent and the
action at the junction appear together. S3-H05 is shown by its saved log.

| ID | What is tested | Requirement | Steps | Expected on the dashboard | Console only (not on the dashboard) |
|---|---|---|---|---|---|
| S3-H01 | Line following: straights, curves, and finding a lost line again | R3-1, R3-2, R3-3 | MQTT image; START on the course (straights and curves); once, nudge the robot ~3 cm off the line and let go | **On the line: yes** while following; after the nudge **no (line lost)**, then **yes** again within ~3 s; Motion **DRIVING** | `[S3-LINE] … FOLLOWING`; after the nudge `GRACE -> SEARCHING -> FOLLOWING` |
| S3-H02 | Barcode shuttle: the same code read again and again in both directions | R3-4 | 1. Build `build-barcode-shuttle` (line steering profile + `-DCMAKE_C_FLAGS="-DROBO_BARCODE_STREAM=1 -DROBO_BARCODE_SHUTTLE=1"`, no Wi-Fi, so only A and Z are read) and flash it. 2. Tape the small `*Z*` on the floor; put the robot ~7 cm before it, pointing across the bars, with ~40 cm clear. 3. Power on and save the serial log: it drives 40 cm forward and back 20 times (40 passes). 4. Score with `python tools/barcode_log.py --expect Z FILE` | — (no Wi-Fi in this image) | `shuttle: 20 round trips …`, `barcode: *Z*` per pass, `shuttle: done`; score ≥ 95 % (≥ 38/40), no other letter |
| S3-H03 | Barcode read while following the line, then its action at the junction | R3-4, R3-6 | Course with `*Z*` across the line before a junction (Z = left in the built-in route); START | **Barcode** shows **Z → turn left**; the robot turns left at the junction and keeps following the line | `barcode: *Z*`, then `route: junction after *Z*, …` and the turn |
| S3-H04 | Route letters from the dashboard drive the car | R3-5, R3-6 | MQTT image, hotspot + broker + `web_dashboard.py`. 1. Type the four letters in the Left / Right / Straight / U-turn boxes (e.g. Z, B, C, A) and press **Send route**. 2. START and drive over one of the codes before a junction. 3. Optional: send the same letter twice (e.g. Left Z, Right Z); otherwise rejection is covered by S3-U02. Can be the same run as S3-H03, with the dashboard overlaid; give the code an action that differs from the built-in route (e.g. Right = Z) | 1. Event log `robot route: ok L=Z R=B S=C U=A`. 2. **Barcode** shows the letter and the action from the sent route; the robot does that action at the junction. 3. (if done) `robot route: rejected …`, old route kept | `route: ok …`; `barcode: *X*`; on a straight letter `route: straight on, junction ignored` |
| S3-H05 | An undecodable barcode is ignored and the car carries on (**log only, no video**) | R3-7 | Any run where a pass cannot be decoded (e.g. a code not in the route) before a junction; save the serial log | Motion stays **DRIVING**; no turn at the junction after that code | `barcode: no read at any sensitivity …`, then `route: barcode read failed, carrying on` |

## Running the video tests

1. **Commit first.** Note `git rev-parse --short HEAD`; it goes in the video
   and the results row.
2. **Build the image** the test names: the shuttle image for S3-H02, the
   MQTT production image (VS Code "Compile Project" with `config/wifi_secrets.h`
   present) for the rest. Flash with motor power disconnected.
3. **Start the broker and dashboard on the laptop** (MQTT image only):
   ```sh
   mosquitto -c mosquitto.conf -v
   python tools/web_dashboard.py --host 192.168.137.1 --log testing/subsystem3/logs/<date>_S3-xxx_dashboard.log
   ```
4. **Start the serial log** too (`logs/<date>_S3-xxx.txt`) for the console-only items.
5. **Power up, place the robot on the line, then press START** for every run.

**Video format** (see `videos/README.md`): one test per video, 15–60 s,
landscape, unlisted YouTube. A longer video may cover several tests if each
test's start is shown with its ID.

1. Start: say or show the test ID and name, and the commit hash.
2. Show the setup: the barcode or route letters, the robot on the line.
3. Do the action.
4. End holding on the dashboard or console line that proves the result, then
   say PASS or FAIL.

Keep the robot, the dashboard and the serial terminal in one frame.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-10 | S3-U01 | uncommitted (on `e8f5da0`) | Claude | PASS | `ctest -R barcode_decode_checks` | Synthetic 80/80 Z. Full logs: small Z 16/17 (`logs/2026-11-06_barcode_run_145806.txt`), 10/40 (`logs/2026-11-06_barcode_run_143512.txt`, white ≈ 680); big A/Z reads kept (27, 38, 40, 8); 0 wrong letters. Old decoder: small 0/57. |
| 2026-11-10 | S3-H02 | uncommitted | fatmosthegreat / Claude | FAIL → re-tuned | `logs/2026-11-10_shuttle_small_A.txt` | Shuttle at 0.05 m/s over the small A: firmware decoded 1/9 (narrow bars +15..20 over the dips beside them). Bar-end rule changed to a third of the fall and a lower in-pass bar threshold: replay now 9/9, 0 wrong. Needs a fresh hardware run. |
| 2026-11-18 | S3-U02 | uncommitted (on `bdfff3e`) | Claude | PASS | `ctest -R "route_checks|barcode_decode_route4"` | Letters AZBC with ladder: 0 wrong letters. |
| 2026-11-06 | S3-H03 | not recorded | fatmosthegreat | PASS | `logs/2026-11-06_line_barcode_junction.txt` | `barcode: *Z*`, then `route: junction after Z, path 468 mm` and again at 1161 mm; car kept following the line. Older firmware (before the MQTT route). |
| 2026-11-12 | S3-H05 | not recorded | fatmosthegreat | PASS | `logs/2026-11-12_course_failed_read_stop.txt` lines 84–86 | A pass that did not decode gave `[S3-LINE] BARCODE -> STOPPED` and `route: barcode read failed, car stopped`. Cause was a misread real code, not a letter outside the route as the steps describe. Behaviour changed 2026-11-24 (failed reads no longer stop the car): this row is the old behaviour; rerun S3-H05. |
| 2026-11-12 | S3-H01 | not recorded | fatmosthegreat | PASS (partial) | `logs/2026-11-12_course_line_recovery.txt`, `logs/2026-11-12_course_failed_read_stop.txt` | Line lost on the course and found again every time (`GRACE -> SEARCHING -> FOLLOWING`, 13 times). Natural losses on the course; the line-following video adds the deliberate nudge. |
| 2026-11-24 | S3-U01 | `d0d4033` | Claude | PASS | `ctest -R barcode_decode` | All 6 decoder tests pass, including the 4-letter route set. |
| 2026-11-24 | S3-U02 | `d0d4033` | Claude | PASS | `ctest -R route_checks` | `route_checks passed (0 failures)`. |
| 2026-11-24 | S3-U03 | `d0d4033` | Claude Code for 2500622 | PASS | `logs/2026-11-24_host.txt` | `line_test_checks`, `line_pid_checks` passed (built with `-Wno-error=float-conversion`, see Script tests); same run: all barcode and route tests, 9/9 |
| 2026-11-24 | S3-H01 | not recorded | 2500622 | PASS | <https://youtu.be/t8ZaVSFGggE> | Line following on straights and curves; line lost and found again |
| 2026-11-24 | S3-H02 | not recorded | 2500622 | PASS | <https://www.youtube.com/shorts/dHfarWP__Gw> | Barcode shuttle over `*Z*`, read in both directions; pass score not recorded |
| 2026-11-24 | S3-H03 | not recorded | 2500622 | PASS | <https://youtu.be/GsEvuXFR86U> | Read `*Z*` while following the line, turned left at the junction; then read `*A*` and did a U-turn. Same video as S3-H04 |
| 2026-11-24 | S3-H04 | not recorded | 2500622 | PASS | <https://youtu.be/GsEvuXFR86U> | Route sent from the dashboard boxes (screen overlay, same run): Z → left at the junction, A → U-turn. Same letters as the built-in route, so the overlay's `robot route: ok` is what shows the dashboard route was used. Rejection covered by S3-U02 |
