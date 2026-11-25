# Videos

The index of every video of a test or demo on the real robot. A video shows
what a log cannot: the motor turning, the robot reaching the line, the
dashboard changing as it happens. Videos are the evidence for the robot tests
(`S1-..` to `S5-..`, `INT-..`, `SYS-..`) recorded in the test logs under
[`testing/`](../testing/README.md). List **every** video in the index at the
bottom of this page.

## What to record

- **Say what it is.** Show or say the test ID (e.g. `S4-H11`, `INT-03`) or the
  demo name at the start. One video may cover several tests if each test's start
  is shown with its ID.
- **Keep it short:** about 15–60 seconds per test.
- **Show the result.** Keep the robot and the dashboard in view, and the serial
  console too when the test's expected result is a console line.
- **Record the commit.** Test committed code, and put the commit
  (`git rev-parse --short HEAD`) in the index row and the test's result row.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the index below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's result row in the
matching test log:

| Tests | Test log |
|---|---|
| `S1-..` | [`testing/subsystem1/TEST_LOG.md`](../testing/subsystem1/TEST_LOG.md) |
| `S2-..` | [`testing/subsystem2/TEST_LOG.md`](../testing/subsystem2/TEST_LOG.md) |
| `S3-..` | [`testing/subsystem3/TEST_LOG.md`](../testing/subsystem3/TEST_LOG.md) |
| `S4-..` | [`testing/subsystem4/TEST_LOG.md`](../testing/subsystem4/TEST_LOG.md) |
| `S5-..` | [`testing/subsystem5/TEST_LOG.md`](../testing/subsystem5/TEST_LOG.md) |
| `INT-..` | [`testing/integration/TEST_LOG.md`](../testing/integration/TEST_LOG.md) |
| `SYS-..` | [`testing/system/TEST_LOG.md`](../testing/system/TEST_LOG.md) |

## Video index

Newest at the bottom. Never delete a row. One row per video: date, subsystem
(`S1`..`S5`, `INT`, `SYS`), the test IDs it shows, the commit, your student ID,
the link and one line on what it shows.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| 2026-11-19 | S4 | S4-H03, S4-H01 | main after `610725f` | 2500910 | <https://youtu.be/b0VxlQ2r320> | Overturn and lift while driving stop the motors |
| 2026-11-19 | S4 | S4-H08, S4-H11, S4-H04, S4-H12 | main after `610725f` | 2500910 | <https://youtu.be/O-3cIuu86ew> | Full run: calibration after START, 10 mm hump, motion classes and turn rate |
| 2026-11-19 | S4 | S4-H10 | main after `610725f` | 2500910 | <https://youtu.be/OaEkDdhVMTM> | Hand tilt: pitch, uphill/downhill state, hump states |
| 2026-11-24 | S3 | S3-H01 | not recorded | 2500622 | <https://youtu.be/t8ZaVSFGggE> | Line following and line recovery |
| 2026-11-24 | S3 | S3-H02 | not recorded | 2500622 | <https://www.youtube.com/shorts/dHfarWP__Gw> | Barcode shuttle, both directions |
| 2026-11-24 | S3 | S3-H03, S3-H04 | not recorded | 2500622 | <https://youtu.be/GsEvuXFR86U> | Route from the dashboard: `*Z*` → left at the junction, `*A*` → U-turn |
| 2026-11-24 | S2 | S2-H01 | Not supplied | <student ID> | <https://youtu.be/lzCOjuQXgig> | Startup and START; one trial; user-reported PASS; submitted 2026-11-24. |
| 2026-11-24 | S2 | S2-H02 | Not supplied | <student ID> | <https://youtu.be/qB7hPMGxUTQ> | Independent signed encoders; one trial; user-reported PASS; submitted 2026-11-24. |
| 2026-11-24 | S2 | S2-H03 | Not supplied | <student ID> | <https://youtu.be/8NpKcqakr0c> | Wheel-speed control; one trial; user-reported functional PASS; repeatability/accuracy not established; submitted 2026-11-24. |
| 2026-11-24 | S2 | S2-H04 | Not supplied | <student ID> | <https://youtu.be/yS6CX1FzbrM> | STOP/hold/restart; one trial; user-reported functional PASS; repeatability/latency not established; submitted 2026-11-24. |
| 2026-11-24 | S5 | S5-U01, S5-U02, S5-U03 | `fa36f67` + width scale (later `e4e5b3e`) | 2500344 | <https://youtu.be/I9SrSpBJh9Q> | Normal run with the dashboard: stops about 150 mm from the box, width estimate, goes round it and back to the line |
| 2026-11-24 | S5 | S5-U04 | `fa36f67` (not confirmed) | 2500344 | <https://youtu.be/RCM5-oiSxDs> | More clearance on the left: passes on the left |
| 2026-11-24 | S5 | S5-U05 | `fa36f67` (not confirmed) | 2500344 | <https://youtu.be/O0wmRNscU0k> | More clearance on the right: passes on the right |
| 2026-11-24 | S5 | S5-U06 | `fa36f67` (not confirmed) | 2500344 | <https://youtu.be/5Ype3yBMZK4> | Sudden obstacle after a turn: stops and goes round it |
| 2026-11-24 | S5 | S5-U07 | `fa36f67` + width scale (later `e4e5b3e`) | 2500344 | <https://youtu.be/FtWQj1oa1vA> | False halt: stops for a hand and scans; the clip ends before it drives on |
| 2026-11-24 | S4 | S4-H15, S4-H11, S4-H14, S4-H04 | `4033ff7` | 2500910 | <https://youtu.be/jCLuBsej2is> | Re-test after the turn-pause fix: no false humps on barcode turns, U-turns and pivots; 10 mm hump measured within ± 3 mm and kept; never LIFTED |
