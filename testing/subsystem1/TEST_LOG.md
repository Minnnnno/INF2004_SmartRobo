# Subsystem 1 — test log

**Subsystem:** WiFi communication, command and telemetry (Buddy 1: Wi-Fi, MQTT, web dashboard)  ·  **Student ID:** 2500842  ·  **Code:** `app_program/subsystem1/`, `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, `tools/web_dashboard.py`, `tools/dashboard.py`

See `testing/README.md` for how to run tests and record results. The frame
layout is in `include/telemetry_frame.h`; the topics and commands are in `docs/API.md`.

Buddy 1 moves data and commands; it does not create them. These tests check
the **transport** in isolation: a value sent by another buddy arrives
unchanged, on time, and the robot never waits on the network. Whether that
value is *correct* (a hump height, a wheel speed) is the other buddy's test.
The interfaces with each buddy are integration tests (`../integration/TEST_LOG.md`).

Two kinds of test (the IDs keep the letters they were given in October, so they
differ from Buddy 4's):

- **Laptop tests (S1-H..)** run the Buddy 1 C code and the dashboard decoder on
  the laptop. They prove the **logic** (every field, every command word,
  every malformed frame). Evidence: the saved console output. No video.
- **Robot tests (S1-U..)** run on the Pico W with the web dashboard. They
  prove the **requirement is met on the real hardware**. Evidence: a video,
  plus the dashboard and serial logs. S1-U01 to U04, U09 and U10 were
  firmware self-tests (see the note under *Robot tests*).

## Requirements

Sources: **Briefing** = the Project Briefing (mission 5 REPORT: status and
telemetry); **FR/NFR** = the team's Week 6 Design Review; **Tribunal 1** = the
compact-representation review (21-byte frame instead of JSON); **B2 handoff** =
`docs/BUDDY1_HANDOFF_README.md`; **B4 CR** = `docs/BUDDY4_FRAME_CHANGE_REQUEST.md`;
**B3 bug** = barcode not updating on the dashboard (fixed in layout v3);
**Start gate** = robot boots held until the dashboard START (18 Nov 2026).

| Req | Requirement | Source | Laptop test | Robot test |
|---|---|---|---|---|
| R1-1 | Join Wi-Fi and the MQTT broker; announce online/offline (retained status, last will) and send a heartbeat every 1 s | Briefing mission 5; FR15 | — | S1-U05, S1-U08 |
| R1-2 | Send every subsystem's status at 10 Hz in a compact binary frame the dashboard decodes | Briefing mission 5; FR16; Tribunal 1 | S1-H01, S1-H02, S1-H05 | S1-U05, S1-U15 |
| R1-3 | Carry the fields other buddies asked for: B4 motion class and turn rate (v2), B3 barcode reads (v3), B2 encoder counts (encoder frame) | FR16; B4 CR; B3 bug; B2 handoff | S1-H01, S1-H03, S1-H04 | S1-U13, S1-U14, S1-U16 |
| R1-4 | Remote START / STOP / E-STOP: only valid commands reach the Vehicle Controller queue, repeats of START are ignored, every command is acknowledged; the robot boots held until START | FR17; Start gate | S1-H01, S1-H05 | S1-U06, S1-U17, S1-U21 |
| R1-5 | Send the barcode route from the dashboard to the robot and report whether it was accepted | FR17; Briefing mission 2 | S1-H05 | S1-U18 |
| R1-6 | Recover by itself after the broker or the Wi-Fi is lost | FR18 | — | S1-U07, S1-U19 |
| R1-7 | Never block or slow the motion task: bounded queues, a full queue drops and counts, oversized payloads refused, small copy time | NFR6, NFR8 | — | S1-U12, S1-U19 |
| R1-8 | Dashboard shows events once and marks stale or offline data | FR15, FR16; B2 handoff item 5 | — | S1-U11, S1-U20 |

*Check the FR/NFR numbers against the Design Review before hand-in: this table
uses the numbers already in the logs (FR15 status, FR16 telemetry, FR17
commands, FR18 reconnect, NFR6 timing, NFR8 bounded messages).*

## Laptop tests (no robot)

```sh
cmake -S . -B build-host-cmake -G Ninja -DROBO_HOST_TESTS=ON
cmake --build build-host-cmake --target buddy1_frame_checks
./build-host-cmake/buddy1_frame_checks > testing/subsystem1/logs/<date>_host.txt
python tools/dashboard.py selftest > testing/subsystem1/logs/<date>_dashboard_selftest.txt
python tests/buddy1_frame/web_dashboard_check.py > testing/subsystem1/logs/<date>_web_dashboard_demo.txt
```

| ID | What is tested | Requirement | Expected result (in the log) |
|---|---|---|---|
| S1-H01 | Every field, clamp, malformed frame, all 65,536 command words, snapshot mapping, encoder frame (32-bit counts, wrap, freshness), JSON vs packed size | R1-2, R1-3, R1-4 | `PASS: <n> checks, 0 failed` (947 at layout v3) |
| S1-H02 | Dashboard decoder agrees with the firmware codec (same golden status and encoder frames) | R1-2 | `PASS: dashboard selftest, 0 failures` |
| S1-H03 | Layout v2 Buddy 4 fields: `M_CLASS`/`TURN_RATE` mapping from slot bit 9, every class code, `TURN_RATE` −256..255, `HUMP_MM` 127 + clamp, v1 frame rejected | R1-3 | `test_v2_extremes`, `test_terrain_v2_mapping` pass inside S1-H01 |
| S1-H04 | Layout v3 barcode: every Code 39 character maps from B3's ASCII letter, `BC_READ` flips per read, the same letter read twice raises two barcode events, failed reads/junctions do not, v1/v2 frames rejected | R1-3 | `test_barcode_mapping`, `test_edges` pass inside S1-H01; `167 bits used, 1 spare` |
| S1-H05 | Web dashboard against its simulated robot (`web_dashboard.py --demo`), driven through the same HTTP API as the browser buttons: status and encoder frames at 10 Hz with none lost, fresh heartbeat, layout v3; START, STOP, E-STOP sent with rising sequence numbers and acknowledged; E-STOP shown as a latched fault START cannot clear; unknown command and empty route refused; route acknowledged | R1-2, R1-4, R1-5 | 12 `PASS` lines, ending `PASS: web dashboard demo, 12 passed, 0 failed` |

S1-H05 tests the dashboard, not the robot: the simulator always accepts a route
and never goes silent, so a rejected route (S1-U18) and stale marking (S1-U20)
still need the robot.

The Start gate's controller side (`web_start_gate_tests` in `tests/test_motion.c`)
belongs to Buddy 2's `motion_tests` suite; it is evidence for INT-09, not a Buddy 1 test.

## Robot tests (robot + dashboard)

**Isolation:** unless the step says otherwise, put the robot **on a stand with
the wheels off the ground** and do not press START, so only Buddy 1's
behaviour is observed. "Dashboard" = `tools/web_dashboard.py` in the browser:
the **Link** card (Broker, Heartbeat, Status frames, Encoder frames,
Lost / bad, Frame layout), the status pills at the top, the buttons, the route
form and the log. The last column lists what the dashboard cannot show; read
it from the serial console, kept in the same video frame.

**Firmware self-tests (S1-U01 to U04, U09, U10):** these ran in the `TEST=ON`
image (`subsystem1_test.c`). The integrated firmware rejects `TEST=ON`
(`docs/BUILD.md`), so they can no longer be re-run on the robot. Their
October results stand. The same logic is now covered by S1-H01 (U04 and U10)
and observed on the robot by S1-U17 (U04, U06) and S1-U15 (U01, U02).

| ID | What is tested | Requirement | Steps | Expected on the dashboard | Console only |
|---|---|---|---|---|---|
| S1-U01 | Telemetry is queued while there is room | R1-2 | firmware self-test (`subsystem1_test.c`) | — | `[TEST] S1-U01 PASS` |
| S1-U02 | A full queue drops the message without waiting, and counts it | R1-7 | firmware self-test | — | `[TEST] S1-U02 PASS` |
| S1-U03 | An oversized payload is refused | R1-7 | firmware self-test | — | `[TEST] S1-U03 PASS` |
| S1-U04 | A valid binary command reaches `mbf_comms_cmd`; wrong length, truncated and reserved opcodes do not | R1-4 | firmware self-test | — | `[TEST] S1-U04 PASS` |
| S1-U05 | Status, heartbeat and 10 Hz frames reach the broker and decode | R1-1, R1-2 | 1. Flash the MQTT image. 2. `python tools/dashboard.py listen --host <broker> --csv logs/<date>_frames.csv` | — (terminal tool) | retained `robot/status online`, `robot/heartbeat` every 1 s, `robot/frame` decoded every 100 ms, `lost=0` |
| S1-U06 | Binary commands are acknowledged | R1-4 | `python tools/dashboard.py send stop --host <broker>` while `listen` runs | — (terminal tool) | `ACK ok opcode=2` |
| S1-U07 | Reconnects after the broker is lost | R1-6 | 1. Stop the broker for 20 s. 2. Start it again. Watch the USB console | Link card Broker goes down then **connected**; pill **robot online** again; log `status frames resumed` | `mqtt: retrying in 1000 ms`, 2000, 4000, 8000, 10000, then `mqtt: connected to ...` |
| S1-U08 | Broker reports the robot offline | R1-1 | Unplug the Pico while subscribed | Pill **robot offline** after the keep-alive expires (up to ~90 s) | `robot/status offline` (in `listen`) |
| S1-U09 | Repeated or stale START is not forwarded; STOP always is | R1-4 | firmware self-test | — | `[TEST] S1-U09 PASS` |
| S1-U10 | 21-byte frame round trip keeps negatives, clamps out-of-range | R1-2 | firmware self-test | — | `[TEST] S1-U10 PASS` |
| S1-U11 | Latched events are published once and merged | R1-8 | With the dashboard running, trigger a line loss (lift the IR sensor off the line) twice quickly | Log shows one `event: ...` line per 100 ms period, each event once | — |
| S1-U12 | Packed telemetry does not disturb motion timing | R1-7 | Run normally for 1 min, stop the broker 20 s, start it again; read the `comms:` line every 10 s | Motion card never shows a FAULT | `comms:` line: `skipped` stays 0, `copy_us_max` well under 10000 (the 10 ms motion period); no motion timing fault |
| S1-U13 | Dashboard shows Buddy 4 motion class and turn rate | R1-3 | Robot on the floor, START. Drive straight, turn right, turn left, cross a hump, overturn by hand | IMU & terrain card *Motion class* STATIONARY → MOVING → TURNING; *Turn rate* negative on right turns, positive on left; **Highest hump** rises after the hump; **OVERTURNED** when overturned | matching `[MOTION]` lines |
| S1-U14 | Dashboard updates on every barcode read | R1-3 | Robot on the floor. Drive over the same barcode twice | Line & barcode card shows the letter and its route action; read count rises by 1 each pass | one `barcode read:` line per pass |
| S1-U15 | Telemetry rate and loss over 60 s | R1-2 | Robot on a stand, dashboard open. Note **Status frames** and **Lost / bad** on the Link card; wait 60 s on a stopwatch; note them again | **Status frames** up by 600 ± 10; **Encoder frames** up by the same; **Lost** up by ≤ 6 (1 %), **bad** stays 0; Heartbeat **< 1.5 s ago**; Frame layout **v3**; pill **telemetry live** throughout | `comms:` line: `skipped=0`, `dropped` not rising |
| S1-U16 | Encoder counts reach the dashboard | R1-3 | Robot on a stand, motors not driving. Turn the left wheel forward by hand, then backward; repeat with the right wheel | Encoders card: only the turned wheel's count changes, and it changes the other way when turned backward; **Sample age** ≤ 100 ms (not *unknown / old*) | — |
| S1-U17 | Dashboard START / STOP / E-STOP reach the robot and are acknowledged | R1-4 | Robot on a stand. 1. Press START. 2. Press STOP. 3. Press E-STOP. 4. Press START again. 5. Power-cycle to clear the E-STOP | 1. Log `robot received START #n: queued for the controller`; Motion card leaves **STOPPED (held)**. 2. `received STOP #n: queued ...`; Motion **STOPPED (held)**. 3. `received ESTOP #n: queued ...`; Motion fault **ESTOP**; Troubleshooting shows *E-STOP is latched*. 4. START acknowledged but the robot stays stopped (START does not clear E-STOP) | `mqtt: RX [robot/cmd] ...` once per press |
| S1-U18 | Barcode route is sent, acknowledged and kept across a reboot | R1-5 | 1. In the Line & barcode card's route boxes type Left **B**, Right **C**, Straight **D**, U-turn **A**; press *Send route*. 2. Type **B** in both Left and Right (letter used twice), D and A in the others; send. 3. Power-cycle the robot | 1. Log `sent route ...` then `robot route: ok L=B R=C S=D U=A`. 2. `robot route: rejected L=B R=B S=D U=A`; the route stays the step 1 route. 3. After reconnecting, `robot route: ok L=B R=C S=D U=A` again (retained) | `route: ok ...`, `route: rejected ...` |
| S1-U19 | Recovers after the Wi-Fi is lost | R1-6, R1-7 | Robot on a stand, dashboard open. Switch the hotspot off for 15 s, then on. Wait up to 60 s | Pill **telemetry stale** during the outage; then **robot online**, **telemetry live**, log `status frames resumed`, without touching the robot; Motion card never shows a FAULT | `wifi: link status -> ...`, `wifi: link up, ip=...`, `mqtt: connected to ...` |
| S1-U20 | Dashboard marks stale data | R1-8 | Dashboard running with live telemetry. Unplug the robot's USB/battery. After 10 s plug it back in | Within ~1 s: pill **telemetry stale**, Motion/Line/IMU/Obstacle cards greyed out, log `no status frames for 1.x s` and `no encoder frames ...`; after ~3 s `no heartbeat ...`. After power-up: `status frames resumed`, pill **telemetry live** | — |
| S1-U21 | Robot boots held and moves only after the dashboard START (Start gate, `8ca46a0`) | R1-4 | Robot **on a stand**, current MQTT image, dashboard open. 1. Power up; wait 30 s, then press STOP and wait 10 s. 2. Press START. 3. Press STOP | 1. Wheels never turn; Motion **STOPPED (held)**, wheel speed **0 / 0** the whole time, STOP acknowledged. 2. Log `robot received START #n: queued for the controller`; Motion leaves **held** (no *START had no effect* warning). 3. STOP acknowledged; Motion back to **STOPPED (held)**, wheels stop | `mqtt: RX [robot/cmd] ...` per press |

## Running the robot tests

1. **Commit first.** Note `git rev-parse --short HEAD`; it goes in the video
   and the results row.
2. **Build the MQTT production image**: VS Code "Compile Project"
   (`ROBO_NETWORK=AUTO`, `config/wifi_secrets.h` present) → `build/mtkernel_3.uf2`.
   Hold BOOTSEL, plug in the Pico, copy the `.uf2` across. Flash with motor
   power disconnected.
3. **Start the broker and dashboard on the laptop:**
   ```sh
   mosquitto -c tools/mosquitto_robot.conf -v
   python tools/web_dashboard.py --host 127.0.0.1 --log testing/subsystem1/logs/<date>_S1-Uxx_dashboard.log
   ```
   Open the address it prints (port 8080); check the Link card counts rising.
4. **Start the serial log** (PuTTY *Session → Logging → All session output*,
   `logs/<date>_S1-Uxx.txt`) for the console-only items.
5. **Video:** say the test ID and commit, show the robot (on its stand), the
   dashboard and the serial console in one frame, do the steps, end on the
   value that proves the result and say PASS or FAIL. Unlisted YouTube link in
   the results row (see `videos/README.md`). One video may cover several tests
   if each start is shown with its ID; S1-U19 and S1-U20 fit in one clip, and
   S1-U15 to U18 and U21 in another.
6. **Keep the dashboard log.** Always start `web_dashboard.py` with `--log`;
   without it the dashboard side is lost when the window closes (this is what
   happened to the 17 November session).

## Other logs in this folder

- `logs/2026-11-17_console.txt`: serial capture of the 17 November dashboard
  session, committed in `b7f0afb` as `2026-11-04_console.txt` and renamed on
  24 November to its real date. Firmware stamp `20261114`: the image was built
  before `8ca46a0` (Start gate, STOP hold, barcode fix), so it is **not** the
  current build. No dashboard log or video was kept (start
  `web_dashboard.py` with `--log` next time). Recorded below as S1-U12 PASS
  and S1-U07 PARTIAL; nothing in it fits an integration or system test (no
  dashboard side to compare, no mid-run Wi-Fi loss).
- `logs/2026-11-04_frames.csv`: a `listen --csv` frame log from 4 November
  (layout v1, robot idle). Not a test result; S1-U05 needs a fresh run.

## Recording plan (2026-11-25)

Nine videos cover every Buddy 1 robot, integration and system test. Videos
1–5 are Buddy 1 alone; 6–9 are team runs where Buddy 1's dashboard must be in
frame. The *Results* rows below, the integration and system logs and
`videos/README.md` already have a row for each, with `VIDEO-n-LINK`,
`m:ss` and **TO RECORD** in place of the link, the timestamp and the result.

**After uploading:** in VS Code press Ctrl+Shift+H (replace in files) and
replace each `VIDEO-n-LINK` (n = 1 to 9) with that video's YouTube link, one
video at a time: it fills in every file at once. Then, row by row, replace
`m:ss` with the time the test starts in the video, fill in the `___` blanks,
and replace each
**TO RECORD** with PASS or FAIL. A FAIL needs a note saying what happened. A
test you did not get to: delete its row (it was never a result).

**Before leaving home:**

- [ ] Build and flash the MQTT image from this branch (VS Code "Compile Project"). Firmware = `b51982f`; if you flash anything newer, replace `b51982f` in the new rows.
- [ ] `git rev-parse --short HEAD` in a terminal you can show on camera.
- [ ] Pack the stand, a ruler, the laptop charger and the phone (DroidCam). OBS scene: browser + serial console + phone camera.

**At school, every video:** start PuTTY logging (`logs/2026-11-25_Vn.txt`) and
the dashboard with `--log testing/subsystem1/logs/2026-11-25_Vn_dashboard.log`.
Start each test by saying *"S1-Uxx, &lt;name&gt;, commit b51982f"*, and end each on
the value that proves it, saying PASS or FAIL.

| Video | Tests, in order | Set-up | What to do (details: the test tables above) |
|---|---|---|---|
| 1 Start-up and buttons | S1-U21, S1-U17 | Stand | Power up, stopwatch 30 s held, STOP, 10 s, START, STOP. Then START, STOP, E-STOP, START (stays stopped). Power-cycle |
| 2 Telemetry and route | S1-U15, S1-U16, S1-U18, S1-U12 | Stand | Read *Status frames* / *Lost* on the Link card, 60 s stopwatch, read again. Turn each wheel forward and back by hand. Route boxes B C D A → send; B B D A → send (rejected); power-cycle and show the route ack again. Zoom on a `comms:` console line |
| 3 Terminal tools | S1-U05, S1-U06 | Stand | Terminal: `python tools/dashboard.py listen --host 127.0.0.1 --csv testing/subsystem1/logs/2026-11-25_frames.csv`; second terminal: `python tools/dashboard.py send stop --host 127.0.0.1` |
| 4 Connection loss | S1-U07, S1-U19, S1-U20, S1-U08 | Stand | Stop mosquitto 20 s, start it (console back-off 1, 2, 4, 8, 10 s). Hotspot off 15 s, on. Unplug the robot: stale within 1 s; keep `listen` open until `robot/status offline` (up to 90 s); plug back in |
| 5 On the course | S1-U13, S1-U14, S1-U11 | Floor, course | START; straight, right turn, left turn, hump; same barcode twice; lift the IR sensor off the line twice quickly; overturn by hand to end |
| 6 Integrated full run (team) | INT-04, INT-08, INT-11, INT-13, SYS-01, SYS-07 | Full course: barcodes, obstacle, hump | Power up, 30 s held, START. Keep the **whole dashboard and the console** in frame for the full run. Lift to end. Each buddy checks their card |
| 7 Route (with B3) | INT-12 | Junction, code B before it | Send B C D A; START; robot turns left after B. Send C B D A; power-cycle; START; turns right |
| 8 STOP mid-run (with B2) | INT-09 | Course | While line following: STOP, wait 5 s after the wheels stop, START. Shares footage with SYS-09 |
| 9 Wi-Fi loss mid-run | SYS-08 | Course | While driving: hotspot off 10 s, then on. Robot keeps driving; dashboard stale, then reconnects by itself |

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
| 2026-11-14 | S1-H01 | not committed yet | Claude Code for 2500842 | PASS | console: `PASS: 935 checks, 0 failed`; `size: JSON 395 bytes, packed 21 bytes (163 bits used, 5 spare), 5.3% of JSON` | layout v2; MinGW gcc, -Wall -Wextra -Werror -Wconversion -Wshadow |
| 2026-11-14 | S1-H02 | not committed yet | Claude Code for 2500842 | PASS | console: `PASS: dashboard selftest, 0 failures` | layout v2 golden `20730394…7b00` matches the C test |
| 2026-11-14 | S1-H03 | not committed yet | Claude Code for 2500842 | PASS | same run as S1-H01 | |
| 2026-11-24 | S1-H01 | `b51982f` | Claude Code for 2500842 | PASS | `logs/2026-11-24_host.txt` | `PASS: 947 checks, 0 failed`; layout v3, `167 bits used, 1 spare`, 5.3 % of JSON |
| 2026-11-24 | S1-H02 | `b51982f` | Claude Code for 2500842 | PASS | `logs/2026-11-24_dashboard_selftest.txt` | `PASS: dashboard selftest, 0 failures` |
| 2026-11-24 | S1-H03 | `b51982f` | Claude Code for 2500842 | PASS | `logs/2026-11-24_host.txt` | same run as S1-H01 |
| 2026-11-24 | S1-H04 | `b51982f` | Claude Code for 2500842 | PASS | `logs/2026-11-24_host.txt` | same run as S1-H01 |
| 2026-11-17 | S1-U12 | image built 2026-11-14 (before `8ca46a0`) | 2500842 | PASS | `logs/2026-11-17_console.txt` | Recorded 2026-11-24 from the saved capture. ~8 min of telemetry (frames 2200 → 7100): `skipped=0` throughout, `copy_us_max=675` µs (6.8 % of the 10 ms motion period), no motion fault. `dropped` 5 → 147, mostly the burst around the broker reconnect (`mqtt: 44 messages dropped in last 5 s`): drops instead of blocking, as designed. Not the current build: re-run |
| 2026-11-17 | S1-U07 | image built 2026-11-14 (before `8ca46a0`) | 2500842 | PARTIAL | `logs/2026-11-17_console.txt` (lines 2724–2729) | Recorded 2026-11-24 from the saved capture. One unplanned broker drop (`mqtt: connection status 256`), `mqtt: retrying in 1000 ms`, then `mqtt: connected` and re-subscribed by itself. The full 1, 2, 4, 8, 10 s back-off after a 20 s broker stop was not shown. Re-run on the current build |
| 2026-11-24 | S1-H05 | `b51982f` + new `tests/buddy1_frame/web_dashboard_check.py` (uncommitted; `tools/` unchanged) | Claude Code for 2500842 | PASS | `logs/2026-11-24_web_dashboard_demo.txt` | `PASS: web dashboard demo, 12 passed, 0 failed`; Python 3.14 |
| 2026-11-25 | S1-U21 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-1-LINK> @ m:ss, `logs/2026-11-25_V1.txt` | Boots held 30 s, START releases, STOP holds |
| 2026-11-25 | S1-U17 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-1-LINK> @ m:ss, `logs/2026-11-25_V1_dashboard.log` | START / STOP / E-STOP acknowledged; START does not clear E-STOP |
| 2026-11-25 | S1-U15 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-2-LINK> @ m:ss, `logs/2026-11-25_V2_dashboard.log` | Status frames +___ in 60 s, lost +___, bad +___ |
| 2026-11-25 | S1-U16 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-2-LINK> @ m:ss | Encoder counts follow each wheel turned by hand |
| 2026-11-25 | S1-U18 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-2-LINK> @ m:ss, `logs/2026-11-25_V2.txt` | Route B C D A ok, B B D A rejected, route acknowledged again after power-cycle |
| 2026-11-25 | S1-U12 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-2-LINK> @ m:ss, `logs/2026-11-25_V2.txt` | `comms:` skipped=___, copy_us_max=___ µs; current build |
| 2026-11-25 | S1-U05 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-3-LINK> @ m:ss, `logs/2026-11-25_frames.csv` | status online, heartbeat 1 s, frames every 100 ms, lost=___ |
| 2026-11-25 | S1-U06 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-3-LINK> @ m:ss | `ACK ok opcode=2` |
| 2026-11-25 | S1-U07 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-4-LINK> @ m:ss, `logs/2026-11-25_V4.txt` | Broker stopped 20 s; back-off 1, 2, 4, 8, 10 s, reconnected by itself |
| 2026-11-25 | S1-U19 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-4-LINK> @ m:ss, `logs/2026-11-25_V4.txt` | Hotspot off 15 s; back online after ___ s without touching the robot |
| 2026-11-25 | S1-U20 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-4-LINK> @ m:ss, `logs/2026-11-25_V4_dashboard.log` | Telemetry stale within ~1 s of unplugging; resumed after power-up |
| 2026-11-25 | S1-U08 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-4-LINK> @ m:ss | `robot/status offline` after ___ s |
| 2026-11-25 | S1-U13 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-5-LINK> @ m:ss | Motion class and turn-rate sign follow the run; OVERTURNED at the end |
| 2026-11-25 | S1-U14 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-5-LINK> @ m:ss | Same barcode twice: read count +1 per pass |
| 2026-11-25 | S1-U11 | `b51982f` | 2500842 | **TO RECORD** | <VIDEO-5-LINK> @ m:ss, `logs/2026-11-25_V5_dashboard.log` | Two quick line losses: each `event: line lost` once |
