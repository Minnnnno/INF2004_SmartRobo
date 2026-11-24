# Buddy 3 compact representation (Tribunal 1 follow-ups)

Date: 24 November 2026. Student ID: 2500622. Scope: Buddy 3 line following and
barcode (`app_program/subsystem3/`) only. No changes to the controller, telemetry
frame, MQTT, dashboard or Buddies 1/2/4/5.

Source: the Week 8 Tribunal 1 document (Buddy 3 rows of the change log, curveball
response, interface map and assumptions A3/A4). Each item was checked against the
current code. The document was not applied wholesale, and **not breaking working
code** came first, so there is only one small code change.

## Implemented: IR readings packed into one bit each

Tribunal change log: "pack each sensor's line-detection result into one bit using
masks and shifts; verify all four left/right combinations".

- `line_read_ir()` in `line_following.c` reads both digital IR sensors once and
  returns one byte with one bit per sensor.
- GP26 is still read before GP27.
- It is used in the line loop and in the route pivot. Both then derive the same
  0/1 values as before, so the behaviour and console output are unchanged.
- The IR-monitor diagnostic images are unchanged.

| Bit | Mask | Sensor |
|---:|---|---|
| 0 | `LINE_IR_RIGHT` | GP26, right (offset right of centre) |
| 1 | `LINE_IR_CENTRE` | GP27, centre |
| 2–7 | — | unused, always 0 |

Four combinations, as mapped by Buddy 2's `motion_line_direction()` after 2 equal
reads (debounce):

| Centre | Right | Direction | Covered by `line_pid_checks` |
|---:|---:|---|---|
| 0 | 0 | LOST → grace, then search | yes |
| 1 | 0 | LEFT (line is to the left) | yes |
| 0 | 1 | RIGHT (line is to the right) | yes |
| 1 | 1 | STRAIGHT | yes |

The existing `tests/test_line_pid.c` table already holds each combination for at
least 2 reads. Its fake `gpio_get` also checks that GP26 is read before GP27 on
every tick, so the helper is tested through the real `line_task` loop without
changing the test.

## Already minimum-bit (no change)

| Item | Representation | Where |
|---|---|---|
| Line error | 2 bits, −1/0/+1 (three-level error kept: digital sensors give no finer information) | `include/telemetry_frame.h` `LINE_ERR` |
| Line status | `LINE_AVAIL` 1 bit, `LINE_TRACK` 1 bit | `include/telemetry_frame.h` |
| Code 39 symbols | each character's 9 bars/spaces as a 9-bit pattern | `barcode_decode.c` `code39_pattern[]` |
| Barcode result | `BC_CHAR` 6 bits (44 characters) + `BC_READ` 1 bit | `include/telemetry_frame.h` |
| Route letters | all four letters in one 32-bit word, one store | `route.c` |
| Pending route action | one byte; 0 = none acts as the validity flag | `line_following.c` `route_pending` |

## Already met (Tribunal interface rules)

| Rule | How |
|---|---|
| Single ADC owner | Line sensors are digital GPIO; only `barcode_task` uses the ADC (GP28) |
| No motor calls from B3 | Motion goes through `motion_steer*` and the Vehicle Controller |
| No-line / invalid code / missing code kept apart | `LINE_TRACK`=0; failed-read counter stops the car; a junction only counts after a read |
| Stop if navigation is unreliable | A failed barcode read stops the car |
| Acknowledged B5 → B3 handover | `line_rejoin_seen()`, `TF_EV_HANDOVER` |
| Shared updates protected | Bus writes via `robot_update_subsystem()` under the bus mutex; route word written in one store |
| A4: samples per narrowest bar | 10 ms at 0.07 m/s = 0.7 mm per sample: small narrow bar (~3 mm) ≈ 4 samples, big (~6 mm) ≈ 8; widths come from encoder distance |

## Not adopted (with reasons)

| Tribunal item | Reason |
|---|---|
| 2-bit `BC_CMD` + valid bit in the telemetry frame | Buddy 1's frame v3 replaced it with `BC_CHAR` + `BC_READ`; only 1 spare bit; the dashboard shows the action from the acknowledged route |
| Bit-field for the pending action | Already one byte in a padded struct: no RAM saved, and a second encoding beside `route_action_t` |
| Separate ADC acquisition task | One ADC user only; would add scheduling cost |
| Smaller decoder buffers | Possible: `path[1000]` `uint32_t` → `uint8_t` step (~3 KB), `heading[1000]` `uint16_t` → `int8_t` (~1 KB). Deferred: the decoder is tuned and working |

## Still open (robot tests, no code)

- **A3: start without a visible line.** In code the robot doesn't move until it
  sees the line, because the search runs only once already driving. Not yet tried
  at different alignments and lighting.
- **All four commands on hardware.** Left and U-turn are shown on video; Right and
  Straight are tested only on the laptop. Run one of each on the course and save
  the serial log here.

## Evidence

`testing/subsystem3/logs/compact_representation_2026-11-24/`:
- `host.txt`: line, barcode and route host tests on this branch.
- `integration.txt`: integration check with all buddies' latest code (this branch
  trial-merged onto main, not committed): no conflicts, 24/24 team host tests,
  Buddy 1 dashboard selftest, all Buddy 4 suites, and the integrated firmware
  with and without MQTT built with no warnings. Buddy 2's compact and
  forward-console suites need tools this laptop lacks (UB sanitizer, clang).
- Robot logs (line-following run on the packed reads, and the two open tests)
  are added when they are run.
