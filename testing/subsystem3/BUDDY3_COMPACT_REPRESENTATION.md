# Buddy 3 data representation (line following and barcode)

Student ID: 2500622. Week 8 Tribunal 1 minimum-bit follow-ups for
`app_program/subsystem3/`. Docs only on `main`: the one code change (IR bits,
section 1) is on the team's minimum-bit branch `buddy2/minimum-bit-review`
(commit `109a511`) and is **not** merged into `main`, so the working robot code
is unchanged.

Each Tribunal item was checked against the current code and sorted into:
implemented, already minimum-bit, already met, not adopted, still open.

## 1. Implemented (minimum-bit branch): IR readings packed into one bit each

Tribunal change log: "pack each sensor's line-detection result into one bit using
masks and shifts; verify all four left/right combinations".

`line_read_ir()` reads both digital IR sensors once (GP26 before GP27, as before)
and returns one byte, one bit per sensor. The line loop and the route pivot use
it; behaviour and console output are unchanged.

| Bit | Mask | Sensor |
|---:|---|---|
| 0 | `LINE_IR_RIGHT` | GP26, right |
| 1 | `LINE_IR_CENTRE` | GP27, centre |
| 2–7 | — | unused, always 0 |

All four combinations, as mapped by `motion_line_direction()` after 2 equal reads:

| Centre | Right | Direction | Covered by `line_pid_checks` |
|---:|---:|---|---|
| 0 | 0 | LOST → grace, then search | yes |
| 1 | 0 | LEFT (line is to the left) | yes |
| 0 | 1 | RIGHT (line is to the right) | yes |
| 1 | 1 | STRAIGHT | yes |

## 2. Already minimum-bit (no change)

| Item | Representation | Where |
|---|---|---|
| Line error | 2 bits, −1/0/+1 (digital sensors give no finer information) | `telemetry_frame.h` `LINE_ERR` |
| Line status | `LINE_AVAIL` 1 bit, `LINE_TRACK` 1 bit | `telemetry_frame.h` |
| Code 39 symbols | each character's 9 bars/spaces as a 9-bit pattern | `barcode_decode.c` `code39_pattern[]` |
| Barcode result | `BC_CHAR` 6 bits (44 characters) + `BC_READ` 1 bit | `telemetry_frame.h` |
| Route letters | all four letters in one 32-bit word, written in one store | `route.c` `route_word` |
| Pending route action | 1 byte; 0 = none doubles as the validity flag | `line_following.c` `route_pending` |

## 3. RAM usage (measured)

From the integrated MQTT firmware (`build/mtkernel_3.elf`, Subsystem 3 code as on
`main` at `4a29d68`), `arm-none-eabi-nm -S`. Full list:
`logs/compact_representation_2026-11-25/sizes.txt`.

| What | Bytes | Note |
|---|---:|---|
| Barcode pass buffer (`seg`: samples, path, heading) | 8,424 | 1,000 samples per pass |
| Decoder work buffers (`bc_work`, `bc_widths`, `e`) | 2,792 | |
| Line task status copies (4 × `robot_status_t`) | 2,304 | static, off the nearly full line stack |
| Line task state (`ctx`) | 176 | |
| Route (`route_word`) | 4 | four letters |
| Other counters and flags | 87 | |
| **Total static RAM, Subsystem 3** | **13,787** | of the RP2040's 264 KB |
| Task stacks | 8,192 | line and barcode, 4,096 each |

Possible saving, **not done** (the decoder is tuned and working): `path[]` as an
8-bit step instead of `uint32_t` (~3 KB) and `heading[]` as `int8_t` instead of
`uint16_t` (~1 KB), about 4 KB in all.

## 4. Message sizes

| Message | Size | Note |
|---|---|---|
| Line + barcode in the status frame | 11 bits | `LINE_AVAIL` 1, `LINE_ERR` 2, `LINE_TRACK` 1, `BC_CHAR` 6, `BC_READ` 1, inside Buddy 1's 21-byte frame |
| Route in (`robot/route`) | 15 bytes text | `L=B R=C S=D U=A`; stored on the robot as 4 bytes |
| Route reply (`robot/route/ack`) | `ok …` / `rejected …` | text, once per route |

## 5. Already met (Tribunal interface rules)

| Rule | How |
|---|---|
| Single ADC owner | Line sensors are digital GPIO; only `barcode_task` uses the ADC (GP28) |
| No motor calls from B3 | Motion goes through `motion_steer*` and the Vehicle Controller |
| No-line / invalid code / missing code kept apart | `LINE_TRACK`=0; an undecodable code is counted as a failed read and never acted on; a junction only counts after a read |
| Unreliable navigation | A failed barcode read is never acted on and the car keeps following the line (stopping mid-course was judged less safe); the car stops only if a route turn cannot find the line again |
| Acknowledged B5 → B3 handover | `line_rejoin_seen()`, `TF_EV_HANDOVER` |
| Shared updates protected | Bus writes via `robot_update_subsystem()` under the bus mutex; route word written in one store |
| A4: samples per narrowest bar | 10 ms at 0.07 m/s = 0.7 mm per sample: small narrow bar (~3 mm) ≈ 4 samples, big (~6 mm) ≈ 8; widths come from encoder distance |

## 6. Not adopted (with reasons)

| Tribunal item | Reason |
|---|---|
| 2-bit `BC_CMD` + valid bit in the telemetry frame | Buddy 1's frame v3 replaced it with `BC_CHAR` + `BC_READ`; only 1 spare bit; the dashboard shows the action from the acknowledged route |
| Bit-field for the pending action | Already one byte in a padded struct: no RAM saved, and a second encoding beside `route_action_t` |
| Separate ADC acquisition task | One ADC user only; would add scheduling cost |
| Smaller decoder buffers | See section 3: ~4 KB possible, deferred |
