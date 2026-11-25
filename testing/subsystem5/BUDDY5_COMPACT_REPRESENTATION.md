# Buddy 5 data representation (obstacle detection)

Date: 25 November 2026. Student ID: 2500344. Scope: `app_program/subsystem5/`
(ultrasonic, servo, obstacle scan, detour) and the data it shares with other
subsystems.

Each row gives an application data field, its valid range, the bits that range
needs, and how it is represented.

| Status | Meaning |
|---|---|
| **In code** | Minimum width, or minimum whole-byte width, in the firmware |
| **Team** | Not implemented; the layout is shared, so other subsystems must agree |
| **Kept wide** | Deliberately wider than the stated bits, or no finite bound exists; reason given |

"Type before" is the RP2040 type before the packing (commit `20405f9`): `double` is 64 bits, `int` and `unsigned` 32, and
an enum or `bool` 8. "Bits" for a signed field includes the sign.

Ranges assume the production constants: ultrasonic limit 4000 mm
(`S5_US_MAX_MM`), sensor offset 30 mm, sweep 20..160 deg and beam correction
22 deg. With these, an edge lies at most 4030 x tan(48 deg) = 4476 mm to the side.

## 1. Ultrasonic and servo

| Field | Valid range, units | Bits | Type before | Status and representation |
|---|---|---:|---|---|
| Echo pulse | 117..23,091 us (20..4000 mm at 25 C) | 15 | hardware 16-bit PWM counter | **In code** |
| Ping result `s5_us_status_t` | 4 codes | 2 | enum (8) | **In code** (whole byte; codes are mutually exclusive) |
| Ping range | 20..4000 mm, 1 us = 0.17 mm | 15 | `double` local | **Kept wide**: working value in the estimate arithmetic; stored results are packed (section 2) |
| Ping time | s since boot | unbounded | `double` local | **Kept wide**: shared `motion_hal_time()` clock |
| `us_last_trigger` | us; only the time since the last ping is used | 32 (wraps at 71 min) | `uint64_t` | **In code**: `uint32_t`, modular difference; a wrap can only cost one 60 ms wait |
| `us_ready` | false/true | 1 | `bool` | **In code** (whole byte) |
| `us_slice` | 0 (fixed by GP17) | 0 | `unsigned` | **In code**: compile-time constant |
| `servo_deg` | -1 (not started), 20..160 deg, 1 deg | 8 | `int` | **In code**: `uint8_t`, 0 = not started |
| `servo_slice` | 7 (fixed by GP15) | 0 | `unsigned` | **In code**: compile-time constant |
| Servo pulse | 500..2500 us | 12 | `uint16_t` | **In code** (hardware compare width) |
| Front history: range x3 | 20..4000 mm, 0.25 mm | 14 each | `double` | **In code**: `uint16_t` |
| Front history: time x3 | ms; only ages up to 250 ms count | 32 (modular) | `double` | **In code**: `uint32_t`; a stale entry could only look fresh after 49 days without a ping |
| Front history: echo x3, next index 0..2 | flags, index | 3 + 2 | `bool` x3, `unsigned` | **In code**: one `uint8_t` |

## 2. Scan point (`s5_point_t`, 196 stored)

This was the largest Subsystem 5 object: 64 bytes x 196 = 12,544 of the 12,904
bytes of `s5_scan_t`. **In code**: one `uint32_t` per point, using masks and shifts
(`s5_point_pack()` and the `s5_point_*()` readers in `obstacle_scan.c`).

| Field | Valid range, units | Bits | Type before | Position |
|---|---|---:|---|---|
| Range (mean of agreeing pings) | 0..4000 mm, 0.25 mm (code 0..16,000) | 14 | `double` | bits 0..13 |
| Servo angle | 20..160 deg, 1 deg (code 0..140) | 8 | `int` | bits 14..21 |
| `support`, pings that agree | 0..10 | 4 | `unsigned` | bits 22..25 |
| `valid`, pings with an echo | 0..10 | 4 | `unsigned` | bits 26..29 |
| `total`, pings taken | 5 or 10 | 1 | `unsigned` | bit 30 |
| `b_object` | false/true | 1 | `bool` | bit 31 |
| `status` | 3 codes | 2 | enum | not stored: derived from support, valid and total by the same function `s5_estimate()` uses |
| `nearest_mm`, `farthest_mm` | 20..4000 mm | 14 each | `double` | not stored: no code reads them from a stored point |

Total **32 bits**. The rounding error is at most 0.125 mm, against a sensor repeatability of
about 3 mm and a smallest decision threshold of 5 mm. A range is rounded only when stored.

An unfilled point needs no code of its own because the point counts below say how many are used.

| Counts | Valid range | Bits | Type before | Status and representation |
|---|---|---:|---|---|
| `coarse_count` | 0..15 | 4 | `unsigned` | **In code**: `uint8_t` |
| `fine_count[2]` | 0..90 | 7 each | `unsigned` | **In code**: `uint8_t` |
| `fine_outward[2]` | 0..70 | 7 each | `unsigned` | **In code**: `uint8_t` |
| `s5_scan_t.b_valid` | false/true | 1 | `bool` | **In code** (whole byte) |

## 3. Scan results

There is one instance of each result, and together they took 360 bytes including the counts above.

| Field | Valid range, units | Bits | Type before | Status and representation |
|---|---|---:|---|---|
| Edge `last_object_deg`, `first_clear_deg` | 20..160 deg | 8 each | `int` | **In code**: `uint8_t` |
| Edge `b_found`, `b_inward` | flags | 1 each | `bool` | **In code**: shared flag byte |
| Edge `outward_deg`, `inward_deg` | 20..160 deg, 0.5 deg | 9 each | `double` | **Kept wide**: geometry inputs |
| Edge `angle_deg` | 42..138 deg, 0.25 deg | 9 | `double` | **Kept wide**: input to `tan()` |
| Edge `range_mm` | 0..4000 mm | 14 at 0.25 mm | `double` | **Kept wide**: reported only, one instance |
| Edge `lateral_mm` | -4476..4476 mm | 14 at 1 mm | `double` | **Kept wide**: gives the width and the sideways move |
| Face `tilt_deg` | -45..45 deg | 10 at 0.1 deg | `double` | **Kept wide**: rotates the detour frame |
| `squared_deg` | at most two turns of 45 deg each way | 11 at 0.1 deg | `double` | **Kept wide**: added to the tilt |
| Face `distance_mm` and corner reaches | 0..4030 mm | 12 at 1 mm | `double` | **Kept wide**: geometry |
| Face `far_mm` | 0..4000 mm | 12 at 1 mm | `double` | **Kept wide**: geometry |
| Face `end_deg[2]` | 20..160 deg | 8 each | `int` | **In code**: `uint8_t` |
| Face `b_valid`, `b_corner[2]` | flags | 3 | `bool` x3 | **In code**: shared flag byte |
| `position` | 4 codes | 2 | enum (8) | **In code** (whole byte) |
| Placement `b_bounded` | flag | 1 | `bool` | **In code** (whole byte) |
| Placement left and right edge angle | 0..48 deg, 0.25 deg | 8 each | `double` | **Kept wide**: printed only, one instance |
| `width_mm` | 0..8952 mm | 14 at 1 mm | `double` | **Kept wide**: reported to others in minimum bits (sections 6 and 7) |
| `offset_mm` | -2238..2238 mm | 13 at 1 mm | `double` | **Kept wide**: as above |
| Avoid `side` | 3 codes | 2 | enum (8) | **In code** (whole byte) |
| Avoid `shift_mm[2]` | 0..4606 mm | 13 each at 1 mm | `double` | **Kept wide**: plans the detour |
| Avoid `clear_mm[2]` | 0..4072 mm, or none heard | 12 each at 1 mm | `double` (INFINITY = none) | **Kept wide**: the INFINITY comparison is part of the decision |
| Avoid `b_room[2]` | flags | 2 | `bool` x2 | **In code** (whole bytes) |

Why these stay `double`: they feed trigonometry directly and there is only one of each, so packing them would
add conversions and code for little RAM. The stored sweep in section 2 is where the memory goes.

Temporary data on the task stack:

| Field | Valid range, units | Bits | Type before | Status and representation |
|---|---|---:|---|---|
| Face profile range x141 | 0..4000 mm, 0.25 mm | 14 each | `double` | **In code**: `uint16_t` |
| Face profile taken x141, echo x141 | flags | 1 each | `bool` | **In code**: two 141-bit bitmaps (18 bytes each) |
| Ping buffer length | 10 readings used | - | 16 slots | **In code**: `S5_MAX_SAMPLES` 16 -> 10 |
| Ping buffer echo | flags | 1 each | `bool` | **In code**: one 10-bit mask |

The profile went from 1,416 to 318 bytes (both compiled for the RP2040).

## 4. Curve history (`s5_curve_t`)

Used only for the "curve check" log line; no decision reads it.

| Field | Valid range, units | Bits | Type before | Status and representation |
|---|---|---:|---|---|
| Path x16 | mm; only differences up to 300 mm are used | 16 (modular) | `double` | **In code**: `uint16_t` |
| Heading x16 | 0.1 deg; only differences are used | 16 (modular) | `double` | **In code**: `uint16_t` |
| `now_mm`, `until_mm` | mm; at most 300 mm apart | 16 each (modular) | `double` | **In code**: `uint16_t`, with the mark cleared once passed |
| `swing_deg` | 0..3276.8 deg, 0.1 deg | 16 | `double` | **In code**: `uint16_t` |
| `count` 0..16, `next` 0..15, `b_marked` | counters, flag | 5 + 4 + 1 | `unsigned` x2, `bool` | **In code**: one `uint16_t` |

296 bytes before, 72 bytes now.

## 5. Task and detour state

| Field | Valid range, units | Bits | Type before | Status and representation |
|---|---|---:|---|---|
| `s5_phase` | 4 states | 2 | enum (8) | **In code**: byte A bits 0..1 |
| `s5_blocked`, `s5_decided`, `s5_sensor_ok` | flags | 1 each | `bool` | **In code**: byte A bits 4..6 |
| `s5_spurious_count` | only 0, 1 and "more than 1" matter | 2 | `unsigned` | **In code**: byte A bits 2..3, saturating |
| `s5_rejoin_armed`, `s5_rejoined`, `s5_line_touched`, `s5_line_noted`, `s5_line_goal`, `s5_pass_stalled`, `s5_out_blocked`, `s5_sweep_blocked` | flags | 1 each | `bool` x8 | **In code**: byte B, all 8 bits |
| Move result | 6 codes | 3 | enum (8) | **In code** (whole byte, local) |
| Look result | 4 codes | 2 | enum (8) | **In code** (whole byte, local) |
| Abort code (`s5_why`, `s5_scan_why`) | 40 codes, plus none | 6 | `const char *` | **In code**: `uint8_t` index into one name table |
| Abort `a`, `b` | -1..5999 (move ID, mm, deg or state); coordinates may be negative | 14 each | `int` | **In code**: `int16_t`, clamped to -8192..8191 |
| `s5_block_angle` | 20..160 deg | 8 | `int` | **In code**: `uint8_t` |
| `s5_block_range` | 0..4000 mm | 12 | `int` | **In code**: `uint16_t` |
| `s5_note_count` | only "changed during this leg" is used | 1 | `unsigned` | **In code**: flag |
| `s5_move_id` | 5100..5999 | 10 | `uint32_t` | **In code**: `uint16_t`; the shared command ID stays `uint32_t` |
| Pass `clear_run` | 0..2 | 2 | `unsigned` | **In code**: `uint8_t` |
| Pass `seen` | looks that heard the obstacle; printed only | 8, saturating | `unsigned` | **In code**: `uint8_t` |
| Pass `b_ended`, plan `b_ok` | flags | 1 each | `bool` | **In code** (whole bytes) |
| Plan `side` | 3 codes | 2 | enum (8) | **In code** (whole byte) |
| Attempt table: other side, turn limit 45..75 deg, wait 0 or 2000 ms | constants | 1 + 7 + 1 | `bool`, `double`, `unsigned` (flash) | **Kept wide**: 144 bytes of flash, no RAM |
| Axle position, noted line position | mm | see below | `double` | **Kept wide** |
| Centre travel, path, wheel travel, start heading | m, rad | unbounded | `double` | **Kept wide** |
| Stall and move times | s since boot | unbounded | `double` | **Kept wide** |
| Plan and pass distances and angles | mm, deg | 12..14 at 1 mm | `double` | **Kept wide** |

Why these stay `double`: positions are running sums of encoder travel, and they are compared with Motion
Control's own `double` travel and heading, which have no fixed lifetime bound. Rounding each step would add
up; `docs/BUDDY2_COMPACT_REPRESENTATION.md` gives the precision argument. Plan and pass values are
short-lived geometry on the stack.

## 6. Data shared between tasks

Subsystem 5 has no interrupt handler: the echo is timed by PWM hardware and polled by `obstacle_task`.

| Field | Valid range, units | Bits | Type before | Shared with, protected by | Status and representation |
|---|---|---:|---|---|---|
| `values[0]` distance | 0..4000 mm | 12 | `double`, m | Buddies 1 and 3; bus mutex | **Team** |
| `values[1]` obstacle centre | -2238..2238 mm | 13 | `double`, m | bus mutex | **Team** |
| `values[2]` width | 0..8952 mm | 14 | `double`, m | Buddy 1; bus mutex | **Team** |
| `values[3]` state `S5_STATE_*` | 8 states | 3 | `double` | Buddies 1 and 3; bus mutex | **Team** |
| `available` | flag | 1 | `bool` | Buddies 1 and 3; bus mutex | **Team** |
| `timestamp` | s since boot | unbounded | `double` | bus mutex | **Kept wide**: shared clock |
| Event bits `TF_EV_OBSTACLE`, `TF_EV_SCAN_DONE` | flags | 1 each | 2 bits of the shared event byte | Buddy 1; interrupts disabled (`DI`/`EI`) | **In code** |
| Crash stage `CRASH_OBSTACLE_*` | 14 stages, plus none | 4 | watchdog scratch register (32) | read after a reset | **Kept wide**: hardware register, one whole-word write |
| Decision copy `s5_avoid` | section 3 | - | 48-byte struct | any task, through `s5_get_avoidance()`; `DI`/`EI` | Removed with `s5_get_avoidance()`, which nothing called; the decision is read from the scan |

The first five fields need 43 bits (6 bytes), against a 48-byte `subsystem_status_t` including the timestamp.
The struct is common to all five subsystems, so it cannot change for Subsystem 5 alone.

**Synchronisation of packed data.** Bytes A and B and the packed scan are read and written only by
`obstacle_task`, so their read-modify-write needs no lock. If another task
ever reads them, the copy must go through the bus mutex or `DI`/`EI`, as the status above does.

## 7. MQTT

### Status frame (21 bytes, packed, sent at 10 Hz)

| Field | Valid range, units | Bits now | Bits required | Status |
|---|---|---:|---:|---|
| `O_AVAIL` | 0..1 | 1 | 1 | **In code** |
| `O_DIST_CM` | 0..400 cm | 9 | 9 | **In code** |
| `O_WIDTH_CM` | 0..255 cm, clamped | 8 | 8 | **In code** |
| `O_FLAGS` | 8 states | 5 | 3 | **Team**: make it a 3-bit state field. The header describes five flag bits, but Subsystem 5 publishes `S5_STATE_*` and the dashboard decodes that |

The `O_FLAGS` change frees 2 bits. The frame stays 21 bytes.

### Diagnostics (text now, binary proposed)

**Team**: these need a binary send call from Buddy 1 and a decoder in the dashboard.

| Message | Fields (bits) | Text now | Binary |
|---|---|---:|---:|
| Scan result | type 2, width 14, distance 12, offset 13, complete 1, position 2, avoid side 2, shift left 13, shift right 13 = **72** | two messages, typically 73 + 50 bytes | 9 bytes |
| Abort | type 2, code 6, a 14, b 14, x 14, y 14, tries 3 (0..6), outside stop 1 = **68** | typically 62..65 bytes | 9 bytes |
| Spurious stop | type 2, ahead 12 (0..4000 mm) + none 1, resumed 1 = **16** | 46 bytes | 2 bytes |

Values outside a field's range are clamped, never wrapped, as in `include/telemetry_frame.h`. An unused abort
number is sent as -1 or 0, as it is now.

## 8. Faults

| Condition | Type before | Bits | Representation |
|---|---|---:|---|
| No echo | ping result code | 1 | **Team**: bit in the shared fault bitmask. It means "nothing in range", so it is not a fault on its own |
| No pulse (sensor not answering) | ping result code, then `s5_sensor_ok` | 1 | **Team**: bit in the shared fault bitmask |
| ECHO stuck high | ping result code, then `s5_sensor_ok` | 1 | **Team**: bit in the shared fault bitmask |

`TF_OF_NO_ECHO` is defined in `include/telemetry_frame.h`, but Subsystem 5 never sets it.

## 9. Results

Integrated image (line steering, Subsystems 4 and 5, Wi-Fi and MQTT; Release), built before and
after the change with the same toolchain (Arm GNU 15.2.Rel1):

| Object | Before (bytes) | After (bytes) |
|---|---:|---:|
| `s5_point_t` (196 stored) | 64 each | 4 each |
| `s5_scan_t` | 12,904 | 1,072 |
| `s5_curve_t` | 296 | 72 |
| Edge result `s5_edge_t` | 64 | 48 |
| Face result `s5_face_t` | 80 | 64 |
| Face profile (task stack) | 1,416 | 318 |
| Pass state `s5_pass_t` (task stack) | 40 | 32 |
| Other Subsystem 5 task, detour, servo and ultrasonic variables | 257 | 160 |
| **All Subsystem 5 static RAM** | **13,457** | **1,304** |
| **Whole image, data + bss** | **111,912** | **99,752** |
| Whole image, text (flash) | 476,876 | 478,212 |

RAM saved: 12,160 bytes. Cost: 1,336 bytes of flash for packing and unpacking.

Checks (host, MSVC; **not yet run on the robot**):

- All 24 host tests pass, including 731 geometry, 464 runtime and 11 servo checks for Subsystem 5.
- New checks: every point field read back at both ends of its range, clamping, marking a point changes
  no other field, the derived status against `s5_estimate()` for all 72 possible votes, and curve history
  across the 16-bit wrap.
- Old and new code on the same 3,000 simulated scans (3 mm noise, stray and missing echoes): the same
  pass side in 2,988 (99.6%), the same "plan found" in all. Where the decisions agree, width differs by
  0.12 mm (median) and 0.87 mm (99th percentile).
- The 12 side differences are scans at a threshold. Ten measured a rise of 4.81..4.99 mm across the
  face, which rounds to 5.00 mm when the two stored ranges are subtracted and so passes the face fit's
  5 mm minimum. The other two differ in whether a corner at the limit of the sweep counts as placed.
  Sensor repeatability is about 3 mm, so such scans could already go either way.

The **Team** rows, and the diagnostics in binary, are not implemented.
