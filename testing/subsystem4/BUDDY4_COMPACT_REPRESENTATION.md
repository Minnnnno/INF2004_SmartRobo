# Buddy 4 data representation (IMU and terrain)

Date: 25 November 2026. Student ID: 2500910. Scope: `app_program/subsystem4/`
(GY-511 IMU, tilt, hump detection, motion classification, lift/overturn stop,
compass) and the data it shares with other subsystems.

Each row gives an application data field, its valid range, the bits that range
needs, and how it is represented.

| Status | Meaning |
|---|---|
| **In code** | Already at its minimum width, or its minimum whole-byte width |
| **Planned** | Not implemented yet; the last column is the proposed layout |
| **Team** | Not implemented; the layout is shared, so other subsystems must agree |
| **Kept wide** | Deliberately wider than the stated bits, or no finite bound exists; reason given |

"Type now" is the RP2040 type: `float` is 32 bits, `int32_t`/`uint32_t` 32, and an
enum or `bool` 8. "Bits" for a signed field includes the sign.

Ranges assume the production constants: track 115 mm (`B4_TRACK_MM`), 20 ms task
period, ±2 g accelerometer range at 12 bits, and the 250 mm/s wheel speed limit.

Sections 1–9 are the representation tables. Sections 10–12 give the measured RAM
and message-size comparisons and the correctness and timing evidence (Week 8).

## 1. GY-511 samples (`gy511_read.h`)

| Field | Valid range, units | Bits | Type now | Status and representation |
|---|---|---:|---|---|
| Acceleration X, Y, Z | −2048..2047 counts (≈ mg at ±2 g) | 12 each | `gy511_acceleration_t`, 5 bytes | **In code**: 3 × 12-bit two's complement packed into 36 bits (4 padding bits). 12 bits is the sensor's own resolution, so nothing is lost |
| Magnetic X, Y, Z | −2048..2047 counts | 12 each | `gy511_magnetic_t`, 5 bytes | **In code**: same 36-bit packing; out-of-range words are rejected before packing |
| Decoded axis value | −2048..2047 | 12 | `int16_t` local | **In code** (minimum whole type), decoded only where used |

## 2. Tilt (`tilt.h`)

| Field | Valid range, units | Bits | Type now | Status and representation |
|---|---|---:|---|---|
| Tilt result: pitch | ±75 °, 0.1 ° | 11 | packed in one `uint32_t` | **In code**: bits 0..10, offset +900 |
| Tilt result: roll | ±75 °, 0.1 ° | 11 | same `uint32_t` | **In code**: bits 11..21, offset +900 |
| Tilt result: pitch, roll condition | level / positive / negative | 2 each | same `uint32_t` | **In code**: bits 22..23 and 24..25 |
| Tilt result: valid | flag | 1 | same `uint32_t` | **In code**: bit 26 (27 bits used) |
| Capture progress | 0..50 samples, calibrated, filter ready | 6 + 1 + 1 | `uint8_t progress` | **In code**: count bits 0..5, flags bits 6..7 |
| Tilt conditions (hysteresis memory) | 2 × 3 values | 2 + 2 | `uint8_t conditions` | **In code** |
| Reference axes `up`, `forward`, `side`, filter, capture sums | unit vectors and mg | — | `float` × 18 + gravity | **Kept wide**: rotation and filtering need the precision; only the results are packed |

## 3. Hump detection (`terrain.h`)

| Field | Valid range, units | Bits | Type now | Status and representation |
|---|---|---:|---|---|
| Smoothed height | −1150..1150, 0.1 mm (± track) | 12 | `int16_t` | **In code** (minimum whole type) |
| Current peak | 0..1150, 0.1 mm | 11 | `uint16_t` | **In code** (minimum whole type) |
| Phase + flags | 3 phases; valid, pending, right side, completed event | 2 + 4 | `uint8_t flags` | **In code**: one byte |
| Last completed peak | 0..127 mm + side | 7 + 1 | `uint8_t` | **In code**: mm in bits 0..6, right wheel in bit 7 |
| Run maximum | 0..127 mm + valid | 7 + 1 | `uint8_t` | **In code**: mm in bits 0..6, valid in bit 7 |
| Candidate start, last sample time | ms since boot | 32 (modular) | `uint32_t` | **Kept wide**: wrap-safe millisecond arithmetic |

## 4. Motion classification (`motion_class.h`)

| Field | Valid range, units | Bits | Type now | Status and representation |
|---|---|---:|---|---|
| Reported class, candidate, repeat count | 8 classes, 8 classes, 0..3 | 3 + 3 + 2 | `uint8_t packed` | **In code**: one byte |
| Heading ring index, fill count, turn debounce | 0..4, 0..5, 0..3 | 3 + 3 + 2 | `uint8_t ring` | **In code**: one byte |
| Flags (valid, speed window seeded, turn in progress, turn done) | flags | 4 | `uint8_t flags` | **In code** |
| Compass validity of the 5 heading slots | 5 flags | 5 | `uint8_t compass_bits` | **In code**: bit mask |
| Yaw rate, speed change, turn peak | ±249 deg/s; mm/s² | 9; 16 | `int16_t` | **In code** (minimum whole type) |
| Heading and compass rings (5 slots each), turn start | unwrapped heading, 0.1 ° | > 16 | `int32_t` | **Kept wide**: unwrapped headings grow past ±3276.7 ° over a run |
| Window and turn times | ms since boot | 32 (modular) | `uint32_t` | **Kept wide**: wrap-safe |

## 5. Lift and overturn (`lift.h`)

| Field | Valid range, units | Bits | Type now | Status and representation |
|---|---|---:|---|---|
| Latched condition | none / lifted / overturned | 2 | `uint8_t flags` | **In code**: bits 0..1 |
| Timing flags (overturn timing, clear timing, last time valid) | flags | 3 | same byte | **In code**: bits 2, 4, 5 |
| Upright percentage | −100..100 % | 8 | `int8_t` | **In code** |
| Consecutive lift samples, suspect hold | 0..4, 0..25 samples | 3, 5 | `uint8_t` each | **In code** (minimum whole type) |
| Vertical acceleration, trigger value | ±2048 mg | 12 | `int16_t` | **In code** (minimum whole type) |
| Overturn start, clear start, last sample | ms since boot | 32 (modular) | `uint32_t` | **Kept wide**: wrap-safe |

## 6. Lifecycle and task state (`buddy4.h`, `terrain_task.c`)

| Field | Valid range, units | Bits | Type now | Status and representation |
|---|---|---:|---|---|
| Lifecycle state, retry attempts, sub-phase | 5 states, 0..3, 3 phases | 3 + 2 + 2 | `buddy4_control_t` bit-field | **In code**: one byte |
| Hump freeze | none / wait / held | 2 | `uint8_t` | **In code** |
| Magnetometer sample present | flag | 1 | `uint8_t mag_flags` | **In code** |
| Web Start gate | none / held / given | 2 | `uint8_t` | **In code** |
| Hump paused, turn paused, turn seen | flags | 1 each | `bool` | **In code** |
| Forward dynamic acceleration (diagnostic) | ±2048 mg | 12 | `int16_t` | **In code** |
| Last error | kernel error code | — | `ER` (32-bit) | **Kept wide**: kernel type, passed through unchanged |
| Phase start, last fresh sample, turn time | ms since boot | 32 (modular) | `uint32_t` | **Kept wide**: wrap-safe |

## 7. Data shared between tasks

Buddy 4 publishes one terrain slot (`subsystem_status_t`) through
`robot_update_subsystem()` every 20 ms, under the bus mutex.

| Field | Valid range, units | Bits | Type now | Shared with, protected by | Status and representation |
|---|---|---:|---|---|---|
| `values[3]` bits 0..2: validity (pitch, hump) | flags | 3 | `double` holding an integer word | Buddies 1, 3, 5; bus mutex | **In code** |
| `values[3]` bits 3..5: `T_STATE` | 8 states | 3 | same word | Buddies 1, 3, 5; bus mutex | **In code** |
| `values[3]` bits 6..8: motion class | 8 classes | 3 | same word | Buddies 1, 5; bus mutex | **In code** |
| `values[3]` bit 9: motion class + turn rate valid | flag | 1 | same word | Buddy 1; bus mutex | **In code** |
| `values[0]` turn rate | ±249 deg/s | 9 | `double`, rad/s | Buddy 1; bus mutex | **Team** |
| `values[1]` pitch | ±90 ° | 8 | `double`, rad | Buddy 1; bus mutex | **Team** |
| `values[2]` highest hump | 0..127 mm | 7 | `double`, m | Buddy 1; bus mutex | **Team** |
| `available`, `timestamp` | flag; s since boot | 1; unbounded | `bool`, `double` | bus mutex | **Team** / **Kept wide** (shared clock) |
| Event bit `TF_EV_HUMP_PEAK` | flag | 1 | 1 bit of the shared event byte | Buddy 1; interrupt-safe latch | **In code** |
| Lift/overturn stop | command | — | `motion_stop()` call | Vehicle Controller; control latch | **In code**: no message, a latch |

Buddy 4 packs its state, class and flags into one of the four values instead of
using more. The slot struct itself is common to all five subsystems, so it can't
change for Subsystem 4 alone.

## 8. MQTT

### Status frame (21 bytes, packed, sent at 10 Hz)

| Field | Valid range, units | Bits now | Bits required | Status |
|---|---|---:|---:|---|
| `T_VALID` | 3 flags (motion class + turn rate, pitch, hump) | 3 | 3 | **In code** |
| `T_STATE` | 8 states (0 INIT … 7 LIFTED) | 3 | 3 | **In code** |
| `M_CLASS` | 8 classes (0 stationary … 7 overturned) | 3 | 3 | **In code** |
| `PITCH` | −90..90 °, 1 ° | 8 | 8 | **In code** |
| `TURN_RATE` | −256..255 deg/s, + left | 9 | 9 | **In code**: the fastest pivot is ≈ 249 deg/s (both wheels at 250 mm/s, track 115 mm), so 8 bits (±127) would clip |
| `HUMP_MM` | 0..127 mm | 7 | 7 | **In code** |
| **Buddy 4 total** | | **33** | **33** | 4.125 bytes of the shared frame |

**Frame step v1 → v2** (`e564fd7` → `b7f0afb`, `docs/BUDDY4_FRAME_CHANGE_REQUEST.md`):
Buddy 4 went from 31 to 33 bits while gaining the motion class and turn rate:
- `M_CLASS` added (3 bits);
- the unused `T_HEADING`'s 9 bits reused for `TURN_RATE`;
- `HUMP_MM` trimmed from 8 to 7 bits.

The frame stayed 21 bytes.

### Diagnostics

Buddy 4 sends no MQTT text any more. Everything reaches the dashboard through the
status frame above. Per-turn angles, the compass and lift details stay on the
serial console only.

Before the change (`b96a929`), Buddy 4 sent three JSON text messages; section 11
measures them.

## 9. Faults

| Condition | Type now | Bits | Representation |
|---|---|---:|---|
| Sensor recovering (I2C error or 500 ms without fresh data) | `T_STATE` code 4 | 3 (shared code) | **In code**: state code in the slot and frame, no extra field |
| Sensor fault (retries exhausted or capture timeout) | `T_STATE` code 5 | 3 (shared code) | **In code**: read by other subsystems with `terrain_imu_faulted()` |
| Lifted (motors stopped) | `T_STATE` code 7 | 3 (shared code) | **In code** |
| Overturned (motors stopped) | motion class 7 | 3 (shared code) | **In code** |
| Cause of the fault | `ER` kernel code | 32 | **Kept wide**: console diagnostic only, never sent |

## 10. RAM usage comparison (Week 8)

**What "before" means.** The first integrated Buddy 4 (commit `b96a929`,
11 October 2026) used unpacked types:
- six `int16_t` per sample;
- the tilt result as four `float`s, three enums and a `bool`;
- Buddy 4's data sent as three MQTT JSON text messages.

The packed representations came with the rewrite in `0b4e01f` (6 November 2026).
The two versions don't have the same features (`b96a929` had no humps, lift or
motion classes), so a whole-image before/after would not compare like with like.
The comparison is therefore made **per data structure**:
- each "before" type is copied from `b96a929`, or is the plain unpacked type for
  data `b96a929` didn't have;
- each "after" type is taken from today's headers.

Measured with the Pico's compiler and the firmware's own flags (`-mcpu=cortex-m0plus
-mthumb -mfloat-abi=soft -ffreestanding -O2`; enums are 1 byte on this toolchain),
padding included (`testing/subsystem4/logs/compact_representation_2026-11-25/sizes.txt`):

| Data | Before (bytes) | After (bytes) | Saved |
|---|---:|---:|---:|
| Raw GY-511 sample (acceleration + magnetic) | 12 | 10 | 2 |
| Tilt result | 20 | 4 | 16 |
| Lifecycle control | 12 | 1 | 11 |
| Hump results (last peak, run maximum) | 16 | 2 | 14 |
| Hump detector phase + flags | 5 | 1 | 4 |
| Tilt capture progress + conditions | 8 | 2 | 6 |
| Motion class debounce | 3 | 1 | 2 |
| Compass validity, 5 slots | 5 | 1 | 4 |
| Lift latch | 4 | 1 | 3 |
| **Total** | **85** | **23** | **62 (73 % smaller)** |

**Buddy 4 in today's production firmware** (MQTT image, `arm-none-eabi-size` on its
object files):
- **Static RAM:** 264 bytes. All of it is `.bss` in `terrain_task.c`, holding the
  whole Buddy 4 state: lifecycle 152, motion class 92, compass 12, and 8 bytes of
  pause/Start-gate flags and turn time. Buddy 4 has no `.data`.
- **Task stack:** 4096 bytes, unchanged.
- **Whole image:** `bss 111792` for every buddy and the kernel together. Buddy 4's
  264 bytes are about 0.2 % of it.

## 11. Message size comparison (Week 8)

Measured from the original message formats and today's frame definition
(`testing/subsystem4/logs/compact_representation_2026-11-25/messages.txt`).

**Before (`b96a929`):** three JSON text messages every 0.25 s (4 Hz):

| Topic | Payload (bytes) | MQTT packet (bytes) |
|---|---:|---:|
| `robot/telemetry/imu_acc` `{"x":..,"y":..,"z":..}` | 19..31 | 46..58 |
| `robot/telemetry/imu_mag` (same format) | 19..31 | 46..58 |
| `robot/telemetry/terrain` `{"p":..,"r":..,"t":..,"m":..,"f":..,"s":..,"c":..,"v":..}` | 49..61 | 76..88 |
| **Per update** | **87..123** | **168..204** |
| **Per second** | 348..492 (2784..3936 bits/s) | 672..816 |

**After:** Buddy 4's 33 bits inside the shared 21-byte binary frame on `robot/frame`,
10 times a second.

| | Before | After |
|---|---|---|
| One update | 87..123 bytes in 3 messages | 33 bits (4.125 bytes) in 1 shared frame |
| Buddy 4 data per second | 2784..3936 bits | 330 bits |
| Update rate | 4 Hz | 10 Hz |

Result: **8.4× to 11.9× less data per second**, while updating 2.5× as often.

## 12. Correctness and performance (Week 8)

**No loss of required functionality:**
- **Frame round-trip** (`buddy1_frame_checks`, team suite, 24/24 pass in
  `team_host.txt`): every Buddy 4 field at full range, i.e. `TURN_RATE` −256, −1, 0,
  1, 255, `HUMP_MM` 127, and every motion class including OVERTURNED. Also a golden
  frame shared with the dashboard decoder.
- **Packed sample decode** (`host.txt`): all 4096 values of every axis, for both
  acceleration and magnetic (12288 conversions each).
- **Tilt result** pack and unpack: 682 checks.
- **Packed lifecycle, hump, class and lift state**: exercised by the lifecycle
  (296), terrain task (14749) and Start-gate (4973) suites, all passing
  (`host.txt`).
- **On the robot**: the S4-H rows in `testing/subsystem4/TEST_LOG.md`, run on
  firmware with these packed representations. The 24 November re-tests
  (<https://youtu.be/jCLuBsej2is>) show:
  - hump measured within ± 3 mm;
  - no false humps;
  - the lift stop working.
- **Precision vs requirement:**
  - hump height: 1 mm steps against the ± 3 mm acceptance;
  - pitch: 1 ° in the frame, 0.1 ° inside Buddy 4;
  - turn rate: 1 deg/s;
  - acceleration: 12-bit sensor resolution kept.

**No unacceptable timing degradation:**
- **Task period:** the terrain task still runs every 20 ms. `test_terrain_task`
  checks there is exactly one sensor sample per 20 ms release.
- **Cost of packing:** only shifts and masks, with no allocation and no loops over
  buffers. Decoding happens only where a value is used.
- **Messages:** sending 33 bits in the shared frame replaces formatting three JSON
  strings with `tm_sprintf` every 0.25 s.
- **Not measured:** the terrain task's CPU time per cycle on the robot. The robot
  videos show the functional result (humps, turns, lift stop) but not cycle timing;
  a serial log of a run would show any `[B4-STATE] ... RECOVERY` or fault lines.

## Evidence

`testing/subsystem4/logs/compact_representation_2026-11-25/`:
- `sizes.txt`: per-structure before/after sizes, Buddy 4's firmware object sizes and
  the whole-image size;
- `messages.txt`: JSON vs frame sizes and rates;
- `host.txt`: Buddy 4 host suites (13 pass);
- `team_host.txt`: team host suite (24/24 pass).

Re-run all numbers with `python tests/subsystem4/measure_compact.py <firmware build
dir> <output dir>`. Tools: `tests/subsystem4/memory_probe.c` (the before/after
types) and `tests/subsystem4/measure_compact.py`. Neither is part of the firmware or
the host test build.
