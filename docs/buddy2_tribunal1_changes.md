# Buddy 2: Tribunal 1 data representation changes

**Subsystem:** Motion control, motor driver, encoders, PI/PID and odometry

**Implementation:** 22 October 2026

**Report and ARM layout verification:** 25 November 2026

The Buddy 2 change replaces oversized private command, event, encoder and driver
storage with explicit integer units, packed flags and smaller configuration
copies. The measured historical production image uses **400 fewer bytes of
static RAM: 20,832 -> 20,432 bytes (1.92%)**. Private active-command storage falls
from **72 to 56 bytes**, and eight private events from **128 to 72 bytes**.
Public inter-task command, status and event layouts remain unchanged.

## Comparison scope and evidence

“Before” means the pre-compaction workspace used for the original paired build;
“after” means the compact representation implemented in
[12d2fec](https://github.com/Minnnnno/INF2004_SmartRobo/commit/12d2fecc9c8c455d336aa2ccf14a38906f11a9fa).
The original baseline included workspace changes that were not yet committed.
Building that commit's parent alone would also remove diagnostic changes and
would not reproduce the controlled representation comparison.

The original measurements are recorded in
[BUDDY2_MEMORY_RESULTS.json](BUDDY2_MEMORY_RESULTS.json) and
[BUDDY2_COMPACT_REPRESENTATION.md](BUDDY2_COMPACT_REPRESENTATION.md).
The raw before/after records were preserved in commit
[d74a93a](https://github.com/Minnnnno/INF2004_SmartRobo/commit/d74a93a13ff483d2639308b4e69b831744d9b06c):

- [Before: image sizes](https://github.com/Minnnnno/INF2004_SmartRobo/blob/d74a93a13ff483d2639308b4e69b831744d9b06c/testing/subsystem2/logs/compact_representation_2026-10-22/before/image-size.txt),
  [type sizes](https://github.com/Minnnnno/INF2004_SmartRobo/blob/d74a93a13ff483d2639308b4e69b831744d9b06c/testing/subsystem2/logs/compact_representation_2026-10-22/before/type-sizes.txt)
  and [RAM symbols](https://github.com/Minnnnno/INF2004_SmartRobo/blob/d74a93a13ff483d2639308b4e69b831744d9b06c/testing/subsystem2/logs/compact_representation_2026-10-22/before/buddy2-symbols.txt).
- [After: image sizes](https://github.com/Minnnnno/INF2004_SmartRobo/blob/d74a93a13ff483d2639308b4e69b831744d9b06c/testing/subsystem2/logs/compact_representation_2026-10-22/after/image-size.txt),
  [type sizes](https://github.com/Minnnnno/INF2004_SmartRobo/blob/d74a93a13ff483d2639308b4e69b831744d9b06c/testing/subsystem2/logs/compact_representation_2026-10-22/after/type-sizes.txt)
  and [RAM symbols](https://github.com/Minnnnno/INF2004_SmartRobo/blob/d74a93a13ff483d2639308b4e69b831744d9b06c/testing/subsystem2/logs/compact_representation_2026-10-22/after/buddy2-symbols.txt).

These are historical before/after measurements for this change, rather than the
size of today's integrated firmware. Later additions to fault diagnostics,
configuration, networking and sensors change the current image's totals.

## 1. Data representation table

“Required bits” means the logical width needed for the stated range and
resolution. Actual C allocation includes container widths, alignment and padding.
The RP2040 ARM compiler used here stores the relevant enums in **one byte**;
the host compiler's enum sizes must not be used to estimate target RAM.

### Command fields

The public API continues to accept SI units in `motion_command_t`. Only the
motion task's private stored copy changes. Implementation:
[motion_storage.h](../include/motion_storage.h) and
[motion_compact.h](../include/motion_compact.h).

| Application field | Valid range / resolution | Required bits | Before | After, private implementation |
| --- | --- | ---: | --- | --- |
| Distance / ARC centre travel | -5,000..5,000 mm, 1 mm steps | 14 signed | `double value`, metres, 64 bits | Payload bits 0..13, signed integer millimetres |
| Left wheel-speed command | -250..250 mm/s, 1 mm/s steps | 9 signed | `double speed[0]`, m/s, 64 bits | Payload bits 14..22 |
| Right wheel-speed command | -250..250 mm/s, 1 mm/s steps | 9 signed | `double speed[1]`, m/s, 64 bits | Payload bits 23..31 |
| Turn / ARC heading change | -360..360 degrees, 1 degree steps | 10 signed | Binary64 radians; `value` for ENCODER_TURN, `angle` for ARC_MOVE | Payload bits 32..41; decoded back to radians |
| Motion mode | Five values, codes 0..4 | 3 unsigned | One-byte public enum | Payload bits 42..44 |
| Reserved payload bits | Must be zero | 3 allocated | No private payload | Bits 45..47; nonzero values rejected |
| Command ID | 0..4,294,967,295 | 32 | `uint32_t id` | Retained as `uint32_t` |
| Generation | 0..4,294,967,295 | 32 | `uint32_t generation` | Retained as `uint32_t`; stale-command protection preserved |
| Expiry / deadline | Finite absolute monotonic seconds; no reduced uptime bound specified | No justified reduced width; retain 64 each | Two `double` fields | Two `float64_t` fields, copied exactly |
| ARC duration | Finite seconds, validated against trajectory/deadline rules | No justified reduced width; retain 64 | `double duration` | `float64_t duration` retained |
| Manual PWM fractions / ARC wheel proportions | Mode-dependent; manual output obeys configured duty limit; ARC obeys speed/trajectory limits | No justified reduced width; retain 64 per wheel | Two binary64 values through `speed[2]` | Two `float64_t precise_speed[2]` extension values |

The numeric/mode payload uses **14 + 9 + 9 + 10 + 3 = 45 bits**, allocated as
**six bytes** with three reserved bits. Explicit unsigned masks define the byte
layout; compiler-dependent C bit-fields are not used.

For a signed two's-complement field, the width must contain the complete signed
range: 14 bits cover -5,000..5,000; 9 bits cover -250..250; and 10 bits cover
-360..360. All inputs are checked before rounding. Rounding is nearest with
halfway values away from zero; the nominal maximum quantization errors are
0.5 mm, 0.5 mm/s and 0.5 degree.

The full private command is **56 bytes**, rather than six:

```text
ID + generation                  8 bytes
expiry + deadline               16 bytes
duration + precise_speed[2]     24 bytes
packed numeric/mode payload      6 bytes
tail padding                     2 bytes
total                           56 bytes
```

Manual PWM fractions are not converted to mm/s. For timed ARCs, wheel velocities
are derived from the rounded distance/angle and the preserved duration; separately
rounding those derived velocities would break the trajectory relationship.

### State, events and flags

| Application field | Valid range / meaning | Required bits | Before | After implementation |
| --- | --- | ---: | --- | --- |
| Motion state | INITIALISING through SHUTDOWN, seven values | 3 | One-byte enum | Public enum retained |
| Primary fault | Historical snapshot codes 0..10; current code extends through 12 | 4 | One-byte enum | Public enum retained; private event metadata allocates four bits |
| Feedback diagnostic cause | Historical five values; current six values | 3 | Public enum in the comparison baseline | Public enum retained |
| Motion result | R_OK through R_FULL, six values | 3 | Enum | Enum retained |
| Event type | Five values, codes 0..4 | 3 | One-byte enum in each event structure | Bits 0..2 of each private event info byte |
| Event primary fault | Same primary fault domain as above | 4 | One-byte enum with structure padding | Bits 3..6 of each private event info byte; bit 7 reserved |
| Event ID / generation | Full unsigned 32-bit identities | 32 each | Two `uint32_t` members per event | Separate `uint32_t id[8]` and `generation[8]` arrays |
| Event queue head | 0..7 | 3 | 32-bit `unsigned event_head` | Bits 0..2 of `uint8_t event_queue` |
| Event occupancy | 0..8, including full | 4 | 32-bit `unsigned event_count` | Bits 3..6 of `event_queue`; full and empty remain distinct |
| Still tracking / goal pending / feedback latched | Three independent booleans | 1 each | Three separate `bool` fields | Three bits in `uint8_t runtime_flags` |
| Active command flag | false / true | 1 | `bool active` | Retained for the controller interface |
| Independent fault history | Historical ten nonzero fault codes; current twelve | 10 historical / 12 current | Only primary cause recorded | Independent bits in `uint16_t fault_flags` |
| Public status booleans | Stationary, feedback trusted, inhibited, hold, timing fault recorded | 1 each | Separate `bool` members | Public fields retained to preserve shared layouts |
| Request source / owner | Six sources, with telemetry code 7 for no owner | 3 | Enum | Shared enum retained |

Fault history is a bit mask because several faults can occur independently.
It cannot be replaced with a mutually exclusive four-bit primary-fault code.
The stored-event arrays remove per-event padding while adapters reconstruct the
original public `motion_event_t` when a consumer requests an event.

### Encoder and motor-driver fields

| Application field | Valid range / meaning | Required bits | Before | After implementation |
| --- | --- | ---: | --- | --- |
| Left/right encoder counts | 0..2^32-1, modulo wrap | 32 each | `uint32_t count[2]` | Unchanged, full precision |
| Invalid-transition totals | 0..2^32-1, modulo wrap | 32 each | `uint32_t invalid[2]` | Unchanged |
| Edge totals for two channels per wheel | 0..2^32-1, modulo wrap | 32 each | `uint32_t edges[2][2]` | Unchanged |
| Four encoder GPIO numbers | GP0..GP29 | 5 each | Four 32-bit `unsigned` cells | Four `uint8_t` cells; not packed across byte boundaries |
| Previous A/B state per wheel | 0..3 | 2 each | Two `uint8_t` cells | Four bits in one `uint8_t previous_ab` |
| Quadrature enable per wheel | false / true | 1 each | Two `bool` cells | Bits 0..1 of capture flags |
| Encoder sign per wheel | -1 / +1 | 1 each | Two 32-bit `int` cells | Bits 2..3 of capture flags |
| Inferred direction per wheel | -1 / +1 | 1 each | Two 32-bit `int` cells | Bits 4..5 of capture flags |
| Encoder-ready / IRQ-registered / PWM-ready | Three independent booleans | 1 each | Three separate `bool` globals | Three bits in `uint8_t g_driver_flags` |
| Last motor direction per wheel | -1 / 0 / +1 | 2 each | Two 32-bit `int` cells | Two-bit codes in `g_wheel_flags` |
| Neutral flag per wheel | false / true | 1 each | Two `bool` cells | Bits 0..1 of `g_wheel_flags`; directions use bits 2..5 |
| Brake / two motor polarity signs | One brake boolean and two -1/+1 signs | 3 total | Members in a duplicate full configuration | Three bits in `motion_motor_storage_t.flags` |
| Driver maximum output / reversal time | Duty in (0,1]; positive seconds | Retain 64 each | Part of copied `motion_config_t` | Only these two binary64 values and the flags are retained by the driver |
| Neutral timestamps | Monotonic seconds; uptime not reduced | Retain 64 each | Two `double` values | Unchanged |
| Motor GPIO constants | GP8/9 and GP10/11 | 4 each for these values | Exported `const unsigned[2][2]` | Exported type retained |
| PWM wrap / compare operands | Wrap 4,095; compare can reach 4,096 for coast | 12 / 13 unsigned | `uint16_t` hardware operands | Hardware-width values retained |
| PWM frequency | 1,000..20,000 Hz | 15 unsigned | Frequency included in copied configuration | Used during initialization; omitted from reduced driver configuration |

See [encoder_capture.h](../include/encoder_capture.h) and
[drv_motion.c](../app_program/subsystem2/drv_motion.c).

### Precision retained in control calculations

| Application field | Valid range / resolution | Required bits / decision | Before -> after |
| --- | --- | --- | --- |
| Odometry previous counts, start counts and prior invalid totals | Full modulo 2^32 counters | 32 each | `uint32_t` -> unchanged |
| Wheel travel, centre travel, path and heading accumulators | Cumulative metres/radians; not bounded by a single 5 m / 360-degree command | Retain 64; no proven small fixed range | Binary64 -> unchanged |
| Estimated wheel speeds | Encoder-count deltas divided by measured time | Retain 64; not limited to command quantization steps | Binary64 -> unchanged |
| PID integral / previous error / derivative | Depend on configured gains, measured speed and time step | Retain 64; fixed-point bounds not established | Binary64 -> unchanged |
| PID gains / derivative time constants | Finite nonnegative configuration; no universal maximum specified | Retain 64 each | Binary64 -> unchanged |
| PID seeded flags | false / true | 1 each | Public `bool` fields retained |
| Geometry, tolerances, acceleration and control limits | Validated finite configuration in `motion_config_t` | Retain existing types; no blanket width reduction | Authoritative configuration retained |
| Progress, settling, sample and timing diagnostics | Metres / monotonic seconds; variable uptime | Retain binary64; diagnostic counters retain 32 bits | Unchanged in the controlled representation comparison |
| Encoder sample reliability | false / true | 1 | Public `bool reliable` retained |

At the recorded left-wheel geometry, one encoder count is approximately
**0.083408395 mm**. Rounding encoder travel to whole millimetres would lose count
information. The change quantizes command targets while preserving encoder,
feedback, PID and odometry precision.

## 2. RAM usage comparison

### Linked firmware measurements

Both recorded builds use Pico W, ARM GCC 15.2.1, `-O2`, `SMP=OFF`, USB CDC,
Wi-Fi/networking off and calibration/IR-monitor profiles off. Values below come
from the archived `arm-none-eabi-size` output and include alignment.

| Linked image section | Before, bytes | After, bytes | After minus before |
| --- | ---: | ---: | ---: |
| `.data` | 28 | 28 | 0 |
| `.bss` | 20,804 | 20,404 | -400 |
| **Static RAM = .data + .bss** | **20,832** | **20,432** | **-400 (-1.92%)** |
| `text` | 95,620 | 99,716 | +4,096 |

The additional 4,096 bytes of text are the code-size cost of conversion and
validation. Static RAM is not a runtime stack high-water measurement. Task stack
margin, conversion latency and combined-load timing require target measurement.

The saved firmware build IDs are `b2-production-v1-20261022T122402Z` (before) and
`b2-production-v1-20261022T123408Z` (after). Firmware hashes are retained in the
[measurement JSON](BUDDY2_MEMORY_RESULTS.json); original build folders/binaries
were removed during cleanup.

### Object-level breakdown

| Object / allocation | Before, bytes | After, bytes | Saved, bytes |
| --- | ---: | ---: | ---: |
| Whole motion core, `motion_t` | 936 | 856 | 80 |
| Encoder capture, `encoder_capture_t` | 72 | 40 | 32 |
| Driver's duplicate configuration | 304 | 24 | 280 |
| Named driver state/direction/neutral globals | 13 | 2 | 11 |
| Other named globals: encoder IRQ masks (16), neutral timestamps (16), PWM wrap (2) | 34 | 34 | 0 |
| **Affected named globals total** | **1,359** | **956** | **403** |
| Active command, already included inside `motion_t` | 72 | 56 | 16 |
| Eight-event payload, already included inside `motion_t` | 128 | 72 | 56 |
| Each `pid_t` | 32 | 32 | 0 |
| `odometry_t` | 64 | 64 | 0 |

The affected-global sum saves 403 bytes; the final linked RAM saves 400 bytes
because of compiler/linker alignment. Command and event savings must not be
added to the total again: they are already inside `motion_t`.

The driver configuration keeps 17 member bytes plus seven alignment bytes
(24 total). The private command has 54 member bytes plus two padding bytes;
encoder capture has 38 member bytes plus two padding bytes. Required field bits
alone therefore do not equal allocated RAM.

## 3. Message size comparison

The optimization changes motion-owner storage. Shared requests and snapshots
still cross the existing controller/status mutex boundary in their public
formats. The six-byte payload is not an implemented replacement for the whole
inter-task command.

The following target sizes were checked again on 25 November 2026 with
ARM GCC 15.2.1 using the Tribunal-era headers from `12d2fec`. For the controlled
“before” layout check, only the private command, event, flag and encoder layouts
were restored to their pre-compaction definitions, keeping the same public
diagnostic fields. This layout reconstruction matches the archived type sizes;
it is not a newly rebuilt historical firmware image.

| Relevant data/message | Before, bytes | After, bytes | Reduction | Implementation meaning |
| --- | ---: | ---: | ---: | --- |
| Public `motion_command_t` | 72 | 72 | 0% | Shared API command remains ABI-compatible |
| Inter-task `robot_request_t` | 80 | 80 | 0% | Source plus public command, including target alignment |
| Eight public requests, payload only | 640 | 640 | 0% | Shared request queue remains unchanged |
| Private active-command storage | 72 | 56 | 22.22% | Internal stored representation; not the inter-task message |
| Public `motion_event_t` | 16 | 16 | 0% | Consumer-facing event remains unchanged |
| Eight private stored events | 128 | 72 | 43.75% | Structure-of-arrays: 32 bytes IDs + 32 bytes generations + 8 info bytes |
| Public `motion_status_t` snapshot | 200 | 200 | 0% | Historical comparison snapshot, not today's expanded diagnostics |
| Public `robot_status_t` snapshot | 472 | 472 | 0% | Historical combined status delivered to consumers |
| Whole historical `robot_bus_t` | 1,344 | 1,344 | 0% | Storage comparison; this is not one transmitted message |

Each compact stored event uses an effective **nine bytes** across the arrays:
four ID bytes, four generation bytes and one metadata byte. This is not a claim
that the public event structure becomes nine bytes.

Buddy 1 owns MQTT serialization. Its binary status-frame work is a separate
change, so no network-packet saving is attributed to Buddy 2's private storage
optimization. Today's 21-byte status frame and separate 26-byte encoder frame
must not be substituted for the historical inter-task sizes above.

## Measurement method and validation

1. Historical linked RAM: read `text`, `data` and `bss` from the paired
   production ELF size records; calculate RAM as `data + bss`.
2. Target type/message sizes: compile arrays whose lengths are `sizeof(type)`
   and inspect their symbol lengths with ARM `nm`. This avoids running target
   code or substituting host alignment/enum assumptions.
3. Named static globals: compare symbol lengths for `motion`, `capture`,
   configuration and driver flags in the archived firmware symbol lists.

Representative commands, using the appropriate snapshot headers/ELF:

```powershell
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -O2 -std=c11 -I<snapshot-include> -c probe.c -o probe.o
arm-none-eabi-nm -S --radix=d probe.o
arm-none-eabi-size <firmware.elf>
```

A probe declares, for example,
`const unsigned char request_bytes[sizeof(robot_request_t)] = {0};`.
The existing [memory probe](../tests/buddy2_compact/memory_probe.c) uses this
method for the motion, command, event, capture and other core types.

Recorded implementation validation comprises 15 regression suites and two
additional compact/driver suites. The compact suite performed **85,465 checks**.
The later saved [compact/driver host log](../testing/subsystem2/logs/2026-11-24_compact_representation_motor_driver.txt)
also records these checks passing. The report's fresh ARM probes confirm the
layout sizes; hardware accuracy, runtime stack usage and control timing were not
measured for this report.
