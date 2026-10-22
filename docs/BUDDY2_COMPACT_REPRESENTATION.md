# Buddy 2 compact representation and validation

Implementation date: 22 October 2026. Scope: Buddy 2 motor driver, motion core,
encoder capture, private storage headers, tests and this documentation. No changes
to the controller, app startup, MQTT, dashboard or Buddies 1/3/4/5 for this task.
Pre-existing workspace changes in those areas were preserved. No applicable
AGENTS.md was found in the workspace or its ancestors.

The supplied BARR-C:2018 PDF is a coding-style reference, not an instruction to
change robot behaviour beyond the user's explicit requirements.

## Implemented storage and conversion

Public `motion_command_t`, `motion_status_t`, `motion_event_t`, `encoder_sample_t`,
robot bus layouts and teammate function signatures remain unchanged. The core's
owner-only `motion_t` command/event/flag storage changes; rebuild all objects.
Do not access those internal fields from another subsystem. The unchanged
controller still uses `motion.status`, `motion.active` and the normal core APIs.

`motion_accept()` performs the original identity, generation, time, state, mode,
configuration, manual-enable and physical-trajectory checks before storing the
compact command. It encodes/decodes into local temporary objects, revalidates
rounded ARC trajectories, and only then commits the command. Failed conversions
do not change the running command. Control calculations decode an owner-local
SI-unit view; invalid stored data stops motion with a system fault.

| Command field | Public representation | Valid range and resolution | Minimum signed bits | Private allocation |
|---|---|---|---:|---|
| Distance/ARC centre travel | binary64 metres | -5000..5000 mm, 1 mm | 14 | payload bits0..13 |
| Left wheel target | binary64 m/s | -250..250 mm/s, 1 mm/s | 9 | bits14..22 |
| Right wheel target | binary64 m/s | -250..250 mm/s, 1 mm/s | 9 | bits23..31 |
| Turn/ARC heading change | binary64 radians | -360..360 degrees, 1 degree | 10 | bits32..41 |
| Mode | public enum | five modes | 3 unsigned | bits42..44 |
| Reserved | none | must be zero | 3 | bits45..47 |

The combined numerical/mode payload is **45 bits in six bytes**. It is a defined
byte/bit layout, not a compiler C bit-field layout. All bit operations are on
unsigned integers. Negative values are encoded as unsigned two's-complement codes;
decoding uses bounded signed subtraction instead of signed shifts or an
implementation-defined large-unsigned-to-signed cast.

Bounds are checked before rounding. Nearest rounding uses halfway away from zero:
+0.5mm -> +1mm, -0.5mm -> -1mm, -0.49mm -> 0mm. Maximum nominal quantization errors
are 0.5mm, 0.5mm/s and 0.5degree. NaN, infinity and out-of-range values are rejected;
rounding is never used to pull an invalid input back inside the allowed range.
Configuration must fit the codec's envelopes: max_distance<=5m, max_speed<=0.25m/s,
max_angle<=2pi radians. Current board values meet those bounds. A rounded target
outside a smaller configured limit is also rejected. A future larger range needs
an explicit codec/schema change, not silent truncation.

Signed distance is retained. Magnitude0..5000 requires13bits and a separate
direction requires1bit: this uses the same14bits and does not save memory.

Identity (`id`, `generation`) remains uint32, and absolute expiry/deadline times
remain binary64 seconds, copied without quantization. No timestamp wrap protocol
or ID-width change is introduced. The full stored command is56bytes including
2tail-padding bytes: 8identity +16timestamps +24extension +6payload +2padding.

The extension preserves binary64 manual PWM fractions, ARC duration and untimed
ARC wheel-speed proportions. These are different quantities/contracts from
independent wheel-speed commands. For timed ARCs, rounded distance/angle goals
derive exact wheel velocities as goal/duration; those derived velocities are
not independently rounded to1mm/s, which would break the trajectory identity.
Both original and reconstructed trajectories must satisfy speed, direction,
duration and acceleration limits. An ARC rounded to zero distance and angle is
rejected. Public continuous-steering helpers retain their existing documented
curvature-preserving saturation before submission; the compact core itself
rejects an out-of-range command rather than saturating it.

## Field audit

Sizes below are measured with the actual RP2040 ARM toolchain. Its enum types
here occupy **one byte**, unlike the four-byte enums of the host test compiler.
Do not infer firmware memory from host sizeof values.

| Fields/group before this change | Previous type | Range, units, resolution | Required bits / decision |
|---|---|---|---|
| Command distance, velocity, angle | double | bounds above, previously continuous binary64 | 14/9/9/10 implemented internally |
| Command ID/generation | uint32_t each | full0..2^32-1, identity | retain32; no session bound justifies less |
| Expiry, deadline, duration | double | monotonic seconds; lifetime0.3s, movement30s; absolute uptime unbounded | retain binary64; no new time rounding/wrap |
| Motion state | enum, ARM8bits | INITIALISING..SHUTDOWN, seven values | logical3; public snapshot stays ABI-compatible |
| Mode | enum, ARM8bits | five values | private3 implemented; public enum retained |
| Primary fault | enum, ARM8bits | NONE plus eleven causes | logical4; packed4 in private event metadata; public primary cause retained |
| Independent fault causes | previously only first primary cause | eleven independently seen causes |11 in uint16_t `fault_flags`; no mutually exclusive encoding |
| Event type | enum, ARM8bits | five values |3, packed beside4-bit primary fault |
| Event IDs/generation | uint32_t each | full32bit identity | retain32; event adapters reconstruct public events |
| Event head / occupancy | unsigned each | head0..7; occupancy0..8 |3+4 in one uint8_t; full and empty stay distinct |
| still-tracking, goal-pending, feedback-latched | bool each | independent false/true |3bits in owner-only runtime_flags |
| active | bool | independent false/true |1logical bit; existing controller bridge member retained |
| Public status booleans | bool each | stationary, trusted, inhibited, hold, first timing fault recorded |1each; shared ABI retained, no duplicate packed shadow |
| Feedback diagnostic cause | enum, ARM8bits | five values |3logical; public snapshot ABI retained |
| Encoder position[2] | uint32_t each | full modulo2^32 count, one x4 edge/count | retain32; reverse delta decoded in int64_t |
| Invalid-transition[2], edge totals[4] | uint32_t each | modulo2^32 diagnostic totals, one event | retain32; do not change counter lifetime semantics |
| Encoder GPIO numbers[4] | unsigned each | RP2040 GP0..29 |5logical each; uint8_t cells; four-bit-state/sign packing below gives40byte total with natural alignment |
| Encoder previous AB[2] | uint8_t each |0..3, raw two-phase state |2each, packed into one uint8_t |
| Encoder quadrature[2], signs[2], inferred directions[2] | bool[2], int[2], int[2] | quad yes/no, polarity +/-1, inferred +/-1 |6independent bits in one uint8_t |
| Encoder sample `reliable` | bool | false/true | public snapshot retained; not treated as count quantization |
| Driver initialized/IRQ/PWM-ready | bool each | three independent flags |3bits in one uint8_t |
| Driver last directions[2], neutral[2] | int[2], bool[2] | each direction -1/0/+1; each neutral false/true |2+2+1+1 bits in one uint8_t |
| Driver brake and motor polarity[2] | bool, int[2] in copied config | brake yes/no, two +/-1 signs |3bits in driver configuration.flags |
| Driver max_output/reversal_time | double | current0.25fraction /0.1s, config-dependent | binary64 retained; reversal timing unchanged |
| Motor pins | const unsigned[2][2] | GP8/9 and10/11, exact GPIO indices |4logical bits each; existing exported constant array type retained |
| PWM wrap/frequency | uint16_t / unsigned | wrap4095, frequency1000..20000Hz |12/15logical bits; hardware-width wrap retained; frequency copy removed from driver after initialization |
| PWM compare values | uint16_t | driven duty0..1024 at25%; coast may use4096 |11bits for limited drive,13 including coast; uint16_t hardware operands retained; atomic slice write unchanged |
| Neutral timestamps[2] | double | seconds, unbounded uptime | retain binary64 |
| PID gains[2], derivative time constants | double | nonnegative finite config, no specified universal maximum | retain binary64; no safe fixed-point range proven |
| PID integral/previous/derivative[2] | double | integral bounded by output; history/derivative depend on measured speed, gains and dt | retain binary64 intermediates |
| PID seeded[2] | bool | yes/no | logical1 each; existing pid_t API/layout retained, including padding |
| Odometry previous count[2] | uint32_t | same exact modulo counts | retain32; no mm quantization |
| Wheel travel, centre, path, heading | double | metres/radians; cumulative lifetime not bounded by per-command5m/360degrees | retain binary64, no finite small-bit accumulator proven |
| Estimated wheel speeds | double | m/s, one encoder count divided by measured dt | retain binary64; feedback is not restricted to commanded1mm/s steps |
| Motion output/target telemetry | double | duty fraction /m/s, ramp increments at10ms | public layout and full intermediate precision retained |
| Start counts and previous invalid totals | uint32_t | full modular counter domains | retain32 each |
| Start wheel travel, progress/expectation/settling timestamps | double | metres/seconds, variable uptime | retain binary64 |
| Timing diagnostics / startup gaps | double /uint32_t | seconds and cumulative gap count | retained; no timing policy relaxation |
| Geometry, tolerances, limits, periods, gains in config | double; signs int; booleans bool | current board config validated by motion_config_valid | authoritative public config retained; only redundant HAL copy reduced |
| Encoder diagnostic pins/AB/functions/IRQ flags | unsigned /uint8_t | pins0..29; AB0..3; function0..31; IRQ0..15 | logical5/2/5/4; public diagnostic adapter retained |
| Calibration-only motor/button/angle diagnostics | enums/unsigned/int/bool/double/uint32_t | wheel0..2; direction+/-1; state finite; counters32; timed outcomes | standalone test interfaces retained; do not shorten timestamps/counts or alter working test behaviour for production RAM savings |

Explicit public-state packing would require a new snapshot schema/accessor boundary
in the central controller and telemetry consumers. That cross-team change is
**not implemented** under Buddy2-only scope. The same applies to the shared robot
request queue, which still stores full public commands. A coordinated future
change could pack state/mode/fault/public booleans and bus command queues, but
must update every consumer and the synchronization contract together.

## Encoder, PID and odometry precision

At production left geometry67.5mm/2542.4counts, one count is0.083408395mm.
At10ms this represents about8.34084mm/s per count in the instantaneous estimate.
Quantizing feedback to1mm would discard roughly12counts of wheel travel. Only
commands are coarsened; no counts, phase states or reverse information are lost.
The existing unsigned modular addition and signed relative delta handle rollover
in either direction, provided each relative interval is less than2^31counts.

Tests demonstrate that binary32 accumulated travel at2048m cannot retain one
encoder increment: spacing there is0.244140625mm and half-spacing exceeds one
count. Binary64 retains it; a10,000step forward/reverse rollover sequence stays
within5e-9m of the independently calculated wheel travel. Since lifetime travel
is unbounded by command length, reducing accumulators is not justified. PID
fixed-point bounds are also unproven with configurable gains and dt; all existing
binary64 integration, anti-windup, filtering and intermediate arithmetic remain.
The test uses no invented hardware accuracy claim.

## Synchronization and safety

- Core command, queue, runtime and fault-flag storage remains motion-owner-only.
- Public request submission/status copies keep their existing RTOS mutex boundary.
- ISR metadata remains volatile, with coherent count-pair snapshots and direction
  flag updates under the existing interrupt-disable critical sections. Volatile
  is not substituted for locking. Packed read/modify/write does not cross cores.
- GPIO IRQ dispatch still acknowledges only Buddy2-owned encoder latches. Counter
  snapshots and edge/invalid totals remain individual per wheel.
- Primary fault remains first-cause latched; fault_flags additionally ORs every
  independently raised cause. Reset/init clears this history. No automatic restart.
- Stop, generation invalidation, command refresh identity, expiry and shutdown
  precedence are unchanged. Motor ownership and reversal neutral-time checks remain.
- Nominal production period remains10ms; max running dt remains50ms. No claim of
  physical10ms execution or USB stability is made by passing host tests.

## Measured memory (ARM, including padding)

Same production settings before/after: PicoW, GCC15.2.1, O2, SMPoff, WiFi/network
off, USB CDC, calibration/IR modes off. The baseline was built from the actual
workspace before this task's storage edits, not from an older Git revision.

| Object/type | Before bytes | After bytes | Reduction |
|---|---:|---:|---:|
| Active command storage |72 |56 |16 |
| Private8-event queue payload |128 |72 |56 |
| Whole motion_t |936 |856 |80 |
| Whole encoder_capture_t |72 |40 |32 |
| Motor driver's configuration copy |304 |24 |280 |
| Driver state/directions/neutral named globals |13 |2 |11 |
| Named affected globals combined |1359 |956 |403 |
| Public motion_command_t |72 |72 |0 |
| Public motion_status_t |200 |200 |0 |
| Public motion_event_t |16 |16 |0 |
| Each pid_t |32 |32 |0 |
| odometry_t |64 |64 |0 |

Padding examples: private command54member bytes+2tail padding=56; capture38member
bytes+2padding=40; motor config17member bytes+7padding=24; each PID25member
bytes+7padding=32. Public command retains its ABI padding; individual old events
were16bytes despite one-byte ARM enums. Structure-of-arrays removes those per-entry
gaps. Whole-type sizes and actual symbol sizes, rather than theoretical bit sums,
are used for the savings claim.

Linked production image before: text95620, data28, bss20804bytes.
After: text99716, data28, bss20404bytes. Thus actual linked static RAM is
**400bytes smaller** (20832 ->20432), with **4096bytes additional text/flash**.
The403bytes of named-object reduction differ from the400byte image reduction
because of linker/compiler alignment. This is static RAM, not a measured runtime
stack high-water mark. Temporary conversion views can increase stack use; task
stack margin and control latency still need target measurement.

Evidence: `build-buddy2-compact-before/{type-sizes,buddy2-symbols,image-size}.txt`
and matching files in `build-buddy2-compact-after`. ARM object array probes in
`tests/buddy2_compact/memory_probe.c` encode sizeof values as symbol lengths.

## Tests, builds and reproducibility

- Existing15CTest suites: passed after rebuilding, including encoder sequences,
  reverse/rollover, motion simulation, final goals, stop/expiry/staleness,
  event overflow, timed arcs, steering and isolated actuator cutoffs.
- New compact suite: 85,465 checks, including exhaustive signed-field integer round trips and public
  distance/velocity/angle limits; rounding, invalid numbers and corrupt encodings;
  exact identity/timestamps; ARC/manual adapters; independent fault history;
  expiry and stale/foreign refreshes; numerical precision/rollover checks.
- New actual motor-driver host suite: PWM polarity, independent wheel reversal
  neutral time, inhibition, coherent encoder capture, diagnostics and emergency stop.
- Host checks use strict warning flags and undefined-behaviour instrumentation.
  These are software/fake-register results, not on-robot performance measurements.
- Production firmware built in `build-buddy2-compact-after`; UF2 is
  `build-buddy2-compact-after/buddy2_production.uf2`. This is the production image,
  not the standalone serial motion-test firmware used for the mat experiments.

```powershell
cmake -S tests/buddy2_compact -B build-buddy2-compact-tests -G Ninja -DCMAKE_C_COMPILER="C:/Y2 Tri 1/Embedded Systems/Assignment/INF2004_SmartRobo/.tools/zig/ziglang/zig.exe" -DCMAKE_C_COMPILER_ARG1=cc
cmake --build build-buddy2-compact-tests -j4
ctest --test-dir build-buddy2-compact-tests --output-on-failure
cmake --build build-steering-host -j4
ctest --test-dir build-steering-host --output-on-failure
```

Root CMake is deliberately unchanged for this task: Buddy2's existing production
source discovery and header adapters require no central build-file changes.

## BARR-C application and remaining hardware validation

New storage/adapter code uses fixed-width integers, unsigned mask operations,
explicit bounded signed conversion, named layouts, static assertions, descriptive
module-prefixed types/functions, pointer/Boolean prefixes, braces and finite-input
checks. Binary64 support is asserted; new storage uses a float64_t alias because
the C11 toolchain does not define that type. Header guards and existing ownership
are preserved. No heap allocation, signed C bit-fields or packed unaligned double
accesses are introduced. The new encoder implementation uses this style too.

Compatibility exceptions are explicit: old public names/types and existing
legacy formatting/zero sentinels are retained; the register API supplies some
unsigned/int arguments; host fake-HAL tests match those existing prototypes.
The build has not been certified fully BARR-C compliant by an independent static
analysis tool. No claim is made that untouched legacy modules now conform.

Hardware still required before claiming production calibration/accuracy:
1. With wheels raised, verify forward/reverse polarity, independent counts,
   physical stop, reversal delay, timeout, disabled startup and fault inhibition.
2. Test fractional/negative/limit commands through teammates' production APIs;
   confirm refresh expiry stops the bot and stale generations cannot restart it.
3. Repeat measured distances, speed checks and left/right15/30/45/60/90/180degree
   turns on the mat, especially near rounding boundaries. Visual angle checks
   from the earlier standalone firmware are not precision measurements of this build.
4. Measure actual motion-step dt, worst case packing/decoding time, IRQ latency,
   dropped encoder transitions and task stack high-water under combined-task load.
   Nominal10ms is a target, not a hardware result.
5. Check encoder counts/invalid totals under realistic wheel speed and USB load,
   full stop/hold/release/fault-reset paths and stopping distances.

The standalone test's mat scale0.971022,4.5mm braking allowance and open-loop
correction have **not** been transferred into production PID by this representation
change. Production geometry/control policy and hardware-confirmation flags are
unchanged from the task's starting workspace. That integration is a separate
behavioural change and must not be claimed complete here.
