# Interfaces, states and ownership

Production motion is enabled in normal builds. Calibration profiles retain
their inhibited controller. See [production integration](PRODUCTION_INTEGRATION.md)
for the correct UF2, build flags and SOURCE_LINE examples.

`include/robot.h` is the task-context public interface. `include/motion.h` is the
single-owner portable core: only Motion Control may call its mutation functions
in target firmware. `motion_hal.h` is private to startup/Motion Control (passive
encoder reads are permitted by the manual diagnostic console).

## Commands

Buddy 3/Buddy 5 steering helpers and encoder feedback are in `include/steering.h`.
See [steering and line-tracking](STEERING.md) for continuous minor corrections,
heading changes over a distance/time and robot tuning. Those helpers use degrees;
the original API below uses radians. ARC_MOVE core commands add `angle` radians
and `duration` seconds to the by-value command; duration zero selects encoder
distance stopping, while a positive duration selects a timed smooth trajectory.

All ordinary calls return immediately after copying a request. `R_OK` means
queued, not physical completion or even controller approval. Watch the status and
events for the same command ID. SI units: metres, seconds, radians, m/s; positive
wheel speeds mean robot forward, positive angle means left/counterclockwise.
`motion_turn_left(id, 3.141592653589793)` is a U-turn. Forward/backward and left/right
wrappers require nonnegative magnitudes; negative, NaN and infinity are rejected.

`robot_submit(source, id, mode, value, left, right)` is the integration point for
line, barcode, obstacle, terrain and WiFi owners. Distance uses signed `value` in
metres; turn uses signed radians. Speed mode uses `left/right` in m/s; distance
and turn choose their cruise speed from configuration. Manual output uses signed
duty fractions and requires `manual_enabled` plus all critical hardware values.
Finite manual pulses end in a cancellation hold; they cannot persist after their
deadline. Only the test console should use this mode.

```c
/* Called by a real line-following task after producing a genuine steering request. */
motion_result_t result = robot_submit(SOURCE_LINE, 42U,
    WHEEL_SPEED_TRACKING, 0.0, 0.10, 0.08);
/* Refresh ID 42 while it owns motion; handle R_BUSY/R_FULL/R_HELD explicitly. */
```

No command writes motor hardware directly. Motion Control performs final checks:
state, mode, finite values, configured limits, generation, expiry and deadline.
Speed targets with the same active ID replace targets without resetting PID.
Older refresh timestamps are rejected. A different active ID or incompatible
mode is rejected; first cancel, wait for STOPPED, release after checks, then submit
a new command. Do not retry a failed request indefinitely without observing state.
Zero distance/angle still pass through stopping and final validation. Zero wheel
speeds remain a freshness-monitored speed session until cancelled.

## Communication and timing

The target uses the actual micro T-Kernel `TA_INHERIT` mutex, not volatile-only
sharing. Request and event rings each have eight by-value entries; approved motion
has one slot. No caller-owned pointer outlives a send. Normal request/status APIs
poll the mutex (`TMO_POL`) and return busy/false on contention. Full request rings
return `R_FULL`. Pending continuous targets for the same source and ID overwrite
their previous entry instead of creating a steering backlog. The controller
processes at most one ordinary request per cycle.

Stop/fault/shutdown are separate latched control bits, never entries behind
ordinary navigation. These calls briefly acquire the inheritance mutex; they do
not wait for physical motion. They immediately invalidate pending generations and
clear queues. Motion applies controls before ordinary commands and rechecks them
before writing PWM. Stop latency is bounded by scheduling and a control cycle,
subject to on-board timing validation. These APIs are task-context only.

Core events have a separate eight-entry ring. Motion drains them into the controller
event ring; overflow latches a local fault and inhibits outputs even if no event
can be sent. The snapshot retains the fault, command ID and drop count. An existing
root fault is retained rather than overwritten by overflow. The controller consumes
completion/fault events and exposes the last event. Extend event dispatch there
if multiple subscribers need every event; the last-event snapshot is not a log.

Generation changes on cancellation, fault, reset, release and shutdown invalidate
old approved/pending requests. Command IDs identify results; use a fresh nonzero ID
for each finite command and one stable ID for a continuous session. Deadlines use
the local monotonic clock, not wall time or a remote clock. Supplied absolute core
timestamps are bounded by configured lifetimes. Generations are 32-bit and must
not be deliberately wrapped/reused within a boot session.

Motion priority 2, Controller and Obstacle 4, Line/Barcode/Terrain 8, WiFi 12:
smaller micro T-Kernel numbers mean higher priority. Proposed Motion period 10 ms,
Controller 10 ms; Motion measures real elapsed time and blocks between iterations.
Periods/priorities and 4 KiB task stacks require measurement under interrupt load.
Placeholders sleep indefinitely. No control loop allocates memory. Short mutex
sections cover copies, bounded command validation and event processing; PID and
odometry run outside the mutex. Never hold this mutex during network I/O, printing,
sensor processing or delays.

## Snapshot for other buddies

Buddy 2 exposes live encoder data through `motion_get_status(&status)`:
`status.motion.encoder.count[0/1]` are left/right raw `uint32_t` counts,
`encoder.invalid[0/1]` are cumulative invalid quadrature-transition totals,
and `encoder.reliable` is the acquisition reliability flag. Use
`status.motion.feedback_trusted` for the controller's qualified feedback health;
raw acquisition reliability alone does not prove that an encoder works.
`status.motion.encoder_sample_time` is the sampling time in seconds; consumers
must check freshness and finite timestamps. Counts update on each motion sample,
including in FAULT, independently of the preserved `feedback_fault_sample`.
Counts wrap modulo 2^32 and may decrease for reverse rotation; compare modular
deltas, not unsigned greater-than tests. A stationary wheel legitimately has an
unchanging count. These are raw counts, not reset-relative distance accumulators;
reboot/reinitialization can restart their baseline.

Buddy 1 reads these fields from the existing copied snapshot every 100 ms and
publishes them as the separate 26-byte encoder frame on `robot/encoder` (QoS 0),
straight after the status frame with the same SEQ. It carries both raw 32-bit
counts and invalid totals unclamped, plus encoder-reliable, feedback-trusted and
fresh flags, the feedback cause, motion state and fault, the sample time in ms
and its age on the robot's clock. The layout is in `include/telemetry_frame.h`
("Encoder frame"); the 21-byte status frame is unchanged. Never read encoder
hardware or perform network I/O in the motion loop for telemetry.

`motion_get_status` copies a snapshot; publish after releasing all application
locks. `robot_update_subsystem` copies a genuine owner sample. `available=false`
means no measurement, not a zero measurement. Proposed `values[4]` contracts:

| Owner | values[0..3] |
|---|---|
| Line | line error -1/0/+1, trackable (> 0.5), junction flag, reserved |
| Barcode | `tf_barcode_t` (left, right, straight, U-turn), valid (> 0.5), reserved, reserved |
| Terrain | heading rad, pitch rad, hump height m, validity bits 0..2 (`TF_TV_*`) plus terrain state << 3 |
| Obstacle | nearest distance m, scan angle rad, width m, scan flags (`TF_OF_*`, 5 bits) |
| WiFi | connected flag, publish count, receive count, reserved |

Buddy 1 copies this snapshot every 100 ms and publishes it as the 21-byte binary
frame defined in `include/telemetry_frame.h` (field ranges, bit widths, signs and
units are in that one table; the mapping is `app_program/subsystem1/telemetry_map.c`).
A value reaches the dashboard only through these slots. Out-of-range values are
clamped, not wrapped. One-off events (hump peak, obstacle, scan done, B5->B3
handover) are latched with `comms_latch_event(TF_EV_...)` from `comms.h`; motion
fault, line lost and new barcode are detected by Buddy 1 from the snapshot.
Commands are 2-byte binary (opcode, session, seq); see the header.

These are interface slots, not implemented algorithms. Agree enum/validity values
before integration. A magnetometer cannot supply pitch or hump height by itself;
leave unavailable fields invalid until Buddy 4 confirms the sensor. Invalid/nonfinite
status payloads return false. Epoch timestamp is monotonic seconds.

## State and fault policy

Production runtime ownership: `app_program/subsystem2/motion_runtime.c` owns
the motion instance, periodic task and motor fault diagnostics.
`app_program/controller/robot_runtime.c` owns the request bus, its mutex,
submission/status APIs and controller task. `app_program/system/system_runtime.c`
owns watchdog/crash handling and system supervision. `app_main.c` initializes
these components and creates/starts the same RTOS tasks; public APIs below remain
declared in `include/robot.h`, `include/motion.h` and `include/steering.h`.

INITIALISING keeps outputs stopped, verifies configuration/setup and requires a
fresh settling interval. READY accepts commands. DRIVING has distance, encoder
turn, continuous-speed and explicitly enabled manual modes. Finite motion slows
using remaining wheel distance and an acceleration limit. Straight correction
only applies to distance mode, never deliberate unequal wheel speeds.

STOPPING applies configured brake/coast immediately, observes wheel speeds and
checks its deadline. Completion requires a settling interval and final centre
distance/heading tolerance, plus a drift/centre constraint. Cancellation sets a
hold and enters STOPPED after settling. STOPPED rejects motion until checks and
stationarity allow a controller release. `motion_stop` also serves as controller
hold or mode-change barrier. Stop during STOPPING sets the hold without extending
the stop deadline. Recovery is stop/hold followed by explicit checks/release.

FAULT stops outputs, invalidates motion and never restarts automatically. Fault
beats simultaneous cancellation; shutdown is terminal but retains any fault cause.
`robot_system_fault` is the sensor/controller route to the fault latch.
Remote ESTOP uses `F_ESTOP` (11); internal failures use `F_SYSTEM` (10).
Hardware initialization failure uses `F_SETUP` (2); startup timeout uses
`F_SETUP_TIMEOUT` (12). Existing codes and four-bit telemetry layout are preserved;
M_FAULT receivers must accept codes through 12. See [fault diagnostics and recovery](MOTION_FAULT_DIAGNOSTICS.md).
Reset requires explicit cause-cleared confirmation and rest. A feedback/timing
fault invalidates stationarity: missing pulses cannot clear it. An operator or
independent trusted sensor must verify rest, repair/check feedback, and pass the
external-rest flag. Reset reinitialises into a hold; wait for STOPPED and explicitly
release. Configuration/setup changes require rebuilding/restarting the target.
Known encoder faults also survive watchdog/hard-fault restarts and restore
`F_FEEDBACK` until an accepted checked reset. Startup silence is not proof of repair.
This latch is held in watchdog scratch storage and clears on removal of power.
Shutdown prevents release/reset/new motion for the remainder of the boot session.

Corrections to supplied statechart: the shared recovery arrow must not erase a
FAULT; it remains latched. `hold && goal_ok` in the stopping-to-fault label is
contradictory: cancellation does not require reaching the abandoned goal. This
implementation faults only failed *normal* final-goal validation. A stop output
command is not equivalent to stationary wheels.

Detectable faults include invalid configuration/setup, startup/stop deadlines,
missing expected progress (grace plus two count periods at target speed), excessive
or nonpositive elapsed time, invalid quadrature transitions/impossible count rate,
continuous expiry, movement deadline, final goal error and event overflow. No
motor-current, temperature, driver-error or independent slip sensor is invented.
Missing progress can be stall, wiring, encoder loss or another cause.

## Encoder and PID details

Quadrature uses x4, both edges of A and B. Positive sequence `00,01,11,10,00`
with A as bit 1; `encoder_sign` aligns each physical wheel to forward. Two-bit
changes increment the invalid counter and fault on the next sample. Single channel
counts A rising edges and infers sign from the last robot-forward motor command;
encoder_sign has no independent physical meaning in that mode. Coast, reversal
and external motion can be signed incorrectly. Use quadrature for reliable signed
odometry. Coalesced/lost GPIO edges and complete pulse-cycle losses cannot all be
detected; validate maximum ISR edge rate. No button-style debounce is applied.

Counts are uint32 modulo counters; differences are formed in uint32 and converted
via int64, avoiding signed overflow. Fewer than 2^31 edges per sample is required.
Both wheels and invalid counters are snapshotted with interrupts disabled briefly
on core 0. Travel is count delta times circumference divided by software counts
per wheel revolution. Heading is (right-left)/track. `centre` is signed integrated
centre displacement, `path` accumulates absolute centre increments (an in-place
turn adds zero centre path). Quantised speed uses measured elapsed time. No claim
is made that odometry measures actual ground motion when wheels slip.

PID derivative is on measurement with a configurable first-order filter, so target
refresh does not cause derivative kick. Integral uses conditional anti-windup and
limits; state resets at stops/faults/mode transitions, not speed refresh. Per-wheel
gains and PWM may differ to achieve equal wheel speeds. Configure stopping distance,
low-speed resolution and settling jointly: pulses absent for the settling interval
cannot prove zero speed below the encoder's measurable resolution.

Subsystem 4 (when `ROBO_SUBSYSTEM4=ON`) supplies the Terrain snapshot every
20 ms. The validity mask uses bit 1 (value 2) for pitch and bit 2 (value 4) for
hump height; bit 0 (heading) stays clear because the magnetometer is not used.
`values[0]` is the turn rate in rad/s (+ = left), from Buddy 2 wheel speeds.
Bits 6..8 of `values[3]` carry the motion class (0 STATIONARY, 1 MOVING,
2 ACCELERATING (speed changing, braking included), 3 REVERSING, 4 TURNING, 5 CLIMBING, 6 DESCENDING, 7 OVERTURNED) and
bit 9 marks the class and turn rate valid; decode with `terrain_motion_class()`.
`values[2]` is the highest hump peak of the run in metres (1 mm steps), measured
from the sideways tilt of the axle for humps under one wheel. `values[3]` also carries
the 3-bit `T_STATE` << 3: 0 INIT, 1 WAITING, 2 CLIMBING, 3 DESCENDING, 4 RECOVERY,
5 FAULT, 6 STOP, 7 LIFTED. `available` is true once the task runs; the timestamp is the
last valid tilt sample time. Hump completion latches `TF_EV_HUMP_PEAK`. No MQTT
text topics are sent. See app_program/subsystem4/README.md. Buddy 4 never changes the robot's speed. Its one
motion command is `motion_stop()` (controlled STOP, released by START) when the
robot is lifted (T_STATE 7) or overturned (motion class 7); this replaced IMU
collision detection (Change Requirement, 11 November 2026). Decode the slot with
`include/terrain_status.h` rather than by hand: `terrain_published_state()` gives
T_STATE (`TERRAIN_TS_UNKNOWN` = 0xFF when unusable), and
`terrain_imu_faulted(&status)` (T_STATE 5) is the IMU fault indicator; an IMU
fault never stops the motors. Buddy 4 enters
STOP (T_STATE 6, completed peaks kept) when motion reports `SHUTDOWN`. A new run
(statechart NEW_RUN) and recovery from a latched IMU fault are a power-up, which
the level tilt reference needs anyway.
