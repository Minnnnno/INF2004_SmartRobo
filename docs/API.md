# Interfaces, states and ownership

`include/robot.h` is the task-context public interface. `include/motion.h` is the
single-owner portable core: only Motion Control may call its mutation functions
in target firmware. `motion_hal.h` is private to startup/Motion Control (passive
encoder reads are permitted by the manual diagnostic console).

## Commands

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

`motion_get_status` copies a snapshot; publish after releasing all application
locks. `robot_update_subsystem` copies a genuine owner sample. `available=false`
means no measurement, not a zero measurement. Proposed `values[4]` contracts:

| Owner | values[0..3] |
|---|---|
| Line | normalised line error, confidence [0,1], junction flag, reserved |
| Barcode | decoded team command enum, confidence [0,1], reserved, reserved |
| Terrain | heading rad, pitch rad, hump height m, validity bitmask |
| Obstacle | nearest distance m, scan angle rad, width m, blocked flag |
| WiFi | connected flag, publish count, receive count, reserved |

These are interface slots, not implemented algorithms. Agree enum/validity values
before integration. A magnetometer cannot supply pitch or hump height by itself;
leave unavailable fields invalid until Buddy 4 confirms the sensor. Invalid/nonfinite
status payloads return false. Epoch timestamp is monotonic seconds.

## State and fault policy

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
Reset requires explicit cause-cleared confirmation and rest. A feedback/timing
fault invalidates stationarity: missing pulses cannot clear it. An operator or
independent trusted sensor must verify rest, repair/check feedback, and pass the
external-rest flag. Reset reinitialises into a hold; wait for STOPPED and explicitly
release. Configuration/setup changes require rebuilding/restarting the target.
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
