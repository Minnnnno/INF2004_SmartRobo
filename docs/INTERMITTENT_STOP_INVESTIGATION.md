# Intermittent motor stop investigation — 3 November 2026

## Confirmed from COM4

A 45-second passive capture is saved in
`testing/system/com4_2026-11-03_initial.log`. No serial commands were sent,
and no firmware was flashed or motor settings changed during this investigation.
The capture started after the stop, so it does not include startup, the running
wheel targets, or the transition into the fault.

The repeated messages are:

```text
line_pid centre_ir=1 right_ir=0 direction=3 sweep=0 request=3 state=5 fault=4 speed_mm_s=0,0 output_permille=0,0
line_fb cause=4 reliable=1 counts=163,0 invalid=0,0 previous_invalid=0,0
[TERRAIN] Unavailable (error=-42)
```

The enums in `include/motion.h` decode these as:

| Field | Meaning |
| --- | --- |
| `state=5` | `FAULT` |
| `fault=4` | `F_FEEDBACK` |
| `cause=4` | `FB_NO_PROGRESS`: a commanded wheel failed to show encoder progress within its timeout |
| `request=3` | `R_HELD`: line following is held after the fault; this is not the original trigger |
| `counts=163,0` | Retained counts at the first feedback failure: left 163, right 0 |
| `invalid=0,0` | No invalid quadrature transitions recorded at that failure |
| `reliable=1` | The encoder driver was initialized; this flag does **not** verify that both physical encoders work |

The user reports both wheels stop together while console output continues.
That matches the fault handler deliberately disabling both motor outputs while
the scheduler and sensor/reporting tasks continue running. It does not match a
whole-system freeze. The captured fault is a feedback fault, rather than a
timing, command-expiry, movement-deadline, or watchdog fault.

## What causes the stop and why it stays stopped

`app_program/subsystem2/motion.c`, function `drive`, checks encoder progress
separately for each wheel with a nonzero target and nonzero controller output.
It faults once both conditions hold:

```text
time since motion was expected >= startup_grace
time since last encoder count change > feedback_timeout + 2 * encoder_quantum / abs(target)
```

The board configuration sets both grace and feedback timeout to 0.4 seconds.
These conditions overlap; they do not provide a combined 0.8-second grace.
At settled targets of 0.05–0.11 m/s, the no-progress timeout is approximately
0.402–0.403 seconds, evaluated on the 10 ms motion tick.

`motion_raise_fault` zeros both outputs, inhibits drive, latches the hold and
marks feedback untrusted. `line_task` does not clear that fault. Merely seeing
the line again cannot restart it, and the remote `start` command does not reset
feedback faults. The later zero speeds and zero PWM are consequences of the stop.

## Primary suspect: right wheel feedback

The right count was zero when the fault was recorded, while the left had
accumulated 163 counts. If the right wheel was turning before cutoff, its
movement was not being represented by the right encoder count. Inspect the
right encoder connection and signal path first:

| Encoder connector pin | Right wheel connection |
| --- | --- |
| 1 / GND | GND |
| 2 / A | GP5 |
| 3 / B | GP7 |
| 4 / supply | 3V3 |

The left channels are GP4/GP6. Check actual connector orientation and continuity,
including loose contacts that fail under motion. Zero invalid transitions does
not rule out a disconnected encoder or one stuck quadrature channel: one
working channel can produce cancelling forward/backward counts.

The capture does not retain the failing wheel index, encoder edge totals, or
the target/output at the failure. Consequently the physical cause is not yet
proved, and the count snapshot alone cannot establish that the right wheel
was the exact wheel whose timeout fired.

## Additional findings across the active system

* **Low-speed startup margin:** line correction requests 0.05/0.11 m/s or the
  reverse. With the configured `kp=3`, `ki=2`, acceleration ramp and no measured
  movement, a newly started 0.05 m/s wheel reaches only about 18% PWM around the
  timeout. A wheel whose breakaway duty exceeds that can trigger the same fault.
  This is a tuning possibility, not a measured motor threshold. If both wheels
  physically turn until cutoff, investigate encoder feedback before changing gains.
* **Obstacle detection:** subsystem 5 intentionally latches a stop near 250 mm,
  scans, and leaves the robot held. Its ultrasonic PWM slice 0 and servo slice 6
  do not overlap motor slices 4/5 or encoder GPIO4–7. No obstacle-stop message
  appears in this capture; an earlier obstacle event cannot be ruled out from
  this post-fault window. An ordinary obstacle stop does not by itself explain
  the recorded `F_FEEDBACK / FB_NO_PROGRESS`.
* **Line loss:** the current search logic pivots, reverses, then refreshes zero
  wheel targets after its search limit. It can stop while retaining the session,
  but that is distinct from the observed latched feedback fault. The captured
  line is detected (`direction=3`, left correction), with `sweep=0`.
* **IMU:** terrain is unavailable. `-42` is `E_NOEXS`, but `terrain_task` also
  uses it as its default read result after startup configuration fails. It does
  not prove the I2C device object is missing: the original `[IMU] ... init=...`
  startup message is needed. The task never retries a failed configuration.
  This task publishes status and does not command motor stops.
* **Communications:** local build caches disable WiFi/networking. Queued
  commands are processed separately from motion; remote `start` only releases
  an otherwise healthy settled stop. No remote reset path clears this fault.
* **Scheduling and USB:** motion has priority 2 and runs on a nominal 10 ms
  period; USB priority is 3. The shared bus mutex uses priority inheritance,
  and USB output is buffered without waiting for the host. Continued logs and
  the recorded fault do not indicate a system-wide scheduling deadlock.
* **Encoder path:** source inspection confirms independent GP4/6 and GP5/7
  capture, IRQ-protected snapshots, and no active subsystem reassigning those
  pins. This inspection cannot verify physical signals or exclude a runtime
  fault in the flashed binary. The capture contains no firmware build banner,
  so its identity was not independently verified against the local build.
* **Barcode:** the production barcode task is currently dormant.

## Next check that separates wiring from motor tuning

With motor power disconnected, use the read-only encoder diagnostic profile
described in `docs/ENCODER_INVESTIGATION.md`. The production line-following
image does not accept its `encdiag` serial command.

Rotate each wheel independently and compare counts and per-channel edge totals.
Only that wheel's count should change, and both of its channels should register
edges. Right physical rotation with no GP5/GP7 edges identifies the missing
signal path; one fixed channel identifies the channel to trace. Correct edges
with a nonchanging net count require inspecting the quadrature sequence and
capture configuration. If both encoders pass, capture startup through the next
cutoff and inspect wheel speeds/PWM before tuning motor startup behavior.

No protection was bypassed and no automatic fault recovery was added. Firmware
and host regression suites were not run because this investigation changed no
application code.
