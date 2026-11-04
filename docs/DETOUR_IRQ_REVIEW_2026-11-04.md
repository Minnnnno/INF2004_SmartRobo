# Detour completion and interrupt review - 4 November 2026

## Reproduced motion-control defect

The supplied log records move 5101 failing with `F_FEEDBACK / FB_NO_PROGRESS`
after the outward turn. It also records both wheels travelling forward during
that move, followed by a small reverse request on the left wheel. Thus this
capture reached the diagonal leg; it does not establish that every later stop
reported by the user is the same fault.

Subsystem 5 submits its straight legs through `motion_steer_distance()`, which
uses `ARC_MOVE` with zero angle. The untimed arc controller previously required
each individual wheel to be within:

```text
min(distance_tolerance / 2, angle_tolerance * track / 4)
= min(5 mm, 1.4375 mm)
= 1.4375 mm
```

That unnecessarily applies the tight angular constraint to common distance
error. For example, both wheels being 4 mm ahead means a 4 mm centre error and
no heading error. It fits the existing stopping allowances, but the old code
kept trying to reverse. If the wheels cannot move for that correction, the
normal no-progress protection aborts the detour.

A regression with measured endpoints 2.2 mm and 1.1 mm beyond the wheel targets
failed before the fix with `state=5 fault=4 feedback=4`, matching the log's fault
codes. This is a controlled host fixture, not a reconstruction of the physical
wheel dynamics or proof of the exact hardware cause.

The controller now tests centre distance and heading separately to enter
STOPPING: within 5 mm and 0.025 rad (about 1.43 degrees). After the wheels settle,
the existing full limits of 10 mm and 0.05 rad still decide success. Timed arcs
still wait until their specified duration. Output caps, feedback timeouts and
other motion modes were not changed by this fix.

Tests cover forward/reverse/curved endpoints, common distance error, an incorrect
distance with correct heading, an incorrect heading with correct distance,
and coasting beyond either final limit after entering STOPPING.

## Interrupt printing audit

The source and compiled ARM call paths were reviewed for the active single-core,
USB CDC, Wi-Fi-off, subsystem-4-off configuration.

| Context | Finding |
| --- | --- |
| Encoder GPIO IRQ | Fixed-size register reads, acknowledgement and quadrature updates; no console calls or floating-point control loop. |
| SysTick | Kernel timers and wait callbacks; no console calls in the active callbacks. Application alarm/cyclic handlers are not registered in this build. |
| Freeze-check NMI | Timestamp/counter check, and save/reset on failure; no console calls. |
| USB IRQ | Controller/event handling and service-task wakeup; no console calls. TinyUSB logging is disabled (`CFG_TUSB_DEBUG=0`). The CDC driver has no SOF callback. |
| HardFault | **Contained `tm_printf` calls**, enabled by `USE_EXCEPTION_DBG_MSG=1`. Removed. PC/LR/task capture, motor cutoff and reboot remain; `usermain()` prints the saved report after reboot. |
| Default and weak NMI/SVCall fallbacks | **Contained console output.** Removed. These are fatal fallback paths, not normal periodic handlers. Their existing halt behaviour remains. |
| Obstacle scan/detour messages | Execute in `obstacle_task`, not in an ISR. Servo and ultrasonic timing use hardware PWM; the obstacle task waits/polls for completion. |

Disassembly of the new hard-fault reporter shows its only function call is the
motor cutoff; scratch-register writes and watchdog reset are inline. No print
calls remain in the ARMv6-M exception source. Direct-call audit results are in
`build-detour-irq-fix/interrupt-console-audit.txt`; indirect timer/USB callbacks
were also inspected in source. Host tests cannot validate interrupt latency.

Two latency qualifications remain: TinyUSB's `reset_ep0()` contains a hardware
abort-completion wait in the USB control-transfer IRQ path, and the task-stack
diagnostic scans a bounded allocated stack under an interrupt lock. Neither
has been timed on hardware. Neither is established as the cause of this log's
encoder no-progress fault. The USB wait was not modified; the reported
power-bank-only reproduction makes USB host control transfers a weaker lead.
The per-task stack scanner was absent from the older image identified by the
user, so it cannot explain that image's failure.

The exception printing was undesirable, but runs after a fatal exception is
already raised. It does not by itself explain the supplied capture continuing
to print normal task status after a motion fault.

## Added task-level progress messages

Each accepted detour submission now prints its ID, requested movement and a
completion line. For the supplied plan, the expected sequence is approximately:

```text
[DETOUR] move 5100 queued: turn -66 deg to heading -66 deg
[DETOUR] move 5100 complete: turn
[DETOUR] move 5101 queued: straight 230 mm at 60 mm/s
[DETOUR] move 5101 complete: straight
```

`queued` means accepted for arbitration, not proof that the motor has started.
Faults retain the recurring `[FAULT]` snapshots added in the previous review.

## Validation and image to test

- All 10 normal CTest suites passed after the motion/exception changes.
- The expanded motion and obstacle runtime suites also passed with MSVC
  AddressSanitizer, including the added post-stop final-goal checks.
- Strict ARM GCC checks passed for motion and obstacle runtime test sources.
- Both the dedicated analysis build and the user's usual `build` firmware
  output compile successfully. No firmware was flashed or run on the robot.

The user identified `build/mtkernel_3.uf2` as the image being used. That folder
previously contained build ID `b2-line-steering-v1-20261104T024005Z-no-s4`, without
the latest per-task stack diagnostics. Its UF2 and matching ELF are preserved
under `build/pre-detour-irq-fix-20261104T024005Z/` before rebuilding.

**Updated normal output:** `build/mtkernel_3.uf2`.
Pinned copy: `build/buddy2_detour_irq_fix.uf2`.
Matching pinned symbols: `build/buddy2_detour_irq_fix.elf`.

Build ID: `b2-line-steering-v1-20261104T033007Z-no-s4`.

UF2 SHA256: `078AB0C2704663CF177E0ED1AB664A1A2556355E6231B41A3067166DB0B34A61`.

Flash with motor power disconnected. On the next run, verify the recurring
firmware ID and capture the queued/completed move lines and any `[FAULT]` or
`LAST CRASH` report. Hardware confirmation is still required.
