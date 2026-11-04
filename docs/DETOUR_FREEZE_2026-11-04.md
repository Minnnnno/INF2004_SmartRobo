# COM4 detour freeze investigation — 4 November 2026

Latest review: the subsequently supplied `logs.txt` shows a live firmware
encoder no-progress fault. See [memory review and new debug firmware](MEMORY_DEBUG_2026-11-04.md)
for the buffer fix, expanded diagnostics, and latest image. The findings and
image below describe the earlier watchdog investigation.

## Second code review: additional defects corrected

The second review found independent avoidance/diagnostic defects. None is proof
of the cause of the captured watchdog resets:

- **Sensor faults counted as clear space.** `detour_ping` previously reduced
  every non-OK sensor result to a missing echo. A disconnected sensor
  (`S5_US_NO_PULSE`) or stuck ECHO (`S5_US_BUSY`) could therefore permit forward
  motion and count towards the end of the obstacle. These now abort the detour;
  initial monitoring also holds motion on those failures. A responding sensor's
  normal `S5_US_NO_ECHO` still permits clear space. Unusable conflicting side or
  front samples also abort the detour.
- **Scanning without a verified stop.** The task explicitly continued scanning
  after `s5_wait_still` timed out. A scan point also ignored failed status reads,
  movement and releases during its servo wait/sample period. It now cancels
  the sweep and holds if rest/hold is not verified before and after measurement.
- **Mixed range origins.** Scanning includes the 30 mm sensor-to-servo offset,
  but passing used raw transducer distances as servo-axis distances. This put a
  side hit about 10.3 mm too far back at the 70-degree look angle. Passing now
  includes the offset; the front clearance test uses the same origin too.
  Obstacle status publication also uses the current time after a long sweep.
- **Crash details could be lost before saving.** Both fault handlers read the
  current TCB before saving PC/LR. If scheduler corruption invalidated that
  pointer, the reporter could fault again with no saved address. PC/LR are now
  saved first, with task 255 until the task lookup succeeds. This improves
  diagnosis; it does not prevent the original fault or recover an invalid frame.

Validation:

- New runtime regressions failed against the prior behavior, and the corrected
  sensor-origin regression also failed before its fix.
- All nine CTest suites pass in `build-host-review` using MSVC, including
  27 obstacle runtime checks and 176 geometry checks. Runtime tests execute
  the actual obstacle/detour source against simulated motion and sensor APIs.
- The runtime test translation unit passes GCC's strict warning checks.
- The Pico W production firmware builds successfully. The ELF still has an
  aligned 2048-byte idle stack; disassembly confirms that both reporters write
  PC/LR before dereferencing the current TCB.
- No firmware was flashed and no hardware run was performed in this review.

Latest image: `build-detour-debug/buddy2_detour_review_fix.uf2`, with matching
symbols in `build-detour-debug/buddy2_detour_review_fix.elf`. Build ID:
`b2-line-steering-v1-20261103T170850Z-no-s4`.

SHA256: `5F2C491F37FFD5E253273AF0BA876F8F3512112BD2D13C6A2687C59005D6D7D9`.

This image includes the earlier idle-stack correction described below. Motor
output remains capped at 25%; watchdog and feedback protections remain enabled.
The original watchdog freeze still requires a hardware retest.

## Follow-up: power bank reproduction and idle-stack correction

The user subsequently reports the same mid-detour failure with only a power
bank connected. Laptop USB is therefore not required to trigger the symptom.
This does not distinguish power-bank voltage disturbance from software failure.

A concrete software defect was found in `kernel/tkernel/task.c`: the private
idle task borrowed `TMP_STACK_SIZE`, only 256 bytes in the single-core build.
All exceptions use the interrupted MSP in this kernel, so timer, encoder and
NMI handlers consume the idle stack whenever they interrupt idle. The obstacle
task's separate 8192-byte allocation cannot protect that stack.

GCC 15.2.1 `-fstack-usage` output from `build-detour-debug` gives this possible
nested path (SysTick priority 3, GPIO priority 1, NMI highest):

| Active frame | Bytes |
| --- | ---: |
| `knl_idle_task` | 16 |
| SysTick hardware frame | 32 |
| `knl_systim_inthdr` | 8 |
| `knl_timer_handler` | 16 |
| GPIO hardware frame | 32 |
| `knl_hll_inthdr` | 8 |
| GPIO `dispatch` | 16 |
| `encoder_irq` | 32 |
| `encoder_capture_events` | 80 |
| `encoder_quadrature` | 16 |
| NMI hardware frame | 32 |
| `robo_freeze_check` | 16 |
| **Total** | **304** |

SysTick masks ordinary interrupts inside its critical section, but GPIO can
preempt its unmasked entry/exit code; NMI can preempt the encoder path. These
figures describe a possible nesting, not a measured stack trace from a crash.
Alignment padding and other paths may add more space.

The original ELF places `knl_idle_stack` at 0x20001a98, size 0x100, immediately
above `knl_idletsk` at 0x20001a94 and `knl_idle_tcb` at 0x20001a28. A downward
overflow can overwrite the scheduler's idle pointer and context. That is a
credible explanation for the motion-sleep/watchdog signature; confirmation
requires a hardware retest with the corrected image.

Changes:

- Give idle its own minimum 2048-byte stack, aligned to 8 bytes.
- Report idle capacity at startup and its painted high-water mark every 2 s.
- Check a 32-byte painted margin and shut down motion if it is overwritten.

Validation: firmware compilation/linking passes; ELF symbol inspection confirms
an aligned 0x800-byte idle stack. The compiled nesting above totals 304 bytes.
`git diff --check` passes. No hardware retest or flash has been performed for
this correction. The powerbank-only crash has no newly captured serial report.

Prepared image: `build-detour-debug/buddy2_detour_idle_stack_fix.uf2`, build ID
`b2-line-steering-v1-20261103T165133Z-no-s4`. It retains line steering, subsystem 5,
the 25% output cap, feedback checks and the 1-second watchdog. On the next run,
expect `[STACK] idle_capacity=2048` and `[STACK] idle_used=... capacity=2048
guard_ok=1`. The historic capture and matching original ELF remain available.

## Live reproduction

The user reports laptop USB plus a connected battery. COM4 was initially held by
an existing logger; after it was released, a receive-only capture was saved to
`testing/system/com4_2026-11-04_codex_live.log`. No commands were transmitted and
no firmware was flashed or application source changed during this investigation.
The capture contains the previous crash followed by a fresh boot and new failure.
Only host reconnect markers are timestamped in this raw capture.

The new startup banner identifies build `20261103T162422Z-no-s4`, with subsystem 5
enabled, subsystem 4 disabled, production motion enabled and output capped at 25%.
This agrees with `build/firmware-build-id.txt`.

The new run:

1. Followed the line, then stopped at a reported 154 mm and 84 mm/s.
2. Completed the sweep and measured a 177 mm obstacle, 160 mm ahead.
3. Chose the right side and planned a -63 degree turn followed by a 221 mm
   diagonal, for a 197 mm sideways displacement.
4. Reported opposite wheel speeds during the turn (92, -83 mm/s), then positive
   speeds on both wheels during the following straight leg (roughly 33–66 mm/s).
   The failure therefore occurred after motion had started, not before the first
   detour command.
5. Disconnected and re-enumerated on COM4 around 00:44:16 Singapore time, reporting:

```text
LAST CRASH: watchdog timeout, code froze >1 s (motors held; power-cycle to run) motion_stage=4 obstacle_stage=7 last_feed_ms=26586
```

`motion_stage=4` is the marker immediately before Motion Control delays until its
next cycle. `obstacle_stage=7` covers the entire detour, including sleeping and
waiting for motion; it is not an instruction address or proof that Buddy 5 was
executing when the CPU stalled. The last feed was 26.586 seconds after boot.

No feedback-fault message preceded this new reset. This is distinct from the
00:30 run in `com4_2026-11-04_servo_relax.log`, which deliberately stopped with
`F_FEEDBACK / FB_NO_PROGRESS` near the end of its first turn and continued logging.

## Subsequent runs in the same live capture

Further user-run trials were also received without any transmitted commands:

- A short run reset while pinging, with `obstacle_stage=1` and
  `last_feed_ms=2846`. The watchdog problem is not confined to detour execution.
- The boot received at 00:45:10 completed its outward diagonal, then deliberately
  aborted move 5102 while turning back parallel: `hold 1, state 5, fault 8`.
  Fault 8 is `F_FINAL_GOAL`, meaning the settled encoder pose failed the motion
  goal check. The detour reported x=97 mm, y=-210 mm, heading=-1 degree; console
  output continued. The turn goal also requires centre travel within 10 mm, not
  just heading within 0.05 radians. The log does not contain that move's exact
  centre-travel error, so it cannot identify which final check failed.
- The boot received at 00:46:02 scanned and began another outward diagonal, then
  reset at approximately 00:46:26 with `motion_stage=4`, `obstacle_stage=7`, and
  `last_feed_ms=23426`.

These are two distinct failure classes: whole-system watchdog resets and
intentional motion-fault holds. Power conditions for each subsequent trial have
not been confirmed; none is treated as a verified battery-only comparison.

## Code and earlier evidence

- The sensor loops and detour waits have bounded timeouts and yield to the kernel.
  Motion runs at priority 2, above the priority-4 obstacle task. A normal long scan
  or detour should not prevent watchdog feeds.
- Servo PWM slice 6 and ultrasonic slice 0 do not overlap the motor slices 4/5.
  Current encoder assignments are GP2–GP5, separate from these sensors.
- The existing source already allocates 8192 bytes to the obstacle task and has
  an NMI freeze catcher. The matching built ELF contains that catcher. This
  reproduction produced only the hardware-watchdog report, not its saved PC/LR.
  That limits diagnosis; it does not by itself establish an electrical cause.
- The prior `S5_DIAG=9` self-test in
  `testing/system/com4_2026-11-03_battery_detour.log` did produce NMI freeze reports.
- In the same older log, `S5_DIAG=4` starts at 23:46:25 and reports a watchdog
  reset at 23:47:15 (`last_feed_ms=49535`). The source defines this mode as running
  the servo PWM slice without routing its pin, then sleeping, with ultrasonic
  scanning and detour execution bypassed. A system freeze is therefore also
  recorded outside the detour path. Physical conditions of that older test are
  not independently known.
- The checked-in working source and the current build differ: the ELF contains
  `s5_servo_relax` and calls to it; current servo/detour source no longer contains
  that function. The captured run tests the existing built image, not every
  change currently visible in the editor.

## Power isolation still needed

Cytron documents automatic selection between USB, a single-cell battery and VIN:
<https://my.cytron.io/p-robo-pico>. Its manufacturer-authored datasheet gives
Vmotor/Vservo as VUSB minus approximately 0.4 V when USB and LiPo are both
connected: <https://www.farnell.com/datasheets/4248509.pdf>.
Consequently, attaching a battery alongside laptop USB does not demonstrate that
the motor/servo load is isolated from the USB supply. The actual supply rails
have not been measured.

Requested next test: run the same image from the battery with laptop USB
disconnected, then reconnect USB without removing battery power to inspect any
retained crash report. First clear the existing crash latch by fully removing
power. A failure or success in that comparison narrows the cause; rail-voltage
and signal measurements may still be needed. The exact root cause is not yet
proved, and no timeout, feedback protection or output limit has been relaxed.
