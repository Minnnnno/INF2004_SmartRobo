# Memory review and detour fault diagnostics - 4 November 2026

Follow-up: [detour completion and interrupt review](DETOUR_IRQ_REVIEW_2026-11-04.md)
reproduces and fixes an overly strict movement-completion check, removes
exception-context printing, and updates the usual `build/mtkernel_3.uf2` output.
The image and findings below describe the preceding memory review.

## What the supplied log establishes

The user's `Downloads/logs.txt` is preserved as
`testing/system/com4_2026-11-04_feedback_fault.log`. It contains:

```text
[DETOUR] move 5101 interrupted: hold 1, state 5, fault 4
[DETOUR] gave up at x 97 y -210, heading -66 deg
[OBSTACLE] ===== DETOUR ABANDONED: stopped and holding =====
line_fb cause=4 reliable=1 counts=10712,11075 invalid=0,0 previous_invalid=0,0
[STACK] idle_used=264 capacity=2048 guard_ok=1
```

State 5 is `FAULT`, fault 4 is `F_FEEDBACK`, and feedback cause 4 is
`FB_NO_PROGRESS`: a wheel was commanded to move but stopped producing encoder
progress for the allowed interval. Repeated status and stack lines after the
stop show that the firmware continued running. This capture does not show a
CPU crash or watchdog reset, so the reboot-only `LAST CRASH` message does not
apply. Earlier watchdog failures are documented separately in
[the freeze investigation](DETOUR_FREEZE_2026-11-04.md).

The final pre-fault output includes a small left reverse request of -58
permille (-5.8%). The odometry is near the end of the planned 230 mm diagonal.
An end-of-move correction that cannot turn the wheel is one possibility; this
log cannot establish which wheel timed out, its exact remaining distance, or
whether the cause was mechanical, electrical, encoder-related, or control
tuning. No feedback timeout, movement tolerance, or motor output cap was relaxed.

The idle-stack guard is intact in this capture. Its 264-byte high-water mark
would exceed the original 256-byte allocation, but fits the corrected 2048-byte
stack. The log has no measurements for the other task stacks.

## Memory defects corrected

- `report_scan()` formatted the combined MQTT report into a 96-byte array using
  unbounded `tm_sprintf()`. Ordinary readings already exceed that buffer; the
  length check happened after the overwrite. It now uses bounded formatting
  and two events, preserving the existing field names. Oversized events are
  skipped with a console message. This path requires communications online;
  it does not explain the supplied offline capture.
- Added `tm_snprintf()` to the existing T-Monitor formatter, with truncation,
  NUL termination, full-length return, and argument checks. Formatting signed
  `INT32_MIN` now uses unsigned magnitude instead of overflowing signed negation.

The scan sample arrays, coarse/fine sweep bounds, detour status copies, and
relevant formatting buffers were reviewed. No further concrete out-of-bounds
write was identified in those paths. This is not proof that every possible
hardware or kernel execution is free of memory corruption.

## New recurring console output

Every two seconds, the diagnostics task prints:

- `[FAULT] FEEDBACK(4) feedback=NO_ENCODER_PROGRESS(4)` and the command ID.
  A snapshot taken before outputs are cleared supplies each wheel's target,
  measured speed, requested output, signed remaining distance, time without
  encoder progress, and feedback timeout. `stalled=1` identifies the first
  wheel that tripped the check; it does not prove the other wheel was moving.
- `[STACK]` for every created task, including kernel tasks, plus the separate
  idle stack. It reports high-water use, allocated capacity, bottom guard, and
  whether the saved stack pointer lies in its allocation. The obstacle task
  receives 8320 bytes: the requested 8192 plus the kernel's 128-byte allowance.
- `[MEMORY]` if stack metadata or the idle/scheduler guards are damaged. A
  detected failure requests shutdown and immediately disables the motor outputs.
- `[DEBUG]` with the firmware ID, watchdog-feed age, NMI check count, USB dropped
  byte count, and scheduler guard state. Drops can accumulate with no USB host
  attached; increasing drops during a capture indicate incomplete console data.
- `[INIT ERROR]` on task, mutex, or communications startup failures.

Distances in `remaining_um` are micrometres; 1000 means 1 mm. The value is signed
target minus signed encoder travel; interpret it with the commanded direction.
Ignore this field when `finite_goal=0`.
`request_permille=58` means a requested 5.8% output, not a measured motor voltage.
`-2147483648` is the invalid-value sentinel for numeric fault snapshots.

The first fault snapshot remains available until motion is reset. Printing
runs below the motion and USB task priorities and occurs outside the motion
task. Existing hard-fault/watchdog reports and the idle-stack correction remain.

## Validation and firmware

- All 10 CTest suites pass normally and with MSVC AddressSanitizer, including
  real formatter truncation/canary tests, the MQTT buffer regression, and
  simulated left/right encoder no-progress faults. Host tests do not execute
  the ARM kernel or physical devices.
- Motion and obstacle runtime test sources pass strict ARM GCC syntax/warning
  checks. The Pico W firmware builds successfully.
- ARM stack-usage output was reviewed after adding the snapshots. The detour
  call chain exceeds 4 KiB, so the 8 KiB obstacle stack remains necessary;
  runtime stack diagnostics must verify its actual high-water use on hardware.
- No firmware was flashed and no hardware run was performed in this review.

Image: `build-memory-debug/buddy2_memory_debug.uf2`.
Matching symbols: `build-memory-debug/buddy2_memory_debug.elf`.

Build ID: `b2-line-steering-v1-20261104T025917Z-no-s4`.

UF2 SHA256: `F9A91F224770C6AC5467E7DB0F2619EFB1E7C1075BA1AF06A9D0D9FC08CCD99D`.

Configuration: Pico W, single core, USB CDC, Wi-Fi/network disabled, line
steering and subsystem 5 enabled, subsystem 4 disabled, motor cap 25%.

Flash with motor power disconnected. For the next avoidance test, capture the
new `[FAULT]`, `[STACK]`, and `[DEBUG]` lines through several seconds after the
stop. The repeated firmware ID distinguishes this image from the earlier one.
Keep the matching ELF for decoding any future `LAST CRASH` PC/LR addresses.
