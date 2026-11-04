# Buddy 5 passing-to-return transition review - 4 November 2026

The user reports a new stop after passing the obstacle, before returning to the
line. No console capture for this new stop has been provided during this review.
Earlier logs describe other stages and cannot establish the cause of this event.
The defects below are reproducible in tests; they are not proof of the cause
of the newly reported physical stop.

## Defects corrected

1. **Temporary command-bus contention abandoned the detour.** The public APIs
   use a nonblocking mutex attempt and return `R_BUSY` if the lock is unavailable;
   no command has been queued. Buddy 5 treated this as a permanent refusal,
   including for the final rear-clearance move and first return turn. It now
   retries only `R_BUSY`, every 20 ms, for less than 500 ms. A hold, fault, state
   change or generation change cancels retries. Successful submissions are not
   repeated; other rejection codes still abort.
2. **Lost final position could silently retain an old pose.** A failed status
   read after a straight move skipped updating `s5_x/s5_y`, but could return
   success. Clearance and return calculations then used an old position. It now
   stops and prints `lost final position; holding`. Failure to confirm the
   line-acquisition stop is also propagated instead of ignored.
3. **Completion could hide simultaneous line detection.** `wait_move()` returned
   success before examining fresh IR data when a completion event was present.
   The return routine then reported that the line was not found. Fresh black
   detection within the expected-position window now takes priority over
   successful completion. Fault, cancellation and rejection still take priority
   over line detection.

The old implementation failed 13 assertions in the initial transition tests.
The fixed implementation passes those tests and additional interruption and
left/right return fixtures.

## Stage diagnostics

New `[DETOUR] stage=... x=... y=...` messages execute in the obstacle task.
Matching watchdog scratch markers are saved before each stage. Existing marker
values 1 through 7 retain their meanings.

| Watchdog obstacle stage | Console stage | Operation |
| --- | --- | --- |
| 8 | `out` | Leave the line and become parallel to it |
| 9 | `pass` | Look alongside and advance past the obstacle |
| 10 | `rear-clearance` | Advance until the rear corners can turn safely |
| 11 | `return-turn` | Centre the servo and turn towards the line |
| 12 | `return-path-check` | Check the return path with ultrasonic |
| 13 | `return-to-line` | Drive while watching for black |
| 14 | `line-alignment` | Move the axle onto the line and restore heading |

The reported boundary is principally stages 10 through 12. Existing messages
distinguish refused moves, motion faults, sensor failures, blocked paths and
missed lines. Path geometry, motor limits and sensor-failure policy were not
changed by this review.

## Validation

- All 10 normal CTest suites pass, including **66/66 obstacle runtime checks**.
- Tests execute the actual Buddy 5 source with simulated command/status/sensor
  APIs: transient contention at rear clearance/return turn, bounded persistent
  contention, interruption by stop/reset/fault/another move, lost final status,
  simultaneous completion and line detection, rejection despite black, and
  successful left/right return and alignment.
- Strict ARM GCC checks pass for the runtime test translation unit.
- The production Pico W firmware builds. Compiler stack-usage files are in
  `build/stack-check/`; the added retry helper has a 560-byte frame. The requested
  8192-byte obstacle stack and runtime guards remain in place.
- The additional AddressSanitizer executable was blocked by the organization's
  **Windows Device Guard policy**. That sanitizer run did not execute for this
  revision; the ordinary host tests did execute and pass.
- No firmware was flashed or tested on the robot by the agent.

## Image

Updated normal output: `build/mtkernel_3.uf2`.
Pinned image/symbols: `build/buddy5_pass_return_fix.uf2` and `.elf`.
Previous normal image/symbols:
`build/pre-pass-return-fix-20261104T042813Z/`.

Build ID: `b2-line-steering-v1-20261104T045154Z-no-s4`.

UF2 SHA256: `9739216082D352DBD8AE10E946A4CD53A5D76A94060BA0A5B5680CFD2C843EA5`.

Flash with motor power disconnected. Capture the firmware ID and final stage/
move messages, plus `[FAULT]` or `LAST CRASH`, on the next run. A new capture is
still needed to identify the actual cause of the reported stop.
