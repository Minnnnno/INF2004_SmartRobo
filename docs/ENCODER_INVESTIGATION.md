# Buddy 2 encoder investigation and read-only firmware

Production counts/revolution are now **2542.4 left, 2543.4 right**, per the
user's measurements. Geometry and enabling flags still inhibit production
motion. For the new isolated motor-capable test image, use
[safe motor testing](SAFE_MOTOR_TEST.md). The encoder-only image remains read-only;
its latest ID/hash are recorded in `build-encoder-diag/encoder-build-manifest.json`.

## Current calibration update: original GP5/GP7 restored

The user restored **right A to GP5**, with right B on GP7 and left A/B on
GP4/GP6. Move the right encoder A signal off GP12 onto GP5. GP9 and GP12
retain their motor-driver and scanning-servo allocations. Board pin validation
again accepts only the original GP4..GP7 encoder allocation in all profiles.
The read-only image retains the improved interrupt handling and diagnostics.
Keep the battery and motor power wires disconnected throughout this test.

Flash `build-encoder-diag/buddy2_encoder_diag.uf2`, then send `encdiag`. Verify
right `A=GP5`, `B=GP7`, and firmware ID **b2-encdiag-v1-20261011T061554Z**.
The latest encoder-only size and SHA256 are recorded in
`build-encoder-diag/encoder-build-manifest.json`.
The real Pico W build passed; **1879** sanitized host checks passed, including
forward/reverse GP5/GP7 sequences and unchanged left-wheel state.
Right connector pin 2 now connects to GP5; pin 3 connects to GP7.

The sections below record the earlier GP5/GP7 investigation and build; its
older ID and hash are historical. Use the current ID above and the current
`build-encoder-diag/encoder-build-manifest.json` to identify the new UF2.

## Findings

The reported right-wheel/left-counter behaviour is not yet explained by a
confirmed cause. The source does not route GP5/GP7 to the left wheel. Do not
assume a wiring fault or successful hardware repair from the host tests alone.

The actual diagnostic build was inspected, including its compiler commands and
linked ELF. `encoder_pins` contains four 32-bit values **4, 6, 5, 7**, in that
order: left A/B, right A/B. `BOARD_ENC_*` are unconditional enum constants in
`include/board_config.h`; this build has no pin overrides. It includes
`drv_motion.c`, the real RP2040 register backend, and `encoder_capture.c`.
No simulated HAL is linked. The build uses Pico W/RP2040, single-core micro
T-Kernel, USB CDC, and disabled WiFi/networking.

The path is GPIO input initialization -> IO_BANK0 edge masks -> IRQ13 shared
dispatcher -> encoder handler -> per-wheel capture -> IRQ-protected snapshot ->
console task printing. Four GPIOs occupy INTR0: each pin has four event bits;
fall/rise are 4/8, INTR0 is offset 0xf0, PROC0_INTE0 is 0x100, PROC0_INTS0 is
0x120. These match the installed SDK's RP2040 register definitions. The USB
controller uses IRQ5 separately. Other built application subsystems are inactive
placeholders; no active subsystem replaces the encoder callback or reconfigures
GP4/5/6/7. GPIO ownership rejects overlapping registrations.

A is decoder bit 1, B bit 0; positive sequence is 00 -> 01 -> 11 -> 10 -> 00.
An impossible two-bit transition increments that wheel's invalid total and adds
no count, then resynchronizes previous state. Counts use defined unsigned modular
storage; `4294941920` represents signed **-25376**, not a huge positive travel.
No calibration value has been accepted from this observation.

Identified corrections:

- The old handler read A and B separately. A transition between those reads
  could produce a mixed state. The handler now takes one GPIO_IN snapshot for
  both wheels, acknowledges only owned pending edge flags, and uses independently
  indexed per-wheel storage. This removes a sampling race; it does not prove the
  cause of the reported cross-wheel changes or guarantee capture of fast edges.
- Previous states are initialized from one snapshot after clearing old latches.
  Interrupt enable no longer clears a newly arrived edge after this baseline.
- Snapshot critical sections include compiler memory barriers, with single-core
  IRQ masking. No ISR printing or RTOS waits are added.
- `status` prints signed count interpretations and separate `raw_counts`, with
  matching argument types and explicit sign/magnitude formatting, including
  INT32_MIN. This avoids signed-negation overflow in the legacy formatter.
- `encdiag` reports actual pins, raw levels, per-pin observed edge totals,
  previous A/B states, IRQ masks, function selectors, init/registration/NVIC
  state, signed counts, and invalid totals from a protected snapshot.
- `usbdiag` exposes existing USB initialization, link, interrupt, registration,
  and dropped-output diagnostics without calling TinyUSB from another task.
- The diagnostic profile compiles out motor activation in the HAL and console.
  The linked `motion_hal_write` is a return instruction. Startup holds GP8/9/10/11
  LOW with PWM disabled. ARM, movement, release, and reset commands are unavailable.

Configuration now uses `counts_per_rev={2542.4,2543.4}`, `diameter={0,0}`, `track=0`,
`hardware_confirmed=false`, `manual_enabled=false`. The expected configuration
fault does not prevent encoder initialization: the HAL is initialized before
the motion core validates geometry. Raw sampling and diagnostics stay available
in that fault. Speed/odometry in this configuration are not calibration evidence.

## Build and validation

Configure and build from the project directory:

```powershell
cmake -S . -B build-encoder-diag -G Ninja -DROBO_MANUAL_CONSOLE=ON -DROBO_ENCODER_DIAGNOSTICS_ONLY=ON -DSMP=OFF -DWIFI=none -DNET=none -DCONSOLE=usb_cdc -DPython3_EXECUTABLE=C:/Users/mervy/.pico-sdk/python/3.13.7/python.exe
cmake --build build-encoder-diag -j 8
```

GCC 15.2.1 and Pico SDK 2.3.0 successfully built the actual Pico W target.
The UF2 is copied to `build-encoder-diag/buddy2_encoder_diag.uf2`; the kernel's
original filename `build-encoder-diag/mtkernel_3.uf2` contains identical bytes.
Use this new build directory, not the older `build` or `build-template-manual`.
`encoder-build-id.txt` records the configure-time ID. `encdiag` must show that
same ID after flashing. A rebuild after reconfiguration may have a new ID/hash.

For the delivered build:

- ID: `b2-encdiag-v1-20261011T033236Z`
- UF2 size: 150528 bytes
- SHA256: `295E1C1D47E921E8B7C379205381F7A34E786C09E8172084D21DE7D947B212E7`

Host tests using the actual production capture helper passed **1857 checks**
with `-Wall -Wextra -Werror -Wconversion -Wshadow` and undefined-behaviour
sanitization. Added tests cover both directions on each GPIO pair, unchanged
other-wheel counters/state/edges, nonzero startup states, simultaneous wheels,
right-only invalid transitions, rollover, polarity inversion, ignored unrelated
GPIOs, and single-channel fallback. These are synthetic tests; hardware counting
accuracy and the currently flashed firmware have not been verified remotely.

Changed files: `CMakeLists.txt`, `include/encoder_capture.h`,
`include/motion_hal.h`, `app_program/subsystem2/encoder_capture.c`,
`app_program/subsystem2/drv_motion.c`, `app_program/subsystem2/template_hal.h`,
`app_program/subsystem2/manual_console.c`, `tests/test_motion.c`, and documentation.

## Hardware verification

1. Leave battery and motor power disconnected. Keep encoder connector pin 1
   connected to Pico GND and pin 4 to 3.3 V. Lift/support the robot so each wheel
   can be rotated independently. Do not push the robot along the ground.
2. Hold BOOTSEL while connecting USB, copy the new UF2 to RPI-RP2, then reconnect
   the serial monitor to the COM port Windows currently assigns. Only one serial
   application should have the port open. Send `encdiag` and verify the build ID,
   `motors_inhibited=1`, `enc_init=1`, `irq_registered=1`, `nvic13_enabled=1`.
   Pins must be left GP4/6, right GP5/7; each `irq_mask=12` and `func=5` (SIO).
3. Send `status` and record both counts and both invalid totals. Also save
   `encdiag` as the baseline for per-pin edges.
4. Mark the left wheel. Rotate only it slowly through exactly one revolution.
   Send `status`, then `encdiag`. Only the first count and GP4/GP6 edge totals
   should change; both invalid totals should stay at their baseline values.
5. Rotate only the right wheel slowly through exactly one revolution. Send
   `status`, then `encdiag`. Only the second count and GP5/GP7 edge totals should
   change; both invalid totals should again stay unchanged.
6. Repeat independently in both directions. Opposite rotation should reverse
   the count change. A negative count is acceptable at this stage. Once both
   wheels count independently and repeatably with unchanged invalid totals,
   repeat measured multi-revolution trials before setting any calibration values.

Edge totals count observed hardware rise/fall latches, not guaranteed physical
edge totals: repeated edges can coalesce during IRQ latency. Do not compare an
individual edge total directly to counts per revolution. Pause at successive
wheel positions and send `encdiag` to observe raw A/B levels if necessary.

## Using diagnostic evidence to check wiring

| Evidence while rotating only the right wheel | Check with USB disconnected |
| --- | --- |
| GP4/GP6 edge totals change, GP5/GP7 do not | Trace right connector pin 2 to GP5 and pin 3 to GP7; check for crossed connections into the left pair. Left connector pin 2 must reach GP4 and pin 3 GP6. |
| GP5 changes but GP7 stays fixed | Trace right connector pin 3/B to GP7 and check connector continuity. |
| GP7 changes but GP5 stays fixed | Trace right connector pin 2/A to GP5 and check connector continuity. |
| Neither right pin changes; IRQ/init settings correct | Verify right connector pin 1/GND and pin 4/3.3 V, then pin 2/3 signal continuity and electrical signal levels. |
| Right pins and left pins all change | Check accidental bridging, crossed signal wires, and common-ground continuity. |
| Correct right edges, left count still changes without left edges | Save before/after `status` and `encdiag`, including ID; this contradicts the tested routing and requires further firmware investigation. |
| Both expected channels change but invalid rises | Rotate more slowly; check continuity and unstable signal levels. Missed transitions or electrical noise remain possibilities, not established causes. |

For left-wheel anomalies apply the same checks to connector pin 2/GP4 and
pin 3/GP6. Verify connector orientation against actual pin numbering; wire colors
and comments do not establish electrical function. Motor connector pins 5/6
remain disconnected throughout these checks.

## USB investigation limits

USB initializes once in its service task, which alone calls TinyUSB; ISR5
dispatches controller events and wakes that task. The service and motion tasks
yield between cycles, and SMP is disabled. Output buffering drops/counts full
ring output rather than waiting indefinitely for a host. Source inspection has
not established a firmware cause for the intermittent COM4 connection problem.

When connected, save `usbdiag`: expected init=1, link=3 if the monitor asserts
DTR (2 can be valid otherwise), define_irq_result=0, nvic_enabled=1, with a
changing irq_count. tx_dropped reports missing output, not a proven disconnect.
These readings are only available while serial works and cannot explain a
past failure alone. Port ownership, changed COM assignment after flashing,
cable/contact problems, and wiring/power disturbances require host/hardware
evidence. No speculative USB fix or motor test was introduced.
