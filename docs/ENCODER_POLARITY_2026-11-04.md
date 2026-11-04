# Forward-motion encoder polarity correction - 4 November 2026

## Evidence

The supplied capture is preserved in
`testing/system/com4_2026-11-04_encoder_polarity.log`. It reports firmware
`b2-line-steering-v1-20261104T041249Z-no-s4` and includes:

```text
speed_mm_s=-157,-74 output_permille=250,250
speed_mm_s=-108,-83 output_permille=250,250
```

All 12 complete samples with both outputs at +250 permille report both speeds
negative. The user explicitly confirmed that the chassis was physically moving
forward. The configured encoder feedback polarity is therefore reversed on both
wheels for the current wiring and motor installation. This is separate from a
possible difference in the motors' physical speeds.

The PID error is target minus measured speed. A forward target of +100 mm/s and
a measured speed of -150 mm/s produces +250 mm/s error. As forward speed
increases, that incorrect feedback increases the requested effort instead of
reducing it. Both outputs can saturate at the 25% cap, preventing normal speed
regulation. Incorrect signed speed and heading also affect obstacle braking
prediction and line-search turns.

The eventual fault snapshot is:

```text
[FAULT] FEEDBACK(4) feedback=NO_ENCODER_PROGRESS(4) id=3001 held=1; firmware running
[FAULT] LEFT stalled=0 target_mm_s=0 speed_mm_s=0 request_permille=0
[FAULT] RIGHT stalled=1 target_mm_s=100 speed_mm_s=0 request_permille=250
[FAULT] RIGHT remaining_um=0 no_progress_ms=408 limit_ms=401 count=4294919013 invalid=0
```

Command 3001 is line tracking/search, not an obstacle detour command. Here the
left wheel was intentionally commanded to zero during a pivot. The right wheel
stopped producing encoder progress while commanded forward. The log cannot
distinguish physical wheel stoppage from missing encoder edges at that point.
Reversed polarity alone does not prove why those edges stopped.

All reported stack guards and saved stack pointers are valid. The firmware
continues feeding its watchdog and printing. Counts near 2^32 are expected when
unsigned encoder counters decrement; they do not establish memory corruption.

## Code and PWM review

- `board_motion_config().encoder_sign` changed from `{+1, -1}` to `{-1, +1}`.
  Motor direction settings remain `{+1, -1}`, matching confirmed physical
  forward motion. GPIO assignments, PID gains and output caps were not retuned.
- The regular `line_pid` message now includes `target_mm_s=left,right`, so the
  next capture can compare requested and measured speed and identify deliberate
  pivots and steering corrections.
- Encoders are digital A/B inputs: left GP2/GP3, right GP4/GP5. They are not PWM
  outputs. Capture takes a coherent input snapshot and applies the configured
  direction sign once before odometry calculates speed.
- Left motor PWM uses GP8/GP9, slice 4 A/B; right uses GP10/GP11, slice 5 A/B.
  Both use the same frequency configuration, counter wrap and duty calculation.
- Servo GP7 uses slice 3 B and ultrasonic ECHO GP17 uses slice 0 B. They do not
  overwrite either motor's PWM slice.

The log shows larger left speed magnitudes in many samples, but includes pivots
and steering corrections. No left-only PWM or encoder scaling defect was found
in these paths. Speed imbalance needs a straight-motion comparison after the
feedback sign correction, with both requested speeds equal.

## Validation and firmware

A regression exercises inferred forward/reverse A/B phase sequences through the
real encoder capture and odometry code using the production configuration. It
fails with the previous signs and passes with the correction. The phase fixture
is inferred from the reported signs; it is not a captured electrical waveform.

All 10 host CTest suites passed. Strict ARM GCC syntax/warning checks passed for
the modified configuration and motion test source. The firmware builds.
No physical motion test or flashing was performed by the agent.

Updated image: `build/mtkernel_3.uf2`.
Pinned image/symbols: `build/buddy2_encoder_polarity_fix.uf2` and `.elf`.
The previous firmware/ELF are under
`build/pre-encoder-polarity-20261104T041249Z/`.

Build ID: `b2-line-steering-v1-20261104T042443Z-no-s4`.

UF2 SHA256: `8356F7E8BCEDD6E53AD10434FBFAC02C2AC25C34D77526A784825843D7C2D0AF`.

Flash with motor power disconnected. On the next run, forward wheel travel must
report positive speeds. Compare `target_mm_s`, `speed_mm_s` and
`output_permille` during a straight segment before considering motor-specific
tuning. The right-wheel no-progress symptom still needs a hardware retest.
