# Verification of the template integration

Verified using the template source at c3b09544cb1fd6dac6ad4b8f523be0af59b44482,
Pico SDK 2.3.0, Arm GCC 15.2.Rel1, CMake/Ninja and Zig's native C compiler.

- Normal kernel-owned Pico W build: build-template/mtkernel_3.uf2.
- Deliberate calibration-console build: build-template-manual/mtkernel_3.uf2.
- Relocated portable host suite: **1,769 assertions passed**, compiled with strict
  warnings and undefined-behaviour instrumentation.
- The optional CMake host-compiler probe with Zig was stopped after it stalled;
  the host suite was built and run directly with Zig cc. No CTest pass is claimed.
- Host cases cover simulated forward/backward distance, both turn directions,
  U-turn/zero movement, encoder rollover/decoding, odometry, PID saturation and
  anti-windup, steering, cancellation, stale generations/targets, feedback grace
  and loss, low-speed/coarse-encoder timing, final target errors, stop deadlines,
  reset guards, shutdown and full buffers.

These are synthetic tests and cross-compilation results. No robot was flashed or
connected. Boot behaviour, actual PWM waveforms, USB interaction, encoder edge
capture, ISR load, task timing, stop distances, calibrated geometry and PID gains
still require controlled physical testing. Hardware motion remains inhibited by
the default configuration. SMP and network-enabled profiles are not validated.

The previous SDK-runtime build artifacts are historical and should not be flashed
as the integrated template build. Use the new output paths above.
