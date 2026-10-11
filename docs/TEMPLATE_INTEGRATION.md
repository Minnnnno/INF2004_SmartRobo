# Template import and integration record

Imported https://github.com/sirfonzie/INF2004_ProjectTemplate.git at
c3b09544cb1fd6dac6ad4b8f523be0af59b44482 into the existing repository, without
changing its Git remote or importing the template's .git directory. The user's
CMakeLists.txt from the parent Assignment directory is the root build basis;
that original external file was not edited.

The kernel/config/device/lib/include/etc/tools/testing/videos/build_make trees
are present locally. Original demo applications and instructions are archived in
docs/template_original. Prior SDK-based application sources and build files are
archived in docs/previous_integration. Neither archive is compiled.

Key adaptations:
- Seven robot application tasks replace LED/producer/consumer demo tasks.
- Buddy 2 portable math/state machine and controller retain their host tests.
- Kernel owns reset, vectors, SysTick and linkage; no pico_stdlib runtime or old
  SDK startup adapter is linked.
- Motor HAL uses RP2040 registers through template BSP primitives, with an explicit
  shared GPIO dispatcher via tk_def_int. PWM slices 4/5 and reversal guards remain.
- Single-core is enforced until a cross-core motion port is validated.
- 1 ms kernel tick, 4 KiB initial stack; sample sensor drivers disabled.
- Early startup, kernel shutdown and hard-fault reporting force GP8..11 low.
- Hardware TIMER reset is released and its 1 MHz tick configured for measured dt.
- Optional calibration console uses template USB RX buffering and T-Monitor output.
  RX overflow cancels/disarms; normal firmware discards console input.
- Newlib libm gets a shared errno slot without importing stdio/heap syscalls.
  Application APIs return explicit error results; per-task libc errno is not provided.
- WiFi/networking remain optional and off by default. No credentials are imported.

The old docs describing SDK startup no longer apply. docs/BUILD.md is authoritative.
Neither compilation nor synthetic tests prove on-board scheduler timing or motor accuracy.
