# Passive encoder calibration with Pico SDK USB

This standalone Pico W image isolates the native SDK USB console from the
micro T-Kernel integration. It reuses the existing tested quadrature/capture
code. No motor driver, PWM actuator, button control, IR, WiFi or RTOS task runs.
GP8..GP11 are held LOW by the application. Keep motor power disconnected during
flashing and passive tests; software does not control the pre-application boot.

Build separately from the root RTOS application:

```powershell
cmake -S calibration/encoder_usb -B build-passive-encoder -G Ninja -DPICO_BOARD=pico_w -DPICO_SDK_PATH=C:/Users/mervy/.pico-sdk/sdk/2.3.0 -DPICO_TOOLCHAIN_PATH=C:/Users/mervy/.pico-sdk/toolchain/15_2_Rel1 -Dpicotool_DIR=C:/Users/mervy/.pico-sdk/picotool/2.3.0/picotool -DPython3_EXECUTABLE=C:/Users/mervy/.pico-sdk/python/3.13.7/python.exe
cmake --build build-passive-encoder -j 8
```

UF2: `build-passive-encoder/buddy2_encoder_calibration.uf2`.
USB may enumerate under a different COM number from the RTOS image because its
USB identity differs. Select the port Windows assigns. Use one terminal, 115200,
LF or CRLF. DTR is not required for output. Only `status` and `encdiag` are
supported; every reply includes build ID, reply sequence and uptime. Repeated
commands must keep replying without resetting uptime or counts. No reset-count
command exists, so save snapshots and subtract or reboot between trials.

Left encoder A/B stays GP4/GP6; right stays GP5/GP7. Preserve encoder 3.3 V and
common GND while disconnecting motor power. Raw direction signs remain {1,1};
negative forward left counts observed previously are not silently inverted.
Distances/speeds/angle commands and production configuration remain unchanged.

First send ten `encdiag` commands while stationary. If stable, turn one wheel
slowly for an exact revolution (5-10 seconds) between readings, three times per
wheel. The other wheel's count should stay fixed and invalid totals should not
increase. Once reliable, perform the passive 500 mm rolling calibration.
An initial counter of zero is not evidence that a disconnected encoder works.

GPIO interrupt decoding can still miss fast transitions; this image is not a
PIO decoder and does not establish performance at running motor speeds. Native
USB operation, reconnects and motor-pin levels still need physical verification.
If this image also fails to open, retain the exact host error and distinguish
port ownership, USB disconnect and a firmware reset. Do not assume a cable or
firmware cause solely from a failed terminal open.
