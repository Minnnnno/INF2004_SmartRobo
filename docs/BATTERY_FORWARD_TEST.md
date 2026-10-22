# Buddy 2 button-controlled battery forward test

Flash `build-battery-forward/buddy2_button_forward_2s.uf2` with motor power
disconnected. This replaces the automatic-start experiment: the robot stays
stopped after flashing and after battery or USB power-up. It requires neither
a serial command nor an attached USB host.

On the Cytron Robo Pico carrier, use the existing onboard GP20 and GP21
pushbuttons; their PCB connections need no external wires. Cytron documents
both as active LOW with pull-ups in its
[button guide](https://sg.cytron.io/tutorial/get-started-robo-pico-circuitpython-press-the-buttons).
Only for a bare Pico or a carrier without these onboard buttons, wire normally
open momentary switches from GP20 to GND for START/STOP and GP21 to GND for
an additional STOP input.
Both GPIOs are inputs with internal pull-ups; a press pulls the input LOW.

- Left motor: 24.9%; right motor: 25%, using the existing corrected polarity.
  Left increased from 24.7% for the next requested mat trial; this
  remains a provisional calibration value pending repeated ground trials.
- Press GP20 to start, release it, then press GP20 again to stop. Start requires
  a debounced press; stopping uses a GPIO falling-edge interrupt after a stable
  release. GP21 is an additional falling-edge interrupt stop and wins when both
  switches are pressed. The nominal 20 ms task poll remains a fallback.
- Duration: at most two seconds from motor enable, then both motor outputs stop.
- Alarm3 is armed before PWM is enabled; its interrupt supplies the cutoff.
  Version `b2-button-irq-2s-v4` also checks an absolute 64-bit deadline from task
  context on every button poll, stopping even if the alarm interrupt is missed.
  This fallback needs task scheduling; neither path is a physical stop guarantee
  until checked on the board.
- Button interrupts register only GP20/GP21 through the shared GPIO dispatcher;
  encoder IRQ ownership is preserved. The stop handler disables outputs and
  captures the completion snapshot without printing or waiting for a task.
  Motor-test initialization fails with outputs stopped if IRQ registration fails.
- Start and release require 40 ms of stable observations; neither stop input
  requires press debounce. Release both switches
  after power-up, stop or timeout, then press GP20 again for the next run. A
  held start at boot cannot start movement. Holding start cannot extend a run
  or restart it after stop/timeout; releasing stop while start stays held also
  cannot restart it. GP21 has no debounce delay on its stop action.
- IR automatic movement is disabled for this profile, both in the line task and
  at the actuator entry point, so black/white changes cannot restart it.
- Serial `stop` can cancel the run early; `shutdown` disables all further tests.
- Explicit serial motor commands remain one-second tests. `status`, `encdiag`
  and signed completion reports remain available when connected to a host.
- Production remains inhibited: `hardware_confirmed=false`,
  `manual_enabled=false`. Geometry and encoder calibration are unchanged.

After flashing, disconnect USB, place the robot on a clear floor, and power it
using its existing battery arrangement that powers both Pico and motor driver.
With both buttons released, it stays stopped. Press GP20 to run forward,
release GP20, then press it again to stop early, or wait for the two-second
cutoff. GP21 is still an additional stop button. Release both
switches, reposition the stopped robot, and press GP20 for another trial.
Allow at least 0.3 seconds after stopping for the existing actuator cooldown.
Motor completion and signed count deltas can be read over USB if desired.

## Check a STOP button that does not respond

Disconnect motor power and leave the Pico on USB while checking switch inputs.
Run `buttons` once with the switches released and again while holding STOP.
It reports the firmware ID, `GP20`, `GP21`, `pressed_level=0`, readiness, armed/
active state, button poll count and number of active runs cancelled by either button.
It also reports `button_irq_registered` and `button_irq_events`: registration
must be 1 for this profile, and edge events should increase when pressing or
releasing the buttons, even with motor power disconnected.
`cutoff_irq_count` counts alarm-handler entries, `polled_cutoffs` counts deadline
stops performed by the fallback, and `remaining_ms` shows time left in an active
test. To diagnose without powering motors: release both buttons, press/release
GP20, wait three seconds, then type `buttons`. A completion report should show
`reason=timeout`, `active=0`, and `remaining_ms=0`. For early stop, start again,
release GP20 for half a second, then press GP20 again: expect `reason=stop` and an
increase in `stop_actions`. Preserve the whole log if the physical motor keeps
turning despite a reported stop; a software flag alone does not prove the driver
received a stopped output.
The same snapshot is included in `status`; neither command starts movement.
GP21 must change from 1 released to 0 pressed. On Robo Pico, press the onboard
button labelled GP21; no extra switch wiring is needed. For an external switch,
verify that it connects GP21 to GND, not 3.3 V; GPIO21 is physical header pin 27,
not physical pin 21.
If it stays 1, investigate the switch terminals, ground, GPIO connection or
pressed polarity before powered tests. If it becomes 0 but stopping fails,
save the full snapshot; poll count should advance between readings.

This profile builds `line_task` as a sleeping loop: it does not read the IR
inputs or issue steering requests. Changes in physical path on a soft mat do
not establish IR software control. Confirm the flashed profile before drawing
that conclusion, and compare flat/soft surfaces under otherwise equal conditions.

This is an open-loop trim experiment, not a verified straight-line calibration.
Record drift direction and physical path. Hardware stopping latency, wheel slip
and real-world tracking still require robot testing.

## Build

```powershell
cmake -S . -B build-battery-forward -G Ninja -DROBO_BATTERY_FORWARD_TEST=ON -DROBO_SAFE_MOTOR_TEST=ON -DROBO_MANUAL_CONSOLE=ON -DROBO_LINE_STEERING=OFF -DROBO_ANGLE_TEST=OFF -DROBO_ENCODER_DIAGNOSTICS_ONLY=OFF -DSMP=OFF -DWIFI=none -DNET=none -DCONSOLE=usb_cdc -DPython3_EXECUTABLE=C:/Users/mervy/.pico-sdk/python/3.13.7/python.exe
cmake --build build-battery-forward -j 8
```

The legacy option name `ROBO_BATTERY_FORWARD_TEST` now selects this button
profile, with no automatic startup. Default is OFF; the existing isolated
motor-test profile retains its one-second serial commands and startup behaviour.
Build ID: `build-battery-forward/firmware-build-id.txt`.
File metadata: `build-battery-forward/battery-build-manifest.json`.

Validation includes actual actuator
two-second deadline/rollover and GP21 early-stop cases, start debounce,
held-at-boot rejection, simultaneous-button stop priority, held-start rejection
after stop/timeout, repeated fresh-press trials, IR rejection, and preservation
of explicit one-second manual commands. A dedicated GP20 toggle-stop case
verifies single-sample quick-tap stop, release-bounce rejection and no restart while held.
The console integration test passed
without incoming serial bytes and verifies no automatic start and continued
button polling, including after initialization failure.
Pico W UF2 built successfully. No robot was flashed or physically tested.
All 12 registered host suites passed after adding GP20/GP21 interrupt-stop
tests that perform no button task polling. Failed stop-IRQ setup is also tested
to keep motor outputs disabled. These software results do not establish the
electrical state of the buttons on the physical robot.
