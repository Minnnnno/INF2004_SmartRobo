# Isolated powered distance calibration

UF2: `build-distance-sdk/buddy2_distance_calibration.uf2`.
Separate Pico SDK firmware, using native USB as the working passive image did.
No production controller, RTOS, WiFi or IR is linked. No automatic start.
Production hardware confirmation is not consulted or enabled by this image.

Build:
```powershell
cmake -S calibration/distance_usb -B build-distance-sdk -G Ninja -DPICO_BOARD=pico_w -DPICO_SDK_PATH=C:/Users/mervy/.pico-sdk/sdk/2.3.0 -DPICO_TOOLCHAIN_PATH=C:/Users/mervy/.pico-sdk/toolchain/15_2_Rel1 -Dpicotool_DIR=C:/Users/mervy/.pico-sdk/picotool/2.3.0/picotool -DPython3_EXECUTABLE=C:/Users/mervy/.pico-sdk/python/3.13.7/python.exe
cmake --build build-distance-sdk -j8
```

Host controller checks (goal, deadline, stall, feedback, wrong direction, counter
wrap, correction direction, shared slowing/stopping, explicit stop, and 100/500mm
simulations with unequal motor response):
```powershell
& '.tools/zig/ziglang/zig.exe' cc -std=c11 -O2 -Wall -Wextra -Werror -Wconversion -Wshadow -fsanitize=undefined -Iinclude calibration/distance_usb/test_control.c app_program/subsystem2/math.c -o build-host/distance_sdk_checks.exe
& './build-host/distance_sdk_checks.exe'
```
These host checks do not exercise physical buttons, PWM hardware or native USB.

Pins: left encoder GP4/6; right GP5/7. Encoder signs -1/+1 make forward positive.
Left motor GP8/9, right GP10/11, electrical signs +1/-1. Maximum PWM left24.9%,
right25%. Version2 uses a shared slowdown toward22% in the final80mm of axle
centre travel, preserving the24.9:25 trim ratio. Encoder travel difference
reduces only the leading wheel's output, by0.75percentage points per millimetre
in version5 (previously0.5),
capped at4percentage points. Output never exceeds its original limit and never
reverses. Both wheels stop together at the axle-centre distance target, using
LOW/LOW braking; a wheel no longer stops independently and causes a final pivot.
Distance conversion starts with provisional effective diameter67.5/67.9mm and
CPR2542.4/2543.4. Version4 applies a common scale of296/304.833333 (about0.971022)
from the last3mat trials: physical295/297/296mm versus settled encoder
304.7/304.8/305.0mm. The first290mm trial had no separately reported settled
snapshot and is not used in this paired correction. Wheel ratio is preserved.
This standalone correction also scales reported speed; it has not yet been
copied into production geometry. Track remains115mm; turn accuracy is uncalibrated.

Version4 switches both outputs off4.5mm before the requested centre distance,
allowing for the approximately4.7mm of corrected post-stop travel observed.
The requested `goal_mm` remains300 for `move_300`; the brake threshold is295.5mm.
Final settled travel must still be measured physically. The fixed allowance is
provisional for the current mat and output, not a measured braking model for
all speeds and surfaces. PWM limits, synchronization gain and buttons are unchanged.

Version5 increases only synchronization gain, after version4 physically measured
298/300/301mm and showed a repeatable2-4mm left-wheel lead. Distance scale,
4.5mm braking allowance,4percentage-point correction cap, and PWM limits are
unchanged. Verify that the slightly stronger correction reduces right drift
without creating oscillation on the mat; hardware behaviour is not simulated proof.

Commands: `status`, `encdiag`, `move_100`, `move_300`, `move_500`,
`turn_left DEGREES`, `turn_right DEGREES`, `stop`. LF/CRLF.
Version3: GP20 press starts300mm; release for50ms then press again stops. GP21 stops and
blocks starting while held. Buttons must connect their GPIO to GND when pressed.
Both released at boot; cooldown500ms after stopping. No queued movement.

Check with wheels raised before floor testing:
1. Flash with motor power disconnected. Verify `status` ID starts `b2-motion-sdk-v7`.
2. Reconnect motor power with wheels raised. Confirm startup stays stopped.
3. Press GP20, release fully, then press again. Confirm immediate stop.
4. Start again and press GP21. Confirm immediate stop.
5. A blocked/non-counting wheel must stop the test within700ms; never hold a
   powered wheel by hand. The run must end by5s even if the goal is not reached.
6. Only after stops work, put the bot on the calibration surface. Send `move_100`.
   Measure hub-centre displacement and check encoder forward counts positive.
7. On the430-440mm clear path use `move_300`; measure final hub displacement
   including coasting. Keep all wheels on the mat throughout the run. Wait1second
   after the completion message before sending `status` for the settled reading.
   `move_500` remains available only for longer surfaces.
   Repeat3times and report physical distance, reported wheel distances and state.

Run ends on any added invalid transition, wrong count direction beyond4counts,
no progress for700ms, either stop button, or5s deadline. A1ms SDK timer checks
these even during USB printing. If main stops servicing for300ms, timer stops
outputs; hardware watchdog resets after500ms if scheduling/interrupts hang.
These mechanisms need physical verification; no claim that the old RTOS stop
failure has been diagnosed. Stops do not depend on a USB connection.

`speed_mm_s` is the signed100ms encoder-derived wheel speed, so it returns to0
at rest. `distance_mm` includes post-stop motion since the latest command
baseline. Neither readout proves physical accuracy; measure actual distance/time.
`GOAL` means the encoder braking threshold was reached; it does not mean the
robot has already settled at the final requested physical distance.
Version2 also prints current and maximum wheel-travel difference in millimetres,
and current commanded output percentages. Differences inferred from encoders
cannot detect tyres sliding, mat deformation, or incorrect effective geometry.
There is no claim that the reported right curve began only during the previous
80mm slowdown: previous firmware also lacked travel synchronization in cruise.

Version6 adds automatic speed measurement only; version5 motor behaviour,
calibration and stops are unchanged. During `move_300`, the1ms timer saves wheel
distances and elapsed timestamps at the first encoder-centre crossing of70mm
and170mm. The sample uses actual captured distance differences and elapsed time,
not a nominal100mm assumption. Endpoints can exceed the marks by one timer tick.
No printing occurs in the ISR. The completion report and later `status` retain:
`speed_window complete=1 from_mm=... to_mm=... elapsed_s=...`, then
`window_speed_mm_s=left,right centre_speed_mm_s=...`.
These are average speeds across the saved interval, distinct from the live
100ms `speed_mm_s` estimate, which reaches zero after stopping. A `move_100` or
early stop before170mm reports an incomplete interval. A new accepted movement
clears the previous saved interval. Rejected starts do not clear it.

Film a `move_300` and measure physical axle-centre crossing times at the70mm and
170mm ground marks. Compare100/(video end-start) with the saved
`centre_speed_mm_s`. Encoder crossings use calibrated travel rather than an
external position sensor, so agreement must be checked against video; the
automatic sample is not independent proof of physical speed accuracy.

## Multiple-angle testing (version7)

The same UF2 adds encoder-stopped in-place turns for any finite angle from15
through180degrees inclusive. Commands include `turn_left 30`, `turn_right 45`,
`turn_left 60`, `turn_right 90`, `turn_left 135`, `turn_right 180`.
Underscore forms such as `turn_left_90` and `turn_right_45` also work.
Invalid, missing or trailing nonnumeric arguments are rejected without movement.
A new motion while one is running is rejected. No commands are queued.

Left turns reverse the left wheel and advance the right; right turns do the
opposite. Both wheels use their calibrated millimetres/count and the provisional
115mm track. Wheel goal is pi*track*angle/360mm. Progress is synchronized using
the existing bounded correction; both wheels brake together when the encoder
angle reaches the requested target. Slowing begins in the final30mm of mean
wheel progress, capped at left24.9%/right25%. The straight4.5mm braking allowance
is not applied to turns: turn scrubbing/coasting needs its own measurements.
Production controller remains absent; IR disabled; startup stopped. GP20 still
starts300mm when idle and stops any active test after release; GP21/serialstop
stop any test. Five-second cutoff,700ms per-wheel no-progress cutoff, feedback
fault stops, main-service timeout and watchdog also apply to turns.

1. Flash with motor power off. Confirm version7 with `status`.
2. With wheels raised, verify `turn_left 30` drives left backward/right forward;
   check `stop` and buttons before mat testing. Stop if polarity is wrong.
3. On the mat mark an initial heading and angle rays using a protractor around
   the midpoint of the wheel axle. Keep tape/paper out from under the tyres and
   leave enough room for the entire robot to rotate on the same surface.
4. Start with `turn_left 45`; wait1second after completion, then send `status`.
   Read the actual body heading from above, relative to the starting heading.
5. Reposition while stopped before each run. Capture `status` before repositioning
   so manual movement does not contaminate settled angle measurements.
6. Test left90 and right90 three times each; then test30/45/60/135/180 as needed.
   Record command, actual angle, and the settled `encoder_angle_deg` for each.

`requested_deg` and `encoder_angle_deg` use positiveleft/negativeright. The
encoder estimate is not an external measurement of physical heading. `GOAL`
means the encoder threshold caused braking, not proof of exact final heading.
Wait for coasting to finish before measuring. Testing multiple angles helps
distinguish a scale/track error from a roughly constant braking overshoot.
Do not alter the now-working straight distance calibration to correct turn-only
errors. Production APIs and the old RTOS angle target have not been changed.
