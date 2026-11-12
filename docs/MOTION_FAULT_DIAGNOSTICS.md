# Buddy 2 motion fault diagnostics

Runtime entry points and motor diagnostics live in
`app_program/subsystem2/motion_runtime.c`. Shared watchdog/crash handling and
supervision live in `app_program/system/system_runtime.c`; the command bus and
controller task live in `app_program/controller/robot_runtime.c`. `app_main.c`
only coordinates startup and supervision. Core motion/PID/encoder behavior is
unchanged by this source-file separation.

Motion fault latching is enabled by default (`ROBO_MOTION_FAULTS_DISABLED=OFF`).
An existing CMake build directory may still cache `ON`: explicitly configure it
with `cmake -S . -B <build-directory> -DROBO_MOTION_FAULTS_DISABLED=OFF`, then rebuild.
The production USB/UART startup banner must show `[MOTION] faults_enabled=1`.
The changed firmware must be flashed before this behavior takes effect on the robot.

On a fault, the motion task inhibits both motors and holds movement. The lower
priority diagnostics task prints the saved cause every two seconds, ahead of
routine stack output. No serial printing is added to the control loop or encoder ISR.

Read the first `[ERROR]` and `[FAULT]` lines for the fault name, feedback cause,
command ID and captured time. Additional lines show the previous state, control
interval, each wheel's target/measured speed, requested PWM, remaining wheel
distance and no-progress duration/limit. `INT32_MIN` (`-2147483648`) means an invalid
numeric diagnostic value. `[FEEDBACK]` includes the previous and triggering raw
counts and invalid-transition counters. `[DRIVER]`, `[PWM]`, `[PWM REGS]` and
`[ENCODER]` describe hardware captured before the fault's output cutoff.

| Fault | Observed failure |
| --- | --- |
| `CONFIG` | Invalid motion configuration or initial timestamp. |
| `SETUP` (2) | Hardware initialization failed. |
| `SETUP_TIMEOUT` (12) | Hardware initialization or stationary/cadence qualification exceeded the setup deadline. |
| `TIMING` | Nonfinite/nonpositive control interval or interval above `max_dt`. |
| `FEEDBACK` | Unreliable sample, invalid A/B transition, rejected odometry or no encoder progress. |
| `DEADLINE` | Active command reached its completion deadline. |
| `EXPIRED` | Continuous wheel-speed command was not refreshed before expiry. |
| `STOP_TIMEOUT` | Stationary qualification did not complete within the stop timeout. |
| `FINAL_GOAL` | Settled movement failed its final distance/heading check. |
| `EVENT_OVERFLOW` | An event buffer filled before its consumer drained it. |
| `SYSTEM` (10) | Internal system-fault request, motion synchronization or command decoding failure. |
| `ESTOP` (11) | Remote emergency-stop command. |

Existing fault codes 0..10 retain their meanings except that SETUP and SYSTEM
now exclude the separately identified timeout and emergency-stop conditions.
Buddy 1's four-bit M_FAULT field accepts 0..12 without changing frame size or
field positions. Receivers must accept the new range and label codes 11 and 12.
Emergency stop invalidates queued and approved movement and blocks a simultaneous
reset/release. If emergency stop and system fault arrive in the same cycle,
ESTOP is the primary fault and both causes are retained in fault_flags. A fault
already latched in an earlier cycle remains the primary cause.

No encoder progress does not establish a physical cause: a stalled/unpowered
motor and a missing encoder signal can produce the same observation. Use the
saved PWM/edge evidence to guide inspection. The pre-stop hardware snapshot
is from the latest applied output; the saved PID request is from the fault cycle.
Both wheels are evaluated before cutoff. The saved stalled_wheels mask is 1 for
left, 2 for right and 3 for both. Only wheels commanded to move whose grace and
no-progress limits have elapsed are marked; an intentionally stationary wheel is
excluded. An earlier limit on one wheel can still stop the robot before the other
wheel's limit expires. Later samples do not change the captured mask.

Faults do not restart movement automatically. Recovery uses the existing checked
reset, fresh rest qualification and explicit release APIs. Feedback/timing faults
require externally verified rest for reset. The normal production image has no
manual calibration command parser; remote START alone cannot clear a fault.

The checked recovery sequence is:

1. Correct the fault cause and independently verify rest for feedback/timing faults.
2. Request motion_fault_reset(cause_cleared, externally_verified_rest).
3. Wait for the published state to pass through INITIALISING to STOPPED with
   stationary=true, feedback_trusted=true and fault=F_NONE.
4. Request motion_controller_release(checks_ok), then wait for READY.
5. Have the owning planner submit a fresh command with a new ID. Old commands and
   generations cannot restart movement. Reset clears the active command and odometry.

The motion task already feeds a one-second hardware watchdog. A freeze detector
checks every 50 ms and records a freeze after 800 ms without a feed. Hard-fault,
freeze and watchdog reports survive reboot in scratch registers. On crash reboot,
startup copies the crash evidence into RAM and proceeds through normal motor
initialization. The permanent crash-report loop has been removed. Production
diagnostics repeat the saved crash report every two seconds even after the new
watchdog reuses the scratch registers. A reset caused by the bootloader/UF2 load
without our armed marker is not classified as a control-task watchdog failure.

Motor restart follows this sequence:

1. Boot code stops the motor pins; startup stops the old watchdog and saves its report.
2. Reinitialize motor PWM, encoder capture and the motion controller with zero output.
3. Start the motion task, rearm the watchdog and keep feeding it each cycle.
4. Require valid configuration/setup, fresh feedback and qualified stationary rest.
5. Enter READY and accept fresh movement commands from the existing control/sensor tasks.

Known encoder faults are an exception to automatic motor readiness. When feedback
fails, the motion task retains an encoder-fault latch and its first observed cause
in unused bits of watchdog scratch register 0. Watchdog rearming and crash logging
preserve those bits. After a watchdog/hard-fault restart, the motion controller
restores F_FEEDBACK with cause MANUAL_RESET_REQUIRED_AFTER_RESTART before publishing
its initial state. Quiet encoder counts cannot move it to READY. The original
pre-restart cause is printed separately; current counts are not presented as proof
that the previous fault has gone away.

Only an accepted checked reset clears the retained latch. Cause-cleared confirmation,
externally verified rest and a reliable current sample are required; a rejected
reset, remote START or release request cannot clear it. Fresh stationary qualification
and explicit release still follow a successful reset. Any new encoder error latches
again. Watchdog scratch retention survives warm resets, not removal of power; after
a power-cycle the normal startup and runtime encoder checks still apply.

The watchdog protects stationary qualification as well as driving. No position or
mission checkpoint is required for this motor restart. Sensor tasks remain
responsible for navigation decisions. Old PWM, commands and ownership are not
restored. A repeated initialization/feedback failure latches a new motion fault.
An ordinary latched motion fault leaves the task and watchdog running and does
not trigger a reboot; it still uses checked reset/release. Emergency stop also
retains checked reset/release while the system is running.

Host tests cover retained crash evidence and encoder-fault latches across watchdog
rearming, rejected and accepted checked resets, recurring feedback errors, and motor
driver reinitialization followed by a fresh PWM command. Physical motor operation
after a hardware watchdog reset still requires testing on the robot.
