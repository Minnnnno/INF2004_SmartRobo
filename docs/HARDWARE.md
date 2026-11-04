# Hardware evidence and unresolved measurements

Evidence reviewed: Cytron **Robo Pico datasheet Rev 1.0, April 2023**, pages 3, 5 and 7:
[manufacturer-authored PDF hosted by Farnell](https://www.farnell.com/datasheets/4248509.pdf).
The [Cytron motor tutorial](https://my.cytron.io/tutorial/get-started-robo-pico-circuitpython-dc-motor)
also identifies the motor-terminal connections. These establish a supported board
profile, not the revision of the user's physical board. Confirm its markings and
electrical interface before setting `hardware_confirmed=true`.

For that profile, GP8/9 control M1 and GP10/11 control M2. LOW/LOW brakes;
HIGH/LOW and LOW/HIGH drive opposite directions; HIGH/HIGH makes the outputs
high impedance (coast). PWM alternates drive and LOW/LOW braking. Stopped behaviour
selects brake/coast; startup and fatal shutdown always force LOW/LOW. Normal polarity
depends on wiring and is configured separately for each wheel.

The datasheet limits motor PWM to 20 kHz. Software rejects higher values, rounds
the divider upward to avoid exceeding the request, and limits duty separately.
The default 20 kHz request, output cap and reversal delay still require bench
measurement. Motor/servo power follows board power, not GPIO; current and supply
ratings must be checked against the actual motors. GPIO input compatibility and
encoder supply/output circuit are **not confirmed**. Do not assume a 5 V encoder
signal can connect directly to Pico GPIO. Confirm the ultrasonic echo interface too.

| Signal | GPIO / status |
|---|---|
| Left motor M1A/M1B | 8/9, fixed board driver links |
| Right motor M2A/M2B | 10/11, fixed board driver links |
| Left encoder A/B | GP2/GP3 |
| Right encoder A/B | GP4/GP5 |
| Magnetometer/IMU | GP6/GP7 (SDA/SCL), **I2C1**; subsystem 4 temporarily disabled by default |
| Servo | GP15; PWM slice 7B, 50 Hz, when subsystem 5 is enabled |
| Ultrasonic | GP16/GP17, TRIG/ECHO, when subsystem 5 is enabled |
| Line sensors | GP26 right / GP27 centre, digital inputs in line-steering firmware |
| Barcode | GP28, reserved ADC2; barcode acquisition is currently inactive |
| UART0 TX/RX | GP0/GP1, only when UART console/crash reporting is enabled |
| Test buttons | GP20 start/stop / GP21 stop, only in button calibration profiles |
| Pico W internal signals | GP23/GP24/GP25/GP29, reserved for board/radio use |

Current encoder connector wiring (GPIO assignments supersede the Week 6 report):

| Encoder connector pin | Signal | Left encoder (M1) | Right encoder (M2) |
|---|---|---|---|
| 1 | GND | GND | GND |
| 2 | A | GP2 | GP4 |
| 3 | B | GP3 | GP5 |
| 4 | Supply | 3V3 | 3V3 |

Connector pins 2/3 label channels A/B. The left encoder now connects these to
GP2/GP3 and the right encoder to GP4/GP5. Move the IMU SDA/SCL wires to GP6/GP7
to avoid sharing GP2/GP3 with the left encoder. Named definitions are in
`include/board_config.h`; the I2C1 driver pin settings are in
`device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h`. Physical encoder compatibility,
signs and counts still require the calibration checks.

Production encoder direction signs are now left **-1**, right **+1**. The user
confirmed physical forward motion while both measured speeds were negative
with the previous signs. See [encoder polarity evidence and correction](ENCODER_POLARITY_2026-11-04.md).

Motors use PWM slices 4 and 5; encoder signals are digital inputs, not PWM. ADC
use on 26/27 does not claim PWM slice 5. The I2C1 driver on GP6/GP7 is only
initialized when `ROBO_SUBSYSTEM4=ON` (currently OFF by default). The subsystem 5
servo uses GP15 on PWM slice 7B, separate from I2C1, the motor PWM slices and
the ultrasonic ECHO timer on slice 0B. Diagnostic mode `S5_DIAG=6` instead
uses GP14 on slice 7A.
The earlier BSP I2C0 defaults GP8/9 overlap motor pins
and I2C0 must remain inactive. Do not start core 1 or overwrite
the board GPIO dispatcher. Other buddies register nonoverlapping pin masks through
board_gpio_irq_register; see TEAM.md. The template USB IRQ shim is not a GPIO dispatcher.

Mandatory values still missing: software counts per full wheel revolution,
left/right effective diameters, effective track width, motor signs, encoder signs
and channel type, actual board revision, sensor electrical levels and gearing.
Zero geometry defaults intentionally fail validation. No simulated fixture value
may be copied into hardware configuration as if measured. The GPIO arrays and
all tuning live in `app_program/subsystem2/board_config.c`; preserve the board's fixed motor
links and check every changed encoder assignment against this reservation table.

A magnetometer alone cannot provide pitch, collision acceleration or hump height.
Buddy 4 must confirm whether an accelerometer/gyro is present; unavailable telemetry
remains marked unavailable. Driver overcurrent/thermal diagnostics and independent
wheel-slip detection are not provided by this code.
