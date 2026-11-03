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
| Magnetometer/IMU | 6/7 (SDA/SCL), **I2C1**, owned by subsystem 4 |
| Servo | 12, reserved; PWM slice 6A |
| Ultrasonic | 16/17, reserved TRIG/ECHO |
| Right/centre line sensors | 26/27, ADC0/1 |
| Barcode | 28, ADC2, reserved and inactive; confirm VBAT-sense jumper open |
| Calibration buttons | 20/21, optional |
| UART TX/RX | 0/1, optional (`CONSOLE=uart` / `CRASH_REPORT`) |
| Pico W internal | 23/24/25/29, reserved for the radio |

Encoder connector wiring:

| Encoder connector pin | Signal | Left encoder (M1) | Right encoder (M2) |
|---|---|---|---|
| 1 | GND | GND | GND |
| 2 | A | GP2 | GP4 |
| 3 | B | GP3 | GP5 |
| 4 | Supply | 3V3 | 3V3 |

Connector pins 2/3 are labels on the encoder connector, not Pico GPIO numbers.
Named definitions are in `include/board_config.h`; the I2C1 pins are set in
`device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h`. Physical encoder
compatibility, signs and counts still require the calibration checks.

Motors use PWM slices 4 and 5; encoder signals are digital inputs, not PWM. ADC
use on 26/27 does not claim PWM slice 5. Only the I2C1 driver is enabled,
on GP6/GP7; the earlier BSP I2C0 defaults GP8/9 overlap motor pins
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
