#ifndef ROBO_BOARD_CONFIG_H
#define ROBO_BOARD_CONFIG_H
#include "motion.h"
extern const unsigned g_motor_pins[2][2];
extern const unsigned g_encoder_pins[2][2];
/* Pico GPIO numbers, not connector pin numbers. Optional: buttons GP20/GP21,
 * UART GP0/GP1. */
enum
{
    BOARD_ENC_L_A = 2,
    BOARD_ENC_L_B = 3,
    BOARD_ENC_R_A = 4,
    BOARD_ENC_R_B = 5
};
enum
{
    BOARD_I2C_SDA = 6,
    BOARD_I2C_SCL = 7,
    BOARD_SERVO   = 15,
    BOARD_US_TRIG = 16,
    BOARD_US_ECHO = 17,
    BOARD_LINE_R  = 26,
    BOARD_LINE_C  = 27,
    BOARD_BARCODE = 28,
    /* Optional, active low: barcode calibration (line images). */
    BOARD_CAL_BUTTON = 21
};
bool            board_pins_valid(void);
motion_config_t board_motion_config(void);
#endif

/*** end of file ***/
