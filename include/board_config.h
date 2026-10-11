#ifndef ROBO_BOARD_CONFIG_H
#define ROBO_BOARD_CONFIG_H
#include "motion.h"
extern const unsigned motor_pins[2][2];
extern const unsigned encoder_pins[2][2];
/* Week 6 report allocation: Pico GPIO numbers, not connector pin numbers. */
enum { BOARD_ENC_L_A = 4, BOARD_ENC_L_B = 6,
       BOARD_ENC_R_A = 5, BOARD_ENC_R_B = 7 };
enum { BOARD_I2C_SDA = 2, BOARD_I2C_SCL = 3, BOARD_SERVO = 12,
       BOARD_US_TRIG = 16, BOARD_US_ECHO = 17, BOARD_LINE_L = 26,
       BOARD_LINE_C = 27, BOARD_BARCODE = 28 };
bool board_pins_valid(void);
motion_config_t board_motion_config(void);
#endif
