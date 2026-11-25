/** @file motor_test.h
 *
 * @brief Safe motor test interface.
 */

#ifndef ROBO_MOTOR_TEST_H
#define ROBO_MOTOR_TEST_H
#include "motion.h"
typedef struct
{
    unsigned wheel; /* 0=left, 1=right, 2=both */
    int      direction;

    uint32_t delta[2];
    uint32_t invalid_delta[2];
    bool     timed_out;
} motor_test_result_t;

#ifdef ROBO_SAFE_MOTOR_TEST
bool motor_test_init(void);
bool motor_test_start(unsigned wheel, int direction);
bool motor_test_start_pair(int direction);
/*! @brief Absolute-time cutoff fallback; task context, no output. */
void motor_test_poll(void);
#ifdef ROBO_BATTERY_FORWARD_TEST
/*! @brief GP20 toggles run/stop; GP21 also stops. Active LOW; two-second cap.
 */
void motor_test_poll_buttons(void);

typedef struct
{
    bool     start_high;
    bool     stop_high;
    bool     ready;
    bool     armed;
    bool     active;
    uint32_t polls;
    uint32_t stop_actions;
    bool     irq_registered;
    uint32_t irq_events;

    uint32_t cutoff_irq_count;
    uint32_t polled_cutoffs;
    uint32_t remaining_ms;
} motor_test_button_diag_t;

/*! @brief Read-only GPIO/task snapshot. */
bool motor_test_get_buttons(motor_test_button_diag_t * p_diagnostics);
#endif
/*! @brief Continuous forward while refreshed by IR task. */
bool motor_test_sensor_forward(void);
/*! @brief Latch IR movement off until reboot; manual tests remain available. */
void motor_test_pause_sensor(void);
void motor_test_stop(void);
void motor_test_shutdown(void);
bool motor_test_result(motor_test_result_t * p_result);
bool motor_test_active(void);
#endif
#endif

/*** end of file ***/
