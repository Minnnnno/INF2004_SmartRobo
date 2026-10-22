#ifndef ROBO_MOTOR_TEST_H
#define ROBO_MOTOR_TEST_H
#include "motion.h"
typedef struct {
    unsigned wheel; /* 0=left, 1=right, 2=both */
    int direction;
    uint32_t delta[2], invalid_delta[2];
    bool timed_out;
} motor_test_result_t;
#ifdef ROBO_SAFE_MOTOR_TEST
bool motor_test_init(void);
bool motor_test_start(unsigned wheel, int direction);
bool motor_test_start_pair(int direction);
void motor_test_poll(void); /* Absolute-time cutoff fallback; task context, no output. */
#ifdef ROBO_BATTERY_FORWARD_TEST
void motor_test_poll_buttons(void); /* GP20 toggles run/stop; GP21 also stops. Active LOW; two-second cap. */
typedef struct {
    bool start_high, stop_high, ready, armed, active;
    uint32_t polls, stop_actions;
    bool irq_registered;
    uint32_t irq_events;
    uint32_t cutoff_irq_count, polled_cutoffs, remaining_ms;
} motor_test_button_diag_t;
bool motor_test_get_buttons(motor_test_button_diag_t *diagnostics); /* Read-only GPIO/task snapshot. */
#endif
bool motor_test_sensor_forward(void); /* Continuous forward while refreshed by IR task. */
void motor_test_pause_sensor(void); /* Latch IR movement off until reboot; manual tests remain available. */
void motor_test_stop(void);
void motor_test_shutdown(void);
bool motor_test_result(motor_test_result_t *result);
bool motor_test_active(void);
#endif
#endif
