#ifndef ROBO_ANGLE_TEST_H
#define ROBO_ANGLE_TEST_H
#include "float_types.h"
#include "motion.h"
typedef enum
{
    ANGLE_IDLE,
    ANGLE_TURNING,
    ANGLE_BRAKING,
    ANGLE_DONE,
    ANGLE_STOPPED,
    ANGLE_TIMEOUT,
    ANGLE_FEEDBACK,
    ANGLE_OVERSHOOT
} angle_state_t;

typedef struct
{
    motion_config_t config;
    angle_state_t   state;
    bool            ready;
    int             direction;

    encoder_sample_t baseline;
    encoder_sample_t previous;
    uint32_t         delta[2];

    float64_t goal[2];
    float64_t output[2];
    float64_t angle;
    float64_t deadline;
    float64_t brake_deadline;
    float64_t last_change[2];
    float64_t quiet_since;
} angle_control_t;

void angle_control_init(angle_control_t *       control,
                        const motion_config_t * config);
bool angle_control_start(angle_control_t * control, int direction,
                         encoder_sample_t sample, float64_t now);
void angle_control_step(angle_control_t * control, encoder_sample_t sample,
                        float64_t now);
void angle_control_stop(angle_control_t * control);
#ifdef ROBO_ANGLE_TEST
bool            angle_test_init(void);
bool            angle_test_start(int direction);
void            angle_test_poll(void);
void            angle_test_stop(void);
angle_control_t angle_test_status(void);
bool            angle_test_active(void);
#endif
#endif

/*** end of file ***/
