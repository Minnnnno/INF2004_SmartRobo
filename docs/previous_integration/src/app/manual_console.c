#ifdef ROBO_MANUAL_CONSOLE
#include "robot_rtos.h"
#include "robot.h"
#include "motion_hal.h"
#include <stdio.h>
#include <string.h>

/* Test-only task. No motor activation on USB connection or ARM alone. */
void manual_console(void)
{
    char line[64];
    unsigned used = 0U;
    uint32_t id = 1U;
    bool armed = false;
    double armed_at = 0.0;
    motion_hal_console_init();
    for (;;) {
        int input;
        unsigned budget = 32U;
        while ((budget-- != 0U) && ((input = motion_hal_console_read()) >= 0)) {
            if ((input == '\n') || (input == '\r')) {
                robot_status_t status;
                line[used] = '\0';
                used = 0U;
                if (strcmp(line, "status") == 0) {
                    encoder_sample_t sample = motion_hal_sample();
                    if (motion_get_status(&status)) {
                        printf("state=%d fault=%d hold=%d rest=%d trusted=%d id=%lu generation=%lu\n",
                            status.motion.state, status.motion.fault, status.motion.hold,
                            status.motion.stationary, status.motion.feedback_trusted,
                            (unsigned long)status.motion.id, (unsigned long)status.motion.generation);
                        printf("counts=%lu,%lu invalid=%lu,%lu speed=%.4f,%.4f output=%.3f,%.3f\n",
                            (unsigned long)sample.count[0], (unsigned long)sample.count[1],
                            (unsigned long)sample.invalid[0], (unsigned long)sample.invalid[1],
                            status.motion.speed[0], status.motion.speed[1],
                            status.motion.output[0], status.motion.output[1]);
                    }
                } else if (strcmp(line, "ARM") == 0) {
                    armed = true;
                    armed_at = motion_hal_time();
                    puts("One command armed for 5 seconds; configuration gates still apply.");
                } else if (strcmp(line, "stop") == 0) {
                    motion_stop(); armed = false;
                } else if (strcmp(line, "shutdown") == 0) {
                    motion_request_shutdown(); armed = false;
                } else if (armed && ((motion_hal_time() - armed_at) <= 5.0)) {
                    motion_result_t result = R_INVALID;
                    armed = false;
                    if (strcmp(line, "left") == 0) { result = motion_manual_output(id++, 0.10, 0.0); }
                    else if (strcmp(line, "right") == 0) { result = motion_manual_output(id++, 0.0, 0.10); }
                    else if (strcmp(line, "left_reverse") == 0) { result = motion_manual_output(id++, -0.10, 0.0); }
                    else if (strcmp(line, "right_reverse") == 0) { result = motion_manual_output(id++, 0.0, -0.10); }
                    else if (strcmp(line, "forward") == 0) { result = motion_move_forward(id++, 0.10); }
                    else if (strcmp(line, "backward") == 0) { result = motion_move_backward(id++, 0.10); }
                    else if (strcmp(line, "turn_left") == 0) { result = motion_turn_left(id++, 1.57079632679); }
                    else if (strcmp(line, "turn_right") == 0) { result = motion_turn_right(id++, 1.57079632679); }
                    else if (strcmp(line, "release_checked") == 0) { motion_controller_release(true); result = R_OK; }
                    else if (strcmp(line, "reset_cause_cleared_rest_verified") == 0) {
                        motion_fault_reset(true, true); result = R_OK;
                    }
                    printf("submission=%d; inspect status for approval/result\n", result);
                } else if (line[0] != '\0') {
                    puts("status | stop | shutdown | ARM then a documented test command");
                }
            } else if (used < sizeof(line) - 1U) { line[used++] = (char)input; }
            else { used = 0U; armed = false; }
        }
        (void)tk_dly_tsk(20U);
    }
}
#endif
