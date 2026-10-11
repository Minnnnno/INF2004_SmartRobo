#ifdef ROBO_MANUAL_CONSOLE
#include "robot_rtos.h"
#include "robot.h"
#include "subsystems.h"
#include "motion_hal.h"
#include "template_hal.h"
#include "motor_test.h"
#include "angle_test.h"
#include <tm/tmonitor.h>
#define printf(format, ...) tm_printf((const UB *)(format), __VA_ARGS__)
#define puts(text) tm_putstring((const UB *)(text "\n"))
#include <string.h>
#include <math.h>

#ifdef ROBO_ANGLE_TEST
static const UB *angle_state_name(angle_state_t state)
{
    static const char *names[] = {"idle", "turning", "braking", "done", "stopped", "timeout", "feedback_fault", "overshoot"};
    return (const UB *)names[(unsigned)state];
}
#endif

#ifndef ROBO_ENCODER_BUILD_ID
#define ROBO_ENCODER_BUILD_ID "manual-console-encdiag-v1"
#endif

static const UB *count_sign(uint32_t raw)
{
    return (const UB *)((raw > INT32_MAX) ? "-" : "");
}
static unsigned long count_magnitude(uint32_t raw)
{
    return (unsigned long)((raw > INT32_MAX) ? 0U - raw : raw);
}
static void print_counts(const encoder_sample_t *sample)
{
    /* Keep modulo uint32 counters internally. Explicit sign/magnitude formatting
     * also handles INT32_MIN without the legacy formatter's signed negation. */
    printf("counts=%s%lu,%s%lu raw_counts=%lu,%lu invalid=%lu,%lu\n",
           count_sign(sample->count[0]), count_magnitude(sample->count[0]),
           count_sign(sample->count[1]), count_magnitude(sample->count[1]),
           (unsigned long)sample->count[0], (unsigned long)sample->count[1],
           (unsigned long)sample->invalid[0], (unsigned long)sample->invalid[1]);
}

static void encoder_diagnostics(void)
{
    encoder_diagnostics_t d;
    unsigned wheel;
    motion_hal_encoder_diagnostics(&d);
    printf("firmware=%s built=%s %s backend=rp2040-registers motors_inhibited=%u\n",
           (const UB *)ROBO_ENCODER_BUILD_ID, (const UB *)__DATE__, (const UB *)__TIME__,
           (unsigned)d.motors_inhibited);
    printf("enc_init=%u irq_registered=%u nvic13_enabled=%u\n",
           (unsigned)d.initialised, (unsigned)d.irq_registered, (unsigned)d.nvic_enabled);
    for (wheel = 0; wheel < 2U; ++wheel) {
        printf("%s A=GP%u level=%u edges=%lu irq_mask=%u func=%u B=GP%u level=%u edges=%lu irq_mask=%u func=%u prev_AB=%u\n",
               (const UB *)((wheel == 0U) ? "left" : "right"), d.pins[wheel][0],
               (unsigned)((d.levels >> d.pins[wheel][0]) & 1U), (unsigned long)d.edges[wheel][0],
               (unsigned)d.irq_events[wheel][0], (unsigned)d.function[wheel][0], d.pins[wheel][1],
               (unsigned)((d.levels >> d.pins[wheel][1]) & 1U), (unsigned long)d.edges[wheel][1],
               (unsigned)d.irq_events[wheel][1], (unsigned)d.function[wheel][1],
               (unsigned)d.previous_ab[wheel]);
    }
    print_counts(&d.sample);
    puts("edges count observed rising/falling latches; fast repeated edges may coalesce.");
}

static void usb_diagnostics(void)
{
#if TM_CONSOLE_USB_CDC
    extern volatile UW tm_usb_link_state, tm_usb_init_ok, tm_usb_irq_count;
    extern volatile UW tm_usb_nvic_enabled, tm_usb_dropped;
    extern volatile W tm_usb_def_int_ercd;
    uint32_t irq_state = save_and_disable_interrupts();
    UW link = tm_usb_link_state, init = tm_usb_init_ok, interrupts = tm_usb_irq_count;
    UW enabled = tm_usb_nvic_enabled, dropped = tm_usb_dropped;
    W registered = tm_usb_def_int_ercd;
    restore_interrupts(irq_state);
    printf("usb init=%lu link=%lu irq_count=%lu nvic_enabled=%lu define_irq_result=%ld tx_dropped=%lu\n",
           init, link, interrupts, enabled, registered, dropped);
    puts("link: 1=not mounted, 2=mounted, 3=mounted with DTR; define_irq_result=0 means success.");
#else
    puts("console=uart; USB CDC is not part of this firmware.");
#endif
}

/* Test-only task. No motor activation on USB connection or ARM alone. */
void manual_console(void)
{
    char line[64];
    unsigned used = 0U;
#if !defined(ROBO_ENCODER_DIAGNOSTICS_ONLY) && !defined(ROBO_SAFE_MOTOR_TEST) && !defined(ROBO_ANGLE_TEST)
    uint32_t id = 1U;
    bool armed = false;
    double armed_at = 0.0;
#else
    bool armed = false;
#endif
    motion_hal_console_init();
#ifdef ROBO_ANGLE_TEST
    bool angle_shutdown = false, angle_discard = false;
    angle_state_t reported = ANGLE_IDLE;
    printf("firmware=%s angle_test_ready=%u production_inhibited=1 max_output_percent=25 timeout_ms=8000\n",
           (const UB *)ROBO_ENCODER_BUILD_ID, (unsigned)angle_test_init());
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
    bool test_shutdown = false;
    bool discard_line = false;
    printf("firmware=%s safe_motor_test_ready=%u production_inhibited=1 output=25%% serial_max_ms=1000 ir_continuous=1\n",
           (const UB *)ROBO_ENCODER_BUILD_ID, (unsigned)motor_test_init());
#endif
    for (;;) {
        int input;
        unsigned budget = 32U;
#ifdef ROBO_ANGLE_TEST
        angle_test_poll();
        angle_control_t angle_status = angle_test_status();
        if (angle_status.state != reported) {
            reported = angle_status.state;
            printf("angle_state=%s angle_mdeg=%d delta_counts=%s%lu,%s%lu motors_stopped=%u\n",
                   angle_state_name(reported), (int)(angle_status.angle * 57295.779513),
                   count_sign(angle_status.delta[0]), count_magnitude(angle_status.delta[0]),
                   count_sign(angle_status.delta[1]), count_magnitude(angle_status.delta[1]),
                   (unsigned)!angle_test_active());
        }
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
        motor_test_result_t result;
        if (motor_test_result(&result)) {
            printf("motor_test_done wheel=%s direction=%s reason=%s delta_counts=%s%lu,%s%lu invalid_delta=%lu,%lu motors_stopped=1\n",
                   (const UB *)(result.wheel == 2U ? "both" : (result.wheel == 0U ? "left" : "right")),
                   (const UB *)(result.direction > 0 ? "forward" : "reverse"),
                   (const UB *)(result.timed_out ? "timeout" : "stop"),
                   count_sign(result.delta[0]), count_magnitude(result.delta[0]),
                   count_sign(result.delta[1]), count_magnitude(result.delta[1]),
                   (unsigned long)result.invalid_delta[0], (unsigned long)result.invalid_delta[1]);
        }
#endif
        while (budget-- != 0U) {
            input = motion_hal_console_read();
            if (input == -2) {
                used = 0U; armed = false; motion_stop();
#ifdef ROBO_ANGLE_TEST
                angle_test_stop(); angle_discard = true;
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
                line_forward_pause(); motor_test_stop(); discard_line = true;
#endif
                continue;
            }
            if (input < 0) { break; }
#ifdef ROBO_ANGLE_TEST
            if (angle_discard) {
                if (input == '\n' || input == '\r') { angle_discard = false; }
                continue;
            }
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
            if (discard_line) {
                if ((input == '\n') || (input == '\r')) { discard_line = false; }
                continue;
            }
#endif
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
                        printf("speed_mm_s=%d,%d output_permille=%d,%d\n",
                            (int)(status.motion.speed[0] * 1000.0), (int)(status.motion.speed[1] * 1000.0),
                            (int)(status.motion.output[0] * 1000.0), (int)(status.motion.output[1] * 1000.0));
                    }
                    print_counts(&sample);
#ifdef ROBO_ANGLE_TEST
                    angle_control_t turn = angle_test_status();
                    printf("firmware=%s production_inhibited=1 angle_state=%s angle_mdeg=%d target_mdeg=%d output_permille=%d,%d goal_counts=%u,%u\n",
                           (const UB *)ROBO_ENCODER_BUILD_ID, angle_state_name(turn.state),
                           (int)(turn.angle * 57295.779513), turn.direction * 90000,
                           (int)(turn.output[0] * 1000.0), (int)(turn.output[1] * 1000.0),
                           (unsigned)ceil(turn.goal[0]), (unsigned)ceil(turn.goal[1]));
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
                    printf("production_inhibited=1 motor_test_active=%u output_percent=25 serial_max_ms=1000 ir_continuous=1\n",
                           (unsigned)motor_test_active());
#endif
                } else if (strcmp(line, "encdiag") == 0) {
                    encoder_diagnostics();
                } else if (strcmp(line, "usbdiag") == 0) {
                    usb_diagnostics();
#if !defined(ROBO_ENCODER_DIAGNOSTICS_ONLY) && !defined(ROBO_SAFE_MOTOR_TEST) && !defined(ROBO_ANGLE_TEST)
                } else if (strcmp(line, "ARM") == 0) {
                    armed = true;
                    armed_at = motion_hal_time();
                    puts("One command armed for 5 seconds; configuration gates still apply.");
#endif
                } else if (strcmp(line, "stop") == 0) {
#ifdef ROBO_ANGLE_TEST
                    angle_test_stop(); puts("angle_test_stopped motors_stopped=1");
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
                    line_forward_pause(); motor_test_stop();
                    puts("motors_stopped=1");
#endif
                    motion_stop(); armed = false;
                } else if (strcmp(line, "shutdown") == 0) {
#ifdef ROBO_ANGLE_TEST
                    angle_test_stop(); angle_shutdown = true;
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
                    motor_test_shutdown(); test_shutdown = true;
#endif
                    motion_request_shutdown(); armed = false;
#ifdef ROBO_ANGLE_TEST
                } else if (strcmp(line, "turn_left_90") == 0 || strcmp(line, "turn_right_90") == 0) {
                    if (!angle_shutdown && angle_test_start(strcmp(line, "turn_left_90") == 0 ? 1 : -1)) {
                        puts("angle_test_started target_deg=90 max_output_percent=25 timeout_ms=8000");
                    } else { puts("angle_test_rejected: busy, cooling down or initialization failed"); }
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
                } else if ((strcmp(line, "left_forward") == 0) || (strcmp(line, "left_reverse") == 0) ||
                           (strcmp(line, "right_forward") == 0) || (strcmp(line, "right_reverse") == 0) ||
                           (strcmp(line, "forward") == 0) || (strcmp(line, "backward") == 0)) {
                    unsigned wheel = line[0] == 'l' ? 0U : 1U;
                    bool both = (strcmp(line, "forward") == 0) || (strcmp(line, "backward") == 0);
                    int direction = (strstr(line, "reverse") != NULL || strcmp(line, "backward") == 0) ? -1 : 1;
                    if (!test_shutdown && (both ? motor_test_start_pair(direction) : motor_test_start(wheel, direction))) {
                        puts("motor_test_started output_percent=25 cutoff_ms=1000");
                    } else { puts("motor_test_rejected: busy, cooling down, disabled or initialization failed"); }
#endif
#if !defined(ROBO_ENCODER_DIAGNOSTICS_ONLY) && !defined(ROBO_SAFE_MOTOR_TEST) && !defined(ROBO_ANGLE_TEST)
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
#endif
                } else if (line[0] != '\0') {
                    puts("status | encdiag | usbdiag | stop | shutdown");
#ifdef ROBO_ANGLE_TEST
                    puts("turn_left_90 | turn_right_90 | status | stop");
#elif defined(ROBO_SAFE_MOTOR_TEST)
                    puts("forward | backward | left_forward | left_reverse | right_forward | right_reverse; wait for completion between tests");
#elif defined(ROBO_ENCODER_DIAGNOSTICS_ONLY)
                    puts("Read-only diagnostics build: movement, ARM and reset are unavailable.");
#else
                    puts("ARM then a documented test command");
#endif
                }
            } else if (used < sizeof(line) - 1U) { line[used++] = (char)input; }
            else {
                used = 0U; armed = false;
#ifdef ROBO_ANGLE_TEST
                angle_test_stop(); angle_discard = true;
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
                line_forward_pause(); motor_test_stop(); discard_line = true;
#endif
            }
        }
#ifdef ROBO_ANGLE_TEST
        (void)tk_dly_tsk(10U);
#else
        (void)tk_dly_tsk(20U);
#endif
    }
}
#endif
