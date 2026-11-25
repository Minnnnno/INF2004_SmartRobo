#ifdef ROBO_MANUAL_CONSOLE
#include "robot_rtos.h"
#include "../../include/float_types.h"
#include <stdint.h>
#include "robot.h"
#include "subsystems.h"
#include "motion_hal.h"
#include "template_hal.h"
#include "motor_test.h"
#include "angle_test.h"
#include <tm/tmonitor.h>
#include <string.h>
#include <math.h>

#ifdef ROBO_ANGLE_TEST
static const UB *
angle_state_name (angle_state_t state)
{
    static const char * names[]
        = {"idle",    "turning", "braking",        "done",
           "stopped", "timeout", "feedback_fault", "overshoot"};

    return (const UB *)names[(unsigned)state];
}
#endif

#ifndef ROBO_ENCODER_BUILD_ID
#define ROBO_ENCODER_BUILD_ID "manual-console-encdiag-v1"
#endif

static const UB *
count_sign (uint32_t raw)
{
    return (const UB *)((raw > INT32_MAX) ? "-" : "");
}

static uint32_t
count_magnitude (uint32_t raw)
{
    return (uint32_t)((raw > INT32_MAX) ? 0U - raw : raw);
}

static void
print_counts (const encoder_sample_t * p_sample)
{
    /* Keep modulo uint32 counters internally. Explicit sign/magnitude
     * formatting also handles INT32_MIN without the legacy formatter's signed
     * negation. */
    tm_printf(
        (const UB *)"counts=%s%lu,%s%lu raw_counts=%lu,%lu invalid=%lu,%lu\n",
        count_sign(p_sample->count[0]), count_magnitude(p_sample->count[0]),
        count_sign(p_sample->count[1]), count_magnitude(p_sample->count[1]),
        (uint32_t)p_sample->count[0], (uint32_t)p_sample->count[1],
        (uint32_t)p_sample->invalid[0], (uint32_t)p_sample->invalid[1]);
}

static void
encoder_diagnostics (void)
{
    encoder_diagnostics_t diagnostics;
    unsigned              wheel;

    motion_hal_encoder_diagnostics(&diagnostics);
    tm_printf((const UB *)"firmware=%s built=%s %s backend=rp2040-registers "
                          "motors_inhibited=%u\n",
              (const UB *)ROBO_ENCODER_BUILD_ID, (const UB *)__DATE__,
              (const UB *)__TIME__, (unsigned)diagnostics.motors_inhibited);
    tm_printf((const UB *)"enc_init=%u irq_registered=%u nvic13_enabled=%u\n",
              (unsigned)diagnostics.initialised,
              (unsigned)diagnostics.irq_registered,
              (unsigned)diagnostics.nvic_enabled);

    for (wheel = 0; wheel < 2U; ++wheel)
    {
        tm_printf(
            (const UB *)"%s A=GP%u level=%u edges=%lu irq_mask=%u func=%u "
                        "B=GP%u level=%u "
                        "edges=%lu irq_mask=%u func=%u prev_AB=%u\n",
            (const UB *)((0U == wheel) ? "left" : "right"),
            diagnostics.pins[wheel][0],
            (unsigned)((diagnostics.levels >> diagnostics.pins[wheel][0]) & 1U),
            (uint32_t)diagnostics.edges[wheel][0],
            (unsigned)diagnostics.irq_events[wheel][0],
            (unsigned)diagnostics.function[wheel][0],
            diagnostics.pins[wheel][1],
            (unsigned)((diagnostics.levels >> diagnostics.pins[wheel][1]) & 1U),
            (uint32_t)diagnostics.edges[wheel][1],
            (unsigned)diagnostics.irq_events[wheel][1],
            (unsigned)diagnostics.function[wheel][1],
            (unsigned)diagnostics.previous_ab[wheel]);
    }

    print_counts(&diagnostics.sample);
    tm_putstring((const UB *)"edges count observed rising/falling latches; "
                             "fast repeated edges may "
                             "coalesce.\n");
}

static void
usb_diagnostics (void)
{
#if TM_CONSOLE_USB_CDC
    extern volatile UW tm_usb_link_state;
    extern volatile UW tm_usb_init_ok;
    extern volatile UW tm_usb_irq_count;
    extern volatile UW tm_usb_nvic_enabled;
    extern volatile UW tm_usb_dropped;
    extern volatile W  tm_usb_def_int_ercd;
    uint32_t           irq_state  = save_and_disable_interrupts();
    UW                 link       = tm_usb_link_state;
    UW                 init       = tm_usb_init_ok;
    UW                 interrupts = tm_usb_irq_count;
    UW                 enabled    = tm_usb_nvic_enabled;
    UW                 dropped    = tm_usb_dropped;

    W registered = tm_usb_def_int_ercd;
    restore_interrupts(irq_state);
    tm_printf(
        (const UB *)"usb init=%lu link=%lu irq_count=%lu nvic_enabled=%lu "
                    "define_irq_result=%ld tx_dropped=%lu\n",
        init, link, interrupts, enabled, registered, dropped);
    tm_putstring(
        (const UB *)"link: 1=not mounted, 2=mounted, 3=mounted with DTR; "
                    "define_irq_result=0 means success.\n");
#else
    tm_putstring(
        (const UB *)"console=uart; USB CDC is not part of this firmware.\n");
#endif
}

#ifdef ROBO_BATTERY_FORWARD_TEST
static void
button_diagnostics (void)
{
    motor_test_button_diag_t delta;

    if (!motor_test_get_buttons(&delta))
    {
        return;
    }

    tm_printf((const UB *)"firmware=%s profile=button_test gp20_mode=toggle "
                          "gp21_mode=stop "
                          "ir_enabled=0 automatic_start=0\n",
              (const UB *)ROBO_ENCODER_BUILD_ID);
    tm_printf((const UB *)"GP20=%u GP21=%u pressed_level=0 ready=%u armed=%u "
                          "active=%u "
                          "polls=%lu stop_actions=%lu\n",
              (unsigned)delta.start_high, (unsigned)delta.stop_high,
              (unsigned)delta.ready, (unsigned)delta.armed,
              (unsigned)delta.active, (uint32_t)delta.polls,
              (uint32_t)delta.stop_actions);
    tm_printf((const UB *)"button_irq_registered=%u button_irq_events=%lu "
                          "cutoff_ms=2000\n",
              (unsigned)delta.irq_registered, (uint32_t)delta.irq_events);
    tm_printf((const UB *)"cutoff_irq_count=%lu polled_cutoffs=%lu "
                          "remaining_ms=%lu\n",
              (uint32_t)delta.cutoff_irq_count, (uint32_t)delta.polled_cutoffs,
              (uint32_t)delta.remaining_ms);
}
#endif

/* Test-only task. No motor activation on USB connection or ARM alone. */
void
manual_console (void)
{
    char     line[64];
    unsigned used = 0U;
#if !defined(ROBO_ENCODER_DIAGNOSTICS_ONLY) && !defined(ROBO_SAFE_MOTOR_TEST) \
    && !defined(ROBO_ANGLE_TEST)
    uint32_t  command_id = 1U;
    bool      b_armed    = false;
    float64_t armed_at   = 0.0;
#else
    bool b_armed = false;
#endif
    motion_hal_console_init();
#ifdef ROBO_ANGLE_TEST
    bool          b_angle_shutdown = false;
    bool          b_angle_discard  = false;
    angle_state_t reported         = ANGLE_IDLE;

    tm_printf(
        (const UB *)"firmware=%s angle_test_ready=%u production_inhibited=1 "
                    "max_output_percent=25 timeout_ms=8000\n",
        (const UB *)ROBO_ENCODER_BUILD_ID, (unsigned)angle_test_init());
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
    bool b_test_shutdown = false;
    bool b_discard_line  = false;
    bool b_test_ready    = motor_test_init();

    tm_printf((const UB *)"firmware=%s safe_motor_test_ready=%u "
                          "production_inhibited=1 "
                          "single_output_percent=25 paired_left_percent=24.9 "
                          "paired_right_percent=25 serial_max_ms=1000\n",
              (const UB *)ROBO_ENCODER_BUILD_ID, (unsigned)b_test_ready);
#ifdef ROBO_BATTERY_FORWARD_TEST
    tm_putstring((const UB *)"button_test_ready start_stop=GP20 "
                             "extra_stop=GP21 active_low=1 "
                             "cutoff_ms=2000 ir_enabled=0 automatic_start=0\n");
#endif
#endif
    for (;;)
    {
        int      input;
        unsigned budget = 32U;
#ifdef ROBO_BATTERY_FORWARD_TEST
        motor_test_poll_buttons();
#elif defined(ROBO_SAFE_MOTOR_TEST)
        motor_test_poll();
#endif
#ifdef ROBO_ANGLE_TEST
        angle_test_poll();
        angle_control_t angle_status = angle_test_status();

        if (angle_status.state != reported)
        {
            reported = angle_status.state;
            tm_printf((const UB *)"angle_state=%s angle_mdeg=%d "
                                  "delta_counts=%s%lu,%s%lu "
                                  "motors_stopped=%u\n",
                      angle_state_name(reported),
                      (int)(angle_status.angle * 57295.779513),
                      count_sign(angle_status.delta[0]),
                      count_magnitude(angle_status.delta[0]),
                      count_sign(angle_status.delta[1]),
                      count_magnitude(angle_status.delta[1]),
                      (unsigned)!angle_test_active());
        }
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
        motor_test_result_t result;

        if (motor_test_result(&result))
        {
            tm_printf(
                (const UB *)"motor_test_done wheel=%s direction=%s reason=%s "
                            "delta_counts=%s%lu,%s%lu invalid_delta=%lu,%lu "
                            "motors_stopped=1\n",
                (const UB *)(result.wheel == 2U
                                 ? "both"
                                 : (0U == result.wheel ? "left" : "right")),
                (const UB *)(result.direction > 0 ? "forward" : "reverse"),
                (const UB *)(result.timed_out ? "timeout" : "stop"),
                count_sign(result.delta[0]), count_magnitude(result.delta[0]),
                count_sign(result.delta[1]), count_magnitude(result.delta[1]),
                (uint32_t)result.invalid_delta[0],
                (uint32_t)result.invalid_delta[1]);
        }
#endif
        while (0U != budget--)
        {
            input = motion_hal_console_read();

            if (-2 == input)
            {
                used    = 0U;
                b_armed = false;
                motion_stop();
#ifdef ROBO_ANGLE_TEST
                angle_test_stop();
                b_angle_discard = true;
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
                line_forward_pause();
                motor_test_stop();
                b_discard_line = true;
#endif
                continue;
            }

            if (input < 0)
            {
                break;
            }
#ifdef ROBO_ANGLE_TEST
            if (b_angle_discard)
            {
                if (('\n' == input) || ('\r' == input))
                {
                    b_angle_discard = false;
                }

                continue;
            }
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
            if (b_discard_line)
            {
                if (('\n' == input) || ('\r' == input))
                {
                    b_discard_line = false;
                }

                continue;
            }
#endif
            if (('\n' == input) || ('\r' == input))
            {
                robot_status_t status;

                line[used] = '\0';
                used       = 0U;

                if (0 == strcmp(line, "status"))
                {
                    encoder_sample_t sample = motion_hal_sample();

                    if (motion_get_status(&status))
                    {
                        tm_printf((const UB *)"state=%d fault=%d hold=%d "
                                              "rest=%d trusted=%d "
                                              "id=%lu generation=%lu\n",
                                  status.motion.state, status.motion.fault,
                                  status.motion.hold, status.motion.stationary,
                                  status.motion.feedback_trusted,
                                  (uint32_t)status.motion.id,
                                  (uint32_t)status.motion.generation);
                        tm_printf((const UB *)"speed_mm_s=%d,%d "
                                              "output_permille=%d,%d\n",
                                  (int)(status.motion.speed[0] * 1000.0),
                                  (int)(status.motion.speed[1] * 1000.0),
                                  (int)(status.motion.output[0] * 1000.0),
                                  (int)(status.motion.output[1] * 1000.0));
                    }

                    print_counts(&sample);
#ifdef ROBO_ANGLE_TEST
                    angle_control_t turn = angle_test_status();

                    tm_printf((const UB *)"firmware=%s production_inhibited=1 "
                                          "angle_state=%s "
                                          "angle_mdeg=%d target_mdeg=%d "
                                          "output_permille=%d,%d "
                                          "goal_counts=%u,%u\n",
                              (const UB *)ROBO_ENCODER_BUILD_ID,
                              angle_state_name(turn.state),
                              (int)(turn.angle * 57295.779513),
                              turn.direction * 90000,
                              (int)(turn.output[0] * 1000.0),
                              (int)(turn.output[1] * 1000.0),
                              (unsigned)ceil(turn.goal[0]),
                              (unsigned)ceil(turn.goal[1]));
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
                    tm_printf((const UB *)"production_inhibited=1 "
                                          "motor_test_active=%u "
                                          "single_output_percent=25 "
                                          "paired_left_percent=24.9 "
                                          "paired_right_percent=25 "
                                          "serial_max_ms=1000\n",
                              (unsigned)motor_test_active());
#ifdef ROBO_BATTERY_FORWARD_TEST
                    button_diagnostics();
#endif
#endif
#ifdef ROBO_BATTERY_FORWARD_TEST
                }
                else if (0 == strcmp(line, "buttons"))
                {
                    button_diagnostics();
#endif
                }
                else if (0 == strcmp(line, "encdiag"))
                {
                    encoder_diagnostics();
                }
                else if (0 == strcmp(line, "usbdiag"))
                {
                    usb_diagnostics();
#if !defined(ROBO_ENCODER_DIAGNOSTICS_ONLY) && !defined(ROBO_SAFE_MOTOR_TEST) \
    && !defined(ROBO_ANGLE_TEST)
                }
                else if (0 == strcmp(line, "ARM"))
                {
                    b_armed  = true;
                    armed_at = motion_hal_time();
                    tm_putstring((const UB *)"One command armed for 5 seconds; "
                                             "configuration gates "
                                             "still apply.\n");
#endif
                }
                else if (0 == strcmp(line, "stop"))
                {
#ifdef ROBO_ANGLE_TEST
                    angle_test_stop();
                    tm_putstring(
                        (const UB *)"angle_test_stopped motors_stopped=1\n");
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
                    line_forward_pause();
                    motor_test_stop();
                    tm_putstring((const UB *)"motors_stopped=1 "
                                             "ir_paused_until_reboot=1\n");
#endif
                    motion_stop();
                    b_armed = false;
                }
                else if (0 == strcmp(line, "shutdown"))
                {
#ifdef ROBO_ANGLE_TEST
                    angle_test_stop();
                    b_angle_shutdown = true;
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
                    motor_test_shutdown();
                    b_test_shutdown = true;
#endif
                    motion_request_shutdown();
                    b_armed = false;
#ifdef ROBO_ANGLE_TEST
                }
                else if ((0 == strcmp(line, "turn_left_90"))
                         || (0 == strcmp(line, "turn_right_90")))
                {
                    if ((!b_angle_shutdown)
                        && (angle_test_start(
                            (0 == strcmp(line, "turn_left_90")) ? 1 : -1)))
                    {
                        tm_putstring(
                            (const UB *)"angle_test_started target_deg=90 "
                                        "max_output_percent=25 "
                                        "timeout_ms=8000\n");
                    }
                    else
                    {
                        tm_putstring((const UB *)"angle_test_rejected: busy, "
                                                 "cooling down or "
                                                 "initialization failed\n");
                    }
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
                }
                else if ((0 == strcmp(line, "left_forward"))
                         || (0 == strcmp(line, "left_reverse"))
                         || (0 == strcmp(line, "right_forward"))
                         || (0 == strcmp(line, "right_reverse"))
                         || (0 == strcmp(line, "forward"))
                         || (0 == strcmp(line, "backward")))
                {
                    unsigned wheel     = ('l' == line[0]) ? 0U : 1U;
                    bool     b_both    = (0 == strcmp(line, "forward"))
                                         || (0 == strcmp(line, "backward"));
                    int      direction = ((NULL != strstr(line, "reverse"))
                                          || (0 == strcmp(line, "backward")))
                                             ? -1
                                             : 1;

                    /* Calibration pulses cannot be followed by automatic IR
                     * movement. */
                    line_forward_pause();

                    if ((!b_test_shutdown)
                        && (b_both ? motor_test_start_pair(direction)
                                   : motor_test_start(wheel, direction)))
                    {
                        if (b_both)
                        {
                            tm_putstring((const UB *)"motor_test_started "
                                                     "left_output_percent=24.9 "
                                                     "right_output_percent=25 "
                                                     "cutoff_ms=1000\n");
                        }
                        else
                        {
                            tm_putstring((const UB *)"motor_test_started "
                                                     "output_percent=25 "
                                                     "cutoff_ms=1000\n");
                        }
                    }
                    else
                    {
                        tm_putstring(
                            (const UB *)"motor_test_rejected: busy, cooling "
                                        "down, "
                                        "disabled or initialization failed\n");
                    }
#endif
#if !defined(ROBO_ENCODER_DIAGNOSTICS_ONLY) && !defined(ROBO_SAFE_MOTOR_TEST) \
    && !defined(ROBO_ANGLE_TEST)
                }
                else if (b_armed && ((motion_hal_time() - armed_at) <= 5.0))
                {
                    motion_result_t result = R_INVALID;

                    b_armed = false;

                    if (0 == strcmp(line, "left"))
                    {
                        result = motion_manual_output(command_id++, 0.10, 0.0);
                    }
                    else if (0 == strcmp(line, "right"))
                    {
                        result = motion_manual_output(command_id++, 0.0, 0.10);
                    }
                    else if (0 == strcmp(line, "left_reverse"))
                    {
                        result = motion_manual_output(command_id++, -0.10, 0.0);
                    }
                    else if (0 == strcmp(line, "right_reverse"))
                    {
                        result = motion_manual_output(command_id++, 0.0, -0.10);
                    }
                    else if (0 == strcmp(line, "forward"))
                    {
                        result = motion_move_forward(command_id++, 0.10);
                    }
                    else if (0 == strcmp(line, "backward"))
                    {
                        result = motion_move_backward(command_id++, 0.10);
                    }
                    else if (0 == strcmp(line, "turn_left"))
                    {
                        result = motion_turn_left(command_id++, 1.57079632679);
                    }
                    else if (0 == strcmp(line, "turn_right"))
                    {
                        result = motion_turn_right(command_id++, 1.57079632679);
                    }
                    else if (0 == strcmp(line, "release_checked"))
                    {
                        motion_controller_release(true);
                        result = R_OK;
                    }
                    else if (0
                             == strcmp(line,
                                       "reset_cause_cleared_rest_verified"))
                    {
                        motion_fault_reset(true, true);
                        result = R_OK;
                    }

                    tm_printf((const UB *)"submission=%d; inspect status for "
                                          "approval/result\n",
                              result);
#endif
                }
                else if ('\0' != line[0])
                {
                    tm_putstring((const UB *)"status | encdiag | usbdiag | "
                                             "stop | shutdown\n");
#ifdef ROBO_BATTERY_FORWARD_TEST
                    tm_putstring(
                        (const UB *)"buttons: raw GP20/GP21 levels and button "
                                    "polling/stop diagnostics\n");
#endif
#ifdef ROBO_ANGLE_TEST
                    tm_putstring((const UB *)"turn_left_90 | turn_right_90 | "
                                             "status | stop\n");
#elif defined(ROBO_SAFE_MOTOR_TEST)
                    tm_putstring((const UB *)"forward | backward | "
                                             "left_forward | left_reverse | "
                                             "right_forward | right_reverse; "
                                             "wait for completion "
                                             "between tests\n");
#elif defined(ROBO_ENCODER_DIAGNOSTICS_ONLY)
                    tm_putstring((const UB *)"Read-only diagnostics build: "
                                             "movement, ARM and reset "
                                             "are unavailable.\n");
#else
                    tm_putstring(
                        (const UB *)"ARM then a documented test command\n");
#endif
                }
            }
            else if (used < (sizeof(line) - 1U))
            {
                line[used++] = (char)input;
            }
            else
            {
                used    = 0U;
                b_armed = false;
#ifdef ROBO_ANGLE_TEST
                angle_test_stop();
                b_angle_discard = true;
#endif
#ifdef ROBO_SAFE_MOTOR_TEST
                line_forward_pause();
                motor_test_stop();
                b_discard_line = true;
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

/*** end of file ***/
