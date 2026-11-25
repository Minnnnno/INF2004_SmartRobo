/** @file motor_test.c
 *
 * @brief Safe motor test image: short timed pulses with a hardware cut-off.
 */

#ifdef ROBO_SAFE_MOTOR_TEST
#include "motor_test.h"
#include "motion_hal.h"
#include "board_config.h"
#include <stddef.h>
#ifdef ROBO_MOTOR_TEST_HOST
#include "motor_test_fake_hal.h"
#else
#include "template_hal.h"
#endif

/* No production configuration gates are changed. Only this isolated profile
 * can call the test actuator. Alarm3/IRQ3 is unused elsewhere in this build. */
static volatile bool gb_ready;
static volatile bool gb_active;
static volatile bool gb_result_pending;
static bool          gb_sensor_forward;
#ifdef ROBO_BATTERY_FORWARD_TEST
static volatile bool     gb_sensor_paused = true;
static volatile bool     gb_button_armed;
static volatile bool     gb_release_tracking;
static volatile bool     gb_press_tracking;
static uint64_t          g_released_since;
static uint64_t          g_pressed_since;
static uint32_t          g_button_polls;
static uint32_t          g_button_stop_actions;
static bool              gb_button_irq_registered;
static volatile bool     gb_stop_release_seen;
static volatile uint64_t g_stop_released_at;
static volatile uint32_t g_button_irq_events;
/*! @brief Ignore the buttons until they are released. */
static void
disarm_buttons (void)
{
    gb_button_armed      = false;
    gb_release_tracking  = false;
    gb_press_tracking    = false;
    gb_stop_release_seen = false;
}
#else
/* Only reboot clears a serial stop/manual-test latch. */
static volatile bool gb_sensor_paused;
#endif
static encoder_sample_t             g_baseline;
static volatile motor_test_result_t g_completed;
static uint64_t                     g_cutoff_at;
static uint32_t                     g_cutoff_irq_count;
static uint32_t                     g_polled_cutoffs;
static uint64_t                     g_cooldown_until;
static int                          g_motor_sign[2];
enum
{
    TEST_DUTY          = 1000,
    PAIR_LEFT_DUTY     = 996,
    TEST_DURATION_US   = 1000000,
    TEST_COOLDOWN_US   = 300000,
    SENSOR_WATCHDOG_US = 300000,
    BUTTON_DURATION_US = 2000000
};

/*! @brief Both motor pins low. */
static void
stop_outputs (void)
{
    pwm_set_both_levels(4U, 0U, 0U);
    pwm_set_both_levels(5U, 0U, 0U);
    pwm_set_enabled(4U, false);
    pwm_set_enabled(5U, false);
    motion_hal_emergency_stop();  /* GP8..11 LOW, PWM disconnected. */
    out_w(TIMER_ARMED, 1U << 3U); /* Write-one-to-disarm. */
    out_w(TIMER_INTR, 1U << 3U);
    ClearInt(3U);
}

/*! @brief End the run with a result, outputs off. */
static void
finish (bool b_timed_out)
{
    unsigned         wheel;
    encoder_sample_t end;

    /* Stop before sampling, publishing, or printing anything. */
    stop_outputs();
#ifdef ROBO_BATTERY_FORWARD_TEST
    /* Stop/timeout requires release and a fresh start press. */
    disarm_buttons();
#endif
    if (!gb_active)
    {
        return;
    }

    end = motion_hal_sample();

    for (wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        g_completed.delta[wheel] = end.count[wheel] - g_baseline.count[wheel];
        g_completed.invalid_delta[wheel]
            = end.invalid[wheel] - g_baseline.invalid[wheel];
    }

    g_completed.timed_out = b_timed_out;
    g_cooldown_until      = time_us_64() + TEST_COOLDOWN_US;
    gb_active             = false;
    gb_sensor_forward     = false;
    gb_result_pending     = true;
}

/*! @brief Hardware deadline: cut the motors. */
static void
deadline_irq (UINT number)
{
    (void)number;
    ++g_cutoff_irq_count;
    finish(true); /* Serial cutoff or missed-sensor-update watchdog. */
}

void
motor_test_poll (void)
{
    uint32_t irq_state = save_and_disable_interrupts();

    if (gb_active && (time_us_64() >= g_cutoff_at))
    {
        ++g_polled_cutoffs;
        finish(true); /* Independent fallback if Alarm3 delivery is missed. */
    }

    restore_interrupts(irq_state);
}

#ifdef ROBO_BATTERY_FORWARD_TEST
/*! @brief Stop from a button press. */
static void
button_stop_now (void)
{
    uint32_t irq_state = save_and_disable_interrupts();

    if (gb_active)
    {
        ++g_button_stop_actions;
        finish(false);
    }

    restore_interrupts(irq_state);
}

/*! @brief Button edges. */
static void
button_irq (void)
{
    uint32_t start_events = gpio_get_irq_event_mask(20U);
    uint32_t stop_events  = gpio_get_irq_event_mask(21U);
    uint64_t now;

    if (0U != start_events)
    {
        gpio_acknowledge_irq(20U, start_events);
    }

    if (0U != stop_events)
    {
        gpio_acknowledge_irq(21U, stop_events);
    }

    if (0U == (start_events | stop_events))
    {
        return;
    }

    ++g_button_irq_events;

    if (!gb_active)
    {
        return;
    }

    if (0U != (stop_events & GPIO_IRQ_EDGE_FALL))
    {
        button_stop_now();
        return;
    }

    now = time_us_64();

    if (0U != (start_events & GPIO_IRQ_EDGE_RISE))
    {
        g_stop_released_at   = now;
        gb_stop_release_seen = true;
    }

    if (0U != (start_events & GPIO_IRQ_EDGE_FALL))
    {
        if (gb_stop_release_seen && ((now - g_stop_released_at) >= 40000U))
        {
            button_stop_now();
        }

        gb_stop_release_seen = false;
    }
}
#endif

/*! @brief Set up the motor test. */
bool
motor_test_init (void)
{
    motion_config_t  config = board_motion_config();
    encoder_sample_t sample = motion_hal_sample();

    pwm_config_t pwm        = pwm_get_default_config();
    T_DINT       definition = {.intatr = TA_HLNG, .inthdr = (FP)deadline_irq};

    gb_ready = false;
    stop_outputs();
#ifdef ROBO_BATTERY_FORWARD_TEST
    disarm_buttons();
    gpio_init(20U);
    gpio_set_dir(20U, false);
    gpio_pull_up(20U);
    gpio_init(21U);
    gpio_set_dir(21U, false);
    gpio_pull_up(21U);
    gpio_set_irq_enabled(20U, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    gpio_set_irq_enabled(21U, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
#endif
    if ((!board_pins_valid()) || (!sample.reliable)
        || (BOARD_ENC_L_A != g_encoder_pins[0][0])
        || (BOARD_ENC_L_B != g_encoder_pins[0][1])
        || (BOARD_ENC_R_A != g_encoder_pins[1][0])
        || (BOARD_ENC_R_B != g_encoder_pins[1][1]))
    {
        return false;
    }

    if (E_OK != tk_def_int(3U, &definition))
    {
        return false;
    }
#ifdef ROBO_BATTERY_FORWARD_TEST
    if (!gb_button_irq_registered)
    {
        if (!board_gpio_irq_register((1UL << 20U) | (1UL << 21U), button_irq))
        {
            return false;
        }

        gb_button_irq_registered = true;
    }

    gpio_acknowledge_irq(20U, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_acknowledge_irq(21U, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_set_irq_enabled(20U, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(21U, GPIO_IRQ_EDGE_FALL, true);
#endif
    g_motor_sign[0] = config.motor_sign[0];
    g_motor_sign[1] = config.motor_sign[1];
    /* 125 MHz / 1.5625 / 4000 = 20 kHz; 1000/4000 is exactly 25%. */
    pwm_config_set_wrap(&pwm, 3999U);
    pwm_config_set_clkdiv(&pwm, 1.5625f);
    pwm_init(4U, &pwm, false);
    pwm_init(5U, &pwm, false);
    set_w(TIMER_INTE, 1U << 3U);
    /* Highest available priority; no console work in the ISR. */
    EnableInt(3U, 0);
    gb_ready = true;
    return true;
}

/*! @brief Start a timed test of one or both wheels. */
static bool
start_test (unsigned wheel_mask, int direction, bool b_sensor,
            uint32_t duration_us)
{
    uint32_t irq_state = save_and_disable_interrupts();
    uint64_t now       = time_us_64();
    unsigned wheel;
#ifdef ROBO_BATTERY_FORWARD_TEST
    if (!gpio_get(21U))
    { /* Raw stop level also blocks explicit serial starts. */
        restore_interrupts(irq_state);
        return false;
    }
#endif
    if ((!gb_ready) || gb_active || gb_result_pending
        || (now < g_cooldown_until) || (0U == wheel_mask) || (wheel_mask > 3U)
        || ((1 != direction) && (-1 != direction)))
    {
        restore_interrupts(irq_state);
        /* Never queue another pulse or extend the running deadline. */
        return false;
    }

    stop_outputs();
    g_baseline = motion_hal_sample();
    g_completed.wheel
        = (3U == wheel_mask) ? 2U : ((1U == wheel_mask) ? 0U : 1U);
    g_completed.direction = direction;
    gb_active             = true;
    gb_sensor_forward     = b_sensor;
#ifdef ROBO_BATTERY_FORWARD_TEST
    gb_stop_release_seen = gpio_get(20U);
    g_stop_released_at   = now;
#endif
    /* Arm the serial cutoff or sensor watchdog BEFORE enabling PWM. */
    g_cutoff_at = time_us_64() + duration_us;
    out_w(TIMER_ALARM3, (uint32_t)g_cutoff_at);

    for (wheel = 0U; wheel < MOTION_WHEELS; ++wheel)
    {
        unsigned slice                = 4U + wheel;
        int      electrical_direction = direction * g_motor_sign[wheel];
        /* Ground-test setting: serial pairs/button runs use L=24.9%, R=25%.
         * Individual and automatic IR tests retain their original 25% duty. */
        uint16_t duty
            = (uint16_t)(((3U == wheel_mask) && (!b_sensor) && (0U == wheel))
                             ? PAIR_LEFT_DUTY
                             : TEST_DUTY);

        if (0U == (wheel_mask & (1U << wheel)))
        {
            /* Nothing to do. */
        }
        else
        {
            gpio_set_function(g_motor_pins[wheel][0], GPIO_FUNC_PWM);
            gpio_set_function(g_motor_pins[wheel][1], GPIO_FUNC_PWM);
            pwm_set_both_levels(slice, (electrical_direction > 0) ? duty : 0U,
                                (electrical_direction < 0) ? duty : 0U);
            pwm_set_enabled(slice, true);
        }
    }

    restore_interrupts(irq_state);
    return true;
}

/*! @brief Start one wheel. */
bool
motor_test_start (unsigned wheel, int direction)
{
    bool b_result = false;

    if (wheel > 1U)
    {
        b_result = false;
    }
    else
    {
        b_result = start_test(1U << wheel, direction, false, TEST_DURATION_US);
    }

    return b_result;
}

/*! @brief Start both wheels. */
bool
motor_test_start_pair (int direction)
{
    return start_test(3U, direction, false, TEST_DURATION_US);
}

#ifdef ROBO_BATTERY_FORWARD_TEST
void
motor_test_poll_buttons (void)
{
    motor_test_poll();
    uint64_t now = time_us_64();

    ++g_button_polls;

    if (!gb_ready)
    {
        return;
    }

    if (!gpio_get(21U))
    { /* Stop wins immediately on the next 20 ms poll. */
        disarm_buttons();
        button_stop_now();
        return;
    }

    if (gpio_get(20U))
    {
        gb_press_tracking = false;

        if (!gb_release_tracking)
        {
            g_released_since    = now;
            gb_release_tracking = true;
        }

        if ((now - g_released_since) >= 40000U)
        {
            gb_button_armed = true;
        }

        return;
    }

    gb_release_tracking = false;

    if (!gb_button_armed)
    {
        return;
    }

    if (gb_active)
    {
        /* GPIO interrupt also handles stop between task polls. */
        button_stop_now();
        return;
    }

    if (!gb_press_tracking)
    {
        g_pressed_since   = now;
        gb_press_tracking = true;
        return;
    }

    if ((now - g_pressed_since) >= 40000U)
    {
        disarm_buttons();
        (void)start_test(3U, 1, false, BUTTON_DURATION_US);
    }
}

bool
motor_test_get_buttons (motor_test_button_diag_t * p_delta)
{
    uint32_t irq_state;

    bool b_result = false;

    if (NULL == p_delta)
    {
        b_result = false;
    }
    else
    {
        irq_state = save_and_disable_interrupts();
        *p_delta  = (motor_test_button_diag_t){
            .start_high     = gpio_get(20U),
            .stop_high      = gpio_get(21U),
            .ready          = gb_ready,
            .armed          = gb_button_armed,
            .active         = gb_active,
            .polls          = g_button_polls,
            .stop_actions   = g_button_stop_actions,
            .irq_registered = gb_button_irq_registered,
            .irq_events     = g_button_irq_events};
        p_delta->cutoff_irq_count = g_cutoff_irq_count;
        p_delta->polled_cutoffs   = g_polled_cutoffs;
        uint64_t now              = time_us_64();

        p_delta->remaining_ms
            = (gb_active && (g_cutoff_at > now))
                  ? (uint32_t)((g_cutoff_at - now + 999U) / 1000U)
                  : 0U;
        restore_interrupts(irq_state);
        b_result = true;
    }

    return b_result;
}
#endif

bool
motor_test_sensor_forward (void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    bool     b_accepted;

    if (gb_sensor_paused)
    {
        b_accepted = false;
    }
    else if (gb_ready && gb_active && gb_sensor_forward)
    {
        /* Fresh sensor readings extend the watchdog without touching PWM.
         * Normal 100 ms updates therefore drive continuously, with no pauses.
         */
        g_cutoff_at = time_us_64() + SENSOR_WATCHDOG_US;
        out_w(TIMER_ALARM3, (uint32_t)g_cutoff_at);
        b_accepted = true;
    }
    else
    {
        b_accepted = start_test(3U, 1, true, SENSOR_WATCHDOG_US);
    }

    restore_interrupts(irq_state);
    return b_accepted;
}

void
motor_test_pause_sensor (void)
{
    uint32_t irq_state = save_and_disable_interrupts();

    gb_sensor_paused = true;

    if (gb_active && gb_sensor_forward)
    {
        finish(false);
    }

    restore_interrupts(irq_state);
}

/*! @brief Stop the test. */
void
motor_test_stop (void)
{
    uint32_t irq_state = save_and_disable_interrupts();

    finish(false);
    restore_interrupts(irq_state);
}

/*! @brief True while a test runs. */
bool
motor_test_active (void)
{
    return gb_active;
}

/*! @brief Stop and allow no more tests. */
void
motor_test_shutdown (void)
{
    uint32_t irq_state = save_and_disable_interrupts();

    /* Block both serial and sensor entry points until reboot. */
    gb_ready = false;
    finish(false);
    restore_interrupts(irq_state);
}

/*! @brief Take the result of the last test. False: none. */
bool
motor_test_result (motor_test_result_t * p_result)
{
    uint32_t irq_state   = save_and_disable_interrupts();
    bool     b_available = gb_result_pending;

    if (b_available)
    {
        *p_result         = g_completed;
        gb_result_pending = false;
    }

    restore_interrupts(irq_state);
    return b_available;
}
#endif

/*** end of file ***/
