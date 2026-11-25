/* Mock the watchdog registers to verify crash evidence survives normal rearming. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t in_w(uint32_t address);
void out_w(uint32_t address, uint32_t value);
#include "crash_log.h"
#include "motion.h"
#include "board_config.h"
#include "robot.h"

static uint32_t registers[7], reset_mask;
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)

uint32_t in_w(uint32_t address)
{
    CHECK(address >= CRASH_WDT_CTRL && address <= CRASH_WDT_SCRATCH0 + 12U);
    CHECK((address - CRASH_WDT_CTRL) % 4U == 0U);
    return registers[(address - CRASH_WDT_CTRL) / 4U];
}

void out_w(uint32_t address, uint32_t value)
{
    if (address == CRASH_PSM_WDSEL) { reset_mask = value; return; }
    CHECK(address >= CRASH_WDT_CTRL && address <= CRASH_WDT_SCRATCH0 + 12U);
    CHECK((address - CRASH_WDT_CTRL) % 4U == 0U);
    registers[(address - CRASH_WDT_CTRL) / 4U] = value;
}

static void encoder_latch_tests(void)
{
    memset(registers, 0, sizeof(registers));
    crash_watchdog_start(1000U);
    crash_encoder_fault_update(TRUE, FB_NO_PROGRESS);
    out_w(CRASH_WDT_REASON, CRASH_REASON_TIMER);
    const uint32_t causes[] = {CRASH_MAGIC_ARMED, CRASH_MAGIC_FREEZE, CRASH_MAGIC_FAULT};
    for (unsigned i = 0U; i < sizeof(causes) / sizeof(causes[0]); ++i) {
        crash_log_save(causes[i], 123U, 456U, 789U);
        crash_snapshot_t saved;
        crash_log_capture(&saved);
        CHECK(crash_log_present(&saved) && saved.magic == causes[i]);
        CHECK(saved.encoder_latched && saved.encoder_cause == FB_NO_PROGRESS);
        crash_watchdog_start(1000U);
        /* A restored latch must not overwrite the original observed cause. */
        crash_encoder_fault_update(TRUE, FB_RESTART_LATCH);
        crash_log_capture(&saved);
        CHECK(saved.encoder_latched && saved.encoder_cause == FB_NO_PROGRESS);
    }

    crash_snapshot_t saved;
    crash_log_capture(&saved);
    motion_config_t config = board_motion_config();
    motion_t motion;
    encoder_sample_t sample = {{0U, 0U}, {0U, 0U}, true};
    double now = 0.0;
    motion_init(&motion, &config, sample, now, true);
    CHECK(saved.encoder_latched);
    motion_restore_encoder_fault(&motion, sample, now);
    CHECK(motion.status.fault == F_FEEDBACK && motion.status.feedback_cause == FB_RESTART_LATCH);
    /* Quiet/reliable samples after reboot do not prove a formerly stalled or
     * disconnected encoder was repaired. Never reach READY automatically. */
    for (unsigned i = 0U; i < 100U; ++i) {
        now += config.period;
        motion_step(&motion, sample, now);
        CHECK(motion.status.state == FAULT && motion.status.hold && motion.status.inhibited);
        CHECK(!motion.status.feedback_trusted && !motion.status.stationary);
        CHECK(motion.status.output[0] == 0.0 && motion.status.output[1] == 0.0);
    }
    CHECK(!motion_release(&motion, true));
    robot_bus_t bus;
    robot_bus_init(&bus, &motion.status);
    CHECK(vehicle_controller_remote_command(&bus, REMOTE_START) == R_HELD);
    robot_control(&bus, CONTROL_RELEASE, true, true);
    robot_motion_inputs(&bus, &motion, sample, now);
    CHECK(motion.status.state == FAULT && motion.status.hold);
    CHECK(!motion_reset(&motion, false, true, sample, now));
    CHECK(!motion_reset(&motion, true, false, sample, now));
    sample.reliable = false;
    CHECK(!motion_reset(&motion, true, true, sample, now));
    CHECK(motion.status.state == FAULT);
    crash_encoder_fault_update(TRUE, motion.status.feedback_cause);
    crash_log_capture(&saved);
    CHECK(saved.encoder_latched && saved.encoder_cause == FB_NO_PROGRESS);

    sample.reliable = true;
    CHECK(motion_reset(&motion, true, true, sample, now));
    CHECK(motion.status.state == INITIALISING && motion.status.hold);
    crash_encoder_fault_update((motion.fault_flags & (1U << ((unsigned)F_FEEDBACK - 1U))) != 0U,
                               motion.status.feedback_cause);
    crash_log_capture(&saved);
    CHECK(!saved.encoder_latched);
    crash_watchdog_start(1000U);
    crash_log_capture(&saved);
    CHECK(!saved.encoder_latched); /* Accepted manual reset remains cleared. */
    for (unsigned i = 0U; i < 40U; ++i) { now += config.period; motion_step(&motion, sample, now); }
    CHECK(motion.status.state == STOPPED && motion.status.stationary);
    CHECK(!motion_release(&motion, false));
    CHECK(motion_release(&motion, true));
    CHECK(motion.status.state == READY);

    /* A real encoder error following manual reset must latch again. */
    ++sample.invalid[0];
    now += config.period;
    motion_step(&motion, sample, now);
    CHECK(motion.status.state == FAULT && motion.status.feedback_cause == FB_INVALID_TRANSITION);
    crash_encoder_fault_update(TRUE, motion.status.feedback_cause);
    crash_watchdog_start(1000U);
    crash_log_capture(&saved);
    CHECK(saved.encoder_latched && saved.encoder_cause == FB_INVALID_TRANSITION);
}

int main(void)
{
    const uint32_t causes[] = {CRASH_MAGIC_FAULT, CRASH_MAGIC_FREEZE, CRASH_MAGIC_ARMED};
    for (unsigned i = 0U; i < sizeof(causes) / sizeof(causes[0]); ++i) {
        memset(registers, 0, sizeof(registers));
        out_w(CRASH_WDT_CTRL, CRASH_CTRL_ENABLE);
        out_w(CRASH_WDT_REASON, CRASH_REASON_TIMER);
        crash_log_save(causes[i], 0x12345678U, 0x87654321U, 0x1d020304U);
        crash_watchdog_stop();
        crash_snapshot_t saved;
        crash_log_capture(&saved);
        CHECK(in_w(CRASH_WDT_CTRL) == 0U && crash_log_present(&saved));
        CHECK(saved.magic == causes[i] && saved.data[0] == 0x12345678U);
        CHECK(saved.data[1] == 0x87654321U && saved.data[2] == 0x1d020304U);
        crash_snapshot_t original = saved;

        /* Startup can rearm protection immediately; reconnecting USB later
         * still gets the previous crash, not the new watchdog's stage markers. */
        crash_watchdog_start(1000U);
        CHECK(in_w(CRASH_WDT_CTRL) == (CRASH_CTRL_ENABLE | CRASH_CTRL_PAUSE));
        CHECK(in_w(CRASH_WDT_LOAD) == 2000000U && reset_mask == 0x0001fffcU);
        CHECK(in_w(CRASH_WDT_SCRATCH0) == CRASH_MAGIC_ARMED);
        CHECK(in_w(CRASH_MARK_MOTION) == 0U && in_w(CRASH_MARK_OBSTACLE) == 0U);
        crash_mark(CRASH_MARK_MOTION, CRASH_MOTION_STEP);
        crash_watchdog_feed(1000U);
        CHECK(memcmp(&original, &saved, sizeof(saved)) == 0);
        CHECK(crash_log_present(&saved));
    }

    crash_snapshot_t normal = {0};
    CHECK(!crash_log_present(&normal));
    normal.reason = CRASH_REASON_TIMER; /* Bootloader/UF2 watchdog reset. */
    CHECK(!crash_log_present(&normal));
    normal.magic = CRASH_MAGIC_ARMED;
    normal.reason = 0U; /* Armed marker alone does not prove watchdog expiry. */
    CHECK(!crash_log_present(&normal));
    encoder_latch_tests();
    puts("PASS: watchdog crash evidence and encoder faults survive restart; checked manual recovery required");
    return 0;
}
