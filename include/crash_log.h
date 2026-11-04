#ifndef ROBO_CRASH_LOG_H
#define ROBO_CRASH_LOG_H
/* Crash log that survives a reboot (RP2040). A hard fault cannot reach the USB
 * console (its service task never runs again), so the details are kept in the
 * watchdog scratch registers, the chip reboots, and usermain() reports them.
 * Scratch 0-3 survive a watchdog reset and are cleared by power-on; the
 * bootrom owns scratch 4-7. The watchdog tick is already 1 MHz (hw_setting.c). */
#include <tk/tkernel.h>

#define CRASH_WDT_CTRL      0x40058000U
#define CRASH_WDT_LOAD      0x40058004U
#define CRASH_WDT_REASON    0x40058008U
#define CRASH_WDT_SCRATCH0  0x4005800cU
#define CRASH_PSM_WDSEL     0x40010008U
#define CRASH_CTRL_TRIGGER  (1UL << 31U)
#define CRASH_CTRL_ENABLE   (1UL << 30U)
#define CRASH_CTRL_PAUSE    (7UL << 24U)  /* Pause while a debugger halts the cores. */
#define CRASH_REASON_TIMER  1U
#define CRASH_MAGIC_FAULT   0xC0FFEE01U
/* REASON.TIMER alone is not ours: the bootrom also reboots through a watchdog
 * timeout after a UF2 copy or picotool load. Only a timeout with this mark in
 * scratch 0 (set when our watchdog is armed) is a real freeze. */
#define CRASH_MAGIC_ARMED   0xC0FFEE02U
/* The freeze catcher (app_main.c) saw no watchdog feed for CRASH_FREEZE_LIMIT_US:
 * scratch 1/2 = interrupted pc/lr, scratch 3 = exception number << 24 (0 = task
 * code) | task id << 16 | motion stage << 8 | obstacle stage.
 * Task 255 in either fault report means scheduler lookup did not finish. */
#define CRASH_MAGIC_FREEZE  0xC0FFEE03U
#define CRASH_FREEZE_LIMIT_US 800000U  /* Before the 1 s hardware watchdog. */
#define CRASH_FREEZE_CHECK_US 50000U
/* While armed, scratch 1-3 say where the code was when a timeout hit (a hard
 * fault overwrites them with pc/lr/task). Stage values: CRASH_MOTION_* and
 * CRASH_OBSTACLE_*; the time is ms since boot of the last watchdog feed. */
#define CRASH_MARK_MOTION   (CRASH_WDT_SCRATCH0 + 4U)
#define CRASH_MARK_OBSTACLE (CRASH_WDT_SCRATCH0 + 8U)
#define CRASH_MARK_FEED_MS  (CRASH_WDT_SCRATCH0 + 12U)
enum { CRASH_MOTION_INPUTS = 1, CRASH_MOTION_STEP, CRASH_MOTION_OUTPUT, CRASH_MOTION_SLEEP };
enum { CRASH_OBSTACLE_PING = 1, CRASH_OBSTACLE_DECIDE, CRASH_OBSTACLE_STOP, CRASH_OBSTACLE_SETTLE,
       CRASH_OBSTACLE_SCAN, CRASH_OBSTACLE_REPORT, CRASH_OBSTACLE_DETOUR,
       CRASH_OBSTACLE_OUT, CRASH_OBSTACLE_PASS, CRASH_OBSTACLE_REAR_CLEAR,
       CRASH_OBSTACLE_RETURN_TURN, CRASH_OBSTACLE_RETURN_LOOK,
       CRASH_OBSTACLE_RETURN_DRIVE, CRASH_OBSTACLE_ALIGN };
static inline void crash_mark(UW mark, UW value) { out_w(mark, value); }

/* RP2040-E1: the counter decrements twice per tick. */
static inline void crash_watchdog_feed(UW timeout_ms) { out_w(CRASH_WDT_LOAD, timeout_ms * 2000U); }
static inline void crash_watchdog_start(UW timeout_ms)
{
    out_w(CRASH_WDT_SCRATCH0, CRASH_MAGIC_ARMED);
    crash_mark(CRASH_MARK_MOTION, 0U);
    crash_mark(CRASH_MARK_OBSTACLE, 0U);
    crash_mark(CRASH_MARK_FEED_MS, 0U);
    crash_watchdog_feed(timeout_ms);
    out_w(CRASH_PSM_WDSEL, 0x0001fffcU); /* A timeout resets everything except the oscillators. */
    out_w(CRASH_WDT_CTRL, CRASH_CTRL_ENABLE | CRASH_CTRL_PAUSE);
}
/* Watchdog registers survive their own reset: stop it before anything else. */
static inline void crash_watchdog_stop(void) { out_w(CRASH_WDT_CTRL, 0U); }

/* Save first: anything after this in a fault handler may hang, and the watchdog
 * timeout then still reboots with the details kept. */
static inline void crash_log_save(UW magic, UW pc, UW lr, UW task)
{
    out_w(CRASH_WDT_SCRATCH0, magic);
    out_w(CRASH_WDT_SCRATCH0 + 4U, pc);
    out_w(CRASH_WDT_SCRATCH0 + 8U, lr);
    out_w(CRASH_WDT_SCRATCH0 + 12U, task);
}
static inline void crash_log_reboot(void)
{
    out_w(CRASH_PSM_WDSEL, 0x0001fffcU); /* Reset everything except the oscillators. */
    out_w(CRASH_WDT_CTRL, CRASH_CTRL_TRIGGER);
    for (;;) { }
}
#endif
