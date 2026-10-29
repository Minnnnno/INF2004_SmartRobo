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

/* RP2040-E1: the counter decrements twice per tick. */
static inline void crash_watchdog_feed(UW timeout_ms) { out_w(CRASH_WDT_LOAD, timeout_ms * 2000U); }
static inline void crash_watchdog_start(UW timeout_ms)
{
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
