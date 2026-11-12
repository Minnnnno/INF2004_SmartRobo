/* Shared watchdog/crash handling and low-priority system supervision. */
#include "system_runtime.h"
#include "../app.h"
#include "robot.h"
#include "motion_hal.h"
#include "crash_log.h"
#include "stack_debug.h"

static crash_snapshot_t previous_crash;

/* The private idle context also carries interrupts on this MSP-based port. */
extern UW knl_idle_stack_used(UINT core);
extern UW knl_idle_stack_size(void);
extern BOOL knl_idle_stack_intact(UINT core);
extern BOOL knl_guards_intact(void);

/* Freeze catcher: TIMER alarm 2, routed to the NMI, checks that motion_task still
 * feeds the watchdog. A stall is recorded with the code the CPU was running before
 * the hardware watchdog fires. As an NMI it also sees code that masked interrupts,
 * a handler spinning at priority 0 (Default_Handler is while(1)) and a hung hard
 * fault handler. A plain watchdog timeout means no NMI report was saved;
 * it does not distinguish a core lockup from failure of the catcher itself. */
#define FREEZE_ALARM 2U
#define SYSCFG_PROC0_NMI_MASK 0x40004000U
#define EXC_NMI 2U
static volatile UW last_feed_us;
static volatile UW freeze_check_count;
extern UW exchdr_tbl[]; /* Kernel's RAM vector table; VTOR points here. */
extern void robo_freeze_entry(void);
void robo_freeze_check(const UW *frame);

/* NMI entry: the exception frame is at the stack pointer (tasks and handlers
 * both run on MSP here). Returns through the C function with EXC_RETURN still
 * in lr. No kernel services: it can interrupt the kernel anywhere. */
__asm__(
"	.syntax	unified			\n"
"	.thumb				\n"
"	.text				\n"
"	.align	2			\n"
"	.globl	robo_freeze_entry	\n"
"	.thumb_func			\n"
"robo_freeze_entry:			\n"
"	mov	r0, lr			\n"
"	movs	r1, #4			\n"
"	tst	r0, r1			\n"
"	beq	1f			\n"
"	mrs	r0, psp			\n"
"	b	2f			\n"
"1:	mrs	r0, msp			\n"
"2:	ldr	r1, =robo_freeze_check	\n"
"	bx	r1			\n"
"	.ltorg				\n"
);

__attribute__((used)) void robo_freeze_check(const UW *frame)
{
    UW now = in_w(TIMER_TIMERAWL);
    UW where;
    ++freeze_check_count;
    out_w(TIMER_INTR, 1UL << FREEZE_ALARM);
    out_w(TIMER_ALARM2, now + CRASH_FREEZE_CHECK_US);
    if ((now - last_feed_us) < CRASH_FREEZE_LIMIT_US) { return; }
    /* Save the frame before consulting scheduler memory: stack corruption can
     * invalidate its current-TCB pointer and fault even inside this NMI.
     * Task 255 means the lookup did not finish; the watchdog still keeps PC/LR. */
    where = ((frame[7] & 0x3fU) << 24U) | (0xffU << 16U) |
            ((in_w(CRASH_MARK_MOTION) & 0xffU) << 8U) | (in_w(CRASH_MARK_OBSTACLE) & 0xffU);
    crash_log_save(CRASH_MAGIC_FREEZE, frame[6], frame[5], where);
    where = (where & ~(0xffU << 16U)) | (((UW)tk_get_tid() & 0xffU) << 16U);
    out_w(CRASH_WDT_SCRATCH0 + 12U, where);
    crash_log_reboot();
}

static void freeze_catcher_start(void)
{
    UINT state;
    /* The kernel leaves SYSCFG, which holds the NMI routing, in reset. */
    clr_w(RESETS_RESET, RESETS_RESET_SYSCFG);
    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_SYSCFG) == 0U) { }
    DI(state);
    last_feed_us = in_w(TIMER_TIMERAWL);
    exchdr_tbl[EXC_NMI] = (UW)robo_freeze_entry;
    out_w(TIMER_INTR, 1UL << FREEZE_ALARM);
    set_w(TIMER_INTE, 1UL << FREEZE_ALARM);
    out_w(TIMER_ALARM2, last_feed_us + CRASH_FREEZE_CHECK_US);
    /* NMI only: the IRQ stays disabled in the NVIC. */
    set_w(SYSCFG_PROC0_NMI_MASK, 1UL << FREEZE_ALARM);
    EI(state);
}


void system_watchdog_start(void)
{
    crash_watchdog_start(1000U); /* Reboot and report if this loop stalls for 1 s. */
    freeze_catcher_start();
}

void system_watchdog_feed(double now)
{
    crash_watchdog_feed(1000U);
    last_feed_us = in_w(TIMER_TIMERAWL);
    crash_mark(CRASH_MARK_FEED_MS, (UW)(now * 1000.0));
}

void system_report_previous_crash(void)
{
    if (!crash_log_present(&previous_crash)) { return; }
    const UW *data = previous_crash.data;
    if (previous_crash.magic == CRASH_MAGIC_FAULT) {
        tm_printf((const UB *)"LAST CRASH: hard fault task=%u pc=%08x lr=%08x\n",
                  (unsigned)data[2], (unsigned)data[0], (unsigned)data[1]);
    } else if (previous_crash.magic == CRASH_MAGIC_FREEZE) {
        UW where = data[2];
        tm_printf((const UB *)"LAST CRASH: froze, motion not run for 0.8 s "
                  "pc=%08x lr=%08x exception=%u task=%u motion_stage=%u obstacle_stage=%u\n",
                  (unsigned)data[0], (unsigned)data[1], (unsigned)(where >> 24U),
                  (unsigned)((where >> 16U) & 0xffU), (unsigned)((where >> 8U) & 0xffU),
                  (unsigned)(where & 0xffU));
    } else {
        tm_printf((const UB *)"LAST CRASH: watchdog timeout, code froze >1 s "
                  "motion_stage=%u obstacle_stage=%u last_feed_ms=%u\n",
                  (unsigned)data[0], (unsigned)data[1], (unsigned)data[2]);
    }
    tm_printf((const UB *)"[RESTART] motor initialization retried; startup checks and any retained encoder fault must clear before movement.\n");
}

void system_boot_init(void)
{
    crash_watchdog_stop();
    crash_log_capture(&previous_crash);
    system_report_previous_crash();
}

bool system_encoder_fault_latched(void) { return previous_crash.encoder_latched; }
UW system_encoder_fault_cause(void) { return previous_crash.encoder_cause; }

void system_report_startup(void)
{
#ifdef ROBO_PRODUCTION_BUILD
    tm_printf((const UB *)"[STACK] idle_capacity=%u bytes (interrupts share the idle stack)\n",
              (unsigned)knl_idle_stack_size());
#endif
}

#ifdef ROBO_PRODUCTION_BUILD
static bool report_task_stack(ID id, const char *name)
{
    stack_debug_t info;
    ER result = knl_task_stack_debug(id, &info);
    if (result == E_NOEXS) { return true; }
    if (result != E_OK) {
        tm_printf((const UB *)"[MEMORY] task=%u name=%s stack metadata invalid error=%d\n",
                  (unsigned)id, name, (int)result);
        return false;
    }
    tm_printf((const UB *)"[STACK] task=%u name=%s used=%u capacity=%u guard_ok=%u saved_sp_ok=%u\n",
              (unsigned)id, name, (unsigned)info.used, (unsigned)info.capacity,
              (unsigned)info.guard_ok, (unsigned)info.saved_sp_ok);
    return info.guard_ok && info.saved_sp_ok;
}
#endif

void system_supervise(const app_task_spec_t *tasks, const ID *ids, unsigned task_count,
                      void (*report_motion_fault)(void))
{
#ifdef ROBO_PRODUCTION_BUILD
    unsigned i;
#else
    (void)tasks; (void)ids; (void)task_count; (void)report_motion_fault;
#endif
    bool memory_ok = knl_idle_stack_intact(0U) && knl_guards_intact();
    if (!memory_ok) {
        motion_hal_emergency_stop();
        motion_request_shutdown();
        tm_printf((const UB *)"[MEMORY] idle stack or scheduler guard damaged; motors shut down\n");
    }
#ifdef ROBO_PRODUCTION_BUILD
    /* Put the fault evidence ahead of routine stack/debug output, so it is
     * first in the bounded USB transmit queue. Printing stays off motion_task. */
    report_motion_fault();
    system_report_previous_crash();
    tm_printf((const UB *)"[STACK] idle_used=%u capacity=%u guard_ok=%u\n",
              (unsigned)knl_idle_stack_used(0U), (unsigned)knl_idle_stack_size(),
              (unsigned)knl_idle_stack_intact(0U));
    for (ID id = 1; id <= CNF_MAX_TSKID; ++id) {
        const char *name = id == tk_get_tid() ? "diagnostics" : "kernel";
        for (i = 0U; i < task_count; ++i) {
            if (ids[i] == id) { name = tasks[i].name; break; }
        }
        if (!report_task_stack(id, name)) {
            motion_hal_emergency_stop();
            motion_request_shutdown();
        }
    }
    {
        UW dropped = 0U;
#if TM_CONSOLE_USB_CDC
        extern volatile UW tm_usb_dropped;
        dropped = tm_usb_dropped;
#endif
        tm_printf((const UB *)"[DEBUG] firmware=%s\n", ROBO_ENCODER_BUILD_ID);
        tm_printf((const UB *)"[DEBUG] feed_age_ms=%u nmi_checks=%u usb_dropped=%u scheduler_guard=%u\n",
                  (unsigned)((in_w(TIMER_TIMERAWL) - last_feed_us) / 1000U),
                  (unsigned)freeze_check_count, (unsigned)dropped, (unsigned)knl_guards_intact());
    }
#endif
}
