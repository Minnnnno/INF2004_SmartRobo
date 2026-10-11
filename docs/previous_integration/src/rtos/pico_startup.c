/* Narrow startup adaptation of the official BSP to Pico SDK CRT.
 * No scheduler/syscall emulation. Core 1 must remain unused. */
#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/structs/scb.h"
#include "hardware/structs/m0plus.h"
#include "hardware/sync.h"
#include "motion_hal.h"

extern int robo_kernel_main(void);
extern void knl_dispatch_entry(void);
extern void knl_systim_inthdr(void);
extern void Svcall_Handler(void);
extern uint32_t exchdr_tbl[];
static uint64_t kernel_heap[12288]; /* 96 KiB, disjoint from SDK heap/stack. */
void *knl_lowmem_top = kernel_heap;
void *knl_lowmem_limit = kernel_heap + 12288;

void knl_shutdown_hw(void)
{
    (void)save_and_disable_interrupts();
    motion_hal_emergency_stop();
    for (;;) { tight_loop_contents(); }
}
int knl_restart_hw(int mode) { (void)mode; return -9; /* E_NOSPT */ }
void isr_hardfault(void) { knl_shutdown_hw(); }

int main(void)
{
    unsigned i;
    uint32_t *vectors;
    /* BSP TMCLK is 125 MHz: keep SDK and RTOS clocks in agreement. */
    set_sys_clock_khz(125000U, true);
    motion_hal_emergency_stop();
    (void)save_and_disable_interrupts();
    vectors = (uint32_t *)(uintptr_t)scb_hw->vtor;
    /* Keep SDK RAM vectors active (correct alignment and SDK GPIO dispatcher).
     * Copy for tk_def_int bookkeeping. This application uses SDK IRQ registration only. */
    for (i = 0; i < 48U; ++i) { exchdr_tbl[i] = vectors[i]; }
    vectors[11] = (uint32_t)(uintptr_t)Svcall_Handler;
    vectors[14] = (uint32_t)(uintptr_t)knl_dispatch_entry;
    vectors[15] = (uint32_t)(uintptr_t)knl_systim_inthdr;
    ppb_hw->shpr[0] = 0U;
    ppb_hw->shpr[1] = 0x00c00000U;
    return robo_kernel_main();
}
