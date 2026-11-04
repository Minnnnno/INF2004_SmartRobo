/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.0 BSP
 *
 *    Copyright (C) 2021-2022 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2022/11.
 *
 *----------------------------------------------------------------------
 */

#include <sys/machine.h>
#ifdef CPU_CORE_ARMV6M

/*
 *	exc_hdr.c (ARMv6-M)
 *	Exception handler
 */

#include <tk/tkernel.h>
#include <kernel.h>
#include "../../../sysdepend.h"
#include <crash_log.h>

/* Never format or transmit console output in exception context. USB needs
 * its service task, and UART output can busy-wait. Hard faults save the
 * register snapshot and reboot; usermain prints it in task context. */

/*
 * NMI handler
 */
WEAK_FUNC EXPORT void NMI_Handler(void)
{
	while(1);
}

/*
 * Hard fault handler
 */
/*
 * Hard fault reporting.
 *
 * Three separate hypotheses about a fault in this port were wrong, each
 * costing a flash. The exception frame holds the address that faulted, so
 * capture it instead of guessing: a naked entry stub records the frame
 * pointer and tail-calls the reporter.
 *
 * ARMv6-M stacks r0-r3, r12, lr, pc, xpsr. Bit 2 of EXC_RETURN selects which
 * stack was in use, and this kernel runs everything on MSP, but both are
 * handled so the report stays correct if that ever changes.
 */
EXPORT volatile UW *knl_fault_frame;

IMPORT void knl_hard_fault_report(void);

__asm__(
"	.syntax unified			\n"
"	.thumb				\n"
"	.text				\n"
"	.align	2			\n"
"	.globl	HardFault_Handler	\n"
"	.thumb_func			\n"
"HardFault_Handler:			\n"
"	mov	r0, lr			\n"
"	movs	r1, #4			\n"
"	tst	r0, r1			\n"
"	beq	1f			\n"
"	mrs	r0, psp			\n"
"	b	2f			\n"
"1:	mrs	r0, msp			\n"
"2:	ldr	r1, =knl_fault_frame	\n"
"	str	r0, [r1]		\n"
"	b	knl_hard_fault_report	\n"
"	.ltorg				\n"
);

EXPORT void knl_hard_fault_report(void)
{
    extern void robo_board_force_stop(void);
    robo_board_force_stop();

	ID		ctskid;
	volatile UW	*f = knl_fault_frame;

	/* Preserve PC/LR even if the fault also damaged the scheduler's TCB
	   pointer. Task 255 means its identity could not be recovered. */
	crash_log_save(CRASH_MAGIC_FAULT, (f != NULL)? f[6]: 0U,
		       (f != NULL)? f[5]: 0U, 255U);
	if(knl_ctxtsk != NULL) {
		ctskid = knl_ctxtsk->tskid;
	} else {
		ctskid = 0;
	}
	out_w(CRASH_WDT_SCRATCH0 + 12U, (UW)ctskid);
	/* The USB console cannot drain from here: reboot and let usermain()
	   report the saved details. */
	crash_log_reboot();
}

/*
 * Svcall
 */
WEAK_FUNC EXPORT void Svcall_Handler(void)
{
	while(1);
}

/*
 * Default Handler
 */
WEAK_FUNC EXPORT void Default_Handler(void)
{
	while(1);
}

#endif /* CPU_CORE_ARMV6M */
