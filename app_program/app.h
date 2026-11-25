/*
 *  app.h
 *  Definitions shared by the whole team.  Changed only by team agreement.
 *
 *  Anything that belongs to ONE subsystem goes in that subsystem's own
 *  header instead (app_program/subsystemN/subsystemN.h).
 */

#ifndef APP_H
#define APP_H

#include <tk/tkernel.h>  /* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h> /* T-Monitor API: tm_printf() console output */
#include <bsp/libbsp.h>  /* Board helpers: gpio_set_pin/val, pwm_... */

/* ---------------------------------------------------------------------
 *  Pins: free for projects are GP0..GP22 and GP26..GP28.  GP0/GP1 are
 *  taken by UART0 when built with "make CRASH_REPORT=1" (see README).
 *  Never use GP23/24/25/29: they are wired to the Pico W radio.
 *  Claim every pin in TEAM.md section 2 before using it.
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  Task priorities: 1 is the highest; the kernel's USB console (3) and
 *  WiFi (4) tasks sit above every application task.  Each subsystem
 *  has its own band (TEAM.md section 4), defined in its own header.
 * --------------------------------------------------------------------- */
#define TASK_STACK_SIZE 2048 /* bytes; tm_printf needs ~1 KB */

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in system/app_tasks.c. */
IMPORT ID app_start_task(FP entry, PRI pri, UW prc, const char * name);

/* ---------------------------------------------------------------------
 *  Subsystem hooks retained by the shared application interface.
 *  Buddy 1 creates its communications objects before tasks are started.
 *  Buddy 2 uses subsystem2/motion_runtime.h; its motion task is created
 *  by usermain() in app_main.c. Legacy on-target test declarations below
 *  are conditional; use CMake host tests for current software validation.
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);

IMPORT ER subsystem1_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem5_test(void);
#endif

/* Print a readable message if a kernel call failed (negative error code) */
#define CHECK(er, what)                                           \
    do                                                            \
    {                                                             \
        INT check_er_ = (INT)(er); /* evaluate once */            \
        if (check_er_ < E_OK)                                     \
        {                                                         \
            tm_printf((UB *)"ERROR %s: %d\n", (what), check_er_); \
        }                                                         \
    } while (0)

#endif /* APP_H */

/*** end of file ***/
