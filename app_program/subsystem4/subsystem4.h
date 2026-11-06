/* Public interface for Buddy 4's sensor configuration stage. */
#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include <tk/tkernel.h>

/*
 * Configure an LSM303DLHC-based GY-511 using the kernel I2C1 driver.
 * Call in task context after device initialisation, before any sampling task.
 * Returns E_OK only when identity, all register readbacks and close succeed.
 * Identity/readback mismatch returns E_IO; driver errors are passed through.
 * One attempt only. Failure may leave partially configured sensor registers.
 * This does not calibrate, read measurements, or establish measurement validity.
 */
ER subsystem4_init(void);

#endif /* SUBSYSTEM4_H */
