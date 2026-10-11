/* Host-only kernel declarations. No hardware driver runs in these tests. */
#ifndef HOST_KERNEL_STUB_H
#define HOST_KERNEL_STUB_H

#include <stdint.h>
#include <stddef.h>

/* Skip app.h's board-specific includes when testing the real subsystem.c. */
#define APP_H
#define IMPORT extern
typedef int32_t BOOL;
#define EXPORT
#define E_OK      (0)
#define E_IO      (-57)
#define E_TMOUT   (-50)
#define E_NOEXS   (-42)
#define E_NOMEM   (-10)
#define E_PAR     (-17)
#define E_BUSY    (-65)
#define TD_UPDATE (0x0003U)
#define TD_EXCL   (0x0100U)
#define TA_TFIFO  (0x0000U)
#define TA_WMUL   (0x0008U)
#define TA_HLNG   (0x0001U)

typedef int32_t ER;
typedef int32_t ID;
typedef int32_t INT;
typedef int32_t PRI;
typedef int32_t SZ;
typedef uint32_t UW;
typedef uint8_t UB;
typedef void (*FP)(void);
typedef struct { uint32_t flgatr; uint32_t iflgptn; } T_CFLG;
typedef struct { uint32_t mbfatr; int32_t bufsz; int32_t maxmsz; } T_CMBF;
typedef struct { uint32_t almatr; FP almhdr; } T_CALM;
typedef struct
{
    UW sadr;
    SZ snd_size;
    UB *snd_data;
    SZ rcv_size;
    UB *rcv_data;
} T_I2C_EXEC;
#define TDN_I2C_EXEC (-100)

ID tk_opn_dev(UB const *p_name, uint32_t mode);
ER tk_cls_dev(ID h_device, uint32_t option);
ER i2c_read_reg(ID h_device, UW address, UW reg, UB *p_data);
ER i2c_write_reg(ID h_device, UW address, UW reg, UB value);
ER tk_swri_dev(ID h_device, int32_t start, void const *p_buffer,
               SZ size, SZ *p_actual_size);
ID tk_cre_flg(T_CFLG const *p_configuration);
ID tk_cre_mbf(T_CMBF const *p_configuration);
ID tk_cre_alm(T_CALM const *p_configuration);
ID app_start_task(FP entry, PRI priority, UW core, char const *p_name);
ER subsystem4_init(void);
ER subsystem4_start(void);

ER tk_dly_tsk(UW delay);
#endif /* HOST_KERNEL_STUB_H */
