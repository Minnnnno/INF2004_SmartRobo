/* Exercise the real read/packing code with simulated kernel transfers. */
#include "../../app_program/subsystem4/gy511_read.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static ER open_result, read_result, transfer_result, close_result;
static uint8_t ready, raw[6];
static unsigned opens, closes, transfers;
static int short_transfer;
static void check(int condition, char const *message)
{
    if (!condition) { fprintf(stderr, "FAIL: %s\n", message); exit(1); }
}
static void reset(void)
{
    open_result=7; read_result=E_OK; transfer_result=E_OK; close_result=E_OK;
    ready=8U; opens=0; closes=0; transfers=0; short_transfer=0;
    memset(raw, 0, sizeof(raw));
}
ID tk_opn_dev(UB const *name, uint32_t mode)
{
    check(strcmp((char const *)name,"iicb")==0, "bus");
    check(mode==(TD_UPDATE|TD_EXCL), "exclusive open"); opens++; return open_result;
}
ER tk_cls_dev(ID handle, uint32_t option)
{
    check(handle==7 && option==0U, "close handle"); closes++; return close_result;
}
ER i2c_read_reg(ID handle, UW address, UW reg, UB *data)
{
    check(handle==7 && address==0x19U && reg==0x27U, "status register only");
    *data=ready; return read_result;
}
ER tk_swri_dev(ID handle, int32_t start, void const *buffer, SZ size, SZ *actual)
{
    T_I2C_EXEC const *t=buffer;
    check(handle==7 && start==TDN_I2C_EXEC && size==(SZ)sizeof(*t), "descriptor");
    check(t->sadr==0x19U && t->snd_size==1 && t->snd_data[0]==0xA8U && t->rcv_size==6, "six-byte burst");
    transfers++; memcpy(t->rcv_data, raw, 6);
    *actual=short_transfer ? 0 : (SZ)sizeof(*t); return transfer_result;
}
static void expect_failure(ER expected)
{
    gy511_acceleration_t sample={{1,2,3,4,5}}, before=sample;
    check(gy511_read_acceleration(&sample)==expected, "error returned");
    check(memcmp(&sample,&before,sizeof(sample))==0, "failed read leaves output unchanged");
    check(closes==(open_result<0 ? 0U:1U), "cleanup");
}
int main(void)
{
    gy511_acceleration_t sample={{0}};
    int16_t decoded=123;
    reset(); check(gy511_read_acceleration(NULL)==E_PAR && opens==0U, "NULL read");
    reset(); open_result=E_NOEXS; expect_failure(E_NOEXS);
    reset(); read_result=E_TMOUT; expect_failure(E_TMOUT); check(transfers==0U,"no burst after error");
    reset(); ready=7U; expect_failure(E_BUSY); check(transfers==0U,"no burst without XYZ ready");
    reset(); ready=0U; close_result=E_IO; expect_failure(E_IO);
    reset(); transfer_result=E_TMOUT; expect_failure(E_TMOUT);
    reset(); short_transfer=1; expect_failure(E_IO);
    reset(); close_result=E_IO; expect_failure(E_IO);
    reset(); transfer_result=E_TMOUT; close_result=E_IO; expect_failure(E_TMOUT);
    check(gy511_acceleration_mg(NULL,0U,&decoded)==E_PAR,"NULL decode");
    check(gy511_acceleration_mg(&sample,3U,&decoded)==E_PAR && decoded==123,"invalid axis unchanged");
    check(gy511_acceleration_mg(&sample,0U,NULL)==E_PAR,"NULL destination");
    check(sizeof(sample)==5U,"packed sample size");
    /* Every possible 12-bit value on each axis; other axes use different values
       to detect nibble overlap. Nonzero unused input bits must be discarded. */
    for (uint32_t code=0U; code<4096U; code++)
    {
        reset();
        for (uint8_t axis=0U; axis<3U; axis++)
        {
            uint32_t value=(code + (uint32_t)axis*997U)&4095U;
            uint32_t word=(value<<4U)|15U;
            raw[2U*axis]=(uint8_t)(word&255U); raw[2U*axis+1U]=(uint8_t)(word>>8U);
        }
        check(gy511_read_acceleration(&sample)==E_OK,"successful read");
        check(closes==1U && transfers==1U && (sample.bytes[4]&0xF0U)==0U,"transfer and padding");
        for (uint8_t axis=0U; axis<3U; axis++)
        {
            int32_t expected=(int32_t)((code+(uint32_t)axis*997U)&4095U);
            if (expected>=2048) { expected-=4096; }
            check(gy511_acceleration_mg(&sample,axis,&decoded)==E_OK && decoded==expected,"signed XYZ round trip");
        }
    }
    puts("PASS: acceleration failure paths and 4096 XYZ vectors (12288 axis conversions)");
    return 0;
}
