"""Compile the actual receive switch arm against simulated register accesses.

Extracting the arm avoids pulling the complete hardware/kernel port into host
builds. This verifies interrupt-side byte handling and queued commands only;
it does not emulate electrical STOP timing or the full driver state machine.
"""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[3]
source = (root / "device/i2c/sysdepend/rp2040/i2c_rp2040.c").read_text()
start = source.index("case I2C_STS_RECV:")
end = source.index("\n\tdefault:", start)
arm = source[start:end]
output = root / "build/gy511-host"
output.mkdir(parents=True, exist_ok=True)
prefix = r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define I2C_STS_RECV 1U
#define I2C_STS_STOP 2U
#define I2C_INT_RX_FULL 4U
#define I2C_DATA_CMD_CMD_READ 0x100U
#define I2C_DATA_CMD_STOP 0x200U
#define I2C_DATA_CMD_DAT 0xffU
#define I2C_DATA_CMD(unit) (unit)
#define I2C_INTR_MASK(unit) ((unit)+1U)
#define TRUE 1
static uint32_t incoming, commands[32], queued, masked;
static uint32_t in_w(uint32_t reg) { (void)reg; return incoming; }
static void out_w(uint32_t reg,uint32_t value) {
    (void)reg; commands[queued++]=value;
}
static void clr_w(uint32_t reg,uint32_t bits) {
    (void)reg; (void)bits; masked++;
}
static void check(int ok) { if(!ok) { puts("FAIL: I2C receive arm"); exit(1); } }
int main(void) {
    const uint32_t lengths[]={1U,2U,6U,16U};
    for(unsigned n=0U;n<4U;n++) {
        uint8_t buffer[16]={0};
        struct { uint32_t rdat_num,state; uint8_t *rbuf; } cb={lengths[n],I2C_STS_RECV,buffer};
        queued=0U; masked=0U;
        for(uint32_t i=0U;i<lengths[n];i++) {
            uint32_t unit=0U,cmd=0U,i2c_sts=I2C_INT_RX_FULL;
            int abort=0,wup=0;
            __typeof__(cb) *p_cb=&cb;
            incoming=0x40U+i;
            switch(cb.state) {
'''
suffix = r'''
            default: check(0); break;
            }
            check(!abort && buffer[i]==(uint8_t)incoming);
            if(i+1U<lengths[n]) {
                check(!wup && cb.state==I2C_STS_RECV);
                check(commands[queued-1U]==(I2C_DATA_CMD_CMD_READ |
                    ((i+2U==lengths[n]) ? I2C_DATA_CMD_STOP:0U)));
            } else {
                check(wup && cb.state==I2C_STS_STOP && cb.rdat_num==0U);
            }
        }
        check(queued==lengths[n]-1U && masked==1U);
    }
    puts("PASS: production I2C receive arm, 1/2/6/16-byte completion and final STOP command");
    return 0;
}
'''
harness = output / "test_i2c_receive.c"
harness.write_text(prefix + arm + suffix)
exe = output / "test_i2c_receive.exe"
subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wconversion",
                str(harness), "-o", str(exe)], check=True)
subprocess.run([str(exe)], check=True)
