/* Test the real subsystem4.c with simulated I2C and kernel calls. */
#include "../../../../app_program/subsystem4/subsystem4.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t g_registers[128][256];
static uint32_t g_open_calls;
static uint32_t g_close_calls;
static uint32_t g_read_calls;
static uint32_t g_write_calls;
static uint32_t g_fail_read_at;
static uint32_t g_fail_write_at;
static uint32_t g_mismatch_at;
static uint32_t g_cases;
static ER g_open_result;
static ER g_close_result;
static ER g_object_failure;
static uint32_t g_object_calls;
static ER g_task_result;

static void require(bool b_condition, char const *p_description)
{
    if (!b_condition)
    {
        fprintf(stderr, "FAIL: %s\n", p_description);
        exit(EXIT_FAILURE);
    }
}

static void reset_mock(void)
{
    memset(g_registers, 0, sizeof(g_registers));
    g_registers[0x1E][0x0A] = 0x48U;
    g_registers[0x1E][0x0B] = 0x34U;
    g_registers[0x1E][0x0C] = 0x33U;
    g_open_calls = 0U;
    g_close_calls = 0U;
    g_read_calls = 0U;
    g_write_calls = 0U;
    g_fail_read_at = 0U;
    g_fail_write_at = 0U;
    g_mismatch_at = 0U;
    g_open_result = 7;
    g_close_result = E_OK;
    g_object_failure = E_OK;
    g_object_calls = 0U;
    g_task_result = 9;
}

ID tk_opn_dev(UB const *p_name, uint32_t mode)
{
    require(strcmp((char const *)p_name, "iicb") == 0, "opens I2C1");
    require(mode == (TD_UPDATE | TD_EXCL), "exclusive read/write open");
    g_open_calls++;
    return g_open_result;
}

ER tk_cls_dev(ID h_device, uint32_t option)
{
    require(h_device == 7, "closes the opened descriptor");
    require(option == 0U, "normal close");
    g_close_calls++;
    return g_close_result;
}

ER i2c_read_reg(ID h_device, UW address, UW reg, UB *p_data)
{
    require(h_device == 7, "read uses opened descriptor");
    require(address < 128U && reg < 256U, "7-bit address and 8-bit register");
    g_read_calls++;
    if (g_read_calls <= 3U)
    {
        require(address == 0x1EU, "identity read uses magnetometer");
        require(reg == (0x09U + g_read_calls), "identity register order");
    }
    if (g_read_calls == g_fail_read_at)
    {
        return E_TMOUT;
    }
    *p_data = g_registers[address][reg];
    if (g_read_calls == g_mismatch_at)
    {
        *p_data ^= 0x01U;
    }
    return E_OK;
}

ER i2c_write_reg(ID h_device, UW address, UW reg, UB value)
{
    /* Expected values are taken independently from the sensor datasheet. */
    static uint8_t const g_expected[][3] =
    {
        {0x19U, 0x23U, 0x88U},
        {0x19U, 0x20U, 0x57U},
        {0x1EU, 0x00U, 0x14U},
        {0x1EU, 0x01U, 0x20U},
        {0x1EU, 0x02U, 0x00U}
    };

    require(h_device == 7, "write uses opened descriptor");
    require(g_write_calls < 5U, "only five configuration writes");
    require(address == g_expected[g_write_calls][0], "correct sensor address");
    require(reg == g_expected[g_write_calls][1], "correct register and order");
    require(value == g_expected[g_write_calls][2], "correct settings");
    g_write_calls++;
    if (g_write_calls == g_fail_write_at)
    {
        return E_TMOUT;
    }
    g_registers[address][reg] = value;
    return E_OK;
}

static void expect_configuration(ER expected_status)
{
    require(subsystem4_gy511_configure() == expected_status,
            "configuration returns expected status");
    require(g_open_calls == 1U, "one open attempt");
    require(g_close_calls == ((g_open_result < E_OK) ? 0U : 1U),
            "every successful open is closed once");
    g_cases++;
}

int main(void)
{
    reset_mock();
    expect_configuration(E_OK);
    require(g_read_calls == 8U && g_write_calls == 5U,
            "identity plus all five readbacks checked");

    reset_mock();
    g_open_result = E_NOEXS;
    expect_configuration(E_NOEXS);
    require(g_read_calls == 0U && g_write_calls == 0U,
            "no transfers after open failure");

    for (uint32_t index = 1U; index <= 8U; index++)
    {
        reset_mock();
        g_fail_read_at = index;
        expect_configuration(E_TMOUT);
        require(g_read_calls == index, "stops at failed read");
        require(g_write_calls == ((index <= 3U) ? 0U : (index - 3U)),
                "no further writes after failed read");

        reset_mock();
        g_mismatch_at = index;
        expect_configuration(E_IO);
        require(g_read_calls == index, "stops at identity/readback mismatch");
        require(g_write_calls == ((index <= 3U) ? 0U : (index - 3U)),
                "no further writes after mismatch");
    }

    for (uint32_t index = 1U; index <= 5U; index++)
    {
        reset_mock();
        g_fail_write_at = index;
        expect_configuration(E_TMOUT);
        require(g_write_calls == index, "stops at failed write");
        require(g_read_calls == (index + 2U), "no readback after failed write");
    }

    reset_mock();
    g_close_result = E_IO;
    expect_configuration(E_IO);

    reset_mock();
    g_fail_read_at = 1U;
    g_close_result = E_IO;
    expect_configuration(E_TMOUT);

    reset_mock();
    expect_configuration(E_OK);
    reset_mock();
    expect_configuration(E_OK);

    printf("PASS: %u GY-511 configuration scenarios\n",
           (unsigned int)g_cases);
    return EXIT_SUCCESS;
}
