/* Exercise byte ordering, sign conversion, freshness and all error paths. */
#include "../../../app_program/subsystem4/gy511_read.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t g_raw[2][6];
static uint8_t g_acceleration_status;
static uint8_t g_magnetic_status;
static uint32_t g_open_calls;
static uint32_t g_close_calls;
static uint32_t g_read_calls;
static uint32_t g_burst_calls;
static uint32_t g_fail_read_at;
static uint32_t g_fail_burst_at;
static uint32_t g_short_burst_at;
static uint32_t g_cases;
static ER g_open_result;
static ER g_close_result;

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
    static uint8_t const g_data[2][6] =
    {
        {0x10U, 0x00U, 0xF0U, 0xFFU, 0x80U, 0x3EU},
        {0xFFU, 0xFFU, 0x80U, 0x00U, 0x7FU, 0xFFU}
    };
    memcpy(g_raw, g_data, sizeof(g_raw));
    g_acceleration_status = 0x08U;
    g_magnetic_status = 0x01U;
    g_open_calls = 0U;
    g_close_calls = 0U;
    g_read_calls = 0U;
    g_burst_calls = 0U;
    g_fail_read_at = 0U;
    g_fail_burst_at = 0U;
    g_short_burst_at = 0U;
    g_open_result = 7;
    g_close_result = E_OK;
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
    require(h_device == 7 && option == 0U, "closes valid descriptor");
    g_close_calls++;
    return g_close_result;
}

ER i2c_read_reg(ID h_device, UW address, UW reg, UB *p_data)
{
    require(h_device == 7, "valid read descriptor");
    require((address == 0x19U && reg == 0x27U) ||
            (address == 0x1EU && reg == 0x09U), "status register addresses");
    g_read_calls++;
    if (g_read_calls == g_fail_read_at)
    {
        return E_TMOUT;
    }
    *p_data = (address == 0x19U) ? g_acceleration_status : g_magnetic_status;
    return E_OK;
}

ER tk_swri_dev(ID h_device, int32_t start, void const *p_buffer,
               SZ size, SZ *p_actual_size)
{
    T_I2C_EXEC const *p_transaction = p_buffer;
    require(h_device == 7 && start == TDN_I2C_EXEC, "combined I2C transfer");
    require(size == (SZ)sizeof(T_I2C_EXEC), "transaction structure size");
    require(g_burst_calls < 2U, "only two bursts per sample");
    require(p_transaction->snd_size == 1 && p_transaction->rcv_size == 6,
            "one register byte, six response bytes");
    require(p_transaction->sadr == ((g_burst_calls == 0U) ? 0x19U : 0x1EU),
            "accelerometer then magnetometer");
    require(*p_transaction->snd_data ==
            ((g_burst_calls == 0U) ? 0xA8U : 0x03U), "auto-increment register");
    g_burst_calls++;
    if (g_burst_calls == g_fail_burst_at)
    {
        return E_TMOUT;
    }
    memcpy(p_transaction->rcv_data, g_raw[g_burst_calls - 1U], 6U);
    *p_actual_size = (SZ)sizeof(T_I2C_EXEC);
    if (g_burst_calls == g_short_burst_at)
    {
        (*p_actual_size)--;
    }
    return E_OK;
}

static gy511_sample_t expect_read(ER expected_status)
{
    gy511_sample_t sample;
    gy511_sample_t previous;
    memset(&sample, 0x55, sizeof(sample));
    previous = sample;
    require(gy511_read_sample(&sample) == expected_status, "read status");
    require(g_open_calls == 1U, "one open");
    require(g_close_calls == ((g_open_result < E_OK) ? 0U : 1U),
            "close on success and every transfer failure");
    if (expected_status != E_OK)
    {
        require(memcmp(&sample, &previous, sizeof(sample)) == 0,
                "failed read leaves previous sample intact");
    }
    g_cases++;
    return sample;
}

int main(void)
{
    gy511_sample_t sample = {0};
    reset_mock();
    sample = expect_read(E_OK);
    require(sample.acceleration_x == 1 && sample.acceleration_y == -1 &&
            sample.acceleration_z == 1000, "little-endian 12-bit conversion");
    require(sample.magnetic_x == -1 && sample.magnetic_y == 32767 &&
            sample.magnetic_z == -32768, "magnetic X Z Y and signed limits");

    reset_mock();
    require(gy511_read_sample(NULL) == E_PAR, "reject null output");
    require(g_open_calls == 0U, "null output does not touch bus");
    g_cases++;

    reset_mock();
    g_open_result = E_NOEXS;
    (void)expect_read(E_NOEXS);
    require(g_read_calls == 0U && g_burst_calls == 0U, "no invalid transfers");

    for (uint32_t state = 0U; state < 3U; state++)
    {
        reset_mock();
        g_acceleration_status = (state == 1U) ? 0x08U : 0U;
        g_magnetic_status = (state == 2U) ? 0x01U : 0U;
        (void)expect_read(E_BUSY);
        require(g_burst_calls == 0U, "no stale sample bursts");
    }

    for (uint32_t index = 1U; index <= 2U; index++)
    {
        reset_mock();
        g_fail_read_at = index;
        (void)expect_read(E_TMOUT);
        require(g_read_calls == index && g_burst_calls == 0U,
                "stop after failed status read");

        reset_mock();
        g_fail_burst_at = index;
        (void)expect_read(E_TMOUT);
        require(g_burst_calls == index, "stop after failed burst");

        reset_mock();
        g_short_burst_at = index;
        (void)expect_read(E_IO);
        require(g_burst_calls == index, "reject incomplete transaction");
    }

    reset_mock();
    g_close_result = E_IO;
    (void)expect_read(E_IO);

    reset_mock();
    g_fail_read_at = 1U;
    g_close_result = E_IO;
    (void)expect_read(E_TMOUT);

    reset_mock();
    memset(g_raw, 0, sizeof(g_raw));
    sample = expect_read(E_OK);
    require(sample.acceleration_x == 0 && sample.magnetic_z == 0,
            "zero conversion");

    reset_mock();
    g_raw[0][0] = 0x00U;
    g_raw[0][1] = 0x80U;
    g_raw[0][2] = 0xF0U;
    g_raw[0][3] = 0x7FU;
    sample = expect_read(E_OK);
    require(sample.acceleration_x == -2048 && sample.acceleration_y == 2047,
            "accelerometer signed endpoints");

    printf("PASS: %u standalone IMU reading scenarios\n", (unsigned int)g_cases);
    return EXIT_SUCCESS;
}
