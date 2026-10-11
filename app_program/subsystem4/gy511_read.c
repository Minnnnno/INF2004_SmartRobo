/* LSM303DLHC burst reads; BARR-C naming, braces and fixed-width samples. */
#include "gy511_read.h"
#include <tk/device.h>

#define GY511_ACC_STATUS_REG  (0x27U)
#define GY511_MAG_STATUS_REG  (0x09U)
#define GY511_ACC_READY       (0x08U)
#define GY511_MAG_READY       (0x01U)
#define GY511_ACC_BURST_REG   (0xA8U) /* OUT_X_L_A plus auto-increment bit. */
#define GY511_MAG_BURST_REG   (0x03U) /* Magnetometer increments itself. */
#define GY511_AXIS_BYTES      (6U)

static ER gy511_read_block(ID h_device, uint8_t address,
                          uint8_t reg, uint8_t *p_data);
static int16_t gy511_signed_value(uint8_t high_byte, uint8_t low_byte);

ER gy511_read_sample(gy511_sample_t *p_sample)
{
    uint8_t acceleration[GY511_AXIS_BYTES] = {0};
    uint8_t magnetic[GY511_AXIS_BYTES] = {0};
    uint8_t acceleration_status = 0U;
    uint8_t magnetic_status = 0U;
    ID h_device = 0;
    ER status = E_OK;
    ER close_status = E_OK;

    if (p_sample == NULL)
    {
        return E_PAR;
    }
    h_device = tk_opn_dev((UB const *)S4_GY511_I2C_DEVICE,
                          TD_UPDATE | TD_EXCL);
    if (h_device < E_OK)
    {
        return (ER)h_device;
    }

    status = i2c_read_reg(h_device, S4_GY511_ACC_ADDRESS,
                          GY511_ACC_STATUS_REG, &acceleration_status);
    if (status == E_OK)
    {
        status = i2c_read_reg(h_device, S4_GY511_MAG_ADDRESS,
                              GY511_MAG_STATUS_REG, &magnetic_status);
    }
    if ((status == E_OK) &&
        (((acceleration_status & GY511_ACC_READY) == 0U) ||
         ((magnetic_status & GY511_MAG_READY) == 0U)))
    {
        status = E_BUSY;
    }
    if (status == E_OK)
    {
        status = gy511_read_block(h_device, S4_GY511_ACC_ADDRESS,
                                  GY511_ACC_BURST_REG, acceleration);
    }
    if (status == E_OK)
    {
        status = gy511_read_block(h_device, S4_GY511_MAG_ADDRESS,
                                  GY511_MAG_BURST_REG, magnetic);
    }

    close_status = tk_cls_dev(h_device, 0U);
    if (status == E_OK)
    {
        status = close_status;
    }
    if (status == E_OK)
    {
        /* Acceleration is little-endian and left-aligned by four bits. */
        p_sample->acceleration_x = (int16_t)(gy511_signed_value(
            acceleration[1], acceleration[0]) / 16);
        p_sample->acceleration_y = (int16_t)(gy511_signed_value(
            acceleration[3], acceleration[2]) / 16);
        p_sample->acceleration_z = (int16_t)(gy511_signed_value(
            acceleration[5], acceleration[4]) / 16);
        /* Magnetometer is big-endian, with X, Z, Y register order. */
        p_sample->magnetic_x = gy511_signed_value(magnetic[0], magnetic[1]);
        p_sample->magnetic_y = gy511_signed_value(magnetic[4], magnetic[5]);
        p_sample->magnetic_z = gy511_signed_value(magnetic[2], magnetic[3]);
    }
    return status;
}

static ER gy511_read_block(ID h_device, uint8_t address,
                          uint8_t reg, uint8_t *p_data)
{
    T_I2C_EXEC transaction = {0};
    SZ actual_size = 0;
    ER status = E_OK;

    transaction.sadr = address;
    transaction.snd_size = 1;
    transaction.snd_data = &reg;
    transaction.rcv_size = GY511_AXIS_BYTES;
    transaction.rcv_data = p_data;
    status = tk_swri_dev(h_device, TDN_I2C_EXEC, &transaction,
                         sizeof(transaction), &actual_size);
    if ((status == E_OK) && (actual_size != (SZ)sizeof(transaction)))
    {
        status = E_IO;
    }
    return status;
}

/* Convert signed values without an out-of-range unsigned-to-signed cast. */
static int16_t gy511_signed_value(uint8_t high_byte, uint8_t low_byte)
{
    int32_t value = ((int32_t)high_byte * 256) + (int32_t)low_byte;
    if (value >= 32768)
    {
        value -= 65536;
    }
    return (int16_t)value;
}
