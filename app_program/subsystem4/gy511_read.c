/* LSM303DLHC acceleration acquisition; ST DocID018771 Rev 2, sections 5-7. */
#include "gy511_read.h"
#include <tk/device.h>

#define GY511_ACC_ADDRESS (0x19U)
#define GY511_ACC_STATUS  (0x27U)
#define GY511_XYZ_READY   (0x08U)
#define GY511_ACC_BURST   (0xA8U) /* OUT_X_L=0x28 plus auto-increment bit. */
#define GY511_ACC_INT1_SRC (0x31U) /* Latched inertial interrupt source. */

/*
 * Read status first, then all six output bytes in one combined I2C transfer.
 * The accelerometer is little-endian, with 12 useful bits left-aligned in
 * each 16-bit word. Remove the four unused low bits and pack the result.
 * The five-byte output is committed only after transfer AND close succeed.
 * E_BUSY is a normal polling outcome, not a device failure. The 100 Hz sensor
 * can produce more readings than the task consumes; this is latest-sample
 * polling, not a lossless stream. Magnetic status is never consulted.
 */

 /* Get one fresh X/Y/Z sample from accelerator*/
ER gy511_read_acceleration(gy511_acceleration_t *p_sample)
{
    uint8_t raw[6] = {0};  // the 6 bytes from the chip: X low, X high, Y low, Y high, Z low, Z high
    uint8_t ready = 0U;    // the status register value
    uint8_t start_register = GY511_ACC_BURST; 
    T_I2C_EXEC transaction = {0};  // describes one combined write+read I2C transfer
    SZ actual_size = 0;
    ER status = E_OK;
    ER close_status = E_OK;
    ID h_device = 0;

    if (NULL == p_sample)   //reject a null pointer and return a bad parameter error instead of crashing
    {
        return E_PAR;
    }
    //open of I2c bus for reading/writing
    h_device = tk_opn_dev((UB const *)"iicb", TD_UPDATE | TD_EXCL); 
    if (h_device < E_OK)
    {
        return (ER)h_device;
    }
    //read the register from accelorometer
    // if no new data yet, skip the cycle
    status = i2c_read_reg(h_device, GY511_ACC_ADDRESS, GY511_ACC_STATUS, &ready);
    if ((E_OK == status) && (0U == (ready & GY511_XYZ_READY)))
    {
        status = E_BUSY;
    }
    //if new data, read all 6 databytes 
    if (E_OK == status)
    {
        transaction.sadr = GY511_ACC_ADDRESS;
        transaction.snd_size = 1;
        transaction.snd_data = &start_register;
        transaction.rcv_size = sizeof(raw);
        transaction.rcv_data = raw;
        status = tk_swri_dev(h_device, TDN_I2C_EXEC, &transaction,
                             sizeof(transaction), &actual_size);
        /* This driver reports the descriptor size, not the six data bytes. */
        if ((E_OK == status) && (actual_size != sizeof(transaction)))
        {
            status = E_IO;
        }
    }
    //closing of handle, so bus is not left locked.
    close_status = tk_cls_dev(h_device, 0U);
    if ((E_OK == status) || (E_BUSY == status))
    {
        if (E_OK != close_status)
        {
            status = close_status;
        }
    }
    //unpack and pack the result on success
    if (E_OK == status)
    {
        uint32_t x = (((uint32_t)raw[1] << 8U) | raw[0]) >> 4U;
        uint32_t y = (((uint32_t)raw[3] << 8U) | raw[2]) >> 4U;
        uint32_t z = (((uint32_t)raw[5] << 8U) | raw[4]) >> 4U;
        p_sample->bytes[0] = (uint8_t)(x & 0xFFU);
        p_sample->bytes[1] = (uint8_t)((x >> 8U) | ((y & 0x0FU) << 4U));
        p_sample->bytes[2] = (uint8_t)(y >> 4U);
        p_sample->bytes[3] = (uint8_t)(z & 0xFFU);
        p_sample->bytes[4] = (uint8_t)(z >> 8U);
    }
    return status;
}

/* One-byte read of INT1_SRC_A. Reading it clears the latched event (LIR_INT1),
 * so every impact since the previous read is reported exactly once.
 */
ER gy511_read_impact(uint8_t *p_source)
{
    uint8_t source = 0U;
    ER status = E_OK;
    ER close_status = E_OK;
    ID h_device = 0;

    if (NULL == p_source)
    {
        return E_PAR;
    }
    h_device = tk_opn_dev((UB const *)"iicb", TD_UPDATE | TD_EXCL);
    if (h_device < E_OK)
    {
        return (ER)h_device;
    }
    status = i2c_read_reg(h_device, GY511_ACC_ADDRESS, GY511_ACC_INT1_SRC, &source);
    close_status = tk_cls_dev(h_device, 0U);
    if ((E_OK == status) && (E_OK != close_status))
    {
        status = close_status;
    }
    if (E_OK == status)
    {
        *p_source = (uint8_t)(source & 0x7FU);
    }
    return status;
}

/*
 * Extract one unsigned 12-bit pattern, then explicitly convert two's
 * complement to a signed value. Values 2048..4095 represent -2048..-1.
 * All shifts operate on unsigned data; no signed right shift or out-of-range
 * unsigned-to-signed cast is needed. The wider temporary is arithmetic only.
 */

 //unpacking  one axis from the 5 byte packed sample to be used in tlit calculation
ER gy511_acceleration_mg(gy511_acceleration_t const *p_sample,
                       uint8_t axis, int16_t *p_mg)
{
    uint32_t value = 0U;
    int32_t signed_value = 0;
    //if null pointer, return Bad parameter error, write nothing
    if ((NULL == p_sample) || (NULL == p_mg) || (axis > 2U))
    {
        return E_PAR;
    }
    switch (axis)
    {
        //else pull out the 12 bits for the chosen axis
        //rebuilds one axis full 12 bits value from packed bytes
        case 0U: //gives X's low 8bits, all of byte 0 + lower half of byte 1
            value = p_sample->bytes[0] | (((uint32_t)p_sample->bytes[1] & 0x0FU) << 8U);
            break;
        case 1U: //Y , Upper half of byte 1  + al of byte 2
            value = ((uint32_t)p_sample->bytes[1] >> 4U) | ((uint32_t)p_sample->bytes[2] << 4U);
            break;
        default: //Z All of byte 3 + lower half of byte 4
            value = p_sample->bytes[3] | (((uint32_t)p_sample->bytes[4] & 0x0FU) << 8U);
            break;
    }
    //turns the raw 12 bit number into real signed value
    signed_value = (int32_t)value;
    if (value >= 2048U) //if reading is negative (signed bit =1), do wrapping to find its negative value.
    {
        signed_value -= 4096;
    }
    *p_mg = (int16_t)signed_value;
    return E_OK;
}


/* Magnetic registers auto-increment from 0x03, in big-endian X,Z,Y order.
 * Read all six bytes, including when LOCK is set, to finish a previously
 * interrupted output read. Discard that locked set; wait for a fresh one.
 * Datasheet table 75 permits only signed 12-bit values. Reject all other
 * words (including 0xF000 overflow), before packing away sign-extension bits.
 */
ER gy511_read_magnetic_diagnostic(gy511_magnetic_t *p_sample, uint8_t *p_outcome)
{
    uint8_t raw[6] = {0};
    uint8_t ready = 0U;
    uint8_t busy_outcome = GY511_MAG_ERROR;
    uint8_t start_register = 0x03U;
    uint32_t values[3] = {0};
    uint8_t const offsets[3] = {0U, 4U, 2U}; /* Hardware X,Z,Y -> logical X,Y,Z. */
    T_I2C_EXEC transaction = {0};
    SZ actual_size = 0;
    ER status = E_OK;
    ER close_status;
    ID h_device;

    if (NULL != p_outcome)
    {
        *p_outcome = GY511_MAG_ERROR;
    }
    if (NULL == p_sample)
    {
        return E_PAR;
    }
    h_device = tk_opn_dev((UB const *)"iicb", TD_UPDATE | TD_EXCL);
    if (h_device < E_OK)
    {
        return (ER)h_device;
    }
    status = i2c_read_reg(h_device, 0x1EU, 0x09U, &ready);
    if ((E_OK == status) && (0U == (ready & 0x03U)))
    {
        status = E_BUSY;
        busy_outcome = GY511_MAG_NOT_READY;
    }
    if (E_OK == status)
    {
        transaction.sadr = 0x1EU;
        transaction.snd_size = 1;
        transaction.snd_data = &start_register;
        transaction.rcv_size = sizeof(raw);
        transaction.rcv_data = raw;
        status = tk_swri_dev(h_device, TDN_I2C_EXEC, &transaction,
                             sizeof(transaction), &actual_size);
        if ((E_OK == status) && (actual_size != sizeof(transaction)))
        {
            status = E_IO;
        }
        if ((E_OK == status) && (0U != (ready & 0x02U)))
        {
            status = E_BUSY;
            busy_outcome = GY511_MAG_LOCKED;
        }
    }
    close_status = tk_cls_dev(h_device, 0U);
    if (((E_OK == status) || (E_BUSY == status)) && (E_OK != close_status))
    {
        status = close_status;
    }
    for (uint8_t axis = 0U; (E_OK == status) && (axis < 3U); axis++)
    {
        uint8_t offset = offsets[axis];
        uint32_t word = ((uint32_t)raw[offset] << 8U) | raw[offset + 1U];
        if ((word > 0x07FFU) && (word < 0xF800U))
        {
            status = E_IO;
        }
        values[axis] = word & 0x0FFFU;
    }
    if (E_OK == status)
    {
        p_sample->bytes[0] = (uint8_t)(values[0] & 0xFFU);
        p_sample->bytes[1] = (uint8_t)((values[0] >> 8U) | ((values[1] & 0x0FU) << 4U));
        p_sample->bytes[2] = (uint8_t)(values[1] >> 4U);
        p_sample->bytes[3] = (uint8_t)(values[2] & 0xFFU);
        p_sample->bytes[4] = (uint8_t)(values[2] >> 8U);
    }
    if (NULL != p_outcome)
    {
        if (E_OK == status)
        {
            *p_outcome = GY511_MAG_FRESH;
        }
        else if (E_BUSY == status)
        {
            *p_outcome = busy_outcome;
        }
    }
    return status;
}

/* Ordinary acquisition API for callers that do not need diagnostics. */
ER gy511_read_magnetic(gy511_magnetic_t *p_sample)
{
    return gy511_read_magnetic_diagnostic(p_sample, NULL);
}

/* Share the existing signed 12-bit decoder, without aliasing different types.
 * The temporary copies packed bits only; no acceleration scaling is applied.
 */
ER gy511_magnetic_counts(gy511_magnetic_t const *p_sample,
                         uint8_t axis, int16_t *p_counts)
{
    gy511_acceleration_t packed = {{0}};
    if (NULL == p_sample)
    {
        return E_PAR;
    }
    for (uint8_t i = 0U; i < sizeof(packed.bytes); i++)
    {
        packed.bytes[i] = p_sample->bytes[i];
    }
    return gy511_acceleration_mg(&packed, axis, p_counts);
}
