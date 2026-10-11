/*
 * gy511_config.c
 * Configure the GY-511 once at startup using the kernel's I2C1 driver.
 * Sensor reference: ST LSM303DLHC datasheet, DocID018771 Rev 2.
 * Kernel API types are retained at the device and task boundaries.
 */

#include "subsystem4.h"

#include <stdint.h>
#include <tk/device.h>

#define GY511_ACC_CTRL_REG1   (0x20U)
#define GY511_ACC_CTRL_REG4   (0x23U)
#define GY511_MAG_CRA_REG     (0x00U)
#define GY511_MAG_CRB_REG     (0x01U)
#define GY511_MAG_MODE_REG    (0x02U)
#define GY511_MAG_ID_REG      (0x0AU)

#define GY511_ACC_100_HZ_XYZ  (0x57U)
#define GY511_ACC_BDU_HR_2G   (0x88U)
#define GY511_MAG_30_HZ       (0x14U)
#define GY511_MAG_1_3_GAUSS   (0x20U)
#define GY511_MAG_CONTINUOUS  (0x00U)

typedef struct
{
    uint8_t address;
    uint8_t register_address;
    uint8_t value;
} gy511_register_t;

/* Set range/resolution before enabling the accelerometer.
 * BDU prevents a high/low byte pair from changing halfway through a read.
 */
static gy511_register_t const g_gy511_configuration[] =
{
    {S4_GY511_ACC_ADDRESS, GY511_ACC_CTRL_REG4, GY511_ACC_BDU_HR_2G},
    {S4_GY511_ACC_ADDRESS, GY511_ACC_CTRL_REG1, GY511_ACC_100_HZ_XYZ},
    {S4_GY511_MAG_ADDRESS, GY511_MAG_CRA_REG, GY511_MAG_30_HZ},
    {S4_GY511_MAG_ADDRESS, GY511_MAG_CRB_REG, GY511_MAG_1_3_GAUSS},
    {S4_GY511_MAG_ADDRESS, GY511_MAG_MODE_REG, GY511_MAG_CONTINUOUS}
};

/* Documented magnetometer identity registers contain "H43". */
static uint8_t const g_gy511_identity[] = {0x48U, 0x34U, 0x33U};

static ER subsystem4_gy511_check_id(ID h_device);
static ER subsystem4_gy511_write_check(ID h_device,
                                      gy511_register_t const *p_setting);

EXPORT ER subsystem4_gy511_configure(void)
{
    ID h_device = tk_opn_dev((UB const *)S4_GY511_I2C_DEVICE,
                             TD_UPDATE | TD_EXCL);
    ER status = E_OK;
    ER close_status = E_OK;

    if (h_device < E_OK)
    {
        return (ER)h_device;
    }

    status = subsystem4_gy511_check_id(h_device);
    for (uint32_t index = 0U;
         (status == E_OK) &&
         (index < (sizeof(g_gy511_configuration) /
                   sizeof(g_gy511_configuration[0])));
         index++)
    {
        status = subsystem4_gy511_write_check(h_device,
                                             &g_gy511_configuration[index]);
    }

    /* Preserve the original failure if closing also fails. */
    close_status = tk_cls_dev(h_device, 0U);
    if (status == E_OK)
    {
        status = close_status;
    }

    return status;
}



/* Check the device before changing any configuration register. */
static ER subsystem4_gy511_check_id(ID h_device)
{
    ER status = E_OK;
    uint8_t identity = 0U;

    for (uint32_t index = 0U; index < sizeof(g_gy511_identity); index++)
    {
        status = i2c_read_reg(h_device, S4_GY511_MAG_ADDRESS,
                              GY511_MAG_ID_REG + index, &identity);
        if (status != E_OK)
        {
            break;
        }

        if (identity != g_gy511_identity[index])
        {
            status = E_IO;
            break;
        }
    }

    return status;
}

/* Verify each write so a silent configuration failure cannot pass. */
static ER subsystem4_gy511_write_check(ID h_device,
                                      gy511_register_t const *p_setting)
{
    uint8_t actual_value = 0U;
    ER status = i2c_write_reg(h_device, p_setting->address,
                              p_setting->register_address, p_setting->value);

    if (status == E_OK)
    {
        status = i2c_read_reg(h_device, p_setting->address,
                              p_setting->register_address, &actual_value);
        if ((status == E_OK) && (actual_value != p_setting->value))
        {
            status = E_IO;
        }
    }

    return status;
}
