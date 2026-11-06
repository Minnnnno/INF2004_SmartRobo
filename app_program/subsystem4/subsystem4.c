/*
 * Buddy 4 sensor startup using micro T-Kernel's device API.
 * Register definitions: ST LSM303DLHC datasheet, DocID018771 Rev 2.
 * Kernel API types are retained at the kernel boundary.
 */
#include "subsystem4.h"
#include <stdint.h>
#include <tk/device.h>

#define GY511_I2C_DEVICE       "iicb"
#define GY511_ACC_ADDRESS      (0x19U) /* 7-bit address, not shifted. */
#define GY511_MAG_ADDRESS      (0x1EU)
#define GY511_MAG_ID_FIRST     (0x0AU)

#define GY511_ACC_CTRL1        (0x20U)
#define GY511_ACC_CTRL4        (0x23U)
#define GY511_MAG_CRA          (0x00U)
#define GY511_MAG_CRB          (0x01U)
#define GY511_MAG_MODE         (0x02U)

/* 0x57: 100 Hz, normal power mode, X/Y/Z enabled. */
#define GY511_ACC_RATE_AXES    (0x57U)
/* 0x88: block data update, little-endian, +/-2 g, high resolution. */
#define GY511_ACC_FORMAT       (0x88U)
/* 0x14: temperature disabled, magnetic output rate 30 Hz. */
#define GY511_MAG_RATE         (0x14U)
/* 0x20: +/-1.3 gauss magnetic range. * (magnetic field strength,smallest range, give the finest resolution)*/
#define GY511_MAG_RANGE        (0x20U)
/* 0x00: continuous magnetic conversion. */
#define GY511_MAG_CONTINUOUS   (0x00U)

typedef struct
{
    uint8_t address;
    uint8_t reg;
    uint8_t value;
} gy511_setting_t;

/* Constant hardware configuration, not mutable application state.
 * Configure acceleration format before enabling acceleration output.
 * Configure magnetic rate/range before enabling continuous conversion.
 */
static gy511_setting_t const g_settings[] =
{
    {GY511_ACC_ADDRESS, GY511_ACC_CTRL4, GY511_ACC_FORMAT},
    {GY511_ACC_ADDRESS, GY511_ACC_CTRL1, GY511_ACC_RATE_AXES},
    {GY511_MAG_ADDRESS, GY511_MAG_CRA, GY511_MAG_RATE},
    {GY511_MAG_ADDRESS, GY511_MAG_CRB, GY511_MAG_RANGE},
    {GY511_MAG_ADDRESS, GY511_MAG_MODE, GY511_MAG_CONTINUOUS}
};

static ER subsystem4_check_identity(ID h_device);
static ER subsystem4_apply_setting(ID h_device,
                                  gy511_setting_t const *p_setting);

/*
 * Function: subsystem4_init
 * Purpose: Perform the complete stage-1 sensor startup sequence.
 * Inputs: None. I2C1 must already be registered by the kernel as "iicb".
 * Returns: E_OK, E_IO for a mismatched value, or the original driver error.
 * Behaviour: Open exclusively for read/write, check magnetic identity, apply
 *            and verify each setting, then close every successfully opened
 *            handle. Stop on the first failure; do not retry automatically.
 * Notes: Close releases the bus handle; it does not power down the sensor.
 *        If both configuration and close fail, preserve the original error.
 *        The low-level I2C driver provides transfer timeouts. This function
 *        does not impose a separate overall startup deadline.
 */
ER subsystem4_init(void)
{
    /*Open the I2C device*/
    ID h_device = tk_opn_dev((UB const *)GY511_I2C_DEVICE, TD_UPDATE | TD_EXCL);
    ER status = E_OK; 
    ER close_status = E_OK;

    /*Leave early if opening of I2c device fail*/
    if (h_device < E_OK) 
    {
        return (ER)h_device;
    }

    /*Check right chip is connected.*/
    /* Read the ID registers , wrong chip or wiring problem gives E_IO*/
    status = subsystem4_check_identity(h_device);
    for (uint8_t index = 0U;
        /*Write each setting in*/
         (E_OK == status) && (index < (sizeof(g_settings) / sizeof(g_settings[0])));
         index++)
    {
        status = subsystem4_apply_setting(h_device, &g_settings[index]);
    }
    /*Closing of handle so it bus is not left locked. */
    close_status = tk_cls_dev(h_device, 0U);
    if (E_OK == status)
    {
        status = close_status;
    }
    return status;
}

/*
 * Function: subsystem4_check_identity
 * Purpose: Verify the documented magnetic identity bytes before writing.
 * Input: h_device is the successfully opened I2C1 handle.
 * Returns: E_OK for "H43", E_IO for a mismatch, or the failed read's error.
 * Notes: The identity is in the magnetic block at 0x1E; the acceleration
 *        block at 0x19 is subsequently checked by configuration readbacks.
 *        Identity is a communication check, not a physical sensor self-test.
 */
static ER subsystem4_check_identity(ID h_device)
{
    /* Read magnetometer ID registers*/
    /*stores the 3 ID bytes the chip is expected to send back.*/
    static uint8_t const g_expected[] = {0x48U, 0x34U, 0x33U};
    ER status = E_OK;
    uint8_t actual = 0U;

    /* Loop over the 3 iD registers*/
    for (uint8_t index = 0U; index < sizeof(g_expected); index++)
    {
        status = i2c_read_reg(h_device, GY511_MAG_ADDRESS,
                              GY511_MAG_ID_FIRST + index, &actual);
        if (E_OK != status)
        {
            break;
        }
        /*Stop loop if byte does not match.*/
        if (g_expected[index] != actual)
        {
            status = E_IO;
            break;
        }
    }
    return status;
}

/*
 * Function: subsystem4_apply_setting
 * Purpose: Write one configuration byte and verify that it was retained.
 * Inputs: Valid open handle and non-NULL pointer into the private settings
 *         table. The caller guarantees these preconditions.
 * Returns: E_OK if readback matches; E_IO if not; driver error on transfer
 *          failure. A failed write is not followed by a read.
 * Notes: These are readable configuration registers, not data/status flags.
 */
static ER subsystem4_apply_setting(ID h_device,
                                  gy511_setting_t const *p_setting)
{

    /*Writes the value , read it and check if they it match */
    uint8_t actual = 0U;
    ER status = i2c_write_reg(h_device, p_setting->address,
                              p_setting->reg, p_setting->value);

    if (E_OK == status)
    {
        status = i2c_read_reg(h_device, p_setting->address,
                              p_setting->reg, &actual);
        if ((E_OK == status) && (p_setting->value != actual))
        {
            status = E_IO;
        }
    }
    return status;
}
