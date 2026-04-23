#include "mpu6050.h"
#include <stddef.h>

#define REG_SAMPLE_DIV    0x19U
#define REG_CONFIG        0x1AU
#define REG_GYRO_CONFIG   0x1BU
#define REG_ACCEL_CONFIG  0x1CU
#define REG_FIFO_ENABLE   0x23U
#define REG_INT_PIN       0x37U
#define REG_INT_ENABLE    0x38U
#define REG_INT_STATUS    0x3AU
#define REG_ACCEL_X_H     0x3BU
#define REG_USER_CTRL     0x6AU
#define REG_PWR1          0x6BU
#define REG_PWR2          0x6CU
#define REG_WHO_AM_I      0x75U
#define EXPECTED_ID       0x68U
#define IO_TIMEOUT_MS     100U

static bool valid_device(const MPU6050_Device *device)
{
    return device != NULL && device->i2c != NULL &&
           (device->address_7bit == 0x68U || device->address_7bit == 0x69U);
}

static MPU6050_Result read_regs(MPU6050_Device *device, uint8_t reg,
                               uint8_t *data, uint16_t size)
{
    device->last_hal_status = HAL_I2C_Mem_Read(
        device->i2c, (uint16_t)(device->address_7bit << 1), reg,
        I2C_MEMADD_SIZE_8BIT, data, size, IO_TIMEOUT_MS);
    if (device->last_hal_status != HAL_OK)
    {
        device->failed_register = reg;
        return MPU6050_BUS_ERROR;
    }
    return MPU6050_OK;
}

static MPU6050_Result write_reg(MPU6050_Device *device, uint8_t reg,
                                uint8_t value)
{
    device->last_hal_status = HAL_I2C_Mem_Write(
        device->i2c, (uint16_t)(device->address_7bit << 1), reg,
        I2C_MEMADD_SIZE_8BIT, &value, 1U, IO_TIMEOUT_MS);
    if (device->last_hal_status != HAL_OK)
    {
        device->failed_register = reg;
        return MPU6050_BUS_ERROR;
    }
    return MPU6050_OK;
}

static MPU6050_Result write_checked(MPU6050_Device *device, uint8_t reg,
                                    uint8_t value)
{
    MPU6050_Result result = write_reg(device, reg, value);
    if (result != MPU6050_OK) return result;
    uint8_t actual = 0U;
    result = read_regs(device, reg, &actual, 1U);
    if (result != MPU6050_OK) return result;
    if (actual != value)
    {
        device->failed_register = reg;
        return MPU6050_CONFIG_ERROR;
    }
    return MPU6050_OK;
}

static int16_t decode_be16(const uint8_t *data)
{
    uint16_t bits = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    int32_t value = (int32_t)bits;
    if ((bits & 0x8000U) != 0U) value -= 65536L;
    return (int16_t)value;
}

MPU6050_Result MPU6050_Init(MPU6050_Device *device)
{
    if (!valid_device(device)) return MPU6050_INVALID_ARGUMENT;
    device->initialized = false;
    device->who_am_i = 0U;
    device->failed_register = 0U;
    device->last_hal_status = HAL_OK;

    MPU6050_Result result = read_regs(device, REG_WHO_AM_I,
                                     &device->who_am_i, 1U);
    if (result != MPU6050_OK) return result;
    if (device->who_am_i != EXPECTED_ID) return MPU6050_BAD_ID;

    /* Restore a known state; the reset bit clears itself. */
    result = write_reg(device, REG_PWR1, 0x80U);
    if (result != MPU6050_OK) return result;
    HAL_Delay(100U);

    /* Wake up, enable temperature and select the X-gyro PLL clock. */
    result = write_checked(device, REG_PWR1, 0x01U);
    if (result != MPU6050_OK) return result;
    result = write_checked(device, REG_PWR2, 0x00U);
    if (result != MPU6050_OK) return result;
    HAL_Delay(100U);

    static const uint8_t config[][2] = {
        {REG_INT_ENABLE,   0x00U},
        {REG_FIFO_ENABLE,  0x00U},
        {REG_USER_CTRL,    0x00U},
        {REG_CONFIG,       0x03U},
        {REG_SAMPLE_DIV,   0x09U},
        {REG_GYRO_CONFIG,  0x00U},
        {REG_ACCEL_CONFIG, 0x00U},
        {REG_INT_PIN,      0x00U}
    };

    for (size_t i = 0U; i < sizeof(config) / sizeof(config[0]); ++i)
    {
        result = write_checked(device, config[i][0], config[i][1]);
        if (result != MPU6050_OK) return result;
    }

    /* Enable the data-ready source; poll its status without an EXTI wire. */
    result = write_checked(device, REG_INT_ENABLE, 0x01U);
    if (result != MPU6050_OK) return result;

    uint8_t discard;
    result = read_regs(device, REG_INT_STATUS, &discard, 1U);
    if (result != MPU6050_OK) return result;

    device->initialized = true;
    return MPU6050_OK;
}

MPU6050_Result MPU6050_ReadSample(MPU6050_Device *device,
                                 MPU6050_Sample *sample)
{
    if (!valid_device(device) || sample == NULL) return MPU6050_INVALID_ARGUMENT;
    if (!device->initialized) return MPU6050_NOT_INITIALIZED;

    uint8_t status = 0U;
    MPU6050_Result result = read_regs(device, REG_INT_STATUS, &status, 1U);
    if (result != MPU6050_OK) return result;
    if ((status & 0x01U) == 0U) return MPU6050_NO_DATA;

    uint8_t data[14];
    result = read_regs(device, REG_ACCEL_X_H, data, sizeof(data));
    if (result != MPU6050_OK) return result;

    MPU6050_Sample next = {0};
    next.ax_raw = decode_be16(&data[0]);
    next.ay_raw = decode_be16(&data[2]);
    next.az_raw = decode_be16(&data[4]);
    next.temperature_raw = decode_be16(&data[6]);
    next.gx_raw = decode_be16(&data[8]);
    next.gy_raw = decode_be16(&data[10]);
    next.gz_raw = decode_be16(&data[12]);

    /* Fixed-point units avoid floating-point printf configuration. */
    next.ax_mg = (int32_t)next.ax_raw * 1000L / 16384L;
    next.ay_mg = (int32_t)next.ay_raw * 1000L / 16384L;
    next.az_mg = (int32_t)next.az_raw * 1000L / 16384L;
    next.gx_mdps = (int32_t)next.gx_raw * 1000L / 131L;
    next.gy_mdps = (int32_t)next.gy_raw * 1000L / 131L;
    next.gz_mdps = (int32_t)next.gz_raw * 1000L / 131L;
    next.temperature_mdegc = 36530L + (int32_t)next.temperature_raw * 1000L / 340L;
    next.read_time_ms = HAL_GetTick();
    *sample = next;
    return MPU6050_OK;
}

const char *MPU6050_ResultString(MPU6050_Result result)
{
    switch (result)
    {
        case MPU6050_OK: return "OK";
        case MPU6050_NO_DATA: return "NO_DATA";
        case MPU6050_BUS_ERROR: return "BUS_ERROR";
        case MPU6050_BAD_ID: return "BAD_ID";
        case MPU6050_CONFIG_ERROR: return "CONFIG_ERROR";
        case MPU6050_INVALID_ARGUMENT: return "INVALID_ARGUMENT";
        case MPU6050_NOT_INITIALIZED: return "NOT_INITIALIZED";
        default: return "UNKNOWN";
    }
}
