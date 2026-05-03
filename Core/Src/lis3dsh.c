#include "lis3dsh.h"
#include <stddef.h>

#define REG_WHO_AM_I       0x0FU
#define EXPECTED_ID        0x3FU
#define REG_CTRL4          0x20U
#define REG_CTRL5          0x24U
#define REG_CTRL6          0x25U
#define REG_STATUS         0x27U
#define REG_OUT_X_L        0x28U
#define SPI_READ_BIT       0x80U
#define STATUS_NEW_XYZ     0x08U
#define STATUS_OVERRUN     0x80U
#define SPI_TIMEOUT_MS     100U
#define SENSITIVITY_UG     60L

static bool device_valid(const LIS3DSH_Device *device)
{
    return device != NULL && device->spi != NULL &&
           device->cs_port != NULL && device->cs_pin != 0U;
}

static LIS3DSH_Result read_registers(LIS3DSH_Device *device,
                                    uint8_t address, uint8_t *data,
                                    uint16_t count)
{
    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};

    if (data == NULL || count == 0U || count > 6U)
    {
        return LIS3DSH_INVALID_ARGUMENT;
    }

    tx[0] = address | SPI_READ_BIT;
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
    device->last_hal_status = HAL_SPI_TransmitReceive(
        device->spi, tx, rx, count + 1U, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

    if (device->last_hal_status != HAL_OK)
    {
        return LIS3DSH_BUS_ERROR;
    }

    for (uint16_t i = 0U; i < count; ++i)
    {
        data[i] = rx[i + 1U];
    }
    return LIS3DSH_OK;
}

static LIS3DSH_Result write_checked(LIS3DSH_Device *device,
                                   uint8_t address, uint8_t value)
{
    uint8_t tx[2] = {address, value};
    uint8_t readback = 0U;

    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
    device->last_hal_status = HAL_SPI_Transmit(
        device->spi, tx, sizeof(tx), SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

    if (device->last_hal_status != HAL_OK)
    {
        return LIS3DSH_BUS_ERROR;
    }

    LIS3DSH_Result result = read_registers(device, address, &readback, 1U);
    if (result != LIS3DSH_OK)
    {
        return result;
    }
    return readback == value ? LIS3DSH_OK : LIS3DSH_CONFIG_ERROR;
}

static int16_t decode_le16(uint8_t low, uint8_t high)
{
    uint16_t bits = (uint16_t)((uint16_t)low | ((uint16_t)high << 8));

    /* Decode two's complement without an out-of-range unsigned-to-signed cast. */
    int32_t value = (int32_t)bits;
    if ((bits & 0x8000U) != 0U)
    {
        value -= 65536L;
    }
    return (int16_t)value;
}

LIS3DSH_Result LIS3DSH_Init(LIS3DSH_Device *device)
{
    if (!device_valid(device))
    {
        return LIS3DSH_INVALID_ARGUMENT;
    }

    device->initialized = false;
    device->who_am_i = 0U;
    device->last_hal_status = HAL_OK;

    /* Establish the idle clock level before selecting the sensor. */
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);
    __HAL_SPI_ENABLE(device->spi);
    HAL_Delay(20U);

    LIS3DSH_Result result = read_registers(
        device, REG_WHO_AM_I, &device->who_am_i, 1U);
    if (result != LIS3DSH_OK)
    {
        return result;
    }
    if (device->who_am_i != EXPECTED_ID)
    {
        return LIS3DSH_BAD_ID;
    }

    /* Stop acquisition before changing the configuration. */
    result = write_checked(device, REG_CTRL4, 0x07U);
    if (result != LIS3DSH_OK) return result;

    /* Clear interrupt routing retained across MCU-only resets or mode changes. */
    result = write_checked(device, 0x23U, 0x00U);
    if (result != LIS3DSH_OK) return result;

    /* 50 Hz bandwidth, +/-2 g range, self-test off, four-wire SPI. */
    result = write_checked(device, REG_CTRL5, 0xC0U);
    if (result != LIS3DSH_OK) return result;

    /* Enable address increment; disable FIFO and its interrupt routing. */
    result = write_checked(device, REG_CTRL6, 0x10U);
    if (result != LIS3DSH_OK) return result;

    /* Enable 100 Hz acquisition, BDU and all three axes. */
    result = write_checked(device, REG_CTRL4, 0x6FU);
    if (result != LIS3DSH_OK) return result;

    /* Drain any sample retained from a previous MCU-only reset. */
    uint8_t discard[6];
    result = read_registers(device, REG_OUT_X_L, discard, sizeof(discard));
    if (result != LIS3DSH_OK) return result;

    device->initialized = true;
    return LIS3DSH_OK;
}

LIS3DSH_Result LIS3DSH_ReadSample(LIS3DSH_Device *device,
                                  LIS3DSH_Sample *sample)
{
    if (!device_valid(device) || sample == NULL)
    {
        return LIS3DSH_INVALID_ARGUMENT;
    }
    if (!device->initialized)
    {
        return LIS3DSH_NOT_INITIALIZED;
    }

    uint8_t status = 0U;
    LIS3DSH_Result result = read_registers(device, REG_STATUS, &status, 1U);
    if (result != LIS3DSH_OK) return result;

    if ((status & STATUS_NEW_XYZ) == 0U)
    {
        return LIS3DSH_NO_DATA;
    }

    uint8_t data[6];
    result = read_registers(device, REG_OUT_X_L, data, sizeof(data));
    if (result != LIS3DSH_OK) return result;

    LIS3DSH_Sample next = {0};
    next.x_raw = decode_le16(data[0], data[1]);
    next.y_raw = decode_le16(data[2], data[3]);
    next.z_raw = decode_le16(data[4], data[5]);
    next.x_ug = (int32_t)next.x_raw * SENSITIVITY_UG;
    next.y_ug = (int32_t)next.y_raw * SENSITIVITY_UG;
    next.z_ug = (int32_t)next.z_raw * SENSITIVITY_UG;
    next.sensor_status = status;
    next.overrun = (status & STATUS_OVERRUN) != 0U;
    next.read_time_ms = HAL_GetTick();
    *sample = next;
    return LIS3DSH_OK;
}

const char *LIS3DSH_ResultString(LIS3DSH_Result result)
{
    switch (result)
    {
        case LIS3DSH_OK: return "OK";
        case LIS3DSH_NO_DATA: return "NO_DATA";
        case LIS3DSH_BUS_ERROR: return "BUS_ERROR";
        case LIS3DSH_BAD_ID: return "BAD_ID";
        case LIS3DSH_CONFIG_ERROR: return "CONFIG_ERROR";
        case LIS3DSH_INVALID_ARGUMENT: return "INVALID_ARGUMENT";
        case LIS3DSH_NOT_INITIALIZED: return "NOT_INITIALIZED";
        default: return "UNKNOWN";
    }
}
