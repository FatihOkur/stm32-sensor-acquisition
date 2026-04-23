#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MPU6050_OK = 0,
    MPU6050_NO_DATA,
    MPU6050_BUS_ERROR,
    MPU6050_BAD_ID,
    MPU6050_CONFIG_ERROR,
    MPU6050_INVALID_ARGUMENT,
    MPU6050_NOT_INITIALIZED
} MPU6050_Result;

typedef struct
{
    I2C_HandleTypeDef *i2c;
    uint8_t address_7bit;
    uint8_t who_am_i;
    uint8_t failed_register;
    HAL_StatusTypeDef last_hal_status;
    bool initialized;
} MPU6050_Device;

typedef struct
{
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    int16_t temperature_raw;
    int32_t ax_mg, ay_mg, az_mg;
    int32_t gx_mdps, gy_mdps, gz_mdps;
    int32_t temperature_mdegc;
    uint32_t read_time_ms;
} MPU6050_Sample;

/* Initialize for 100 Hz, +/-2 g, +/-250 degrees/s and DLPF setting 3. */
MPU6050_Result MPU6050_Init(MPU6050_Device *device);

/* Read a new sample; leave the output unchanged on errors or no-data. */
MPU6050_Result MPU6050_ReadSample(MPU6050_Device *device,
                                MPU6050_Sample *sample);
const char *MPU6050_ResultString(MPU6050_Result result);

#endif
