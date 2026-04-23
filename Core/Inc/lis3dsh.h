#ifndef LIS3DSH_H
#define LIS3DSH_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LIS3DSH_OK = 0,
    LIS3DSH_NO_DATA,
    LIS3DSH_BUS_ERROR,
    LIS3DSH_BAD_ID,
    LIS3DSH_CONFIG_ERROR,
    LIS3DSH_INVALID_ARGUMENT,
    LIS3DSH_NOT_INITIALIZED
} LIS3DSH_Result;

typedef struct
{
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    HAL_StatusTypeDef last_hal_status;
    uint8_t who_am_i;
    bool initialized;
} LIS3DSH_Device;

typedef struct
{
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    int32_t x_ug;
    int32_t y_ug;
    int32_t z_ug;
    uint8_t sensor_status;
    bool overrun;
    uint32_t read_time_ms;
} LIS3DSH_Sample;

/* Initialize one device for 100 Hz, +/-2 g, BDU and 50 Hz bandwidth. */
LIS3DSH_Result LIS3DSH_Init(LIS3DSH_Device *device);

/* Poll for a new sample. The output is unchanged unless LIS3DSH_OK is returned. */
LIS3DSH_Result LIS3DSH_ReadSample(LIS3DSH_Device *device,
                                  LIS3DSH_Sample *sample);

const char *LIS3DSH_ResultString(LIS3DSH_Result result);

#endif
