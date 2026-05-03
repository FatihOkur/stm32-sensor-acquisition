#ifndef SENSOR_DMA_H
#define SENSOR_DMA_H
#include "lis3dsh.h"
#include "mpu6050.h"

typedef enum { SDMA_OFF=0, SDMA_IDLE, SDMA_ACTIVE, SDMA_DONE, SDMA_FAULT } SensorDMA_State;
typedef struct {
    volatile SensorDMA_State state;
    volatile uint32_t edges, pending, coalesced, complete_ms;
    volatile uint32_t hal_error;
    uint32_t samples, no_data, overruns, start_ms;
    volatile uint32_t errors;
    uint32_t last_transfer_ms, max_transfer_ms;
    volatile HAL_StatusTypeDef last_hal;
} SensorDMA_Status;
typedef struct {
    LIS3DSH_Device base;
    SensorDMA_Status io;
    LIS3DSH_Result init_result;
    LIS3DSH_Sample latest;
    _Alignas(4) uint8_t tx[8];
    _Alignas(4) uint8_t rx[8];
} LIS3DSH_DMA;
typedef struct {
    MPU6050_Device base;
    SensorDMA_Status io;
    MPU6050_Result init_result;
    MPU6050_Sample latest;
    _Alignas(4) uint8_t rx[15];
} MPU6050_DMA;

/* Objects and their buffers must live for the entire acquisition session. */
void LIS3DSH_DMA_Init(LIS3DSH_DMA *device, SPI_HandleTypeDef *spi);
void MPU6050_DMA_Init(MPU6050_DMA *device, I2C_HandleTypeDef *i2c);
/* Call frequently from the main loop; never call the polling readers concurrently. */
void LIS3DSH_DMA_Process(LIS3DSH_DMA *device);
void MPU6050_DMA_Process(MPU6050_DMA *device);
/* Call from the corresponding EXTI and HAL callbacks. */
void SensorDMA_DataReady(SensorDMA_Status *status);
void LIS3DSH_DMA_Complete(LIS3DSH_DMA *device);
void LIS3DSH_DMA_Error(LIS3DSH_DMA *device);
void MPU6050_DMA_Complete(MPU6050_DMA *device);
void MPU6050_DMA_Error(MPU6050_DMA *device);
const char *SensorDMA_StateName(SensorDMA_State state);
#endif
