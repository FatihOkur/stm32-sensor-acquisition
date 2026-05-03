#ifndef SENSOR_IT_H
#define SENSOR_IT_H
#include "lis3dsh.h"
#include "mpu6050.h"

typedef enum { SIT_OFF=0, SIT_IDLE, SIT_ACTIVE, SIT_DONE, SIT_FAULT } SensorIT_State;
typedef struct {
    volatile SensorIT_State state;
    volatile uint32_t edges, pending, coalesced, complete_ms;
    volatile uint32_t hal_error;
    uint32_t samples, no_data, overruns, start_ms;
    volatile uint32_t errors;
    uint32_t last_transfer_ms, max_transfer_ms;
    volatile HAL_StatusTypeDef last_hal;
} SensorIT_Status;
typedef struct {
    LIS3DSH_Device base;
    SensorIT_Status io;
    LIS3DSH_Result init_result;
    LIS3DSH_Sample latest;
    uint8_t tx[8], rx[8];
} LIS3DSH_IT;
typedef struct {
    MPU6050_Device base;
    SensorIT_Status io;
    MPU6050_Result init_result;
    MPU6050_Sample latest;
    uint8_t rx[15];
} MPU6050_IT;

/* Objects and their buffers must live for the entire acquisition session. */
void LIS3DSH_IT_Init(LIS3DSH_IT *device, SPI_HandleTypeDef *spi);
void MPU6050_IT_Init(MPU6050_IT *device, I2C_HandleTypeDef *i2c);
/* Call frequently from the main loop; never call the polling readers concurrently. */
void LIS3DSH_IT_Process(LIS3DSH_IT *device);
void MPU6050_IT_Process(MPU6050_IT *device);
/* Call from the corresponding EXTI and HAL callbacks. */
void SensorIT_DataReady(SensorIT_Status *status);
void LIS3DSH_IT_Complete(LIS3DSH_IT *device);
void LIS3DSH_IT_Error(LIS3DSH_IT *device);
void MPU6050_IT_Complete(MPU6050_IT *device);
void MPU6050_IT_Error(MPU6050_IT *device);
const char *SensorIT_StateName(SensorIT_State state);
#endif
