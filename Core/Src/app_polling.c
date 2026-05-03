#include "acquisition_config.h"
#if ACQ_MODE == ACQ_MODE_POLLING
#include "app_polling.h"
#include "lis3dsh.h"
#include "mpu6050.h"
#include <stdio.h>

static LIS3DSH_Device accel;
static MPU6050_Device imu;
static LIS3DSH_Result init_result;
static MPU6050_Result imu_init;
static LIS3DSH_Sample sample;
static MPU6050_Sample imu_sample;
static uint32_t sample_count, bus_errors, overrun_events;
static uint32_t imu_count, imu_errors, last_log_ms;
static char log_message[256];
static int log_length;
static UART_HandleTypeDef *poll_uart;
static I2C_HandleTypeDef *poll_i2c;

void AppPolling_Init(SPI_HandleTypeDef *spi,I2C_HandleTypeDef *i2c,UART_HandleTypeDef *uart)
{
    poll_uart=uart; poll_i2c=i2c;
    imu.i2c=i2c; imu.address_7bit=0x68U;
    imu_init=MPU6050_Init(&imu);
    /* Keep the existing reset-failure diagnostic. */
    if (imu_init!=MPU6050_OK) {
        log_length=snprintf(log_message,sizeof(log_message),
            "I2C INIT FAIL | SCL=%u SDA=%u | SR2=0x%04lX | state=0x%02lX error=0x%08lX\r\n",
            (unsigned)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8),
            (unsigned)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7),
            (unsigned long)i2c->Instance->SR2,
            (unsigned long)HAL_I2C_GetState(i2c),(unsigned long)HAL_I2C_GetError(i2c));
        if (log_length>0 && log_length<(int)sizeof(log_message))
            HAL_UART_Transmit(uart,(uint8_t*)log_message,(uint16_t)log_length,100U);
    }
    accel.spi=spi; accel.cs_port=GPIOE; accel.cs_pin=GPIO_PIN_3;
    init_result=LIS3DSH_Init(&accel);
    last_log_ms=HAL_GetTick();
    log_length=snprintf(log_message,sizeof(log_message),
        "MODE=POLLING | DMA=OFF\r\n"
        "LIS3DSH init=%s | ID=0x%02X | HAL=%u\r\n"
        "MPU6050 init=%s | ID=0x%02X | HAL=%u | reg=0x%02X\r\n",
        LIS3DSH_ResultString(init_result),(unsigned)accel.who_am_i,(unsigned)accel.last_hal_status,
        MPU6050_ResultString(imu_init),(unsigned)imu.who_am_i,
        (unsigned)imu.last_hal_status,(unsigned)imu.failed_register);
    if (log_length>0 && log_length<(int)sizeof(log_message))
        HAL_UART_Transmit(uart,(uint8_t*)log_message,(uint16_t)log_length,100U);
}

void AppPolling_Process(void)
{

	  /* Poll both devices independently; one failure must not stop the other. */
	  if (init_result == LIS3DSH_OK)
	  {
	      LIS3DSH_Result result = LIS3DSH_ReadSample(&accel, &sample);
	      if (result == LIS3DSH_OK)
	      {
	          ++sample_count;
	          if (sample.overrun) ++overrun_events;
	      }
	      else if (result != LIS3DSH_NO_DATA) ++bus_errors;
	  }

	  if (imu_init == MPU6050_OK)
	  {
	      MPU6050_Result result = MPU6050_ReadSample(&imu, &imu_sample);
	      if (result == MPU6050_OK) ++imu_count;
	      else if (result != MPU6050_NO_DATA)
	      {
	          ++imu_errors;

	          /* Report only the first read failure to avoid flooding the UART. */
	          if (imu_errors == 1U)
	          {
	              uint32_t hal_error = HAL_I2C_GetError(poll_i2c);
	              uint32_t sr2 = poll_i2c->Instance->SR2;

	              log_length = snprintf(
	                  log_message, sizeof(log_message),
	                  "MPU READ FAIL | HAL=%u reg=0x%02X"
	                  " | SCL=%u SDA=%u | SR2=0x%04lX error=0x%08lX\r\n",
	                  (unsigned)imu.last_hal_status,
	                  (unsigned)imu.failed_register,
	                  (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
	                  (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7),
	                  (unsigned long)sr2, (unsigned long)hal_error);

	              if (log_length > 0 && log_length < (int)sizeof(log_message))
	              {
	                  HAL_UART_Transmit(poll_uart, (uint8_t *)log_message,
	                                   (uint16_t)log_length, 100U);
	              }
	          }
	      }
	  }

	  uint32_t now = HAL_GetTick();
	  if ((uint32_t)(now - last_log_ms) >= 500U)
	  {
	      last_log_ms = now;
	      if (sample_count > 0U)
	      {
	          log_length = snprintf(log_message, sizeof(log_message),
	              "LIS N=%lu | A[mg]=%ld,%ld,%ld | age=%lu ms | err=%lu ovr=%lu\r\n",
	              (unsigned long)sample_count,
	              (long)(sample.x_ug / 1000L), (long)(sample.y_ug / 1000L),
	              (long)(sample.z_ug / 1000L),
	              (unsigned long)(now - sample.read_time_ms),
	              (unsigned long)bus_errors, (unsigned long)overrun_events);
	      }
	      else
	      {
	          log_length = snprintf(log_message, sizeof(log_message),
	              "LIS waiting | init=%s | err=%lu\r\n",
	              LIS3DSH_ResultString(init_result), (unsigned long)bus_errors);
	      }
	      if (log_length > 0 && log_length < (int)sizeof(log_message))
	      {
	          HAL_UART_Transmit(poll_uart, (uint8_t *)log_message,
	                           (uint16_t)log_length, 100U);
	      }

	      if (imu_count > 0U)
	      {
	          log_length = snprintf(log_message, sizeof(log_message),
	              "MPU N=%lu | A[mg]=%ld,%ld,%ld | G[mdps]=%ld,%ld,%ld | T[mC]=%ld | age=%lu ms | err=%lu\r\n",
	              (unsigned long)imu_count,
	              (long)imu_sample.ax_mg, (long)imu_sample.ay_mg, (long)imu_sample.az_mg,
	              (long)imu_sample.gx_mdps, (long)imu_sample.gy_mdps, (long)imu_sample.gz_mdps,
	              (long)imu_sample.temperature_mdegc,
	              (unsigned long)(HAL_GetTick() - imu_sample.read_time_ms),
	              (unsigned long)imu_errors);
	      }
	      else
	      {
	          log_length = snprintf(log_message, sizeof(log_message),
	              "MPU waiting | init=%s | err=%lu | HAL=%u | reg=0x%02X\r\n",
	              MPU6050_ResultString(imu_init), (unsigned long)imu_errors,
	              (unsigned)imu.last_hal_status, (unsigned)imu.failed_register);
	      }
	      if (log_length > 0 && log_length < (int)sizeof(log_message))
	      {
	          HAL_UART_Transmit(poll_uart, (uint8_t *)log_message,
	                           (uint16_t)log_length, 100U);
	      }
	      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	  }
}
#endif
