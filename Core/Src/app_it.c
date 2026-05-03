#include "acquisition_config.h"
#if ACQ_MODE == ACQ_MODE_IT
#include "app_it.h"
#include "sensor_it.h"
#include <stdio.h>

static LIS3DSH_IT lis;
static MPU6050_IT mpu;
static UART_HandleTypeDef *console;
static char uart_buffer[768];
static volatile uint32_t uart_busy, uart_fault;
static uint32_t uart_start_ms, last_log_ms, skipped_logs;

static void send_text(int length)
{
    if (length <= 0 || length >= (int)sizeof(uart_buffer)) { ++skipped_logs; return; }
    uart_start_ms = HAL_GetTick();
    uart_busy = 1U;
    __DMB();
    if (HAL_UART_Transmit_IT(console,(uint8_t *)uart_buffer,(uint16_t)length)!=HAL_OK) {
        uart_fault=1U;
        /* Keep the buffer reserved after a failed start. */
    }
}
void AppIT_Init(SPI_HandleTypeDef *spi,I2C_HandleTypeDef *i2c,UART_HandleTypeDef *uart)
{
    console=uart;
    MPU6050_IT_Init(&mpu,i2c);
    LIS3DSH_IT_Init(&lis,spi);
    last_log_ms=HAL_GetTick();
    int length=snprintf(uart_buffer,sizeof(uart_buffer),
        "MODE=EXTI+IT | DMA=OFF\r\n"
        "LIS init=%s ID=0x%02X HAL=%u | DRDY=PE0 CTRL3=0xE8\r\n"
        "MPU init=%s ID=0x%02X HAL=%u reg=0x%02X | DRDY=PD2\r\n",
        LIS3DSH_ResultString(lis.init_result),(unsigned)lis.base.who_am_i,
        (unsigned)lis.base.last_hal_status,
        MPU6050_ResultString(mpu.init_result),(unsigned)mpu.base.who_am_i,
        (unsigned)mpu.base.last_hal_status,(unsigned)mpu.base.failed_register);
    send_text(length);
}
void AppIT_Process(void)
{
    LIS3DSH_IT_Process(&lis);
    MPU6050_IT_Process(&mpu);
    uint32_t now=HAL_GetTick();
    /* Never reuse a UART buffer if its completion callback is missing. */
    if (uart_busy && (uint32_t)(now-uart_start_ms)>100U) uart_fault=1U;
    if (uart_fault || lis.io.state==SIT_FAULT || mpu.io.state==SIT_FAULT)
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
    if ((uint32_t)(now-last_log_ms)<1000U) return;
    last_log_ms=now;
    HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
    if (uart_busy || uart_fault) { ++skipped_logs; return; }

    uint32_t lis_age=lis.io.samples ? now-lis.latest.read_time_ms : 0U;
    uint32_t mpu_age=mpu.io.samples ? now-mpu.latest.read_time_ms : 0U;
    int length=snprintf(uart_buffer,sizeof(uart_buffer),
        "LIS N=%lu irq=%lu | A[mg]=%ld,%ld,%ld | valid=%u age=%lu stale=%u"
        " | err=%lu ovr=%lu coal=%lu nd=%lu dtMax=%lu | state=%s HAL=%u E=0x%08lX\r\n"
        "MPU N=%lu irq=%lu | A[mg]=%ld,%ld,%ld G[mdps]=%ld,%ld,%ld T[mC]=%ld"
        " | valid=%u age=%lu stale=%u | err=%lu coal=%lu nd=%lu dtMax=%lu"
        " | state=%s HAL=%u E=0x%08lX | logSkip=%lu\r\n",
        (unsigned long)lis.io.samples,(unsigned long)lis.io.edges,
        (long)(lis.latest.x_ug/1000L),(long)(lis.latest.y_ug/1000L),(long)(lis.latest.z_ug/1000L),
        (unsigned)(lis.io.samples!=0U),(unsigned long)lis_age,(unsigned)(lis.io.samples==0U || lis_age>100U),
        (unsigned long)lis.io.errors,(unsigned long)lis.io.overruns,
        (unsigned long)lis.io.coalesced,(unsigned long)lis.io.no_data,(unsigned long)lis.io.max_transfer_ms,
        SensorIT_StateName(lis.io.state),(unsigned)lis.io.last_hal,(unsigned long)lis.io.hal_error,
        (unsigned long)mpu.io.samples,(unsigned long)mpu.io.edges,
        (long)mpu.latest.ax_mg,(long)mpu.latest.ay_mg,(long)mpu.latest.az_mg,
        (long)mpu.latest.gx_mdps,(long)mpu.latest.gy_mdps,(long)mpu.latest.gz_mdps,
        (long)mpu.latest.temperature_mdegc,
        (unsigned)(mpu.io.samples!=0U),(unsigned long)mpu_age,(unsigned)(mpu.io.samples==0U || mpu_age>100U),
        (unsigned long)mpu.io.errors,(unsigned long)mpu.io.coalesced,(unsigned long)mpu.io.no_data,
        (unsigned long)mpu.io.max_transfer_ms,SensorIT_StateName(mpu.io.state),
        (unsigned)mpu.io.last_hal,(unsigned long)mpu.io.hal_error,(unsigned long)skipped_logs);
    send_text(length);
}

/* HAL callbacks are defined once here; do not duplicate them in main.c. */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin==GPIO_PIN_0) SensorIT_DataReady(&lis.io);
    if (pin==GPIO_PIN_2) SensorIT_DataReady(&mpu.io);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{ if (spi==lis.base.spi) LIS3DSH_IT_Complete(&lis); }
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi)
{ if (spi==lis.base.spi) LIS3DSH_IT_Error(&lis); }
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *i2c)
{ if (i2c==mpu.base.i2c) MPU6050_IT_Complete(&mpu); }
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *i2c)
{ if (i2c==mpu.base.i2c) MPU6050_IT_Error(&mpu); }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart==console) { __DMB(); uart_busy=0U; }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{ if (uart==console) uart_fault=1U; }
#endif
