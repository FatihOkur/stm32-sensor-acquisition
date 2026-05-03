#ifndef APP_DMA_H
#define APP_DMA_H
#include "stm32f4xx_hal.h"
/* Call once after CubeMX peripheral initialization, then call Process in main. */
void AppDMA_Init(SPI_HandleTypeDef *spi, I2C_HandleTypeDef *i2c, UART_HandleTypeDef *uart);
void AppDMA_Process(void);
#endif
