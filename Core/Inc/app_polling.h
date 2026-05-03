#ifndef APP_POLLING_H
#define APP_POLLING_H
#include "stm32f4xx_hal.h"
void AppPolling_Init(SPI_HandleTypeDef *spi,I2C_HandleTypeDef *i2c,UART_HandleTypeDef *uart);
void AppPolling_Process(void);
#endif
