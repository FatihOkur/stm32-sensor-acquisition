#ifndef APP_IT_H
#define APP_IT_H
#include "stm32f4xx_hal.h"
/* Call once after CubeMX peripheral initialization, then call Process in main. */
void AppIT_Init(SPI_HandleTypeDef *spi, I2C_HandleTypeDef *i2c, UART_HandleTypeDef *uart);
void AppIT_Process(void);
#endif
