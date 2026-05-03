#ifndef ACQUISITION_H
#define ACQUISITION_H
#include "acquisition_config.h"
#include "stm32f4xx_hal.h"

/* Call once after CubeMX initialization. Mode changes require a new build/reset. */
void Acquisition_Init(SPI_HandleTypeDef *spi,I2C_HandleTypeDef *i2c,UART_HandleTypeDef *uart);
/* Owns the mode-specific wait; do not add HAL_Delay or __WFI in main. */
void Acquisition_Process(void);
#endif
