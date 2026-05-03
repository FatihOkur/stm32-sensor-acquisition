#include "acquisition.h"

#if ACQ_MODE == ACQ_MODE_POLLING
#include "app_polling.h"
#elif ACQ_MODE == ACQ_MODE_IT
#include "app_it.h"
#else
#include "app_dma.h"
#endif

void Acquisition_Init(SPI_HandleTypeDef *spi,I2C_HandleTypeDef *i2c,UART_HandleTypeDef *uart)
{
#if ACQ_MODE == ACQ_MODE_POLLING
    /* One CubeMX configuration can serve both modes without EXTI overhead in polling. */
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    AppPolling_Init(spi,i2c,uart);
#elif ACQ_MODE == ACQ_MODE_IT
    /* GPIO routing, IRQ handlers and enabled NVIC entries must come from CubeMX. */
    AppIT_Init(spi,i2c,uart);
#else
    AppDMA_Init(spi,i2c,uart);
#endif
}

void Acquisition_Process(void)
{
#if ACQ_MODE == ACQ_MODE_POLLING
    AppPolling_Process();
    HAL_Delay(1U);
#elif ACQ_MODE == ACQ_MODE_IT
    AppIT_Process();
    __WFI();
#else
    AppDMA_Process();
    __WFI();
#endif
}
