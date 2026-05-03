#include "acquisition_config.h"
#if ACQ_MODE == ACQ_MODE_DMA
#include "sensor_dma.h"
#include <string.h>

#define TRANSFER_DEADLINE_MS 20U
#define SOFTWARE_TIMEOUT 0x80000000UL
#define DMA_CONFIGURATION_ERROR 0x40000000UL

static bool dma_valid(DMA_HandleTypeDef *dma, void *parent,
                      uint32_t direction, uint32_t channel)
{
    return dma != NULL && dma->Instance != NULL && dma->Parent == parent &&
        dma->Init.Channel == channel && dma->Init.Direction == direction &&
        dma->Init.Mode == DMA_NORMAL && dma->Init.PeriphInc == DMA_PINC_DISABLE &&
        dma->Init.MemInc == DMA_MINC_ENABLE &&
        dma->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE &&
        dma->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE &&
        HAL_DMA_GetState(dma) == HAL_DMA_STATE_READY;
}
static bool dma_memory(const void *buffer, size_t length)
{
#if defined(__arm__) || defined(__thumb__)
    /* STM32F407 DMA can access SRAM1/2, but not the 0x10000000 CCM region. */
    uintptr_t address = (uintptr_t)buffer;
    return address >= 0x20000000UL && address < 0x20020000UL &&
           length <= (size_t)(0x20020000UL - address);
#else
    /* Host tests use process memory; target placement is checked in the ELF. */
    return buffer != NULL && length != 0U;
#endif
}
static void stop_stream(DMA_HandleTypeDef *dma)
{
    if (dma == NULL || dma->Instance == NULL) return;
    __HAL_DMA_DISABLE_IT(dma, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
    /* FE is in a different register and must be disabled separately. */
    __HAL_DMA_DISABLE_IT(dma, DMA_IT_FE);
    __HAL_DMA_DISABLE(dma);
}
static void stop_spi(LIS3DSH_DMA *d)
{
    CLEAR_BIT(d->base.spi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    __HAL_SPI_DISABLE_IT(d->base.spi, SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR);
    stop_stream(d->base.spi->hdmarx);
    stop_stream(d->base.spi->hdmatx);
    __HAL_SPI_DISABLE(d->base.spi);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    /* Buffers remain reserved until reset; no synchronous abort wait is needed. */
    __DMB();
}
static void stop_i2c(MPU6050_DMA *d)
{
    CLEAR_BIT(d->base.i2c->Instance->CR2, I2C_CR2_DMAEN | I2C_CR2_LAST);
    __HAL_I2C_DISABLE_IT(d->base.i2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
    stop_stream(d->base.i2c->hdmarx);
    __HAL_I2C_DISABLE(d->base.i2c);
    __DMB();
}

static uint32_t lock(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}
static void unlock(uint32_t saved) { __set_PRIMASK(saved); }
static void fault(SensorDMA_Status *s, HAL_StatusTypeDef hal, uint32_t error)
{
    s->last_hal = hal;
    s->hal_error = error;
    ++s->errors;
    __DMB();
    s->state = SDMA_FAULT;
}
static int16_t signed16(uint16_t bits)
{
    int32_t value = (int32_t)bits;
    if ((bits & 0x8000U) != 0U) value -= 65536L;
    return (int16_t)value;
}
static int16_t le16(const uint8_t *p)
{ return signed16((uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8))); }
static int16_t be16(const uint8_t *p)
{ return signed16((uint16_t)(((uint16_t)p[0] << 8) | p[1])); }

void SensorDMA_DataReady(SensorDMA_Status *s)
{
    if (s->state == SDMA_OFF || s->state == SDMA_FAULT) return;
    ++s->edges;
    if (s->pending != 0U) ++s->coalesced;
    s->pending = 1U;
}
static bool take_request(SensorDMA_Status *s)
{
    uint32_t saved = lock();
    bool take = s->state == SDMA_IDLE && s->pending != 0U;
    if (take) s->pending = 0U;
    unlock(saved);
    return take;
}
static void completed(SensorDMA_Status *s)
{
    s->complete_ms = HAL_GetTick();
    __DMB();
    s->state = SDMA_DONE;
}
static void duration(SensorDMA_Status *s)
{
    s->last_transfer_ms = s->complete_ms - s->start_ms;
    if (s->last_transfer_ms > s->max_transfer_ms)
        s->max_transfer_ms = s->last_transfer_ms;
}

void LIS3DSH_DMA_Init(LIS3DSH_DMA *d, SPI_HandleTypeDef *spi)
{
    memset(d, 0, sizeof(*d));
    d->base.spi = spi; d->base.cs_port = GPIOE; d->base.cs_pin = GPIO_PIN_3;
    if (!dma_valid(spi->hdmarx,spi,DMA_PERIPH_TO_MEMORY,DMA_CHANNEL_3) ||
        !dma_valid(spi->hdmatx,spi,DMA_MEMORY_TO_PERIPH,DMA_CHANNEL_3) ||
        spi->hdmarx->Instance == spi->hdmatx->Instance ||
        !dma_memory(d->tx,sizeof(d->tx)) || !dma_memory(d->rx,sizeof(d->rx))) {
        d->init_result=LIS3DSH_CONFIG_ERROR; d->base.last_hal_status=HAL_ERROR;
        fault(&d->io,HAL_ERROR,DMA_CONFIGURATION_ERROR); return;
    }
    d->init_result = LIS3DSH_Init(&d->base);
    if (d->init_result != LIS3DSH_OK) {
        fault(&d->io, d->base.last_hal_status, HAL_SPI_GetError(spi)); return;
    }
    /* DR_EN | IEA | IEL | INT1_EN: active-high pulsed DRDY on INT1/PE0. */
    uint8_t write[2] = {0x23U, 0xE8U};
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_StatusTypeDef hal = HAL_SPI_Transmit(spi, write, 2U, 100U);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    if (hal == HAL_OK) {
        uint8_t tx[2] = {0xA3U, 0U}, rx[2] = {0U};
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        hal = HAL_SPI_TransmitReceive(spi, tx, rx, 2U, 100U);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
        if (hal == HAL_OK && rx[1] != 0xE8U) d->init_result = LIS3DSH_CONFIG_ERROR;
    }
    d->base.last_hal_status = hal;
    if (hal != HAL_OK) d->init_result = LIS3DSH_BUS_ERROR;
    if (d->init_result != LIS3DSH_OK) { fault(&d->io, hal, HAL_SPI_GetError(spi)); return; }
    d->tx[0] = 0x27U | 0x80U;
    d->io.state = SDMA_IDLE;
}
void MPU6050_DMA_Init(MPU6050_DMA *d, I2C_HandleTypeDef *i2c)
{
    memset(d, 0, sizeof(*d));
    d->base.i2c = i2c; d->base.address_7bit = 0x68U;
    if (!dma_valid(i2c->hdmarx,i2c,DMA_PERIPH_TO_MEMORY,DMA_CHANNEL_1) ||
        !dma_memory(d->rx,sizeof(d->rx))) {
        d->init_result=MPU6050_CONFIG_ERROR; d->base.last_hal_status=HAL_ERROR;
        fault(&d->io,HAL_ERROR,DMA_CONFIGURATION_ERROR); return;
    }
    /* The base initializer enables active-high 50 us data-ready pulses. */
    d->init_result = MPU6050_Init(&d->base);
    if (d->init_result != MPU6050_OK) {
        fault(&d->io, d->base.last_hal_status, HAL_I2C_GetError(i2c)); return;
    }
    d->io.state = SDMA_IDLE;
}
void LIS3DSH_DMA_Complete(LIS3DSH_DMA *d)
{
    if (d->io.state != SDMA_ACTIVE) return;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    completed(&d->io);
}
void MPU6050_DMA_Complete(MPU6050_DMA *d)
{ if (d->io.state == SDMA_ACTIVE) completed(&d->io); }
void LIS3DSH_DMA_Error(LIS3DSH_DMA *d)
{
    if (d->io.state != SDMA_ACTIVE) return;
    fault(&d->io, HAL_ERROR, HAL_SPI_GetError(d->base.spi));
    stop_spi(d);
}
void MPU6050_DMA_Error(MPU6050_DMA *d)
{
    if (d->io.state == SDMA_ACTIVE) {
        fault(&d->io, HAL_ERROR, HAL_I2C_GetError(d->base.i2c));
        stop_i2c(d);
    }
}
void LIS3DSH_DMA_Process(LIS3DSH_DMA *d)
{
    SensorDMA_Status *s = &d->io;
    if (s->state == SDMA_ACTIVE && (uint32_t)(HAL_GetTick()-s->start_ms) > TRANSFER_DEADLINE_MS) {
        /* Latch the fault and stop the peripheral before releasing its buffer. */
        uint32_t saved = lock();
        if (s->state == SDMA_ACTIVE) {
            fault(s, HAL_TIMEOUT, SOFTWARE_TIMEOUT);
            stop_spi(d);
        }
        unlock(saved);
    }
    if (s->state == SDMA_DONE) {
        __DMB(); duration(s);
        if ((d->rx[1] & 0x08U) != 0U) {
            LIS3DSH_Sample next = {0};
            next.x_raw=le16(&d->rx[2]); next.y_raw=le16(&d->rx[4]); next.z_raw=le16(&d->rx[6]);
            next.x_ug=(int32_t)next.x_raw*60L;
            next.y_ug=(int32_t)next.y_raw*60L;
            next.z_ug=(int32_t)next.z_raw*60L;
            next.sensor_status=d->rx[1]; next.overrun=(d->rx[1]&0x80U)!=0U;
            next.read_time_ms=s->complete_ms;
            d->latest=next; ++s->samples;
            if (next.overrun) ++s->overruns;
        } else ++s->no_data;
        s->state=SDMA_IDLE;
    }
    if (!take_request(s)) return;
    s->start_ms=HAL_GetTick(); s->state=SDMA_ACTIVE;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_StatusTypeDef hal=HAL_SPI_TransmitReceive_DMA(d->base.spi,d->tx,d->rx,8U);
    if (hal != HAL_OK) {
        uint32_t saved=lock();
        fault(s,hal,HAL_SPI_GetError(d->base.spi));
        stop_spi(d);
        unlock(saved);
    }
}
void MPU6050_DMA_Process(MPU6050_DMA *d)
{
    SensorDMA_Status *s=&d->io;
    if (s->state==SDMA_ACTIVE && (uint32_t)(HAL_GetTick()-s->start_ms)>TRANSFER_DEADLINE_MS) {
        uint32_t saved=lock();
        if (s->state==SDMA_ACTIVE) {
            fault(s,HAL_TIMEOUT,SOFTWARE_TIMEOUT);
            stop_i2c(d);
        }
        unlock(saved);
    }
    if (s->state==SDMA_DONE) {
        __DMB(); duration(s);
        if ((d->rx[0]&1U)!=0U) {
            MPU6050_Sample next={0};
            next.ax_raw=be16(&d->rx[1]); next.ay_raw=be16(&d->rx[3]); next.az_raw=be16(&d->rx[5]);
            next.temperature_raw=be16(&d->rx[7]);
            next.gx_raw=be16(&d->rx[9]); next.gy_raw=be16(&d->rx[11]); next.gz_raw=be16(&d->rx[13]);
            next.ax_mg=(int32_t)next.ax_raw*1000L/16384L;
            next.ay_mg=(int32_t)next.ay_raw*1000L/16384L;
            next.az_mg=(int32_t)next.az_raw*1000L/16384L;
            next.gx_mdps=(int32_t)next.gx_raw*1000L/131L;
            next.gy_mdps=(int32_t)next.gy_raw*1000L/131L;
            next.gz_mdps=(int32_t)next.gz_raw*1000L/131L;
            next.temperature_mdegc=36530L+(int32_t)next.temperature_raw*1000L/340L;
            next.read_time_ms=s->complete_ms;
            d->latest=next; ++s->samples;
        } else ++s->no_data;
        s->state=SDMA_IDLE;
    }
    if (!take_request(s)) return;
    /* Avoid the HAL's synchronous BUSY wait on an already stuck bus. */
    if (__HAL_I2C_GET_FLAG(d->base.i2c,I2C_FLAG_BUSY)!=RESET) {
        fault(s,HAL_BUSY,HAL_I2C_GetError(d->base.i2c)); return;
    }
    s->start_ms=HAL_GetTick(); s->state=SDMA_ACTIVE;
    HAL_StatusTypeDef hal=HAL_I2C_Mem_Read_DMA(d->base.i2c,
        (uint16_t)(d->base.address_7bit<<1),0x3AU,I2C_MEMADD_SIZE_8BIT,d->rx,15U);
    if (hal!=HAL_OK) {
        uint32_t saved=lock();
        fault(s,hal,HAL_I2C_GetError(d->base.i2c));
        stop_i2c(d);
        unlock(saved);
    }
}
const char *SensorDMA_StateName(SensorDMA_State state)
{
    switch(state) {
        case SDMA_OFF:return "OFF"; case SDMA_IDLE:return "IDLE";
        case SDMA_ACTIVE:return "ACTIVE"; case SDMA_DONE:return "DONE";
        case SDMA_FAULT:return "FAULT"; default:return "UNKNOWN";
    }
}
#endif
