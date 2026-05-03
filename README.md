# STM32 Sensor Acquisition

An embedded C project that acquires data from two sensors on an STM32F407 using selectable **polling, interrupt, and DMA** modes with the same hardware setup.

The goal is to develop SPI/I2C sensor drivers, understand how the three acquisition methods work, and establish a foundation for performance comparisons.

## Features

- Reads three-axis acceleration from the onboard **LIS3DSH** over SPI.
- Reads acceleration, angular velocity, and chip temperature from an external **MPU6050** over I2C.
- Converts raw measurements into physical units and sends periodic summaries to a PC over UART.
- Tracks sample counts, sample age, and errors. Interrupt and DMA modes also report interrupt counts and transfer states.

Both sensors are configured for 100 Hz sampling and a ±2 g acceleration range. The MPU6050 gyroscope range is ±250 °/s. UART output is a periodic summary of the latest measurements, not a stream containing every acquired sample.

## Hardware and Connections

- STM32F407G-DISC1 — MB997-F407VGT6-E01
- GY-521 / MPU6050 module
- CP2102 USB–TTL adapter, USB cables, and jumper wires
- STM32CubeIDE 1.19.0, STM32CubeF4 V1.28.3, C, and STM32 HAL

### Shared Pin Mapping

| Component / signal | STM32 pin | Description |
|---|---|---|
| LIS3DSH SCK / MISO / MOSI | PA5 / PA6 / PA7 | SPI1, onboard connections |
| LIS3DSH CS | PE3 | Active low, onboard connection |
| LIS3DSH INT1/DRDY | PE0 | EXTI0, onboard connection |
| MPU6050 SCL | PB8 | I2C1 |
| MPU6050 SDA | PB7 | I2C1 |
| MPU6050 INT | PD2 | EXTI2 |
| MPU6050 VCC / GND | Board 3V / GND | Shared ground |
| MPU6050 AD0 | GND | 7-bit I2C address: `0x68` |
| CP2102 RXD | PA2 | STM32 USART2 TX |
| CP2102 TXD | PA3 | STM32 USART2 RX; command reception is not implemented |
| CP2102 GND | GND | Shared ground |
| Green / red LED | PD12 / PD14 | Heartbeat / interrupt-DMA fault indicator |

Power the STM32 board through its own USB connection. Leave the CP2102 3.3 V, 5 V, and RST pins unconnected, and use 3.3 V UART logic levels. The GY-521 XDA/XCL pins are not used.

### One Configuration for All Modes

**The repository's current `.ioc` file supports all three modes.** Wiring and CubeMX settings can remain unchanged when switching modes.

| Feature | Polling | Interrupt | DMA |
|---|---|---|---|
| SPI1 / I2C1 data lines | Used | Used | Used |
| PE0 / PD2 data-ready interrupts | Not used | Used | Used |
| Sensor data transfer | Blocking HAL calls | HAL `_IT` calls | HAL `_DMA` calls |
| UART transmission | Blocking | Interrupt | Interrupt |

The MPU6050 INT connection is optional for polling and can remain connected. The polling application disables the relevant EXTI and SPI/I2C/UART NVIC interrupts. DMA streams remain configured but are started for transfers only in DMA mode. The PA0 user button is configured as a GPIO input so that EXTI0 is available for PE0.

Peripheral settings: SPI mode 3, 8-bit transfers, prescaler 128; I2C at 100 kHz; UART at 115200 baud.

| DMA request | Stream | Channel |
|---|---|---|
| SPI1_RX | DMA2 Stream 0 | 3 |
| SPI1_TX | DMA2 Stream 3 | 3 |
| I2C1_RX | DMA1 Stream 0 | 1 |

All three DMA streams use Normal mode, byte data width, memory increment enabled, peripheral increment disabled, and FIFO disabled. `MX_DMA_Init()` is called before SPI/I2C initialization.

## Build, Run, and Select a Mode

1. Import the project into STM32CubeIDE using **Existing Projects into Workspace**.
2. Connect the hardware as shown above and retain the settings in `stm32_sensor_acquisition.ioc`.
3. Edit the existing mode selection in `Core/Inc/acquisition_config.h`:

```c
/* Select one mode, then rebuild and upload the firmware. */
#define ACQ_MODE ACQ_MODE_DMA
```

| Value | Acquisition method |
|---|---|
| `ACQ_MODE_POLLING` | Polling |
| `ACQ_MODE_IT` | Data-ready interrupt + interrupt-driven transfer |
| `ACQ_MODE_DMA` | Data-ready interrupt + DMA transfer |

Change the existing `ACQ_MODE` definition instead of adding a second one. A definition in the IDE's Preprocessor Symbols settings can override the selection in the header.

4. **Clean and build** the project, then flash the board. Mode selection happens at compile time; it cannot be changed from the serial terminal.
5. Open the CP2102 COM port in PuTTY or another serial terminal with **115200 baud, 8 data bits, no parity, 1 stop bit, and no flow control**.

The startup message identifies the selected mode:

```text
MODE=POLLING | DMA=OFF
MODE=EXTI+IT | DMA=OFF
MODE=EXTI+DMA | UART=IT
```

Only the line corresponding to the selected mode appears during a run. There is no need to delete source files, manually exclude them from the build, or maintain a separate CubeMX project for each mode.

## How the Code Works

`main.c` initializes the peripherals and calls the shared acquisition interface:

```c
Acquisition_Init(&hspi1, &hi2c1, &huart2);

while (1)
{
    Acquisition_Process();
}
```

`acquisition.c` dispatches calls to the selected application and manages its waiting behavior. Do not add an extra `HAL_Delay()` or `__WFI()` to the main loop.

- **Polling:** The main loop checks sensor data-ready status, performs blocking reads, and stores the latest measurements. UART summaries are sent every 500 ms.
- **Interrupt:** A sensor data-ready pin triggers an EXTI interrupt. The handler records a request, and the main loop starts the transfer. SPI/I2C interrupts move the data. The completion callback marks the result ready for processing in the main loop.
- **DMA:** The same data-ready mechanism triggers acquisition, while DMA moves sensor data into persistent SRAM buffers. The main loop processes the measurements after the completion callback. In this HAL implementation, the I2C address phase is synchronous; DMA handles the data phase.

Interrupt and DMA modes send UART summaries once per second using interrupt-driven transmission. Buffers remain unchanged until their transfers complete. Sensor initialization and register configuration use blocking calls in all three modes. Sensor transfer faults in interrupt/DMA modes latch the affected sensor in the FAULT state; automatic recovery is not implemented yet.

| File | Responsibility |
|---|---|
| `acquisition_config.h` | Acquisition mode selection |
| `acquisition.c` | Shared application interface and mode dispatch |
| `app_polling.c`, `app_it.c`, `app_dma.c` | Mode-specific application flow and UART output |
| `lis3dsh.c`, `mpu6050.c` | Shared sensor initialization and polling reads |
| `sensor_it.c`, `sensor_dma.c` | Asynchronous transfers, buffers, and state management |
| `stm32f4xx_it.c` | Interrupt handlers that call the HAL handlers |
| `stm32f4xx_hal_msp.c` | GPIO setup, DMA handle links, and peripheral interrupt configuration |

## Reading the Terminal Output

| Field | Meaning |
|---|---|
| `N` | Number of successfully processed samples |
| `A[mg]` | X/Y/Z acceleration; 1000 mg = 1 g |
| `G[mdps]` | X/Y/Z angular velocity; 1000 mdps = 1 °/s |
| `T[mC]` | Chip temperature; 27500 mC = 27.5 °C |
| `age` | Time since the last successful read, in milliseconds |
| `err` | Application error count |
| `ovr` | Number of observations of the LIS3DSH overrun flag |
| `irq` | Number of handled data-ready interrupts, not byte-transfer interrupts |
| `coal` / `nd` | Coalesced read requests / reads with no new data |
| `valid` / `stale` | A sample is available / no sample is available or its age exceeds 100 ms |
| `dtMax` | Maximum observed transfer duration in milliseconds, not CPU execution time |
| `state` | IDLE, ACTIVE, DONE, or FAULT |
| `HAL` / `E` | HAL status and error information for interrupt/DMA diagnostics |
| `logSkip` | Number of skipped UART summaries |

Some fields appear only in interrupt/DMA modes. `ACTIVE` means a transfer was in progress when the log was prepared; it is not an error by itself. Neither `ovr`, `coal`, nor the difference between the two sensors' `N` values provides an exact lost-sample count.
