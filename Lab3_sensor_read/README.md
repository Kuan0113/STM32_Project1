# Lab 3 Sensor Readout - Baremetal & RTOS Task

This lab implements Inter-Integrated Circuit ($\text{I}^2\text{C}$) communication to read data from an external sensor and utilizes a Real-Time Operating System (RTOS) to manage the sensor reading task concurrently. The project is specifically configured for the **STM32 Nucleo-L476RG** board using the **CMSIS-RTOS version 2** wrapper over the **FreeRTOS** kernel.

---

## 1. Objectives

The primary goals of this laboratory work were:

1.  **Implement Robust $\text{I}^2\text{C}$ Communication:** Configure the $\text{STM32}$'s $\text{I}^2\text{C}$ peripheral to initialize and read 16-bit Red, Green, and Blue (RGB) light data from the **ISL29125 RGB Light Sensor**.
2.  **RTOS Integration:** Set up the FreeRTOS kernel via the CMSIS-RTOS V2 API to handle the sensor interaction in a dedicated, scheduled task.
3.  **Task Separation:** Define and utilize two concurrent $\text{RTOS}$ threads:
    * `ReadRGB`: To handle periodic sensor data acquisition and $\text{UART}$ output.
    * `ControlLogicTas`: A low-priority placeholder task for future application control logic.

---

## 2. Hardware and Software Environment

### Hardware Used:

| Component | Detail |
| :--- | :--- |
| **Microcontroller Board** | **STM32 Nucleo-L476RG** |
| **Microcontroller** | STM32L476RGT6 ($\text{ARM Cortex-M4}$) |
| **Sensor** | $\text{ISL29125}$ RGB Color Light Sensor |
| **I²C Port** | $\text{I}^2\text{C1}$ (SCL: **PB6**, SDA: **PB7**) |
| **UART Port** | $\text{USART2}$ ($\text{ST-Link VCP}$) |
| **UART Baud Rate** | **115200** |
| **Sensor I²C Address** | `0x44` (7-bit) or `0x88` (8-bit $\text{HAL}$ address) |

### Software Used:

* **IDE:** STM32CubeIDE
* **RTOS:** FreeRTOS (via CMSIS-RTOS V2)
* **Configuration:** STM32CubeMX
* **Terminal Emulator:** PuTTY at **115200 Baud**.

---

## 3. Implementation Details

### $\text{I}^2\text{C}$ Sensor Driver Functions

The core sensor logic is encapsulated in custom functions that abstract the low-level $\text{I}^2\text{C}$ $\text{HAL}$ calls.

| Function Prototype | Description |
| :--- | :--- |
| `ISL29125_WriteRegister` | **Writes one byte of data** to a specified register address (`reg`) on the $\text{ISL29125}$ sensor. Internally uses `HAL_I2C_Master_Transmit()`. This is essential for configuration and setup. |
| `ISL29125_ReadRegister` | **Reads one byte of data** from a specified register address (`reg`). This involves a two-step $\text{I}^2\text{C}$ transaction: first transmitting the register address, then receiving the data byte into the `*value` pointer. |
| `ISL29125_Init` | **Initializes the sensor** by writing configuration bytes to set the sensor mode (e.g., RGB mode) and $\text{ADC}$ resolution (e.g., 16-bit). It ensures the sensor is ready to provide valid readings. |
| `ISL29125_ReadRGBPercent` | **Reads and processes the main sensor data.** It reads the high and low bytes for the Red, Green, and Blue channels, combines them into 16-bit raw values, and then converts these values into a **0-100%** percentage range. |

### FreeRTOS Task Structure

The application's sensor reporting is isolated within the high-priority `ReadRGB` task to ensure predictable execution.

```c
/* StartReadRGB Task Logic */
void StartReadRGB(void *argument)
{
    // The task runs perpetually
    for(;;)
    {
        // 1. Read and convert RGB sensor values
        if (ISL29125_ReadRGBPercent(&r, &g, &b) == HAL_OK) {
            // 2. Format the output string
            int len = snprintf(msg, sizeof(msg), "R=%d%% G=%d%% B=%d%%\r\n", r, g, b);
            
            // 3. Transmit the data via UART
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
        }
        
        // 4. Suspend the task for 500ms, allowing other tasks (like ControlLogicTas) to run
        osDelay(500); 
    }
}
```

## 4. Results and Demonstration
The project successfully initializes the STM32L476RG's $\text{I}^2\text{C}$ peripheral and the FreeRTOS kernel. The ReadRGB task runs every half-second, polling the ISL29125 sensor for the current Red, Green, and Blue light intensity, which is displayed in the serial terminal.

Expected Output in Serial Terminal:

```
R=40% G=60% B=55%
R=41% G=61% B=56%
R=38% G=59% B=54%
...
```
