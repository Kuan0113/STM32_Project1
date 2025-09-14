# Lab 2 – LED Blink with UART Interval Input

## Objective

- Implement LED blinking on the STM32L476RG Nucleo board.  
- Allow the user to set and dynamically update the LED blink interval via UART.  
- Use the user button (B1) to trigger interval updates.  
- Provide real-time feedback of LED status (`LED ON` / `LED OFF`) over UART.

---

## Equipment

- STM32L476RG Nucleo Board  
- USB cable for power and UART connection  
- PC with PuTTY (or other serial terminal)  

---

## CubeMX Configuration

### GPIO

| Pin  | Function          | Mode          | Pull    |
|------|-----------------|---------------|--------|
| PA5  | LD2 LED          | Output Push-Pull | None |
| PC13 | User Button B1   | GPIO_EXTI13    | Pull-Up |

![](https://github.com/Kuan0113/STM32_Project1/blob/main/Lab2_Blinking_LED/image/gpio_setting.png)

### NVIC

- `EXTI15_10_IRQn` enabled for PC13 interrupt.
   
![](https://github.com/Kuan0113/STM32_Project1/blob/main/Lab2_Blinking_LED/image/nvic_setting.png)

### UART2

- Baud Rate: 115200  
- Data Bits: 8  
- Stop Bits: 1  
- Parity: None  
- Flow Control: None  

---

## Program Flow

1. **Startup Prompt**  
   - Program prints: `"Enter LED interval in ms:"`  
   - User types a value and presses Enter to set the initial LED blink interval.  

2. **Non-blocking LED Blink**  
   - LED toggles using `HAL_GetTick()` timing.  
   - Status is sent to UART: `"LED ON"` / `"LED OFF"`  

3. **Update Interval via Button**  
   - Pressing **B1** triggers: `"Enter new LED interval in ms:"`  
   - User can enter a new value to update the blinking interval.  

---

## User Code Sections

The following sections in [`main.c`](https://github.com/Kuan0113/STM32_Project1/blob/main/Lab2_Blinking_LED/Lab2_blink_Chen/Core/Src/main.c) marked with `/* USER CODE BEGIN */` and `/* USER CODE END */` contain all custom code for this project (Lab2_Blinking_LED). These markers indicate where CubeMX-generated code ends and custom additions begin.
 
### 1. USER CODE BEGIN PV

- **Purpose:** Declare global variables used in the program.  
- **Added code:**
```c
uint32_t blink_interval = 500;   // Default LED blink interval in ms
uint32_t last_toggle = 0;        // Timestamp of last LED toggle
```

### 2. USER CODE BEGIN 0

- **Purpose:** Define global helper functions and variables.  
- **Added code:**
  ```c
  uint32_t blink_interval = 500;
  uint32_t last_toggle = 0;

  void UART_Transmit(char *message)
  {
      HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
  }

  uint32_t UART_ReadNumber(void)
  {
      char rx_buffer[10] = {0};
      uint8_t index = 0;
      uint8_t ch = 0;

      while(1)
      {
          HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);
          if(ch == '\r') break;
          if(index < sizeof(rx_buffer)-1)
          {
              rx_buffer[index++] = ch;
              HAL_UART_Transmit(&huart2, &ch, 1, HAL_MAX_DELAY); // echo
          }
      }
      rx_buffer[index] = '\0';
      return atoi(rx_buffer);
  }

### 3. USER CODE BEGIN 2

- **Purpose: Prompt the user for the initial LED interval at startup.**
- **Added code:**
  ```c
  UART_Transmit("Enter LED interval in ms: ");
  uint32_t new_interval = UART_ReadNumber();
  if(new_interval > 0) blink_interval = new_interval;
  UART_Transmit("\r\nBlinking LED...\r\n");

### 3. USER CODE BEGIN 3

- **Purpose: Main loop with non-blocking LED blinking and UART status messages.**
- **Added code:**
  ```c
  while (1)
  {
      uint32_t current_time = HAL_GetTick();
  
      if(current_time - last_toggle >= blink_interval)
      {
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          last_toggle = current_time;
  
          if(HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin) == GPIO_PIN_SET)
              UART_Transmit("LED ON\r\n");
          else
              UART_Transmit("LED OFF\r\n");
      }
  }
### 4. USER CODE BEGIN 4

- **Purpose:
  
  `UART_Transmit(char *message)` Sends strings to the PC via UART.
  
  `UART_ReadNumber(void)`  Reads a numeric input from UART until Enter is pressed.
  
  `void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)` Handle PB1 button press to allow the user to change the LED interval at runtime.**
  
- **Added code:**
  ```c
  void UART_Transmit(char *message)
  {
      HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
  }
  
  uint32_t UART_ReadNumber(void)
  {
      char rx_buffer[10] = {0};
      uint8_t index = 0;
      uint8_t ch = 0;
  
      while(1)
      {
          HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);
          if(ch == '\r') break; // Stop at Enter
          if(index < sizeof(rx_buffer)-1)
          {
              rx_buffer[index++] = ch;
              HAL_UART_Transmit(&huart2, &ch, 1, HAL_MAX_DELAY); // echo
          }
      }
  
      rx_buffer[index] = '\0';
      uint32_t value = atoi(rx_buffer);
      return value; // Returns the integer entered by user
  }
  
  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  {
      if(GPIO_Pin == B1_Pin)
      {
          UART_Transmit("Enter new LED delay in ms: ");
          uint32_t new_delay = UART_ReadNumber();
          if(new_delay > 0)
          {
              blink_delay = new_delay;
              UART_Transmit("\r\nDelay updated\r\n");
          }
      }
  }



---

## Usage Instructions

1. Connect the Nucleo board to your PC
2. Use device manger confrim com.
   
   ![](https://github.com/Kuan0113/STM32_Project1/blob/main/Lab2_Blinking_LED/image/com_confirm.png)
3. Open PuTTY: 115200.
   
   ![PuTTY Setting](https://github.com/Kuan0113/STM32_Project1/blob/main/Lab2_Blinking_LED/image/putty.png)
4. Power on the board. The terminal shows:

   Setting delay **1000ms**

   ![](https://github.com/Kuan0113/STM32_Project1/blob/main/Lab2_Blinking_LED/image/1000ms.gif)

   Setting delay **500ms**
   
   ![](https://github.com/Kuan0113/STM32_Project1/blob/main/Lab2_Blinking_LED/image/500ms.gif)

   Setting delay **100ms**
   
   ![](https://github.com/Kuan0113/STM32_Project1/blob/main/Lab2_Blinking_LED/image/100ms.gif)
 
---


## References

1. **Blinking an LED with STM32**  
   STMicroelectronics Wiki – Step 2: Blink LED  
   [https://wiki.st.com/stm32mcu/wiki/STM32StepByStep:Step2_Blink_LED](https://wiki.st.com/stm32mcu/wiki/STM32StepByStep:Step2_Blink_LED)

2. **Introduction to UART with STM32**  
   STMicroelectronics Wiki – Step 3: Introduction to the UART  
   [https://wiki.st.com/stm32mcu/wiki/STM32StepByStep:Step3_Introduction_to_the_UART](https://wiki.st.com/stm32mcu/wiki/STM32StepByStep:Step3_Introduction_to_the_UART)

3. **Getting Started with UART on STM32**  
   STMicroelectronics Wiki – Getting Started with UART  
   [https://wiki.st.com/stm32mcu/wiki/Getting_started_with_UART](https://wiki.st.com/stm32mcu/wiki/Getting_started_with_UART)

4. **Push Button Handling with STM32**  
   Microcontrollers Lab – Push Button STM32 Blue Pill Tutorial  
   [https://microcontrollerslab.com/push-button-stm32-blue-pill-stm32cube-ide-tutorial/](https://microcontrollerslab.com/push-button-stm32-blue-pill-stm32cube-ide-tutorial/)

5. **STM32 GPIO Input Configuration**  
   ControllersTech – STM32 GPIO Input Configuration  
   [https://controllerstech.com/stm32-gpio-input-configuration/](https://controllerstech.com/stm32-gpio-input-configuration/)

6. **STM32 UART (USART) Tutorial + Examples**  
   DeepBlue Embedded – STM32 UART Tutorial  
   [https://deepbluembedded.com/stm32-usart-uart-tutorial/](https://deepbluembedded.com/stm32-usart-uart-tutorial/)

7. **Implementing UART Receive and Transmit Functions on STM32**  
   ST Community – Implementing UART Functions  
   [https://community.st.com/t5/stm32-mcus/implementing-uart-receive-and-transmit-functions-on-an-stm32/ta-p/694926](https://community.st.com/t5/stm32-mcus/implementing-uart-receive-and-transmit-functions-on-an-stm32/ta-p/694926)

8. **Push Button Example with STM32 Cube IDE**  
   Microcontrollers Lab – Push Button with STM32 Nucleo  
   [https://microcontrollerslab.com/push-button-with-stm32-nucleo-stm32cubeide/](https://microcontrollerslab.com/push-button-with-stm32-nucleo-stm32cubeide/)

9. **STM32 UART Hands-on Tutorial**  
   Leonardo Cavagnis – Firmware 101: STM32 UART Hands-on  
   [https://leonardocavagnis.medium.com/firmware-101-stm32-uart-hands-on-f5f89f78817d](https://leonardocavagnis.medium.com/firmware-101-stm32-uart-hands-on-f5f89f78817d)

10. **Push Button with STM32 Nucleo Using STM32CubeIDE**  
    Microcontrollers Lab – Push Button with STM32 Nucleo  
    [https://microcontrollerslab.com/push-button-with-stm32-nucleo-stm32cubeide/](https://microcontrollerslab.com/push-button-with-stm32-nucleo-stm32cubeide/)
