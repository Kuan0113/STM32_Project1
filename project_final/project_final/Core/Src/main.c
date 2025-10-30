/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int r;
    int g;
    int b;
    char color[16];
} SensorData_t;

// Command type for ActuatorTask
typedef enum {
    ACTUATOR_CMD_OFF = 0,
    ACTUATOR_CMD_ON = 1
} ActuatorCommand_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ISL29125_ADDR         (0x44 << 1)  // 7-bit address shifted for HAL

// Register map
#define ISL29125_REG_DEVICE_ID   0x00
#define ISL29125_REG_CONFIG1     0x01
#define ISL29125_REG_CONFIG2     0x02
#define ISL29125_REG_CONFIG3     0x03
#define ISL29125_REG_STATUS      0x08
#define ISL29125_REG_GREEN_L     0x09
#define ISL29125_REG_GREEN_H     0x0A
#define ISL29125_REG_RED_L       0x0B
#define ISL29125_REG_RED_H       0x0C
#define ISL29125_REG_BLUE_L      0x0D
#define ISL29125_REG_BLUE_H      0x0E

// Config bits
#define CONFIG1_MODE_RGB_16BIT   0x05
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1; // For Buzzer PWM
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Queues
QueueHandle_t sensorQueue;      // Sensor -> Control (Raw data)
QueueHandle_t actuatorQueue;    // Control -> Actuator (Commands)
QueueHandle_t LogQueue;         // ALL Tasks -> PrintTask (Text messages)

// Semaphores & Mutexes
SemaphoreHandle_t LogSemaphore; // Signals PrintTask [cite: 5757, 5785]
SemaphoreHandle_t LogMutex;     // Protects UART (the "log file") [cite: 5758, 5807]

char selectedColor[16] = "WHITE";

// --- Static buffers for log messages ---
// It's critical that tasks send pointers to persistent memory (static), not stack variables.
static char sensorPrintMsg[128];
static char controlLogMsg[128];
static char actuatorLogMsg[64];
static char menuPrintMsg[256];
static char menuSelectMsg[64];
static const char *invalidMsg = "\r\n> Invalid choice\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);

// Task Prototypes
void StartReadRGB(void *argument);
void StartControlTask(void *argument);
void StartActuatorTask(void *argument);
void StartPrintTask(void *argument);

/* USER CODE BEGIN PFP */
HAL_StatusTypeDef ISL29125_WriteRegister(uint8_t reg, uint8_t value);
HAL_StatusTypeDef ISL29125_ReadRegister(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef ISL29125_Init(void);
HAL_StatusTypeDef ISL29125_ReadRGB255(int *r_val, int *g_val, int *b_val);
void Actuator_SetLED(uint8_t state);
void Actuator_SetBuzzer(uint8_t state);
const char* DetectColor(int r, int g, int b);
void SetSelectedColor(const char* color);
void HandleMenuInteraction(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ISL29125_Init();
  /* USER CODE END 2 */

  /* Create the semaphores(s) */
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  // Create the binary semaphore used to signal the PrintTask
  LogSemaphore = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the mutex(es) */
  /* USER CODE BEGIN RTOS_MUTEXES */
  // Create the mutex to protect the UART
  LogMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEXES */

  /* Create the queue(s) */
  /* USER CODE BEGIN RTOS_QUEUES */
  sensorQueue = xQueueCreate(1, sizeof(SensorData_t));
  actuatorQueue = xQueueCreate(5, sizeof(ActuatorCommand_t));
  // Create the log queue to hold text-based messages (pointers to strings)
  LogQueue = xQueueCreate(15, sizeof(char*));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* USER CODE BEGIN RTOS_THREADS */
  // Priority: Print > Sensor/Actuator > Control
  xTaskCreate(StartPrintTask, "PrintTask", 256, NULL, 3, NULL);
  xTaskCreate(StartReadRGB, "SensorTask", 256, NULL, 2, NULL);
  xTaskCreate(StartActuatorTask, "ActuatorTask", 256, NULL, 2, NULL);
  xTaskCreate(StartControlTask, "ControlTask", 256, NULL, 1, NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  vTaskStartScheduler();

  while (1) {}
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) Error_Handler();
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

/**
  * @brief TIM1 Initialization Function for PWM
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 29;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 987;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 494; // 50% duty cycle
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Out_LED_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LD2_Pin|Out_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


/* USER CODE BEGIN 4 */

void SetSelectedColor(const char* color) {
    strncpy(selectedColor, color, sizeof(selectedColor)-1);
    selectedColor[sizeof(selectedColor)-1] = '\0';
}

HAL_StatusTypeDef ISL29125_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return HAL_I2C_Master_Transmit(&hi2c1, ISL29125_ADDR, buf, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ISL29125_ReadRegister(uint8_t reg, uint8_t *value) {
    HAL_I2C_Master_Transmit(&hi2c1, ISL29125_ADDR, &reg, 1, HAL_MAX_DELAY);
    return HAL_I2C_Master_Receive(&hi2c1, ISL29125_ADDR, value, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ISL29125_Init(void) {
    return ISL29125_WriteRegister(ISL29125_REG_CONFIG1, CONFIG1_MODE_RGB_16BIT);
}

HAL_StatusTypeDef ISL29125_ReadRGB255(int *r_val, int *g_val, int *b_val) {
    uint8_t lo, hi; uint16_t r_raw, g_raw, b_raw;
    ISL29125_ReadRegister(ISL29125_REG_GREEN_L, &lo); ISL29125_ReadRegister(ISL29125_REG_GREEN_H, &hi); g_raw = (hi << 8) | lo;
    ISL29125_ReadRegister(ISL29125_REG_RED_L, &lo); ISL29125_ReadRegister(ISL29125_REG_RED_H, &hi); r_raw = (hi << 8) | lo;
    ISL29125_ReadRegister(ISL29125_REG_BLUE_L, &lo); ISL29125_ReadRegister(ISL29125_REG_BLUE_H, &hi); b_raw = (hi << 8) | lo;
    *r_val = (r_raw * 255) / 65535; *g_val = (g_raw * 255) / 65535; *b_val = (b_raw * 255) / 65535;
    return HAL_OK;
}

void Actuator_SetLED(uint8_t state) {
    HAL_GPIO_WritePin(GPIOA, Out_LED_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Actuator_SetBuzzer(uint8_t state) {
    if (state) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    } else {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    }
}

const char* DetectColor(int r, int g, int b) {
    // --- YELLOW ---
    if (r > 240 && g > 240 && b < 245) return "YELLOW";
    // --- BLUE (covers light/dark) ---
    if (r < 120 && g > 180 && b > 180) return "BLUE";
    // --- GREEN (covers light/dark) ---
    if (r > 100 && r < 200 && g > 230 && b > 150) return "GREEN";
    // --- RED ---
    if (r > 150 && g > 150 && b < 110) return "RED";
    // --- BLACK ---
    if (r < 70 && g > 100 && b > 70 && g - r > 40) return "BLACK";
    // --- WHITE
    if (r > 250 && g > 250 && b > 250) return "WHITE";

    return "UNKNOWN";
}

void HandleMenuInteraction(void)
{
    // Pointers to our static buffers
    char *pMenuMsg = menuPrintMsg;
    char *pSelectMsg = menuSelectMsg;
    const char *pInvalidMsg = invalidMsg; // Pointer to a const string is fine
    const char *newline = "\r\n"; // Also a const string

    // 1. Queue the menu for printing
    snprintf(pMenuMsg, 256, // Use the size of the buffer
            "\r\n--- SELECT A COLOR ---\r\n"
            "1: RED\r\n2: GREEN\r\n3: BLUE\r\n4: YELLOW\r\n5: BLACK\r\n6: WHITE\r\n"
            "Enter your choice followed by ENTER: ");
    xQueueSend(LogQueue, &pMenuMsg, 0);
    xSemaphoreGive(LogSemaphore);

    // 2. Give PrintTask time to run before we block for input
    vTaskDelay(pdMS_TO_TICKS(20));

    // 3. Handle interactive input directly (necessary for echo)
    uint8_t rxByte;
    char line[16];
    uint8_t idx = 0;
    while(1)
    {
        if(HAL_UART_Receive(&huart2, &rxByte, 1, HAL_MAX_DELAY) == HAL_OK) {
            if (rxByte == '\r' || rxByte == '\n') {
                line[idx] = '\0';
                HAL_UART_Transmit(&huart2, (uint8_t*)newline, strlen(newline), HAL_MAX_DELAY);
                break;
            } else if ((rxByte == '\b' || rxByte == 127) && idx > 0) {
                 idx--;
                 HAL_UART_Transmit(&huart2, (uint8_t*)"\b \b", 3, HAL_MAX_DELAY);
            } else if (idx < sizeof(line)-1 && isprint(rxByte)) {
                line[idx++] = rxByte;
                HAL_UART_Transmit(&huart2, &rxByte, 1, HAL_MAX_DELAY);
            }
        }
    }

    // 4. Process choice and queue the result message
    int valid_choice = 1;
    if(strcmp(line, "1") == 0) SetSelectedColor("RED");
    else if(strcmp(line, "2") == 0) SetSelectedColor("GREEN");
    else if(strcmp(line, "3") == 0) SetSelectedColor("BLUE");
    else if(strcmp(line, "4") == 0) SetSelectedColor("YELLOW");
    else if(strcmp(line, "5") == 0) SetSelectedColor("BLACK");
    else if(strcmp(line, "6") == 0) SetSelectedColor("WHITE");
    else valid_choice = 0;

    if (valid_choice) {
        snprintf(pSelectMsg, 64, "> Selected color set to: %s\r\n", selectedColor);
        xQueueSend(LogQueue, &pSelectMsg, 0);
    } else {
        xQueueSend(LogQueue, &pInvalidMsg, 0);
    }
    xSemaphoreGive(LogSemaphore); // Signal PrintTask
}

void StartReadRGB(void *argument)
{
    SensorData_t sensor_data;
    char *pPrintMsg = sensorPrintMsg; // Pointer to the static buffer

    for(;;)
    {
        if (ISL29125_ReadRGB255(&sensor_data.r, &sensor_data.g, &sensor_data.b) == HAL_OK) {
            const char* color = DetectColor(sensor_data.r, sensor_data.g, sensor_data.b);
            strncpy(sensor_data.color, color, sizeof(sensor_data.color)-1);
            sensor_data.color[sizeof(sensor_data.color)-1] = '\0'; // Ensure null-termination

            // 1. Send raw data to ControlTask
            xQueueSend(sensorQueue, &sensor_data, 0);

            // 2. Format and send the display string to the PrintTask (via LogQueue)
            snprintf(sensorPrintMsg, sizeof(sensorPrintMsg), "R=%-3d G=%-3d B=%-3d | Detected: %s\r\n",
                     sensor_data.r, sensor_data.g, sensor_data.b, color);
            xQueueSend(LogQueue, &pPrintMsg, 0);
            xSemaphoreGive(LogSemaphore);  // Signal the PrintTask
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void StartControlTask(void *argument)
{
    SensorData_t received_data;
    BaseType_t xStatus;
    uint8_t rxByte;
    ActuatorCommand_t actuatorCmd;
    char *pLogMsg = controlLogMsg; // Pointer to the static buffer

    for(;;)
    {
        // Non-blocking check for menu activation character '*'
        if (HAL_UART_Receive(&huart2, &rxByte, 1, 0) == HAL_OK)
        {
            if (rxByte == '*')
                HandleMenuInteraction();
        }

        // Wait for sensor data with a timeout
        xStatus = xQueueReceive(sensorQueue, &received_data, pdMS_TO_TICKS(100));

        if (xStatus == pdPASS) // If sensor data was received
        {
            // Perform matching logic
            if (strcmp(received_data.color, selectedColor) == 0) {
                actuatorCmd = ACTUATOR_CMD_ON;
                //snprintf(pLogMsg, 128, "[CONTROL LOG] Match Found! Detected: %s\r\n", received_data.color);
            } else {
                actuatorCmd = ACTUATOR_CMD_OFF;
                //snprintf(pLogMsg, 128, "[CONTROL LOG] No Match. Detected: %s\r\n", received_data.color);
            }

            // Send command to ActuatorTask
            xQueueSend(actuatorQueue, &actuatorCmd, 0);

            // Send log message to PrintTask
            xQueueSend(LogQueue, &pLogMsg, 0);
            xSemaphoreGive(LogSemaphore);
        }
         vTaskDelay(pdMS_TO_TICKS(10)); // Prevent this low-priority task from starving others if queue is always empty
    }
}

/**
  * @brief Function implementing the ActuatorTask thread.
  */
void StartActuatorTask(void *argument)
{
  ActuatorCommand_t command;
  char *pLogMsg = actuatorLogMsg; // Pointer to static buffer
  ActuatorCommand_t lastCmd = ACTUATOR_CMD_OFF; // State to avoid spamming logs

  for(;;)
  {
    // Wait indefinitely for a command from the control task
    if (xQueueReceive(actuatorQueue, &command, portMAX_DELAY) == pdPASS)
    {
      // Only log if the state *changes*
      if (command != lastCmd)
      {
          if (command == ACTUATOR_CMD_ON)
          {
            snprintf(pLogMsg, 64, "[ACTUATOR LOG] Turning ON\r\n");
            Actuator_SetLED(1);
            Actuator_SetBuzzer(1);
          }
          else
          {
            snprintf(pLogMsg, 64, "[ACTUATOR LOG] Turning OFF\r\n");
            Actuator_SetLED(0);
            Actuator_SetBuzzer(0);
          }
          lastCmd = command;

          // Send the log message and signal the PrintTask
          xQueueSend(LogQueue, &pLogMsg, 0);
          xSemaphoreGive(LogSemaphore);
      }
    }
  }
}

/**
  * @brief
  */
void StartPrintTask(void *argument)
{
  char *pMessageToPrint;

  for(;;)
  {
    // 1. Wait to be signaled by the LogSemaphore
    if (xSemaphoreTake(LogSemaphore, portMAX_DELAY) == pdTRUE)
    {
      // 2. A signal was received. Drain the LogQueue completely.
      while (xQueueReceive(LogQueue, &pMessageToPrint, 0) == pdTRUE)
      {
        // 3. Acquire the LogMutex to protect the UART
        if (xSemaphoreTake(LogMutex, portMAX_DELAY) == pdTRUE)
        {
          // 4. Write the log message to the "file" (UART)
          HAL_UART_Transmit(&huart2, (uint8_t*)pMessageToPrint, strlen(pMessageToPrint), HAL_MAX_DELAY);

          // 5. Release the mutex [cite: 5819, 5820]
          xSemaphoreGive(LogMutex);
        }
      }
    }
  }
}
/* USER CODE END 4 */


/**
  * @brief  Period elapsed callback in non blocking mode
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
