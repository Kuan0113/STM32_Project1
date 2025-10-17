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
#define CONFIG1_MODE_RGB_16BIT   0x05   // RGB mode, 16-bit ADC, 375 lux
#define CONFIG1_MODE_RGB_12BIT   0x0D   // RGB mode, 12-bit ADC, 375 lux
#define CONFIG2_IR_MAX           0xBF   // Max IR compensation + IR offset
#define CONFIG3_DEFAULT          0x00   // Default settings
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1; // For Buzzer PWM
UART_HandleTypeDef huart2;
// Note: htim6 is likely handled by HAL internally for the timebase

/* USER CODE BEGIN PV */
char msg[128];
QueueHandle_t sensorQueue;
SemaphoreHandle_t uartPrintSemaphore;
char selectedColor[16] = "GREEN";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
void StartReadRGB(void *argument);
void StartControlTask(void *argument);

/* USER CODE BEGIN PFP */
HAL_StatusTypeDef ISL29125_WriteRegister(uint8_t reg, uint8_t value);
HAL_StatusTypeDef ISL29125_ReadRegister(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef ISL29125_Init(void);
HAL_StatusTypeDef ISL29125_ReadRGB255(int *r_val, int *g_val, int *b_val);
void Actuator_SetLED(uint8_t state);
void Actuator_SetBuzzer(uint8_t state);
const char* DetectColor(int r, int g, int b);
void SetSelectedColor(const char* color);
void traceTaskSwitch(void){}
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
  MX_I2C1_Init();
  MX_TIM1_Init(); // Initialize the timer for PWM

  /* USER CODE BEGIN 2 */
  ISL29125_Init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_QUEUES */
  sensorQueue = xQueueCreate(1, sizeof(SensorData_t));
  uartPrintSemaphore = xSemaphoreCreateBinary();
  if (uartPrintSemaphore != NULL) {
      xSemaphoreGive(uartPrintSemaphore);
  }
  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN RTOS_THREADS */
  xTaskCreate(StartReadRGB, "SensorTask", 256, NULL, 2, NULL);
  xTaskCreate(StartControlTask, "ControlTask", 256, NULL, 1, NULL);
  /* USER CODE END RTOS_THREADS */

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
    if (r > 240 && g > 240 && b < 245)
        return "YELLOW";

    // --- BLUE (covers light/dark) ---
    if (r < 120 && g > 180 && b > 180)
        return "BLUE";

    // --- GREEN (covers light/dark) ---
    if (r > 100 && r < 200 && g > 230 && b > 150)
        return "GREEN";

    // --- RED ---
    if (r > 150 && g > 150 && b < 110)
        return "RED";

    // --- BLACK ---
    if (r < 70 && g > 100 && b > 70 && g - r > 40)
        return "BLACK";

    return "UNKNOWN";
}


void StartReadRGB(void *argument)
{
    SensorData_t sensor_data;
    for(;;)
    {
        if (ISL29125_ReadRGB255(&sensor_data.r, &sensor_data.g, &sensor_data.b) == HAL_OK) {
            const char* color = DetectColor(sensor_data.r, sensor_data.g, sensor_data.b);
            strncpy(sensor_data.color, color, sizeof(sensor_data.color)-1);

            if (xSemaphoreTake(uartPrintSemaphore, portMAX_DELAY) == pdPASS)
            {
                int len = snprintf(msg, sizeof(msg), "R=%-3d G=%-3d B=%-3d | Detected: %s\r\n",
                                   sensor_data.r, sensor_data.g, sensor_data.b, color);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
                xSemaphoreGive(uartPrintSemaphore);
            }
            xQueueSend(sensorQueue, &sensor_data, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void HandleMenuInteraction(void)
{
    uint8_t rxByte;
    if (xSemaphoreTake(uartPrintSemaphore, portMAX_DELAY) == pdPASS)
    {
        const char *menu =
            "\r\n--- SELECT A COLOR ---\r\n"
            "1: RED\r\n2: GREEN\r\n3: BLUE\r\n4: YELLOW\r\n5: BLACK\r\n"
            "Enter your choice followed by ENTER: ";
        HAL_UART_Transmit(&huart2, (uint8_t*)menu, strlen(menu), HAL_MAX_DELAY);
        char line[16];
        uint8_t idx = 0;
        while(1)
        {
            if(HAL_UART_Receive(&huart2, &rxByte, 1, HAL_MAX_DELAY) == HAL_OK) {
                if (rxByte == '\r' || rxByte == '\n') {
                    line[idx] = '\0';
                    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
                    break;
                } else if (rxByte == '\b' || rxByte == 127) {
                    if (idx > 0) { idx--; HAL_UART_Transmit(&huart2, (uint8_t*)"\b \b", 3, HAL_MAX_DELAY); }
                } else if (idx < sizeof(line)-1) {
                    line[idx++] = rxByte; HAL_UART_Transmit(&huart2, &rxByte, 1, HAL_MAX_DELAY);
                }
            }
        }
        int valid_choice = 1;
        if(strcmp(line, "1") == 0) SetSelectedColor("RED");
        else if(strcmp(line, "2") == 0) SetSelectedColor("GREEN");
        else if(strcmp(line, "3") == 0) SetSelectedColor("BLUE");
        else if(strcmp(line, "4") == 0) SetSelectedColor("YELLOW");
        else if(strcmp(line, "5") == 0) SetSelectedColor("BLACK");
        else valid_choice = 0;
        if (valid_choice) {
            int len = snprintf(msg, sizeof(msg), "> Selected color set to: %s\r\n", selectedColor);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
        } else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n> Invalid choice\r\n", 20, HAL_MAX_DELAY);
        }
        xSemaphoreGive(uartPrintSemaphore);
    }
}

void StartControlTask(void *argument)
{
    SensorData_t received_data;
    BaseType_t xStatus;
    uint8_t rxByte;
    for(;;)
    {
        if (HAL_UART_Receive(&huart2, &rxByte, 1, 0) == HAL_OK)
        {
            if (rxByte == '*')
                HandleMenuInteraction();
        }
        xStatus = xQueueReceive(sensorQueue, &received_data, pdMS_TO_TICKS(100));
        if (xStatus == pdPASS)
        {
            if (strcmp(received_data.color, selectedColor) == 0) {
                Actuator_SetLED(1);
                Actuator_SetBuzzer(1);
            } else {
                Actuator_SetLED(0);
                Actuator_SetBuzzer(0);
            }
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) { // *** CORRECTED: Using TIM6 for HAL Tick ***
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
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
