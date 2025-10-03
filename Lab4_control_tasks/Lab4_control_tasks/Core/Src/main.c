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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

UART_HandleTypeDef huart2;

/* Definitions for ReadRGB */
osThreadId_t ReadRGBHandle;
const osThreadAttr_t ReadRGB_attributes = {
  .name = "ReadRGB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
char msg[64];
char detectedColor[16];   // Shared string between tasks

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartReadRGB(void *argument);
void StartControlTask(void *argument);

/* USER CODE BEGIN PFP */
HAL_StatusTypeDef ISL29125_WriteRegister(uint8_t reg, uint8_t value);
HAL_StatusTypeDef ISL29125_ReadRegister(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef ISL29125_Init(void);
HAL_StatusTypeDef ISL29125_ReadRGB255(int *r_val, int *g_val, int *b_val);
void Actuator_SetLED(uint8_t state);
const char* DetectColor(int r, int g, int b);
//void Queue_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void traceTaskSwitch(void)
{
    uint8_t buf1[50];

    HAL_GPIO_TogglePin(GPIOA, LD2_Pin);

    // Print the current task name to UART
    snprintf((char*)buf1, sizeof(buf1),
             "Current task: %s\r\n", pcTaskGetName(NULL));
    HAL_UART_Transmit(&huart2, buf1, strlen((char*)buf1), HAL_MAX_DELAY);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ISL29125_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ReadRGB */
  ReadRGBHandle = osThreadNew(StartReadRGB, NULL, &ReadRGB_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  if (ISL29125_ReadRGB255(&r_pct, &g_pct, &b_pct) == HAL_OK) {
//	      int len = snprintf(msg, sizeof(msg),"R=%d%% G=%d%% B=%d%%\r\n",r_pct, g_pct, b_pct);
//	      HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
//
//	  }
//	  HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Out_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Out_LED_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Out_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef ISL29125_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return HAL_I2C_Master_Transmit(&hi2c1, ISL29125_ADDR, buf, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ISL29125_ReadRegister(uint8_t reg, uint8_t *value) {
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Transmit(&hi2c1, ISL29125_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    return HAL_I2C_Master_Receive(&hi2c1, ISL29125_ADDR, value, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ISL29125_Init(void)
{
    HAL_StatusTypeDef ret;
//    uint8_t device_id;

    //  Configure sensor for RGB mode, 16-bit ADC, 375 lux
    ret = ISL29125_WriteRegister(ISL29125_REG_CONFIG1, CONFIG1_MODE_RGB_16BIT);
    if (ret != HAL_OK) return ret;

    // Sensor initialized successfully
    return HAL_OK;
}


HAL_StatusTypeDef ISL29125_ReadRGB255(int *r_val, int *g_val, int *b_val) {
    uint8_t lo, hi;
    uint16_t r_raw, g_raw, b_raw;

    // Read Green
    ISL29125_ReadRegister(ISL29125_REG_GREEN_L, &lo);
    ISL29125_ReadRegister(ISL29125_REG_GREEN_H, &hi);
    g_raw = (hi << 8) | lo;

    // Read Red
    ISL29125_ReadRegister(ISL29125_REG_RED_L, &lo);
    ISL29125_ReadRegister(ISL29125_REG_RED_H, &hi);
    r_raw = (hi << 8) | lo;

    // Read Blue
    ISL29125_ReadRegister(ISL29125_REG_BLUE_L, &lo);
    ISL29125_ReadRegister(ISL29125_REG_BLUE_H, &hi);
    b_raw = (hi << 8) | lo;

    // Convert to percentage of sensor range (0–65535)
    *r_val = (r_raw * 255) / 65535;
    *g_val = (g_raw * 255) / 65535;
    *b_val = (b_raw * 255) / 65535;

    return HAL_OK;
}
void Actuator_SetLED(uint8_t state) {
    HAL_GPIO_WritePin(GPIOA, Out_LED_Pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
const char* DetectColor(int r, int g, int b) {
    // --- BLACK detection ---
    if (r < 60 && g < 100 && 50 < b && b < 80) return "BLACK";

    // --- YELLOW detection ---
    if (r > 240 && g > 240 && b > 190) return "YELLOW";

    // --- RED detection ---
    if (r > 145 && g > 145) {
        if (g - b > 60) return "RED";
    }

    // --- GREEN (Dark/Light) detection ---
    if (g > 250) {
        if (g > r + 70 && g > b + 70) return "GREEN";
        if (r > 150 && b > 150) return "GREEN";
    }

    // --- BLUE (Dark/Light) detection ---
    if (g > 170 && b > 170 && r < 100) {
        if (b > r + 90) return "BLUE";
    }

    // Fallback if nothing matches cleanly
    return "UNKNOWN";
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartReadRGB */
/**
* @brief Function implementing the ReadRGB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadRGB */
void StartReadRGB(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	for(;;)
	{
        int r, g, b;

        if (ISL29125_ReadRGB255(&r, &g, &b) == HAL_OK) {
            const char* color = DetectColor(r, g, b);

            // Save detected color string globally (for ControlTask)
            strncpy(detectedColor, color, sizeof(detectedColor)-1);
            detectedColor[sizeof(detectedColor)-1] = '\0';

            // Print RGB + Detected color
            int len = snprintf(msg, sizeof(msg),
                               "R=%d G=%d B=%d | Detected: %s\r\n",
                               r, g, b, color);

            HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
        }
        osDelay(500);
    }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for(;;)
  {
      uint8_t state = 0;

      // Make a local copy of the detected color (thread safety)
      char color_local[16];
      strncpy(color_local, detectedColor, sizeof(color_local));
      color_local[sizeof(color_local)-1] = '\0';

      // Decide based on color
      if (strcmp(color_local, "GREEN") == 0) {
          state = 1; // turn LED ON
      }
      else if (strcmp(color_local, "YELLOW") == 0) {
          state = 1; // also turn LED ON
      }
      else {
          state = 0; // LED OFF otherwise
      }

      Actuator_SetLED(state);

      osDelay(200);
  }
  /* USER CODE END StartControlTask */
}

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
  if (htim->Instance == TIM6)
  {
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
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
