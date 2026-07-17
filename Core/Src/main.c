/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "OLED.h"
#include "Delay.h"
#include "Servo.h"
#include "Serial.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t panAngle    = 90;
uint8_t tiltAngle   = 90;
uint8_t scanRunning = 0;
uint8_t scanPanDir  = 0;
uint8_t scanTiltDir = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  OLED_Init();
  OLED_Clear();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  Servo_SetPanAngle(90);
  Servo_SetTiltAngle(90);

  Serial_Init();

  OLED_ShowString(1, 1, "Pan-Tilt Gimbal");
  OLED_ShowString(2, 1, "LD-1501MG x2");
  OLED_ShowString(3, 1, "Init done!     ");
  OLED_ShowString(4, 1, "UART 115200bps");
  HAL_Delay(1500);

  Serial_SendString("\r\n========================================\r\n");
  Serial_SendString("  2-Axis Pan-Tilt Gimbal Controller\r\n");
  Serial_SendString("  LD-1501MG x2  |  STM32F103C8T6\r\n");
  Serial_SendString("========================================\r\n");
  Serial_SendString("Commands:\r\n");
  Serial_SendString("  P=0~180  Set Pan angle\r\n");
  Serial_SendString("  T=0~180  Set Tilt angle\r\n");
  Serial_SendString("  SCAN     Scan all axes\r\n");
  Serial_SendString("  STOP     Stop scan\r\n");
  Serial_SendString("  RST      Reset to 90deg\r\n");
  Serial_SendString("  INFO     Show angle range\r\n");
  Serial_SendString("  CP       Show Pan angle & PWM\r\n");
  Serial_SendString("  CP+n     Pan +n deg (default 5)\r\n");
  Serial_SendString("  CP-n     Pan -n deg (default 5)\r\n");
  Serial_SendString("  CT       Show Tilt angle & PWM\r\n");
  Serial_SendString("  CT+n     Tilt +n deg (default 5)\r\n");
  Serial_SendString("  CT-n     Tilt -n deg (default 5)\r\n");
  Serial_SendString("========================================\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (g_CommandReady)
    {
        uint8_t cmdType = g_CommandReady;
        g_CommandReady = 0;

        if (cmdType == 2)
        {
            scanRunning = 0;
            panAngle  = 90;
            tiltAngle = 90;
            Servo_SetPanAngle(panAngle);
            Servo_SetTiltAngle(tiltAngle);
            Serial_SendString("OK RESET -> (P=90, T=90)\r\n");
        }
        else if (cmdType == 3)
        {
            Serial_SendString("Angle Range:\r\n");
            Serial_SendString("  Pan:  ");
            Serial_SendNum(SERVO_PAN_MIN);
            Serial_SendString(" ~ ");
            Serial_SendNum(SERVO_PAN_MAX);
            Serial_SendString(" deg\r\n");
            Serial_SendString("  Tilt: ");
            Serial_SendNum(SERVO_TILT_MIN);
            Serial_SendString(" ~ ");
            Serial_SendNum(SERVO_TILT_MAX);
            Serial_SendString(" deg\r\n");
        }
        else if (cmdType == 4)
        {
            uint16_t currentPulse = 0;
            
            if (g_CalibMode == 1) {
                currentPulse = Servo_AngleToPulse_Pan(panAngle);
                Serial_SendString("P Axis:\r\n");
                Serial_SendString("  Angle: ");
                Serial_SendNum(panAngle);
                Serial_SendString(" deg\r\n");
                Serial_SendString("  PWM:   ");
                Serial_SendNum(currentPulse);
                Serial_SendString(" us\r\n");
            }
            else if (g_CalibMode == 2) {
                if (panAngle + g_CalibValue <= SERVO_PAN_MAX) {
                    panAngle += g_CalibValue;
                } else {
                    panAngle = SERVO_PAN_MAX;
                }
                Servo_SetPanAngle(panAngle);
                currentPulse = Servo_AngleToPulse_Pan(panAngle);
                Serial_SendString("P+ -> ");
                Serial_SendNum(panAngle);
                Serial_SendString(" deg, ");
                Serial_SendNum(currentPulse);
                Serial_SendString(" us\r\n");
            }
            else if (g_CalibMode == 3) {
                if (panAngle >= g_CalibValue) {
                    panAngle -= g_CalibValue;
                } else {
                    panAngle = SERVO_PAN_MIN;
                }
                Servo_SetPanAngle(panAngle);
                currentPulse = Servo_AngleToPulse_Pan(panAngle);
                Serial_SendString("P- -> ");
                Serial_SendNum(panAngle);
                Serial_SendString(" deg, ");
                Serial_SendNum(currentPulse);
                Serial_SendString(" us\r\n");
            }
            else if (g_CalibMode == 4) {
                currentPulse = Servo_AngleToPulse_Tilt(tiltAngle);
                Serial_SendString("T Axis:\r\n");
                Serial_SendString("  Angle: ");
                Serial_SendNum(tiltAngle);
                Serial_SendString(" deg\r\n");
                Serial_SendString("  PWM:   ");
                Serial_SendNum(currentPulse);
                Serial_SendString(" us\r\n");
            }
            else if (g_CalibMode == 5) {
                if (tiltAngle + g_CalibValue <= SERVO_TILT_MAX) {
                    tiltAngle += g_CalibValue;
                } else {
                    tiltAngle = SERVO_TILT_MAX;
                }
                Servo_SetTiltAngle(tiltAngle);
                currentPulse = Servo_AngleToPulse_Tilt(tiltAngle);
                Serial_SendString("T+ -> ");
                Serial_SendNum(tiltAngle);
                Serial_SendString(" deg, ");
                Serial_SendNum(currentPulse);
                Serial_SendString(" us\r\n");
            }
            else if (g_CalibMode == 6) {
                if (tiltAngle >= g_CalibValue) {
                    tiltAngle -= g_CalibValue;
                } else {
                    tiltAngle = SERVO_TILT_MIN;
                }
                Servo_SetTiltAngle(tiltAngle);
                currentPulse = Servo_AngleToPulse_Tilt(tiltAngle);
                Serial_SendString("T- -> ");
                Serial_SendNum(tiltAngle);
                Serial_SendString(" deg, ");
                Serial_SendNum(currentPulse);
                Serial_SendString(" us\r\n");
            }
            
            g_CalibMode = 0;
            g_CalibValue = 0;
        }
        else if (g_ScanMode > 0)
        {
            scanRunning = g_ScanMode;
            g_ScanMode  = 0;
            scanPanDir  = 0;
            scanTiltDir = 0;

            const char *modeName[] = {"", "SCAN ALL", "SCAN PAN", "SCAN TILT", "LIMIT DETECT"};
            Serial_SendString("OK ");
            Serial_SendString(modeName[scanRunning]);
            Serial_SendString("\r\n");
        }
        else
        {
            if (g_ParsedPanAngle > 0 || (g_ParsedPanAngle == 0 && strstr(g_RawCommand, "P=0") != NULL))
            {
                panAngle = g_ParsedPanAngle;
                Servo_SetPanAngle(panAngle);
            }
            if (g_ParsedTiltAngle > 0 || (g_ParsedTiltAngle == 0 && strstr(g_RawCommand, "T=0") != NULL))
            {
                tiltAngle = g_ParsedTiltAngle;
                Servo_SetTiltAngle(tiltAngle);
            }
            scanRunning = 0;

            Serial_SendString("OK P=");
            Serial_SendNum(panAngle);
            Serial_SendString(", T=");
            Serial_SendNum(tiltAngle);
            Serial_SendString("\r\n");

            g_ParsedPanAngle  = 0;
            g_ParsedTiltAngle = 0;
        }
    }

    if (scanRunning)
    {
        uint8_t step = (scanRunning == 4) ? 1 : 2;

        if (scanRunning == 1 || scanRunning == 2 || scanRunning == 4)
        {
            if (scanPanDir == 0) {
                panAngle += step;
                if (panAngle >= SERVO_PAN_MAX) { panAngle = SERVO_PAN_MAX; scanPanDir = 1; }
            } else {
                panAngle -= step;
                if (panAngle <= SERVO_PAN_MIN) { panAngle = SERVO_PAN_MIN; scanPanDir = 0; }
            }
            Servo_SetPanAngle(panAngle);
        }

        if (scanRunning == 1 || scanRunning == 3 || scanRunning == 4)
        {
            if (scanTiltDir == 0) {
                tiltAngle += step;
                if (tiltAngle >= SERVO_TILT_MAX) { tiltAngle = SERVO_TILT_MAX; scanTiltDir = 1; }
            } else {
                tiltAngle -= step;
                if (tiltAngle <= SERVO_TILT_MIN) { tiltAngle = SERVO_TILT_MIN; scanTiltDir = 0; }
            }
            Servo_SetTiltAngle(tiltAngle);
        }
    }

    uint16_t panPulse  = Servo_AngleToPulse_Pan(panAngle);
    uint16_t tiltPulse = Servo_AngleToPulse_Tilt(tiltAngle);

    OLED_ShowString(1, 1, "P:");
    OLED_ShowNum(1, 3, panAngle, 3);
    OLED_ShowString(1, 6, "deg ");
    OLED_ShowNum(1, 10, panPulse, 4);
    OLED_ShowString(1, 15, "us");

    OLED_ShowString(2, 1, "T:");
    OLED_ShowNum(2, 3, tiltAngle, 3);
    OLED_ShowString(2, 6, "deg ");
    OLED_ShowNum(2, 10, tiltPulse, 4);
    OLED_ShowString(2, 15, "us");

    OLED_ShowString(3, 1, "M:");
    if (scanRunning == 0)      OLED_ShowString(3, 3, "MANUAL   ");
    else if (scanRunning == 1) OLED_ShowString(3, 3, "SCAN-ALL ");
    else if (scanRunning == 2) OLED_ShowString(3, 3, "SCAN-PAN ");
    else if (scanRunning == 3) OLED_ShowString(3, 3, "SCAN-TILT");
    else if (scanRunning == 4) OLED_ShowString(3, 3, "LIMIT    ");

    if (scanRunning) {
        OLED_ShowChar(3, 13, scanPanDir  ? '<' : '>');
        OLED_ShowChar(3, 15, scanTiltDir ? 'v' : '^');
    } else {
        OLED_ShowString(3, 13, "  ");
    }

    OLED_ShowString(4, 1, "                ");
    char dispCmd[17];
    strncpy(dispCmd, g_RawCommand, 16);
    dispCmd[16] = '\0';
    OLED_ShowString(4, 1, dispCmd);

    HAL_Delay(20);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        Serial_ProcessByte(g_RxByte);
    }
}
/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
