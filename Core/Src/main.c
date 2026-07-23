/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "SquareDraw.h"
#include "Motion.h"
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
// 角度存储: 0.1° 单位 (913 = 91.3°)
uint16_t panAngle10  = 900;   // 90.0°
uint16_t tiltAngle10 = 900;   // 90.0°
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

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  Servo_SetPanAngle10(900);   // 90.0°
  Servo_SetTiltAngle10(900);  // 90.0°

  Serial_Init();
  SquareDraw_Init();
  Motion_Init();

  OLED_Init();
  OLED_Clear();

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
  Serial_SendString("  P=0~180  Set Pan angle (e.g. P=91.3)\r\n");
  Serial_SendString("  T=0~90   Set Tilt angle (e.g. T=45.7)\r\n");
  Serial_SendString("  TRACK,dp,dt  Incremental tracking\r\n");
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
  Serial_SendString("  SQUARE n Draw nxn cm square\r\n");
  Serial_SendString("  CONFIG ARM n   Set arm length (cm)\r\n");
  Serial_SendString("  CONFIG DIST n  Set mount distance (cm)\r\n");
  Serial_SendString("  CONFIG SHOW    Show current config\r\n");
  Serial_SendString("========================================\r\n");
  Serial_SendString("Current Config:\r\n");
  Serial_SendString("  Arm Length: ");
  Serial_SendNum((uint32_t)Config_GetArmLength());
  Serial_SendString("cm\r\n");
  Serial_SendString("  Mount Distance: ");
  Serial_SendNum((uint32_t)Config_GetMountDistance());
  Serial_SendString("cm\r\n");
  Serial_SendString("========================================\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t last_tick = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t current_tick = HAL_GetTick();
    if (current_tick - last_tick < 20) {
        continue;
    }
    last_tick = current_tick;

    Motion_Update();

    if (g_CommandReady)
    {
        uint8_t cmdType = g_CommandReady;
        g_CommandReady = 0;

        if (cmdType == 2)
        {
            scanRunning = 0;
            panAngle10  = 900;   // 90.0°
            tiltAngle10 = 900;
            Servo_SetPanAngle10(panAngle10);
            Servo_SetTiltAngle10(tiltAngle10);
            Serial_SendString("OK RESET -> (P=90.0, T=90.0)\r\n");
        }
        else if (cmdType == 3)
        {
            Serial_SendString("Angle Range:\r\n");
            Serial_SendString("  Pan:  ");
            Serial_SendNum(SERVO_PAN_MAX / 10);
            Serial_SendString(" deg\r\n");
            Serial_SendString("  Tilt: ");
            Serial_SendNum(SERVO_TILT_MAX / 10);
            Serial_SendString(" deg\r\n");
        }
        else if (cmdType == 4)
        {
            uint16_t currentPulse = 0;

            if (g_CalibMode == 1) {
                currentPulse = Servo_Angle10ToPulse_Pan(panAngle10);
                Serial_SendString("P Axis:\r\n");
                Serial_SendString("  Angle: ");
                Serial_SendNum(panAngle10 / 10);
                Serial_SendString(".");
                Serial_SendNum(panAngle10 % 10);
                Serial_SendString(" deg\r\n");
                Serial_SendString("  PWM:   ");
                Serial_SendNum(currentPulse);
                Serial_SendString(" us\r\n");
            }
            else if (g_CalibMode == 2) {
                uint16_t delta10 = g_CalibValue * 10;  // deg → ×10
                if (panAngle10 + delta10 <= SERVO_PAN_MAX) {
                    panAngle10 += delta10;
                } else {
                    panAngle10 = SERVO_PAN_MAX;
                }
                Servo_SetPanAngle10(panAngle10);
                currentPulse = Servo_Angle10ToPulse_Pan(panAngle10);
                Serial_SendString("P+ -> ");
                Serial_SendNum(panAngle10 / 10);
                Serial_SendString(".");
                Serial_SendNum(panAngle10 % 10);
                Serial_SendString(" deg, ");
                Serial_SendNum(currentPulse);
                Serial_SendString(" us\r\n");
            }
            else if (g_CalibMode == 3) {
                uint16_t delta10 = g_CalibValue * 10;
                if (panAngle10 >= delta10) {
                    panAngle10 -= delta10;
                } else {
                    panAngle10 = SERVO_PAN_MIN;
                }
                Servo_SetPanAngle10(panAngle10);
                currentPulse = Servo_Angle10ToPulse_Pan(panAngle10);
                Serial_SendString("P- -> ");
                Serial_SendNum(panAngle10 / 10);
                Serial_SendString(".");
                Serial_SendNum(panAngle10 % 10);
                Serial_SendString(" deg, ");
                Serial_SendNum(currentPulse);
                Serial_SendString(" us\r\n");
            }
            else if (g_CalibMode == 4) {
                currentPulse = Servo_Angle10ToPulse_Tilt(tiltAngle10);
                Serial_SendString("T Axis:\r\n");
                Serial_SendString("  Angle: ");
                Serial_SendNum(tiltAngle10 / 10);
                Serial_SendString(".");
                Serial_SendNum(tiltAngle10 % 10);
                Serial_SendString(" deg\r\n");
                Serial_SendString("  PWM:   ");
                Serial_SendNum(currentPulse);
                Serial_SendString(" us\r\n");
            }
            else if (g_CalibMode == 5) {
                uint16_t delta10 = g_CalibValue * 10;
                if (tiltAngle10 + delta10 <= SERVO_TILT_MAX) {
                    tiltAngle10 += delta10;
                } else {
                    tiltAngle10 = SERVO_TILT_MAX;
                }
                Servo_SetTiltAngle10(tiltAngle10);
                currentPulse = Servo_Angle10ToPulse_Tilt(tiltAngle10);
                Serial_SendString("T+ -> ");
                Serial_SendNum(tiltAngle10 / 10);
                Serial_SendString(".");
                Serial_SendNum(tiltAngle10 % 10);
                Serial_SendString(" deg, ");
                Serial_SendNum(currentPulse);
                Serial_SendString(" us\r\n");
            }
            else if (g_CalibMode == 6) {
                uint16_t delta10 = g_CalibValue * 10;
                if (tiltAngle10 >= delta10) {
                    tiltAngle10 -= delta10;
                } else {
                    tiltAngle10 = SERVO_TILT_MIN;
                }
                Servo_SetTiltAngle10(tiltAngle10);
                currentPulse = Servo_Angle10ToPulse_Tilt(tiltAngle10);
                Serial_SendString("T- -> ");
                Serial_SendNum(tiltAngle10 / 10);
                Serial_SendString(".");
                Serial_SendNum(tiltAngle10 % 10);
                Serial_SendString(" deg, ");
                Serial_SendNum(currentPulse);
                Serial_SendString(" us\r\n");
            }

            g_CalibMode = 0;
            g_CalibValue = 0;
        }
        else if (cmdType == 5)
        {
            int8_t result = SquareDraw_Start(g_SquareSize_cm, g_SquareSize_cm);
            if (result == -1) {
                Serial_SendString("错误：参数无效\r\n");
            } else if (result == -2) {
                Serial_SendString("错误：角度超出范围，请减小尺寸或调整原点位置\r\n");
            } else if (result == -3) {
                Serial_SendString("错误：SCAN模式正在运行，请先停止\r\n");
            }
        }
        else if (cmdType == 6)
        {
            if (g_ConfigParamType == 1) {
                if (Config_SetArmLength((float)g_ConfigParamValue) == 0) {
                    Serial_SendString("OK 舵机臂长已设置为 ");
                    Serial_SendNum(g_ConfigParamValue);
                    Serial_SendString("cm\r\n");
                } else {
                    Serial_SendString("错误：设置失败\r\n");
                }
            }
            else if (g_ConfigParamType == 2) {
                if (Config_SetMountDistance((float)g_ConfigParamValue) == 0) {
                    Serial_SendString("OK 安装距离已设置为 ");
                    Serial_SendNum(g_ConfigParamValue);
                    Serial_SendString("cm\r\n");
                } else {
                    Serial_SendString("错误：设置失败\r\n");
                }
            }
            else if (g_ConfigParamType == 3) {
                Serial_SendString("当前配置:\r\n");
                Serial_SendString("  舵机臂长: ");
                Serial_SendNum((uint32_t)Config_GetArmLength());
                Serial_SendString("cm\r\n");
                Serial_SendString("  安装距离: ");
                Serial_SendNum((uint32_t)Config_GetMountDistance());
                Serial_SendString("cm\r\n");
            }

            g_ConfigParamType = 0;
            g_ConfigParamValue = 0;
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
            // P= / T= 绝对角度命令 (0.1° 单位)
            if (g_ParsedPanAngle10 != 0xFFFF)
            {
                panAngle10 = g_ParsedPanAngle10;
                Servo_SetPanAngle10(panAngle10);
            }
            if (g_ParsedTiltAngle10 != 0xFFFF)
            {
                tiltAngle10 = g_ParsedTiltAngle10;
                Servo_SetTiltAngle10(tiltAngle10);
            }
            scanRunning = 0;

            Serial_SendString("OK P=");
            Serial_SendNum(panAngle10 / 10);
            Serial_SendString(".");
            Serial_SendNum(panAngle10 % 10);
            Serial_SendString(", T=");
            Serial_SendNum(tiltAngle10 / 10);
            Serial_SendString(".");
            Serial_SendNum(tiltAngle10 % 10);
            Serial_SendString("\r\n");

            g_ParsedPanAngle10  = 0xFFFF;
            g_ParsedTiltAngle10 = 0xFFFF;
        }
    }

    if (scanRunning)
    {
        uint8_t step = (scanRunning == 4) ? 10 : 20;  // ×10: 1°→10, 2°→20

        if (scanRunning == 1 || scanRunning == 2 || scanRunning == 4)
        {
            if (scanPanDir == 0) {
                panAngle10 += step;
                if (panAngle10 >= SERVO_PAN_MAX) { panAngle10 = SERVO_PAN_MAX; scanPanDir = 1; }
            } else {
                panAngle10 -= step;
                if (panAngle10 <= SERVO_PAN_MIN) { panAngle10 = SERVO_PAN_MIN; scanPanDir = 0; }
            }
            Servo_SetPanAngle10(panAngle10);
        }

        if (scanRunning == 1 || scanRunning == 3 || scanRunning == 4)
        {
            if (scanTiltDir == 0) {
                tiltAngle10 += step;
                if (tiltAngle10 >= SERVO_TILT_MAX) { tiltAngle10 = SERVO_TILT_MAX; scanTiltDir = 1; }
            } else {
                tiltAngle10 -= step;
                if (tiltAngle10 <= SERVO_TILT_MIN) { tiltAngle10 = SERVO_TILT_MIN; scanTiltDir = 0; }
            }
            Servo_SetTiltAngle10(tiltAngle10);
        }
    }

    SquareDraw_Execute();

    uint16_t panPulse  = Servo_Angle10ToPulse_Pan(panAngle10);
    uint16_t tiltPulse = Servo_Angle10ToPulse_Tilt(tiltAngle10);

    OLED_ShowString(1, 1, "P:");
    OLED_ShowNum(1, 3, panAngle10 / 10, 3);
    OLED_ShowString(1, 6, ".");
    OLED_ShowNum(1, 7, panAngle10 % 10, 1);
    OLED_ShowString(1, 8, "deg  ");

    OLED_ShowString(2, 1, "T:");
    OLED_ShowNum(2, 3, tiltAngle10 / 10, 3);
    OLED_ShowString(2, 6, ".");
    OLED_ShowNum(2, 7, tiltAngle10 % 10, 1);
    OLED_ShowString(2, 8, "deg  ");

    OLED_ShowString(3, 1, "PWM P:");
    OLED_ShowNum(3, 7, panPulse, 4);
    OLED_ShowString(3, 11, "us");

    OLED_ShowString(4, 1, "PWM T:");
    OLED_ShowNum(4, 7, tiltPulse, 4);
    OLED_ShowString(4, 11, "us");

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

static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

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
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART1_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        Serial_ProcessByte(g_RxByte);
    }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
