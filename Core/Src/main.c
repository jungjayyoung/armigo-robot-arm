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
#include "cmsis_os2.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ax12.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SHARP_AVERAGE_SAMPLE_COUNT       5U
#define SHARP_DETECT_CONFIRM_COUNT       3U
#define SHARP_VALID_MIN_MV             900U
#define SHARP_VALID_MAX_MV            2600U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
AX12_AppState ax12_app;
volatile uint16_t g_sharp_mv = 0U;
volatile uint16_t g_sharp_distance_cm = 80U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t Sharp_MillivoltsToDistanceCm(uint16_t mv)
{
  /* Typical GP2Y0A21YK0F transfer-curve points. Linear interpolation keeps
   * the ISR lightweight; final accuracy should be calibrated on the robot. */
  static const uint16_t voltage_mv[] =
      {2300U, 1650U, 1300U, 900U, 700U, 600U, 500U, 400U};
  static const uint8_t distance_cm[] =
      {10U, 15U, 20U, 30U, 40U, 50U, 60U, 80U};

  if (mv >= voltage_mv[0]) return distance_cm[0];
  if (mv <= voltage_mv[7]) return distance_cm[7];

  for (uint8_t i = 0U; i < 7U; ++i)
  {
    if ((mv <= voltage_mv[i]) && (mv >= voltage_mv[i + 1U]))
    {
      uint32_t voltage_span = voltage_mv[i] - voltage_mv[i + 1U];
      uint32_t distance_span = distance_cm[i + 1U] - distance_cm[i];
      uint32_t offset = voltage_mv[i] - mv;
      return (uint16_t)(distance_cm[i] +
                        ((offset * distance_span) / voltage_span));
    }
  }
  return 80U;
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1U, 0xFFFFU);
  return ch;
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100U);

  if (!AX12_AppInit(&ax12_app, &huart1, &huart6))
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    printf("AX12 init failed\r\n");
  }
  if (!AX12_AppStartConsoleRx(&ax12_app, &huart2))
  {
    printf("AX12 console rx start failed\r\n");
  }

  if (!AX12_AppStartLinkRx(&ax12_app))
  {
    printf("HC05 link rx start failed\r\n");
  }

  /* USER CODE END 2 */

  osKernelInitialize();
  MX_FREERTOS_Init();
  osKernelStart();

  /* The scheduler owns execution from this point. */
  while (1)
  {
  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  AX12_AppUartRxCpltCallback(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  AX12_AppUartTxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  AX12_AppUartErrorCallback(huart);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    static uint16_t samples[SHARP_AVERAGE_SAMPLE_COUNT] = {0U};
    static uint32_t sample_sum = 0U;
    static uint8_t sample_index = 0U;
    static uint8_t sample_count = 0U;
    static uint8_t detect_count = 0U;
    uint16_t mv =
        (uint16_t)((uint32_t)HAL_ADC_GetValue(hadc) * 3300U / 4095U);

    sample_sum -= samples[sample_index];
    samples[sample_index] = mv;
    sample_sum += mv;
    sample_index = (uint8_t)((sample_index + 1U) %
                             SHARP_AVERAGE_SAMPLE_COUNT);
    if (sample_count < SHARP_AVERAGE_SAMPLE_COUNT) ++sample_count;

    g_sharp_mv = (uint16_t)(sample_sum / sample_count);
    g_sharp_distance_cm = Sharp_MillivoltsToDistanceCm(g_sharp_mv);

    /* GP2Y0A21YK0F is specified for 10..80 cm. For this project, release
     * the first Auto step only after three consecutive 10..30 cm samples. */
    if ((g_sharp_mv >= SHARP_VALID_MIN_MV) &&
        (g_sharp_mv <= SHARP_VALID_MAX_MV) &&
        (g_sharp_distance_cm <= 30U))
    {
      if (detect_count < SHARP_DETECT_CONFIRM_COUNT) ++detect_count;
    }
    else
    {
      detect_count = 0U;
    }

    AX12_AppSetSharpDetected(
        &ax12_app, detect_count >= SHARP_DETECT_CONFIRM_COUNT);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM10)
  {
    HAL_IncTick();
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
