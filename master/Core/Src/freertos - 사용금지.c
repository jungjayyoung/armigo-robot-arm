/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Standalone Dual LCD & Keypad Test Code (No Robot/BT Required)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "lcd_font.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    MODE_TEACHING  = 0,
    MODE_AUTO      = 1,
    MODE_ADMIN_JOG = 2
} SystemMode_t;

typedef enum {
    RUN_STATE_STOPPED = 0,
    RUN_STATE_RUNNING = 1
} RunState_t;

typedef struct {
    uint16_t axis[4];
} TeachingPoint_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 화면 테스트용 가상 더미 데이터
static uint16_t g_robot_axis[4]      = {[0]=512, [1]=300, [2]=750, [3]=1023};
static uint16_t g_motor_speed        = 500;
static uint16_t g_motor_load         = 100;
static uint8_t  g_motor_volt_raw[4]  = {[0]=115, [1]=114, [2]=115, [3]=113};

// 시스템 제어 상태 변수
static SystemMode_t g_system_mode     = MODE_TEACHING;
static RunState_t   g_run_state       = RUN_STATE_STOPPED;
static bool         g_emergency_stop  = false;
static uint8_t      g_selected_preset  = 0;

static TeachingPoint_t g_teach_memory[13];

// 4x4 키패드 GPIO 매핑
static GPIO_TypeDef* ROW_PORTS[4] = {GPIOC, GPIOC, GPIOC, GPIOC};
static uint16_t      ROW_PINS[4]  = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};

static GPIO_TypeDef* COL_PORTS[4] = {GPIOC, GPIOC, GPIOC, GPIOC};
static uint16_t      COL_PINS[4]  = {GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9};

// 1~4번과 5~8번 버튼 위치 교정 매핑
static const uint8_t KEY_MAP[4][4] = {
    { 5,  6,  7,  8},
    { 1,  2,  3,  4},
    { 9, 10, 11, 12},
    {13, 14, 15, 16}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t Keypad_Scan(void);
static void Process_Key_Event(uint8_t key);
static void Update_LCD1_Display(void);
static void Update_LCD2_Display(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// int __io_putchar(int ch)
// {
//   HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
//   return ch;
// }

static uint8_t Keypad_Scan(void)
{
  for (int r = 0; r < 4; r++)
  {
    for (int i = 0; i < 4; i++) {
      HAL_GPIO_WritePin(ROW_PORTS[i], ROW_PINS[i], GPIO_PIN_RESET);
    }

    HAL_GPIO_WritePin(ROW_PORTS[r], ROW_PINS[r], GPIO_PIN_SET);

    for(volatile int d=0; d<100; d++);

    for (int c = 0; c < 4; c++)
    {
      if (HAL_GPIO_ReadPin(COL_PORTS[c], COL_PINS[c]) == GPIO_PIN_SET)
      {
        return KEY_MAP[r][c];
      }
    }
  }

  return 0;
}

/* ---------------------------------------------------------------------------
 * LCD 1 화면 출력 (로봇 축 현재 위치)
 * --------------------------------------------------------------------------- */
static void Update_LCD1_Display(void)
{
  char buf[30];

  LCD2_CS_HIGH();
  LCD1_CS_LOW();

  LCD_PutString(10, 10, "[ CURRENT POSITION ]", ILI9341_YELLOW, ILI9341_BLACK, 2);

  sprintf(buf, "Axis1 : %04d        ", g_robot_axis[0]);
  LCD_PutString(10, 45, buf, ILI9341_GREEN, ILI9341_BLACK, 2);

  sprintf(buf, "Axis2 : %04d        ", g_robot_axis[1]);
  LCD_PutString(10, 75, buf, ILI9341_GREEN, ILI9341_BLACK, 2);

  sprintf(buf, "Axis3 : %04d        ", g_robot_axis[2]);
  LCD_PutString(10, 105, buf, ILI9341_GREEN, ILI9341_BLACK, 2);

  sprintf(buf, "Axis4 : %04d        ", g_robot_axis[3]);
  LCD_PutString(10, 135, buf, ILI9341_GREEN, ILI9341_BLACK, 2);

  LCD1_CS_HIGH();
}

/* ---------------------------------------------------------------------------
 * LCD 2 화면 출력 (모드 및 관리자 정보)
 * --------------------------------------------------------------------------- */
static void Update_LCD2_Display(void)
{
  char buf[30];

  LCD1_CS_HIGH();
  LCD2_CS_LOW();

  // 1. 비상 정지 (16번 버튼)
  if (g_emergency_stop)
  {
    LCD_PutString(10, 10, "[ EMERGENCY STOP! ] ", ILI9341_RED, ILI9341_BLACK, 2);
    LCD_PutString(10, 45, "* ALL BTNS LOCKED * ", ILI9341_WHITE, ILI9341_RED, 2);
    LCD_PutString(10, 80, "Press BTN15 for    ", ILI9341_YELLOW, ILI9341_BLACK, 2);
    LCD_PutString(10, 110, "ADMIN JOG MODE     ", ILI9341_YELLOW, ILI9341_BLACK, 2);
  }
  // 2. 관리자 JOG 모드 (15번 버튼)
  else if (g_system_mode == MODE_ADMIN_JOG)
  {
    LCD_PutString(10, 10, "[ ADMIN JOG MODE ]  ", ILI9341_CYAN, ILI9341_BLACK, 2);

    // 1줄: 속도, 하중
    sprintf(buf, "SPD:%04d LOAD:%04d  ", g_motor_speed, g_motor_load);
    LCD_PutString(10, 40, buf, ILI9341_WHITE, ILI9341_BLACK, 2);

    // 2줄: Axis 1, 2 전압 (소수점 둘째 자리)
    sprintf(buf, "A1:%.2fv A2:%.2fv   ", (float)g_motor_volt_raw[0]/10.0f, (float)g_motor_volt_raw[1]/10.0f);
    LCD_PutString(10, 70, buf, ILI9341_GREEN, ILI9341_BLACK, 2);

    // 3줄: Axis 3, 4 전압 (소수점 둘째 자리)
    sprintf(buf, "A3:%.2fv A4:%.2fv   ", (float)g_motor_volt_raw[2]/10.0f, (float)g_motor_volt_raw[3]/10.0f);
    LCD_PutString(10, 100, buf, ILI9341_GREEN, ILI9341_BLACK, 2);
  }
  // 3. 티칭 모드
  else if (g_system_mode == MODE_TEACHING)
  {
    LCD_PutString(10, 10, "[ TEACHING MODE ]   ", ILI9341_MAGENTA, ILI9341_BLACK, 2);
    if (g_selected_preset > 0) {
      sprintf(buf, "Saved Preset #%02d!  ", g_selected_preset);
      LCD_PutString(10, 45, buf, ILI9341_GREEN, ILI9341_BLACK, 2);
    } else {
      LCD_PutString(10, 45, "Press 1~12 to Save  ", ILI9341_WHITE, ILI9341_BLACK, 2);
    }
    LCD_PutString(10, 80, "Status: STOPPED     ", ILI9341_RED, ILI9341_BLACK, 2);
    LCD_PutString(10, 110, "                    ", ILI9341_BLACK, ILI9341_BLACK, 2);
  }
  // 4. 오토 모드
  else if (g_system_mode == MODE_AUTO)
  {
    LCD_PutString(10, 10, "[ AUTO MODE ]       ", ILI9341_YELLOW, ILI9341_BLACK, 2);
    
    sprintf(buf, "State: %s      ", (g_run_state == RUN_STATE_RUNNING) ? "RUNNING" : "STOPPED");
    LCD_PutString(10, 40, buf, (g_run_state == RUN_STATE_RUNNING) ? ILI9341_GREEN : ILI9341_RED, ILI9341_BLACK, 2);

    if (g_selected_preset > 0) {
      sprintf(buf, "Preset #%02d Target: ", g_selected_preset);
      LCD_PutString(10, 70, buf, ILI9341_WHITE, ILI9341_BLACK, 2);

      sprintf(buf, "A1:%04d A2:%04d ", g_teach_memory[g_selected_preset].axis[0], g_teach_memory[g_selected_preset].axis[1]);
      LCD_PutString(10, 95, buf, ILI9341_CYAN, ILI9341_BLACK, 2);

      sprintf(buf, "A3:%04d A4:%04d ", g_teach_memory[g_selected_preset].axis[2], g_teach_memory[g_selected_preset].axis[3]);
      LCD_PutString(10, 120, buf, ILI9341_CYAN, ILI9341_BLACK, 2);
    } else {
      LCD_PutString(10, 70, "Select Preset 1~12  ", ILI9341_WHITE, ILI9341_BLACK, 2);
      LCD_PutString(10, 95, "                    ", ILI9341_BLACK, ILI9341_BLACK, 2);
      LCD_PutString(10, 120, "                    ", ILI9341_BLACK, ILI9341_BLACK, 2);
    }
  }

  LCD2_CS_HIGH();
}

/* ---------------------------------------------------------------------------
 * 버튼 이벤트 로직
 * --------------------------------------------------------------------------- */
static void Process_Key_Event(uint8_t key)
{
  // 16번 비상 정지
  if (key == 16)
  {
    g_emergency_stop = true;
    g_run_state = RUN_STATE_STOPPED;
    printf("[EMERGENCY] E-STOP ACTIVATED!\r\n");
    return;
  }

  // 15번 관리자 모드
  if (key == 15)
  {
    if (g_emergency_stop) {
      g_emergency_stop = false;
      printf("[ADMIN] E-STOP Unlocked!\r\n");
    }
    g_system_mode = MODE_ADMIN_JOG;
    g_run_state = RUN_STATE_STOPPED;
    printf("[ADMIN] Entered ADMIN JOG Mode\r\n");
    return;
  }

  // 비상 정지 락 상태
  if (g_emergency_stop)
  {
    printf("[LOCKED] Press BTN 15 first.\r\n");
    return;
  }

  // 1~12번 프리셋
  if (key >= 1 && key <= 12)
  {
    g_selected_preset = key;

    if (g_system_mode == MODE_TEACHING)
    {
      g_teach_memory[key].axis[0] = g_robot_axis[0];
      g_teach_memory[key].axis[1] = g_robot_axis[1];
      g_teach_memory[key].axis[2] = g_robot_axis[2];
      g_teach_memory[key].axis[3] = g_robot_axis[3];
      printf("[TEACHING] Saved Preset #%02d\r\n", key);
    }
    else if (g_system_mode == MODE_AUTO)
    {
      printf("[AUTO] Selected Preset #%02d\r\n", key);
    }
  }
  // 13번 모드 전환
  else if (key == 13)
  {
    if (g_system_mode == MODE_TEACHING) {
      g_system_mode = MODE_AUTO;
      printf("[MODE] Switched to AUTO MODE\r\n");
    } else {
      g_system_mode = MODE_TEACHING;
      g_run_state = RUN_STATE_STOPPED;
      printf("[MODE] Switched to TEACHING MODE\r\n");
    }
  }
  // 14번 구동 토글
  else if (key == 14)
  {
    if (g_system_mode == MODE_AUTO)
    {
      g_run_state = (g_run_state == RUN_STATE_STOPPED) ? RUN_STATE_RUNNING : RUN_STATE_STOPPED;
      printf("[RUN] State: %s\r\n", (g_run_state == RUN_STATE_RUNNING) ? "RUNNING" : "STOPPED");
    }
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init(); // Teleport 콘솔 (115200bps)
  MX_SPI1_Init();        // Dual LCD 통신
  MX_USART1_UART_Init(); // 블루투스 (미연결 상태)

  /* USER CODE BEGIN 2 */
  // Dual LCD 하드웨어 리셋 시퀀스
  LCD1_CS_HIGH();
  LCD2_CS_HIGH();
  HAL_Delay(50);

  LCD_RST_LOW();
  HAL_Delay(50);
  LCD_RST_HIGH();
  HAL_Delay(150);

  // LCD 1 초기화 (검은색 바탕)
  LCD1_CS_LOW();
  LCD2_CS_HIGH();
  ILI9341_Init();
  ILI9341_FillScreen(ILI9341_BLACK);
  LCD1_CS_HIGH();

  HAL_Delay(50);

  // LCD 2 초기화 (검은색 바탕)
  LCD1_CS_HIGH();
  LCD2_CS_LOW();
  ILI9341_Init();
  ILI9341_FillScreen(ILI9341_BLACK);
  LCD2_CS_HIGH();

  // 최초 화면 출력
  Update_LCD1_Display();
  Update_LCD2_Display();

  printf("\r\n========================================\r\n");
  printf("  LCD & KEYPAD DISPLAY TEST READY       \r\n");
  printf("========================================\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t key = 0;
  uint8_t prev_key = 0;

  while (1)
  {
    // 1. 키패드 스캔
    key = Keypad_Scan();

    if (key != 0 && prev_key == 0) // 버튼 눌림 감지
    {
      printf("[KEYPAD] Pressed BTN #%02d\r\n", key);

      // 로직 처리 및 LCD 화면 즉시 새로고침
      Process_Key_Event(key);
      Update_LCD1_Display();
      Update_LCD2_Display();
    }
    prev_key = key;

    HAL_Delay(30); // 디바운스 대기
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */