/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file          : freertos.c
  * @brief         : FreeRTOS Tasks & Hardware Control Logic
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "spi.h"
#include "gpio.h"
#include "ili9341.h"
#include "ax12.h"
#include "lcd_font.h"
#include "teaching_storage.h"
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
    RUN_STATE_RUNNING = 1,
    RUN_STATE_COMPLETED = 2
} RunState_t;

typedef enum {
    MOTION_NONE = 0,
    MOTION_AUTO,
    MOTION_HOME,
    MOTION_AUTO_RETURN_HOME
} MotionType_t;

typedef enum {
    MASTER_CTRL_ACTION_NONE = 0,
    MASTER_CTRL_ACTION_HOME,
    MASTER_CTRL_ACTION_ESTOP_SYNC,
    MASTER_CTRL_ACTION_MANUAL
} MasterControllerAction_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BT_CMD_SET_GOAL_POS 0x01
#define BT_CMD_SET_TORQUE   0x02
#define BT_CMD_REQ_STATUS   0x03
#define BT_CMD_HOME_POS     0x04
#define BT_CMD_SET_ALL_POS  0x05
#define BT_CMD_START_AUTO    0x06
#define BT_CMD_RUN_AUTO      0x07
#define BT_CMD_HOLD_CURRENT  0x08
#define BT_CMD_STATUS_REPLY 0x83

#define BT_DEFAULT_JOG_PERIOD_MS       10U
#define BT_DEFAULT_STATUS_PERIOD_MS   200U
/* Faster arrival feedback only while BTN14 sequence execution is active. */
#define BT_AUTO_STATUS_PERIOD_MS        10U /* Auto arrival feedback period. */
#define BT_STATUS_PAYLOAD_LENGTH       20U
#define BT_RX_BUFFER_SIZE              64U
#define SHARP_AUTO_COUNTDOWN_MS       5000U
#define SHARP_AUTO_COUNTDOWN_SECONDS      5U
#define SHARP_AUTO_START_DISPLAY_MS    750U
#define SHARP_STATUS_TIMEOUT_MS         600U
/* Home completion is based on actual slave positions.  Keep this slightly
 * wider than normal settling noise so a mechanically limited axis cannot
 * leave the startup interlock permanently on the MOVING screen. */
#define HOME_POSITION_TOLERANCE        30U
#define TEACHING_STEP_POSITION_TOLERANCE 5U
#define MOTION_STABLE_STATUS_COUNT      3U
#define AX12_DEFAULT_POSITION         512U
#define EMERGENCY_STOP_ENABLED          1U


/* LCD Display Layout & Font Configuration Macros */
#define LCD1_TITLE_SCALE    2     
#define LCD1_BODY_SCALE     3     
#define LCD2_TITLE_SCALE    2     
#define LCD2_BODY_SCALE     2     

#define LCD1_START_X        10    
#define LCD1_START_Y        10    
#define LCD1_LINE_HEIGHT    48    // 수정: 0으로 되어 있어 겹치던 줄 간격을 글자 크기에 맞게 35로 수정

#define LCD2_START_X        10    
#define LCD2_START_Y        10    
#define LCD2_LINE_HEIGHT    30    
#define KEYPAD_SCAN_PERIOD_MS        2U
#define KEYPAD_DEBOUNCE_SAMPLES      3U
#define KEYPAD_EVENT_QUEUE_DEPTH    16U
#define AX12_ADMIN_MONITOR_PERIOD_MS 150U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 전역 변수 영역에 버퍼 및 포인터 선언 (또는 파일 상단 USER CODE BEGIN PV 영역)
uint8_t pc_to_bt[64];
uint8_t bt_to_pc[64];
uint8_t pc_to_bt_head = 0;
uint8_t pc_to_bt_tail = 0;
uint8_t bt_to_pc_head = 0;
uint8_t bt_to_pc_tail = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t g_robot_axis[4]       = {512, 512, 512, 512};
volatile uint16_t g_master_voltage_mv[4] = {0U, 0U, 0U, 0U};
volatile uint8_t g_master_temperature_c[4] = {0U, 0U, 0U, 0U};
uint16_t g_prev_axis[4]       = {0, 0, 0, 0};
volatile uint16_t g_slave_axis[4] = {512, 512, 512, 512};
volatile uint16_t g_slave_load[4] = {0, 0, 0, 0};
uint16_t g_motor_speed        = 300;
volatile uint16_t g_motor_load = 0;

SystemMode_t g_system_mode     = MODE_TEACHING;
SystemMode_t g_prev_mode       = 0xFF; 
RunState_t   g_run_state       = RUN_STATE_STOPPED;
RunState_t   g_prev_run_state  = 0xFF;
bool         g_emergency_stop  = false;
bool         g_prev_estop      = false;
uint8_t      g_selected_preset = 0;
uint8_t      g_homing_status   = 0; /* 0:idle, 1:moving, 2:completed */
/* Boot interlock: no keypad motion command is accepted until BTN16 Home
 * reaches the verified slave positions. */
static volatile bool g_home_ready = false;
volatile bool g_admin_jog_enabled = false;
static volatile bool g_admin_dashboard_enabled = false;
static volatile bool g_admin_dashboard_clear_requested = false;
static volatile bool g_estop_request = false;
volatile uint32_t g_slave_status_sequence = 0U;
volatile bool g_auto_motion_released = false;
static volatile bool g_sharp_detected = false;
static volatile uint16_t g_sharp_mv = 0U;
static volatile uint16_t g_sharp_distance_cm = 80U;
static volatile uint8_t g_slave_status_flags = 0U;
static volatile bool g_sharp_auto_start_pending = false;
/* Require the object to leave the sensing range before another countdown. */
static volatile bool g_sharp_auto_rearm_required = false;
static bool g_sharp_countdown_active = false;
static volatile uint8_t g_sharp_countdown_value = 0U;
static uint32_t g_sharp_detected_since_ms = 0U;
static uint32_t g_sharp_auto_start_due_ms = 0U;
static volatile uint32_t g_sharp_status_updated_ms = 0U;
uint8_t g_teach_save_status = 0U; /* 0:none, 1:saved, 2:flash error */
uint8_t g_teach_delete_status = 0U; /* 0:none, 1:deleted, 2:flash error, 3:no preset */
volatile uint32_t g_teach_save_event_sequence = 0U;
volatile uint32_t g_lcd_event_sequence = 0U;

TeachingSequence_t g_teach_memory[TEACHING_PRESET_COUNT + 1U];
uint8_t g_teach_capture_step = 0U;
uint8_t g_teach_last_saved_step = 0U;
uint8_t g_auto_sequence_step = 0U;
static volatile bool g_auto_step_delay_active = false;
/* True after the final preset target has been confirmed at the robot. */
static volatile bool g_auto_sequence_last_sent = false;
static uint32_t g_auto_step_due_ms = 0U;
static uint16_t g_home_positions[4] = {512U, 512U, 512U, 512U};
static uint32_t g_home_retry_due_ms = 0U;
static uint16_t g_motion_target[4] = {512U, 512U, 512U, 512U};
static uint16_t g_estop_sync_target[4] = {512U, 512U, 512U, 512U};
static volatile bool g_estop_waiting_slave_pose = false;
static volatile bool g_estop_sync_active = false;
static volatile bool g_estop_jog_ready = false;
static uint8_t g_estop_sync_stable_count = 0U;
static volatile MotionType_t g_motion_type = MOTION_NONE;
static volatile MasterControllerAction_t g_master_controller_action =
    MASTER_CTRL_ACTION_NONE;
static volatile uint32_t g_bt_jog_period_ms = BT_DEFAULT_JOG_PERIOD_MS;
static volatile uint32_t g_bt_status_period_ms = BT_DEFAULT_STATUS_PERIOD_MS;
static volatile bool teleplot_tx_busy = false;
static uint8_t bt_rx_byte;
static uint8_t bt_rx_buffer[BT_RX_BUFFER_SIZE];
static volatile uint8_t bt_rx_head = 0U;
static volatile uint8_t bt_rx_tail = 0U;
static volatile uint32_t bt_rx_dropped_count = 0U;

// Keypad GPIO
static GPIO_TypeDef* ROW_PORTS[4] = {GPIOC, GPIOC, GPIOC, GPIOC};
static uint16_t       ROW_PINS[4]  = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};
static GPIO_TypeDef* COL_PORTS[4] = {GPIOC, GPIOC, GPIOC, GPIOC};
static uint16_t       COL_PINS[4]  = {GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9};

static const uint8_t KEY_MAP[4][4] = {
    { 1,  2,  3,  4},
    { 5,  6,  7,  8},
    { 9, 10, 11, 12},
    {13, 14, 15, 16}
};
/* USER CODE END Variables */
/* Definitions for Bluetooth */
osThreadId_t BluetoothHandle;
const osThreadAttr_t Bluetooth_attributes = {
  .name = "Bluetooth",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LCD1_Task */
osThreadId_t LCD1_TaskHandle;
const osThreadAttr_t LCD1_Task_attributes = {
  .name = "LCD1_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for LCD2_Task */
osThreadId_t LCD2_TaskHandle;
const osThreadAttr_t LCD2_Task_attributes = {
  .name = "LCD2_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Keypad_TaskHand */
osThreadId_t Keypad_TaskHandHandle;
const osThreadAttr_t Keypad_TaskHand_attributes = {
  .name = "Keypad_TaskHand",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for AX_12 */
osThreadId_t AX_12Handle;
const osThreadAttr_t AX_12_attributes = {
  .name = "AX_12",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Teleplot */
osThreadId_t TeleplotHandle;
const osThreadAttr_t Teleplot_attributes = {
  .name = "Teleplot",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for lcdSpiMutex */
osMutexId_t lcdSpiMutexHandle;
const osMutexAttr_t lcdSpiMutex_attributes = {
  .name = "lcdSpiMutex"
};
/* Definitions for btUartMutex */
osMutexId_t btUartMutexHandle;
const osMutexAttr_t btUartMutex_attributes = {
  .name = "btUartMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint8_t Keypad_Scan(void);
void Process_Key_Event(uint8_t key);
void BT_SendPacket(uint8_t cmd, uint8_t *data, uint8_t len);
void Robot_SetGoalPosition(uint8_t motor_id, uint16_t position);
void Robot_SetTorque(uint8_t enable);
void Robot_MoveToHome(void);
void Robot_SetHomePositions(const uint16_t positions[4]);
static void ActivateEmergencyStop(void);
static void CancelHomeMotion(void);
static void MasterController_RequestAction(MasterControllerAction_t action);
static void SetSystemMode(SystemMode_t mode);
static uint8_t TeachingSequence_CountSaved(uint8_t preset);
static uint8_t TeachingSequence_Rank(uint8_t preset, uint8_t step);
void BT_SetJogTransmitPeriod(uint32_t period_ms);
void BT_SetStatusPeriod(uint32_t period_ms);
static void BT_ProcessReceivedByte(uint8_t byte);
static void BT_SendAllPositions(const uint16_t positions[4]);
static void BT_SendAutoPositions(const uint16_t positions[4]);
static void BT_SendAutoStartPositions(const uint16_t positions[4]);
static void BT_SendPositionsCommand(uint8_t cmd, const uint16_t positions[4]);
static void BT_RequestStatus(void);
static bool BT_QueueTransmit(const uint8_t *data, uint16_t length);
static void AutoScheduleFollowingStep(void);
static bool AutoStartPreset(uint8_t preset);
static void SharpAuto_ResetCountdown(void);
void Update_LCD1_Clean(void);
void Update_LCD2_Clean(void);
/* USER CODE END FunctionPrototypes */

void Start_Bluetooth(void *argument);
void StartLCD1Task(void *argument);
void StartLCD2Task(void *argument);
void Keypad_Task(void *argument);
void Start_AX_12(void *argument);
void Start_Teleplot(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  for(uint8_t p=0U;p<=TEACHING_PRESET_COUNT;p++)
  {
    g_teach_memory[p].saved_mask=0U;
    for(uint8_t s=0U;s<TEACHING_SEQUENCE_STEPS;s++)
      for(uint8_t a=0U;a<4U;a++)
        g_teach_memory[p].step[s].axis[a]=TEACHING_EMPTY_AXIS_VALUE;
  }
  (void)TeachingStorage_Load(g_teach_memory);
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of lcdSpiMutex */
  lcdSpiMutexHandle = osMutexNew(&lcdSpiMutex_attributes);

  /* creation of btUartMutex */
  btUartMutexHandle = osMutexNew(&btUartMutex_attributes);

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
  /* creation of Bluetooth */
  BluetoothHandle = osThreadNew(Start_Bluetooth, NULL, &Bluetooth_attributes);

  /* creation of LCD1_Task */
  LCD1_TaskHandle = osThreadNew(StartLCD1Task, NULL, &LCD1_Task_attributes);

  /* creation of LCD2_Task */
  LCD2_TaskHandle = osThreadNew(StartLCD2Task, NULL, &LCD2_Task_attributes);

  /* creation of Keypad_TaskHand */
  Keypad_TaskHandHandle = osThreadNew(Keypad_Task, NULL, &Keypad_TaskHand_attributes);

  /* creation of AX_12 */
  AX_12Handle = osThreadNew(Start_AX_12, NULL, &AX_12_attributes);

  /* creation of Teleplot */
  TeleplotHandle = osThreadNew(Start_Teleplot, NULL, &Teleplot_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_Bluetooth */
/**
  * @brief  Function implementing the Bluetooth thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Bluetooth */
void Start_Bluetooth(void *argument)
{
  /* USER CODE BEGIN Start_Bluetooth */
  (void)argument;
  (void)HAL_UART_Receive_IT(&huart6, &bt_rx_byte, 1U);
  /* AT bridge: COM7 (USART2) <-> Bluetooth module (USART6). */
  for(;;)
  {
    uint8_t byte;

    if (HAL_UART_Receive(&huart2, &byte, 1U, 1U) == HAL_OK)
    {
      (void)BT_QueueTransmit(&byte, 1U);
    }

    /* USART6 runs in one-byte interrupt mode.  Drain every queued byte here;
     * polling one byte then delaying 1ms loses most of a 115200-bps status
     * frame and prevents AUTO Step completion from being detected. */
    while (bt_rx_tail != bt_rx_head)
    {
      byte = bt_rx_buffer[bt_rx_tail];
      bt_rx_tail = (uint8_t)((bt_rx_tail + 1U) % BT_RX_BUFFER_SIZE);
      BT_ProcessReceivedByte(byte);
    }

    osDelay(1U);
  }
  /* USER CODE END Start_Bluetooth */
}

/* USER CODE BEGIN Header_StartLCD1Task */
/**
* @brief Function implementing the LCD1_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCD1Task */
void StartLCD1Task(void *argument)
{
  /* USER CODE BEGIN StartLCD1Task */
  LCD1_CS_HIGH(); 
  osDelay(50);
  LCD_RST_LOW(); osDelay(50); LCD_RST_HIGH(); osDelay(150);

  if (osMutexWait(lcdSpiMutexHandle, osWaitForever) == osOK) {
    LCD1_CS_LOW();
    ILI9341_Init(); 
    ILI9341_FillScreen(ILI9341_BLACK);
    LCD1_CS_HIGH();
    osMutexRelease(lcdSpiMutexHandle);
  }

  for(;;)
  {
    Update_LCD1_Clean();
    osDelay(20U); // 최적화: LCD 갱신 주기를 당겨서 반응 속도 향상 (기존 100ms -> 50ms)
  }
  /* USER CODE END StartLCD1Task */
}

/* USER CODE BEGIN Header_StartLCD2Task */
/**
* @brief Function implementing the LCD2_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCD2Task */
void StartLCD2Task(void *argument)
{
  /* USER CODE BEGIN StartLCD2Task */
  osDelay(200); // LCD1과 초기화 충돌 방지 딜레이
  if (osMutexWait(lcdSpiMutexHandle, osWaitForever) == osOK) {
    LCD2_CS_LOW();
    ILI9341_Init(); 
    ILI9341_FillScreen(ILI9341_BLACK);
    LCD2_CS_HIGH();
    osMutexRelease(lcdSpiMutexHandle);
  }

  for(;;)
  {
    Update_LCD2_Clean();
    osDelay(50U); // 최적화: LCD2 갱신 주기 단축 (기존 150ms -> 80ms)
  }
  /* USER CODE END StartLCD2Task */
}

/* USER CODE BEGIN Header_Keypad_Task */
/**
* @brief Function implementing the Keypad_TaskHand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Keypad_Task */
void Keypad_Task(void *argument)
{
  /* USER CODE BEGIN Keypad_Task */
  uint8_t key = 0, stable_key = 0, candidate_key = 0, stable_count = 0;
  bool estop_button_was_pressed =
      (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_SET);

  for(;;)
  {
    /* PB2 is edge-triggered.  Polling is only a backup for EXTI2 and must
     * also detect an edge, never a continuously HIGH input. */
    bool estop_button_pressed =
        (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_SET);
    if (g_estop_request ||
        (estop_button_pressed && !estop_button_was_pressed))
    {
      g_estop_request = false;
      ActivateEmergencyStop();
    }
    estop_button_was_pressed = estop_button_pressed;

    /* 20 ms stable debounce prevents matrix bounce without missing a short
     * button press to be missed or interpreted as a second press. */
    key = Keypad_Scan();
    if (key == candidate_key)
    {
      if (stable_count < KEYPAD_DEBOUNCE_SAMPLES) ++stable_count;
    }
    else
    {
      candidate_key = key;
      stable_count = 1U;
    }
    if ((stable_count >= KEYPAD_DEBOUNCE_SAMPLES) && (stable_key != candidate_key))
    {
      stable_key = candidate_key;
      if (stable_key != 0U)
      {
        /* The keypad task is the single event consumer.  Dispatch the stable
         * press here; the previous queue handle was removed by CubeMX and had
         * no consumer, which left key events unprocessed. */
        Process_Key_Event(stable_key);
      }
    }

    osDelay(KEYPAD_SCAN_PERIOD_MS);
  }
  /* USER CODE END Keypad_Task */
}

/* USER CODE BEGIN Header_Start_AX_12 */
/**
* @brief Function implementing the AX_12 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_AX_12 */
void Start_AX_12(void *argument)
{
  /* USER CODE BEGIN Start_AX_12 */
  uint8_t motor_index = 0U;
  uint8_t voltage_motor_index = 0U;
  bool read_pending = false;
  uint32_t read_started_ms = 0U;
  uint32_t last_voltage_read_ms = 0U;
  uint32_t last_tx_ms = 0U;
  uint32_t last_status_request_ms = 0U;
  (void)argument;
  osDelay(350U);

  /* Initialization may block once; the continuous position loop below does not. */
  for (uint8_t i = 0U; i < AX12_NUM_MOTORS; ++i)
  {
    uint8_t id = AX12_GetMotorId(i);
    uint8_t status[6];

    if (AX12_Ping(id, status, sizeof(status)) == HAL_OK)
    {
      printf("Master controller AX12 ID %u: detected\r\n", (unsigned)id);
    }
    else
    {
      printf("Master controller AX12 ID %u: no response\r\n", (unsigned)id);
    }

    /* The master is a hand-operated controller.  Always request torque OFF,
     * even when the initial Ping was delayed, so every axis can be moved. */
    (void)AX12_Write1(id, AX12_ADDR_TORQUE_ENABLE, 0U);
  }

  for (;;)
  {
    uint32_t now_ms = HAL_GetTick();

    if (g_sharp_detected &&
        ((now_ms - g_sharp_status_updated_ms) > SHARP_STATUS_TIMEOUT_MS))
    {
      g_sharp_detected = false;
      SharpAuto_ResetCountdown();
      ++g_lcd_event_sequence;
    }

    /* The IR sensor is armed only after the operator enters AUTO and selects
     * a valid teaching preset. After a run, the object must leave the sensor
     * range before another five-second countdown may begin. */
    if (g_sharp_auto_rearm_required)
    {
      SharpAuto_ResetCountdown();
      if (!g_sharp_detected && (g_run_state == RUN_STATE_COMPLETED))
      {
        g_sharp_auto_rearm_required = false;
        if (g_run_state == RUN_STATE_COMPLETED)
        {
          g_run_state = RUN_STATE_STOPPED;
        }
        ++g_lcd_event_sequence;
      }
    }

    if (g_home_ready && !g_emergency_stop &&
        (g_system_mode == MODE_AUTO) &&
        (g_run_state == RUN_STATE_STOPPED) &&
        !g_sharp_auto_rearm_required &&
        (g_selected_preset >= 1U) &&
        (g_selected_preset <= TEACHING_PRESET_COUNT) &&
        (g_teach_memory[g_selected_preset].saved_mask != 0U))
    {
      if (!g_sharp_detected)
      {
        SharpAuto_ResetCountdown();
      }
      else if (!g_sharp_auto_start_pending)
      {
        uint32_t elapsed_ms;
        uint8_t remaining;

        if (!g_sharp_countdown_active)
        {
          g_sharp_countdown_active = true;
          g_sharp_detected_since_ms = now_ms;
          g_sharp_countdown_value = SHARP_AUTO_COUNTDOWN_SECONDS;
          ++g_lcd_event_sequence;
        }

        elapsed_ms = now_ms - g_sharp_detected_since_ms;
        if (elapsed_ms >= SHARP_AUTO_COUNTDOWN_MS)
        {
          g_sharp_auto_start_pending = true;
          g_sharp_auto_start_due_ms =
              now_ms + SHARP_AUTO_START_DISPLAY_MS;
          g_sharp_countdown_value = 0U;
          ++g_lcd_event_sequence;
        }
        else
        {
          remaining = (uint8_t)(SHARP_AUTO_COUNTDOWN_SECONDS -
                                (elapsed_ms / 1000U));
          if (remaining != g_sharp_countdown_value)
          {
            g_sharp_countdown_value = remaining;
            ++g_lcd_event_sequence;
          }
        }
      }
      else if ((int32_t)(now_ms - g_sharp_auto_start_due_ms) >= 0)
      {
        g_sharp_auto_start_pending = false;
        (void)AutoStartPreset(g_selected_preset);
      }
    }
    else
    {
      SharpAuto_ResetCountdown();
    }

    if (g_auto_step_delay_active && ((int32_t)(now_ms - g_auto_step_due_ms) >= 0))
    {
      uint8_t next_step = (uint8_t)(g_auto_sequence_step + 1U);
      while ((next_step < TEACHING_SEQUENCE_STEPS) &&
             ((g_teach_memory[g_selected_preset].saved_mask &
               (1U << next_step)) == 0U))
      {
        ++next_step;
      }

      g_auto_step_delay_active = false;
      if (next_step < TEACHING_SEQUENCE_STEPS)
      {
        g_auto_sequence_step = next_step;
        for (uint8_t i = 0U; i < 4U; ++i)
          g_motion_target[i] = g_teach_memory[g_selected_preset].step[next_step].axis[i];
        BT_SendAutoPositions(g_motion_target);
        ++g_lcd_event_sequence;
      }
      else { g_auto_sequence_last_sent = true; }
    }

    /* Home is safety-critical.  The first command is sent immediately by
     * BTN16; repeat it while Home is still active so a busy/temporarily lost
     * Bluetooth frame cannot leave only the master controller moving. */
    if (((g_motion_type == MOTION_HOME) ||
         (g_motion_type == MOTION_AUTO_RETURN_HOME)) &&
        ((int32_t)(now_ms - g_home_retry_due_ms) >= 0))
    {
      BT_SendPositionsCommand(BT_CMD_HOME_POS, g_home_positions);
      g_home_retry_due_ms = now_ms + 250U;
    }

    /* BTN16 home / manual-release commands are queued here rather than sent
     * from a keypad handler, so they never collide with USART1 read IT. */
    if (g_master_controller_action != MASTER_CTRL_ACTION_NONE)
    {
      MasterControllerAction_t action;
      if (read_pending)
      {
        AX12_CancelPositionReadIT();
        read_pending = false;
      }

      __disable_irq();
      action = g_master_controller_action;
      g_master_controller_action = MASTER_CTRL_ACTION_NONE;
      __enable_irq();

      for (uint8_t i = 0U; i < AX12_NUM_MOTORS; ++i)
      {
        uint8_t id = AX12_GetMotorId(i);
        if (action == MASTER_CTRL_ACTION_HOME)
        {
          (void)AX12_Write1(id, AX12_ADDR_TORQUE_ENABLE, 1U);
          (void)AX12_Write2(id, AX12_ADDR_MOVING_SPEED,
                            AX12_MASTER_HOME_SPEED);
          (void)AX12_Write2(id, AX12_ADDR_GOAL_POSITION, 512U);
        }
        else if (action == MASTER_CTRL_ACTION_ESTOP_SYNC)
        {
          (void)AX12_Write1(id, AX12_ADDR_TORQUE_ENABLE, 1U);
          (void)AX12_Write2(id, AX12_ADDR_MOVING_SPEED,
                            AX12_MASTER_ESTOP_SYNC_SPEED);
          (void)AX12_Write2(id, AX12_ADDR_GOAL_POSITION,
                            g_estop_sync_target[i]);
        }
        else
        {
          (void)AX12_Write1(id, AX12_ADDR_TORQUE_ENABLE, 0U);
        }
      }
      osDelay(1U);
      continue;
    }

    if (read_pending)
    {
      uint16_t position;
      HAL_StatusTypeDef result = AX12_GetPositionReadITResult(&position);

      if (result == HAL_OK)
      {
        uint8_t motor_id = AX12_GetMotorId(motor_index);
        uint16_t min_position = AX12_MASTER_4_MIN_POSITION;
        uint16_t max_position = AX12_MASTER_4_MAX_POSITION;

        if (motor_id == AX12_MASTER_1_ID)
        {
          min_position = AX12_MASTER_1_MIN_POSITION;
          max_position = AX12_MASTER_1_MAX_POSITION;
        }
        else if (motor_id == AX12_MASTER_2_ID)
        {
          min_position = AX12_MASTER_2_MIN_POSITION;
          max_position = AX12_MASTER_2_MAX_POSITION;
        }
        else if (motor_id == AX12_MASTER_3_ID)
        {
          min_position = AX12_MASTER_3_MIN_POSITION;
          max_position = AX12_MASTER_3_MAX_POSITION;
        }

        if (position < min_position)
        {
          position = min_position;
        }
        else if (position > max_position)
        {
          position = max_position;
        }
        g_robot_axis[motor_index] = position;

        if (g_estop_sync_active)
        {
          bool aligned = true;
          for (uint8_t axis = 0U; axis < AX12_NUM_MOTORS; ++axis)
          {
            uint16_t error = (g_robot_axis[axis] > g_estop_sync_target[axis]) ?
                             (g_robot_axis[axis] - g_estop_sync_target[axis]) :
                             (g_estop_sync_target[axis] - g_robot_axis[axis]);
            if (error > AX12_MASTER_ESTOP_SYNC_TOLERANCE)
            {
              aligned = false;
              break;
            }
          }
          if (aligned)
          {
            if (g_estop_sync_stable_count < 3U) ++g_estop_sync_stable_count;
            if (g_estop_sync_stable_count >= 3U)
            {
              g_estop_sync_active = false;
              g_estop_jog_ready = true;
              MasterController_RequestAction(MASTER_CTRL_ACTION_MANUAL);
              ++g_lcd_event_sequence;
            }
          }
          else
          {
            g_estop_sync_stable_count = 0U;
          }
        }
        read_pending = false;
        motor_index = (uint8_t)((motor_index + 1U) % AX12_NUM_MOTORS);
      }
      else if ((result == HAL_ERROR) ||
               ((now_ms - read_started_ms) >= 20U))
      {
        AX12_CancelPositionReadIT();
        read_pending = false;
        motor_index = (uint8_t)((motor_index + 1U) % AX12_NUM_MOTORS);
      }
    }
    else if ((now_ms - last_voltage_read_ms) >=
             (g_admin_jog_enabled ? AX12_ADMIN_MONITOR_PERIOD_MS : 250U))
    {
      uint8_t voltage_raw = 0U;
      uint8_t temperature_raw = 0U;

      /* AX-12A: address 42 is Present Voltage (0.1 V), address 43 is temperature. */
      if (AX12_Read2(AX12_GetMotorId(voltage_motor_index),
                     AX12_ADDR_PRESENT_VOLTAGE,
                     &voltage_raw, &temperature_raw) == HAL_OK)
      {
        g_master_voltage_mv[voltage_motor_index] =
            (uint16_t)voltage_raw * 100U;
        g_master_temperature_c[voltage_motor_index] = temperature_raw;
      }

      voltage_motor_index =
          (uint8_t)((voltage_motor_index + 1U) % AX12_NUM_MOTORS);
      last_voltage_read_ms = now_ms;
    }
    else if (AX12_StartPositionReadIT(AX12_GetMotorId(motor_index)) == HAL_OK)
    {
      read_pending = true;
      read_started_ms = now_ms;
    }

    if (g_admin_jog_enabled &&
        ((now_ms - last_tx_ms) >= g_bt_jog_period_ms))
    {
      last_tx_ms = now_ms;
      BT_SendAllPositions(g_robot_axis);
    }

    uint32_t status_period_ms =
        (((g_system_mode == MODE_AUTO) && (g_run_state == RUN_STATE_RUNNING)) ||
         (g_motion_type == MOTION_HOME) ||
         (g_motion_type == MOTION_AUTO_RETURN_HOME) ||
         g_sharp_auto_rearm_required) ?
        BT_AUTO_STATUS_PERIOD_MS : g_bt_status_period_ms;
    if (!g_emergency_stop &&
        ((now_ms - last_status_request_ms) >= status_period_ms))
    {
      last_status_request_ms = now_ms;
      BT_RequestStatus();
    }

    osDelay(1U);
  }
  /* USER CODE END Start_AX_12 */
}

/* USER CODE BEGIN Header_Start_Teleplot */
/**
* @brief Function implementing the Teleplot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Teleplot */
void Start_Teleplot(void *argument)
{
  /* USER CODE BEGIN Start_Teleplot */
  (void)argument;
  for(;;)
  {
    /* Teleplot format: >variable_name:value */
    printf(">master_axis1:%u\r\n", (unsigned)g_robot_axis[0]);
    printf(">master_axis2:%u\r\n", (unsigned)g_robot_axis[1]);
    printf(">master_axis3:%u\r\n", (unsigned)g_robot_axis[2]);
    printf(">master_axis4:%u\r\n", (unsigned)g_robot_axis[3]);
    printf(">master_voltage1_mV:%u\r\n", (unsigned)g_master_voltage_mv[0]);
    printf(">master_voltage2_mV:%u\r\n", (unsigned)g_master_voltage_mv[1]);
    printf(">master_voltage3_mV:%u\r\n", (unsigned)g_master_voltage_mv[2]);
    printf(">master_voltage4_mV:%u\r\n", (unsigned)g_master_voltage_mv[3]);
    printf(">master_sharp_detected:%u\r\n",
           g_sharp_detected ? 1U : 0U);
    printf(">master_sharp_mv:%u\r\n", (unsigned)g_sharp_mv);
    printf(">master_sharp_cm:%u\r\n", (unsigned)g_sharp_distance_cm);
    printf(">slave_status_flags:%u\r\n",
           (unsigned)g_slave_status_flags);
    printf(">slave_status_age_ms:%lu\r\n",
           (unsigned long)((g_sharp_status_updated_ms == 0U) ? 0U :
                           (HAL_GetTick() - g_sharp_status_updated_ms)));
    printf(">auto_home_ready:%u\r\n", g_home_ready ? 1U : 0U);
    printf(">auto_system_mode:%u\r\n", (unsigned)g_system_mode);
    printf(">auto_run_state:%u\r\n", (unsigned)g_run_state);
    printf(">auto_selected_preset:%u\r\n", (unsigned)g_selected_preset);
    printf(">auto_selected_preset_mask:%lu\r\n",
           (unsigned long)((g_selected_preset <= TEACHING_PRESET_COUNT) ?
                           g_teach_memory[g_selected_preset].saved_mask : 0U));
    printf(">auto_countdown:%u\r\n",
           (unsigned)g_sharp_countdown_value);
    printf(">auto_start_pending:%u\r\n",
           g_sharp_auto_start_pending ? 1U : 0U);
    osDelay(50U);
  }
  /* USER CODE END Start_Teleplot */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#define BT_TX_BUFFER_SIZE 32U

static uint8_t bt_tx_active[BT_TX_BUFFER_SIZE];
static uint8_t bt_tx_pending[BT_TX_BUFFER_SIZE];
static volatile bool bt_tx_busy;
static volatile bool bt_tx_pending_ready;
static uint16_t bt_tx_active_length;
static uint16_t bt_tx_pending_length;

static bool BT_IsUrgentFrame(const uint8_t *data, uint16_t length)
{
  return (length >= 5U) &&
         ((data[2] == BT_CMD_HOME_POS) ||
          (data[2] == BT_CMD_START_AUTO) ||
          (data[2] == BT_CMD_RUN_AUTO) ||
          (data[2] == BT_CMD_HOLD_CURRENT) ||
          (data[2] == BT_CMD_SET_TORQUE));
}

static bool BT_IsStatusFrame(const uint8_t *data, uint16_t length)
{
  return (length >= 5U) && (data[2] == BT_CMD_REQ_STATUS);
}

static bool BT_IsJogFrame(const uint8_t *data, uint16_t length)
{
  return (length >= 5U) && (data[2] == BT_CMD_SET_ALL_POS);
}

int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

static bool BT_QueueTransmit(const uint8_t *data, uint16_t length)
{
  bool start_now = false;

  if ((data == NULL) || (length == 0U) || (length > BT_TX_BUFFER_SIZE))
  {
    return false;
  }

  __disable_irq();
  if (!bt_tx_busy)
  {
    memcpy(bt_tx_active, data, length);
    bt_tx_active_length = length;
    bt_tx_busy = true;
    start_now = true;
  }
  else
  {
    /* Do not let the periodic status request replace a one-shot Home/Auto
     * command while another Bluetooth frame is being transmitted. */
    bool pending_is_urgent = bt_tx_pending_ready &&
                             BT_IsUrgentFrame(bt_tx_pending,
                                              bt_tx_pending_length);
    bool pending_is_jog = bt_tx_pending_ready &&
                          BT_IsJogFrame(bt_tx_pending,
                                        bt_tx_pending_length);
    bool keep_pending =
        (pending_is_urgent && BT_IsStatusFrame(data, length)) ||
        (pending_is_jog && BT_IsStatusFrame(data, length));
    if (!keep_pending)
    {
      memcpy(bt_tx_pending, data, length);
      bt_tx_pending_length = length;
      bt_tx_pending_ready = true;
    }
  }
  __enable_irq();

  if (start_now &&
      (HAL_UART_Transmit_IT(&huart6, bt_tx_active,
                            bt_tx_active_length) != HAL_OK))
  {
    __disable_irq();
    bt_tx_busy = false;
    __enable_irq();
    return false;
  }
  return true;
}

void BT_UartTxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart6)
  {
    return;
  }

  if (bt_tx_pending_ready)
  {
    memcpy(bt_tx_active, bt_tx_pending, bt_tx_pending_length);
    bt_tx_active_length = bt_tx_pending_length;
    bt_tx_pending_ready = false;
    if (HAL_UART_Transmit_IT(&huart6, bt_tx_active,
                             bt_tx_active_length) == HAL_OK)
    {
      return;
    }
  }
  bt_tx_busy = false;
}

void BT_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart6)
  {
    return;
  }

  uint8_t next = (uint8_t)((bt_rx_head + 1U) % BT_RX_BUFFER_SIZE);
  if (next != bt_rx_tail)
  {
    bt_rx_buffer[bt_rx_head] = bt_rx_byte;
    bt_rx_head = next;
  }
  else
  {
    ++bt_rx_dropped_count;
  }

  (void)HAL_UART_Receive_IT(&huart6, &bt_rx_byte, 1U);
}

void BT_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart6)
  {
    bt_tx_pending_ready = false;
    bt_tx_busy = false;
    (void)HAL_UART_Receive_IT(&huart6, &bt_rx_byte, 1U);
  }
}

void Teleplot_UartTxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    teleplot_tx_busy = false;
  }
}

void Teleplot_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    teleplot_tx_busy = false;
  }
}

void BT_SendPacket(uint8_t cmd, uint8_t *data, uint8_t len)
{
  uint8_t b[32], n=0U, sum=(uint8_t)(cmd+len);
  if (len>27U || (len && data==NULL)) return;
  b[n++]=0xAAU; b[n++]=0x55U; b[n++]=cmd; b[n++]=len;
  for(uint8_t i=0U;i<len;i++){b[n++]=data[i];sum=(uint8_t)(sum+data[i]);} b[n++]=sum;
  (void)BT_QueueTransmit(b, n);
}
static void BT_SendPositionsCommand(uint8_t cmd,const uint16_t p[4]){uint8_t d[8];for(uint8_t i=0;i<4;i++){d[i*2]=(uint8_t)p[i];d[i*2+1]=(uint8_t)(p[i]>>8);}BT_SendPacket(cmd,d,8);}
static void BT_SendAllPositions(const uint16_t p[4]){BT_SendPositionsCommand(BT_CMD_SET_ALL_POS,p);}
static void BT_SendAutoPositions(const uint16_t p[4]){BT_SendPositionsCommand(BT_CMD_RUN_AUTO,p);}
static void BT_SendAutoStartPositions(const uint16_t p[4]){uint8_t d[9]={1U};for(uint8_t i=0U;i<4U;i++){d[1U+i*2U]=(uint8_t)p[i];d[2U+i*2U]=(uint8_t)(p[i]>>8U);}BT_SendPacket(BT_CMD_START_AUTO,d,9U);}
static void BT_RequestStatus(void){BT_SendPacket(BT_CMD_REQ_STATUS,NULL,0U);}
void BT_SetJogTransmitPeriod(uint32_t ms){g_bt_jog_period_ms=(ms<10U)?10U:((ms>1000U)?1000U:ms);}
void BT_SetStatusPeriod(uint32_t ms){g_bt_status_period_ms=(ms<50U)?50U:((ms>5000U)?5000U:ms);}
static void BT_ProcessReceivedByte(uint8_t x)
{
  static uint8_t st = 0U, cmd = 0U, len = 0U, i = 0U, sum = 0U, d[27];

  if (st == 0U) { if (x == 0xAAU) st = 1U; return; }
  if (st == 1U) { st = (x == 0x55U) ? 2U : 0U; return; }
  if (st == 2U) { cmd = x; sum = x; st = 3U; return; }
  if (st == 3U) { len = x; sum = (uint8_t)(sum + x); i = 0U; st = (len > sizeof(d)) ? 0U : ((len == 0U) ? 5U : 4U); return; }
  if (st == 4U) { d[i++] = x; sum = (uint8_t)(sum + x); if (i >= len) st = 5U; return; }

  if ((x == sum) && (cmd == BT_CMD_STATUS_REPLY) &&
      (len == BT_STATUS_PAYLOAD_LENGTH))
  {
    uint32_t load_sum = 0U;
    bool arrived = (g_motion_type != MOTION_NONE);
    {
      bool sharp_detected = ((d[16U] & 0x01U) != 0U);
      g_slave_status_flags = d[16U];
      g_sharp_mv = (uint16_t)d[17U] |
                   ((uint16_t)d[18U] << 8U);
      g_sharp_distance_cm = d[19U];
      g_sharp_status_updated_ms = HAL_GetTick();
      if (sharp_detected != g_sharp_detected)
      {
        g_sharp_detected = sharp_detected;
        ++g_lcd_event_sequence;
      }
    }
    g_auto_motion_released = ((d[16U] & 0x02U) != 0U) || g_auto_motion_released;

    for (uint8_t k = 0U; k < 4U; ++k)
    {
      uint16_t tolerance = (g_motion_type == MOTION_AUTO) ?
                           ((k == AX12_AUTO_GRIPPER_AXIS_INDEX) ?
                            AX12_AUTO_GRIPPER_TOLERANCE :
                            TEACHING_STEP_POSITION_TOLERANCE) :
                           HOME_POSITION_TOLERANCE;
      uint16_t error;
      g_slave_axis[k] = (uint16_t)d[k * 2U] |
                        ((uint16_t)d[k * 2U + 1U] << 8U);
      g_slave_load[k] = (uint16_t)d[8U + k * 2U] |
                        ((uint16_t)d[9U + k * 2U] << 8U);
      load_sum += g_slave_load[k];
      error = (g_slave_axis[k] > g_motion_target[k]) ?
              (g_slave_axis[k] - g_motion_target[k]) :
              (g_motion_target[k] - g_slave_axis[k]);
      if (error > tolerance) arrived = false; /* boundary value is included. */
    }

    /* Home is complete only when both sides have reached 512.  The Follower
     * can arrive first; switching to Teaching at that moment would enable
     * SET_ALL_POS and send the Leader's still-moving positions back to it. */
    if ((g_motion_type == MOTION_HOME) ||
        (g_motion_type == MOTION_AUTO_RETURN_HOME))
    {
      for (uint8_t k = 0U; k < 4U; ++k)
      {
        uint16_t master_error = (g_robot_axis[k] > g_home_positions[k]) ?
                                (g_robot_axis[k] - g_home_positions[k]) :
                                (g_home_positions[k] - g_robot_axis[k]);
        if (master_error > HOME_POSITION_TOLERANCE)
        {
          arrived = false;
          break;
        }
      }
    }

    g_motor_load = (uint16_t)(load_sum / 4U);
    ++g_slave_status_sequence;
    /* PB2 does not use a previously cached status.  The slave sends this
     * frame after it has latched its held pose; use those values to align the
     * physical master controller. */
    if (g_emergency_stop && g_estop_waiting_slave_pose)
    {
      for (uint8_t axis = 0U; axis < 4U; ++axis)
      {
        g_estop_sync_target[axis] = g_slave_axis[axis];
      }
      g_estop_waiting_slave_pose = false;
      g_estop_sync_active = true;
      g_estop_sync_stable_count = 0U;
      MasterController_RequestAction(MASTER_CTRL_ACTION_ESTOP_SYNC);
      ++g_lcd_event_sequence;
    }
    if (arrived && ((g_motion_type != MOTION_AUTO) || g_auto_motion_released))
    {
      if ((g_motion_type == MOTION_HOME) ||
          (g_motion_type == MOTION_AUTO_RETURN_HOME))
      {
        bool auto_return = (g_motion_type == MOTION_AUTO_RETURN_HOME);
        g_homing_status = 0U;
        g_home_ready = true;
        g_motion_type = MOTION_NONE;
        g_run_state = auto_return ? RUN_STATE_COMPLETED : RUN_STATE_STOPPED;
        if (!auto_return)
        {
          /* Home only establishes a safe starting pose.  Do not arm the IR
           * sensor or enter AUTO until the operator explicitly presses BTN14
           * and selects a teaching preset. */
          g_selected_preset = 0U;
          SetSystemMode(MODE_TEACHING);
        }
        /* Do not leave either controller holding torque after a home pose or
         * start the teaching stream until the operator explicitly requests it. */
        g_admin_jog_enabled = false;
        MasterController_RequestAction(MASTER_CTRL_ACTION_MANUAL);
        Robot_SetTorque(0U);
        SharpAuto_ResetCountdown();
        ++g_lcd_event_sequence;
        g_prev_mode = (SystemMode_t)0xFF;
      }
      else if (!g_auto_sequence_last_sent)
      {
        /* Advance only after the currently commanded target has physically
         * arrived.  A short, configurable settling time is then applied. */
        AutoScheduleFollowingStep();
      }
      else
      {
        /* Every Auto cycle ends at the neutral 512 pose.  Keep this separate
         * from manual Home so Auto remains armed and can re-arm after the
         * object leaves the sensor range. */
        for (uint8_t axis = 0U; axis < 4U; ++axis)
        {
          g_motion_target[axis] = g_home_positions[axis];
        }
        g_motion_type = MOTION_AUTO_RETURN_HOME;
        g_homing_status = 1U;
        MasterController_RequestAction(MASTER_CTRL_ACTION_HOME);
        BT_SendPositionsCommand(BT_CMD_HOME_POS, g_home_positions);
        g_home_retry_due_ms = HAL_GetTick() + 250U;
        ++g_lcd_event_sequence;
      }
    }
  }
  st = 0U;
}
void Robot_SetGoalPosition(uint8_t id,uint16_t p){uint8_t d[3]={id,(uint8_t)p,(uint8_t)(p>>8)};BT_SendPacket(BT_CMD_SET_GOAL_POS,d,3);}
void Robot_SetTorque(uint8_t e){uint8_t d[1]={(e!=0U)?1U:0U};BT_SendPacket(BT_CMD_SET_TORQUE,d,1);}
void Robot_SetHomePositions(const uint16_t p[4]){if(p)for(uint8_t i=0;i<4;i++)g_home_positions[i]=(p[i]<=1023U)?p[i]:1023U;}
static void MasterController_RequestAction(MasterControllerAction_t action){__disable_irq();g_master_controller_action=action;__enable_irq();}
static void SetSystemMode(SystemMode_t mode){g_system_mode=mode;g_admin_jog_enabled=(mode==MODE_ADMIN_JOG)||(mode==MODE_TEACHING);if(mode!=MODE_ADMIN_JOG){g_admin_dashboard_enabled=false;g_admin_dashboard_clear_requested=false;}++g_lcd_event_sequence;}
static uint8_t TeachingSequence_CountSaved(uint8_t preset){uint8_t count=0U;if(preset<=TEACHING_PRESET_COUNT)for(uint8_t i=0U;i<TEACHING_SEQUENCE_STEPS;i++)count+=(g_teach_memory[preset].saved_mask&(1U<<i))?1U:0U;return count;}
static uint8_t TeachingSequence_Rank(uint8_t preset,uint8_t step){uint8_t rank=0U;if(preset<=TEACHING_PRESET_COUNT)for(uint8_t i=0U;i<=step&&i<TEACHING_SEQUENCE_STEPS;i++)rank+=(g_teach_memory[preset].saved_mask&(1U<<i))?1U:0U;return rank;}
static void AutoScheduleFollowingStep(void)
{
  uint8_t next_step = (uint8_t)(g_auto_sequence_step + 1U);
  while ((next_step < TEACHING_SEQUENCE_STEPS) &&
         ((g_teach_memory[g_selected_preset].saved_mask &
           (1U << next_step)) == 0U))
  {
    ++next_step;
  }
  if (next_step < TEACHING_SEQUENCE_STEPS)
  {
    g_auto_step_due_ms = HAL_GetTick() + AUTO_SEQUENCE_STEP_DELAY_MS;
    g_auto_step_delay_active = true;
  }
  else
  {
    g_auto_sequence_last_sent = true;
    g_auto_step_delay_active = false;
  }
}
static void SharpAuto_ResetCountdown(void)
{
  bool changed = (g_sharp_detected_since_ms != 0U) ||
                 g_sharp_countdown_active ||
                 g_sharp_auto_start_pending ||
                 (g_sharp_countdown_value != 0U);
  g_sharp_countdown_active = false;
  g_sharp_detected_since_ms = 0U;
  g_sharp_auto_start_pending = false;
  g_sharp_countdown_value = 0U;
  if (changed) ++g_lcd_event_sequence;
}

static bool AutoStartPreset(uint8_t preset)
{
  /* Re-check the live sensor state immediately before sending START_AUTO.
   * The countdown is intentionally not enough by itself: a stale status
   * frame or a detection that disappeared during the display delay must
   * never start the robot. */
  if (!g_sharp_detected ||
      ((HAL_GetTick() - g_sharp_status_updated_ms) > SHARP_STATUS_TIMEOUT_MS))
  {
    SharpAuto_ResetCountdown();
    return false;
  }

  if (!g_home_ready || g_emergency_stop ||
      (preset < 1U) || (preset > TEACHING_PRESET_COUNT) ||
      (g_teach_memory[preset].saved_mask == 0U))
  {
    return false;
  }

  g_selected_preset = preset;
  SetSystemMode(MODE_AUTO);
  g_auto_sequence_step = 0U;
  while ((g_auto_sequence_step < TEACHING_SEQUENCE_STEPS) &&
         ((g_teach_memory[preset].saved_mask &
           (1U << g_auto_sequence_step)) == 0U))
  {
    ++g_auto_sequence_step;
  }
  if (g_auto_sequence_step >= TEACHING_SEQUENCE_STEPS) return false;

  for (uint8_t i = 0U; i < AX12_NUM_MOTORS; ++i)
  {
    g_motion_target[i] =
        g_teach_memory[preset].step[g_auto_sequence_step].axis[i];
  }
  g_motion_type = MOTION_AUTO;
  g_run_state = RUN_STATE_RUNNING;
  g_auto_motion_released = false;
  g_auto_sequence_last_sent = false;
  g_auto_step_delay_active = false;
  g_sharp_auto_rearm_required = true;
  BT_SendAutoStartPositions(g_motion_target);
  ++g_lcd_event_sequence;
  g_prev_mode = (SystemMode_t)0xFF;
  return true;
}
void EmergencyStop_Request(void){g_estop_request=true;g_admin_jog_enabled=false;}
static void ActivateEmergencyStop(void){if(g_emergency_stop)return;g_emergency_stop=true;g_run_state=RUN_STATE_STOPPED;g_admin_jog_enabled=false;g_motion_type=MOTION_NONE;g_auto_step_delay_active=false;g_auto_sequence_last_sent=false;g_homing_status=0U;g_estop_waiting_slave_pose=true;g_estop_sync_active=false;g_estop_jog_ready=false;g_estop_sync_stable_count=0U;MasterController_RequestAction(MASTER_CTRL_ACTION_MANUAL);/* Slave replies with its actual latched pose; do not use a cached pose. */BT_SendPacket(BT_CMD_HOLD_CURRENT,NULL,0U);BT_RequestStatus();++g_lcd_event_sequence;printf("[E-STOP] waiting for slave held-pose frame\r\n");g_prev_mode=(SystemMode_t)0xFF;}
static void CancelHomeMotion(void){if((g_motion_type==MOTION_HOME)||(g_homing_status==1U)){g_motion_type=MOTION_NONE;g_homing_status=0U;g_run_state=RUN_STATE_STOPPED;BT_SendPacket(BT_CMD_HOLD_CURRENT,NULL,0U);printf("Home motion cancelled: hold torque ON\r\n");}else if(g_homing_status==2U){g_homing_status=0U;}}
void Robot_MoveToHome(void){static const uint16_t home[4]={512U,512U,512U,512U};g_admin_jog_enabled=false;g_auto_step_delay_active=false;g_auto_sequence_last_sent=false;g_home_ready=false;g_homing_status=1U;g_run_state=RUN_STATE_STOPPED;g_motion_type=MOTION_HOME;for(uint8_t i=0;i<4;i++){g_home_positions[i]=home[i];g_motion_target[i]=home[i];}MasterController_RequestAction(MASTER_CTRL_ACTION_HOME);BT_SendPositionsCommand(BT_CMD_HOME_POS,home);g_home_retry_due_ms=HAL_GetTick()+250U;++g_lcd_event_sequence;g_prev_mode=(SystemMode_t)0xFF;}
void Process_Key_Event(uint8_t key)
{
  /* Startup safety interlock: only Home can start motion before a verified
   * 512-position return.  Keep BTN15 available only to release a real E-stop. */
  if (!g_home_ready && !(g_emergency_stop && (key == 15U)))
  {
    if ((key == 16U) && (g_homing_status != 1U))
    {
      Robot_MoveToHome();
    }
    return;
  }

  /* Preset buttons are valid only after entering Teaching (BTN13) or
   * Auto (BTN14).  In Admin JOG and during Home they must not redraw the
   * display, change a preset, or cancel the running Home command. */
  if ((key >= 1U) && (key <= TEACHING_PRESET_COUNT) &&
      ((g_homing_status != 0U) ||
       ((g_system_mode != MODE_TEACHING) && (g_system_mode != MODE_AUTO))))
  {
    return;
  }

  /* BTN16 is the only key that may retain/restart a home sequence. */
  if (key != 16U) CancelHomeMotion();

  if (key == 15U)
  {
    /* BTN15 is the only command accepted during E-STOP: release and JOG. */
    if (g_emergency_stop && !g_estop_jog_ready)
    {
      ++g_lcd_event_sequence;
      return;
    }
    /* First BTN15 enters normal Admin JOG.  A second press, while already
     * jogging, opens the dashboard without restarting or interrupting JOG. */
    if ((g_system_mode == MODE_ADMIN_JOG) && !g_emergency_stop)
    {
      g_admin_dashboard_enabled = true;
      g_admin_dashboard_clear_requested = true;
      ++g_lcd_event_sequence;
      g_prev_mode = (SystemMode_t)0xFF;
      return;
    }
    g_emergency_stop = false;
    g_estop_waiting_slave_pose = false;
    g_estop_sync_active = false;
    g_admin_dashboard_enabled = false;
    SetSystemMode(MODE_ADMIN_JOG);
    g_run_state = RUN_STATE_STOPPED;
    g_motion_type = MOTION_NONE;
    g_auto_step_delay_active = false;
    g_auto_sequence_last_sent = false;
    g_teach_save_status = 0U;
    g_teach_capture_step = 0U;
    MasterController_RequestAction(MASTER_CTRL_ACTION_MANUAL);
    /* Do not re-enable the previous AUTO goal.  The slave first captures its
     * present positions as hold goals, then enables torque for safe JOG. */
    BT_SendPacket(BT_CMD_HOLD_CURRENT, NULL, 0U);
    printf("ADMIN JOG enabled (BTN15)\r\n");
    g_prev_mode = (SystemMode_t)0xFF;
    return;
  }

  if (g_emergency_stop) return;

  /* Any key other than BTN15 leaves administrator JOG and stops its stream. */
  g_admin_jog_enabled = false;

  if (key == 13U)
  {
    /* Never let a stale homing screen override the teaching screen. */
    g_homing_status = 0U;
    SetSystemMode(MODE_TEACHING);
    g_run_state = RUN_STATE_STOPPED;
    g_motion_type = MOTION_NONE;
    g_auto_step_delay_active = false;
    g_auto_sequence_last_sent = false;
    /* Teaching records the live master pose, so it must stream just like
     * Admin JOG while the operator moves the controller. */
    g_selected_preset = 0U;
    g_teach_save_status = 0U;
    g_teach_delete_status = 0U;
    MasterController_RequestAction(MASTER_CTRL_ACTION_MANUAL);
    Robot_SetTorque(1U);
    g_prev_mode = (SystemMode_t)0xFF;
    return;
  }

  if (key == 14U)
  {
    g_admin_jog_enabled = false;
    g_teach_save_status = 0U;
    /* BTN14 enters AUTO sensor mode. The selected teaching preset is started
     * by the IR sensor, not by a second BTN14 press. */
    if (g_system_mode != MODE_AUTO)
    {
      SetSystemMode(MODE_AUTO);
      g_run_state = RUN_STATE_STOPPED;
      g_motion_type = MOTION_NONE;
      g_auto_step_delay_active = false;
      g_auto_sequence_last_sent = false;
      g_auto_motion_released = false;
    }
    g_prev_mode = (SystemMode_t)0xFF;
    return;
  }

  if (key == 16U)
  {
    Robot_MoveToHome();
    return;
  }

  /* In Teaching mode BTN11 deletes the complete selected preset (P01..P10)
   * and commits the empty sequence to Flash immediately. */
  if ((key == 11U) && (g_system_mode == MODE_TEACHING))
  {
    g_teach_save_status = 0U;
    if ((g_selected_preset >= 1U) &&
        (g_selected_preset <= TEACHING_PRESET_COUNT))
    {
      TeachingSequence_t *preset = &g_teach_memory[g_selected_preset];
      preset->saved_mask = 0U;
      for (uint8_t step = 0U; step < TEACHING_SEQUENCE_STEPS; ++step)
      {
        for (uint8_t axis = 0U; axis < TEACHING_AXIS_COUNT; ++axis)
        {
          preset->step[step].axis[axis] = TEACHING_EMPTY_AXIS_VALUE;
        }
      }
      g_teach_capture_step = 0U;
      g_teach_delete_status = TeachingStorage_Save(g_teach_memory) ? 1U : 2U;
      if (g_teach_delete_status == 1U) ++g_teach_save_event_sequence;
    }
    else
    {
      g_teach_delete_status = 3U;
    }
    ++g_lcd_event_sequence;
    g_prev_mode = (SystemMode_t)0xFF;
    return;
  }

  if ((key == 12U) && (g_system_mode == MODE_TEACHING))
  {
    if ((g_selected_preset == 0U) ||
        (g_selected_preset > TEACHING_PRESET_COUNT))
    {
      g_teach_save_status = 4U;
      g_teach_delete_status = 0U;
      ++g_lcd_event_sequence;
      g_prev_mode = (SystemMode_t)0xFF;
      return;
    }
    TeachingPoint_t *slot=&g_teach_memory[g_selected_preset].step[g_teach_capture_step];
    bool changed=false;
    for(uint8_t i=0U;i<4U;i++){uint16_t diff=(slot->axis[i]>g_robot_axis[i])?(slot->axis[i]-g_robot_axis[i]):(g_robot_axis[i]-slot->axis[i]);changed=changed||(diff>TEACHING_SAVE_DEADBAND);}
    if(changed || ((g_teach_memory[g_selected_preset].saved_mask & (1U<<g_teach_capture_step))==0U)){
      for(uint8_t i=0U;i<4U;i++)slot->axis[i]=g_robot_axis[i];
      g_teach_memory[g_selected_preset].saved_mask|=(uint32_t)(1U<<g_teach_capture_step);
      g_teach_last_saved_step=g_teach_capture_step;
      g_teach_save_status=TeachingStorage_Save(g_teach_memory)?1U:2U;
      if(g_teach_save_status==1U){++g_teach_save_event_sequence;g_teach_capture_step=(uint8_t)((g_teach_capture_step+1U)%TEACHING_SEQUENCE_STEPS);}
    }else g_teach_save_status=3U;
    g_teach_delete_status=0U;
    g_admin_jog_enabled=true;
    ++g_lcd_event_sequence;
    g_prev_mode=(SystemMode_t)0xFF;
    return;
  }

  if ((key >= 1U) && (key <= TEACHING_PRESET_COUNT))
  {
    g_selected_preset = key;
    if (g_system_mode == MODE_TEACHING)
    {
      g_teach_capture_step = 0U;
      g_teach_save_status = 0U;
      g_teach_delete_status = 0U;
      g_admin_jog_enabled = true;
      ++g_lcd_event_sequence;
    }
    else if (g_system_mode == MODE_AUTO)
    {
      g_run_state = RUN_STATE_STOPPED;
      g_motion_type = MOTION_NONE;
      g_auto_step_delay_active = false;
      g_auto_sequence_last_sent = false;
      SharpAuto_ResetCountdown();
    }
    g_prev_mode = (SystemMode_t)0xFF;
  }
}
uint8_t Keypad_Scan(void)
{
  for (int r = 0; r < 4; r++) {
    for (int i = 0; i < 4; i++) {
      HAL_GPIO_WritePin(ROW_PORTS[i], ROW_PINS[i], GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(ROW_PORTS[r], ROW_PINS[r], GPIO_PIN_SET);
    
    // 최적화: 기존 복잡한 소프트웨어 루프 딜레이를 초단축하여 반응 속도 대폭 개선
    for(volatile int d=0; d<15; d++);

    for (int c = 0; c < 4; c++) {
      if (HAL_GPIO_ReadPin(COL_PORTS[c], COL_PINS[c]) == GPIO_PIN_SET) {
        return KEY_MAP[r][c];
      }
    }
  }
  return 0;
}

static void LCD_DrawDashboardGrid(void)
{
  for (uint16_t y = 0U; y < ILI9341_HEIGHT; y += 8U)
  {
    ILI9341_DrawPixelScaled(158U, y, ILI9341_BLUE, 2U);
  }
  for (uint16_t x = 0U; x < ILI9341_WIDTH; x += 8U)
  {
    ILI9341_DrawPixelScaled(x, 118U, ILI9341_BLUE, 2U);
  }
}

static void LCD_DrawDashboardArc(uint16_t center_x, uint16_t center_y,
                                 uint8_t active_dots, uint16_t active_color)
{
  static const int8_t gauge_x[13] =
      {-48, -46, -40, -32, -22, -11, 0, 11, 22, 32, 40, 46, 48};
  static const int8_t gauge_y[13] =
      {0, -17, -30, -40, -47, -51, -53, -51, -47, -40, -30, -17, 0};

  for (uint8_t dot = 0U; dot < 13U; ++dot)
  {
    ILI9341_DrawPixelScaled((uint16_t)((int16_t)center_x + gauge_x[dot]),
                            (uint16_t)((int16_t)center_y + gauge_y[dot]),
                            (dot < active_dots) ? active_color : ILI9341_BLACK,
                            4U);
  }
}

static void LCD_DrawDashboardNeedle(uint16_t center_x, uint16_t center_y,
                                    uint8_t dot_index, uint16_t color)
{
  static const int8_t gauge_x[13] =
      {-48, -46, -40, -32, -22, -11, 0, 11, 22, 32, 40, 46, 48};
  static const int8_t gauge_y[13] =
      {0, -17, -30, -40, -47, -51, -53, -51, -47, -40, -30, -17, 0};
  int16_t x0 = (int16_t)center_x;
  int16_t y0 = (int16_t)center_y;
  int16_t x1;
  int16_t y1;
  int16_t dx;
  int16_t dy;
  int16_t sx;
  int16_t sy;
  int16_t error;

  if (dot_index >= 13U) dot_index = 12U;
  x1 = (int16_t)center_x + gauge_x[dot_index];
  y1 = (int16_t)center_y + gauge_y[dot_index];
  dx = (x1 >= x0) ? (x1 - x0) : (x0 - x1);
  dy = (y1 >= y0) ? (y1 - y0) : (y0 - y1);
  sx = (x0 < x1) ? 1 : -1;
  sy = (y0 < y1) ? 1 : -1;
  error = (int16_t)(dx - dy);

  for (;;)
  {
    ILI9341_DrawPixelScaled((uint16_t)x0, (uint16_t)y0, color, 2U);
    if ((x0 == x1) && (y0 == y1)) break;
    int16_t twice_error = (int16_t)(2 * error);
    if (twice_error > -dy) { error = (int16_t)(error - dy); x0 += sx; }
    if (twice_error < dx)  { error = (int16_t)(error + dx); y0 += sy; }
  }
}

static void LCD_GetMasterAxisLimits(uint8_t axis, uint16_t *min_position,
                                    uint16_t *max_position)
{
  uint8_t motor_id = AX12_GetMotorId(axis);
  uint16_t min_value = AX12_MASTER_4_MIN_POSITION;
  uint16_t max_value = AX12_MASTER_4_MAX_POSITION;

  if (motor_id == AX12_MASTER_1_ID)
  {
    min_value = AX12_MASTER_1_MIN_POSITION;
    max_value = AX12_MASTER_1_MAX_POSITION;
  }
  else if (motor_id == AX12_MASTER_2_ID)
  {
    min_value = AX12_MASTER_2_MIN_POSITION;
    max_value = AX12_MASTER_2_MAX_POSITION;
  }
  else if (motor_id == AX12_MASTER_3_ID)
  {
    min_value = AX12_MASTER_3_MIN_POSITION;
    max_value = AX12_MASTER_3_MAX_POSITION;
  }

  if (min_position != NULL) *min_position = min_value;
  if (max_position != NULL) *max_position = max_value;
}

static bool LCD_IsMasterAxisAtLimit(uint8_t axis, uint16_t position)
{
  uint16_t min_position;
  uint16_t max_position;

  LCD_GetMasterAxisLimits(axis, &min_position, &max_position);
  return (position <= min_position) || (position >= max_position);
}

void Update_LCD1_Clean(void)
{
  bool any_change = false;
  static bool dashboard_drawn = false;
  static bool estop_screen_drawn = false;
  static bool previous_estop_sync = false;
  static uint8_t previous_needle_dot[AX12_NUM_MOTORS] =
      {0xFFU, 0xFFU, 0xFFU, 0xFFU};
  char buf[20];

  if (g_emergency_stop)
  {
    if (estop_screen_drawn && (previous_estop_sync == g_estop_sync_active)) return;
    if (osMutexWait(lcdSpiMutexHandle, 20U) != osOK) return;
    LCD2_CS_HIGH();
    LCD1_CS_LOW();
    ILI9341_FillScreen(ILI9341_RED);
    LCD_PutString(28U, 72U, "EMERGENCY STOP", ILI9341_WHITE, ILI9341_RED, 2U);
    /* Leave two blank lines beneath the title, then show the sole action. */
    LCD_PutString(20U, 168U, "BTN15 : JOG ENABLE", ILI9341_YELLOW, ILI9341_RED, 2U);
    LCD1_CS_HIGH();
    osMutexRelease(lcdSpiMutexHandle);
    estop_screen_drawn = true;
    previous_estop_sync = g_estop_sync_active;
    dashboard_drawn = false;
    for (uint8_t axis = 0U; axis < AX12_NUM_MOTORS; ++axis)
      previous_needle_dot[axis] = 0xFFU;
    return;
  }

  if (estop_screen_drawn)
  {
    if (osMutexWait(lcdSpiMutexHandle, 20U) != osOK) return;
    LCD2_CS_HIGH();
    LCD1_CS_LOW();
    ILI9341_FillScreen(ILI9341_BLACK);
    LCD1_CS_HIGH();
    osMutexRelease(lcdSpiMutexHandle);
    estop_screen_drawn = false;
    previous_estop_sync = false;
    dashboard_drawn = false;
    for (uint8_t axis = 0U; axis < AX12_NUM_MOTORS; ++axis)
      previous_needle_dot[axis] = 0xFFU;
  }

  for (uint8_t i = 0U; i < 4U; ++i)
  {
    if (g_robot_axis[i] != g_prev_axis[i])
    {
      any_change = true;
    }
  }
  if (!any_change && dashboard_drawn) return;

  if (osMutexWait(lcdSpiMutexHandle, 5U) != osOK) return;
  LCD2_CS_HIGH();
  LCD1_CS_LOW();

  if (!dashboard_drawn)
  {
    /* A dashboard redraw begins from a clean screen, so mode/error screens
     * can never leave larger glyphs behind it. */
    ILI9341_FillScreen(ILI9341_BLACK);
  }
  LCD_DrawDashboardGrid();

  for (uint8_t i = 0U; i < 4U; ++i)
  {
    uint16_t center_x = ((i & 1U) == 0U) ? 80U : 240U;
    uint16_t center_y = (i < 2U) ? 60U : 180U;
    uint16_t min_position;
    uint16_t max_position;
    uint8_t position_dots;
    LCD_GetMasterAxisLimits(i, &min_position, &max_position);
    if (g_robot_axis[i] <= min_position)
      position_dots = 0U;
    else if (g_robot_axis[i] >= max_position)
      position_dots = 13U;
    else
      position_dots = (uint8_t)(((uint32_t)(g_robot_axis[i] - min_position) * 13U) /
                                (max_position - min_position));
    uint16_t position_color = LCD_IsMasterAxisAtLimit(i, g_robot_axis[i]) ?
                              ILI9341_RED : ILI9341_GREEN;
    uint16_t label_x = ((i & 1U) == 0U) ? 8U : 168U;
    uint16_t label_y = (i < 2U) ? 6U : 126U;

    if (previous_needle_dot[i] != 0xFFU)
      LCD_DrawDashboardNeedle(center_x, center_y, previous_needle_dot[i], ILI9341_BLACK);

    snprintf(buf, sizeof(buf), "A%u", (unsigned)(i + 1U));
    LCD_PutString(label_x, label_y,
                  buf, ILI9341_CYAN, ILI9341_BLACK, 1U);
    snprintf(buf, sizeof(buf), "%04u", (unsigned)g_robot_axis[i]);
    LCD_PutString((uint16_t)(center_x - 32U), (uint16_t)(center_y + 4U),
                  buf, position_color, ILI9341_BLACK, 2U);
    snprintf(buf, sizeof(buf), "%03u-%04u", (unsigned)min_position,
             (unsigned)max_position);
    LCD_PutString((uint16_t)(center_x - 32U), (uint16_t)(center_y + 40U),
                  buf, ILI9341_WHITE, ILI9341_BLACK, 1U);
    LCD_DrawDashboardArc(center_x, center_y, position_dots, ILI9341_GREEN);
    LCD_DrawDashboardNeedle(center_x, center_y, position_dots, ILI9341_RED);
    previous_needle_dot[i] = position_dots;
    g_prev_axis[i] = g_robot_axis[i];
  }

  dashboard_drawn = true;

  LCD1_CS_HIGH();
  osMutexRelease(lcdSpiMutexHandle);
}

void Update_LCD2_Clean(void)
{
  static uint8_t prev_preset = 0xFF;
  static uint8_t prev_homing_status = 0xFF;
  static uint16_t prev_master_voltage_mv[AX12_NUM_MOTORS] = {
      0xFFFFU, 0xFFFFU, 0xFFFFU, 0xFFFFU};
  static uint8_t prev_master_temperature_c[AX12_NUM_MOTORS] = {
      0xFFU, 0xFFU, 0xFFU, 0xFFU};
  static uint8_t previous_voltage_needle_dot[AX12_NUM_MOTORS] = {
      0xFFU, 0xFFU, 0xFFU, 0xFFU};
  static bool admin_screen_drawn = false;
  static uint32_t prev_slave_status_sequence = 0xFFFFFFFFUL;
  static uint32_t prev_teach_save_event_sequence = 0xFFFFFFFFUL;
  static uint32_t prev_lcd_event_sequence = 0xFFFFFFFFUL;
  static bool prev_home_ready = false;
  static uint8_t prev_auto_display_step = 0xFFU;
  static bool prev_sharp_detected = false;
  static bool prev_sharp_auto_start_pending = false;
  static uint8_t prev_sharp_countdown_value = 0xFFU;
  static uint32_t last_sensor_screen_refresh_ms = 0U;
  uint32_t lcd_now_ms = HAL_GetTick();
  bool preset_changed = (g_selected_preset != prev_preset);
  bool homing_changed = (g_homing_status != prev_homing_status);
  bool sensor_display_changed =
      (g_sharp_detected != prev_sharp_detected) ||
      (g_sharp_auto_start_pending != prev_sharp_auto_start_pending) ||
      (g_sharp_countdown_value != prev_sharp_countdown_value);
  bool sensor_refresh_due =
      (g_system_mode == MODE_AUTO) &&
      (g_run_state == RUN_STATE_STOPPED) &&
      ((lcd_now_ms - last_sensor_screen_refresh_ms) >= 200U);
  bool admin_monitor_changed = false;

  if (g_system_mode != MODE_ADMIN_JOG)
  {
    admin_screen_drawn = false;
    for (uint8_t axis = 0U; axis < AX12_NUM_MOTORS; ++axis)
      previous_voltage_needle_dot[axis] = 0xFFU;
  }

  if ((g_system_mode == MODE_ADMIN_JOG) && g_admin_dashboard_enabled)
  {
    for (uint8_t axis = 0U; axis < AX12_NUM_MOTORS; ++axis)
    {
      if ((g_master_voltage_mv[axis] != prev_master_voltage_mv[axis]) ||
          (g_master_temperature_c[axis] != prev_master_temperature_c[axis]))
      {
        admin_monitor_changed = true;
        break;
      }
    }
  }

  if (g_system_mode == g_prev_mode && 
      g_run_state == g_prev_run_state && 
      g_emergency_stop == g_prev_estop &&
      !preset_changed && !homing_changed &&
      !sensor_display_changed && !sensor_refresh_due &&
      (g_teach_save_event_sequence == prev_teach_save_event_sequence) &&
       (g_lcd_event_sequence == prev_lcd_event_sequence) &&
       (g_home_ready == prev_home_ready) &&
       !admin_monitor_changed &&
       (((g_system_mode != MODE_ADMIN_JOG) && (g_homing_status == 0U)) ||
       (g_slave_status_sequence == prev_slave_status_sequence))) {
    return;
  }

  /* A mode transition needs a full refresh. */
  bool layout_changed = (g_system_mode != g_prev_mode) ||
                        (g_run_state != g_prev_run_state) ||
                        (g_emergency_stop != g_prev_estop) || homing_changed ||
                        (g_home_ready != prev_home_ready);
  char buf[30];
  
  /* LCD1 dashboard drawing can hold the shared SPI bus longer than 20 ms.
   * Wait long enough so a sensor-state transition cannot starve LCD2. */
  if (osMutexWait(lcdSpiMutexHandle, 200) == osOK) {
    LCD1_CS_HIGH();
    LCD2_CS_LOW();

    /* The first Admin screen is explicitly cleared.  This removes the larger
     * Home-mode glyphs even if the state changed while LCD2 was busy. */
    if (layout_changed ||
        ((g_system_mode == MODE_ADMIN_JOG) &&
         (!admin_screen_drawn || g_admin_dashboard_clear_requested)))
    {
      ILI9341_FillScreen(ILI9341_BLACK);
      g_admin_dashboard_clear_requested = false;
    }

    uint16_t y1 = LCD2_START_Y;
    uint16_t y2 = LCD2_START_Y + LCD2_LINE_HEIGHT;
    uint16_t y3 = LCD2_START_Y + (LCD2_LINE_HEIGHT * 2);
    uint16_t y4 = LCD2_START_Y + (LCD2_LINE_HEIGHT * 3);
    uint16_t y5 = LCD2_START_Y + (LCD2_LINE_HEIGHT * 4);
    uint16_t y6 = LCD2_START_Y + (LCD2_LINE_HEIGHT * 5);

    if (g_emergency_stop)
    {
      LCD_PutString(LCD2_START_X, y1, "EMERGENCY STOP", ILI9341_RED, ILI9341_BLACK, LCD2_TITLE_SCALE);
      LCD_PutString(LCD2_START_X, y2, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y3, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y4, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y5, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y6, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
    }
    else if (!g_home_ready && (g_homing_status == 0U))
    {
      /* The LCD font is ASCII-only, so use a reliable English industrial
       * label instead of Korean glyphs that would render as blanks. */
      /* 8-pixel fixed-width font: x positions are calculated for a 320-pixel
       * screen, so the boot title stays exactly centred. */
      LCD_PutString(48U, 35U, "PROJECT ARMIGO", ILI9341_CYAN, ILI9341_BLACK, 2U);
      LCD_PutString(88U, 100U, "4-AXIS", ILI9341_WHITE, ILI9341_BLACK, 3U);
      LCD_PutString(32U, 170U, "INDUSTRIAL ROBOT", ILI9341_WHITE, ILI9341_BLACK, 2U);
    }
    else if (g_homing_status != 0U)
    {
      LCD_PutString(LCD2_START_X, y1, "[HOME POSITION]     ", ILI9341_CYAN,   ILI9341_BLACK, LCD2_TITLE_SCALE);
      LCD_PutString(LCD2_START_X, y2, (g_homing_status == 1U) ? "STATUS: MOVING      " : "STATUS: COMPLETED   ", (g_homing_status == 1U) ? ILI9341_YELLOW : ILI9341_GREEN, ILI9341_BLACK, LCD2_BODY_SCALE);
      sprintf(buf, "A1:%04u A2:%04u", (unsigned)g_slave_axis[0], (unsigned)g_slave_axis[1]);
      LCD_PutString(LCD2_START_X, y3, buf, ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      sprintf(buf, "A3:%04u A4:%04u", (unsigned)g_slave_axis[2], (unsigned)g_slave_axis[3]);
      LCD_PutString(LCD2_START_X, y4, buf, ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y5, (g_homing_status == 1U) ? "ALL AXIS -> 512     " : "INPUT ENABLED       ", ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y6, "                    ", ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
    }
    else if ((g_system_mode == MODE_ADMIN_JOG) && g_admin_dashboard_enabled)
    {
      /* Four 160 x 120 dashboard cells.  Each is a dot-drawn semicircular
       * voltage gauge like a car speedometer; temperature is digital text. */
      LCD_DrawDashboardGrid();

      for (uint8_t axis = 0U; axis < AX12_NUM_MOTORS; ++axis)
      {
        uint16_t center_x = ((axis & 1U) == 0U) ? 80U : 240U;
        uint16_t center_y = (axis < 2U) ? 60U : 180U;
        uint16_t label_x = ((axis & 1U) == 0U) ? 8U : 168U;
        uint16_t label_y = (axis < 2U) ? 6U : 126U;
        uint16_t voltage_mv = g_master_voltage_mv[axis];
        uint8_t temperature_c = g_master_temperature_c[axis];
        uint8_t voltage_dots = 0U;
        uint16_t voltage_color;

        if (voltage_mv > 9000U)
        {
          voltage_dots = (uint8_t)(((voltage_mv - 9000U) * 13U) / 5000U);
          if (voltage_dots > 13U) voltage_dots = 13U;
        }

        voltage_color = (voltage_mv < 10000U) ? ILI9341_RED :
                        ((voltage_mv < 11000U) ? ILI9341_YELLOW : ILI9341_GREEN);

        if (previous_voltage_needle_dot[axis] != 0xFFU)
          LCD_DrawDashboardNeedle(center_x, center_y,
                                  previous_voltage_needle_dot[axis], ILI9341_BLACK);

        snprintf(buf, sizeof(buf), "A%u", (unsigned)(axis + 1U));
        LCD_PutString(label_x, label_y, buf,
                      ILI9341_CYAN, ILI9341_BLACK, 1U);
        snprintf(buf, sizeof(buf), "%2u.%uV",
                 (unsigned)(voltage_mv / 1000U),
                 (unsigned)((voltage_mv % 1000U) / 100U));
        LCD_PutString((uint16_t)(center_x - 40U), (uint16_t)(center_y - 4U), buf,
                      ILI9341_WHITE, ILI9341_BLACK, 2U);
        snprintf(buf, sizeof(buf), "%02uC", (unsigned)temperature_c);
        LCD_PutString((uint16_t)(center_x - 24U),
                      (uint16_t)(center_y + 28U), buf,
                      (temperature_c >= 65U) ? ILI9341_RED :
                      ((temperature_c >= 55U) ? ILI9341_YELLOW : ILI9341_GREEN),
                      ILI9341_BLACK, 2U);
        LCD_DrawDashboardArc(center_x, center_y, voltage_dots, voltage_color);
        LCD_DrawDashboardNeedle(center_x, center_y, voltage_dots, ILI9341_RED);
        previous_voltage_needle_dot[axis] = voltage_dots;
      }
      admin_screen_drawn = true;
    }
    else if (g_system_mode == MODE_ADMIN_JOG)
    {
      LCD_PutString(LCD2_START_X, y1, "[ADMIN JOG MODE]", ILI9341_CYAN,
                    ILI9341_BLACK, LCD2_TITLE_SCALE);
      LCD_PutString(LCD2_START_X, y2, "JOG RUNNING", ILI9341_GREEN,
                    ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y3, "BTN15: DASHBOARD", ILI9341_YELLOW,
                    ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y4, "                    ", ILI9341_BLACK,
                    ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y5, "                    ", ILI9341_BLACK,
                    ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y6, "                    ", ILI9341_BLACK,
                    ILI9341_BLACK, LCD2_BODY_SCALE);
      admin_screen_drawn = true;
    }
    else if (g_system_mode == MODE_TEACHING)
    {
      LCD_PutString(LCD2_START_X, y1, "[TEACHING MODE]     ", ILI9341_MAGENTA, ILI9341_BLACK, LCD2_TITLE_SCALE);
      
      if (g_teach_delete_status == 1U) {
        sprintf(buf, "PRESET %02u DELETED", (unsigned)g_selected_preset);
        LCD_PutString(LCD2_START_X, y2, buf, ILI9341_GREEN, ILI9341_BLACK, LCD2_BODY_SCALE);
        LCD_PutString(LCD2_START_X, y3, "FLASH UPDATED       ", ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      } else if (g_teach_delete_status == 2U) {
        LCD_PutString(LCD2_START_X, y2, "DELETE FLASH ERROR! ", ILI9341_RED, ILI9341_BLACK, LCD2_BODY_SCALE);
      } else if (g_teach_delete_status == 3U) {
        LCD_PutString(LCD2_START_X, y2, "SELECT PRESET 1~10 ", ILI9341_YELLOW, ILI9341_BLACK, LCD2_BODY_SCALE);
        LCD_PutString(LCD2_START_X, y3, "THEN PRESS BTN11    ", ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      } else if (g_teach_save_status == 1U) {
        sprintf(buf, "P%02u STEP%u SAVED", (unsigned)g_selected_preset, (unsigned)(g_teach_last_saved_step+1U));
        LCD_PutString(LCD2_START_X, y2, buf, ILI9341_GREEN, ILI9341_BLACK, LCD2_BODY_SCALE);
      } else if (g_teach_save_status == 2U) {
        LCD_PutString(LCD2_START_X, y2, "FLASH SAVE ERROR!   ", ILI9341_RED, ILI9341_BLACK, LCD2_BODY_SCALE);
      } else if (g_teach_save_status == 3U) {
        LCD_PutString(LCD2_START_X, y2, "STEP UNCHANGED      ", ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      } else if (g_teach_save_status == 4U) {
        LCD_PutString(LCD2_START_X, y2, "SELECT PRESET FIRST ", ILI9341_YELLOW, ILI9341_BLACK, LCD2_BODY_SCALE);
      } else {
        if (g_selected_preset >= 1U)
        {
          sprintf(buf, "PRESET %02u SELECTED", (unsigned)g_selected_preset);
          LCD_PutString(LCD2_START_X, y2, buf, ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
        }
        else
        {
          LCD_PutString(LCD2_START_X, y2, "SELECT PRESET 1~10 ", ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
        }
      }
      
      if ((g_teach_delete_status != 1U) && (g_teach_delete_status != 3U))
      {
        sprintf(buf, "STEPS: %u", (unsigned)TeachingSequence_CountSaved(g_selected_preset));
        LCD_PutString(LCD2_START_X, y3, buf, ILI9341_CYAN, ILI9341_BLACK, LCD2_BODY_SCALE);
      }
      LCD_PutString(LCD2_START_X, y4, "1-10: SELECT PRESET ", ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y5, "12: SAVE STEP       ", ILI9341_GREEN, ILI9341_BLACK, LCD2_BODY_SCALE);
      LCD_PutString(LCD2_START_X, y6, "11: DELETE 14:AUTO  ", ILI9341_YELLOW, ILI9341_BLACK, LCD2_BODY_SCALE);
    }
    else if (g_system_mode == MODE_AUTO)
    {
      uint16_t auto_y2 = (uint16_t)(y2 + 10U);
      uint16_t auto_y3 = (uint16_t)(y3 + 20U);
      uint16_t auto_y4 = (uint16_t)(y4 + 30U);
      uint16_t auto_y5 = (uint16_t)(y5 + 40U);
      bool auto_step_changed = (g_auto_sequence_step != prev_auto_display_step);

      if (g_run_state == RUN_STATE_STOPPED)
      {
        LCD_PutString(LCD2_START_X, y1, "[AUTO SENSOR MODE]  ",
                      ILI9341_CYAN, ILI9341_BLACK, LCD2_TITLE_SCALE);
        if (g_selected_preset == 0U)
        {
          LCD_PutString(LCD2_START_X, auto_y2, "PRESET: SELECT 1-10 ",
                        ILI9341_YELLOW, ILI9341_BLACK, LCD2_BODY_SCALE);
          LCD_PutString(LCD2_START_X, auto_y3, "NO PRESET SELECTED  ",
                        ILI9341_RED, ILI9341_BLACK, LCD2_BODY_SCALE);
          LCD_PutString(LCD2_START_X, auto_y4, "PRESS PRESET BUTTON ",
                        ILI9341_YELLOW, ILI9341_BLACK, LCD2_BODY_SCALE);
        }
        else if (g_teach_memory[g_selected_preset].saved_mask == 0U)
        {
          sprintf(buf, "PRESET %02u EMPTY     ",
                  (unsigned)g_selected_preset);
          LCD_PutString(LCD2_START_X, auto_y2, buf,
                        ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
          LCD_PutString(LCD2_START_X, auto_y3, "TEACH PRESET FIRST  ",
                        ILI9341_RED, ILI9341_BLACK, LCD2_BODY_SCALE);
          LCD_PutString(LCD2_START_X, auto_y4, "SELECT ANOTHER      ",
                        ILI9341_YELLOW, ILI9341_BLACK, LCD2_BODY_SCALE);
        }
        else if (!g_sharp_detected)
        {
          sprintf(buf, "PRESET: %02u        ",
                  (unsigned)g_selected_preset);
          LCD_PutString(LCD2_START_X, auto_y2, buf,
                        ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
          LCD_PutString(LCD2_START_X, auto_y3, "WAITING             ",
                        ILI9341_YELLOW, ILI9341_BLACK, LCD2_BODY_SCALE);
          LCD_PutString(LCD2_START_X, auto_y4, "OBJECT 10-17 CM     ",
                        ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
        }
        else
        {
          sprintf(buf, "PRESET: %02u        ",
                  (unsigned)g_selected_preset);
          LCD_PutString(LCD2_START_X, auto_y2, buf,
                        ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
          LCD_PutString(LCD2_START_X, auto_y3, "DETECTED            ",
                        ILI9341_GREEN, ILI9341_BLACK, LCD2_BODY_SCALE);
          if (g_sharp_auto_start_pending)
          {
            LCD_PutString(LCD2_START_X, auto_y4, "START               ",
                          ILI9341_GREEN, ILI9341_BLACK, 3U);
          }
          else
          {
            snprintf(buf, sizeof(buf), "%u                   ",
                     (unsigned)g_sharp_countdown_value);
            LCD_PutString(LCD2_START_X, auto_y4, buf,
                          ILI9341_YELLOW, ILI9341_BLACK, 3U);
          }
        }
        LCD_PutString(LCD2_START_X, auto_y5, "                    ",
                      ILI9341_BLACK, ILI9341_BLACK, LCD2_BODY_SCALE);
      }
      /* On an AUTO step change, redraw only the STEP line.  The title, preset
       * and command hints stay untouched, avoiding a slow full-screen clear. */
      else if (layout_changed)
      {
        LCD_PutString(LCD2_START_X, y1, "[AUTO MOVE MODE]    ", ILI9341_YELLOW, ILI9341_BLACK, LCD2_TITLE_SCALE);
      }

      if ((g_run_state != RUN_STATE_STOPPED) &&
          (layout_changed || preset_changed))
      {
        sprintf(buf, "PRESET: %02u        ", (unsigned)g_selected_preset);
        LCD_PutString(LCD2_START_X, auto_y2, buf, ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      }
      if ((g_run_state != RUN_STATE_STOPPED) &&
          (layout_changed || preset_changed || auto_step_changed))
      {
        sprintf(buf, "STEP: %u/%u          ", (unsigned)TeachingSequence_Rank(g_selected_preset,g_auto_sequence_step), (unsigned)TeachingSequence_CountSaved(g_selected_preset));
        LCD_PutString(LCD2_START_X, auto_y3, buf, ILI9341_WHITE, ILI9341_BLACK, LCD2_BODY_SCALE);
      }
      if ((g_run_state != RUN_STATE_STOPPED) && layout_changed)
      {
        LCD_PutString(LCD2_START_X, auto_y4, "BTN14               ", ILI9341_YELLOW, ILI9341_BLACK, LCD2_BODY_SCALE);
        LCD_PutString(LCD2_START_X, auto_y5, "ENTER / START       ", ILI9341_YELLOW, ILI9341_BLACK, LCD2_BODY_SCALE);
      }
    }

    LCD2_CS_HIGH();
    osMutexRelease(lcdSpiMutexHandle);

    /* Commit display state only after a complete LCD transaction. */
    g_prev_mode = g_system_mode;
    g_prev_run_state = g_run_state;
    g_prev_estop = g_emergency_stop;
    prev_preset = g_selected_preset;
    prev_homing_status = g_homing_status;
    for (uint8_t axis = 0U; axis < AX12_NUM_MOTORS; ++axis)
    {
      prev_master_voltage_mv[axis] = g_master_voltage_mv[axis];
      prev_master_temperature_c[axis] = g_master_temperature_c[axis];
    }
    prev_slave_status_sequence = g_slave_status_sequence;
    prev_teach_save_event_sequence = g_teach_save_event_sequence;
    prev_lcd_event_sequence = g_lcd_event_sequence;
    prev_home_ready = g_home_ready;
    prev_auto_display_step = g_auto_sequence_step;
    prev_sharp_detected = g_sharp_detected;
    prev_sharp_auto_start_pending = g_sharp_auto_start_pending;
    prev_sharp_countdown_value = g_sharp_countdown_value;
    if ((g_system_mode == MODE_AUTO) &&
        (g_run_state == RUN_STATE_STOPPED))
    {
      last_sensor_screen_refresh_ms = lcd_now_ms;
    }
  }
}
/* USER CODE END Application */
