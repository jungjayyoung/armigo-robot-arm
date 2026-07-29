/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos.c
  * @brief   AX_SLAVE FreeRTOS tasks.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "main.h"

#include "ax12.h"
#include "gpio.h"
#include "usart.h"
#include "adc.h"
#include <stdio.h>

extern AX12_AppState ax12_app;

static osThreadId_t slaveControlTaskHandle;
static osThreadId_t telemetryTaskHandle;
static osThreadId_t sharpSensorTaskHandle;

static const osThreadAttr_t slaveControlTaskAttributes = {
  .name = "SlaveControl",
  .stack_size = 1536U,
  .priority = osPriorityAboveNormal,
};

static const osThreadAttr_t telemetryTaskAttributes = {
  .name = "Telemetry",
  .stack_size = 768U,
  .priority = osPriorityLow,
};

static const osThreadAttr_t sharpSensorTaskAttributes = {
  .name = "SharpSensor", .stack_size = 512U, .priority = osPriorityNormal,
};

static void SharpSensorTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    (void)HAL_ADC_Start_IT(&hadc1);
    osDelay(20U);
  }
}

static void SlaveControlTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    /*
     * Keep console commands and motor/link processing in one task so they
     * cannot concurrently claim the single-wire AX-12 UART.
     */
    (void)AX12_AppProcessSerial(&ax12_app, &huart2);
    AX12_AppUpdate(&ax12_app);
    osDelay(1U);
  }
}

static void TelemetryTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    if (ax12_app.ready && !ax12_app.hc05_at_mode)
    {
      for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
      {
        printf(">slave_%u_pos:%u\r\n",
               (unsigned int)(i + 1U),
               (unsigned int)ax12_app.motor_present[i]);
      }
    }
    osDelay(100U);
  }
}

void MX_FREERTOS_Init(void)
{
  slaveControlTaskHandle =
      osThreadNew(SlaveControlTask, NULL, &slaveControlTaskAttributes);
  telemetryTaskHandle =
      osThreadNew(TelemetryTask, NULL, &telemetryTaskAttributes);
  sharpSensorTaskHandle =
      osThreadNew(SharpSensorTask, NULL, &sharpSensorTaskAttributes);

  if ((slaveControlTaskHandle == NULL) || (telemetryTaskHandle == NULL) ||
      (sharpSensorTaskHandle == NULL))
  {
    Error_Handler();
  }
}
