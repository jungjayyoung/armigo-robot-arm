#include "ax12.h"

#include <stdio.h>
#include <string.h>

static AX12_AppState *s_console_app = NULL;
static UART_HandleTypeDef *s_console_uart = NULL;
static uint8_t s_console_rx_byte = 0U;

/* Fixed S-curve scheduler details.  Tune only AUTO_SCURVE_MIDDLE_STEP in
 * ax12_config.h; keeping these fixed preserves stable communication timing. */
#define AX12_AUTO_SCURVE_UPDATE_MS         20U
#define AX12_AUTO_SCURVE_MIN_DURATION_MS  300U

static bool AX12_SetAllTorque(AX12_AppState *app, bool enabled);
static bool AX12_SetAllMovingSpeed(AX12_AppState *app, uint16_t speed);
static bool AX12_SelectMotionProfile(AX12_AppState *app, uint16_t speed,
                                     uint16_t goal_max_step);

static bool AX12_StartLinkReceive(AX12_AppState *app)
{
  if ((app == NULL) || (app->link_uart == NULL))
  {
    return false;
  }

  return (HAL_UART_Receive_IT(app->link_uart, &app->link_rx_byte, 1U) ==
          HAL_OK);
}

static bool AX12_ReconfigureLinkUart(AX12_AppState *app, uint32_t baudrate)
{
  if ((app == NULL) || (app->link_uart == NULL) ||
      (app->link_uart->Instance != USART6))
  {
    return false;
  }

  (void)HAL_UART_AbortReceive(app->link_uart);
  if (HAL_UART_DeInit(app->link_uart) != HAL_OK)
  {
    return false;
  }

  app->link_uart->Init.BaudRate = baudrate;
  if (HAL_UART_Init(app->link_uart) != HAL_OK)
  {
    return false;
  }

  __HAL_UART_CLEAR_OREFLAG(app->link_uart);
  return true;
}

static void AX12_QueueHc05AtByte(AX12_AppState *app, uint8_t byte)
{
  uint16_t next;

  if (app == NULL)
  {
    return;
  }

  next = (uint16_t)((app->hc05_at_rx_head + 1U) %
                    HC05_AT_RX_BUFFER_SIZE);
  if (next == app->hc05_at_rx_tail)
  {
    ++app->hc05_at_rx_dropped;
    return;
  }

  app->hc05_at_rx_buffer[app->hc05_at_rx_head] = byte;
  app->hc05_at_rx_head = next;
}

static void AX12_FlushHc05AtResponse(AX12_AppState *app)
{
  if ((app == NULL) || (s_console_uart == NULL))
  {
    return;
  }

  while (app->hc05_at_rx_tail != app->hc05_at_rx_head)
  {
    uint8_t byte;

    __disable_irq();
    byte = app->hc05_at_rx_buffer[app->hc05_at_rx_tail];
    app->hc05_at_rx_tail =
        (uint16_t)((app->hc05_at_rx_tail + 1U) %
                   HC05_AT_RX_BUFFER_SIZE);
    __enable_irq();

    (void)HAL_UART_Transmit(s_console_uart, &byte, 1U,
                            AX12_APP_TIMEOUT_MS);
  }
}

static bool AX12_EnterHc05AtMode(AX12_AppState *app)
{
  if ((app == NULL) || (app->link_uart == NULL))
  {
    return false;
  }

  if (app->ready)
  {
    (void)AX12_SetAllTorque(app, false);
  }

  app->link_active = false;
  app->link_timed_out = false;
  app->link_command_ready = false;
  app->link_rx_state = 0U;
  app->hc05_at_rx_head = 0U;
  app->hc05_at_rx_tail = 0U;
  app->hc05_at_rx_dropped = 0U;

  if (!AX12_ReconfigureLinkUart(app, HC05_AT_BAUDRATE))
  {
    return false;
  }

  app->hc05_at_mode = true;
  if (!AX12_StartLinkReceive(app))
  {
    app->hc05_at_mode = false;
    return false;
  }

  printf("HC05 AT bridge active: USB/USART2=115200, USART6=38400\r\n");
  printf("Type AT commands, or type hc05exit to restore data mode\r\n");
  return true;
}

static bool AX12_ExitHc05AtMode(AX12_AppState *app)
{
  if ((app == NULL) || !app->hc05_at_mode)
  {
    return false;
  }

  app->hc05_at_mode = false;
  if (!AX12_ReconfigureLinkUart(app, HC05_LINK_BAUDRATE))
  {
    return false;
  }

  app->link_rx_state = 0U;
  app->link_command_ready = false;
  if (!AX12_StartLinkReceive(app))
  {
    return false;
  }

  printf("HC05 data mode restored: USART6=115200\r\n");
  printf("Power-cycle HC05 without KEY/button after AT setup\r\n");
  return true;
}

static bool AX12_SendHc05AtCommand(AX12_AppState *app, const char *line)
{
  static const uint8_t line_end[] = {'\r', '\n'};
  size_t length;

  if ((app == NULL) || (line == NULL) || !app->hc05_at_mode)
  {
    return false;
  }

  length = strlen(line);
  if ((length == 0U) || (length >= AX12_SERIAL_LINE_SIZE))
  {
    return false;
  }

  if (HAL_UART_Transmit(app->link_uart, (uint8_t *)line,
                        (uint16_t)length, 100U) != HAL_OK)
  {
    return false;
  }

  return (HAL_UART_Transmit(app->link_uart, (uint8_t *)line_end,
                            sizeof(line_end), 100U) == HAL_OK);
}

static uint16_t AX12_ClampGoal(int32_t goal)
{
  if (goal < (int32_t)AX12_GOAL_MIN)
  {
    return AX12_GOAL_MIN;
  }

  if (goal > (int32_t)AX12_GOAL_MAX)
  {
    return AX12_GOAL_MAX;
  }

  return (uint16_t)goal;
}

static uint16_t AX12_MapMasterPosition(uint8_t index, uint16_t position)
{
  int32_t mapped = position;

  if (AX12_SLAVE_MOTORS[index].reversed)
  {
    mapped = (int32_t)AX12_GOAL_MAX - mapped;
  }

  mapped += AX12_SLAVE_MOTORS[index].offset;
  return AX12_ClampGoal(mapped);
}

static bool AX12_WriteAllGoals(AX12_AppState *app,
                               const uint16_t goals[AX12_SLAVE_MOTOR_COUNT])
{
  uint8_t ids[AX12_SLAVE_MOTOR_COUNT];

  if ((app == NULL) || (goals == NULL))
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    ids[i] = AX12_SLAVE_MOTORS[i].slave_id;
  }

  return (AX12_SyncWriteGoalPositionsIT(&app->ax12, ids, goals,
                                        AX12_SLAVE_MOTOR_COUNT) == AX12_OK);
}

static bool AX12_GoalRampActive(const AX12_AppState *app)
{
  if (app == NULL)
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    int32_t difference =
        (int32_t)app->motor_target[i] - (int32_t)app->motor_goal[i];

    if ((difference > (int32_t)AX12_GOAL_DEADBAND) ||
        (difference < -(int32_t)AX12_GOAL_DEADBAND))
    {
      return true;
    }
  }
  return false;
}

static void AX12_UpdateSmoothedGoals(AX12_AppState *app, uint32_t now_ms)
{
  uint16_t next_goals[AX12_SLAVE_MOTOR_COUNT];
  bool changed = false;

  if ((app == NULL) ||
      ((now_ms - app->last_goal_update_ms) < AX12_GOAL_UPDATE_MS) ||
      AX12_IsAsyncBusBusy(&app->ax12))
  {
    return;
  }

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    int32_t current = app->motor_goal[i];
    int32_t target = app->motor_target[i];
    int32_t difference = target - current;

    if ((difference <= (int32_t)AX12_GOAL_DEADBAND) &&
        (difference >= -(int32_t)AX12_GOAL_DEADBAND))
    {
      next_goals[i] = (uint16_t)target;
    }
    else if (difference > (int32_t)app->goal_max_step)
    {
      next_goals[i] = (uint16_t)(current + app->goal_max_step);
    }
    else if (difference < -(int32_t)app->goal_max_step)
    {
      next_goals[i] = (uint16_t)(current - app->goal_max_step);
    }
    else
    {
      next_goals[i] = (uint16_t)target;
    }

    if (next_goals[i] != app->motor_goal[i])
    {
      changed = true;
    }
  }

  app->last_goal_update_ms = now_ms;
  if (changed && AX12_WriteAllGoals(app, next_goals))
  {
    for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
    {
      app->motor_goal[i] = next_goals[i];
    }
  }
}

/* Cubic smoothstep (3t² - 2t³) starts and ends at zero velocity.  The
 * intermediate synchronous goal packets produce an S-curve path without
 * changing the AX-12 Moving Speed configured for Auto mode. */
static void AX12_UpdateAutoSCurveGoals(AX12_AppState *app, uint32_t now_ms)
{
  uint16_t next_goals[AX12_SLAVE_MOTOR_COUNT];
  uint32_t elapsed_ms;
  uint32_t t;
  uint32_t curve;

  if ((app == NULL) || !app->auto_scurve_active ||
      AX12_IsAsyncBusBusy(&app->ax12) ||
      ((now_ms - app->auto_scurve_last_update_ms) < AX12_AUTO_SCURVE_UPDATE_MS))
  {
    return;
  }

  elapsed_ms = now_ms - app->auto_scurve_started_ms;
  t = (elapsed_ms >= app->auto_scurve_duration_ms) ? 1000U :
      (elapsed_ms * 1000U) / app->auto_scurve_duration_ms;
  /* curve is 0..1000, using 64-bit arithmetic to avoid overflow. */
  curve = (uint32_t)(((uint64_t)t * t * (3000U - (2U * t))) / 1000000U);

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    int32_t delta = (int32_t)app->motor_target[i] -
                    (int32_t)app->auto_scurve_start[i];
    next_goals[i] = AX12_ClampGoal((int32_t)app->auto_scurve_start[i] +
                                   (int32_t)(((int64_t)delta * curve) / 1000));
  }

  app->auto_scurve_last_update_ms = now_ms;
  if (AX12_WriteAllGoals(app, next_goals))
  {
    for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
    {
      app->motor_goal[i] = next_goals[i];
    }
    if (t >= 1000U)
    {
      app->auto_scurve_active = false;
    }
  }
}

static bool AX12_SetAllTorque(AX12_AppState *app, bool enabled)
{
  bool all_ok = true;

  if (app == NULL)
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    if (AX12_SetTorque(&app->ax12, AX12_SLAVE_MOTORS[i].slave_id,
                       enabled) == AX12_OK)
    {
      app->motor_torque_enabled[i] = enabled;
    }
    else
    {
      all_ok = false;
    }
  }

  return all_ok;
}

static bool AX12_SetAllMovingSpeed(AX12_AppState *app, uint16_t speed)
{
  if ((app == NULL) || (speed > AX12_GOAL_MAX))
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    if (AX12_SetMovingSpeed(&app->ax12, AX12_SLAVE_MOTORS[i].slave_id,
                            speed) != AX12_OK)
    {
      return false;
    }
  }
  app->motion_speed = speed;
  return true;
}

static bool AX12_SelectMotionProfile(AX12_AppState *app, uint16_t speed,
                                     uint16_t goal_max_step)
{
  if ((app == NULL) || (goal_max_step == 0U))
  {
    return false;
  }

  if ((app->motion_speed != speed) && !AX12_SetAllMovingSpeed(app, speed))
  {
    return false;
  }
  app->goal_max_step = goal_max_step;
  return true;
}

static bool AX12_StartNextPresentPositionRead(AX12_AppState *app)
{
  if (app == NULL)
  {
    return false;
  }

  return (AX12_StartPresentPositionReadIT(
              &app->ax12,
              AX12_SLAVE_MOTORS[app->telemetry_motor_index].slave_id) ==
          AX12_OK);
}

static void AX12_ResetFusionParser(AX12_AppState *app)
{
  app->link_rx_state = 0U;
  app->link_payload_index = 0U;
  app->link_payload_length = 0U;
  app->link_checksum = 0U;
}

static void AX12_QueueFusionCommand(AX12_AppState *app)
{
  /* E-STOP is an urgent command. It must replace a queued JOG frame. */
  if ((app->link_command == FUSION_CMD_SET_TORQUE) &&
      (app->link_payload_length == 1U) && (app->link_payload[0] == 0U))
  {
    app->pending_command = app->link_command;
    app->pending_payload_length = app->link_payload_length;
    app->pending_payload[0] = 0U;
    app->link_command_ready = true;
    ++app->valid_frame_count;
    return;
  }

  if (app->link_command_ready)
  {
    ++app->dropped_frame_count;

    /* HOME/Auto are one-shot motion commands.  They must never be replaced by
     * the periodic status request that the master sends every 50 ms. */
    if ((app->link_command == FUSION_CMD_HOME_POS) ||
        (app->link_command == FUSION_CMD_START_AUTO) ||
        (app->link_command == FUSION_CMD_RUN_AUTO) ||
        (app->link_command == FUSION_CMD_HOLD_CURRENT) ||
        (app->link_command == FUSION_CMD_SET_TORQUE))
    {
      app->pending_command = app->link_command;
      app->pending_payload_length = app->link_payload_length;
      for (uint8_t i = 0U; i < app->link_payload_length; ++i)
      {
        app->pending_payload[i] = app->link_payload[i];
      }
      app->link_command_ready = true;
      ++app->valid_frame_count;
      return;
    }

    /* A live JOG frame must replace a waiting status request. */
    if ((app->link_command == FUSION_CMD_SET_ALL_POS) &&
        (app->pending_command == FUSION_CMD_REQ_STATUS))
    {
      app->pending_command = app->link_command;
      app->pending_payload_length = app->link_payload_length;
      for (uint8_t i = 0U; i < app->link_payload_length; ++i)
      {
        app->pending_payload[i] = app->link_payload[i];
      }
      app->link_command_ready = true;
      ++app->valid_frame_count;
      return;
    }

    /* Status requests must not be starved by the 10 ms JOG position stream,
     * but they do not override a one-shot motion command. */
    if (app->link_command == FUSION_CMD_REQ_STATUS)
    {
      if ((app->pending_command == FUSION_CMD_HOME_POS) ||
          (app->pending_command == FUSION_CMD_START_AUTO) ||
          (app->pending_command == FUSION_CMD_RUN_AUTO) ||
          (app->pending_command == FUSION_CMD_HOLD_CURRENT) ||
          (app->pending_command == FUSION_CMD_SET_TORQUE) ||
          (app->pending_command == FUSION_CMD_SET_ALL_POS))
      {
        return;
      }
      app->pending_command = app->link_command;
      app->pending_payload_length = app->link_payload_length;
      app->link_command_ready = true;
      ++app->valid_frame_count;
      return;
    }

    /*
     * Position streaming is latest-value control. Replacing an old SET_ALL
     * command prevents the slave from applying stale hand positions.
     */
    if ((app->pending_command != FUSION_CMD_SET_ALL_POS) ||
        (app->link_command != FUSION_CMD_SET_ALL_POS))
    {
      return;
    }
  }

  app->pending_command = app->link_command;
  app->pending_payload_length = app->link_payload_length;
  for (uint8_t i = 0U; i < app->link_payload_length; ++i)
  {
    app->pending_payload[i] = app->link_payload[i];
  }
  app->link_command_ready = true;
  ++app->valid_frame_count;
}

static void AX12_ConsumeLinkByte(AX12_AppState *app, uint8_t byte)
{
  if (app == NULL)
  {
    return;
  }

  switch (app->link_rx_state)
  {
    case 0U:
      if (byte == FUSION_FRAME_HEADER_1)
      {
        app->link_rx_state = 1U;
      }
      break;

    case 1U:
      if (byte == FUSION_FRAME_HEADER_2)
      {
        app->link_rx_state = 2U;
      }
      else if (byte != FUSION_FRAME_HEADER_1)
      {
        AX12_ResetFusionParser(app);
      }
      break;

    case 2U:
      app->link_command = byte;
      app->link_checksum = byte;
      app->link_rx_state = 3U;
      break;

    case 3U:
      if (byte > FUSION_MAX_PAYLOAD_LENGTH)
      {
        ++app->invalid_frame_count;
        AX12_ResetFusionParser(app);
        break;
      }
      app->link_payload_length = byte;
      app->link_payload_index = 0U;
      app->link_checksum = (uint8_t)(app->link_checksum + byte);
      app->link_rx_state = (byte == 0U) ? 5U : 4U;
      break;

    case 4U:
      app->link_payload[app->link_payload_index++] = byte;
      app->link_checksum = (uint8_t)(app->link_checksum + byte);
      if (app->link_payload_index >= app->link_payload_length)
      {
        app->link_rx_state = 5U;
      }
      break;

    case 5U:
      if (byte == app->link_checksum)
      {
        AX12_QueueFusionCommand(app);
      }
      else
      {
        ++app->invalid_frame_count;
      }
      AX12_ResetFusionParser(app);
      break;

    default:
      AX12_ResetFusionParser(app);
      break;
  }
}

static bool AX12_FindFusionMotorIndex(uint8_t id, uint8_t *index)
{
  if (index == NULL)
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    if ((id == AX12_SLAVE_MOTORS[i].master_id) ||
        (id == AX12_SLAVE_MOTORS[i].slave_id))
    {
      *index = i;
      return true;
    }
  }

  return false;
}

static bool AX12_EnableTorqueForMotion(AX12_AppState *app)
{
  if (app->link_active)
  {
    return true;
  }

  if (!AX12_SetAllTorque(app, true))
  {
    return false;
  }

  app->link_active = true;
  printf("Fusion link active: first motion command, slave torque on\r\n");
  return true;
}

static bool AX12_ApplyAllPositionPayload(AX12_AppState *app,
                                         const uint8_t *payload,
                                         uint8_t length)
{
  uint16_t goals[AX12_SLAVE_MOTOR_COUNT];

  if ((app == NULL) || (payload == NULL) ||
      (length != (AX12_SLAVE_MOTOR_COUNT * 2U)))
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    uint16_t position = (uint16_t)payload[i * 2U] |
                        ((uint16_t)payload[(i * 2U) + 1U] << 8U);

    if (position > AX12_GOAL_MAX)
    {
      return false;
    }
    goals[i] = AX12_MapMasterPosition(i, position);
  }

  if (!AX12_EnableTorqueForMotion(app))
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    app->motor_target[i] = goals[i];
  }
  return true;
}

static bool AX12_BeginAutoSCurve(AX12_AppState *app)
{
  uint16_t max_delta = 0U;
  uint16_t delta[AX12_SLAVE_MOTOR_COUNT];
  uint32_t duration_ms;

  if (app == NULL)
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    delta[i] = (app->motor_target[i] > app->motor_goal[i]) ?
               (app->motor_target[i] - app->motor_goal[i]) :
               (app->motor_goal[i] - app->motor_target[i]);
    app->auto_scurve_start[i] = app->motor_goal[i];
    if (delta[i] > max_delta) max_delta = delta[i];
  }

  if (max_delta == 0U)
  {
    app->auto_scurve_active = false;
    return true;
  }

  /* Keep one common AX-12 speed.  Sequential AUTO waits for confirmed
   * arrival before issuing its next S-curve target, avoiding speed mismatch
   * and jitter between axes. */
  if (!AX12_SetAllMovingSpeed(app, AX12_AUTO_MOVING_SPEED))
  {
    return false;
  }

  /* smoothstep's maximum slope is 1.5.  Derive the duration from the actual
   * start-to-target difference so this one value controls the middle speed. */
  duration_ms = (uint32_t)(((uint64_t)max_delta * 3U *
                             AX12_AUTO_SCURVE_UPDATE_MS) /
                            (2U * AX12_AUTO_SCURVE_MIDDLE_STEP));
  if (duration_ms < AX12_AUTO_SCURVE_MIN_DURATION_MS)
  {
    duration_ms = AX12_AUTO_SCURVE_MIN_DURATION_MS;
  }
  app->auto_scurve_started_ms = HAL_GetTick();
  app->auto_scurve_last_update_ms = app->auto_scurve_started_ms -
                                     AX12_AUTO_SCURVE_UPDATE_MS;
  app->auto_scurve_duration_ms = duration_ms;
  app->auto_scurve_active = true;
  return true;
}

void AX12_AppSetSharpDetected(AX12_AppState *app, bool detected)
{
  if (app != NULL)
  {
    app->sharp_detected = detected;
  }
}

static bool AX12_SendFusionPacket(AX12_AppState *app, uint8_t command,
                                  const uint8_t *payload, uint8_t length)
{
  uint8_t frame[2U + 1U + 1U + FUSION_MAX_PAYLOAD_LENGTH + 1U];
  uint8_t checksum = (uint8_t)(command + length);
  uint8_t index = 0U;

  if ((app == NULL) || (app->link_uart == NULL) ||
      (length > FUSION_MAX_PAYLOAD_LENGTH) ||
      ((length > 0U) && (payload == NULL)))
  {
    return false;
  }

  frame[index++] = FUSION_FRAME_HEADER_1;
  frame[index++] = FUSION_FRAME_HEADER_2;
  frame[index++] = command;
  frame[index++] = length;
  for (uint8_t i = 0U; i < length; ++i)
  {
    frame[index++] = payload[i];
    checksum = (uint8_t)(checksum + payload[i]);
  }
  frame[index++] = checksum;

  return (HAL_UART_Transmit(app->link_uart, frame, index,
                            AX12_APP_TIMEOUT_MS) == HAL_OK);
}

static bool AX12_SendFusionStatus(AX12_AppState *app)
{
  uint8_t payload[FUSION_STATUS_PAYLOAD_LENGTH];

  if (app == NULL)
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    payload[i * 2U] = (uint8_t)(app->motor_present[i] & 0xFFU);
    payload[(i * 2U) + 1U] =
        (uint8_t)((app->motor_present[i] >> 8U) & 0xFFU);
    payload[8U + (i * 2U)] =
        (uint8_t)(app->motor_load[i] & 0xFFU);
    payload[9U + (i * 2U)] =
        (uint8_t)((app->motor_load[i] >> 8U) & 0xFFU);
  }
  payload[16U] = (app->sharp_detected ? 0x01U : 0U) |
                 (app->auto_motion_released ? 0x02U : 0U);

  return AX12_SendFusionPacket(app, FUSION_CMD_STATUS_REPLY, payload,
                               sizeof(payload));
}

static bool AX12_ProcessFusionCommand(AX12_AppState *app, uint8_t command,
                                      const uint8_t *payload, uint8_t length)
{
  if (app == NULL)
  {
    return false;
  }

  app->last_link_rx_ms = HAL_GetTick();
  app->link_timed_out = false;

  if (command == FUSION_CMD_SET_ALL_POS)
  {
    app->auto_start_requested = false;
    app->auto_motion_released = false;
    app->auto_scurve_active = false;
    if (!AX12_SelectMotionProfile(app, AX12_TEACH_MOVING_SPEED,
                                  AX12_TEACH_GOAL_MAX_STEP))
    {
      return false;
    }
    return AX12_ApplyAllPositionPayload(app, payload, length);
  }

  if ((command == FUSION_CMD_HOME_POS) &&
      (length == (AX12_SLAVE_MOTOR_COUNT * 2U)))
  {
    app->auto_start_requested = false;
    app->auto_motion_released = false;
    app->auto_scurve_active = false;
    if (!AX12_SelectMotionProfile(app, AX12_HOME_MOVING_SPEED,
                                  AX12_HOME_GOAL_MAX_STEP))
    {
      return false;
    }
    if (!AX12_EnableTorqueForMotion(app))
    {
      return false;
    }
    for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
    {
      /* Home is a physical AX-12 centre position, independent of mapping. */
      app->motor_target[i] = AX12_DEFAULT_GOAL;
    }
    return true;
  }

  if ((command == FUSION_CMD_START_AUTO) && (length == 9U) &&
      (payload[0] == 1U))
  {
    bool applied;
    app->auto_start_requested = true;
    app->auto_motion_released = false;
    app->auto_scurve_active = false;
    if (!AX12_SelectMotionProfile(app, AX12_AUTO_MOVING_SPEED,
                                  AX12_AUTO_GOAL_MAX_STEP))
    {
      return false;
    }
    applied = AX12_ApplyAllPositionPayload(app, &payload[1], 8U);
    if (applied)
    {
      /* Prepare the same S-curve as RUN_AUTO, but AppUpdate holds it until
       * Sharp detection releases this first automatic movement. */
      applied = AX12_BeginAutoSCurve(app);
    }
    return applied;
  }

  /* BTN14 teaching sequence: use the Auto profile immediately.  Unlike
   * START_AUTO this command intentionally does not wait for Sharp detection. */
  if ((command == FUSION_CMD_RUN_AUTO) &&
      (length == (AX12_SLAVE_MOTOR_COUNT * 2U)))
  {
    bool applied;
    app->auto_start_requested = false;
    app->auto_motion_released = true;
    if (!AX12_SelectMotionProfile(app, AX12_AUTO_MOVING_SPEED,
                                  AX12_AUTO_GOAL_MAX_STEP))
    {
      return false;
    }
    applied = AX12_ApplyAllPositionPayload(app, payload, length);
    if (applied)
    {
      applied = AX12_BeginAutoSCurve(app);
    }
    return applied;
  }

  if ((command == FUSION_CMD_SET_GOAL_POS) && (length == 3U))
  {
    uint8_t index;
    uint16_t position = (uint16_t)payload[1] |
                        ((uint16_t)payload[2] << 8U);

    if (!AX12_FindFusionMotorIndex(payload[0], &index) ||
        (position > AX12_GOAL_MAX))
    {
      return false;
    }

    position = AX12_MapMasterPosition(index, position);
    app->motor_target[index] = position;
    return AX12_EnableTorqueForMotion(app);
  }

  if ((command == FUSION_CMD_SET_TORQUE) && (length == 1U))
  {
    bool enabled = (payload[0] != 0U);
    bool result = AX12_SetAllTorque(app, enabled);

    if (result)
    {
      app->link_active = enabled;
    }
    return result;
  }

  if ((command == FUSION_CMD_HOLD_CURRENT) && (length == 0U))
  {
    /* An AX-12 retains its old goal while torque is disabled.  Write the
     * measured pose first, so releasing E-stop cannot resume AUTO or HOME. */
    app->auto_start_requested = false;
    app->auto_motion_released = false;
    app->auto_scurve_active = false;
    for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
    {
      uint16_t hold = app->motor_present[i];
      app->motor_goal[i] = hold;
      app->motor_target[i] = hold;
      if (AX12_SetGoalPosition(&app->ax12, AX12_SLAVE_MOTORS[i].slave_id,
                               hold) != AX12_OK)
      {
        return false;
      }
    }
    {
      bool held = AX12_SetAllTorque(app, true);
      if (held)
      {
        /* Send the exact latched pose immediately for master-controller
         * alignment after PB2 E-stop. */
        (void)AX12_SendFusionStatus(app);
      }
      return held;
    }
  }

  if ((command == FUSION_CMD_REQ_STATUS) && (length == 0U))
  {
    return AX12_SendFusionStatus(app);
  }

  return false;
}

static void AX12_PrintStatus(const AX12_AppState *app)
{
  if (app == NULL)
  {
    return;
  }

  printf("slave status: link=%s, timeout=%s, frames=%lu, bad=%lu, dropped=%lu\r\n",
         app->link_active ? "active" : "waiting",
         app->link_timed_out ? "yes" : "no",
         (unsigned long)app->valid_frame_count,
         (unsigned long)app->invalid_frame_count,
         (unsigned long)app->dropped_frame_count);

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    printf("slave %u: master_id=%u -> id=%u, goal=%u, position=%u, torque=%s\r\n",
           (unsigned int)(i + 1U),
           (unsigned int)AX12_SLAVE_MOTORS[i].master_id,
           (unsigned int)AX12_SLAVE_MOTORS[i].slave_id,
           (unsigned int)app->motor_goal[i],
           (unsigned int)app->motor_present[i],
           app->motor_torque_enabled[i] ? "on" : "off");
  }
}

static bool AX12_ChangeUartBaud(UART_HandleTypeDef *uart,
                                uint32_t baudrate)
{
  if ((uart == NULL) || (uart->Instance != USART1))
  {
    return false;
  }

  if (HAL_UART_DeInit(uart) != HAL_OK)
  {
    return false;
  }

  uart->Init.BaudRate = baudrate;
  return (HAL_HalfDuplex_Init(uart) == HAL_OK);
}

static bool AX12_ReconfigureBusTo1Mbps(AX12_AppState *app)
{
  bool all_ok = true;

  if ((app == NULL) || (app->ax12.uart == NULL))
  {
    return false;
  }

  printf("bus1m: probing slave IDs at 115200\r\n");
  if (!AX12_ChangeUartBaud(app->ax12.uart, AX12_LEGACY_BAUDRATE))
  {
    return false;
  }

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    uint8_t id = AX12_SLAVE_MOTORS[i].slave_id;

    if (AX12_Ping(&app->ax12, id) == AX12_OK)
    {
      printf("bus1m: ID %u found, changing to 1 Mbps\r\n", id);
      (void)AX12_SetBaudRate(&app->ax12, id,
                            AX12_BAUD_VALUE_1MBPS);
      HAL_Delay(20U);
    }
    else
    {
      printf("bus1m: ID %u not found at 115200 (may already be 1 Mbps)\r\n",
             id);
    }
  }

  if (!AX12_ChangeUartBaud(app->ax12.uart, AX12_BUS_BAUDRATE))
  {
    return false;
  }
  HAL_Delay(20U);

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    uint8_t id = AX12_SLAVE_MOTORS[i].slave_id;

    if (AX12_Ping(&app->ax12, id) == AX12_OK)
    {
      printf("bus1m: ID %u OK at 1 Mbps\r\n", id);
    }
    else
    {
      printf("bus1m: ID %u FAILED at 1 Mbps\r\n", id);
      all_ok = false;
    }
  }

  printf(all_ok ? "bus1m: success, press RESET\r\n"
                : "bus1m: failed, check power/data/GND/IDs\r\n");
  return all_ok;
}

static bool AX12_HandleCommand(AX12_AppState *app, const char *line)
{
  if ((app == NULL) || (line == NULL))
  {
    return false;
  }

  if (strcmp(line, "help") == 0)
  {
    printf("cmds: status, gripper, bus1m, torque on, torque off, hc05at\r\n");
    return true;
  }

  if (strcmp(line, "status") == 0)
  {
    AX12_PrintStatus(app);
    return true;
  }

  if (strcmp(line, "gripper") == 0)
  {
    uint16_t max_torque = 0U;
    uint16_t torque_limit = 0U;
    AX12_Result max_result = AX12_ReadWord(&app->ax12, AX12_SLAVE_2_ID,
                                            AX12_ADDR_MAX_TORQUE,
                                            &max_torque);
    AX12_Result limit_result = AX12_ReadWord(&app->ax12, AX12_SLAVE_2_ID,
                                              AX12_ADDR_TORQUE_LIMIT,
                                              &torque_limit);
    if ((max_result == AX12_OK) && (limit_result == AX12_OK))
    {
      printf(">gripper_max_torque:%u\r\n", (unsigned)max_torque);
      printf(">gripper_torque_limit:%u\r\n", (unsigned)torque_limit);
      return true;
    }
    printf("GRIPPER ID2: torque read failed (%d/%d)\r\n",
           (int)max_result, (int)limit_result);
    return false;
  }

  if (strcmp(line, "bus1m") == 0)
  {
    return AX12_ReconfigureBusTo1Mbps(app);
  }

  if (strcmp(line, "torque on") == 0)
  {
    return AX12_SetAllTorque(app, true);
  }

  if (strcmp(line, "torque off") == 0)
  {
    app->link_active = false;
    return AX12_SetAllTorque(app, false);
  }

  if (strcmp(line, "hc05at") == 0)
  {
    return AX12_EnterHc05AtMode(app);
  }

  return false;
}

bool AX12_AppInit(AX12_AppState *app, UART_HandleTypeDef *ax12_uart,
                  UART_HandleTypeDef *link_uart)
{
  if ((app == NULL) || (ax12_uart == NULL) || (link_uart == NULL))
  {
    printf("AX12 slave init: invalid argument\r\n");
    return false;
  }

  memset(app, 0, sizeof(*app));
  s_console_app = app;
  AX12_Init(&app->ax12, ax12_uart, AX12_APP_TIMEOUT_MS);
  app->link_uart = link_uart;

  for (uint8_t i = 0U; i < AX12_SLAVE_MOTOR_COUNT; ++i)
  {
    uint8_t id = AX12_SLAVE_MOTORS[i].slave_id;
    uint16_t current = AX12_DEFAULT_GOAL;

    if (AX12_Ping(&app->ax12, id) != AX12_OK)
    {
      printf("AX12 slave init: ping failed (ID=%u)\r\n", id);
      return false;
    }

    if (id == AX12_SLAVE_2_ID)
    {
      uint16_t max_torque = 0U;
      uint16_t torque_limit = 0U;
      AX12_Result max_result = AX12_ReadWord(&app->ax12, id,
                                              AX12_ADDR_MAX_TORQUE,
                                              &max_torque);
      AX12_Result limit_result = AX12_ReadWord(&app->ax12, id,
                                                AX12_ADDR_TORQUE_LIMIT,
                                                &torque_limit);
      if ((max_result == AX12_OK) && (limit_result == AX12_OK))
      {
        printf("GRIPPER ID2: MaxTorque=%u TorqueLimit=%u\r\n",
               (unsigned)max_torque, (unsigned)torque_limit);
        /* Teleplot-compatible values: appear in the left telemetry filter
         * of the currently open COM8 serial view after reset. */
        printf(">gripper_max_torque:%u\r\n", (unsigned)max_torque);
        printf(">gripper_torque_limit:%u\r\n", (unsigned)torque_limit);
      }
      else
      {
        printf("GRIPPER ID2: torque read failed (%d/%d)\r\n",
               (int)max_result, (int)limit_result);
      }
    }

    if (AX12_SetTorque(&app->ax12, id, false) != AX12_OK)
    {
      printf("AX12 slave init: torque off failed (ID=%u)\r\n", id);
      return false;
    }

    if (AX12_SetMovingSpeed(&app->ax12, id,
                            AX12_TEACH_MOVING_SPEED) != AX12_OK)
    {
      printf("AX12 slave init: speed setup failed (ID=%u)\r\n", id);
      return false;
    }

    if (AX12_GetPresentPosition(&app->ax12, id, &current) != AX12_OK)
    {
      printf("AX12 slave init: position read failed (ID=%u)\r\n", id);
      return false;
    }

    app->motor_present[i] = current;
    app->motor_goal[i] = current;
    app->motor_target[i] = current;
    app->motor_torque_enabled[i] = false;
  }

  if (!AX12_WriteAllGoals(app, app->motor_goal))
  {
    printf("AX12 slave init: initial goal sync write failed\r\n");
    return false;
  }

  app->last_link_rx_ms = HAL_GetTick();
  app->last_goal_update_ms = HAL_GetTick();
  app->last_position_poll_ms = HAL_GetTick();
  app->motion_speed = AX12_TEACH_MOVING_SPEED;
  app->goal_max_step = AX12_TEACH_GOAL_MAX_STEP;
  app->ready = true;

  return true;
}

bool AX12_AppStartConsoleRx(AX12_AppState *app,
                            UART_HandleTypeDef *console_uart)
{
  if ((app == NULL) || (console_uart == NULL))
  {
    return false;
  }

  s_console_app = app;
  s_console_uart = console_uart;
  app->serial_line_len = 0U;
  app->serial_line_ready = false;
  app->serial_line[0] = '\0';

  return (HAL_UART_Receive_IT(s_console_uart, &s_console_rx_byte, 1U) ==
          HAL_OK);
}

bool AX12_AppStartLinkRx(AX12_AppState *app)
{
  if ((app == NULL) || (app->link_uart == NULL))
  {
    return false;
  }

  AX12_ResetFusionParser(app);
  app->link_command_ready = false;
  return AX12_StartLinkReceive(app);
}

void AX12_AppUartRxCpltCallback(UART_HandleTypeDef *huart)
{
  AX12_AppState *app = s_console_app;

  if ((huart == NULL) || (app == NULL))
  {
    return;
  }

  AX12_UartRxCpltCallback(&app->ax12, huart);

  if (huart == s_console_uart)
  {
    if (!app->serial_line_ready)
    {
      if ((s_console_rx_byte == '\r') || (s_console_rx_byte == '\n'))
      {
        if (app->serial_line_len > 0U)
        {
          app->serial_line[app->serial_line_len] = '\0';
          app->serial_line_ready = true;
        }
      }
      else if (app->serial_line_len <
               (sizeof(app->serial_line) - 1U))
      {
        app->serial_line[app->serial_line_len++] =
            (char)s_console_rx_byte;
      }
      else
      {
        app->serial_line_len = 0U;
      }
    }

    (void)HAL_UART_Receive_IT(s_console_uart, &s_console_rx_byte, 1U);
  }

  if (huart == app->link_uart)
  {
    if (app->hc05_at_mode)
    {
      AX12_QueueHc05AtByte(app, app->link_rx_byte);
    }
    else
    {
      AX12_ConsumeLinkByte(app, app->link_rx_byte);
    }
    (void)AX12_StartLinkReceive(app);
  }
}

void AX12_AppUartTxCpltCallback(UART_HandleTypeDef *huart)
{
  AX12_AppState *app = s_console_app;

  if (app != NULL)
  {
    AX12_UartTxCpltCallback(&app->ax12, huart);
  }
}

void AX12_AppUartErrorCallback(UART_HandleTypeDef *huart)
{
  AX12_AppState *app = s_console_app;

  if (app != NULL)
  {
    AX12_UartErrorCallback(&app->ax12, huart);
    if (huart == app->link_uart)
    {
      __HAL_UART_CLEAR_OREFLAG(huart);
      (void)AX12_StartLinkReceive(app);
    }
  }
}

bool AX12_AppProcessSerial(AX12_AppState *app,
                           UART_HandleTypeDef *console_uart)
{
  bool handled;
  char line_copy[AX12_SERIAL_LINE_SIZE];

  if ((app == NULL) || (console_uart == NULL) ||
      !app->serial_line_ready)
  {
    return false;
  }

  __disable_irq();
  strncpy(line_copy, app->serial_line, sizeof(line_copy));
  line_copy[sizeof(line_copy) - 1U] = '\0';
  app->serial_line_ready = false;
  app->serial_line_len = 0U;
  app->serial_line[0] = '\0';
  __enable_irq();

  if (app->hc05_at_mode)
  {
    if (strcmp(line_copy, "hc05exit") == 0)
    {
      handled = AX12_ExitHc05AtMode(app);
      printf(handled ? "CMD OK: hc05exit\r\n"
                     : "CMD ERR: hc05exit\r\n");
    }
    else
    {
      handled = AX12_SendHc05AtCommand(app, line_copy);
      if (!handled)
      {
        printf("HC05 AT send failed\r\n");
      }
    }
    return handled;
  }

  handled = AX12_HandleCommand(app, line_copy);
  printf(handled ? "CMD OK: %s\r\n" : "CMD ERR: %s\r\n", line_copy);
  return handled;
}

void AX12_AppUpdate(AX12_AppState *app)
{
  uint32_t now_ms;

  if (app == NULL)
  {
    return;
  }

  if (app->hc05_at_mode)
  {
    AX12_FlushHc05AtResponse(app);
    return;
  }

  if (!app->ready)
  {
    return;
  }

  now_ms = HAL_GetTick();

  if (app->position_read_pending)
  {
    uint16_t position;
    AX12_Result read_result =
        AX12_GetPresentPositionReadITResult(&app->ax12, &position);

    if (read_result == AX12_OK)
    {
      app->motor_present[app->telemetry_motor_index] = position;
      app->telemetry_motor_index =
          (uint8_t)((app->telemetry_motor_index + 1U) %
                    AX12_SLAVE_MOTOR_COUNT);
      app->position_read_pending = false;
    }
    else if (read_result != AX12_ERROR_BUSY)
    {
      app->telemetry_motor_index =
          (uint8_t)((app->telemetry_motor_index + 1U) %
                    AX12_SLAVE_MOTOR_COUNT);
      app->position_read_pending = false;
    }
    else if (app->link_command_ready ||
             ((now_ms - app->position_read_started_ms) >=
              AX12_ASYNC_READ_TIMEOUT_MS))
    {
      bool timed_out =
          ((now_ms - app->position_read_started_ms) >=
           AX12_ASYNC_READ_TIMEOUT_MS);

      AX12_CancelPresentPositionReadIT(&app->ax12);
      app->position_read_pending = false;
      if (timed_out)
      {
        app->telemetry_motor_index =
            (uint8_t)((app->telemetry_motor_index + 1U) %
                      AX12_SLAVE_MOTOR_COUNT);
      }
    }
  }

  if (app->link_command_ready && !AX12_IsAsyncBusBusy(&app->ax12))
  {
    uint8_t command;
    uint8_t length;
    uint8_t payload[FUSION_MAX_PAYLOAD_LENGTH];

    __disable_irq();
    command = app->pending_command;
    length = app->pending_payload_length;
    for (uint8_t i = 0U; i < length; ++i)
    {
      payload[i] = app->pending_payload[i];
    }
    app->link_command_ready = false;
    __enable_irq();

    if (!AX12_ProcessFusionCommand(app, command, payload, length))
    {
      ++app->invalid_frame_count;
    }
  }

  if (app->link_active &&
      ((now_ms - app->last_link_rx_ms) >= HC05_LINK_TIMEOUT_MS) &&
      !app->link_timed_out)
  {
    app->link_timed_out = true;
    printf("HC05 link timeout: holding last goal positions\r\n");
  }

  /* Sharp detection releases an Auto move once.  The move then completes
   * even if the object leaves the 10–80 cm sensing range. */
  if (app->auto_start_requested && app->sharp_detected)
  {
    app->auto_motion_released = true;
  }
  if (!app->auto_start_requested || app->auto_motion_released)
  {
    if (app->auto_scurve_active)
    {
      AX12_UpdateAutoSCurveGoals(app, now_ms);
    }
    else
    {
      AX12_UpdateSmoothedGoals(app, now_ms);
    }
  }

  if (!app->position_read_pending &&
      !app->link_command_ready &&
      !AX12_IsAsyncBusBusy(&app->ax12) &&
      !AX12_GoalRampActive(app) &&
      ((now_ms - app->last_position_poll_ms) >= AX12_POSITION_POLL_MS))
  {
    if (AX12_StartNextPresentPositionRead(app))
    {
      app->last_position_poll_ms = now_ms;
      app->position_read_started_ms = now_ms;
      app->position_read_pending = true;
    }
  }
}
