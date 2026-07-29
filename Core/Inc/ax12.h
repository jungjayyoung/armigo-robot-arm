#ifndef AX12_H
#define AX12_H

/**
 * @file ax12.h
 * @brief Minimal Dynamixel Protocol 1.0 driver for AX-12/AX-12A motors.
 *
 * The driver owns no UART instance. The application supplies a UART already
 * configured for STM32 half-duplex (single-wire) operation.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "ax12_config.h"

/* AX-12/AX-12A communicates with Dynamixel Protocol 1.0 packets. */
#define AX12_PROTOCOL_1_0          1U

/* Protocol-wide IDs and defaults. 0xFE addresses every motor on the bus. */
#define AX12_HEADER                0xFFU
#define AX12_BROADCAST_ID          0xFEU
#define AX12_DEFAULT_ID            2U

/* AX-12A control-table addresses used by this project. */
#define AX12_ADDR_MODEL_NUMBER     0U
#define AX12_ADDR_FIRMWARE_VER     2U
#define AX12_ADDR_ID               3U
#define AX12_ADDR_BAUD_RATE        4U
#define AX12_ADDR_RETURN_DELAY     5U
#define AX12_ADDR_CW_LIMIT         6U
#define AX12_ADDR_CCW_LIMIT        8U
#define AX12_ADDR_TEMP_LIMIT       11U
#define AX12_ADDR_MAX_TORQUE       14U
#define AX12_ADDR_STATUS_LEVEL     16U
#define AX12_ADDR_ALARM_LED        17U
#define AX12_ADDR_SHUTDOWN         18U
#define AX12_ADDR_TORQUE_ENABLE    24U
#define AX12_ADDR_LED              25U
#define AX12_ADDR_GOAL_POSITION    30U
#define AX12_ADDR_MOVING_SPEED     32U
#define AX12_ADDR_TORQUE_LIMIT     34U
#define AX12_ADDR_PRESENT_POSITION 36U
#define AX12_ADDR_PRESENT_LOAD     40U

typedef enum
{
  AX12_OK = 0,             /* Transaction completed without an error. */
  AX12_ERROR_ARGUMENT,     /* Null pointer, invalid range, or invalid count. */
  AX12_ERROR_BUSY,         /* An interrupt-driven bus transfer is active. */
  AX12_ERROR_UART,         /* STM32 HAL transmit/receive operation failed. */
  AX12_ERROR_TIMEOUT,      /* No complete status packet arrived in time. */
  AX12_ERROR_PACKET,       /* Status packet ID, length, or payload is invalid. */
  AX12_ERROR_CHECKSUM,     /* Protocol 1.0 checksum verification failed. */
  AX12_ERROR_DEVICE        /* AX-12 status packet reported a device error. */
} AX12_Result;

/* Runtime state shared by all driver calls for one AX-12 bus. */
typedef struct
{
  UART_HandleTypeDef *uart;  /* Half-duplex UART connected to the DATA line. */
  uint32_t timeout_ms;       /* Total timeout for one command/response pair. */
  uint8_t last_device_error; /* Error byte from the latest status packet. */
} AX12_Handle;

/**
 * @brief Bind the driver to an initialized half-duplex UART.
 * @param ax12 Driver state to initialize.
 * @param uart STM32 HAL UART handle connected to AX-12 DATA.
 * @param timeout_ms Maximum time for one transaction.
 */
void AX12_Init(AX12_Handle *ax12, UART_HandleTypeDef *uart, uint32_t timeout_ms);

/* Returns the raw AX-12 error byte from the latest status packet. */
uint8_t AX12_GetLastDeviceError(const AX12_Handle *ax12);

/* Checks whether the selected motor responds on the current bus baud rate. */
AX12_Result AX12_Ping(AX12_Handle *ax12, uint8_t id);

/* Low-level control-table write helpers. Words are sent little-endian. */
AX12_Result AX12_WriteByte(AX12_Handle *ax12, uint8_t id,
                           uint8_t address, uint8_t value);
AX12_Result AX12_WriteWord(AX12_Handle *ax12, uint8_t id,
                           uint8_t address, uint16_t value);

/* Low-level control-table read helpers. Output is written only on success. */
AX12_Result AX12_ReadByte(AX12_Handle *ax12, uint8_t id,
                          uint8_t address, uint8_t *value);
AX12_Result AX12_ReadWord(AX12_Handle *ax12, uint8_t id,
                          uint8_t address, uint16_t *value);

/* Configuration and command wrappers around the AX-12A control table. */
AX12_Result AX12_SetId(AX12_Handle *ax12, uint8_t id, uint8_t new_id);
AX12_Result AX12_SetBaudRate(AX12_Handle *ax12, uint8_t id, uint8_t baud_value);
AX12_Result AX12_SetReturnDelayTime(AX12_Handle *ax12, uint8_t id, uint8_t delay_time);
AX12_Result AX12_SetPositionMode(AX12_Handle *ax12, uint8_t id);
AX12_Result AX12_SetWheelMode(AX12_Handle *ax12, uint8_t id);
AX12_Result AX12_SetLed(AX12_Handle *ax12, uint8_t id, bool enabled);
AX12_Result AX12_SetTorque(AX12_Handle *ax12, uint8_t id, bool enabled);
AX12_Result AX12_SetGoalPosition(AX12_Handle *ax12, uint8_t id, uint16_t position);
AX12_Result AX12_SetMovingSpeed(AX12_Handle *ax12, uint8_t id, uint16_t speed);

/**
 * @brief Update several motors' Goal Position with one broadcast packet.
 * @param ax12 Initialized AX-12 bus handle.
 * @param ids Array of destination motor IDs.
 * @param positions Array of 10-bit Goal Position values.
 * @param count Number of entries in both arrays.
 *
 * A Sync Write is not acknowledged because it uses the broadcast ID.
 */
AX12_Result AX12_SyncWriteGoalPositions(AX12_Handle *ax12,
                                        const uint8_t *ids,
                                        const uint16_t *positions,
                                        uint8_t count);
AX12_Result AX12_SyncWriteGoalPositionsIT(AX12_Handle *ax12,
                                          const uint8_t *ids,
                                          const uint16_t *positions,
                                          uint8_t count);
AX12_Result AX12_StartPresentPositionReadIT(AX12_Handle *ax12, uint8_t id);
AX12_Result AX12_GetPresentPositionReadITResult(AX12_Handle *ax12,
                                                uint16_t *position);
void AX12_CancelPresentPositionReadIT(AX12_Handle *ax12);
bool AX12_IsAsyncTxBusy(const AX12_Handle *ax12);
bool AX12_IsAsyncBusBusy(const AX12_Handle *ax12);
void AX12_UartTxCpltCallback(AX12_Handle *ax12,
                             UART_HandleTypeDef *huart);
void AX12_UartRxCpltCallback(AX12_Handle *ax12,
                             UART_HandleTypeDef *huart);
void AX12_UartErrorCallback(AX12_Handle *ax12,
                            UART_HandleTypeDef *huart);

/* Read-only information and live-state wrappers. */
AX12_Result AX12_GetId(AX12_Handle *ax12, uint8_t id, uint8_t *value);
AX12_Result AX12_GetBaudRate(AX12_Handle *ax12, uint8_t id, uint8_t *value);
AX12_Result AX12_GetReturnDelayTime(AX12_Handle *ax12, uint8_t id, uint8_t *value);
AX12_Result AX12_GetModelNumber(AX12_Handle *ax12, uint8_t id, uint16_t *model);
AX12_Result AX12_GetFirmwareVersion(AX12_Handle *ax12, uint8_t id, uint8_t *version);
AX12_Result AX12_GetPresentPosition(AX12_Handle *ax12, uint8_t id, uint16_t *position);

/* Fusion-link application state and API. */
typedef struct
{
  AX12_Handle ax12; UART_HandleTypeDef *link_uart;
  uint16_t motor_goal[AX12_SLAVE_MOTOR_COUNT], motor_target[AX12_SLAVE_MOTOR_COUNT], motor_present[AX12_SLAVE_MOTOR_COUNT], motor_load[AX12_SLAVE_MOTOR_COUNT];
  bool motor_torque_enabled[AX12_SLAVE_MOTOR_COUNT];
  bool ready, link_active, link_timed_out, auto_start_requested, auto_motion_released; volatile bool sharp_detected;
  bool auto_scurve_active; uint16_t auto_scurve_start[AX12_SLAVE_MOTOR_COUNT];
  uint32_t auto_scurve_started_ms, auto_scurve_duration_ms, auto_scurve_last_update_ms;
  uint32_t last_link_rx_ms, last_goal_update_ms, last_position_poll_ms, position_read_started_ms;
  uint16_t motion_speed, goal_max_step;
  uint8_t telemetry_motor_index; bool position_read_pending;
  volatile uint32_t valid_frame_count, invalid_frame_count, dropped_frame_count;
  uint8_t link_rx_byte, link_rx_state, link_command, link_payload_length, link_payload_index, link_checksum;
  uint8_t link_payload[FUSION_MAX_PAYLOAD_LENGTH], pending_command, pending_payload_length, pending_payload[FUSION_MAX_PAYLOAD_LENGTH]; volatile bool link_command_ready;
  bool hc05_at_mode; uint8_t hc05_at_rx_buffer[HC05_AT_RX_BUFFER_SIZE]; volatile uint16_t hc05_at_rx_head, hc05_at_rx_tail; volatile uint32_t hc05_at_rx_dropped;
  char serial_line[AX12_SERIAL_LINE_SIZE]; uint8_t serial_line_len; volatile bool serial_line_ready;
} AX12_AppState;
bool AX12_AppInit(AX12_AppState *, UART_HandleTypeDef *, UART_HandleTypeDef *);
bool AX12_AppStartConsoleRx(AX12_AppState *, UART_HandleTypeDef *);
bool AX12_AppStartLinkRx(AX12_AppState *);
bool AX12_AppProcessSerial(AX12_AppState *, UART_HandleTypeDef *);
void AX12_AppUpdate(AX12_AppState *);
void AX12_AppSetSharpDetected(AX12_AppState *, bool);
void AX12_AppUartRxCpltCallback(UART_HandleTypeDef *);
void AX12_AppUartTxCpltCallback(UART_HandleTypeDef *);
void AX12_AppUartErrorCallback(UART_HandleTypeDef *);

#ifdef __cplusplus
}
#endif

#endif
