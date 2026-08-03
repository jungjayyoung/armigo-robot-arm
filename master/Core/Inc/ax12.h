
#ifndef __AX12_H
#define __AX12_H

#include "main.h"
#include "usart.h"
#include "ax12_config.h"
#include <string.h>

/* ===========================================================================
 * 모터 설정 (이 값만 변경하면 전체 모터 제어가 자동으로 적용됩니다)
 * =========================================================================== */

// 함수 프로토타입
HAL_StatusTypeDef AX12_Ping(uint8_t id, uint8_t *status_buf, uint16_t status_len);
HAL_StatusTypeDef AX12_Write1(uint8_t id, uint8_t address, uint8_t value);
HAL_StatusTypeDef AX12_Write2(uint8_t id, uint8_t address, uint16_t value);
HAL_StatusTypeDef AX12_Read2(uint8_t id, uint8_t address, uint8_t *out_low, uint8_t *out_high);
HAL_StatusTypeDef AX12_GetPosition(uint8_t id, uint16_t *position);
uint8_t AX12_GetMotorId(uint8_t index);

/* Non-blocking position read used by the real-time FreeRTOS loop. */
HAL_StatusTypeDef AX12_StartPositionReadIT(uint8_t id);
HAL_StatusTypeDef AX12_GetPositionReadITResult(uint16_t *position);
void AX12_CancelPositionReadIT(void);
void AX12_UartTxCpltCallback(UART_HandleTypeDef *huart);
void AX12_UartRxCpltCallback(UART_HandleTypeDef *huart);
void AX12_UartErrorCallback(UART_HandleTypeDef *huart);

// 일괄 제어 유틸리티 함수
void AX12_InitAll(uint16_t speed);
void AX12_SetTorqueAll(uint8_t enable);
void AX12_SetGoalPositionAll(uint16_t goal_pos);

#endif /* __AX12_H */
