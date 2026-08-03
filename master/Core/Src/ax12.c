#include "ax12.h"

static const uint8_t ax12_motor_ids[AX12_NUM_MOTORS] = {
    AX12_MASTER_1_ID, AX12_MASTER_2_ID, AX12_MASTER_3_ID, AX12_MASTER_4_ID
};

typedef enum {
    AX12_IT_IDLE = 0,
    AX12_IT_TX,
    AX12_IT_RX,
    AX12_IT_DONE,
    AX12_IT_ERROR
} AX12_ItState;

static volatile AX12_ItState ax12_it_state = AX12_IT_IDLE;
static uint8_t ax12_it_id;
static uint8_t ax12_it_tx[8];
static uint8_t ax12_it_rx[8];
static uint16_t ax12_it_position;

static uint8_t ax12_checksum(const uint8_t *buf, uint8_t len_without_header)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len_without_header; i++) {
        sum += buf[i];
    }
    return (uint8_t)(~(sum & 0xFF));
}

static HAL_StatusTypeDef ax12_send_packet(uint8_t id, uint8_t instruction,
                                          const uint8_t *params, uint8_t param_len)
{
    uint8_t tx[AX12_MAX_PACKET];
    uint8_t idx = 0;

    tx[idx++] = 0xFF;
    tx[idx++] = 0xFF;
    tx[idx++] = id;
    tx[idx++] = (uint8_t)(param_len + 2);
    tx[idx++] = instruction;

    for (uint8_t i = 0; i < param_len; i++) {
        tx[idx++] = params[i];
    }

    tx[idx] = ax12_checksum(&tx[2], (uint8_t)(idx - 2U));
    idx++;

    /* Remove any stale status byte before taking ownership of the bus. */
    __HAL_UART_FLUSH_DRREGISTER(&huart1);

    if (HAL_HalfDuplex_EnableTransmitter(&huart1) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_UART_Transmit(&huart1, tx, idx, 50) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_HalfDuplex_EnableReceiver(&huart1) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static uint32_t ax12_remaining_time(uint32_t started_at, uint32_t timeout_ms)
{
    uint32_t elapsed = HAL_GetTick() - started_at;
    return (elapsed < timeout_ms) ? (timeout_ms - elapsed) : 0U;
}

static HAL_StatusTypeDef ax12_read_status(uint8_t expected_id, uint8_t *rx,
                                          uint16_t rx_len, uint32_t timeout_ms)
{
    uint8_t byte = 0U;
    uint8_t previous = 0U;
    uint32_t started_at = HAL_GetTick();
    uint32_t remaining;

    if ((rx == NULL) || (rx_len < 6U) || (rx_len > AX12_MAX_PACKET)) {
        return HAL_ERROR;
    }

    /* Synchronize on FF FF so an old or echoed byte cannot shift the packet. */
    do {
        remaining = ax12_remaining_time(started_at, timeout_ms);
        if ((remaining == 0U) ||
            (HAL_UART_Receive(&huart1, &byte, 1U, remaining) != HAL_OK)) {
            return HAL_TIMEOUT;
        }
        if ((previous == 0xFFU) && (byte == 0xFFU)) {
            break;
        }
        previous = byte;
    } while (1);

    rx[0] = 0xFFU;
    rx[1] = 0xFFU;
    remaining = ax12_remaining_time(started_at, timeout_ms);
    if ((remaining == 0U) ||
        (HAL_UART_Receive(&huart1, &rx[2], (uint16_t)(rx_len - 2U), remaining) != HAL_OK)) {
        return HAL_TIMEOUT;
    }

    if ((rx[2] != expected_id) ||
        (rx[3] != (uint8_t)(rx_len - 4U)) ||
        (rx[4] != 0U) ||
        (ax12_checksum(&rx[2], (uint8_t)(rx_len - 3U)) != rx[rx_len - 1U])) {
        return HAL_ERROR;
    }

    return HAL_OK;
}
uint8_t AX12_GetMotorId(uint8_t index)
{
    return (index < AX12_NUM_MOTORS) ? ax12_motor_ids[index] : 0U;
}

HAL_StatusTypeDef AX12_Ping(uint8_t id, uint8_t *status_buf, uint16_t status_len)
{
    uint8_t rx[6] = {0};
    HAL_StatusTypeDef ret = ax12_send_packet(id, 0x01U, NULL, 0U);
    if (ret != HAL_OK) return ret;

    ret = ax12_read_status(id, rx, sizeof(rx), 10U);
    if (ret != HAL_OK) return ret;

    if (status_buf != NULL) {
        if (status_len < sizeof(rx)) return HAL_ERROR;
        memcpy(status_buf, rx, sizeof(rx));
    }
    return HAL_OK;
}
HAL_StatusTypeDef AX12_Write1(uint8_t id, uint8_t address, uint8_t value)
{
    uint8_t params[2] = {address, value};
    uint8_t rx[6];
    HAL_StatusTypeDef ret = ax12_send_packet(id, 0x03U, params, sizeof(params));
    if (ret != HAL_OK) return ret;
    return ax12_read_status(id, rx, sizeof(rx), 10U);
}
HAL_StatusTypeDef AX12_Write2(uint8_t id, uint8_t address, uint16_t value)
{
    uint8_t params[3] = {address, (uint8_t)(value & 0xFFU),
                         (uint8_t)((value >> 8U) & 0xFFU)};
    uint8_t rx[6];
    HAL_StatusTypeDef ret = ax12_send_packet(id, 0x03U, params, sizeof(params));
    if (ret != HAL_OK) return ret;
    return ax12_read_status(id, rx, sizeof(rx), 10U);
}
HAL_StatusTypeDef AX12_Read2(uint8_t id, uint8_t address, uint8_t *out_low, uint8_t *out_high)
{
    HAL_StatusTypeDef ret;
    uint8_t params[2];
    uint8_t rx[8] = {0};

    params[0] = address;
    params[1] = 2;

    ret = ax12_send_packet(id, 0x02, params, 2);
    if (ret != HAL_OK) return ret;

    /* FF FF ID LENGTH ERROR PARAM_L PARAM_H CHECKSUM */
    ret = ax12_read_status(id, rx, sizeof(rx), 10U);
    if (ret != HAL_OK) return ret;

    if (out_low)  *out_low  = rx[5];
    if (out_high) *out_high = rx[6];

    return HAL_OK;
}

HAL_StatusTypeDef AX12_GetPosition(uint8_t id, uint16_t *position)
{
    uint8_t low = 0, high = 0;
    HAL_StatusTypeDef ret = AX12_Read2(id, AX12_ADDR_PRESENT_POS, &low, &high);
    if (ret == HAL_OK && position != NULL) {
        *position = (uint16_t)(low | (high << 8));
    }
    return ret;
}

HAL_StatusTypeDef AX12_StartPositionReadIT(uint8_t id)
{
    if (ax12_it_state != AX12_IT_IDLE) {
        return HAL_BUSY;
    }

    ax12_it_id = id;
    ax12_it_tx[0] = 0xFFU;
    ax12_it_tx[1] = 0xFFU;
    ax12_it_tx[2] = id;
    ax12_it_tx[3] = 0x04U;
    ax12_it_tx[4] = 0x02U;
    ax12_it_tx[5] = AX12_ADDR_PRESENT_POS;
    ax12_it_tx[6] = 0x02U;
    ax12_it_tx[7] = ax12_checksum(&ax12_it_tx[2], 5U);

    __HAL_UART_FLUSH_DRREGISTER(&huart1);
    if (HAL_HalfDuplex_EnableTransmitter(&huart1) != HAL_OK) {
        return HAL_ERROR;
    }

    ax12_it_state = AX12_IT_TX;
    if (HAL_UART_Transmit_IT(&huart1, ax12_it_tx, sizeof(ax12_it_tx)) != HAL_OK) {
        ax12_it_state = AX12_IT_IDLE;
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef AX12_GetPositionReadITResult(uint16_t *position)
{
    if (ax12_it_state == AX12_IT_DONE) {
        if (position != NULL) {
            *position = ax12_it_position;
        }
        ax12_it_state = AX12_IT_IDLE;
        return HAL_OK;
    }
    if (ax12_it_state == AX12_IT_ERROR) {
        ax12_it_state = AX12_IT_IDLE;
        return HAL_ERROR;
    }
    return HAL_BUSY;
}

void AX12_CancelPositionReadIT(void)
{
    (void)HAL_UART_Abort_IT(&huart1);
    (void)HAL_HalfDuplex_EnableReceiver(&huart1);
    ax12_it_state = AX12_IT_IDLE;
}

void AX12_UartTxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != &huart1) || (ax12_it_state != AX12_IT_TX)) {
        return;
    }

    if (HAL_HalfDuplex_EnableReceiver(&huart1) != HAL_OK) {
        ax12_it_state = AX12_IT_ERROR;
        return;
    }

    ax12_it_state = AX12_IT_RX;
    if (HAL_UART_Receive_IT(&huart1, ax12_it_rx, sizeof(ax12_it_rx)) != HAL_OK) {
        ax12_it_state = AX12_IT_ERROR;
    }
}

void AX12_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != &huart1) || (ax12_it_state != AX12_IT_RX)) {
        return;
    }

    if ((ax12_it_rx[0] == 0xFFU) &&
        (ax12_it_rx[1] == 0xFFU) &&
        (ax12_it_rx[2] == ax12_it_id) &&
        (ax12_it_rx[3] == 0x04U) &&
        (ax12_checksum(&ax12_it_rx[2], 5U) == ax12_it_rx[7])) {
        /* AX-12 can report an alarm bit (angle/overload/etc.) together with
         * valid Present Position parameters.  Keep reading the controller
         * position instead of discarding axes such as ID 12 or ID 14. */
        ax12_it_position =
            (uint16_t)ax12_it_rx[5] | ((uint16_t)ax12_it_rx[6] << 8U);
        ax12_it_state = AX12_IT_DONE;
    } else {
        ax12_it_state = AX12_IT_ERROR;
    }
}

void AX12_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == &huart1) && (ax12_it_state != AX12_IT_IDLE)) {
        ax12_it_state = AX12_IT_ERROR;
    }
}

/* 설정된 모터 전체 초기화 및 제어 유틸리티 */
void AX12_InitAll(uint16_t speed)
{
    for (uint8_t i = 0; i < AX12_NUM_MOTORS; i++) {
        uint8_t id = ax12_motor_ids[i];
        AX12_Write1(id, AX12_ADDR_TORQUE_ENABLE, 1);
        AX12_Write2(id, AX12_ADDR_MOVING_SPEED, speed);
    }
}

void AX12_SetTorqueAll(uint8_t enable)
{
    for (uint8_t i = 0; i < AX12_NUM_MOTORS; i++) {
        uint8_t id = ax12_motor_ids[i];
        AX12_Write1(id, AX12_ADDR_TORQUE_ENABLE, enable);
    }
}

void AX12_SetGoalPositionAll(uint16_t goal_pos)
{
    for (uint8_t i = 0; i < AX12_NUM_MOTORS; i++) {
        uint8_t id = ax12_motor_ids[i];
        AX12_Write2(id, AX12_ADDR_GOAL_POSITION, goal_pos);
    }
}
