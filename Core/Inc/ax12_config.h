#ifndef AX12_CONFIG_H
#define AX12_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define AX12_APP_TIMEOUT_MS           20U
#define AX12_BUS_BAUDRATE        1000000U
#define AX12_LEGACY_BAUDRATE      115200U
#define AX12_BAUD_VALUE_1MBPS          1U

#define HC05_LINK_BAUDRATE        115200U
#define HC05_AT_BAUDRATE           38400U
#define HC05_AT_RX_BUFFER_SIZE       128U
#define FUSION_FRAME_HEADER_1         0xAAU
#define FUSION_FRAME_HEADER_2         0x55U
#define FUSION_CMD_SET_GOAL_POS       0x01U
#define FUSION_CMD_SET_TORQUE         0x02U
#define FUSION_CMD_REQ_STATUS         0x03U
#define FUSION_CMD_HOME_POS           0x04U
#define FUSION_CMD_SET_ALL_POS        0x05U
#define FUSION_CMD_START_AUTO          0x06U
#define FUSION_CMD_RUN_AUTO            0x07U /* Auto sequence: no Sharp wait. */
#define FUSION_CMD_HOLD_CURRENT         0x08U /* Latch present pose before E-stop release. */
#define FUSION_CMD_STATUS_REPLY       0x83U
#define FUSION_MAX_PAYLOAD_LENGTH       27U
#define FUSION_STATUS_PAYLOAD_LENGTH    20U /* positions(8)+loads(8)+flags(1)+Sharp mV(2)+cm(1) */
#define HC05_LINK_TIMEOUT_MS           500U

#define AX12_SLAVE_MOTOR_COUNT           4U
#define AX12_MASTER_1_ID                 10U
#define AX12_MASTER_2_ID                 11U
#define AX12_MASTER_3_ID                 12U
#define AX12_MASTER_4_ID                 14U
#define AX12_SLAVE_1_ID                   1U
#define AX12_SLAVE_2_ID                   2U
#define AX12_SLAVE_3_ID                   3U
#define AX12_SLAVE_4_ID                   5U

/* The gripper is Slave ID 2. Limit its output to about 29% of AX-12A
 * maximum torque so a closed gripper does not remain in a stall condition. */
#define AX12_GRIPPER_TORQUE_LIMIT       300U

#define AX12_GOAL_MIN                     0U
#define AX12_GOAL_MAX                  1023U
#define AX12_DEFAULT_GOAL               512U
/*
 * Motion profiles (AX-12 Moving Speed register: 0..1023).
 * Lower values move more slowly.  Adjust only these values to tune motion.
 */
#define AX12_HOME_MOVING_SPEED            80U  /* Slow, safe homing speed. */
#define AX12_TEACH_MOVING_SPEED          300U  /* Teaching preset/JOG speed. */
#define AX12_AUTO_MOVING_SPEED           80U  /* BTN14 sequence speed. Lower = slower. */
#define AX12_HOME_GOAL_MAX_STEP            1U  /* Goal ramp step for home. */
#define AX12_TEACH_GOAL_MAX_STEP           6U  /* 3U Goal ramp step for presets. */
#define AX12_AUTO_GOAL_MAX_STEP            6U  /* 3U Goal ramp step for Auto. */
/* BTN14 S-curve middle-section amount, in AX-12 position counts per update.
 * This is the only S-curve value intended for adjustment: larger = faster
 * middle section, smaller = smoother/slower movement. */
#define AX12_AUTO_SCURVE_MIDDLE_STEP         10U  /* S-curve middle amount; lower = smoother/slower. */
#define AX12_SERIAL_LINE_SIZE             48U
#define AX12_POSITION_POLL_MS              10U /* 4-axis arrival feedback. */
#define AX12_ASYNC_READ_TIMEOUT_MS           4U
#define AX12_GOAL_UPDATE_MS                  5U
#define AX12_GOAL_DEADBAND                   1U

typedef struct
{
  uint8_t master_id;
  uint8_t slave_id;
  bool reversed;
  int16_t offset;
} AX12_SlaveMotorConfig;

extern const AX12_SlaveMotorConfig
    AX12_SLAVE_MOTORS[AX12_SLAVE_MOTOR_COUNT];

#ifdef __cplusplus
}
#endif

#endif
