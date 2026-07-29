#include "ax12_config.h"

const AX12_SlaveMotorConfig AX12_SLAVE_MOTORS[AX12_SLAVE_MOTOR_COUNT] = {
    {.master_id = AX12_MASTER_1_ID, .slave_id = AX12_SLAVE_1_ID, .reversed = false, .offset = 0},
    {.master_id = AX12_MASTER_2_ID, .slave_id = AX12_SLAVE_2_ID, .reversed = false, .offset = 0},
    {.master_id = AX12_MASTER_3_ID, .slave_id = AX12_SLAVE_3_ID, .reversed = false, .offset = 0},
    {.master_id = AX12_MASTER_4_ID, .slave_id = AX12_SLAVE_4_ID, .reversed = false, .offset = 0},
};
