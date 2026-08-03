#ifndef AX12_CONFIG_H
#define AX12_CONFIG_H

#define AX12_NUM_MOTORS               4U
#define AX12_MASTER_1_ID             10U
#define AX12_MASTER_2_ID             11U
#define AX12_MASTER_3_ID             12U
#define AX12_MASTER_4_ID             14U

#define AX12_MASTER_1_MIN_POSITION  140U
#define AX12_MASTER_1_MAX_POSITION  900U
#define AX12_MASTER_2_MIN_POSITION  300U
#define AX12_MASTER_2_MAX_POSITION  750U
#define AX12_MASTER_3_MIN_POSITION  400U
#define AX12_MASTER_3_MAX_POSITION 1023U
#define AX12_MASTER_4_MIN_POSITION    0U
#define AX12_MASTER_4_MAX_POSITION 1023U

/* Master-controller speed used only by BTN16 Home (0..1023).
 * Lower values move more slowly; change this value to tune homing speed. */
#define AX12_MASTER_HOME_SPEED       30U

/* Controller alignment used after PB2 E-stop.  The controller moves to the
 * robot's held pose before BTN15 can enable manual JOG. */
#define AX12_MASTER_ESTOP_SYNC_SPEED 30U
#define AX12_MASTER_ESTOP_SYNC_TOLERANCE 10U  

/* Auto-sequence arrival window.  The gripper is slave axis 2 (array index 1)
 * and may stop short when it clamps an object, so it is allowed a wider
 * completion range.  Other axes continue to use the teaching tolerance. */
#define AX12_AUTO_GRIPPER_AXIS_INDEX 1U
#define AX12_AUTO_GRIPPER_TOLERANCE 80U

#define AX12_MAX_PACKET              32U
#define AX12_ADDR_TORQUE_ENABLE      24U
#define AX12_ADDR_GOAL_POSITION      30U
#define AX12_ADDR_MOVING_SPEED       32U
#define AX12_ADDR_PRESENT_POS        36U
#define AX12_ADDR_PRESENT_VOLTAGE     42U
#define AX12_ADDR_PRESENT_TEMPERATURE 43U

#endif
