//
// Created by 33140 on 2026/10/31.
//

#include "app_Armored_joint.h"

#include "bsp_can.h"
#include "App/Armored/Armored_cfg.h"

dm_joint_motor joint_FL;  //前左4
dm_joint_motor joint_RL;  //后左2
dm_joint_motor joint_FR;  //前右1
dm_joint_motor joint_RR;  //后右3

uint8_t joint_motor_enable_request_flag = 0;

/**
 * @brief 配置四路关节电机及其 PID，任一路电机配置失败时立即返回。
 * @return 1 表示四路配置均成功；0 表示至少一路电机配置失败。
 */
uint8_t joint_motor_config_init(void)
{
    if (drv_motor_init_dm(&joint_FL.dm_motor,DRV_MOTOR_DM_MODE_MIT ,DM_JOINT_FL_CAN_ID,
        DM_JOINT_FL_MASTER_ID,DM_JOINT_MOTOR_P_MAX,DM_JOINT_MOTOR_V_MAX,DM_JOINT_MOTOR_T_MAX) != 1U) {
        return 0U;
    }
    lib_pid_init(&joint_FL.Height_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
    lib_pid_init(&joint_FL.pitch_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
    lib_pid_init(&joint_FL.roll_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);

    if (drv_motor_init_dm(&joint_RL.dm_motor,DRV_MOTOR_DM_MODE_MIT ,DM_JOINT_RL_CAN_ID,
        DM_JOINT_RL_MASTER_ID,DM_JOINT_MOTOR_P_MAX,DM_JOINT_MOTOR_V_MAX,DM_JOINT_MOTOR_T_MAX) != 1U) {
        return 0U;
    }
    lib_pid_init(&joint_RL.Height_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
    lib_pid_init(&joint_RL.pitch_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
    lib_pid_init(&joint_RL.roll_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);

    if (drv_motor_init_dm(&joint_FR.dm_motor,DRV_MOTOR_DM_MODE_MIT ,DM_JOINT_FR_CAN_ID,
        DM_JOINT_FR_MASTER_ID,DM_JOINT_MOTOR_P_MAX,DM_JOINT_MOTOR_V_MAX,DM_JOINT_MOTOR_T_MAX) != 1U) {
        return 0U;
    }
    lib_pid_init(&joint_FR.Height_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
    lib_pid_init(&joint_FR.pitch_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
    lib_pid_init(&joint_FR.roll_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);

    if (drv_motor_init_dm(&joint_RR.dm_motor,DRV_MOTOR_DM_MODE_MIT ,DM_JOINT_RR_CAN_ID,
        DM_JOINT_RR_MASTER_ID,DM_JOINT_MOTOR_P_MAX,DM_JOINT_MOTOR_V_MAX,DM_JOINT_MOTOR_T_MAX) != 1U) {
        return 0U;
    }
    lib_pid_init(&joint_RR.Height_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
    lib_pid_init(&joint_RR.pitch_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
    lib_pid_init(&joint_RR.roll_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
    return 1U;
}

void joint_motor_Rx_can_reg(void)
{
    bsp_can_rx_reg(&hcan2,DM_JOINT_FL_MASTER_ID,NULL);
    bsp_can_rx_reg(&hcan2,DM_JOINT_FR_MASTER_ID,NULL);
    bsp_can_rx_reg(&hcan2,DM_JOINT_RL_MASTER_ID,NULL);
    bsp_can_rx_reg(&hcan2,DM_JOINT_RR_MASTER_ID,NULL);
}

/**
 * @brief 非阻塞地执行一次关节电机使能发送，最多成功提交两帧。
 * @details 请求标志为 1 时，构建使能命令，按 FL、RL、FR、RR 顺序检查电机。
 *          仅向反馈状态为 DISABLED 的电机发送；提交两帧、检查完四路或发送失败时退出。
 * @retval -1 请求标志不为 1，本次不发送使能命令。
 * @retval -2 本次发生发送失败；失败前可能已经成功提交一帧。
 * @retval 0 有使能请求，但没有状态为 DISABLED 的电机，未提交使能帧。
 * @retval 1 成功提交一帧，且本次无发送失败。
 * @retval 2 成功提交两帧，且本次无发送失败。
*/
static int8_t joint_motor_enable_process(void)
{
    uint8_t frame[8] = {0};
    uint8_t i = 0,tx_num = 0,err = 0;
    dm_joint_motor* motor[4] = {&joint_FL,&joint_RL,&joint_FR,&joint_RR};
    if (joint_motor_enable_request_flag==1) {
        drv_motor_dm_command_frame(frame,DRV_MOTOR_DM_CMD_ENABLE);
        for (;tx_num<2&&i<4;i++) {
            if (motor[i]->dm_motor.feedback.err==DRV_MOTOR_DM_STATUS_DISABLED) {
                if (bsp_can_tx(&hcan2,motor[i]->dm_motor.config.cmd_id,frame)!=0){err++;break;}
                tx_num++;
            }
        }
        if (err>0){return -2;}
        return tx_num;
    }
    return -1;
}

/**
 *
 * @return 1 已经使能完毕 0 未使能完毕
 */
uint8_t joint_motor_is_enabled(void)
{
    if (joint_FL.dm_motor.feedback.err==DRV_MOTOR_DM_STATUS_ENABLED &&
        joint_RL.dm_motor.feedback.err==DRV_MOTOR_DM_STATUS_ENABLED &&
        joint_FR.dm_motor.feedback.err==DRV_MOTOR_DM_STATUS_ENABLED &&
        joint_RR.dm_motor.feedback.err==DRV_MOTOR_DM_STATUS_ENABLED) {
        joint_motor_enable_request_flag = 0;
        return 1U;
        }
    return 0U;
}

/**
 * @brief 按 count 奇偶交替发送左右两路关节电机控制帧。
 * @param count 调用计数，偶数发送 FL/RL，奇数发送 FR/RR；全部 uint8_t 值均有效。
 * @param dataFL 前左电机数据，须指向至少 8 字节的有效缓冲区。
 * @param dataRL 后左电机数据，须指向至少 8 字节的有效缓冲区。
 * @param dataFR 前右电机数据，须指向至少 8 字节的有效缓冲区。
 * @param dataRR 后右电机数据，须指向至少 8 字节的有效缓冲区。
 * @return 0 表示本次两帧正常控制帧均提交成功；1 表示控制数据指针为空或至少一次发送失败；
 *         2 表示未全部使能，本次跳过正常控制帧发送。
 * @note 使能处理先于控制数据指针检查，不依赖正常控制帧缓冲区。
 *       发送正常控制帧时，任一数据指针为空则不发送正常控制帧；缓冲区长度由调用方保证。
 */
static uint8_t joint_motor_can_send(uint8_t count,uint8_t dataFL[8],uint8_t dataRL[8],uint8_t dataFR[8],uint8_t dataRR[8])
{
    if (!joint_motor_is_enabled()) {
        int8_t enable_ret = joint_motor_enable_process();
        return (enable_ret == -2) ? 1U : 2U;
    }

    if (!dataFL || !dataRL || !dataFR || !dataRR) {
        return 1U;
    }

    uint8_t ret1 , ret2;
    if (count%2==0) {
        ret1 = bsp_can_tx(&hcan2,DM_JOINT_FL_CAN_ID,dataFL);
        ret2 = bsp_can_tx(&hcan2,DM_JOINT_RL_CAN_ID,dataRL);
    }
    else {
        ret1 = bsp_can_tx(&hcan2,DM_JOINT_FR_CAN_ID,dataFR);
        ret2 = bsp_can_tx(&hcan2,DM_JOINT_RR_CAN_ID,dataRR);
    }
    return ret1 | ret2;
}

void joint_motor_enable_request(void)
{
    if (joint_motor_enable_request_flag == 0U){joint_motor_enable_request_flag = 1;}
}

void joint_motor_init(void)
{
    joint_motor_enable_request_flag = 0U;
    if (joint_motor_config_init() != 1U) {
        return;
    }
    joint_motor_Rx_can_reg();
    if (HAL_CAN_GetState(&hcan2) != HAL_CAN_STATE_LISTENING) return;
    joint_motor_enable_request();
}
