//
// Created by 33140 on 2026/10/31.
//

#include "app_Armored_joint.h"

#include "bsp_can.h"
#include "App/Armored/Armored_cfg.h"
#include <math.h>

// 枚举要放在使用它的结构体之前。
typedef enum {
    JOINT_INACTIVE = 0,
    JOINT_ACTIVE   = 1
} joint_motor_state_t;

typedef struct {
    float start_height;
    float target_height;
    float height_ref;

    float start_pitch;
    float target_pitch;
    float pitch_ref;

    float start_roll;
    float target_roll;
    float roll_ref;

    float duration;
    float elapsed;

    uint8_t mask;                      // 本次参与平滑的项目，按位组合高度、Pitch、Roll
    uint8_t request;                   // 是否有待处理的高度及姿态过渡请求
    joint_motor_state_t state;  // 当前是否正在执行轨迹
} joint_motor_height_traj_t;

static joint_motor_height_traj_t s_height_traj; //高度、Pitch、Roll 共用进度的平滑轨迹

typedef struct {
    // 数组顺序固定为 FL、RL、FR、RR，对应下标 0～3。
    float start_pos[4];   // 起始角度，rad
    float target_pos[4];  // 最终目标角度，rad
    float pos_ref[4];     // 本周期参考角度，rad
    float vel_ref[4];     // 本周期参考速度，rad/s

    float duration;      // 动作总时间，s
    float elapsed;       // 已执行时间，s

    uint8_t motor_mask;  // 参与平滑运动的电机
    uint8_t request;     // 是否有待处理的请求
    joint_motor_state_t state;      // 是否正在执行轨迹
} joint_motor_MIT_traj_t;

static joint_motor_MIT_traj_t s_mit_traj;  //MIT模式的平滑轨迹

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
    joint_FL.MIT_kp = 0.0f;
    joint_FL.MIT_kd = 0.0f;
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
    joint_RL.MIT_kp = 0.0f;
    joint_RL.MIT_kd = 0.0f;
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
    joint_FR.MIT_kp = 0.0f;
    joint_FR.MIT_kd = 0.0f;
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
    joint_RR.MIT_kp = 0.0f;
    joint_RR.MIT_kd = 0.0f;
    lib_pid_init(&joint_RR.Height_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
    lib_pid_init(&joint_RR.pitch_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
    lib_pid_init(&joint_RR.roll_ctrl,0.0f,0.0f,
        0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
    return 1U;
}

static void joint_motor_can_rx_callback(uint32_t std_id,uint8_t *data,uint8_t len)
{
    if (!data || len != 8U) {return;}

    dm_joint_motor* motor = NULL;
    switch (std_id) {
        case DM_JOINT_FL_MASTER_ID:
            motor = &joint_FL;
            break;
        case DM_JOINT_RL_MASTER_ID:
            motor = &joint_RL;
            break;
        case DM_JOINT_FR_MASTER_ID:
            motor = &joint_FR;
            break;
        case DM_JOINT_RR_MASTER_ID:
            motor = &joint_RR;
            break;
        default:
            break;
    }

    if (motor == NULL){return;}

    drv_motor_solve_dm(data, &motor->dm_motor);

}

void joint_motor_Rx_can_reg(void)
{
    bsp_can_rx_reg(&hcan2,DM_JOINT_FL_MASTER_ID,joint_motor_can_rx_callback);
    bsp_can_rx_reg(&hcan2,DM_JOINT_FR_MASTER_ID,joint_motor_can_rx_callback);
    bsp_can_rx_reg(&hcan2,DM_JOINT_RL_MASTER_ID,joint_motor_can_rx_callback);
    bsp_can_rx_reg(&hcan2,DM_JOINT_RR_MASTER_ID,joint_motor_can_rx_callback);
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

void joint_motor_height_traj_init(void)
{
    s_height_traj = (joint_motor_height_traj_t){
        .duration = 2.0f,
        .state = JOINT_INACTIVE
    };
}

void joint_motor_init(void)
{
    joint_motor_enable_request_flag = 0U;
    joint_motor_height_traj_init();
    if (joint_motor_config_init() != 1U) {
        return;
    }
    joint_motor_Rx_can_reg();
    if (HAL_CAN_GetState(&hcan2) != HAL_CAN_STATE_LISTENING) return;
    joint_motor_enable_request();
}

/** @return 1：计算完成；0：输入或反馈角度不是有限值，保留原输出。 */
uint8_t joint_motor_vmc_force_calc(float pitch ,float roll,
            float target_height,float target_pitch,float target_roll, float ff_gravity)
{
    if (!isfinite(pitch) || !isfinite(roll) ||
        !isfinite(target_pitch) || !isfinite(target_roll) ||
        !isfinite(target_height) || !isfinite(ff_gravity) ||
        !isfinite(joint_FL.dm_motor.feedback.POS) ||
        !isfinite(joint_RL.dm_motor.feedback.POS) ||
        !isfinite(joint_FR.dm_motor.feedback.POS) ||
        !isfinite(joint_RR.dm_motor.feedback.POS)) {
        return 0U;
    }

    float height = DM_JOINT_LENGTH * (-cosf(joint_FL.dm_motor.feedback.POS) + cosf(joint_RL.dm_motor.feedback.POS)
        +cosf(joint_FR.dm_motor.feedback.POS) - cosf(joint_RR.dm_motor.feedback.POS)) / 4.0f;

    lib_pid_ff_calc(&joint_FL.Height_ctrl,target_height,height,ff_gravity / 4.0f,0,0.004);
    lib_pid_ff_calc(&joint_RL.Height_ctrl,target_height,height,ff_gravity / 4.0f,0,0.004);
    lib_pid_ff_calc(&joint_FR.Height_ctrl,target_height,height,ff_gravity / 4.0f,0,0.004);
    lib_pid_ff_calc(&joint_RR.Height_ctrl,target_height,height,ff_gravity / 4.0f,0,0.004);

    joint_FL.pitch_output = -lib_pid_calc(&joint_FL.pitch_ctrl,target_pitch,pitch,0.004);
    joint_FR.pitch_output = -lib_pid_calc(&joint_FR.pitch_ctrl,target_pitch,pitch,0.004);
    joint_RL.pitch_output = lib_pid_calc(&joint_RL.pitch_ctrl,target_pitch,pitch,0.004);
    joint_RR.pitch_output = lib_pid_calc(&joint_RR.pitch_ctrl,target_pitch,pitch,0.004);

    joint_FL.roll_output = lib_pid_calc(&joint_FL.roll_ctrl,target_roll,roll,0.004);
    joint_RL.roll_output = lib_pid_calc(&joint_RL.roll_ctrl,target_roll,roll,0.004);
    joint_FR.roll_output = -lib_pid_calc(&joint_FR.roll_ctrl,target_roll,roll,0.004);
    joint_RR.roll_output = -lib_pid_calc(&joint_RR.roll_ctrl,target_roll,roll,0.004);

    joint_FL.total_output_Force = joint_FL.Height_ctrl.out + joint_FL.pitch_output + joint_FL.roll_output;
    joint_RL.total_output_Force = joint_RL.Height_ctrl.out + joint_RL.pitch_output + joint_RL.roll_output;
    joint_FR.total_output_Force = joint_FR.Height_ctrl.out + joint_FR.pitch_output + joint_FR.roll_output;
    joint_RR.total_output_Force = joint_RR.Height_ctrl.out + joint_RR.pitch_output + joint_RR.roll_output;
    return 1U;
}

/**
 * @brief 根据各路虚拟支撑力计算关节扭矩。
 * @return 1：计算成功；0：输入或计算结果无效，保留原扭矩。
 * @note 符号按现有高度公式推导，须与实际机械零位、转向一致。
 */
uint8_t joint_motor_vmc_torque_calc(void)
{
    dm_joint_motor *motors[4] = {&joint_FL, &joint_RL, &joint_FR, &joint_RR};
    const float signs[4] = {1.0f, -1.0f, -1.0f, 1.0f};
    const float length_m = DM_JOINT_LENGTH * 0.001f;
    float torques[4];

    if (!isfinite(length_m) || length_m <= 0.0f) {return 0U;}

    for (uint8_t i = 0U; i < 4U; i++) {
        float angle = motors[i]->dm_motor.feedback.POS;
        float force = motors[i]->total_output_Force;

        if (!isfinite(angle) || !isfinite(force)) {return 0U;}

        torques[i] = signs[i] * force * length_m * sinf(angle);

        if (!isfinite(torques[i])) {return 0U;}
    }

    // 四路全部计算有效后，再统一更新。
    for (uint8_t i = 0U; i < 4U; i++) {
        motors[i]->total_output_Torque = torques[i];
    }

    return 1U;
}

/**
 * @return 1：请求待处理或轨迹正在执行；0：当前空闲。
 */
uint8_t joint_motor_height_is_busy(void)
{
    return (s_height_traj.request != 0U ||s_height_traj.state == JOINT_ACTIVE)
    ? 1U : 0U;
}

/**
 * @brief 保存本次平滑项目和过渡时间，忙碌时拒绝新请求。
 * @param smooth_mask 平滑项目掩码，可按位或组合。
 * @param during_time 过渡总时间，s，须大于 0。
 * @return 1：请求已保存；0：模式非法、时间非有限或不大于 0、当前忙碌。
 */
uint8_t joint_motor_height_move_request(joint_motor_smooth_mask_t smooth_mask,float during_time)
{
    if (smooth_mask == 0 || !isfinite(during_time) || during_time <= 0.0f ||
        (smooth_mask & ~JOINT_SMOOTH_ALL) != 0) {return 0U;}
    if (joint_motor_height_is_busy()) {return 0U;}
    s_height_traj.mask = smooth_mask;
    s_height_traj.duration = during_time;
    s_height_traj.request = 1U;
    return 1U;
}

/**
 * @brief 选中项目共用五次曲线进度，未选中项目直接使用上层目标。
 * @return 1：输出有效；0：参数无效。
 */
static uint8_t joint_motor_pose_ref_update(
    float current_height,float current_pitch,float current_roll,
    float target_height,float target_pitch,float target_roll,float dt,
    float *height_ref,float *pitch_ref,float *roll_ref)
{
    if (!height_ref || !pitch_ref || !roll_ref ||
        !isfinite(current_height) || !isfinite(current_pitch) || !isfinite(current_roll) ||
        !isfinite(target_height) || !isfinite(target_pitch) || !isfinite(target_roll) ||
        !isfinite(dt) || dt <= 0.0f) {return 0U;}

    // 全部参考值有效后再提交状态，失败时保留原轨迹和输出。
    joint_motor_height_traj_t next = s_height_traj;
    if ((next.request || next.state == JOINT_ACTIVE) &&
        (!isfinite(next.duration) || next.duration <= 0.0f)) {return 0U;}

    // 收到请求时，只初始化一次轨迹。
    if (next.request) {
        next.start_height = current_height;
        next.target_height = target_height;
        next.start_pitch = current_pitch;
        next.target_pitch = target_pitch;
        next.start_roll = current_roll;
        next.target_roll = target_roll;
        next.elapsed = 0.0f;
        next.state = JOINT_ACTIVE;
        next.request = 0U;
    }

    if (next.state == JOINT_ACTIVE) {
        next.elapsed = fminf(next.elapsed + dt, next.duration);

        if (next.elapsed >= next.duration) {
            next.height_ref = (next.mask & JOINT_SMOOTH_HEIGHT)? next.target_height : target_height;
            next.pitch_ref = (next.mask & JOINT_SMOOTH_PITCH)? next.target_pitch : target_pitch;
            next.roll_ref = (next.mask & JOINT_SMOOTH_ROLL)? next.target_roll : target_roll;

            next.state = JOINT_INACTIVE;
        } else {
            float s = next.elapsed / next.duration;
            float s2 = s * s;
            float s3 = s2 * s;
            float ratio = s3 * (10.0f - 15.0f * s + 6.0f * s2);
            ratio = fminf(fmaxf(ratio, 0.0f), 1.0f);

            if (next.mask & JOINT_SMOOTH_HEIGHT) {
                next.height_ref = next.start_height + (next.target_height - next.start_height) * ratio;
            }else{next.height_ref = target_height;}
            if (next.mask & JOINT_SMOOTH_PITCH) {
                next.pitch_ref = next.start_pitch + (next.target_pitch - next.start_pitch) * ratio;
            }else{next.pitch_ref = target_pitch;}
            if (next.mask & JOINT_SMOOTH_ROLL) {
                next.roll_ref = next.start_roll + (next.target_roll - next.start_roll) * ratio;
            }else{next.roll_ref = target_roll;}
        }
    } else {
        // 没有平滑动作，直接使用上层给定的高度和姿态目标。
        next.height_ref = target_height;
        next.pitch_ref = target_pitch;
        next.roll_ref = target_roll;
    }

    if (!isfinite(next.height_ref) || !isfinite(next.pitch_ref) || !isfinite(next.roll_ref)) {
        return 0U;
    }
    s_height_traj = next;
    *height_ref = next.height_ref;
    *pitch_ref = next.pitch_ref;
    *roll_ref = next.roll_ref;
    return 1U;
}

/**
 * @brief 执行一次 VMC 关节电机控制。
 * @return 1：本次两路控制帧均提交成功；
 *         0：未全部使能，或计算、构帧、发送失败。
 */
uint8_t joint_motor_vmc_mode_ctrl(uint8_t count,float pitch, float roll,
        float target_height,float target_pitch,float target_roll, float ff_gravity)
{
    if (!joint_motor_is_enabled()) {
        joint_motor_enable_process();
        return 0U;
    }

    float current_height = DM_JOINT_LENGTH *
    (-cosf(joint_FL.dm_motor.feedback.POS)+ cosf(joint_RL.dm_motor.feedback.POS)
     + cosf(joint_FR.dm_motor.feedback.POS)- cosf(joint_RR.dm_motor.feedback.POS)) / 4.0f;

    if (!isfinite(ff_gravity)) {return 0U;}

    float height_ref, pitch_ref, roll_ref;

    if (!joint_motor_pose_ref_update(current_height,pitch,roll,
            target_height,target_pitch,target_roll,0.004f,
            &height_ref,&pitch_ref,&roll_ref)) {return 0U;}

    if (!joint_motor_vmc_force_calc(pitch,roll,height_ref,pitch_ref,roll_ref,ff_gravity)) {return 0U;}
    if (!joint_motor_vmc_torque_calc()) {return 0U;}

    uint8_t Tx_data[4][8] = {0};
    if (!drv_motor_dm_MIT_frame_config(Tx_data[0],
            &joint_FL.dm_motor.config,0,0,joint_FL.total_output_Torque,0,0) ||
        !drv_motor_dm_MIT_frame_config(Tx_data[1],
            &joint_RL.dm_motor.config,0,0,joint_RL.total_output_Torque,0,0) ||
        !drv_motor_dm_MIT_frame_config(Tx_data[2],
            &joint_FR.dm_motor.config,0,0,joint_FR.total_output_Torque,0,0) ||
        !drv_motor_dm_MIT_frame_config(Tx_data[3],
            &joint_RR.dm_motor.config,0,0,joint_RR.total_output_Torque,0,0)) {
        return 0U;
    }

    return joint_motor_can_send(count,Tx_data[0],Tx_data[1],Tx_data[2],Tx_data[3]) == 0U
           ? 1U : 0U;
}

uint8_t joint_motor_zero_torque_ctrl(uint8_t count)
{
    uint8_t Tx_data[4][8] = {0};
    if (!drv_motor_dm_MIT_frame_config(Tx_data[0],
            &joint_FL.dm_motor.config,0,0,0,0,0) ||
        !drv_motor_dm_MIT_frame_config(Tx_data[1],
            &joint_RL.dm_motor.config,0,0,0,0,0) ||
        !drv_motor_dm_MIT_frame_config(Tx_data[2],
            &joint_FR.dm_motor.config,0,0,0,0,0) ||
        !drv_motor_dm_MIT_frame_config(Tx_data[3],
            &joint_RR.dm_motor.config,0,0,0,0,0)) {return 0U;}

    return joint_motor_can_send(count,Tx_data[0],Tx_data[1],Tx_data[2],Tx_data[3]) == 0U
           ? 1U : 0U;
}

/** 计算阻抗控制扭矩（N·m）；位置用 rad、速度用 rad/s，输入或结果非有限时返回 NAN。 */
float MIT_cal(float target_POS,float target_speed,float POS,float speed,float kp,float kd,float t_ff)
{
    if (!isfinite(target_POS) || !isfinite(target_speed) ||
        !isfinite(POS) || !isfinite(speed) ||
        !isfinite(kp) || !isfinite(kd) || !isfinite(t_ff)) {
        return NAN;
    }

    float p_term = kp * (target_POS - POS);
    float d_term = kd * (target_speed - speed);
    float output = p_term + d_term + t_ff;
    return isfinite(output) ? output : NAN;
}

/** 计算并保存关节扭矩（N·m），目标位置用 rad、目标速度用 rad/s；无效时返回 NAN，保留原输出。 */
float joint_motor_MIT_tor_cal(dm_joint_motor* motor,float target_POS,float target_speed)
{
    if (!motor) {return NAN;}

    float limit = motor->dm_motor.config.t_max;
    if (!isfinite(limit) || limit <= 0.0f) {return NAN;}

    float output = MIT_cal(target_POS,target_speed,motor->dm_motor.feedback.POS,
            motor->dm_motor.feedback.VEL,motor->MIT_kp,motor->MIT_kd,0);
    if (!isfinite(output)) {return NAN;}

    output = output > limit ? limit : output < -limit ? -limit : output;
    motor->total_output_Torque = output;
    return output;
}

// 1：请求待处理或正在执行；0：空闲。
uint8_t joint_motor_MIT_is_busy(void)
{
    return (s_mit_traj.request != 0U ||s_mit_traj.state == JOINT_ACTIVE)
    ? 1U : 0U;
}

/** 保存平滑选腿掩码和动作时间（s）；返回 1 已接收，0 参数无效或忙碌。 */
uint8_t joint_motor_MIT_move_request(joint_motor_id_t motor,float duration)
{
    if (motor == 0 || (motor & ~JOINT_MOTOR_ALL) != 0 ||!isfinite(duration) || duration <= 0.0f) {return 0U;}
    if (joint_motor_MIT_is_busy()) {return 0U;}
    s_mit_traj.motor_mask = motor;
    s_mit_traj.duration = duration;
    s_mit_traj.request = 1U;
    return 1U;
}

/** 共用五次曲线进度更新选中电机的位置（rad）和速度（rad/s）；返回 1 有效，0 失败。 */
static uint8_t joint_motor_MIT_ref_update( float target_POSFL, float target_POSRL,
                                    float target_POSFR, float target_POSRR,float dt)
{
    if (!isfinite(dt) || dt <= 0.0f) {return 0U;}

    const float targets[4] = {target_POSFL,target_POSRL,target_POSFR,target_POSRR};
    dm_joint_motor *motors[4] = {&joint_FL,&joint_RL,&joint_FR,&joint_RR};
    for (uint8_t i = 0U; i < 4U; i++) {
        if (!isfinite(targets[i])) {return 0U;}
    }

    // 使用副本，全部计算有效后再写回，失败时保留请求和轨迹进度。
    joint_motor_MIT_traj_t next = s_mit_traj;
    if ((next.request || next.state == JOINT_ACTIVE) &&
        (next.motor_mask == 0U || (next.motor_mask & ~JOINT_MOTOR_ALL) != 0U ||
         !isfinite(next.duration) || next.duration <= 0.0f)) {return 0U;}

    // 收到请求时记录一次实际角度和最终目标，执行过程中不重复初始化。
    if (next.request) {
        for (uint8_t i = 0U; i < 4U; i++) {
            if (next.motor_mask & (1U << i)) {
                if (!isfinite(motors[i]->dm_motor.feedback.POS)) {return 0U;}
                next.start_pos[i] = motors[i]->dm_motor.feedback.POS;
                next.target_pos[i] = targets[i];
            }
        }
        next.elapsed = 0.0f;
        next.state = JOINT_ACTIVE;
        next.request = 0U;
    }

    if (next.state == JOINT_ACTIVE) {
        if (!isfinite(next.elapsed) || next.elapsed < 0.0f) {return 0U;}
        // 累加本周期时间 dt（s），并将已执行时间限制在动作总时间内。
        next.elapsed = fminf(next.elapsed + dt, next.duration);

        if (next.elapsed >= next.duration) {
            for (uint8_t i = 0U; i < 4U; i++) {
                next.pos_ref[i] = (next.motor_mask & (1U << i)) ? next.target_pos[i] : targets[i];
                next.vel_ref[i] = 0.0f;
            }
            next.state = JOINT_INACTIVE;
        } else {
            // 时间归一化：s = 已执行时间 / 总时间，取值从 0 到 1。
            float s = next.elapsed / next.duration;
            float s2 = s * s;  // s 的平方，供后续公式复用。
            float s3 = s2 * s; // s 的立方。
            // 五次插值比例：10s³ - 15s⁴ + 6s⁵，起点和终点的参考速度、加速度均为 0。
            float ratio = s3 * (10.0f - 15.0f * s + 6.0f * s2);
            // 将位置比例限制在 [0, 1]，避免浮点误差使参考位置越过端点。
            ratio = fminf(fmaxf(ratio, 0.0f), 1.0f);
            // 比例对 s 求导为 30s²(1-s)²，再乘 ds/dt = 1/duration，得到比例变化率（1/s）。
            float speed_ratio = 30.0f * s2 * (1.0f - s) * (1.0f - s) / next.duration;

            for (uint8_t i = 0U; i < 4U; i++) {
                if (next.motor_mask & (1U << i)) {
                    // 总角度差（rad），正负决定本次运动方向。
                    float delta = next.target_pos[i] - next.start_pos[i];
                    // 起始角度加上已完成的角度差，得到本周期参考位置（rad）。
                    next.pos_ref[i] = next.start_pos[i] + delta * ratio;
                    // 位置公式对时间求导：总角度差乘比例变化率，得到参考角速度（rad/s）。
                    next.vel_ref[i] = delta * speed_ratio;
                } else {
                    next.pos_ref[i] = targets[i];
                    next.vel_ref[i] = 0.0f;
                }
            }
        }
    } else {
        // 没有平滑动作，直接使用调用方准备好的四路位置目标。
        for (uint8_t i = 0U; i < 4U; i++) {
            next.pos_ref[i] = targets[i];
            next.vel_ref[i] = 0.0f;
        }
    }

    for (uint8_t i = 0U; i < 4U; i++) {
        if (!isfinite(next.pos_ref[i]) || !isfinite(next.vel_ref[i])) {return 0U;}
    }
    s_mit_traj = next;
    return 1U;
}

/** 进入位置模式时调用一次，记录保持角度并清除旧轨迹；返回 1 成功，0 反馈无效。 */
uint8_t joint_motor_MIT_mode_enter(void)
{
    dm_joint_motor *motors[4] = {
        &joint_FL, &joint_RL, &joint_FR, &joint_RR
    };

    // 先全部检查，再统一更新。
    for (uint8_t i = 0; i < 4; i++) {
        if (!isfinite(motors[i]->dm_motor.feedback.POS)) {
            return 0U;
        }
    }

    s_mit_traj = (joint_motor_MIT_traj_t){0};
    for (uint8_t i = 0; i < 4; i++) {
        motors[i]->MIT_target_POS = motors[i]->dm_motor.feedback.POS;
        s_mit_traj.pos_ref[i] = motors[i]->MIT_target_POS;
    }

    return 1U;
}

uint8_t joint_motor_MIT_mode_ctrl(uint8_t count,joint_motor_id_t motor,float target_POSFL,
                                    float target_POSRL,float target_POSFR,float target_POSRR)
{
    if ((motor & ~JOINT_MOTOR_ALL) != 0) {return 0U;}

    if (((motor & JOINT_MOTOR_FL) && !isfinite(target_POSFL)) ||
    ((motor & JOINT_MOTOR_RL) && !isfinite(target_POSRL)) ||
    ((motor & JOINT_MOTOR_FR) && !isfinite(target_POSFR)) ||
    ((motor & JOINT_MOTOR_RR) && !isfinite(target_POSRR))) {return 0U;}

    if (!joint_motor_is_enabled()) {
        joint_motor_enable_process();
        return 0U;}

    // 只更新选中电机的位置目标。
    if (motor & JOINT_MOTOR_FL) joint_FL.MIT_target_POS = target_POSFL;
    if (motor & JOINT_MOTOR_RL) joint_RL.MIT_target_POS = target_POSRL;
    if (motor & JOINT_MOTOR_FR) joint_FR.MIT_target_POS = target_POSFR;
    if (motor & JOINT_MOTOR_RR) joint_RR.MIT_target_POS = target_POSRR;

    if (!joint_motor_MIT_ref_update(joint_FL.MIT_target_POS,joint_RL.MIT_target_POS,
            joint_FR.MIT_target_POS,joint_RR.MIT_target_POS,0.004f)) {return 0U;}

    // 四路均使用本周期的位置、速度参考计算扭矩。
    if (!isfinite(joint_motor_MIT_tor_cal(&joint_FL, s_mit_traj.pos_ref[0], s_mit_traj.vel_ref[0])) ||
        !isfinite(joint_motor_MIT_tor_cal(&joint_RL, s_mit_traj.pos_ref[1], s_mit_traj.vel_ref[1])) ||
        !isfinite(joint_motor_MIT_tor_cal(&joint_FR, s_mit_traj.pos_ref[2], s_mit_traj.vel_ref[2])) ||
        !isfinite(joint_motor_MIT_tor_cal(&joint_RR, s_mit_traj.pos_ref[3], s_mit_traj.vel_ref[3]))) {
        return 0U;
        }

    uint8_t Tx_data[4][8] = {0};
    if (!drv_motor_dm_MIT_frame_config(Tx_data[0],
            &joint_FL.dm_motor.config,0,0,joint_FL.total_output_Torque,0,0) ||
        !drv_motor_dm_MIT_frame_config(Tx_data[1],
            &joint_RL.dm_motor.config,0,0,joint_RL.total_output_Torque,0,0) ||
        !drv_motor_dm_MIT_frame_config(Tx_data[2],
            &joint_FR.dm_motor.config,0,0,joint_FR.total_output_Torque,0,0) ||
        !drv_motor_dm_MIT_frame_config(Tx_data[3],
            &joint_RR.dm_motor.config,0,0,joint_RR.total_output_Torque,0,0)) {
        return 0U;
            }

    return joint_motor_can_send(count,Tx_data[0],Tx_data[1],Tx_data[2],Tx_data[3]) == 0U
           ? 1U : 0U;
}
