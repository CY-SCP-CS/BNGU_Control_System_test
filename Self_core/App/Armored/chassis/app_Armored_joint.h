/** 四路达妙关节电机控制接口。 */

#ifndef BNGU_CONTROL_SYSTEM_APP_ARMORED_JOINT_H
#define BNGU_CONTROL_SYSTEM_APP_ARMORED_JOINT_H

#include "drv_motor.h"
#include "lib_pid.h"

/** 单路关节电机的反馈、控制器及本周期计算结果。 */
typedef struct {
    drv_motor_dm_t dm_motor;   // 电机配置及反馈；反馈角度单位 rad。
    lib_pid_t Height_ctrl;     // 高度控制器，输入高度单位 mm。
    lib_pid_t pitch_ctrl;      // 俯仰控制器，角度单位为度。
    lib_pid_t roll_ctrl;       // 横滚控制器，角度单位为度。
    float pitch_output;       // 分配到本关节的俯仰修正力，N。
    float roll_output;        // 分配到本关节的横滚修正力，N。
    float total_output_Force;  // 高度支撑力与姿态修正力的合力，N。
    float total_output_Torque;// 由合力映射得到的关节扭矩，N·m。
    float MIT_kp;
    float MIT_kd;
    float MIT_target_POS;  // 保存的位置目标，rad
}dm_joint_motor;

typedef enum {
    JOINT_SMOOTH_HEIGHT = 1U << 0, // 平滑高度
    JOINT_SMOOTH_PITCH  = 1U << 1, // 平滑俯仰角
    JOINT_SMOOTH_ROLL   = 1U << 2, // 平滑横滚角
    JOINT_SMOOTH_ALL    = 7U       // 三项同时平滑
} joint_motor_smooth_mask_t; // 可用按位或组合选择平滑项目

typedef enum {
    JOINT_MOTOR_FL  = 1U << 0,
    JOINT_MOTOR_RL  = 1U << 1,
    JOINT_MOTOR_FR  = 1U << 2,
    JOINT_MOTOR_RR  = 1U << 3,
    JOINT_MOTOR_ALL = 0x0FU
} joint_motor_id_t;


/** 初始化四路关节电机、PID 和 CAN2 反馈回调；调用前须启动 CAN2。 */
void joint_motor_init(void);

/** 查询高度及姿态轨迹：1 表示请求待处理或正在执行，0 表示空闲；空闲不代表实际已到位。 */
uint8_t joint_motor_height_is_busy(void);

/**
 * @brief 请求选中项目同步平滑变化，目标由 VMC 控制接口传入。
 * @param smooth_mask 平滑项目，可按位或组合高度、Pitch、Roll。
 * @param during_time 过渡总时间，s，须大于 0。
 * @return 1：请求已保存；0：模式非法、时间非有限或不大于 0、当前忙碌。
 */
uint8_t joint_motor_height_move_request(joint_motor_smooth_mask_t smooth_mask,float during_time);

/**
 * @brief 每 4 ms 执行一次 VMC 高度及姿态控制。
 * @param count 发送计数，偶数发左侧两路，奇数发右侧两路。
 * @param pitch 当前俯仰角，度。
 * @param roll 当前横滚角，度。
 * @param target_height 目标高度，mm。
 * @param target_pitch 目标俯仰角，度。
 * @param target_roll 目标横滚角，度。
 * @param ff_gravity 整车重力支撑前馈，N。
 * @return 1：本次两帧均提交成功；0：未使能或处理失败。
 * @note 仅选中项目平滑，执行期间及结束时保持其目标一致；未选中项目直接跟随上层目标。
 */
uint8_t joint_motor_vmc_mode_ctrl(uint8_t count,float pitch, float roll,
        float target_height,float target_pitch,float target_roll, float ff_gravity);

/**
 * @brief 发送零扭矩指令，不等于失能。
 * @param count 发送计数，偶数发左侧两路，奇数发右侧两路。
 * @return 1：本次两帧均提交成功；0：未使能或处理失败。
 */
uint8_t joint_motor_zero_torque_ctrl(uint8_t count);

/** 查询 MIT 平滑轨迹：1 表示请求待处理或正在执行，0 表示空闲；空闲不代表实际已到位。 */
uint8_t joint_motor_MIT_is_busy(void);

/** 请求选中电机平滑运动，duration 单位 s；返回 1 已接收，0 参数无效或忙碌。
 *  motor 可按位或组合；进入位置模式后发起请求，目标角度由后续周期控制提供。 */
uint8_t joint_motor_MIT_move_request(joint_motor_id_t motor,float duration);

/** 进入位置模式时调用一次，记录四路保持角并清除旧轨迹；返回 1 成功，0 反馈无效。 */
uint8_t joint_motor_MIT_mode_enter(void);

/** 每 4 ms 更新 MIT 位置控制，目标角度单位 rad；平滑期间保持选中电机的最终目标一致。
 *  motor 选择更新目标的电机，未选中电机保持原目标；返回 1 本次两帧提交成功，0 失败。 */
uint8_t joint_motor_MIT_mode_ctrl(uint8_t count,joint_motor_id_t motor,float target_POSFL,
                                float target_POSRL,float target_POSFR,float target_POSRR);

#endif // BNGU_CONTROL_SYSTEM_APP_ARMORED_JOINT_H
