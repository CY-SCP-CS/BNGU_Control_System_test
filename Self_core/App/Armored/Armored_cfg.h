//
// Created by 33140 on 2026/10/26.
//

#ifndef BNGU_CONTROL_SYSTEM_ARMORED_CFG_H
#define BNGU_CONTROL_SYSTEM_ARMORED_CFG_H

/** 放一些电机实例和基本信息*/


#define M3508_SEND_ID 0x1FF

#define MOTOR_DRIVE_FL_FEEDBACKID 0x208  //前左，电机 ID 8
#define MOTOR_DRIVE_RL_FEEDBACKID 0x206  //后左，电机 ID 6
#define MOTOR_DRIVE_FR_FEEDBACKID 0x205  //前右，电机 ID 5
#define MOTOR_DRIVE_RR_FEEDBACKID 0x207  //后右，电机 ID 7

/* CAD 核对记录：artifacts/armored_cad_dimensions.md。 */
#define MOTOR_DRIVE_WHEEL_RADIUS 77.0f  // HEXROLL D154 麦轮名义半径，mm；实车滚动半径需标定

/* 这个待后面修改成变量。 */
#define CHASSIS_ROTATION_COEFF_MM 521.55f  // 麦轮旋转系数：半轴距 + 半轮距，单位 mm

#define DRIVE_MAX_OMEGA_RAD_S 735  //轮向电机最大转子角速度 大约对应减速前7000转


/* 关节电机 ID：CAN_ID 为发送 ID，MASTER_ID 为反馈 ID，须与电机配置一致。 */
#define DM_JOINT_FL_CAN_ID    0x01U   // 前左
#define DM_JOINT_FL_MASTER_ID 0x11U
#define DM_JOINT_RL_CAN_ID    0x04U   // 后左
#define DM_JOINT_RL_MASTER_ID 0x14U
#define DM_JOINT_FR_CAN_ID    0x03U   // 前右
#define DM_JOINT_FR_MASTER_ID 0x13U
#define DM_JOINT_RR_CAN_ID    0x02U   // 后右
#define DM_JOINT_RR_MASTER_ID 0x12U

#define DM_JOINT_MOTOR_P_MAX 12.5    //达妙4340位置最大值rad
#define DM_JOINT_MOTOR_V_MAX 12.5    //达妙4340速度最大值rad/s
#define DM_JOINT_MOTOR_T_MAX 28    //达妙4340输出扭矩最大值 N.m

#define DM_JOINT_LENGTH 136.0f  // 关节转轴与轮轴之间的垂直轴间距，mm；不是电机轴向安装距离

#define GRAVITY_ARMORED 100    //先乱给一个100N重力
#endif // BNGU_CONTROL_SYSTEM_ARMORED_CFG_H
