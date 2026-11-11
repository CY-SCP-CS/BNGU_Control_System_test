//
// Created by 33140 on 2026/10/26.
//

#ifndef BNGU_CONTROL_SYSTEM_ARMORED_CFG_H
#define BNGU_CONTROL_SYSTEM_ARMORED_CFG_H

/** 放一些电机实例和基本信息*/


#define M3508_SEND_ID 0x1FF

#define MOTOR_DRIVE_FL_FEEDBACKID 0x205  //前左
#define MOTOR_DRIVE_RL_FEEDBACKID 0x206
#define MOTOR_DRIVE_FR_FEEDBACKID 0x207
#define MOTOR_DRIVE_RR_FEEDBACKID 0x208

#define MOTOR_DRIVE_WHEEL_RADIUS 75.81f  //车轮半径75.81mm
#define CHASSIS_ROTATION_COEFF_MM 521.55f  // 麦轮旋转系数：半轴距 + 半轮距，单位 mm

#define DRIVE_MAX_OMEGA_RAD_S 735  //轮向电机最大转子角速度 大约对应减速前7000转


/* 关节电机 ID：CAN_ID 为发送 ID，MASTER_ID 为反馈 ID，须与电机配置一致。 */
#define DM_JOINT_FL_CAN_ID    0x04U   // 前左
#define DM_JOINT_FL_MASTER_ID 0x404U
#define DM_JOINT_RL_CAN_ID    0x02U   // 后左
#define DM_JOINT_RL_MASTER_ID 0x402U
#define DM_JOINT_FR_CAN_ID    0x01U   // 前右
#define DM_JOINT_FR_MASTER_ID 0x401U
#define DM_JOINT_RR_CAN_ID    0x03U   // 后右
#define DM_JOINT_RR_MASTER_ID 0x403U

#define DM_JOINT_MOTOR_P_MAX 12.5    //达妙4340位置最大值rad
#define DM_JOINT_MOTOR_V_MAX 12.5    //达妙4340速度最大值rad/s
#define DM_JOINT_MOTOR_T_MAX 28    //达妙4340输出扭矩最大值 N.m

#define  DM_JOINT_LENGTH 14  //关节长度mm

#endif // BNGU_CONTROL_SYSTEM_ARMORED_CFG_H
