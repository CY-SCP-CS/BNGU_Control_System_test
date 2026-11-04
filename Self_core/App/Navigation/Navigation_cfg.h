//
// Created by 33140 on 2026/11/3.
//

#ifndef BNGU_CONTROL_SYSTEM_NAVIGATION_CFG_H
#define BNGU_CONTROL_SYSTEM_NAVIGATION_CFG_H

// 俯视，以 C 板正前方为车头：1 右前，2 左后，3 右后，4 左前。
#define NAV_MOTOR_DRIVE_FL_FEEDBACKID 0x204  // 左前，4 号电机
#define NAV_MOTOR_DRIVE_RL_FEEDBACKID 0x202  // 左后，2 号电机
#define NAV_MOTOR_DRIVE_FR_FEEDBACKID 0x201  // 右前，1 号电机
#define NAV_MOTOR_DRIVE_RR_FEEDBACKID 0x203  // 右后，3 号电机
#define NAV_M3508_SEND_ID 0x200  // 1～4 号 3508 电机共用电流指令 ID

#define NAV_DRIVE_MAX_OMEGA_RAD_S 735.0f  // 电机最大转子角速度，rad/s，约对应减速前 7000 rpm
#define NAV_CHASSIS_CTRL_DT_S 0.005f     // 速度环控制周期，5 ms，与旧导航车工程一致

// 来源：000-旧黑色步兵逆向重建模型总装.SLDASM，单位均为 mm。
// 轮中心取两排滚子的整体宽度中面；半径取滚子外缘相对轮轴的径向极值。
#define NAV_MOTOR_DRIVE_WHEEL_RADIUS 76.5f  // 模型轮缘名义半径
// 暂按模型 X/Z 方向分别记为轴距/轮距；车头若沿 Z，交换这两项，旋转系数不变。
#define NAV_CHASSIS_WHEELBASE_MM 383.689789f    // 模型 X 方向完整轮中心距
#define NAV_CHASSIS_TRACK_WIDTH_MM 379.597149f  // 模型 Z 方向完整轮中心距
// 半轴距 + 半轮距；45° 投影在逆解算中计算。
#define NAV_CHASSIS_ROTATION_COEFF_MM ((NAV_CHASSIS_WHEELBASE_MM + NAV_CHASSIS_TRACK_WIDTH_MM) * 0.5f)

#endif // BNGU_CONTROL_SYSTEM_NAVIGATION_CFG_H
