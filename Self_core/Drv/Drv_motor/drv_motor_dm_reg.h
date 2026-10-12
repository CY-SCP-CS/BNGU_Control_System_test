//
// Created by 33140 on 2026/10/7.
//

#ifndef BNGU_CONTROL_SYSTEM_DRV_MOTOR_DM_REG_H
#define BNGU_CONTROL_SYSTEM_DRV_MOTOR_DM_REG_H

/**
 * @brief 达妙寄存器地址，按 DM-J4340-2EC V1.1 使用说明书 V1.3（2026-08-13）第 18～20 页整理。
 * @note 枚举值为寄存器地址，不是寄存器内的参数值。
 *       RW：可读写；RO：只读。注释中的 float、uint32 是寄存器数据类型。
 *       范围沿用说明书：圆括号不含端点，方括号包含端点；fmax、TBD 保留原文。
 *       “—”表示表中未给出范围；未列出的地址不作定义。
 */
typedef enum {
    DRV_MOTOR_DM_REG_UV_VALUE = 0x00U, // 低压保护值；RW；float；范围 (10.0,fmax]
    DRV_MOTOR_DM_REG_KT_VALUE = 0x01U, // 扭矩系数；RW；float；范围 [0.0,fmax]
    DRV_MOTOR_DM_REG_OT_VALUE = 0x02U, // 过温保护值；RW；float；范围 [80.0,200)
    DRV_MOTOR_DM_REG_OC_VALUE = 0x03U, // 过流保护值；RW；float；范围 (0.0,1.0)
    DRV_MOTOR_DM_REG_ACC = 0x04U, // 加速度；RW；float；范围 (0.0,fmax)
    DRV_MOTOR_DM_REG_DEC = 0x05U, // 减速度；RW；float；范围 [-fmax,0.0)
    DRV_MOTOR_DM_REG_MAX_SPD = 0x06U, // 最大速度；RW；float；范围 (0.0,fmax]
    DRV_MOTOR_DM_REG_MST_ID = 0x07U, // 反馈 ID；RW；uint32；范围 [0,0x7FF]
    DRV_MOTOR_DM_REG_ESC_ID = 0x08U, // 接收 ID；RW；uint32；范围 [0,0x7FF]
    DRV_MOTOR_DM_REG_TIMEOUT = 0x09U, // 超时警报时间；RW；uint32；范围 [0,2^32-1]
    DRV_MOTOR_DM_REG_CTRL_MODE = 0x0AU, // 控制模式；RW；uint32；范围 [0,4]
    DRV_MOTOR_DM_REG_DAMP = 0x0BU, // 电机粘滞系数；RO；float；范围 —
    DRV_MOTOR_DM_REG_INERTIA = 0x0CU, // 电机转动惯量；RO；float；范围 —
    DRV_MOTOR_DM_REG_HW_VER = 0x0DU, // 保留；RO；uint32；范围 —
    DRV_MOTOR_DM_REG_SW_VER = 0x0EU, // 软件版本号；RO；uint32；范围 —
    DRV_MOTOR_DM_REG_SN = 0x0FU, // 保留；RO；uint32；范围 —
    DRV_MOTOR_DM_REG_NPP = 0x10U, // 电机极对数；RO；uint32；范围 —
    DRV_MOTOR_DM_REG_RS = 0x11U, // 电机相电阻；RO；float；范围 —
    DRV_MOTOR_DM_REG_LS = 0x12U, // 电机相电感；RO；float；范围 —
    DRV_MOTOR_DM_REG_FLUX = 0x13U, // 电机磁链值；RO；float；范围 —
    DRV_MOTOR_DM_REG_GR = 0x14U, // 齿轮减速比；RO；float；范围 —
    DRV_MOTOR_DM_REG_PMAX = 0x15U, // 位置映射范围；RW；float；范围 (0.0,fmax]
    DRV_MOTOR_DM_REG_VMAX = 0x16U, // 速度映射范围；RW；float；范围 (0.0,fmax]
    DRV_MOTOR_DM_REG_TMAX = 0x17U, // 扭矩映射范围；RW；float；范围 (0.0,fmax]
    DRV_MOTOR_DM_REG_I_BW = 0x18U, // 电流环控制带宽；RW；float；范围 [100.0,1.0e4]
    DRV_MOTOR_DM_REG_KP_ASR = 0x19U, // 速度环 Kp；RW；float；范围 [0.0,fmax]
    DRV_MOTOR_DM_REG_KI_ASR = 0x1AU, // 速度环 Ki；RW；float；范围 [0.0,fmax]
    DRV_MOTOR_DM_REG_KP_APR = 0x1BU, // 位置环 Kp；RW；float；范围 [0.0,fmax]
    DRV_MOTOR_DM_REG_KI_APR = 0x1CU, // 位置环 Ki；RW；float；范围 [0.0,fmax]
    DRV_MOTOR_DM_REG_OV_VALUE = 0x1DU, // 过压保护值；RW；float；范围 TBD
    DRV_MOTOR_DM_REG_GREF = 0x1EU, // 齿轮力矩效率；RW；float；范围 (0.0,1.0]
    DRV_MOTOR_DM_REG_DETA = 0x1FU, // 速度环阻尼系数；RW；float；范围 [1.0,30.0]
    DRV_MOTOR_DM_REG_V_BW = 0x20U, // 速度环滤波带宽；RW；float；范围 (0.0,500.0)
    DRV_MOTOR_DM_REG_IQ_C1 = 0x21U, // 电流环增强系数；RW；float；范围 [100.0,1.0e4]
    DRV_MOTOR_DM_REG_VL_C1 = 0x22U, // 速度环增强系数；RW；float；范围 (0.0,1.0e4]
    DRV_MOTOR_DM_REG_CAN_BR = 0x23U, // CAN 波特率代码；RW；uint32；范围 [0,9]
    DRV_MOTOR_DM_REG_SUB_VER = 0x24U, // 子版本号；RO；uint32；范围 —
    DRV_MOTOR_DM_REG_BOOT_VER = 0x25U, // Boot 版本号；RO；uint32；范围 —

    DRV_MOTOR_DM_REG_DIR = 0x37U, // 方向；RO；float；范围 —
    DRV_MOTOR_DM_REG_M_OFF = 0x38U, // 电机侧角度偏移；RO；float；范围 —
    DRV_MOTOR_DM_REG_IMAX = 0x3BU, // 驱动板最大电流；RO；float；范围 —
    DRV_MOTOR_DM_REG_VBUS = 0x3CU, // 电源电压；RO；float；范围 —
    DRV_MOTOR_DM_REG_TPCB = 0x3DU, // 驱动温度；RO；float；范围 —
    DRV_MOTOR_DM_REG_TMTR = 0x3EU, // 电机线圈温度；RO；float；范围 —
    DRV_MOTOR_DM_REG_I_U_OFF = 0x3FU, // U 相电流偏置；RO；float；范围 —
    DRV_MOTOR_DM_REG_I_V_OFF = 0x40U, // V 相电流偏置；RO；float；范围 —
    DRV_MOTOR_DM_REG_I_W_OFF = 0x41U, // W 相电流偏置；RO；float；范围 —
    DRV_MOTOR_DM_REG_P_M = 0x50U, // 电机当前位置；RO；float；范围 —
    DRV_MOTOR_DM_REG_XOUT = 0x51U, // 输出轴位置；RO；float；范围 —

} drv_motor_dm_reg_t;

/* 说明书表下注：转子折算到输出轴的位置及输出轴编码器计算的位置，单位均为 rad。 */

#endif // BNGU_CONTROL_SYSTEM_DRV_MOTOR_DM_REG_H
