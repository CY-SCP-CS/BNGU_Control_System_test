/**
 * @file    drv_motor.c
 * @brief   电机协议层实现
 */
#include "drv_motor.h"
#include "lib_math.h"

#include <float.h>
#include <math.h>
#include <string.h>

// ─── 接口实现 ─────────────────────────────────────

void drv_motor_solve_dji(const uint8_t *data, drv_motor_data_t *cur)
{
    cur->angle       = (uint16_t)data[0] << 8 | data[1];
    cur->speed       = (int16_t)((uint16_t)data[2] << 8 | data[3]);
    cur->current     = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    cur->temperature = data[6];//大端序
}

void drv_motor_reset_frame(uint8_t *frame)
{
    memset(frame, 0, 8);
}

void drv_motor_set_dji(uint8_t *frame, uint8_t slot,
                                   int16_t current)
{
    if (slot >= DRV_MOTOR_DJI_FRAME_MAX) return;
    frame[slot * 2]     = LIB_HI_BYTE(current);
    frame[slot * 2 + 1] = LIB_LO_BYTE(current);
}

void drv_motor_solve_lk(const uint8_t *data, drv_motor_data_t *cur)
{
    cur->cmd_id      = data[0];
    cur->temperature = data[1];
    cur->current     = (int16_t)((uint16_t)data[2] | (uint16_t)data[3] << 8);
    cur->speed       = (int16_t)((uint16_t)data[4] | (uint16_t)data[5] << 8);
    cur->angle       = (uint16_t)data[6] | (uint16_t)data[7] << 8;//小端序
}

void drv_motor_get_lk(uint8_t *frame)
{
    memset(frame, 0, 8);
    frame[0] = 0x9C;
}

void drv_motor_set_lk(uint8_t *frame, int16_t current)
{
    memset(frame, 0, 8);
    frame[0] = 0xA1;
    frame[4] = LIB_LO_BYTE(current);
    frame[5] = LIB_HI_BYTE(current);
}

void drv_motor_solve_dm(const uint8_t *data, drv_motor_dm_t *cur)
{
    if (!data || !cur) {
        return;
    }
    if (!isfinite(cur->config.p_max) || cur->config.p_max <= 0.0f
        || !isfinite(cur->config.v_max) || cur->config.v_max <= 0.0f
        || !isfinite(cur->config.t_max) || cur->config.t_max <= 0.0f) {
        return;
    }

    /* 位置占 16 位；速度和扭矩各占 12 位，共用 D[4]。 */
    uint16_t position_raw = ((uint16_t)data[1] << 8) | data[2];
    uint16_t velocity_raw = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    uint16_t torque_raw = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];

    /* 无符号编码线性映射到 [-max, max] */
    cur->feedback.POS = ((float)position_raw / 65535.0f * 2.0f - 1.0f)
                      * cur->config.p_max;
    cur->feedback.VEL = ((float)velocity_raw / 4095.0f * 2.0f - 1.0f)
                      * cur->config.v_max;
    cur->feedback.TOR = ((float)torque_raw / 4095.0f * 2.0f - 1.0f)
                      * cur->config.t_max;
    cur->feedback.err = (drv_motor_dm_err_status_t)(data[0] >> 4);
    cur->feedback.T_MOS = data[6];
    cur->feedback.T_Rotor = data[7];
}

uint8_t drv_motor_init_dm(drv_motor_dm_t *motor,drv_motor_dm_mode_t mode,
           uint16_t can_id , uint16_t Master_id ,float p_max, float v_max, float t_max)
{
    if (!motor || can_id > 0x7FFU || Master_id > 0x7FFU
        || !isfinite(p_max) || p_max <= 0.0f
        || !isfinite(v_max) || v_max <= 0.0f
        || !isfinite(t_max) || t_max <= 0.0f) {
        return 0U;
    }
    switch (mode) {
        case DRV_MOTOR_DM_MODE_MIT: break;
        case DRV_MOTOR_DM_MODE_POSITION_VELOCITY: break;
        case DRV_MOTOR_DM_MODE_VELOCITY: break;
        case DRV_MOTOR_DM_MODE_PVT: break;
        default: return 0U;
    }

    memset(motor, 0, sizeof(*motor));
    motor->config.p_max = p_max;
    motor->config.v_max = v_max;
    motor->config.t_max = t_max;
    motor ->config.cmd_id = can_id;
    motor->config.Master_id = Master_id;
    motor->config.mode = mode;
    return 1U;
}

uint8_t drv_motor_dm_MIT_frame_config(uint8_t *frame,
                                    const drv_motor_dm_config_t *config,
                                    float p_des, float v_des,
                                    float t_ff, float kp, float kd)
{
    if (!frame || !config
        || config->mode != DRV_MOTOR_DM_MODE_MIT
        || !isfinite(config->p_max) || config->p_max <= 0.0f
        || !isfinite(config->v_max) || config->v_max <= 0.0f
        || !isfinite(config->t_max) || config->t_max <= 0.0f
        || !isfinite(p_des) || !isfinite(v_des) || !isfinite(t_ff)
        || !isfinite(kp) || !isfinite(kd)) {
        return 0U;
    }

    uint16_t position = lib_encode_symmetric(p_des, config->p_max, 65535U);
    uint16_t velocity = lib_encode_symmetric(v_des, config->v_max, 4095U);
    uint16_t torque = lib_encode_symmetric(t_ff, config->t_max, 4095U);
    uint16_t kp_raw = lib_encode_unsigned(kp, 500.0f, 4095U);
    uint16_t kd_raw = lib_encode_unsigned(kd, 5.0f, 4095U);

    frame[0] = (uint8_t)(position >> 8);
    frame[1] = (uint8_t)position;
    frame[2] = (uint8_t)(velocity >> 4);
    frame[3] = (uint8_t)(((velocity & 0x0FU) << 4) | (kp_raw >> 8));
    frame[4] = (uint8_t)kp_raw;
    frame[5] = (uint8_t)(kd_raw >> 4);
    frame[6] = (uint8_t)(((kd_raw & 0x0FU) << 4) | (torque >> 8));
    frame[7] = (uint8_t)torque;
    return 1U;
}

uint8_t drv_motor_dm_command_frame(uint8_t *frame, drv_motor_dm_cmd_t command)
{
    if (!frame) return 0U;

    switch (command) {
    case DRV_MOTOR_DM_CMD_ENABLE:
    case DRV_MOTOR_DM_CMD_DISABLE:
    case DRV_MOTOR_DM_CMD_SAVE_ZERO:
    case DRV_MOTOR_DM_CMD_CLEAR_ERROR:
        break;
    default:
        return 0U;
    }

    for (uint8_t i = 0U; i < 7U; i++) {frame[i] = 0xFF;}
    frame[7U] = (uint8_t)command;
    return 1U;
}

uint8_t drv_motor_dm_position_velocity_frame_config(uint8_t *frame,
        const drv_motor_dm_config_t *config,float p_des,float v_des)
{
    if (!frame || !config
       || config->mode != DRV_MOTOR_DM_MODE_POSITION_VELOCITY
       || !isfinite(config->p_max) || config->p_max <= 0.0f
       || !isfinite(config->v_max) || config->v_max <= 0.0f
       || !isfinite(p_des) || !isfinite(v_des)) {
        return 0U;
    }

    /* 先在物理量上限幅，再复制浮点数的原始字节。 */
    if (p_des > config->p_max) p_des = config->p_max;
    else if (p_des < -config->p_max) p_des = -config->p_max;

    if (v_des > config->v_max) v_des = config->v_max;
    else if (v_des < -config->v_max) v_des = -config->v_max;

    memcpy(frame,     &p_des, sizeof(p_des));
    memcpy(frame + 4, &v_des, sizeof(v_des));

    return 1U;
}

uint8_t drv_motor_dm_velocity_frame_config(uint8_t *frame,
          const drv_motor_dm_config_t *config,float v_des)
{
    if (!frame || !config
       || config->mode != DRV_MOTOR_DM_MODE_VELOCITY
       || !isfinite(config->v_max) || config->v_max <= 0.0f
       || !isfinite(v_des)) {
        return 0U;}

    if (v_des > config->v_max) v_des = config->v_max;
    else if (v_des < -config->v_max) v_des = -config->v_max;

    memcpy(frame, &v_des, sizeof(v_des));

    return 1U;
}

uint8_t drv_motor_dm_PVT_frame_config(uint8_t *frame,const drv_motor_dm_config_t *config,
    float p_des, uint16_t v_des,uint16_t i_des)
{
    if (!frame || !config
    || config->mode != DRV_MOTOR_DM_MODE_PVT
    || !isfinite(config->p_max) || config->p_max <= 0.0f
    || !isfinite(p_des)) {
        return 0U;
    }

    if (p_des > config->p_max) p_des = config->p_max;
    else if (p_des < -config->p_max) p_des = -config->p_max;
    if (v_des > 10000U) v_des = 10000U;
    if (i_des > 10000U) i_des = 10000U;

    memcpy(frame, &p_des, sizeof(p_des));
    memcpy(frame + 4, &v_des, sizeof(v_des));
    memcpy(frame + 6, &i_des, sizeof(i_des));

    return 1U;
}

uint8_t drv_motor_dm_read_reg_frame_config(uint8_t *frame,uint16_t cmd_id,const drv_motor_dm_reg_t reg)
{
    uint8_t valid =
       (reg >= DRV_MOTOR_DM_REG_UV_VALUE
        && reg <= DRV_MOTOR_DM_REG_BOOT_VER)
       || reg == DRV_MOTOR_DM_REG_DIR
       || reg == DRV_MOTOR_DM_REG_M_OFF
       || (reg >= DRV_MOTOR_DM_REG_IMAX
           && reg <= DRV_MOTOR_DM_REG_I_W_OFF)
       || reg == DRV_MOTOR_DM_REG_P_M
       || reg == DRV_MOTOR_DM_REG_XOUT;

    if (!valid || !frame || cmd_id > 0x7FFU){return 0U;}

    memcpy(frame, &cmd_id, sizeof(cmd_id));
    frame[2] = 0x33;
    memcpy(frame + 3, &reg, 1);
    return 1U;

}


/* 寄存器数据类型及地址查找表；按 V1.3 说明书寄存器表整理。 */
static const drv_motor_dm_reg_info_t s_reg_table[] = {
    {DRV_MOTOR_DM_REG_UV_VALUE, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 低压保护值
    {DRV_MOTOR_DM_REG_KT_VALUE, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 扭矩系数
    {DRV_MOTOR_DM_REG_OT_VALUE, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 过温保护值
    {DRV_MOTOR_DM_REG_OC_VALUE, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 过流保护值
    {DRV_MOTOR_DM_REG_ACC, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 加速度
    {DRV_MOTOR_DM_REG_DEC, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 减速度
    {DRV_MOTOR_DM_REG_MAX_SPD, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 最大速度
    {DRV_MOTOR_DM_REG_MST_ID, DRV_MOTOR_DM_REG_TYPE_UINT32}, // 反馈 ID
    {DRV_MOTOR_DM_REG_ESC_ID, DRV_MOTOR_DM_REG_TYPE_UINT32}, // 接收 ID
    {DRV_MOTOR_DM_REG_TIMEOUT, DRV_MOTOR_DM_REG_TYPE_UINT32}, // 超时警报时间
    {DRV_MOTOR_DM_REG_CTRL_MODE, DRV_MOTOR_DM_REG_TYPE_UINT32}, // 控制模式
    {DRV_MOTOR_DM_REG_DAMP, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 电机粘滞系数
    {DRV_MOTOR_DM_REG_INERTIA, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 电机转动惯量
    {DRV_MOTOR_DM_REG_HW_VER, DRV_MOTOR_DM_REG_TYPE_UINT32}, // 保留
    {DRV_MOTOR_DM_REG_SW_VER, DRV_MOTOR_DM_REG_TYPE_UINT32}, // 软件版本号
    {DRV_MOTOR_DM_REG_SN, DRV_MOTOR_DM_REG_TYPE_UINT32}, // 保留
    {DRV_MOTOR_DM_REG_NPP, DRV_MOTOR_DM_REG_TYPE_UINT32}, // 电机极对数
    {DRV_MOTOR_DM_REG_RS, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 电机相电阻
    {DRV_MOTOR_DM_REG_LS, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 电机相电感
    {DRV_MOTOR_DM_REG_FLUX, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 电机磁链值
    {DRV_MOTOR_DM_REG_GR, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 齿轮减速比
    {DRV_MOTOR_DM_REG_PMAX, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 位置映射范围
    {DRV_MOTOR_DM_REG_VMAX, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 速度映射范围
    {DRV_MOTOR_DM_REG_TMAX, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 扭矩映射范围
    {DRV_MOTOR_DM_REG_I_BW, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 电流环控制带宽
    {DRV_MOTOR_DM_REG_KP_ASR, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 速度环 Kp
    {DRV_MOTOR_DM_REG_KI_ASR, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 速度环 Ki
    {DRV_MOTOR_DM_REG_KP_APR, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 位置环 Kp
    {DRV_MOTOR_DM_REG_KI_APR, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 位置环 Ki
    {DRV_MOTOR_DM_REG_OV_VALUE, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 过压保护值
    {DRV_MOTOR_DM_REG_GREF, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 齿轮力矩效率
    {DRV_MOTOR_DM_REG_DETA, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 速度环阻尼系数
    {DRV_MOTOR_DM_REG_V_BW, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 速度环滤波带宽
    {DRV_MOTOR_DM_REG_IQ_C1, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 电流环增强系数
    {DRV_MOTOR_DM_REG_VL_C1, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 速度环增强系数
    {DRV_MOTOR_DM_REG_CAN_BR, DRV_MOTOR_DM_REG_TYPE_UINT32}, // CAN 波特率代码
    {DRV_MOTOR_DM_REG_SUB_VER, DRV_MOTOR_DM_REG_TYPE_UINT32}, // 子版本号
    {DRV_MOTOR_DM_REG_BOOT_VER, DRV_MOTOR_DM_REG_TYPE_UINT32}, // Boot 版本号
    {DRV_MOTOR_DM_REG_DIR, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 方向
    {DRV_MOTOR_DM_REG_M_OFF, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 电机侧角度偏移
    {DRV_MOTOR_DM_REG_IMAX, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 驱动板最大电流
    {DRV_MOTOR_DM_REG_VBUS, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 电源电压
    {DRV_MOTOR_DM_REG_TPCB, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 驱动温度
    {DRV_MOTOR_DM_REG_TMTR, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 电机线圈温度
    {DRV_MOTOR_DM_REG_I_U_OFF, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // U 相电流偏置
    {DRV_MOTOR_DM_REG_I_V_OFF, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // V 相电流偏置
    {DRV_MOTOR_DM_REG_I_W_OFF, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // W 相电流偏置
    {DRV_MOTOR_DM_REG_P_M, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 电机当前位置
    {DRV_MOTOR_DM_REG_XOUT, DRV_MOTOR_DM_REG_TYPE_FLOAT}, // 输出轴位置
};

uint8_t drv_motor_dm_read_reg_response_parse(drv_motor_dm_reg_info_t* reg_info,
                                                   const uint8_t *data , uint8_t len)
{
    if ( !reg_info || len != 8U || !data || data[2] != 0x33U) return 0U;

    for (size_t i = 0; i < sizeof(s_reg_table) / sizeof(s_reg_table[0]); i++) {
        if (data[3] == s_reg_table[i].reg) {
            reg_info->reg = data[3];
            if (s_reg_table[i].type == DRV_MOTOR_DM_REG_TYPE_FLOAT) {
                reg_info->type = s_reg_table[i].type;
                memcpy(&reg_info->value.float_value, data + 4, sizeof(float));
                return 1U;
            }
            else if (s_reg_table[i].type == DRV_MOTOR_DM_REG_TYPE_UINT32){
                reg_info->type = s_reg_table[i].type;
                memcpy(&reg_info->value.u32_value, data + 4, sizeof(uint32_t));
                return 1U;
            }
            break;
        }
    }
    return 0U;
}

/* 按寄存器的声明类型读取联合体；只接受当前列表中可写且数值合法的参数。 */
static uint8_t dm_write_reg_value_is_valid(drv_motor_dm_reg_t reg,
                                         drv_motor_dm_reg_type_t type,
                                         drv_motor_dm_w_reg_value_t value)
{
    if (type == DRV_MOTOR_DM_REG_TYPE_UINT32) {
        switch (reg) {
        case DRV_MOTOR_DM_REG_MST_ID:
        case DRV_MOTOR_DM_REG_ESC_ID:
            return value.u32_value <= 0x7FFU;
        case DRV_MOTOR_DM_REG_TIMEOUT:
            return 1U; // 完整 uint32_t 范围，无需额外限幅。
        case DRV_MOTOR_DM_REG_CTRL_MODE:
            return value.u32_value <= 4U;
        case DRV_MOTOR_DM_REG_CAN_BR:
            return value.u32_value <= 9U;
        default:
            return 0U;
        }
    }

    if (type != DRV_MOTOR_DM_REG_TYPE_FLOAT || !isfinite(value.float_value)) {
        return 0U;
    }

    float number = value.float_value;
    /* fmax 按 float 最大有限值 FLT_MAX 处理；圆括号端点不允许写入。 */
    switch (reg) {
    case DRV_MOTOR_DM_REG_UV_VALUE:
        return number > 10.0f;
    case DRV_MOTOR_DM_REG_KT_VALUE:
    case DRV_MOTOR_DM_REG_KP_ASR:
    case DRV_MOTOR_DM_REG_KI_ASR:
    case DRV_MOTOR_DM_REG_KP_APR:
    case DRV_MOTOR_DM_REG_KI_APR:
        return number >= 0.0f;
    case DRV_MOTOR_DM_REG_OT_VALUE:
        return number >= 80.0f && number < 200.0f;
    case DRV_MOTOR_DM_REG_OC_VALUE:
        return number > 0.0f && number < 1.0f;
    case DRV_MOTOR_DM_REG_ACC:
        return number > 0.0f && number < FLT_MAX;
    case DRV_MOTOR_DM_REG_DEC:
        return number < 0.0f;
    case DRV_MOTOR_DM_REG_MAX_SPD:
    case DRV_MOTOR_DM_REG_PMAX:
    case DRV_MOTOR_DM_REG_VMAX:
    case DRV_MOTOR_DM_REG_TMAX:
        return number > 0.0f;
    case DRV_MOTOR_DM_REG_I_BW:
    case DRV_MOTOR_DM_REG_IQ_C1:
        return number >= 100.0f && number <= 10000.0f;
    case DRV_MOTOR_DM_REG_OV_VALUE:
        return 1U; // 说明书范围为 TBD，仅检查有限值。
    case DRV_MOTOR_DM_REG_GREF:
        return number > 0.0f && number <= 1.0f;
    case DRV_MOTOR_DM_REG_DETA:
        return number >= 1.0f && number <= 30.0f;
    case DRV_MOTOR_DM_REG_V_BW:
        return number > 0.0f && number < 500.0f;
    case DRV_MOTOR_DM_REG_VL_C1:
        return number > 0.0f && number <= 10000.0f;
    default:
        return 0U;
    }
}

uint8_t drv_motor_dm_write_reg_frame_config(uint8_t *frame ,uint16_t cmd_id,
                    const drv_motor_dm_reg_t reg,drv_motor_dm_w_reg_value_t value)
{
    uint8_t valid =
    (reg >= DRV_MOTOR_DM_REG_UV_VALUE
     && reg <= DRV_MOTOR_DM_REG_CTRL_MODE)
    || (reg >= DRV_MOTOR_DM_REG_PMAX
        && reg <= DRV_MOTOR_DM_REG_CAN_BR);

    if (!valid || !frame || cmd_id > 0x7FFU) {return 0U;}

    for (size_t i = 0; i < sizeof(s_reg_table) / sizeof(s_reg_table[0]); i++) {
        if (reg == s_reg_table[i].reg) {
            if (!dm_write_reg_value_is_valid(reg, s_reg_table[i].type, value)) {
                return 0U;
            }
            /* 所有检查通过后才修改输出；电机 ID 按小端顺序写入。 */
            frame[0] = (uint8_t)cmd_id;
            frame[1] = (uint8_t)(cmd_id >> 8);
            frame[2] = 0x55U;
            frame[3] = (uint8_t)reg;
            if (s_reg_table[i].type == DRV_MOTOR_DM_REG_TYPE_FLOAT) {
                memcpy(frame + 4, &value.float_value, sizeof(float));
                return 1U;
            } else if (s_reg_table[i].type == DRV_MOTOR_DM_REG_TYPE_UINT32) {
                memcpy(frame + 4, &value.u32_value, sizeof(uint32_t));
                return 1U;
            }
            else return 0U;
        }
    }

    return 0U;
}

uint8_t drv_motor_dm_write_reg_response_parse(drv_motor_dm_reg_info_t *reg_info,
                                                const uint8_t *data,uint8_t len)
{
    if ( !reg_info || len != 8U || !data || data[2] != 0x55U) return 0U;

    for (size_t i = 0; i < sizeof(s_reg_table) / sizeof(s_reg_table[0]); i++) {
        if (data[3] == s_reg_table[i].reg) {
            reg_info->reg = data[3];
            if (s_reg_table[i].type == DRV_MOTOR_DM_REG_TYPE_FLOAT) {
                reg_info->type = s_reg_table[i].type;
                memcpy(&reg_info->value.float_value, data + 4, sizeof(float));
                return 1U;
            }
            else if (s_reg_table[i].type == DRV_MOTOR_DM_REG_TYPE_UINT32){
                reg_info->type = s_reg_table[i].type;
                memcpy(&reg_info->value.u32_value, data + 4, sizeof(uint32_t));
                return 1U;
            }
            break;
        }
    }
    return 0U;

}

uint8_t drv_motor_dm_save_params_frame_config(uint8_t *frame, uint16_t cmd_id)
{
    if (! frame || cmd_id > 0x7FFU) {return 0U;}
    /* STM32 为小端：D[0] 为 CANID_L，D[1] 为 CANID_H。 */
    memcpy(frame, &cmd_id, 2);
    frame[2] = 0xAA; // 保存参数命令字。
    frame[3] = 0x01; // 协议固定值。
    return 1U;
}

uint8_t drv_motor_dm_save_params_response_parse(uint16_t cmd_id,
                                                const uint8_t *data, uint8_t len)
{
    uint16_t response_cmd_id;

    if (!data || len != 4U || cmd_id > 0x7FFU) {
        return 0U;
    }
    if (data[2] != 0xAAU || data[3] != 0x01U) {
        return 0U;
    }

    /* 保存成功回复回显目标电机 CAN ID，低字节在前。 */
    response_cmd_id = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    return response_cmd_id == cmd_id ? 1U : 0U;
}
