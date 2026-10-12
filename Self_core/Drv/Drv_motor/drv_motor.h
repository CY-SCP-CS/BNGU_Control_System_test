/**
 * @file    drv_motor.h
 * @brief   电机协议层: DJI / 翎控 / 达妙 数据解析与命令构建
 */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "lib_typedef.h"
#include "drv_motor_dm_reg.h"
//翎控电机尚未完整适配；达妙已实现反馈解析及 MIT 控制帧构建。

typedef struct {
    uint8_t  cmd_id;//命令ID
    int16_t  speed;//转速
    uint16_t angle;//编码器值
    uint8_t  temperature;//温度
    int16_t  current;//当前电流
} drv_motor_data_t;

#define DRV_MOTOR_DJI_FRAME_MAX  4   /* 每帧最多 4 路电机 */

/* DJI 电机电流指令范围与减速比。 */
#define DRV_MOTOR_GM6020_CURRENT_MIN       (-16384.0f)
#define DRV_MOTOR_GM6020_CURRENT_MAX         16384.0f
#define DRV_MOTOR_M3508_CURRENT_MIN        (-16384.0f)
#define DRV_MOTOR_M3508_CURRENT_MAX          16384.0f
#define DRV_MOTOR_M2006_CURRENT_MIN        (-10000.0f)
#define DRV_MOTOR_M2006_CURRENT_MAX          10000.0f
#define DRV_MOTOR_M3508_REDUCTION         19.0f

//达妙电机信息
typedef enum {
    DRV_MOTOR_DM_STATUS_DISABLED          = 0x0, // 失能
   DRV_MOTOR_DM_STATUS_ENABLED           = 0x1, // 使能
   DRV_MOTOR_DM_STATUS_OVERVOLTAGE       = 0x8, // 超压
   DRV_MOTOR_DM_STATUS_UNDERVOLTAGE      = 0x9, // 欠压
   DRV_MOTOR_DM_STATUS_OVERCURRENT       = 0xA, // 过电流
   DRV_MOTOR_DM_STATUS_MOS_OVERHEAT      = 0xB, // MOS 过温
   DRV_MOTOR_DM_STATUS_WINDING_OVERHEAT  = 0xC, // 电机线圈过温
   DRV_MOTOR_DM_STATUS_COMMUNICATION_LOST = 0xD, // 通信丢失
   DRV_MOTOR_DM_STATUS_OVERLOAD          = 0xE  // 过载
}drv_motor_dm_err_status_t; //反馈状态码

typedef enum {
    DRV_MOTOR_DM_MODE_MIT               = 1, // MIT 模式
    DRV_MOTOR_DM_MODE_POSITION_VELOCITY = 2, // 位置速度模式
    DRV_MOTOR_DM_MODE_VELOCITY          = 3, // 速度模式
    DRV_MOTOR_DM_MODE_PVT               = 4  // PVT 模式 (我猜是力位混控模式)
} drv_motor_dm_mode_t; //控制模式

/* 寄存器回复的数据类型及解析结果。 */
typedef enum {
    DRV_MOTOR_DM_REG_TYPE_FLOAT,
    DRV_MOTOR_DM_REG_TYPE_UINT32
} drv_motor_dm_reg_type_t;

/* 写寄存器时，调用者按寄存器类型填写对应成员。 */
typedef union {
    float float_value;
    uint32_t u32_value;
} drv_motor_dm_w_reg_value_t;

typedef struct {
    drv_motor_dm_reg_t reg;       // 最近一次成功解析的寄存器地址
    drv_motor_dm_reg_type_t type; // 决定应读取哪个联合体成员
    union {
        float float_value;       // type 为 FLOAT 时读取
        uint32_t u32_value;      // type 为 UINT32 时读取
    } value;
} drv_motor_dm_reg_info_t;

typedef struct {
    uint16_t  cmd_id; //使用者配置的发送 CAN ID
    uint16_t Master_id; //使用者配置的反馈 CAN ID，解析函数不修改
    float p_max; // 位置范围参数，rad
    float v_max; // 速度范围参数，rad/s
    float t_max; // 扭矩范围参数，N·m
    drv_motor_dm_mode_t mode;
} drv_motor_dm_config_t; //配置参数

typedef struct {
    float POS;  //反馈位置，rad
    float VEL;  //反馈速度，rad/s
    float TOR; //反馈扭矩，N·m
    int16_t T_MOS; //驱动上 MOS 的平均温度，℃
    int16_t T_Rotor; //电机内部线圈的平均温度，℃
    drv_motor_dm_err_status_t err; //电机状态
}drv_motor_dm_feedback_data_t;  //反馈数据

typedef struct {
    drv_motor_dm_config_t config;
    drv_motor_dm_feedback_data_t feedback;
    drv_motor_dm_reg_info_t cur_reg_info; //最近一次电机返回的寄存器信息
}drv_motor_dm_t; //达妙电机结构体

typedef enum {
    DRV_MOTOR_DM_CMD_ENABLE     = 0xFC, // 使能
    DRV_MOTOR_DM_CMD_DISABLE    = 0xFD, // 失能
    DRV_MOTOR_DM_CMD_SAVE_ZERO  = 0xFE, // 保存位置零点
    DRV_MOTOR_DM_CMD_CLEAR_ERROR = 0xFB // 清除错误
} drv_motor_dm_cmd_t;  //特殊命令类型，枚举值对应数据场 D[7]



//外部接口
/**
 * @brief  解析 DJI 电机回传数据
 * @param  data   CAN 数据场 (8 字节, 大端)
 * @param  cur    电机数据结构体指针
 */
void drv_motor_solve_dji(const uint8_t *data, drv_motor_data_t *cur);

/**
 * @brief  重置电流控制帧 (全部置零)
 * @param  frame  输出缓冲区 (8 字节)
 */
void drv_motor_reset_frame(uint8_t *frame);

/**
 * @brief  设置 DJI 电流帧中某一路电流
 * @param  frame    帧缓冲区 (8 字节)
 * @param  slot     槽位 [0..3], 对应帧内位置
 * @param  current  电流值 (大端编码)
 */
void drv_motor_set_dji(uint8_t *frame, uint8_t slot,
                                   int16_t current);

/**
 * @brief  解析翎控电机回传数据
 * @param  data   CAN 数据场 (8 字节, 小端)
 * @param  cur    电机数据结构体指针
 */
void drv_motor_solve_lk(const uint8_t *data, drv_motor_data_t *cur);

/**
 * @brief  构建翎控电机读取命令帧
 * @param  frame  输出缓冲区 (8 字节)
 */
void drv_motor_get_lk(uint8_t *frame);

/**
 * @brief  构建 翎控 电流指令帧 (小端)
 * @param  frame    输出缓冲区 (8 字节)
 * @param  current  电流值
 */
void drv_motor_set_lk(uint8_t *frame, int16_t current);


/**
 * @brief 初始化本地达妙电机实例，清零旧数据并保存传入配置。
 * @param motor 达妙电机实例指针。
 * @param mode 本地控制模式，取 drv_motor_dm_mode_t 中的四种模式之一。
 * @param can_id 标准发送 CAN ID，范围 0～0x7FF，保存到 config.cmd_id。
 * @param Master_id 标准反馈 CAN ID，范围 0～0x7FF，保存到 config.Master_id。
 * @param p_max 位置范围参数，rad，必须为正有限数。
 * @param v_max 速度范围参数，rad/s，必须为正有限数。
 * @param t_max 扭矩范围参数，N·m，必须为正有限数。
 * @return 1 表示成功；0 表示空指针、ID 越界、非法模式或无效范围，原实例保持不变。
 * @note 配置须与驱动器一致；本函数不发送 CAN 指令或使能电机。
 *       成功时清零反馈，并保存模式、两个 ID 和三个范围；不切换实际电机模式。
 *       仅检查标准 CAN ID 位宽，设备保留 ID 及控制模式对应的实际发送 ID 由调用方确认。
 *       应在接收启动前调用，避免与中断中的反馈更新并发执行。
 */
uint8_t drv_motor_init_dm(drv_motor_dm_t *motor,drv_motor_dm_mode_t mode ,
uint16_t can_id , uint16_t Master_id ,float p_max, float v_max, float t_max);

/**
 * @brief  解析达妙反馈，将位置/速度/扭矩映射到物理量。
 * @param  data   至少 8 字节的 CAN 数据场，调用方须检查 DLC。
 * @param  cur    达妙电机实例；config 中的三个正有限范围必须与驱动器一致。
 * @note   映射范围分别为 [-p_max,p_max]、[-v_max,v_max]、[-t_max,t_max]。
 *         空指针或无效范围时不更新反馈；不修改 config 中的 ID 和模式。
 */
void drv_motor_solve_dm(const uint8_t *data, drv_motor_dm_t *cur);

/**
 * @brief 构建达妙 MIT 模式的 8 字节控制数据包，不发送 CAN。
 * @param frame 至少 8 字节的输出缓冲区。
 * @param config 与驱动器一致的位置、速度、扭矩映射范围。config->mode 应当为 MIT
 * @param p_des 目标位置，rad，限幅到 [-p_max, p_max]。
 * @param v_des 目标速度，rad/s，限幅到 [-v_max, v_max]。
 * @param t_ff 前馈扭矩，N·m，限幅到 [-t_max, t_max]。
 * @param kp 比例系数，限幅到 [0, 500]。
 * @param kd 微分系数，限幅到 [0, 5]。
 * @return 1 表示成功；0 表示非 MIT 模式或指针、配置、输入无效，frame 保持不变。
 * @note 编码采用截断量化；发送由调用者使用配置的 config.cmd_id 完成。本函数不设置设备模式；
 * 调用方须确保实际设备一致。发送 ID 为基础 ID
 */
uint8_t drv_motor_dm_MIT_frame_config(uint8_t *frame,
                                    const drv_motor_dm_config_t *config,
                                    float p_des, float v_des,
                                    float t_ff, float kp, float kd);

/**
 * @brief 构建达妙位置速度模式的 8 字节控制数据包，不发送 CAN。
 * @param frame 至少 8 字节的输出缓冲区。
 * @param config 位置、速度范围须为正有限值，模式须为位置速度模式。
 * @param p_des 目标位置，rad，限幅到 [-p_max, p_max]。
 * @param v_des 速度给定(用来限定运动过程中的最大绝对速度值)，rad/s，限幅到 [-v_max, v_max]。
 * @return 1 表示成功；0 表示指针、模式、范围或输入无效，frame 保持不变。
 * @note 使用配置范围进行软件限幅；不修改驱动器参数。发送 ID 为基础 ID + 0x100。
 */
uint8_t drv_motor_dm_position_velocity_frame_config(uint8_t *frame,
       const drv_motor_dm_config_t *config,float p_des,float v_des);

/**
 * @brief 构建达妙速度模式的 4 字节控制数据包，不发送 CAN。
 * @param frame 至少 4 字节的输出缓冲区。
 * @param config 速度范围须为正有限值，模式须为速度模式。
 * @param v_des 目标速度，rad/s，限幅到 [-v_max, v_max]。
 * @return 1 表示成功；0 表示指针、模式、范围或输入无效，frame 保持不变。
 * @note 发送 ID 为基础 ID + 0x200。
 */
uint8_t drv_motor_dm_velocity_frame_config(uint8_t *frame,
          const drv_motor_dm_config_t *config,float v_des);
/**
 * @brief 构建达妙 PVT（力位混控）模式的 8 字节控制数据包，不发送 CAN。
 * @param frame 至少 8 字节的输出缓冲区。
 * @param config 电机配置；mode 须为 DRV_MOTOR_DM_MODE_PVT，p_max 须为正有限值。
 * @param p_des 目标位置，rad，须为有限值，软件限幅到 [-p_max, p_max]。
 * @param v_des 已缩放的速度限幅编码值，即速度幅值（rad/s）乘以 100；
 *              例如 500 表示 5 rad/s, 输入范围 0-10000。
 * @param i_des 已缩放的转矩电流限幅编码值，即电流标幺幅值乘以 10000；
 *              例如 5000 表示最大相电流基准的 50%，输入范围 0-10000。
 * @return 1 表示成功；0 表示空指针、模式错误、p_max 无效或 p_des 非有限值，
 *         失败时 frame 保持不变；输入超出限幅范围时仍构帧成功。
 * @note 调用方须在转换为 uint16_t 前检查输入，避免负数或越界值发生转换。
 *       发送时 DLC 为 8，CAN ID 为基础 ID（config.cmd_id）+ 0x300；
 *       本函数不切换设备模式，调用方须确保电机实际处于 PVT 模式。
 */
uint8_t drv_motor_dm_PVT_frame_config(uint8_t *frame,const drv_motor_dm_config_t *config,
    float p_des, uint16_t v_des,uint16_t i_des);

/**
 * @brief 构建达妙使能、失能、保存零点或清除错误命令的 8 字节数据包。
 * @param frame 至少 8 字节的输出缓冲区。
 * @param command 具体命令，详情见 drv_motor_dm_cmd_t 枚举
 * @return 1 表示构帧成功；0 表示空指针或非法命令，缓冲区保持不变。
 * @note 前 7 字节为 0xFF，D[7] 为命令值；本函数不发送 CAN。
 */
uint8_t drv_motor_dm_command_frame(uint8_t *frame, drv_motor_dm_cmd_t command);

/**
 * @brief 构建达妙电机读寄存器命令的4字节数据包
 * @param frame 至少 4 字节的输出缓冲区
 * @param cmd_id 电机的标准发送 CAN ID
 * @param reg 寄存器地址
 * @return 1 表示构帧成功；0 表示空指针、ID 越界或非法寄存器地址，失败时 frame 保持不变。
 * @note 发送时使用标准数据帧，CAN ID 为 0x7FF，DLC 为 4。
 */
uint8_t drv_motor_dm_read_reg_frame_config(uint8_t *frame,uint16_t cmd_id,const drv_motor_dm_reg_t reg);

/**
 * @brief 解析达妙读取寄存器回复，根据寄存器表选择浮点或无符号整数类型。
 * @param reg_info 输出结果指针；成功时更新寄存器地址、类型和对应联合体成员。
 * @param data 接收数据场；D[3] 为寄存器地址，D[4..7] 为小端 32 位数据。
 * @param len 接收数据场长度，须为 8 字节。
 * @return 1 表示解析成功；0 表示空指针、长度错误、命令字非 0x33 或未知寄存器。
 * @note 当前表项均为支持的类型，检查失败时保留原输出，调用者须检查本次返回值；
 *       返回 1 后，根据 type 读取 value.float_value 或 value.u32_value。
 *       不检查返回数值是否符合寄存器范围，也不发送 CAN 请求。
 */
uint8_t drv_motor_dm_read_reg_response_parse(drv_motor_dm_reg_info_t* reg_info,
                                                   const uint8_t *data , uint8_t len);

/**
 * @brief 构建达妙写寄存器命令的 8 字节数据包。
 * @param frame 至少 8 字节的输出缓冲区。
 * @param cmd_id 目标电机基础 CAN ID，范围 0～0x7FF。
 * @param reg 可写寄存器地址；只读和未知地址被拒绝。
 * @param value 按寄存器类型填写 float_value 或 u32_value，函数不做类型转换。
 * @return 1 表示构帧成功；0 表示参数、类型或数值范围不合法，失败时 frame 保持不变。
 * @note 浮点数拒绝 NaN 和无穷大，按当前寄存器表检查范围；fmax 按 FLT_MAX 处理。
 *       不对非法值限幅；发送时使用标准数据帧，CAN ID 为 0x7FF，DLC 为 8。本函数不发送 CAN。
 */
uint8_t drv_motor_dm_write_reg_frame_config(uint8_t *frame, uint16_t cmd_id,
                                          const drv_motor_dm_reg_t reg,
                                          drv_motor_dm_w_reg_value_t value);

/**
 * @brief 解析达妙写寄存器回复，根据寄存器表选择浮点或无符号整数类型。
 * @param reg_info 输出结果指针；成功时更新寄存器地址、类型和对应联合体成员。
 * @param data 接收数据场，至少 8 字节；
 * @param len 接收数据场长度，须为 8 字节。
 * @return 1 表示解析成功；0 表示空指针、长度错误、命令字错误或寄存器地址/类型不支持。
 * @note 调用者须检查返回值，成功后根据 type 读取 value.float_value 或 value.u32_value。
 *       本函数不检查数值范围、不比较本次写入请求，也不发送 CAN。
 */
uint8_t drv_motor_dm_write_reg_response_parse(drv_motor_dm_reg_info_t *reg_info,
                                                const uint8_t *data,uint8_t len);

/**
 * @brief 构建达妙保存全部参数到电机 Flash 的 4 字节命令数据包。
 * @param frame 至少 4 字节的输出缓冲区，仅写入 D[0..3]。
 * @param cmd_id 目标电机基础 CAN ID，范围 0～0x7FF。
 * @return 1 表示构帧成功；0 表示空指针或 ID 越界，失败时 frame 保持不变。
 * @note 报文 ID 为 0x7FF，DLC 为 4。
 *       本函数不发送 CAN、不确认保存结果；返回 1 不代表保存完成。
 *       命令仅在电机失能状态下有效，一次保存全部参数；调用方须为最长 30 ms 的操作留出时间。
 *       Flash 擦写寿命约 1 万次，请勿在周期控制循环中反复发送保存命令。
 *       STM32 HAL 发送会读取 8 字节，发送缓冲区应分配并初始化 8 字节，实际 DLC 仍为 4；
 *       当前 bsp_can_tx() 固定 DLC 为 8，调用方须使用支持 DLC 为 4 的发送接口。
 */
uint8_t drv_motor_dm_save_params_frame_config(uint8_t *frame, uint16_t cmd_id);

/**
 * @brief 解析达妙保存参数成功回复，校验命令字和回显的目标电机 CAN ID。
 * @param cmd_id 本次保存请求的目标电机基础 CAN ID，范围 0～0x7FF。
 * @param data 接收数据场；
 * @param len 接收数据场长度，须为 4 字节。
 * @return 1 表示收到匹配目标电机的保存成功回复；0 表示空指针、长度或 ID 无效、
 *         命令字错误或回显 ID 不匹配，不能仅据返回 0 判定电机保存失败。
 * @note 调用方应先按电机配置的 Master_id 分发标准数据帧，再关联本次保存请求；
 *       本函数仅校验数据场，不发送 CAN、不等待回复，也不修改电机状态。
 */
uint8_t drv_motor_dm_save_params_response_parse(uint16_t cmd_id,
                                                const uint8_t *data, uint8_t len);

//Port层

/**
 * @brief  CAN 初始化 (注册接收回调)
 * @param  hcan    CAN 句柄 (void*)
 * @param  can_id  要监听的 CAN ID
 * @param  rx_cb   接收回调
 */
void drv_motor_port_can_init(void *hcan, uint32_t can_id,
                              void (*rx_cb)(uint32_t, uint8_t*, uint8_t));

/**
 * @brief  CAN 发送 (非阻塞)
 * @param  hcan    CAN 句柄 (void*)
 * @param  std_id  标准 ID
 * @param  data    数据 (8 字节)
 */
void drv_motor_port_can_tx(void *hcan, uint32_t std_id, uint8_t *data);

#endif
