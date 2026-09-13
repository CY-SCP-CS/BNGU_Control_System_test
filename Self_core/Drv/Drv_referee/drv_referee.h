/**
 * @file    drv_referee.h
 * @brief   裁判系统串口通信协议 (RoboMaster 2026 官方协议)
 * @note    底盘C板通过 USART6 直连裁判系统模块, 波特率115200/8N1
 *          DMA+空闲中断接收, 帧格式: SOF(0xA5) + 帧头(5B) + 命令(2B) + 数据(NB) + CRC16(2B)
 * @ref     D:/STM32project/referee_system/MDK-ARM/referee_system_data.h
 */
#ifndef DRV_REFEREE_H
#define DRV_REFEREE_H

#include "lib_typedef.h"

// ─── 协议常量 ──────────────────────────────────────

#define PROTOCOL_SOF              0xA5    /**< 帧起始标志                              */
#define PROTOCOL_HEADER_LEN       5       /**< 帧头: SOF(1) + data_len(2) + seq(1) + CRC8(1) */
#define PROTOCOL_CMD_LEN          2       /**< 命令码长度                              */
#define PROTOCOL_CRC16_LEN        2       /**< 帧尾CRC16长度                           */
#define PROTOCOL_MIN_FRAME_LEN    9       /**< 最小帧长: 5+2+0+2=9                    */
#define REFEREE_RX_BUF_SIZE       512     /**< DMA接收缓冲区大小                        */

// 编译器对齐宏 (STM32为小端, 使用packed确保结构体与协议字节对齐)
#if defined(__GNUC__)
#define DRV_REFEREE_PACKED __attribute__((packed))
#else
#define DRV_REFEREE_PACKED
#endif

// ─── 命令码定义 (CMD_ID) ──────────────────────────

typedef enum {
    /* ── 比赛信息 (0x00xx) ── */
    CMD_GAME_STATUS          = 0x0001,   /**< 比赛状态数据                           */
    CMD_GAME_RESULT          = 0x0002,   /**< 比赛结果数据                           */
    CMD_GAME_ROBOT_HP        = 0x0003,   /**< 机器人血量数据                         */

    /* ── 比赛事件 (0x01xx) ── */
    CMD_EVENT_DATA           = 0x0101,   /**< 场地事件数据                           */
    CMD_REFEREE_WARNING      = 0x0104,   /**< 裁判警告信息                           */
    CMD_DART_INFO            = 0x0105,   /**< 飞镖发射相关信息                       */

    /* ── 机器人状态 (0x02xx) ── */
    CMD_ROBOT_STATUS         = 0x0201,   /**< 机器人性能体系数据                      */
    CMD_POWER_HEAT_DATA      = 0x0202,   /**< 实时底盘缓冲能量和枪口热量              */
    CMD_ROBOT_POS            = 0x0203,   /**< 机器人位置数据                         */
    CMD_BUFF_DATA            = 0x0204,   /**< 机器人增益和减益状态                    */
    CMD_AIR_SUPPORT_DATA     = 0x0205,   /**< 空中机器人能量状态                      */
    CMD_HURT_DATA            = 0x0206,   /**< 伤害状态数据                           */
    CMD_SHOOT_DATA           = 0x0207,   /**< 实时射击数据                           */
    CMD_PROJECTILE_ALLOWANCE = 0x0208,   /**< 弹丸剩余许可量                          */
    CMD_RFID_STATUS          = 0x0209,   /**< 场地RFID模块状态                       */
    CMD_DART_CLIENT_CMD      = 0x020A,   /**< 飞镖选手端指令数据                      */
    CMD_GROUND_ROBOT_POS     = 0x020B,   /**< 地面机器人位置数据                      */
    CMD_RADAR_MARK_DATA      = 0x020C,   /**< 雷达标记进度数据                        */
    CMD_SENTRY_INFO          = 0x020D,   /**< 哨兵机器人信息同步                      */
    CMD_RADAR_INFO           = 0x020E,   /**< 雷达站信息同步                          */

    /* ── 机器人交互 (0x03xx) ── */
    CMD_ROBOT_INTERACTION    = 0x0301,   /**< 机器人交互数据(含UI绘制)                */

    /* ── 子命令 (0x0301的子类型) ── */
    SUB_CMD_LAYER_DELETE      = 0x0100,  /**< 选手端删除图层                         */
    SUB_CMD_DRAW_ONE_GRAPHIC  = 0x0101,  /**< 选手端绘制1个图形                       */
    SUB_CMD_DRAW_TWO_GRAPHIC  = 0x0102,  /**< 选手端绘制2个图形                       */
    SUB_CMD_DRAW_FIVE_GRAPHIC = 0x0103,  /**< 选手端绘制5个图形                       */
    SUB_CMD_DRAW_SEVEN_GRAPHIC= 0x0104,  /**< 选手端绘制7个图形                       */
    SUB_CMD_SENTRY_CMD        = 0x0120,  /**< 哨兵机器人命令                         */
    SUB_CMD_RADAR_CMD         = 0x0121   /**< 雷达站命令                             */
} drv_referee_cmd_t;

// ════════════════════════════════════════════════════
// 帧头结构体
// ════════════════════════════════════════════════════

typedef struct DRV_REFEREE_PACKED {
    uint8_t  sof;           /**< 帧起始标志, 固定0xA5                       */
    uint16_t data_length;   /**< 数据段长度(不含帧头/命令/CRC), 小端序      */
    uint8_t  seq;           /**< 包序号, 用于丢帧检测                       */
    uint8_t  crc8;          /**< 帧头CRC8校验 (多项式0x31, 初值0xFF)        */
} drv_referee_frame_header_t;

// ════════════════════════════════════════════════════
// 比赛信息结构体 (0x00xx)
// ════════════════════════════════════════════════════

typedef struct DRV_REFEREE_PACKED {
    uint8_t  game_type   : 4;  /**< 比赛类型: 1=RoboMaster 2=对抗赛 3=3V3 4=单项赛 */
    uint8_t  game_progress: 4; /**< 比赛阶段: 0=未开始 1=准备 2=自检 3=五秒 4=战斗 5=结算 */
    uint16_t stage_remain_time; /**< 当前阶段剩余时间(秒)                     */
    uint64_t sync_time_stamp;   /**< 同步时间戳(UNIX秒)                      */
} drv_referee_game_status_t;

typedef struct DRV_REFEREE_PACKED {
    uint8_t winner;             /**< 获胜方: 0=平局 1=红方 2=蓝方            */
} drv_referee_game_result_t;

typedef struct DRV_REFEREE_PACKED {
    uint16_t ally_1_robot_hp;   /**< 己方1号英雄血量                          */
    uint16_t ally_2_robot_hp;   /**< 己方2号工程血量                          */
    uint16_t ally_3_robot_hp;   /**< 己方3号步兵血量                          */
    uint16_t ally_4_robot_hp;   /**< 己方4号步兵血量                          */
    uint16_t reserved;          /**< 保留                                     */
    uint16_t ally_7_robot_hp;   /**< 己方7号哨兵血量                          */
    uint16_t ally_outpost_hp;   /**< 己方前哨站血量                            */
    uint16_t ally_base_hp;      /**< 己方基地血量                              */
} drv_referee_game_robot_hp_t;

// ════════════════════════════════════════════════════
// 比赛事件结构体 (0x01xx)
// ════════════════════════════════════════════════════

typedef struct DRV_REFEREE_PACKED {
    uint32_t event_data;        /**< 事件状态位, 每位代表一个事件             */
} drv_referee_event_data_t;

typedef struct DRV_REFEREE_PACKED {
    uint8_t level;              /**< 判罚等级: 1=警告 2=严重 3=非常严重      */
    uint8_t offending_robot_id; /**< 违规机器人ID                            */
    uint8_t count;              /**< 违规次数                                */
} drv_referee_warning_t;

typedef struct DRV_REFEREE_PACKED {
    uint8_t  dart_remaining_time; /**< 飞镖发射剩余时间(秒)                   */
    uint16_t dart_info;           /**< 发射口信息位                           */
} drv_referee_dart_info_t;

// ════════════════════════════════════════════════════
// 机器人状态结构体 (0x02xx)
// ════════════════════════════════════════════════════

typedef struct DRV_REFEREE_PACKED {
    uint8_t  robot_id;                   /**< 机器人ID: 1=英雄 2=工程 3/4=步兵 7=哨兵 */
    uint8_t  robot_level;                /**< 机器人等级: 1-3级                        */
    uint16_t current_hp;                 /**< 当前血量                                  */
    uint16_t maximum_hp;                 /**< 血量上限                                  */
    uint16_t shooter_barrel_cooling_value; /**< 枪口冷却值                              */
    uint16_t shooter_barrel_heat_limit;  /**< 枪口热量上限                              */
    uint16_t chassis_power_limit;        /**< 底盘功率限制(W)                           */
    uint8_t  power_management_gimbal_output  : 1;  /**< 云台供电状态              */
    uint8_t  power_management_chassis_output : 1;  /**< 底盘供电状态              */
    uint8_t  power_management_shooter_output : 1;  /**< 发射机构供电状态          */
} drv_referee_robot_status_t;

typedef struct {
    uint8_t  robot_level;               /**< 当前机器人等级                 */
    uint8_t  is_chassis_output_enabled; /**< 裁判系统是否允许底盘供电       */
    uint16_t power_limit;              /**< 裁判系统下发的底盘功率上限 (W) */
} drv_referee_chassis_power_t;

typedef struct DRV_REFEREE_PACKED {
    uint16_t reserved;              /**< 保留                              */
    uint16_t reserved2;             /**< 保留                              */
    float    reserved3;             /**< 保留                              */
    uint16_t buffer_energy;         /**< 缓冲能量(J)                       */
    uint16_t shooter_17mm_barrel_heat;  /**< 17mm枪口热量                  */
    uint16_t shooter_42mm_barrel_heat;  /**< 42mm枪口热量                  */
} drv_referee_power_heat_t;

typedef struct DRV_REFEREE_PACKED {
    float x;        /**< 位置x坐标(m)                                     */
    float y;        /**< 位置y坐标(m)                                     */
    float angle;    /**< 朝向角度(rad), 0=正东, 顺时针为正                 */
} drv_referee_robot_pos_t;

typedef struct DRV_REFEREE_PACKED {
    uint8_t  recovery_buff;      /**< 回血增益(百分比, 0-100)              */
    uint16_t cooling_buff;       /**< 冷却增益(直接加值)                   */
    uint8_t  defence_buff;       /**< 防御增益(百分比, 0-100)              */
    uint8_t  vulnerability_buff; /**< 负防御增益(百分比, 0-100)            */
    uint16_t attack_buff;        /**< 攻击增益(百分比, 0-100)              */
    uint8_t  remaining_energy;   /**< 剩余能量值                          */
} drv_referee_buff_t;

typedef struct DRV_REFEREE_PACKED {
    uint8_t  air_status;         /**< 空中机器人状态                       */
    uint8_t  reserved;           /**< 保留                                */
    uint16_t air_energy;         /**< 空中机器人能量值                     */
} drv_referee_air_support_data_t;

typedef struct DRV_REFEREE_PACKED {
    uint8_t armor_id            : 4;  /**< 装甲模块ID: 0=前 1=左 2=后 3=右 4=上 5=下 */
    uint8_t hp_deduction_reason : 4;  /**< 血量变化原因: 0=装甲伤害 1=模块掉线 2=枪口超限 3=底盘超功率 4=装甲撞击 */
} drv_referee_hurt_data_t;

typedef struct DRV_REFEREE_PACKED {
    uint8_t bullet_type;          /**< 弹丸类型: 1=17mm 2=42mm 3=16mm 4=42mm血弹 */
    uint8_t shooter_number;       /**< 发射器ID: 1=1号 2=2号                       */
    uint8_t launching_frequency;  /**< 发射频率(Hz)                                */
    float   initial_speed;        /**< 弹丸初速度(m/s)                              */
} drv_referee_shoot_data_t;

typedef struct DRV_REFEREE_PACKED {
    uint16_t projectile_allowance_17mm;     /**< 17mm弹丸许可量             */
    uint16_t projectile_allowance_42mm;     /**< 42mm弹丸许可量             */
    uint16_t remaining_gold_coin;           /**< 剩余金币                   */
    uint16_t projectile_allowance_fortress; /**< 堡垒17mm弹丸储备           */
} drv_referee_projectile_allowance_t;

typedef struct DRV_REFEREE_PACKED {
    uint32_t rfid_status;       /**< RFID状态位 (每bit代表一个RFID)         */
    uint8_t  rfid_status_2;     /**< RFID状态位2                           */
} drv_referee_rfid_status_t;

typedef struct DRV_REFEREE_PACKED {
    uint8_t  dart_launch_opening_status;  /**< 飞镖发射站状态: 0=关闭 1=开启     */
    uint8_t  reserved;                    /**< 保留                              */
    uint16_t target_change_time;          /**< 切换目标剩余时间(秒)               */
    uint16_t latest_launch_cmd_time;      /**< 最近发射指令时间(秒)               */
} drv_referee_dart_client_cmd_t;

typedef struct DRV_REFEREE_PACKED {
    float hero_x, hero_y;           /**< 英雄机器人坐标(m)                  */
    float engineer_x, engineer_y;   /**< 工程机器人坐标(m)                  */
    float standard_3_x, standard_3_y; /**< 3号步兵坐标(m)                   */
    float standard_4_x, standard_4_y; /**< 4号步兵坐标(m)                   */
    float reserved, reserved2;      /**< 保留                               */
} drv_referee_ground_robot_pos_t;

typedef struct DRV_REFEREE_PACKED {
    uint16_t mark_progress;         /**< 标记进度状态位                      */
} drv_referee_radar_mark_data_t;

typedef struct DRV_REFEREE_PACKED {
    uint32_t sentry_info;           /**< 哨兵信息位                          */
    uint16_t sentry_info_2;         /**< 哨兵信息位2                         */
} drv_referee_sentry_info_t;

typedef struct DRV_REFEREE_PACKED {
    uint8_t radar_info;             /**< 雷达信息位                          */
} drv_referee_radar_info_t;

// ════════════════════════════════════════════════════
// 机器人交互结构体 (0x03xx)
// ════════════════════════════════════════════════════

typedef struct DRV_REFEREE_PACKED {
    uint16_t data_cmd_id;           /**< 数据命令ID                           */
    uint16_t sender_id;             /**< 发送者机器人ID                       */
    uint16_t receiver_id;           /**< 接收者机器人ID (0xFFFF=广播)         */
    uint8_t  user_data[];           /**< 用户数据段(变长)                     */
} drv_referee_interaction_data_t;

/* ── 交互子类型: 图层删除 ── */
typedef struct DRV_REFEREE_PACKED {
    uint8_t delete_type;            /**< 删除操作: 0=删指定 1=删全部层       */
    uint8_t layer;                  /**< 图层号 (0-9)                        */
} drv_referee_interaction_layer_delete_t;

/* ── 交互子类型: 图形绘制 (单图形) ── */
typedef struct DRV_REFEREE_PACKED {
    uint8_t  figure_name[3];        /**< 图形名称(3字符)                     */
    uint32_t operate_type : 3;      /**< 操作: 1=添加 2=修改 3=删除          */
    uint32_t figure_type  : 3;      /**< 图形类型: 0=线 1=矩形 2=圆 3=椭圆 4=弧 5=整数 6=浮点 7=字符 */
    uint32_t layer        : 4;      /**< 图层号 (0-9)                        */
    uint32_t color        : 4;      /**< 颜色: 0=主色 1=黄 2=绿 3=橙 4=紫红 5=粉 6=青 7=黑 8=白 */
    uint32_t details_a    : 9;      /**< 图形细节参数A                        */
    uint32_t details_b    : 9;      /**< 图形细节参数B                        */
    uint32_t width        : 10;     /**< 线宽(像素)                          */
    uint32_t start_x      : 11;     /**< 起点/圆心x坐标                      */
    uint32_t start_y      : 11;     /**< 起点/圆心y坐标                      */
    uint32_t details_c    : 10;     /**< 图形细节参数C                        */
    uint32_t details_d    : 11;     /**< 图形细节参数D                        */
    uint32_t details_e    : 11;     /**< 图形细节参数E                        */
} drv_referee_interaction_figure_t;

/* ── 交互子类型: 哨兵命令 ── */
typedef struct DRV_REFEREE_PACKED {
    uint32_t sentry_cmd;            /**< 哨兵命令位                          */
} drv_referee_sentry_cmd_t;

/* ── 交互子类型: 雷达命令 ── */
typedef struct DRV_REFEREE_PACKED {
    uint8_t radar_cmd;              /**< 雷达触发双倍伤害命令                  */
    uint8_t password_cmd;           /**< 密钥命令类型                         */
    uint8_t password_1;             /**< 密钥字节1                            */
    uint8_t password_2;             /**< 密钥字节2                            */
    uint8_t password_3;             /**< 密钥字节3                            */
    uint8_t password_4;             /**< 密钥字节4                            */
    uint8_t password_5;             /**< 密钥字节5                            */
    uint8_t password_6;             /**< 密钥字节6                            */
} drv_referee_radar_cmd_t;

// ════════════════════════════════════════════════════
// 全局状态结构体 (存储所有裁判系统数据)
// ════════════════════════════════════════════════════

typedef struct {
    /* 比赛信息 */
    drv_referee_game_status_t      game_status;
    drv_referee_game_result_t      game_result;
    drv_referee_game_robot_hp_t    game_robot_hp;

    /* 比赛事件 */
    drv_referee_event_data_t       event_data;
    drv_referee_warning_t          referee_warning;
    drv_referee_dart_info_t        dart_info;

    /* 机器人状态 */
    drv_referee_robot_status_t     robot_status;
    drv_referee_power_heat_t       power_heat_data;
    drv_referee_robot_pos_t        robot_pos;
    drv_referee_buff_t             buff;
    drv_referee_air_support_data_t air_support;
    drv_referee_hurt_data_t        hurt_data;
    drv_referee_shoot_data_t       shoot_data;
    drv_referee_projectile_allowance_t projectile;
    drv_referee_rfid_status_t      rfid_status;
    drv_referee_dart_client_cmd_t  dart_client_cmd;
    drv_referee_ground_robot_pos_t ground_robot_pos;
    drv_referee_radar_mark_data_t  radar_mark;
    drv_referee_sentry_info_t      sentry_info;
    drv_referee_radar_info_t       radar_info;

    /* 数据有效标志位 */
    uint32_t                   data_valid_flags;
} drv_referee_global_t;

// ════════════════════════════════════════════════════
// 数据有效标志位定义
// ════════════════════════════════════════════════════

#define REFEREE_FLAG_GAME_STATUS          (1UL << 0)
#define REFEREE_FLAG_GAME_RESULT          (1UL << 1)
#define REFEREE_FLAG_GAME_ROBOT_HP        (1UL << 2)
#define REFEREE_FLAG_EVENT_DATA           (1UL << 3)
#define REFEREE_FLAG_REFEREE_WARNING      (1UL << 4)
#define REFEREE_FLAG_DART_INFO            (1UL << 5)
#define REFEREE_FLAG_ROBOT_STATUS         (1UL << 6)
#define REFEREE_FLAG_POWER_HEAT           (1UL << 7)
#define REFEREE_FLAG_ROBOT_POS            (1UL << 8)
#define REFEREE_FLAG_BUFF                 (1UL << 9)
#define REFEREE_FLAG_AIR_SUPPORT          (1UL << 10)
#define REFEREE_FLAG_HURT_DATA            (1UL << 11)
#define REFEREE_FLAG_SHOOT_DATA           (1UL << 12)
#define REFEREE_FLAG_PROJECTILE_ALLOWANCE (1UL << 13)
#define REFEREE_FLAG_RFID_STATUS          (1UL << 14)
#define REFEREE_FLAG_DART_CLIENT_CMD      (1UL << 15)
#define REFEREE_FLAG_GROUND_ROBOT_POS     (1UL << 16)
#define REFEREE_FLAG_RADAR_MARK           (1UL << 17)
#define REFEREE_FLAG_SENTRY_INFO          (1UL << 18)
#define REFEREE_FLAG_RADAR_INFO           (1UL << 19)

// ════════════════════════════════════════════════════
// 接口声明
// ════════════════════════════════════════════════════

/**
 * @brief  初始化裁判协议状态
 */
void drv_referee_init(void);

/**
 * @brief  获取裁判系统全局数据
 * @return 裁判系统数据指针
 */
const drv_referee_global_t *drv_referee_get_data(void);

/**
 * @brief  原子读取裁判系统底盘功率限制快照
 * @param  power       输出等级、功率上限和底盘供电状态
 * @param  timeout_ms  机器人状态数据允许的最大间隔
 * @return 1=数据有效且未超时, 0=无数据、超时或参数无效
 */
uint8_t drv_referee_read_chassis_power(drv_referee_chassis_power_t *power,
                                         uint32_t timeout_ms);

/**
 * @brief  获取并清除数据更新标志
 * @return 更新标志位掩码
 */
uint32_t drv_referee_get_and_clear_flags(void);

/**
 * @brief  检查指定数据是否已更新
 * @param  flag  标志位 (如 REFEREE_FLAG_ROBOT_STATUS)
 * @return 1=已更新  0=未更新
 */
int drv_referee_is_data_updated(uint32_t flag);

/**
 * @brief  解析错误码转字符串
 * @param  result  解析返回值
 * @return 错误描述字符串
 */
const char *drv_referee_parse_error_string(int result);

/** @brief 主循环输入任意分包字节流，校验 CRC 并保留未收齐的帧。 */
void drv_referee_process(const uint8_t *data, uint16_t len);

/** @brief 初始化 USART6 DMA + IDLE 裁判系统接收。 */
void drv_referee_port_init(void);
/** @brief USART6 IDLE 中断处理入口。 */
void drv_referee_port_uart_idle_handler(void);
/** @brief 主循环处理已接收的 DMA 数据。 */
void drv_referee_port_process(void);

#endif /* DRV_REFEREE_H */

