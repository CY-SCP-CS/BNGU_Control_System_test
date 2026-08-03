/**
 * @file    app_referee.c
 * @brief   裁判系统串口通信协议实现 (RoboMaster 2026 官方协议)
 * @note    底盘C板通过 USART6 DMA+IDLE 接收, SOF=0xA5 帧协议
 *          CRC8(多项式0x31)校验帧头, CRC16(多项式0x1021)校验整帧
 *          协议定义参考: D:/STM32project/referee_system/MDK-ARM/referee_system_data.c
 */
#include "app_referee.h"

#include "main.h"       /* HAL 库 / huart6 */
#include "usart.h"       /* huart6 声明 */
#include <string.h>

/* ════════════════════════════════════════════════════
 * CRC 查表与计算
 * ════════════════════════════════════════════════════ */

/**
 * CRC8 配置
 * - 多项式: x^8 + x^5 + x^4 + 1  (0x31)
 * - 初值: 0xFF
 * - 输入不翻转, 输出不翻转
 * 用于校验帧头前 4 字节 (SOF + data_length + seq)
 */
#define APP_REFEREE_CRC8_INIT  0xFF

static const uint8_t crc8_table[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

/**
 * CRC16 配置
 * - 多项式: x^16 + x^12 + x^5 + 1  (0x1021)
 * - 初值: 0xFFFF
 * - 输入不翻转, 输出不翻转
 * 用于校验除 SOF 外的整帧数据
 */
#define APP_REFEREE_CRC16_INIT 0xFFFF

static const uint16_t crc16_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E, 0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD, 0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C, 0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB, 0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A, 0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9, 0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738, 0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7, 0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036, 0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5, 0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134, 0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3, 0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232, 0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1, 0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330, 0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
};

/* ── CRC 计算函数 ──────────────────────────────── */

static uint8_t calc_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = APP_REFEREE_CRC8_INIT;
    while (len--) {
        crc = crc8_table[crc ^ *data++];
    }
    return crc;
}

static uint16_t calc_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = APP_REFEREE_CRC16_INIT;
    while (len--) {
        crc = (uint16_t)((crc >> 8) ^ crc16_table[(crc ^ *data++) & 0x00FF]);
    }
    return crc;
}

/* ════════════════════════════════════════════════════
 * DMA 缓冲区与全局数据
 * ════════════════════════════════════════════════════ */

static uint8_t s_dma_buf[REFEREE_RX_BUF_SIZE];   /* DMA循环缓冲                */
static uint8_t s_temp_buf[REFEREE_RX_BUF_SIZE];  /* 拷贝缓冲(中断安全)          */
static referee_global_t s_referee_data;           /* 全局裁判数据               */

/* ════════════════════════════════════════════════════
 * 各命令处理器 (按 CMD_ID 分派)
 * ════════════════════════════════════════════════════ */

/* ── 0x0001 比赛状态 ──────────────────────────── */
static void handle_game_status(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_game_status_t)) return;
    memcpy(&s_referee_data.game_status, data, sizeof(referee_game_status_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_GAME_STATUS;
}

/* ── 0x0002 比赛结果 ──────────────────────────── */
static void handle_game_result(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_game_result_t)) return;
    memcpy(&s_referee_data.game_result, data, sizeof(referee_game_result_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_GAME_RESULT;
}

/* ── 0x0003 全场机器人血量 ───────────────────── */
static void handle_game_robot_hp(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_game_robot_hp_t)) return;
    memcpy(&s_referee_data.game_robot_hp, data, sizeof(referee_game_robot_hp_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_GAME_ROBOT_HP;
}

/* ── 0x0101 场地事件 ──────────────────────────── */
static void handle_event_data(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_event_data_t)) return;
    memcpy(&s_referee_data.event_data, data, sizeof(referee_event_data_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_EVENT_DATA;
}

/* ── 0x0104 裁判警告 ──────────────────────────── */
static void handle_referee_warning(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_warning_t)) return;
    memcpy(&s_referee_data.referee_warning, data, sizeof(referee_warning_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_REFEREE_WARNING;
}

/* ── 0x0105 飞镖发射信息 ──────────────────────── */
static void handle_dart_info(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_dart_info_t)) return;
    memcpy(&s_referee_data.dart_info, data, sizeof(referee_dart_info_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_DART_INFO;
}

/* ── 0x0201 机器人性能状态 ───────────────────── */
static void handle_robot_status(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_robot_status_t)) return;
    memcpy(&s_referee_data.robot_status, data, sizeof(referee_robot_status_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_ROBOT_STATUS;
}

/* ── 0x0202 功率热量数据 ──────────────────────── */
static void handle_power_heat(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_power_heat_t)) return;
    memcpy(&s_referee_data.power_heat_data, data, sizeof(referee_power_heat_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_POWER_HEAT;
}

/* ── 0x0203 机器人位置 ────────────────────────── */
static void handle_robot_pos(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_robot_pos_t)) return;
    memcpy(&s_referee_data.robot_pos, data, sizeof(referee_robot_pos_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_ROBOT_POS;
}

/* ── 0x0204 增益/减益状态 ─────────────────────── */
static void handle_buff(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_buff_t)) return;
    memcpy(&s_referee_data.buff, data, sizeof(referee_buff_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_BUFF;
}

/* ── 0x0205 空中机器人能量 ────────────────────── */
static void handle_air_support(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_air_support_data_t)) return;
    memcpy(&s_referee_data.air_support, data, sizeof(referee_air_support_data_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_AIR_SUPPORT;
}

/* ── 0x0206 伤害状态 ──────────────────────────── */
static void handle_hurt_data(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_hurt_data_t)) return;
    memcpy(&s_referee_data.hurt_data, data, sizeof(referee_hurt_data_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_HURT_DATA;
}

/* ── 0x0207 实时射击数据 ──────────────────────── */
static void handle_shoot_data(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_shoot_data_t)) return;
    memcpy(&s_referee_data.shoot_data, data, sizeof(referee_shoot_data_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_SHOOT_DATA;
}

/* ── 0x0208 弹丸许可量 ────────────────────────── */
static void handle_projectile_allowance(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_projectile_allowance_t)) return;
    memcpy(&s_referee_data.projectile, data, sizeof(referee_projectile_allowance_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_PROJECTILE_ALLOWANCE;
}

/* ── 0x0209 RFID状态 ──────────────────────────── */
static void handle_rfid_status(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_rfid_status_t)) return;
    memcpy(&s_referee_data.rfid_status, data, sizeof(referee_rfid_status_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_RFID_STATUS;
}

/* ── 0x020A 飞镖选手端指令 ────────────────────── */
static void handle_dart_client_cmd(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_dart_client_cmd_t)) return;
    memcpy(&s_referee_data.dart_client_cmd, data, sizeof(referee_dart_client_cmd_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_DART_CLIENT_CMD;
}

/* ── 0x020B 地面机器人位置 ────────────────────── */
static void handle_ground_robot_pos(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_ground_robot_pos_t)) return;
    memcpy(&s_referee_data.ground_robot_pos, data, sizeof(referee_ground_robot_pos_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_GROUND_ROBOT_POS;
}

/* ── 0x020C 雷达标记进度 ──────────────────────── */
static void handle_radar_mark(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_radar_mark_data_t)) return;
    memcpy(&s_referee_data.radar_mark, data, sizeof(referee_radar_mark_data_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_RADAR_MARK;
}

/* ── 0x020D 哨兵信息 ──────────────────────────── */
static void handle_sentry_info(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_sentry_info_t)) return;
    memcpy(&s_referee_data.sentry_info, data, sizeof(referee_sentry_info_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_SENTRY_INFO;
}

/* ── 0x020E 雷达站信息 ────────────────────────── */
static void handle_radar_info(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(referee_radar_info_t)) return;
    memcpy(&s_referee_data.radar_info, data, sizeof(referee_radar_info_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_RADAR_INFO;
}

/* ── 0x0301 机器人交互 (含UI绘制) ────────────────
 * 结构: data_cmd_id(2) + sender_id(2) + receiver_id(2) + user_data(N)
 * 当前仅做解析框架, 具体交互处理留待后续实现
 */
static void handle_robot_interaction(const uint8_t *data, uint16_t len)
{
    if (len < 6) return;  /* 至少需要 data_cmd_id + sender_id + receiver_id */

    uint16_t sub_cmd_id;
    memcpy(&sub_cmd_id, data, sizeof(sub_cmd_id));  /* 前2字节为子命令ID */

    const uint8_t *user_data = data + 6;   /* 跳过头部6字节 */
    uint16_t user_data_len = len - 6;

    switch (sub_cmd_id) {
    case SUB_CMD_LAYER_DELETE:
        /* 删除图层, 暂不处理 */
        break;
    case SUB_CMD_DRAW_ONE_GRAPHIC:
        /* 绘制1个图形, 暂不处理 */
        break;
    case SUB_CMD_DRAW_TWO_GRAPHIC:
        /* 绘制2个图形, 暂不处理 */
        break;
    case SUB_CMD_DRAW_FIVE_GRAPHIC:
        /* 绘制5个图形, 暂不处理 */
        break;
    case SUB_CMD_DRAW_SEVEN_GRAPHIC:
        /* 绘制7个图形, 暂不处理 */
        break;
    case SUB_CMD_SENTRY_CMD:
        if (user_data_len >= sizeof(referee_sentry_cmd_t)) {
            /* 哨兵命令, 暂不处理 */
        }
        break;
    case SUB_CMD_RADAR_CMD:
        if (user_data_len >= sizeof(referee_radar_cmd_t)) {
            /* 雷达命令, 暂不处理 */
        }
        break;
    default:
        break;
    }
}

/* ════════════════════════════════════════════════════
 * 帧解析 (无CRC校验版本, 裁判系统CRC可选)
 * ════════════════════════════════════════════════════ */

/**
 * @brief  解析单个裁判系统数据包
 * @param  buffer  数据缓冲区 (必须以 SOF 0xA5 开头)
 * @param  length  缓冲区剩余长度
 * @return  0  解析成功
 *         -1  长度不足
 *         -2  未知命令ID
 *         -3  SOF错误
 */
static int parse_referee_packet(const uint8_t *buffer, uint32_t length)
{
    if (!buffer || length < PROTOCOL_MIN_FRAME_LEN) {
        return -1;   /* 长度不足 */
    }

    /* 1. 检查 SOF */
    const referee_frame_header_t *header = (const referee_frame_header_t *)buffer;
    if (header->sof != PROTOCOL_SOF) {
        return -3;   /* SOF 不匹配 */
    }

    /* 2. 验证 data_length 是否越界 */
    /*    全帧 = 帧头5 + 命令2 + 数据N + CRC16(2) */
    uint32_t total_len = PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN
                       + header->data_length + PROTOCOL_CRC16_LEN;
    if (length < total_len) {
        return -1;   /* 数据尚未收齐 */
    }

    /* 3. 提取命令和数据 */
    const uint8_t *cmd_ptr  = buffer + PROTOCOL_HEADER_LEN;          /* 命令码位置 */
    uint16_t cmd_id;
    memcpy(&cmd_id, cmd_ptr, sizeof(cmd_id));                        /* 小端序直接读取 */
    const uint8_t *data_ptr = cmd_ptr + PROTOCOL_CMD_LEN;            /* 数据段位置 */
    uint16_t data_len = header->data_length;

    /* 4. 按命令码分派 */
    switch (cmd_id) {
    case CMD_GAME_STATUS:           handle_game_status(data_ptr, data_len);          break;
    case CMD_GAME_RESULT:           handle_game_result(data_ptr, data_len);          break;
    case CMD_GAME_ROBOT_HP:         handle_game_robot_hp(data_ptr, data_len);        break;
    case CMD_EVENT_DATA:            handle_event_data(data_ptr, data_len);           break;
    case CMD_REFEREE_WARNING:       handle_referee_warning(data_ptr, data_len);      break;
    case CMD_DART_INFO:             handle_dart_info(data_ptr, data_len);            break;
    case CMD_ROBOT_STATUS:          handle_robot_status(data_ptr, data_len);         break;
    case CMD_POWER_HEAT_DATA:       handle_power_heat(data_ptr, data_len);           break;
    case CMD_ROBOT_POS:             handle_robot_pos(data_ptr, data_len);            break;
    case CMD_BUFF_DATA:             handle_buff(data_ptr, data_len);                 break;
    case CMD_AIR_SUPPORT_DATA:      handle_air_support(data_ptr, data_len);          break;
    case CMD_HURT_DATA:             handle_hurt_data(data_ptr, data_len);            break;
    case CMD_SHOOT_DATA:            handle_shoot_data(data_ptr, data_len);           break;
    case CMD_PROJECTILE_ALLOWANCE:  handle_projectile_allowance(data_ptr, data_len); break;
    case CMD_RFID_STATUS:           handle_rfid_status(data_ptr, data_len);          break;
    case CMD_DART_CLIENT_CMD:       handle_dart_client_cmd(data_ptr, data_len);      break;
    case CMD_GROUND_ROBOT_POS:      handle_ground_robot_pos(data_ptr, data_len);     break;
    case CMD_RADAR_MARK_DATA:       handle_radar_mark(data_ptr, data_len);           break;
    case CMD_SENTRY_INFO:           handle_sentry_info(data_ptr, data_len);          break;
    case CMD_RADAR_INFO:            handle_radar_info(data_ptr, data_len);           break;
    case CMD_ROBOT_INTERACTION:     handle_robot_interaction(data_ptr, data_len);    break;
    default:
        return -2;   /* 未知命令ID */
    }

    return 0;
}

/* ════════════════════════════════════════════════════
 * 公有接口
 * ════════════════════════════════════════════════════ */

/**
 * @brief  初始化裁判系统 UART 接收
 * @note   启动 USART6 DMA + IDLE 中断连续接收
 *         裁判系统连接 USART6 (PG9=RX, PG14=TX), 波特率 115200/8N1
 */
void app_referee_init(void)
{
    memset(&s_referee_data, 0, sizeof(s_referee_data));
    memset(s_dma_buf, 0, sizeof(s_dma_buf));
    memset(s_temp_buf, 0, sizeof(s_temp_buf));

    /* 使能 IDLE 中断, 启动 DMA 接收 */
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart6, s_dma_buf, REFEREE_RX_BUF_SIZE);
}

/**
 * @brief  USART6 空闲中断处理
 * @note  必须在 stm32f4xx_it.c 的 USART6_IRQHandler 中调用此函数,
 *         位置在 HAL_UART_IRQHandler(&huart6) 之后
 *
 *   void USART6_IRQHandler(void)
 *   {
 *       HAL_UART_IRQHandler(&huart6);
 *       // USER CODE BEGIN USART6_IRQn 1
 *       app_referee_uart_idle_handler(&huart6);
 *       // USER CODE END USART6_IRQn 1
 *   }
 */
void app_referee_uart_idle_handler(void)
{
    UART_HandleTypeDef *huart = &huart6;

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET) {
        return;  /* 非空闲中断 */
    }
    __HAL_UART_CLEAR_IDLEFLAG(huart);  /* 先清标志 */

    /* 计算本次 DMA 接收的字节数 */
    uint32_t receive_len = REFEREE_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    if (receive_len > 0) {
        /* 拷贝到临时缓冲区, 防止 DMA 覆盖正在处理的数据 */
        memcpy(s_temp_buf, s_dma_buf, receive_len);

        /* 逐帧解析: 从缓冲区头部开始查找 SOF=0xA5 的帧 */
        uint32_t i = 0;
        while (i < receive_len) {
            if (s_temp_buf[i] == PROTOCOL_SOF) {
                /* 尝试解析一帧 */
                int status = parse_referee_packet(&s_temp_buf[i], receive_len - i);

                if (status == 0) {
                    /* 解析成功, 跳过整帧继续 */
                    const referee_frame_header_t *hdr =
                        (const referee_frame_header_t *)&s_temp_buf[i];
                    uint32_t full_len = PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN
                                      + hdr->data_length + PROTOCOL_CRC16_LEN;
                    i += full_len;
                    continue;
                } else if (status == -1) {
                    /* 长度不足, 帧可能被截断, 等下次 DMA 补全 */
                    break;
                }
                /* status == -2 (未知命令) 或 -3 (SOF错误): 继续扫描下一个字节 */
            }
            i++;
        }
    }

    /* 重启 DMA 接收 (HAL_UART_Receive_DMA 不会自动循环) */
    HAL_UART_DMAStop(huart);
    HAL_UART_Receive_DMA(huart, s_dma_buf, REFEREE_RX_BUF_SIZE);
}

const referee_global_t *app_referee_get_data(void)
{
    return &s_referee_data;
}

uint32_t app_referee_get_and_clear_flags(void)
{
    uint32_t flags = s_referee_data.data_valid_flags;
    s_referee_data.data_valid_flags = 0;
    return flags;
}

int app_referee_is_data_updated(uint32_t flag)
{
    return (s_referee_data.data_valid_flags & flag) ? 1 : 0;
}

const char *app_referee_parse_error_string(int result)
{
    switch (result) {
    case  0: return "Parse success";
    case -1: return "Length insufficient";
    case -2: return "Unknown command ID";
    case -3: return "SOF error (not 0xA5)";
    default: return "Unknown error";
    }
}