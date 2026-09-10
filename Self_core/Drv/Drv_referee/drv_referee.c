/**
 * @file    drv_referee.c
 * @brief   裁判系统协议解析 (RoboMaster 2026 官方协议)
 * @note    SOF=0xA5 帧协议
 *          CRC8(多项式0x31)校验帧头, CRC16(多项式0x1021)校验整帧
 *          协议定义参考: D:/STM32project/referee_system/MDK-ARM/referee_system_data.c
 */
#include "drv_referee.h"

#include "main.h"       /* HAL 库 / huart6 */
#include <string.h>

_Static_assert(sizeof(drv_referee_frame_header_t) == PROTOCOL_HEADER_LEN, "Referee header must be packed");
_Static_assert(sizeof(drv_referee_game_status_t) == 11U, "Referee game status must be packed");
_Static_assert(sizeof(drv_referee_robot_status_t) == 13U, "Referee robot status must be packed");

/* ════════════════════════════════════════════════════
 * CRC 查表与计算
 * ════════════════════════════════════════════════════ */

/**
 * CRC8 配置
 * - 多项式: x^8 + x^5 + x^4 + 1  (0x31)
 * - 初值: 0xFF
 * - 按最低有效位优先计算（反射实现）
 * 用于校验帧头前 4 字节 (SOF + data_length + seq)
 */
#define DRV_REFEREE_CRC8_INIT  0xFF

static const uint8_t s_referee_crc8_table[256] = {
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
 * - 按最低有效位优先计算（反射实现）
 * 用于校验从 SOF 开始、除末尾 CRC16 外的整帧数据
 */
#define DRV_REFEREE_CRC16_INIT 0xFFFF

/* ── CRC 计算函数 ──────────────────────────────── */

static uint8_t calc_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = DRV_REFEREE_CRC8_INIT;
    while (len--) {
        crc = s_referee_crc8_table[crc ^ *data++];
    }
    return crc;
}

static uint16_t calc_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = DRV_REFEREE_CRC16_INIT;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0x8408U) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/* ════════════════════════════════════════════════════
 * 协议缓存与全局数据
 * ════════════════════════════════════════════════════ */

static drv_referee_global_t s_referee_data;           /* 全局裁判数据               */
static uint8_t s_stream_buf[REFEREE_RX_BUF_SIZE * 2U];
static uint16_t s_stream_length;
static uint32_t s_robot_status_tick;
static uint8_t s_robot_status_is_valid;

/* ════════════════════════════════════════════════════
 * 各命令处理器 (按 CMD_ID 分派)
 * ════════════════════════════════════════════════════ */

/* ── 0x0001 比赛状态 ──────────────────────────── */
static void handle_game_status(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_game_status_t)) {
        return;
    }
    memcpy(&s_referee_data.game_status, data, sizeof(drv_referee_game_status_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_GAME_STATUS;
}

/* ── 0x0002 比赛结果 ──────────────────────────── */
static void handle_game_result(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_game_result_t)) {
        return;
    }
    memcpy(&s_referee_data.game_result, data, sizeof(drv_referee_game_result_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_GAME_RESULT;
}

/* ── 0x0003 全场机器人血量 ───────────────────── */
static void handle_game_robot_hp(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_game_robot_hp_t)) {
        return;
    }
    memcpy(&s_referee_data.game_robot_hp, data, sizeof(drv_referee_game_robot_hp_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_GAME_ROBOT_HP;
}

/* ── 0x0101 场地事件 ──────────────────────────── */
static void handle_event_data(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_event_data_t)) {
        return;
    }
    memcpy(&s_referee_data.event_data, data, sizeof(drv_referee_event_data_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_EVENT_DATA;
}

/* ── 0x0104 裁判警告 ──────────────────────────── */
static void handle_referee_warning(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_warning_t)) {
        return;
    }
    memcpy(&s_referee_data.referee_warning, data, sizeof(drv_referee_warning_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_REFEREE_WARNING;
}

/* ── 0x0105 飞镖发射信息 ──────────────────────── */
static void handle_dart_info(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_dart_info_t)) {
        return;
    }
    memcpy(&s_referee_data.dart_info, data, sizeof(drv_referee_dart_info_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_DART_INFO;
}

/* ── 0x0201 机器人性能状态 ───────────────────── */
static void handle_robot_status(const uint8_t *data, uint16_t len)
{
    drv_referee_robot_status_t robot_status;
    uint32_t irq_state;

    if (len < sizeof(robot_status)) {
        return;
    }
    memcpy(&robot_status, data, sizeof(robot_status));

    /* 控制中断会读取这组数据，发布过程必须保持完整。 */
    irq_state = __get_PRIMASK();
    __disable_irq();
    s_referee_data.robot_status = robot_status;
    s_referee_data.data_valid_flags |= REFEREE_FLAG_ROBOT_STATUS;
    s_robot_status_tick = HAL_GetTick();
    s_robot_status_is_valid = 1U;
    __set_PRIMASK(irq_state);
}

/* ── 0x0202 功率热量数据 ──────────────────────── */
static void handle_power_heat(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_power_heat_t)) {
        return;
    }
    memcpy(&s_referee_data.power_heat_data, data, sizeof(drv_referee_power_heat_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_POWER_HEAT;
}

/* ── 0x0203 机器人位置 ────────────────────────── */
static void handle_robot_pos(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_robot_pos_t)) {
        return;
    }
    memcpy(&s_referee_data.robot_pos, data, sizeof(drv_referee_robot_pos_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_ROBOT_POS;
}

/* ── 0x0204 增益/减益状态 ─────────────────────── */
static void handle_buff(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_buff_t)) {
        return;
    }
    memcpy(&s_referee_data.buff, data, sizeof(drv_referee_buff_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_BUFF;
}

/* ── 0x0205 空中机器人能量 ────────────────────── */
static void handle_air_support(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_air_support_data_t)) {
        return;
    }
    memcpy(&s_referee_data.air_support, data, sizeof(drv_referee_air_support_data_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_AIR_SUPPORT;
}

/* ── 0x0206 伤害状态 ──────────────────────────── */
static void handle_hurt_data(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_hurt_data_t)) {
        return;
    }
    memcpy(&s_referee_data.hurt_data, data, sizeof(drv_referee_hurt_data_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_HURT_DATA;
}

/* ── 0x0207 实时射击数据 ──────────────────────── */
static void handle_shoot_data(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_shoot_data_t)) {
        return;
    }
    memcpy(&s_referee_data.shoot_data, data, sizeof(drv_referee_shoot_data_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_SHOOT_DATA;
}

/* ── 0x0208 弹丸许可量 ────────────────────────── */
static void handle_projectile_allowance(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_projectile_allowance_t)) {
        return;
    }
    memcpy(&s_referee_data.projectile, data, sizeof(drv_referee_projectile_allowance_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_PROJECTILE_ALLOWANCE;
}

/* ── 0x0209 RFID状态 ──────────────────────────── */
static void handle_rfid_status(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_rfid_status_t)) {
        return;
    }
    memcpy(&s_referee_data.rfid_status, data, sizeof(drv_referee_rfid_status_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_RFID_STATUS;
}

/* ── 0x020A 飞镖选手端指令 ────────────────────── */
static void handle_dart_client_cmd(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_dart_client_cmd_t)) {
        return;
    }
    memcpy(&s_referee_data.dart_client_cmd, data, sizeof(drv_referee_dart_client_cmd_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_DART_CLIENT_CMD;
}

/* ── 0x020B 地面机器人位置 ────────────────────── */
static void handle_ground_robot_pos(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_ground_robot_pos_t)) {
        return;
    }
    memcpy(&s_referee_data.ground_robot_pos, data, sizeof(drv_referee_ground_robot_pos_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_GROUND_ROBOT_POS;
}

/* ── 0x020C 雷达标记进度 ──────────────────────── */
static void handle_radar_mark(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_radar_mark_data_t)) {
        return;
    }
    memcpy(&s_referee_data.radar_mark, data, sizeof(drv_referee_radar_mark_data_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_RADAR_MARK;
}

/* ── 0x020D 哨兵信息 ──────────────────────────── */
static void handle_sentry_info(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_sentry_info_t)) {
        return;
    }
    memcpy(&s_referee_data.sentry_info, data, sizeof(drv_referee_sentry_info_t));
    s_referee_data.data_valid_flags |= REFEREE_FLAG_SENTRY_INFO;
}

/* ── 0x020E 雷达站信息 ────────────────────────── */
static void handle_radar_info(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(drv_referee_radar_info_t)) {
        return;
    }
    memcpy(&s_referee_data.radar_info, data, sizeof(drv_referee_radar_info_t));
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
        if (user_data_len >= sizeof(drv_referee_sentry_cmd_t)) {
            /* 哨兵命令, 暂不处理 */
        }
        break;
    case SUB_CMD_RADAR_CMD:
        if (user_data_len >= sizeof(drv_referee_radar_cmd_t)) {
            /* 雷达命令, 暂不处理 */
        }
        break;
    default:
        break;
    }
}

/* ════════════════════════════════════════════════════
 * 帧解析（CRC8 帧头 + CRC16 全帧校验）
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
    const drv_referee_frame_header_t *header = (const drv_referee_frame_header_t *)buffer;
    if (header->sof != PROTOCOL_SOF) {
        return -3;   /* SOF 不匹配 */
    }

    if (calc_crc8(buffer, PROTOCOL_HEADER_LEN - 1U) != header->crc8) {
        return -4;
    }
    /* 2. 验证 data_length 是否越界 */
    /*    全帧 = 帧头5 + 命令2 + 数据N + CRC16(2) */
    uint32_t total_len = PROTOCOL_HEADER_LEN + PROTOCOL_CMD_LEN
                       + header->data_length + PROTOCOL_CRC16_LEN;
    if (total_len > REFEREE_RX_BUF_SIZE) {
        return -5;
    }
    if (length < total_len) {
        return -1;   /* 数据尚未收齐 */
    }

    uint16_t received_crc;
    memcpy(&received_crc, buffer + total_len - PROTOCOL_CRC16_LEN, sizeof(received_crc));
    if (calc_crc16(buffer, (uint16_t)(total_len - PROTOCOL_CRC16_LEN)) != received_crc) {
        return -4;
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
 * @brief  初始化裁判协议状态
 */
void drv_referee_init(void)
{
    memset(&s_referee_data, 0, sizeof(s_referee_data));

    s_stream_length = 0;
    s_robot_status_tick = 0U;
    s_robot_status_is_valid = 0U;
}

void drv_referee_process(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0U || len > REFEREE_RX_BUF_SIZE) {
        return;
    }
    if (s_stream_length + len > sizeof(s_stream_buf)) {
        s_stream_length = 0;
    }
    memcpy(s_stream_buf + s_stream_length, data, len);
    s_stream_length += len;
    uint16_t offset = 0;
    while (s_stream_length - offset >= PROTOCOL_HEADER_LEN) {
        const uint8_t *frame = s_stream_buf + offset;
        if (frame[0] != PROTOCOL_SOF || calc_crc8(frame, PROTOCOL_HEADER_LEN - 1U) != frame[4]) {
            offset++;
            continue;
        }
        uint16_t payload_length;
        memcpy(&payload_length, frame + 1, sizeof(payload_length));
        uint32_t frame_length = (uint32_t)payload_length + PROTOCOL_MIN_FRAME_LEN;
        if (frame_length > REFEREE_RX_BUF_SIZE) {
            offset++;
            continue;
        }
        if ((uint32_t)(s_stream_length - offset) < frame_length) {
            break;
        }
        int result = parse_referee_packet(frame, frame_length);
        if (result == 0 || result == -2) {
            offset += (uint16_t)frame_length;
        } else {
            offset++;
        }
    }
    s_stream_length -= offset;
    memmove(s_stream_buf, s_stream_buf + offset, s_stream_length);
}

const drv_referee_global_t *drv_referee_get_data(void)
{
    return &s_referee_data;
}

uint8_t drv_referee_read_chassis_power(drv_referee_chassis_power_t *power,
                                        uint32_t timeout_ms)
{
    uint32_t irq_state;
    uint8_t is_valid;

    if (!power) {
        return 0U;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    is_valid = s_robot_status_is_valid
               && (uint32_t)(HAL_GetTick() - s_robot_status_tick) <= timeout_ms;
    if (is_valid) {
        power->robot_level = s_referee_data.robot_status.robot_level;
        power->is_chassis_output_enabled =
            s_referee_data.robot_status.power_management_chassis_output;
        power->power_limit_w = s_referee_data.robot_status.chassis_power_limit;
    }
    __set_PRIMASK(irq_state);

    return is_valid;
}

uint32_t drv_referee_get_and_clear_flags(void)
{
    uint32_t flags = s_referee_data.data_valid_flags;
    s_referee_data.data_valid_flags = 0;
    return flags;
}

int drv_referee_is_data_updated(uint32_t flag)
{
    return (s_referee_data.data_valid_flags & flag) ? 1 : 0;
}

const char *drv_referee_parse_error_string(int result)
{
    switch (result) {
    case  0: return "Parse success";
    case -1: return "Length insufficient";
    case -2: return "Unknown command ID";
    case -3: return "SOF error (not 0xA5)";
    case -4: return "CRC mismatch";
    case -5: return "Frame too large";
    default: return "Unknown error";
    }
}


