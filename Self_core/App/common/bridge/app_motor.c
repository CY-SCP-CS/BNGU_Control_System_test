/**
 * @file    app_motor.c
 * @brief   电机管理桥接层实现
 */
#include "app_motor.h"

#include <string.h>

// ─── 私有类型 ────────────────────────────────────

typedef struct {
    drv_motor_data_t  data;
    uint32_t          can_id;         /* 回传 CAN ID (用于回调解耦) */
    uint32_t          last_tick;      /* 上次收到回传的时间戳 */
    uint8_t           online;         /* 0=离线, 1=在线 */
    int16_t           current_cmd;    /* 当前电流指令 (失控保护) */
    app_motor_type_t  type;           /* 协议类型 */
} motor_instance_t;

// ─── 私有变量 ────────────────────────────────────

static CAN_HandleTypeDef *s_hcan;
static motor_instance_t   s_motors[APP_MOTOR_MAX];
static uint8_t            s_motor_count;

// ─── 统一 CAN 接收回调 ────────────────────────────

static void motor_rx_callback(uint32_t std_id, uint8_t *data, uint8_t len)
{
    uint8_t i;

    (void)len;

    for (i = 0; i < s_motor_count; i++) {
        if (s_motors[i].can_id == std_id) {
            /* 按协议类型选择解析函数 */
            if (s_motors[i].type == APP_MOTOR_TYPE_LINGKONG)
                drv_motor_solve_lk_data(data, &s_motors[i].data);
            else
                drv_motor_solve_dji_data(data, &s_motors[i].data);

            s_motors[i].last_tick = HAL_GetTick();
            s_motors[i].online    = 1;
            return;
        }
    }
}

// ─── 接口实现 ─────────────────────────────────────

void app_motor_init(CAN_HandleTypeDef *hcan,
                    const app_motor_cfg_t *cfgs, uint8_t count)
{
    uint8_t i;

    if (count > APP_MOTOR_MAX) count = APP_MOTOR_MAX;
    s_hcan        = hcan;
    s_motor_count = count;

    memset(s_motors, 0, sizeof(s_motors));

    /* 保存配置 */
    for (i = 0; i < count; i++) {
        s_motors[i].can_id = cfgs[i].can_id;
        s_motors[i].type   = cfgs[i].type;
    }

    /* 注册统一回调 (监听所有电机 ID) */
    for (i = 0; i < count; i++) {
        bsp_can_register_rx_callback(hcan, cfgs[i].can_id, motor_rx_callback);
    }
}

int app_motor_set_current(uint8_t idx, int16_t current)
{
    if (idx >= s_motor_count) return -1;

    /* 失控保护: 离线 → 电流归零 */
    if (!s_motors[idx].online) {
        s_motors[idx].current_cmd = 0;
        return -1;
    }

    s_motors[idx].current_cmd = current;
    return 0;
}

void app_motor_send_frame(uint8_t group)
{
    uint8_t  frame[8];
    uint8_t  start = group * 4;
    uint32_t ctrl_id;
    uint8_t  i;

    if (!s_hcan) return;

    ctrl_id = (group == 0) ? APP_MOTOR_CTRL_GROUP0 : APP_MOTOR_CTRL_GROUP1;

    drv_motor_build_dji_frame_init(frame);

    for (i = 0; i < 4 && (start + i) < s_motor_count; i++) {
        uint8_t idx = start + i;
        if (s_motors[idx].online) {
            drv_motor_build_dji_frame_set(frame, i, s_motors[idx].current_cmd);
        }
        /* 离线: frame 已初始化为 0 */
    }

    bsp_can_send(s_hcan, ctrl_id, frame);
}

int app_motor_get_data(uint8_t idx, drv_motor_data_t *out)
{
    if (idx >= s_motor_count || !out) return -1;
    *out = s_motors[idx].data;
    return 0;
}

uint8_t app_motor_is_online(uint8_t idx)
{
    if (idx >= s_motor_count) return 0;
    return s_motors[idx].online;
}

uint8_t app_motor_get_count(void)
{
    return s_motor_count;
}

void app_motor_refresh_online(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  i;

    for (i = 0; i < s_motor_count; i++) {
        if (now - s_motors[i].last_tick > APP_MOTOR_TIMEOUT_MS) {
            s_motors[i].online = 0;
        }
    }
}
