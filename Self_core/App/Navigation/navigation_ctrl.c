//
// Created by 33140 on 2026/11/4.
//

#include "navigation_ctrl.h"

#include "bsp_cfg.h"
#include "App/common/communication/app_chassis_comm.h"
#include "App/Navigation/chassis.h"
#include "Drv/Drv_imu/drv_imu.h"
#include "Drv/Drv_dbus/drv_dbus.h"
#include "Drv/Drv_game_controller/drv_game_controller.h"
#include "Drv/Drv_vofa/drv_vofa.h"
#include "Lib/Lib_math/lib_math.h"

#include <math.h>

#define NAV_GAME_CONTROLLER_DEADZONE 0.1f  // 归一化摇杆死区，范围内按零处理

typedef enum {
    DBUS_CTRL,
    GAME_CTRLER_CTRL,
    RECEIVE_CAN1_CTRL,
}navigation_ctrl_mode_t; //控制模式‘

typedef enum {
    NAV_FRAME_BODY = 0, // 车体坐标系
    NAV_FRAME_WORLD,    // 世界坐标系
} navigation_frame_t;

typedef struct {
    float vx;  // 向前速度，m/s
    float vy;  // 向左速度，m/s
    float wz;  // 逆时针角速度，rad/s
} navigation_speed_cmd_t;

typedef struct {
    navigation_frame_t frame;
    navigation_speed_cmd_t speed_cmd;
    navigation_ctrl_mode_t mode;
}navigation_state_t; //导航车状态机

static drv_imu_t *imu;
static float s_imu_elapsed_s;
static uint8_t s_imu_ready; // 已完成至少一次有效姿态
static app_chassis_comm_rx_t rx_command;
static const drv_dbus_data_t *dbus;
drv_game_controller_buttons_t game_ctrler;
static navigation_state_t navigation_state;


void navigation_ctrl_init(drv_imu_t *imu_instance)
{
    imu = imu_instance;
    dbus = NULL;
    rx_command = (app_chassis_comm_rx_t){0};
    s_imu_elapsed_s = 0.0f;
    s_imu_ready = 0U;

    if (!imu) {return;}

    drv_imu_set_mounting(imu,0,0,0);
    drv_imu_set_euler_mode(imu,DRV_IMU_EULER_ZYX);
    chassis_init();
    drv_dbus_port_init();
    drv_game_controller_uart_init(&huart1);

    navigation_state.frame = NAV_FRAME_BODY;
    navigation_state.speed_cmd = (navigation_speed_cmd_t){0};
    navigation_state.mode = DBUS_CTRL;
}

/** 将 [in_min, in_max] 线性映射到 [out_min, out_max]，输入越界时限幅。
 *  输入上限须大于下限，输出允许反向；非有限参数或无效输入区间返回 0。
 */
static float navigation_channel_map(float value, float in_min, float in_max,
                                    float out_min, float out_max)
{
    if (!isfinite(value) || !isfinite(in_min) || !isfinite(in_max) ||
        !isfinite(out_min) || !isfinite(out_max) || in_max <= in_min) {
        return 0.0f;
    }

    return lib_remap_clamp(value, in_min, in_max, out_min, out_max);
}

/** 摇杆回中死区；范围外保留原值。 */
static float navigation_game_controller_deadzone(float value)
{
    return fabsf(value) <= NAV_GAME_CONTROLLER_DEADZONE ? 0.0f : value;
}

/** 将毫米转换为米；同样适用于将 mm/s 转换为 m/s。 */
static float navigation_mm_to_m(float value_mm)
{
    return value_mm * 0.001f;
}

/** 每 1 ms 调用；累计经过时间，仅在取得新采样时更新姿态及欧拉角。 */
static void navigation_ahrs_update(float dt)
{
    if (!imu || !isfinite(dt) || dt <= 0.0f || dt > 0.1f) {
        return;
    }

    s_imu_elapsed_s = fminf(s_imu_elapsed_s + dt, 0.1f);
    uint8_t has_new_snapshot = drv_imu_port_snapshot_update(imu);
    drv_imu_port_async_start();

    if (has_new_snapshot) {
        drv_imu_data_convert(imu);
        drv_imu_update(imu, s_imu_elapsed_s);
        drv_imu_quat_to_euler(imu);
        s_imu_elapsed_s = 0.0f;

        s_imu_ready = isfinite(imu->euler.roll)
               && isfinite(imu->euler.pitch)&& isfinite(imu->euler.yaw);
    }
}

static void navigation_read_dbus_data(void)
{
    dbus = drv_dbus_port_get_data();
}

/** VOFA 通道 0～3：lx、ly、rx、ry；其余通道置零。 */
static void navigation_game_controller_vofa_send(void)
{
    float data[DRV_VOFA_CHANNEL_COUNT] = {0};
    data[0] = game_ctrler.lx;
    data[1] = game_ctrler.ly;
    data[2] = game_ctrler.rx;
    data[3] = game_ctrler.ry;
    (void)drv_vofa_port_send(data);
}

static void navigation_mode_choose(void)
{
    if (!dbus) {
        navigation_state.speed_cmd = (navigation_speed_cmd_t){0};
        chassis_stop();
        return;
    }

    switch (dbus->rc.s1) {
        case 3:
            navigation_state.mode = DBUS_CTRL; break;
        case 1:
            navigation_state.mode = GAME_CTRLER_CTRL; break;
        case 2:
            navigation_state.mode = RECEIVE_CAN1_CTRL; break;
        default:
             break;
    }

    if (dbus->rc.s1 == 1 || dbus->rc.s1 == 3) {
        switch (dbus->rc.s2) {
        case 3:
            navigation_state.frame = NAV_FRAME_BODY; break;
        case 1:
            navigation_state.frame = NAV_FRAME_WORLD; break;
        default:
            break;
        }
    }

}



static void navigation_set_speed_cmd(void)
{

    navigation_read_dbus_data();
    drv_game_controller_uart_process();
    uint8_t is_valid = drv_game_controller_uart_read(&game_ctrler, 100U);
    if (is_valid) {navigation_game_controller_vofa_send();}
    if (!dbus) {
        navigation_state.speed_cmd = (navigation_speed_cmd_t){0};
        chassis_stop();
        return;
    }

    navigation_mode_choose();
    switch (navigation_state.mode) {
        case DBUS_CTRL:
            navigation_state.speed_cmd.vx = navigation_channel_map(
                dbus->rc.ch[3], 364.0f, 1684.0f, -6.0f, 6.0f);
            navigation_state.speed_cmd.vy = navigation_channel_map(
                dbus->rc.ch[2], 364.0f, 1684.0f, 6.0f, -6.0f);
            navigation_state.speed_cmd.wz = navigation_channel_map(
                dbus->rc.ch[0], 364.0f, 1684.0f, 6.0f, -6.0f);
            if (navigation_state.frame == NAV_FRAME_BODY) {
                chassis_ctrl_body(navigation_state.speed_cmd.vx,
                navigation_state.speed_cmd.vy,navigation_state.speed_cmd.wz);}
            else if (navigation_state.frame == NAV_FRAME_WORLD) {
                if (!imu || !s_imu_ready || !drv_imu_port_is_online(20U)) {
                    navigation_state.speed_cmd = (navigation_speed_cmd_t){0};
                    chassis_stop();
                    return;
            }
            chassis_ctrl_world(navigation_state.speed_cmd.vx,
                navigation_state.speed_cmd.vy,navigation_state.speed_cmd.wz,imu->euler.yaw);}
            break;

        case GAME_CTRLER_CTRL:
            if (!is_valid) {
                navigation_state.speed_cmd = (navigation_speed_cmd_t){0};
                chassis_stop();
                return;
            }
            navigation_state.speed_cmd.vx =
                navigation_channel_map(
                    navigation_game_controller_deadzone(game_ctrler.ly),
                    -1.0f, 1.0f, 6.0f, -6.0f);
            navigation_state.speed_cmd.vy =
                navigation_channel_map(
                    navigation_game_controller_deadzone(game_ctrler.lx),
                    -1.0f, 1.0f, 6.0f, -6.0f);
            navigation_state.speed_cmd.wz =
                navigation_channel_map(
                    navigation_game_controller_deadzone(game_ctrler.rx),
                    -1.0f, 1.0f, 6.0f, -6.0f);
            if (navigation_state.frame == NAV_FRAME_BODY) {
                chassis_ctrl_body(navigation_state.speed_cmd.vx,
                    navigation_state.speed_cmd.vy,navigation_state.speed_cmd.wz);}
            else if (navigation_state.frame == NAV_FRAME_WORLD) {
                if (!imu || !s_imu_ready || !drv_imu_port_is_online(20U)) {
                    navigation_state.speed_cmd = (navigation_speed_cmd_t){0};
                    chassis_stop();
                    return;
                }
                chassis_ctrl_world(navigation_state.speed_cmd.vx,
                    navigation_state.speed_cmd.vy,navigation_state.speed_cmd.wz,imu->euler.yaw);}
            break;

        case RECEIVE_CAN1_CTRL:
        navigation_state.frame = NAV_FRAME_BODY;
        app_chassis_comm_read_rx(&rx_command);
        navigation_state.speed_cmd.vx =  navigation_mm_to_m(rx_command.speed.vx);
        navigation_state.speed_cmd.vy =  navigation_mm_to_m(rx_command.speed.vy);
        navigation_state.speed_cmd.wz =  rx_command.speed.vz * 0.001f;
        chassis_ctrl_body(navigation_state.speed_cmd.vx,
                navigation_state.speed_cmd.vy,navigation_state.speed_cmd.wz);
    }



}

void navigation_ctrl_1khz(uint8_t count)
{
    navigation_ahrs_update(0.001f);
    if (count % 5 == 0) {
        navigation_set_speed_cmd();
    }
}
