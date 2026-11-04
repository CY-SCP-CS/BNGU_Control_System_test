//
// Created by 33140 on 2026/11/4.
//

#include "navigation_ctrl.h"
#include "App/common/communication/app_chassis_comm.h"
#include "App/Navigation/chassis.h"
#include "Drv/Drv_imu/drv_imu.h"
#include "Drv/Drv_dbus/drv_dbus.h"
#include "Lib/Lib_math/lib_math.h"

#include <math.h>

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

    navigation_state.frame = NAV_FRAME_BODY;
    navigation_state.speed_cmd = (navigation_speed_cmd_t){0};
    navigation_state.mode = DBUS_CTRL;
}

/** 将 DBUS 通道 364～1684 映射到 [-max_value, +max_value]，1024 对应 0。
 *  通道越界时限幅；max_value 必须为非负有限数，否则返回 0。
 */
static float navigation_dbus_channel_map(uint16_t channel, float max_value)
{
    if (!isfinite(max_value) || max_value < 0.0f) {return 0.0f;}

    float value = ((float)channel - DRV_DBUS_CHANNEL_CENTER) / DRV_DBUS_CHANNEL_RANGE;
    return lib_clamp(value, -1.0f, 1.0f) * max_value;
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
    if (!dbus) {
        navigation_state.speed_cmd = (navigation_speed_cmd_t){0};
        chassis_stop();
        return;
    }

    navigation_mode_choose();
    switch (navigation_state.mode) {
        case DBUS_CTRL:
            navigation_state.speed_cmd.vx =  navigation_dbus_channel_map(dbus->rc.ch[3],6);
            navigation_state.speed_cmd.vy =  navigation_dbus_channel_map(dbus->rc.ch[2],6);
            navigation_state.speed_cmd.wz =  navigation_dbus_channel_map(dbus->rc.ch[0],6);
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
            // 还没写，不急
            navigation_state.speed_cmd = (navigation_speed_cmd_t){0};
            chassis_stop();
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
