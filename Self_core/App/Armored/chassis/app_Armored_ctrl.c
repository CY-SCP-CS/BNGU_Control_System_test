/**
 * @file    app_Armored_ctrl.c
 * @brief   重装底盘状态机、目标生成及关节与麦轮协调控制。
 */
#include "app_Armored_ctrl.h"
#include "drv_imu.h"
#include "Drv/Drv_dbus/drv_dbus.h"
#include "Drv/Drv_game_controller/drv_game_controller.h"
#include "Drv/Drv_power_measure/drv_power_measure.h"
#include "App/Armored/chassis/app_Armored_joint.h"
#include "App/Armored/chassis/app_Armored_mecanum.h"
#include "App/Armored/Armored_cfg.h"
#include "Lib/Lib_math/lib_math.h"
#include "drv_vofa.h"

#include <math.h>

#define STANDING_TIME_MAX 15000 //站立最长等待时间

/** 麦轮底盘控制坐标系。 */
typedef enum {
    DRIVE_MODE_BODY = 0, // 车体坐标系
    DRIVE_MODE_WORLD,    // 世界坐标系
} drive_mode_t;

/** 关节控制模式。 */
typedef enum {
    JOINT_MODE_VMC = 0,   // VMC 高度及姿态控制
    JOINT_MODE_POSITION, // 关节位置控制
} joint_normal_mode_t;

/** VMC 关节控制子状态。 */
typedef enum {
    JOINT_VMC_REST = 0,  // 休息：零扭矩
    JOINT_VMC_RISING,    // 起立：平滑升至站立高度
    JOINT_VMC_STANDING,  // 站立：VMC 支撑和姿态控制
    JOINT_VMC_LOWERING,  // 坐下：平滑降至休息高度
} normal_vmc_state_t;

/** 整车行为。 */
typedef enum {
    ARMORED_NORMAL = 0,
    ARMORED_ACTION,
} armored_behavior_t;

/** NORMAL 下的 MIT 连贯动作。 */
typedef enum {
    NORMAL_MIT_FRONT_LIFT = 0,   // 抬前腿，后腿同步回到 0 rad
    NORMAL_MIT_FRONT_REAR_FOLD,  // 收前腿和后腿
    NORMAL_MIT_REAR_LIFT,        // 前后腿到最后一步目标
    NORMAL_MIT_DONE,           // 完成后切回 VMC 并恢复正常高度
} normal_mit_state_t;

/** 额外动作类型。 */
typedef enum {
    ACTION_VMC_SHOW = 0,
    ACTION_MIT_SHOW,
} armored_action_t;

typedef struct {
    joint_normal_mode_t joint_mode;
    normal_vmc_state_t vmc_state;
    normal_mit_state_t mit_state;
}normal_mode_t; //正常模式的集合

typedef struct {
    armored_action_t arem_action;
}action_mode_t;

typedef struct {
    normal_mode_t normal_mode;
    action_mode_t action_mode;
}behavior_state_t;

/** NORMAL VMC 的姿态目标及休息高度。 */
typedef struct {
    float target_height_mm;
    float rest_height_mm;
    float target_pitch_deg;
    float target_roll_deg;
} normal_vmc_control_t;

/** NORMAL MIT 单步参数，角度顺序为 FL、RL、FR、RR。 */
typedef struct {
    joint_motor_id_t motor_mask;
    float target_pos[4];      // rad，只有 motor_mask 选中的目标参与更新
    float duration_s;         // 平滑时间，s
    float tolerance_rad;      // 到位容差，rad
    uint8_t configured;      // 目标角度等参数确认后才置 1；未配置步骤不得启动
} normal_mit_step_config_t;

/** NORMAL MIT 三步参数及执行标志；DONE 负责切回 VMC，不对应参数项。 */
typedef struct {
    normal_mit_step_config_t steps[NORMAL_MIT_DONE];
    uint8_t step_started;     // 当前步骤的平滑请求是否已成功提交
} normal_mit_control_t;

/** 上层控制状态及公共目标。 */
typedef struct {
    armored_behavior_t behavior; // 当前执行 NORMAL 或 ACTION

    normal_mode_t normal_mode;   // NORMAL 的模式和子状态
    action_mode_t action_mode;   // ACTION 的动作状态

    drive_mode_t drive_mode;

    float target_vx;             // m/s
    float target_vy;             // m/s
    float target_wz;             // rad/s

    normal_vmc_control_t normal_vmc;
    normal_mit_control_t normal_mit;

    uint32_t idle_time_ms;       // 日常站立的连续空闲时间
    uint32_t state_time_ms;      // 当前正在执行的子状态耗时
} armored_ctrl_t;

static armored_ctrl_t s_ctrl; //总状态机实例
static drv_imu_t *s_imu;
static float s_imu_elapsed_s;
static uint8_t s_imu_ready;
static drv_game_controller_buttons_t game_ctrler;
static const drv_dbus_data_t *dbus;
volatile drv_power_data_t s_power_rx;

static void armored_power_can_rx(uint32_t std_id, uint8_t *data, uint8_t len)
{
    if (std_id != DRV_POWER_CAN_ID || !data || len != 8U) {
        return;
    }

    drv_power_data_t decoded;
    drv_power_solve(data, &decoded);
    s_power_rx = decoded;
}

static void armored_ahrs_init(drv_imu_t *imu)
{
    s_imu = imu;
    s_imu_elapsed_s = 0.0f;
    s_imu_ready = 0U;
    if (imu) {
        drv_imu_set_mounting(imu, 0.0f, 0.0f, 0.0f);
        drv_imu_set_euler_mode(imu, DRV_IMU_EULER_ZYX);
    }
}

/** 每 1 ms 检查 DMA 快照，累计经过时间，有新样本才更新姿态。
 * dt:每次调用过去了多久
 */
static void armored_ahrs_update(float dt)
{
    if (!s_imu || !isfinite(dt) || dt <= 0.0f || dt > 0.1f) {
        return;
    }

    s_imu_elapsed_s = fminf(s_imu_elapsed_s + dt, 0.1f);
    const uint8_t has_new_snapshot = drv_imu_port_snapshot_update(s_imu);
    drv_imu_port_async_start();

    if (has_new_snapshot) {
        drv_imu_data_convert(s_imu);
        drv_imu_update(s_imu, s_imu_elapsed_s);
        drv_imu_quat_to_euler(s_imu);
        s_imu_elapsed_s = 0.0f;
        s_imu_ready = isfinite(s_imu->euler.yaw);
    }
}

static void Armored_read_dbus_data(void)
{
    dbus = drv_dbus_port_get_data();
}

static uint8_t Armored_read_game_controller_data(void)
{
    uint8_t is_valid = drv_game_controller_uart_read(&game_ctrler, 100U);
    return is_valid;
}

static void normal_mit_steps_init(void)
{
    // 目标角度顺序为 FL、RL、FR、RR，单位 rad。
    s_ctrl.normal_mit = (normal_mit_control_t){
        .steps = {
            [NORMAL_MIT_FRONT_LIFT] = {
                .motor_mask = JOINT_MOTOR_ALL,
                .target_pos = {-2.2814f, 1.6623f, 2.2868f, -1.6394f},
                .duration_s = 2.0f,
                .configured = 1U,
            },
            [NORMAL_MIT_FRONT_REAR_FOLD] = {
                .motor_mask = JOINT_MOTOR_ALL,
                .target_pos = {-0.9321f, 0.3511f, 0.9424f, -0.3317f},
                .duration_s = 1.0f,
                .configured = 1U,
            },
            [NORMAL_MIT_REAR_LIFT] = {
                .motor_mask = JOINT_MOTOR_ALL,
                .target_pos = {-1.3437f, 1.2751f, 1.3487f, -1.2549f},
                .duration_s = 1.0f,
                .configured = 1U,
            },
        },
        .step_started = 0U,
    };
}

static inline void normal_mit_steps_init2(void)
{
    // 第二套标定角度按照片顺序配置，关节顺序为 FL、RL、FR、RR，单位 rad。
    s_ctrl.normal_mit = (normal_mit_control_t){
        .steps = {
            [NORMAL_MIT_FRONT_LIFT] = {
                .motor_mask = JOINT_MOTOR_ALL,
                .target_pos = {-2.2814f, 1.6623f, 2.2868f, -1.6394f},
                .duration_s = 1.0f,
                .configured = 1U,
            },
            [NORMAL_MIT_FRONT_REAR_FOLD] = {
                .motor_mask = JOINT_MOTOR_ALL,
                .target_pos = {-1.2243f, 0.7940f, 1.1912f, -0.7574f},
                .duration_s = 1.0f,
                .configured = 1U,
            },
            [NORMAL_MIT_REAR_LIFT] = {
                .motor_mask = JOINT_MOTOR_ALL,
                .target_pos = {-1.4017f, 1.3193f, 1.3769f, -1.2919f},
                .duration_s = 1.0f,
                .configured = 1U,
            },
        },
        .step_started = 0U,
    };
}

void app_armored_ctrl_init(drv_imu_t *imu)
{
    if (!imu) {return;}
    armored_ahrs_init(imu);
    drv_power_port_can_init(&hcan2, DRV_POWER_CAN_ID, armored_power_can_rx);
    dbus = NULL;
    drv_dbus_port_init();
    drv_game_controller_uart_init(&huart1);
    joint_motor_init();
    mecnum_init();
    s_ctrl.behavior = ARMORED_NORMAL;
    s_ctrl.normal_mode.joint_mode = JOINT_MODE_VMC;
    s_ctrl.normal_mode.vmc_state = JOINT_VMC_REST;
    s_ctrl.normal_mode.mit_state = NORMAL_MIT_FRONT_LIFT;
    s_ctrl.target_vx = 0.0f;
    s_ctrl.target_vy = 0.0f;
    s_ctrl.target_wz = 0.0f;
    s_ctrl.normal_vmc.target_height_mm = 0.0f; //这个后面再改
    s_ctrl.normal_vmc.target_pitch_deg = 0.0f;
    s_ctrl.normal_vmc.target_roll_deg = 0.0f;

    s_ctrl.normal_vmc.rest_height_mm = joint_motor_get_height();  //有可能读不到

    normal_mit_steps_init();
}

void normal_vmc_state_execute(uint8_t count)
{
    switch (s_ctrl.normal_mode.vmc_state) {
    case JOINT_VMC_REST:
        joint_motor_zero_torque_ctrl(count);
        if (s_ctrl.target_vx || s_ctrl.target_vy || s_ctrl.target_wz) {
            if (joint_motor_height_move_request(JOINT_SMOOTH_HEIGHT, 1.0f)) {
                s_ctrl.normal_mode.vmc_state = JOINT_VMC_RISING;
                s_ctrl.state_time_ms = 0U;
            }
        }
        break;

    case JOINT_VMC_RISING:
        if (fabsf(s_ctrl.normal_vmc.target_height_mm - joint_motor_get_height()) <= 5 && !joint_motor_height_is_busy()) {
            s_ctrl.normal_mode.vmc_state = JOINT_VMC_STANDING;
            s_ctrl.state_time_ms = 0U;
            break;
        }
        joint_motor_vmc_mode_ctrl(count, s_imu->euler.pitch, s_imu->euler.roll,
                                 s_ctrl.normal_vmc.target_height_mm, s_ctrl.normal_vmc.target_pitch_deg,
                                 s_ctrl.normal_vmc.target_roll_deg, GRAVITY_ARMORED);
        break;

    case JOINT_VMC_STANDING:
        joint_motor_vmc_mode_ctrl(count, s_imu->euler.pitch, s_imu->euler.roll,
                                 s_ctrl.normal_vmc.target_height_mm, s_ctrl.normal_vmc.target_pitch_deg,
                                 s_ctrl.normal_vmc.target_roll_deg, GRAVITY_ARMORED);
        if (s_ctrl.target_vx || s_ctrl.target_vy || s_ctrl.target_wz) {
            s_ctrl.idle_time_ms = 0U;
        }
        if (s_ctrl.idle_time_ms >= STANDING_TIME_MAX) {
            if (joint_motor_height_move_request(JOINT_SMOOTH_HEIGHT, 1.0f)) {
                s_ctrl.normal_mode.vmc_state = JOINT_VMC_LOWERING;
                s_ctrl.state_time_ms = 0U;
            }
        }
        break;

    case JOINT_VMC_LOWERING:
        joint_motor_vmc_mode_ctrl(count, s_imu->euler.pitch, s_imu->euler.roll,
                                 s_ctrl.normal_vmc.rest_height_mm, s_ctrl.normal_vmc.target_pitch_deg,
                                 s_ctrl.normal_vmc.target_roll_deg, GRAVITY_ARMORED);
        if (fabsf(s_ctrl.normal_vmc.rest_height_mm - joint_motor_get_height()) <= 5 && !joint_motor_height_is_busy()) {
            s_ctrl.normal_mode.vmc_state = JOINT_VMC_REST;
            s_ctrl.state_time_ms = 0U;
            break;
        }
        break;
    }
}

typedef enum {
    MIT_STEP_SELECT_SEQUENTIAL,
    MIT_STEP_SELECT_DIRECT
} MIT_set_step_mode_t;

static uint8_t normal_mit_set_step(MIT_set_step_mode_t mode, normal_mit_state_t next)
{
    if (s_ctrl.normal_mode.joint_mode != JOINT_MODE_POSITION ||
        joint_motor_MIT_is_busy()) {
        return 0U;
    }

    if (mode == MIT_STEP_SELECT_SEQUENTIAL) {
        const unsigned int current = (unsigned int)s_ctrl.normal_mode.mit_state;
        if (current >= (unsigned int)NORMAL_MIT_DONE) {
            return 0U;
        }
        next = (normal_mit_state_t)(current + 1U);
    } else if (mode != MIT_STEP_SELECT_DIRECT) {
        return 0U;
    }

    // DONE 是结束状态，没有对应的 steps[] 配置。
    const unsigned int index = (unsigned int)next;
    if (index > (unsigned int)NORMAL_MIT_DONE ||
        (next != NORMAL_MIT_DONE && !s_ctrl.normal_mit.steps[index].configured)) {
        return 0U;
    }

    s_ctrl.normal_mode.mit_state = next;
    s_ctrl.normal_mit.step_started = 0U;
    s_ctrl.state_time_ms = 0U;
    return 1U;
}

void normal_MIT_state_execute(uint8_t count)
{
    const normal_mit_state_t state = s_ctrl.normal_mode.mit_state;
    if (state == NORMAL_MIT_DONE && !joint_motor_MIT_is_busy() &&
        s_imu && isfinite(s_imu->euler.pitch) && isfinite(s_imu->euler.roll) &&
        isfinite(joint_motor_get_height()) &&
        isfinite(s_ctrl.normal_vmc.target_height_mm) &&
        isfinite(s_ctrl.normal_vmc.target_pitch_deg) &&
        isfinite(s_ctrl.normal_vmc.target_roll_deg) &&
        joint_motor_height_move_request(JOINT_SMOOTH_HEIGHT, 1.0f)) {
        s_ctrl.normal_mit.step_started = 0U;
        s_ctrl.idle_time_ms = 0U;
        s_ctrl.state_time_ms = 0U;
        s_ctrl.normal_mode.vmc_state = JOINT_VMC_RISING;
        s_ctrl.normal_mode.joint_mode = JOINT_MODE_VMC;
        normal_vmc_state_execute(count);
        return;
    }

    // DONE 等待切换期间沿用 MIT 目标，继续推进尚未结束的轨迹。
    if ((unsigned int)state >= (unsigned int)NORMAL_MIT_DONE) {
        joint_motor_MIT_mode_ctrl(
            count, (joint_motor_id_t)0, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    const normal_mit_step_config_t *step = &s_ctrl.normal_mit.steps[state];

    // 参数已配置且本步尚未启动时，尝试提交一次平滑请求。
    if (!s_ctrl.normal_mit.step_started && step->configured) {
        if (joint_motor_MIT_move_request(step->motor_mask, step->duration_s)) {
            s_ctrl.normal_mit.step_started = 1U;
            s_ctrl.state_time_ms = 0U;
        }
    }

    // 请求未成功时保持旧目标，不直接写入新角度。
    if (!s_ctrl.normal_mit.step_started) {
        joint_motor_MIT_mode_ctrl(
            count, (joint_motor_id_t)0, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    // 持续执行本步；轨迹结束后保持目标，步骤切换由外部控制。
    joint_motor_MIT_mode_ctrl(
        count, step->motor_mask,
        step->target_pos[0], step->target_pos[1],
        step->target_pos[2], step->target_pos[3]);
}

void app_armored_ctrl_1khz(void)
{
    armored_ahrs_update(0.001f);
    // TODO: 更新输入，按 4 ms 节拍执行状态机及关节、麦轮控制。
}




/** 临时调试用 */

/* 临时调参直接复用底层电机实例。 */
extern M3508_wheel_drive MOTOR_DRIVE_FL, MOTOR_DRIVE_RL, MOTOR_DRIVE_FR, MOTOR_DRIVE_RR;
extern dm_joint_motor joint_FL, joint_RL, joint_FR, joint_RR;

volatile armored_ctrl_temp_t armored_ctrl_temp = {
    .wheel_kp = 42.0f,
    .wheel_ki = 16.0f,
    .wheel_max_iout = 500.0f,
    .joint_kp = 100.0f,
    .joint_kd = 3.0f,
    .joint_pos = {-1.0f, 1.0f, 1.0f, -1.0f}, // FL、RL、FR、RR，rad
};

// 0 直接使用目标角度作为起立对照；大于 0 启用角度过渡，单位 s。
#define ARMORED_TEMP_MIT_MOVE_DURATION_S 1.0f
#define ARMORED_TEMP_CTRL_PERIOD_MS 4U
#define ARMORED_TEMP_WHEEL_OFFSET_MS 2U
// 临时轮速输入来源，Monitor 模式直接使用手动写入的 vx、vy、wz。
#define ARMORED_TEMP_WHEEL_MONITOR 0
#define ARMORED_TEMP_WHEEL_DBUS 1
#define ARMORED_TEMP_WHEEL_GAMEPAD 2
#define ARMORED_TEMP_WHEEL_SOURCE ARMORED_TEMP_WHEEL_GAMEPAD
#define ARMORED_TEMP_DBUS_LINEAR_MAX 3.0f  // m/s
#define ARMORED_TEMP_DBUS_ANGULAR_MAX 2.0f // rad/s
#define ARMORED_TEMP_DBUS_DEADZONE 15.0f   // 摇杆原始值

typedef enum {
    TEMP_MIT_WAIT_FEEDBACK = 0,
    TEMP_MIT_DAMPING,
    TEMP_MIT_POSITION,
} temp_mit_mode_t;

static uint8_t s_temp_divider;
static uint8_t s_temp_send_count;
static temp_mit_mode_t s_temp_mit_mode;

void app_armored_ctrl_init_temp(drv_imu_t *imu)
{
    armored_ahrs_init(imu);
    drv_power_port_can_init(&hcan2, DRV_POWER_CAN_ID, armored_power_can_rx);
    if (ARMORED_TEMP_WHEEL_SOURCE == ARMORED_TEMP_WHEEL_DBUS) {
        drv_dbus_port_init();
    }
    drv_game_controller_uart_init(&huart1);
    mecnum_init();
    joint_motor_init();
    if (ARMORED_TEMP_MIT_MOVE_DURATION_S == 0.0f) {
        (void)joint_motor_MIT_mode_enter();
    }
    s_temp_divider = 0U;
    s_temp_send_count = 0U;
    s_ctrl.normal_mode.joint_mode = JOINT_MODE_POSITION;
    normal_mit_steps_init();
    s_temp_mit_mode = TEMP_MIT_WAIT_FEEDBACK;
}

static void armored_mit_ctrl_temp(uint8_t count, dm_joint_motor *const joints[4])
{
    const float targets[4] = {
        armored_ctrl_temp.joint_pos[0], armored_ctrl_temp.joint_pos[1],
        armored_ctrl_temp.joint_pos[2], armored_ctrl_temp.joint_pos[3],
    };
    if (ARMORED_TEMP_MIT_MOVE_DURATION_S == 0.0f) {
        (void)joint_motor_MIT_mode_ctrl(count, JOINT_MOTOR_ALL,
                                       targets[0], targets[1], targets[2], targets[3]);
        return;
    }

    if (!joint_motor_is_enabled()) {
        s_temp_mit_mode = TEMP_MIT_WAIT_FEEDBACK;
        (void)joint_motor_zero_torque_ctrl(count);
        return;
    }

    const temp_mit_mode_t mode = joints[0]->MIT_kp > 0.0f
        ? TEMP_MIT_POSITION : TEMP_MIT_DAMPING;
    if (mode != s_temp_mit_mode) {
        // 模式变化时用实际角度更新保持点，并清除上一次轨迹。
        if (!joint_motor_MIT_mode_enter()) {
            (void)joint_motor_zero_torque_ctrl(count);
            return;
        }
        s_temp_mit_mode = mode;
    }

    joint_motor_id_t mask = 0;
    if (mode == TEMP_MIT_POSITION && !joint_motor_MIT_is_busy()) {
        for (uint8_t i = 0U; i < 4U; ++i) {
            if (isfinite(targets[i]) && targets[i] != joints[i]->MIT_target_POS &&
                isfinite(joints[i]->dm_motor.feedback.POS)) {
                mask |= 1U << i;
            }
        }
        if (mask && !joint_motor_MIT_move_request(mask, ARMORED_TEMP_MIT_MOVE_DURATION_S)) {
            mask = 0;
        }
    }

    // 忙碌时用空掩码推进已受理的轨迹，结束后再受理最新目标。
    (void)joint_motor_MIT_mode_ctrl(count, mask, targets[0], targets[1], targets[2], targets[3]);
}

void app_armored_ctrl_vofa_send_temp(void)
{
    const float power_w = s_power_rx.power; // CH0 功率计测得的电池侧功率，W
    (void)drv_vofa_port_send_channels(&power_w, 1U);
}

/** 将 DBUS 摇杆映射到 [-1, 1]，回中死区输出为 0。 */
static float armored_dbus_axis_temp(uint16_t channel)
{
    const float centered = (float)channel - DRV_DBUS_CHANNEL_CENTER;
    const float magnitude = lib_remap_clamp(fabsf(centered), ARMORED_TEMP_DBUS_DEADZONE,
                                            DRV_DBUS_CHANNEL_RANGE, 0.0f, 1.0f);
    return copysignf(magnitude, centered);
}

static void armored_wheel_dbus_ctrl_temp(void)
{
    Armored_read_dbus_data();
    if (!dbus || dbus->rc.s1 != DRV_DBUS_SWITCH_MIDDLE) {
        (void)mecanum_stop();
        return;
    }

    const float vx = ARMORED_TEMP_DBUS_LINEAR_MAX * armored_dbus_axis_temp(dbus->rc.ch[3]);
    const float vy = -ARMORED_TEMP_DBUS_LINEAR_MAX * armored_dbus_axis_temp(dbus->rc.ch[2]);
    const float wz = -ARMORED_TEMP_DBUS_ANGULAR_MAX * armored_dbus_axis_temp(dbus->rc.ch[0]);
    if (dbus->rc.s2 == DRV_DBUS_SWITCH_MIDDLE) {
        (void)mecanum_ctrl_body(vx, vy, wz, 0.0f);
    } else if (dbus->rc.s2 == DRV_DBUS_SWITCH_UP && s_imu_ready && drv_imu_port_is_online(20U)) {
        (void)mecanum_ctrl_world(vx, vy, wz, s_imu->euler.yaw, 0.0f);
    } else {
        (void)mecanum_stop();
    }
}

static float armored_gamepad_axis(float value)
{
    return fabsf(value) <= 0.1f ? 0.0f : value;
}

/** 更新临时车体速度目标，返回 1 表示允许驱动。 */
static uint8_t armored_wheel_gamepad_update_temp(void)
{
    if (!Armored_read_game_controller_data() || !game_ctrler.lb) {
        armored_ctrl_temp.vx = 0.0f;
        armored_ctrl_temp.vy = 0.0f;
        armored_ctrl_temp.wz = 0.0f;
        return 0U;
    }

    armored_ctrl_temp.vx = -1.0f * armored_gamepad_axis(game_ctrler.ly); // m/s
    armored_ctrl_temp.vy = -1.0f * armored_gamepad_axis(game_ctrler.lx); // m/s
    armored_ctrl_temp.wz = -2.0f * armored_gamepad_axis(game_ctrler.rx); // rad/s
    return 1U;
}

static uint8_t armored_gamepad_a_pressed_temp(void)
{
    static uint8_t a_armed = 0U;

    if (!Armored_read_game_controller_data()) {
        a_armed = 0U;  // 恢复连接后，先读到松开再允许触发
        return 0U;
    }

    if (!game_ctrler.a) {
        a_armed = 1U;
        return 0U;
    }

    const uint8_t pressed = a_armed;
    a_armed = 0U;
    return pressed;
}

void app_armored_ctrl_1khz_temp(void)
{
    armored_ahrs_update(0.001f);

    if (++s_temp_divider == ARMORED_TEMP_CTRL_PERIOD_MS) {
        dm_joint_motor *joints[4] = {&joint_FL, &joint_RL, &joint_FR, &joint_RR};
        const float joint_kp = armored_ctrl_temp.joint_kp;
        const float joint_kd = armored_ctrl_temp.joint_kd;
        for (uint8_t i = 0U; i < 4U; ++i) {
            joints[i]->MIT_kp = isfinite(joint_kp) && joint_kp > 0.0f ? joint_kp : 0.0f;
            joints[i]->MIT_kd = isfinite(joint_kd) && joint_kd > 0.0f ? joint_kd : 0.0f;
        }
        static uint8_t s_temp_mit_sequence_active = 0U;
        const uint8_t a_pressed = armored_gamepad_a_pressed_temp();

        if (a_pressed && joint_motor_is_enabled() && !joint_motor_MIT_is_busy() &&
            (!s_temp_mit_sequence_active || s_ctrl.normal_mit.step_started)) {
            if (s_temp_mit_sequence_active && s_ctrl.normal_mode.mit_state == NORMAL_MIT_REAR_LIFT) {
                s_temp_mit_sequence_active = 0U;  // 回到定角 MIT
            } else if (normal_mit_set_step(
                           s_temp_mit_sequence_active ? MIT_STEP_SELECT_SEQUENTIAL : MIT_STEP_SELECT_DIRECT,
                           NORMAL_MIT_FRONT_LIFT)) {
                s_temp_mit_sequence_active = 1U;
            }
        }

        const uint8_t count = s_temp_send_count++;  //另一个计数器
        if (s_temp_mit_sequence_active) {
            normal_MIT_state_execute(count);
        } else {
            armored_mit_ctrl_temp(count, joints);
        }
    } else if (s_temp_divider == ARMORED_TEMP_CTRL_PERIOD_MS + ARMORED_TEMP_WHEEL_OFFSET_MS) {
        // 此时已进入下一个关节周期 2 ms，保留这段计数。
        s_temp_divider = ARMORED_TEMP_WHEEL_OFFSET_MS;
        M3508_wheel_drive *wheels[4] = {
            &MOTOR_DRIVE_FL, &MOTOR_DRIVE_RL, &MOTOR_DRIVE_FR, &MOTOR_DRIVE_RR
        };
        for (uint8_t i = 0U; i < 4U; ++i) {
            wheels[i]->pid.kp = armored_ctrl_temp.wheel_kp;
            wheels[i]->pid.ki = armored_ctrl_temp.wheel_ki;
            wheels[i]->pid.kd = armored_ctrl_temp.wheel_kd;
            wheels[i]->pid.max_iout = armored_ctrl_temp.wheel_max_iout;
        }
        if (ARMORED_TEMP_WHEEL_SOURCE == ARMORED_TEMP_WHEEL_DBUS) {
            armored_wheel_dbus_ctrl_temp();
        } else if (ARMORED_TEMP_WHEEL_SOURCE == ARMORED_TEMP_WHEEL_GAMEPAD &&
                   !armored_wheel_gamepad_update_temp()) {
            (void)mecanum_stop();
        } else {
            if (game_ctrler.rb) {
                (void)mecanum_ctrl_world(armored_ctrl_temp.vx, armored_ctrl_temp.vy, armored_ctrl_temp.wz,s_imu->euler.yaw, 0.0f);
            }
            else{ (void)mecanum_ctrl_body(armored_ctrl_temp.vx, armored_ctrl_temp.vy, armored_ctrl_temp.wz, 0.0f);}

        }
        app_armored_ctrl_vofa_send_temp();
    }
}
