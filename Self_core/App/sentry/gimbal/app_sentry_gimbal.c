/**
 * @file    app_sentry_gimbal.c
 * @brief   哨兵云台控制实现：双 yaw 独立 PID、pitch 控制与发射机构控制
 * @note    AHRS、yaw 和 pitch 按 1 kHz 调度；发射机构按 200 Hz 调度。
 *          CAN2 发送本板电机电流，CAN1 交换板间命令和云台状态。
 */
#include "app_sentry_gimbal.h"

#include "lib_filter.h"
#include "lib_math.h"

#include "bsp_can.h"
#include "bsp_tim.h"

#include "drv_imu.h"
#include "drv_motor.h"

#include "drv_vofa.h"
#include "bsp_cfg.h"
#include "project_cfg.h"

#include "app_sentry_common.h"
#include "app_gimbal_comm.h"
#include "app_chassis_comm.h"

#include <math.h>
#include <string.h>

volatile app_sentry_gimbal_debug_t app_sentry_gimbal_debug;

static app_sentry_gimbal_motor_state_t s_motor_state;
static app_sentry_gimbal_pose_t s_pose;
static app_sentry_gimbal_sampling_t s_sampling;
static app_sentry_gimbal_pitch_ff_t s_pitch_ff = {
    .bias_lpf = {.cutoff_hz = 0.3f},
    .dynamic_lpf = {.cutoff_hz = 25.0f},
    .deadband = 0.05f,
    .gain = 3000.0f,
    .limit = 1900.0f,
};
static app_sentry_gimbal_command_state_t s_command_state;
static app_sentry_gimbal_chassis_state_t s_chassis_state = {
    .input_source = APP_GIMBAL_INPUT_CAN,
};
static app_sentry_gimbal_startup_t s_startup;
static app_sentry_gimbal_tx_stats_t s_chassis_tx_stats;
static app_sentry_gimbal_chassis_feedback_t s_chassis_feedback = {.timeout_ms = 20U};
static lib_pid_t s_pid_yaw_large;
static lib_pid_t s_pid_yaw_small;
static lib_pid_t s_pid_pitch;
static lib_pid_t s_pid_disc;
static lib_pid_t s_pid_fric_l;
static lib_pid_t s_pid_fric_r;

/* 仅单向发布副本；控制、通信及中断均只访问上面的独立状态。 */
static void gimbal_debug_publish(void)
{
    uint32_t irq_state = __get_PRIMASK();

    /* 仅复制中断共享状态时屏蔽中断。 */
    __disable_irq();
    app_sentry_gimbal_debug.motor = s_motor_state;
    app_sentry_gimbal_debug.chassis = s_chassis_state;
    app_sentry_gimbal_debug.chassis_feedback = s_chassis_feedback;
    __set_PRIMASK(irq_state);

    app_sentry_gimbal_debug.pose = s_pose;
    app_sentry_gimbal_debug.pitch_ff = s_pitch_ff;
    app_sentry_gimbal_debug.command = s_command_state;
    app_sentry_gimbal_debug.startup = s_startup;
    app_sentry_gimbal_debug.chassis_tx = s_chassis_tx_stats;
    app_sentry_gimbal_debug.pid.yaw_large = s_pid_yaw_large;
    app_sentry_gimbal_debug.pid.yaw_small = s_pid_yaw_small;
    app_sentry_gimbal_debug.pid.pitch = s_pid_pitch;
    app_sentry_gimbal_debug.pid.disc = s_pid_disc;
    app_sentry_gimbal_debug.pid.fric_l = s_pid_fric_l;
    app_sentry_gimbal_debug.pid.fric_r = s_pid_fric_r;
}

/** 按当前通道配置发送 VOFA 调试数据。 */
static void gimbal_vofa_send(void);
/** 初始化云台、发射机构 PID 及控制滤波器。 */
static void pid_init_all(void);
/** 清零电流和控制目标，复位 PID、滤波及启动状态。 */
static void gimbal_reset(void);
/** 清零发射机构电流并复位拨弹轮、摩擦轮 PID。 */
static void launcher_reset(void);

/** 根据控制源更新云台、底盘和发射机构目标。 */
static void gimbal_update_cmd(const app_gimbal_dbus_input_t *input);
/** 仅应用 updated_mask 标记的新 CAN 指令；rx 为接收数据快照。 */
static void gimbal_apply_can_cmd(const app_gimbal_comm_rx_t *rx,
                                 uint8_t updated_mask);
/** 更新自动巡扫目标，并在 pitch 可达边界处反向。 */
static void gimbal_scan_update(void);

/** 计算相机相对底盘的平面朝向，返回值单位 rad。 */
static float gimbal_coord_trans(void);
/** 通过 CAN1 发送遥控底盘命令或急停；CAN 控制源下跳过。 */
static void gimbal_send_chassis_cmd(void);

/** 根据 IMU 和关节反馈更新控制姿态、角速度及加速度前馈。 */
static void imu_fusion(void);
/** 读取 pitch 编码器相对机械零位的角度，保留电机原始方向，单位 rad。 */
static float pitch_encoder_angle(void);

/** 计算双 yaw 电机电流，并处理小 yaw 机械限位。 */
static void yaw_pid_control(void);
/** 计算 pitch 电流，应用前馈及对地、关节限位。 */
static void pitch_control(void);
/** 根据发射模式计算拨弹轮和摩擦轮电流。 */
static void launch_control(void);

/** 通过 CAN2 发送大 yaw、小 yaw 和 pitch 电机电流。 */
static void gimbal_can2_tx(void);
/** 通过 CAN2 发送摩擦轮和拨弹轮电机电流。 */
static void launcher_can2_tx(void);
/** 通过 CAN1 发送云台角度、角速度及姿态四元数。 */
static void gimbal_can1_tx(void);

/** CAN2 电机反馈回调：解析反馈并更新接收新鲜度。 */
static void on_motor_feedback(uint32_t std_id, uint8_t *data, uint8_t len);
/** CAN1 底盘角速度回调：校验数据并记录接收时间。 */
static void on_chassis_omega_feedback(uint32_t std_id, uint8_t *data, uint8_t len);
void app_sentry_gimbal_init(drv_imu_t *imu)
{
    s_sampling.instance = imu;
    s_sampling.ahrs_elapsed_s = 0.0f;
    s_sampling.is_valid = 0U;
    s_sampling.ahrs_tick_ms = 0U;
    s_motor_state.rx_valid_mask = 0U;
    s_command_state.can.pose_is_valid = 0U;
    s_command_state.scan.is_active = 0U;
    if (imu) {
        /* 本车的机械安装方向由本模块配置；驱动用坐标旋转补偿 */
        drv_imu_set_mounting(imu, 0.0f, -90.0f, 0.0f);
        /* 云台轴映射依赖此分解约定；不使用驱动的其他欧拉角模式。 */
        drv_imu_set_euler_mode(imu, DRV_IMU_EULER_XYZ);
    }
    pid_init_all();
    gimbal_reset();

    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_L_YAW,
                                 on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_S_YAW,
                                 on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_PITCH,
                                 on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_F1,
                                 on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_D1,
                                 on_motor_feedback);
    bsp_can_rx_reg(&hcan2, SENTRY_CAN_GIMBAL_F2,
                                 on_motor_feedback);
    bsp_can_rx_reg(&hcan1, APP_CHASSIS_CAN_ID_OMEGA_FEEDBACK,
                                 on_chassis_omega_feedback);
    gimbal_debug_publish();
}

void app_gimbal_ahrs_update(float dt)
{
    uint8_t has_new_snapshot;

    if (!s_sampling.instance || !isfinite(dt) || dt <= 0.0f || dt > 0.1f) {
        return;
    }
    s_sampling.ahrs_elapsed_s = fminf(s_sampling.ahrs_elapsed_s + dt, 0.1f);
    has_new_snapshot = drv_imu_port_snapshot_update(s_sampling.instance);
    drv_imu_port_async_start();
    if (has_new_snapshot) {
        drv_imu_data_convert(s_sampling.instance);
        drv_imu_update(s_sampling.instance, s_sampling.ahrs_elapsed_s);
        s_sampling.is_valid = 1U;
        s_sampling.ahrs_tick_ms = bsp_tim_get_tick_ms();
        s_sampling.ahrs_elapsed_s = 0.0f;
    }
}

void app_sentry_gimbal_ctrl_1khz(void)
{
    app_gimbal_dbus_input_t input = {
        /* 未连接 DBUS 时默认使用 CAN，姿态命令空闲时允许自动巡扫。 */
        .source = APP_GIMBAL_INPUT_CAN,
    };
    uint8_t feedback_is_ready = 1U;
    uint32_t now;
    uint32_t irq_state = __get_PRIMASK();

    __disable_irq();
    memcpy(s_motor_state.feedback, s_motor_state.rx, sizeof(s_motor_state.feedback));
    now = bsp_tim_get_tick_ms();
    for (uint8_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i) {
        if ((s_motor_state.rx_valid_mask & (1U << i)) == 0U
            || (uint32_t)(now - s_motor_state.rx_tick_ms[i])
                >= SENTRY_GIMBAL_FEEDBACK_TIMEOUT_MS) {
            feedback_is_ready = 0U;
            break;
        }
    }
    __set_PRIMASK(irq_state);

    (void)app_gimbal_comm_dbus_rx(&input);
    /* 首帧尚未到达的全零结构体也是有限数，不能据此使能闭环。 */
    if (!feedback_is_ready || !s_sampling.instance
        || !s_sampling.is_valid
        || (uint32_t)(now - s_sampling.ahrs_tick_ms)
            >= SENTRY_GIMBAL_FEEDBACK_TIMEOUT_MS) {
        gimbal_reset();
        gimbal_can2_tx();
        gimbal_debug_publish();
        return;
    }

    s_motor_state.pitch_cur_speed = lib_lpf_update(
        &s_motor_state.pitch_speed_lpf,
        -lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_PITCH].speed),
        0.001f);

    imu_fusion();

    if (!isfinite(s_pose.euler.yaw) || !isfinite(s_pose.euler.pitch)
        || !isfinite(s_pose.yaw_rate) || !isfinite(s_motor_state.pitch_cur_speed)) {
        gimbal_reset();

        gimbal_can2_tx();
        gimbal_debug_publish();
        return;
    }
    /* 新鲜采样不等于滤波历史已建立；VQF 启动平均期间只更新姿态，不使能电机。 */
    if (!drv_imu_filter_is_ready(s_sampling.instance)) {
        gimbal_reset();
        gimbal_can2_tx();
        gimbal_debug_publish();
        return;
    }
    if (input.source == APP_GIMBAL_INPUT_ESTOP) {
        gimbal_update_cmd(&input);
        gimbal_reset();
        gimbal_can2_tx();
        gimbal_debug_publish();
        return;
    }
    if (!s_startup.is_ready) {
        if (!s_startup.is_settling) {
            s_startup.is_settling = 1U;
            s_startup.ready_tick_ms = now;
        }
        if ((uint32_t)(now - s_startup.ready_tick_ms)
            < SENTRY_GIMBAL_STARTUP_WAIT_MS) {
            gimbal_can2_tx();
            gimbal_debug_publish();
            return;
        }
        s_command_state.yaw_tar_angle = s_pose.euler.yaw;
        s_command_state.pitch_tar_angle = s_pose.euler.pitch;
        lib_pid_reset(&s_pid_yaw_large);
        lib_pid_reset(&s_pid_yaw_small);
        lib_pid_reset(&s_pid_pitch);
        s_startup.is_ready = 1U;
        /* 与急停恢复相同：先对齐真实姿态，下个周期再接收命令并输出。 */
        gimbal_can2_tx();
        gimbal_debug_publish();
        return;
    }
    gimbal_update_cmd(&input);

    yaw_pid_control();

    pitch_control();

    gimbal_can2_tx();
    gimbal_debug_publish();
}

uint8_t app_sentry_gimbal_is_ready(void)
{
    return s_startup.is_ready && !s_chassis_state.emergency_stop;
}

void app_sentry_launcher_ctrl_200hz(void)
{
    if (!s_startup.is_ready || s_chassis_state.emergency_stop) {
        launcher_reset();

    } else {
        launch_control();
    }
    launcher_can2_tx();

    gimbal_vofa_send();
    gimbal_debug_publish();
}

void app_sentry_gimbal_status_tx_200hz(void)
{
    /* 姿态反馈由固定周期调度，不依赖小电脑/CAN 控制源或发射机构状态。 */
    gimbal_can1_tx();
    gimbal_debug_publish();
}

void app_sentry_gimbal_chassis_tx_200hz(void)
{
    gimbal_send_chassis_cmd();
    gimbal_debug_publish();
}

static void on_motor_feedback(uint32_t std_id, uint8_t *data,
                                      uint8_t len)
{
    if (!data || len != 8U) {
        return;
    }
    int motor_index = -1;
    switch (std_id) {
    case SENTRY_CAN_GIMBAL_L_YAW:  motor_index = MOTOR_YAW_L;   break;
    case SENTRY_CAN_GIMBAL_S_YAW:  motor_index = MOTOR_YAW_S;   break;
    case SENTRY_CAN_GIMBAL_PITCH:  motor_index = MOTOR_PITCH;   break;
    case SENTRY_CAN_GIMBAL_F1:  motor_index = MOTOR_FRIC_L;  break;
    case SENTRY_CAN_GIMBAL_D1:  motor_index = MOTOR_DISC;    break;
    case SENTRY_CAN_GIMBAL_F2:  motor_index = MOTOR_FRIC_R;  break;
    default: return;
    }
    if (motor_index >= 0 && motor_index < GIMBAL_MOTOR_COUNT) {
        uint16_t angle = (uint16_t)((uint16_t)data[0] << 8 | data[1]);
        if (angle >= 8192U) {
            return;
        }
        drv_motor_data_t decoded = {0};
        drv_motor_solve_dji(data, &decoded);
        s_motor_state.rx[motor_index].angle = decoded.angle;
        s_motor_state.rx[motor_index].speed = decoded.speed;
        s_motor_state.rx[motor_index].current = decoded.current;
        if (motor_index < GIMBAL_AXIS_COUNT) {
            s_motor_state.rx_tick_ms[motor_index] = bsp_tim_get_tick_ms();
            s_motor_state.rx_valid_mask |= (uint8_t)(1U << motor_index);
        }
    }
}

static void on_chassis_omega_feedback(uint32_t std_id, uint8_t *data,
                                      uint8_t len)
{
    float omega_z;

    (void)std_id;
    if (!data || len < sizeof(omega_z)) {
        return;
    }
    memcpy(&omega_z, data, sizeof(omega_z));
    if (!isfinite(omega_z)
        || fabsf(omega_z) > SENTRY_CHASSIS_VW_TAR_SPEED_MAX * 1.5f) {
        return;
    }

    s_chassis_feedback.vw_cur_speed = omega_z;
    s_chassis_feedback.vw_rx_tick_ms = bsp_tim_get_tick_ms();
    s_chassis_feedback.vw_is_valid = 1U;
}

/* 私有函数实现。 */

static void pid_init_all(void)
{
    s_pitch_ff.current = 0.0f;
    lib_pid_init(&s_pid_yaw_large, 29000.0f, 2000.0f, 7300.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_yaw_small, 32000.0f, 2100.0f, 2100.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_pitch, 26000.0f, 1000.0f, 900.0f, 3800.0f, 0.0f,
                 5000.0f, 0.0f, DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX, 1000.0f);
    /* 拨弹轮前馈增益为 1，电流指令限幅为 ±3000。 */
    lib_pid_init(&s_pid_disc, 85.0f, 150.0f, 0.0f, 1.0f, 0.0f,
                 3000.0f, 0.0f, DRV_MOTOR_M2006_CURRENT_MIN, DRV_MOTOR_M2006_CURRENT_MAX, 7000.0f);
    lib_pid_init(&s_pid_fric_l, 15.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX, 1000.0f);
    lib_pid_init(&s_pid_fric_r, 15.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, DRV_MOTOR_M3508_CURRENT_MIN, DRV_MOTOR_M3508_CURRENT_MAX, 1000.0f);

    lib_lpf_init(&s_sampling.yaw_rate_lpf, 25.0f);
    lib_lpf_init(&s_motor_state.pitch_speed_lpf, 25.0f);
    lib_lpf_init(&s_pitch_ff.bias_lpf, s_pitch_ff.bias_lpf.cutoff_hz);
    lib_lpf_init(&s_pitch_ff.dynamic_lpf, s_pitch_ff.dynamic_lpf.cutoff_hz);
}
static void launcher_reset(void)
{
    s_motor_state.current[MOTOR_FRIC_L] = 0;
    s_motor_state.current[MOTOR_FRIC_R] = 0;
    s_motor_state.current[MOTOR_DISC] = 0;
    lib_pid_reset(&s_pid_disc);
    lib_pid_reset(&s_pid_fric_l);
    lib_pid_reset(&s_pid_fric_r);
}

static void gimbal_reset(void)
{
    memset(s_motor_state.current, 0, sizeof(s_motor_state.current));
    s_command_state.data.fire = APP_SENTRY_FIRE_OFF;
    s_command_state.data.yaw_tar_speed = 0.0f;
    s_command_state.data.pitch_tar_speed = 0.0f;
    s_chassis_state.vx_tar_speed = 0;
    s_chassis_state.vy_tar_speed = 0;
    s_chassis_state.vw_tar_speed = 0;
    s_chassis_state.input_source = APP_GIMBAL_INPUT_ESTOP;
    s_chassis_state.emergency_stop = 1U;
    s_startup.is_ready = 0U;
    s_pitch_ff.bias_lpf.initialized = 0U;
    s_pitch_ff.bias_lpf.out = 0.0f;
    s_pitch_ff.dynamic_lpf.initialized = 0U;
    s_pitch_ff.dynamic_lpf.out = 0.0f;
    s_pitch_ff.current = 0.0f;
    s_startup.is_settling = 0U;
    s_command_state.can.fire_override = 0U;
    s_command_state.can.pose_is_valid = 0U;
    s_command_state.scan.is_active = 0U;
    lib_pid_reset(&s_pid_yaw_large);
    lib_pid_reset(&s_pid_yaw_small);
    lib_pid_reset(&s_pid_pitch);
    lib_lpf_init(&s_sampling.yaw_rate_lpf, 25.0f);
    lib_lpf_init(&s_motor_state.pitch_speed_lpf, 25.0f);
    launcher_reset();
}
static void gimbal_update_cmd(const app_gimbal_dbus_input_t *input)
{
    app_gimbal_comm_rx_t can_rx;
    uint8_t updated_mask = 0U;

    if (!input) {
        return;
    }

    /* 非 CAN 模式也清空更新标志，防止切换控制源时执行旧指令。 */
    (void)app_gimbal_comm_read_rx(&can_rx, &updated_mask);

    if (s_chassis_state.input_source != input->source) {
        /* 切换后等待新 CAN 帧；遥控模式则立即按当前 S1 挡位控制发射。 */
        updated_mask = 0U;
        s_command_state.data.fire = APP_SENTRY_FIRE_OFF;
        s_command_state.can.fire_override = 0U;
        s_command_state.can.pose_is_valid = 0U;
        s_command_state.scan.is_active = 0U;
        s_command_state.data.yaw_tar_speed = 0.0f;
        s_command_state.data.pitch_tar_speed = 0.0f;
        s_command_state.yaw_tar_angle = s_pose.euler.yaw;
        s_command_state.pitch_tar_angle = s_pose.euler.pitch;
        lib_pid_reset(&s_pid_yaw_large);
        lib_pid_reset(&s_pid_yaw_small);
        lib_pid_reset(&s_pid_pitch);
        launcher_reset();
    }
    s_chassis_state.input_source = input->source;
    s_chassis_state.emergency_stop = (input->source == APP_GIMBAL_INPUT_ESTOP);
    if (s_chassis_state.emergency_stop) {
        s_chassis_state.vx_tar_speed = 0;
        s_chassis_state.vy_tar_speed = 0;
        s_chassis_state.vw_tar_speed = 0;
        return;
    }
    if (input->source == APP_GIMBAL_INPUT_CAN) {
        const uint8_t pose_mask = APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT
                               | APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT
                               | APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT
                               | APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT;
        s_command_state.data.yaw_tar_speed = 0.0f;
        s_command_state.data.pitch_tar_speed = 0.0f;
        if ((updated_mask & pose_mask) != 0U) {
            /* 增量命令从当前实际姿态接管，不能继续累加巡扫的跟踪误差。 */
            if (s_command_state.scan.is_active) {
                s_command_state.yaw_tar_angle = s_pose.euler.yaw;
                s_command_state.pitch_tar_angle = s_pose.euler.pitch;
                lib_pid_reset(&s_pid_yaw_large);
                lib_pid_reset(&s_pid_yaw_small);
                lib_pid_reset(&s_pid_pitch);
            }
            s_command_state.can.pose_tick_ms = bsp_tim_get_tick_ms();
            s_command_state.can.pose_is_valid = 1U;
            s_command_state.scan.is_active = 0U;
        }
        gimbal_apply_can_cmd(&can_rx, updated_mask);
        if (SENTRY_GIMBAL_SCAN_ENABLE
            && (!s_command_state.can.pose_is_valid
                || (uint32_t)(bsp_tim_get_tick_ms() - s_command_state.can.pose_tick_ms)
                    >= SENTRY_GIMBAL_CAN_POSE_TIMEOUT_MS)) {
            gimbal_scan_update();
        }
        return;
    }

    s_command_state.data.mode = APP_SENTRY_GIMBAL_MODE_SPEED;
    s_command_state.data.fire = input->launcher_mode;
    s_command_state.data.yaw_tar_speed = -input->yaw_rate_norm * SENTRY_GIMBAL_YAW_TAR_SPEED_MAX;
    s_command_state.data.pitch_tar_speed = -input->pitch_rate_norm * SENTRY_GIMBAL_PITCH_TAR_SPEED_MAX;

    float vx_gimbal_tar_speed = input->chassis_vx_norm * SENTRY_CHASSIS_VX_TAR_SPEED_MAX;
    float vy_gimbal_tar_speed = input->chassis_vy_norm * SENTRY_CHASSIS_VY_TAR_SPEED_MAX;
    float vx_chassis_tar_speed;
    float vy_chassis_tar_speed;
    app_sentry_gimbal_velocity_to_chassis(vx_gimbal_tar_speed, vy_gimbal_tar_speed,
                                          &vx_chassis_tar_speed, &vy_chassis_tar_speed);
    s_chassis_state.vx_tar_speed = (int16_t)lroundf(vx_chassis_tar_speed);
    s_chassis_state.vy_tar_speed = (int16_t)lroundf(vy_chassis_tar_speed);
    s_chassis_state.vw_tar_speed = (int16_t)lroundf(input->chassis_omega_norm
        * SENTRY_CHASSIS_VW_TAR_SPEED_MAX / APP_CHASSIS_OMEGA_RAD_S_PER_LSB);

    s_command_state.yaw_tar_angle += s_command_state.data.yaw_tar_speed * 0.001f;
    s_command_state.pitch_tar_angle += s_command_state.data.pitch_tar_speed * 0.001f;
    s_command_state.yaw_tar_angle = lib_rad_norm(s_command_state.yaw_tar_angle);
}

static void gimbal_scan_update(void)
{
    if (!s_command_state.scan.is_active) {
        s_command_state.yaw_tar_angle = s_pose.euler.yaw;
        s_command_state.pitch_tar_angle = s_pose.euler.pitch;
        s_command_state.scan.pitch_increasing = 0U;
        s_command_state.scan.is_active = 1U;
        lib_pid_reset(&s_pid_yaw_large);
        lib_pid_reset(&s_pid_yaw_small);
        lib_pid_reset(&s_pid_pitch);
        return; /* 切入时只对齐目标，下个控制周期再推进。 */
    }

    s_command_state.data.mode = APP_SENTRY_GIMBAL_MODE_SPEED;
    s_command_state.data.yaw_tar_speed = SENTRY_GIMBAL_SCAN_YAW_SPEED;
    s_command_state.yaw_tar_angle = lib_rad_norm(s_command_state.yaw_tar_angle
        + SENTRY_GIMBAL_SCAN_YAW_SPEED * 0.001f);

    float platform_pitch = s_pose.euler.pitch + pitch_encoder_angle();
    float pitch_min = fmaxf(SENTRY_GIMBAL_SCAN_PITCH_MIN,
                            platform_pitch - SENTRY_GIMBAL_PITCH_JOINT_MAX);
    float pitch_max = fminf(SENTRY_GIMBAL_SCAN_PITCH_MAX,
                            platform_pitch - SENTRY_GIMBAL_PITCH_JOINT_MIN);
    if (pitch_min >= pitch_max) {
        /* 对地与关节范围没有交集时，交给 pitch_control 的现有限位处理。 */
        s_command_state.pitch_tar_angle = s_pose.euler.pitch;
        return;
    }

    float pitch_target = s_command_state.pitch_tar_angle
        + (s_command_state.scan.pitch_increasing ? SENTRY_GIMBAL_SCAN_PITCH_SPEED : -SENTRY_GIMBAL_SCAN_PITCH_SPEED) * 0.001f;
    if (pitch_target <= pitch_min) {
        pitch_target = pitch_min;
        s_command_state.scan.pitch_increasing = 1U;
    } else if (pitch_target >= pitch_max) {
        pitch_target = pitch_max;
        s_command_state.scan.pitch_increasing = 0U;
    }
    s_command_state.pitch_tar_angle = pitch_target;
    s_command_state.data.pitch_tar_speed =
        (s_command_state.scan.pitch_increasing ? SENTRY_GIMBAL_SCAN_PITCH_SPEED : -SENTRY_GIMBAL_SCAN_PITCH_SPEED);
}

static void gimbal_apply_can_cmd(const app_gimbal_comm_rx_t *rx,
                                 uint8_t updated_mask)
{
    if (!rx) {
        return;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT) {
        s_command_state.yaw_tar_angle = lib_rad_norm(
            s_command_state.yaw_tar_angle + rx->speed_no_shoot.yaw_inc);
        s_command_state.pitch_tar_angle += rx->speed_no_shoot.pitch_inc;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT) {
        s_command_state.yaw_tar_angle = lib_rad_norm(
            s_command_state.yaw_tar_angle + rx->speed_shoot.yaw_inc);
        s_command_state.pitch_tar_angle += rx->speed_shoot.pitch_inc;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT) {
        s_command_state.yaw_tar_angle = lib_rad_norm(rx->angle_no_shoot.yaw_abs);
        s_command_state.pitch_tar_angle = rx->angle_no_shoot.pitch_abs;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT) {
        s_command_state.yaw_tar_angle = lib_rad_norm(rx->angle_shoot.yaw_abs);
        s_command_state.pitch_tar_angle = rx->angle_shoot.pitch_abs;
    }
    if (!s_command_state.can.fire_override
        && (updated_mask & APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT)) {
        s_command_state.data.fire = APP_SENTRY_FIRE_ON;
    }
    if (updated_mask & APP_GIMBAL_COMM_UPDATE_SHOOT) {
        /* 显式发射指令保持有效，直到再次收到 0x129 或退出当前 CAN 模式。 */
        s_command_state.can.fire_override = 1U;
        if (rx->shoot.shoot_switch == 0xFFU && rx->shoot.retreat == 0x00U) s_command_state.data.fire = APP_SENTRY_FIRE_ON;
        else if (rx->shoot.shoot_switch == 0xFFU && rx->shoot.retreat == 0xFFU) s_command_state.data.fire = APP_SENTRY_FIRE_FIR;
        else if (rx->shoot.shoot_switch == 0x00U && rx->shoot.retreat == 0xFFU) s_command_state.data.fire = APP_SENTRY_FIRE_REVERSE;
        else s_command_state.data.fire = APP_SENTRY_FIRE_OFF;
    }
}

static float gimbal_coord_trans(void)
{
    float large_yaw_cur_angle = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_YAW_L].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_L_YAW_ZERO, LIB_ENC13_TO_RAD));
    float small_yaw_cur_angle = lib_get_shortest_path(
        lib_enc_conv((float)s_motor_state.feedback[MOTOR_YAW_S].angle, LIB_ENC13_TO_RAD),
        lib_enc_conv((float)SENTRY_GIMBAL_S_YAW_ZERO, LIB_ENC13_TO_RAD));

    return lib_rad_norm(
        -SENTRY_GIMBAL_L_YAW_DIR * large_yaw_cur_angle
        - SENTRY_GIMBAL_S_YAW_DIR * small_yaw_cur_angle);
}

void app_sentry_gimbal_velocity_to_chassis(float vx_gimbal, float vy_gimbal,
                                           float *vx_chassis, float *vy_chassis)
{
    if (!vx_chassis || !vy_chassis) {
        return;
    }

    float yaw_rel = gimbal_coord_trans();
    float cos_yaw = cosf(yaw_rel);
    float sin_yaw = sinf(yaw_rel);
    /* 两个平移轴的实车方向都与命令相反，统一反转转换后的平面速度。 */
    *vx_chassis = -(cos_yaw * vx_gimbal + sin_yaw * vy_gimbal);
    *vy_chassis = -(sin_yaw * vx_gimbal - cos_yaw * vy_gimbal);
}

static void gimbal_send_chassis_cmd(void)
{
    uint8_t data[8] = {0};
    if (s_chassis_state.input_source == APP_GIMBAL_INPUT_CAN) {
        s_chassis_tx_stats.tx_can_skip_count++;
        return;
    }
    int16_t vx = s_chassis_state.emergency_stop ? SENTRY_CHASSIS_ESTOP
                                           : s_chassis_state.vx_tar_speed;

    memcpy(data, &vx, sizeof(vx));
    memcpy(data + 2, &s_chassis_state.vy_tar_speed, sizeof(s_chassis_state.vy_tar_speed));
    memcpy(data + 4, &s_chassis_state.vw_tar_speed, sizeof(s_chassis_state.vw_tar_speed));
    s_chassis_tx_stats.tx_attempt_count++;
    uint8_t result = bsp_can_tx(&hcan1, APP_CHASSIS_CAN_ID_SPEED_CMD, data);
    s_chassis_tx_stats.last_tx_result = result;
    if (result == 0U) {
        s_chassis_tx_stats.tx_accept_count++;
    } else {
        s_chassis_tx_stats.tx_fail_count++;
    }
}

static float pitch_encoder_angle(void)
{
    return lib_enc13_relative_rad(
        s_motor_state.feedback[MOTOR_PITCH].angle, SENTRY_GIMBAL_PITCH_ZERO);
}

static void imu_fusion(void)
{
    if (!s_sampling.instance) {
        return;
    }
    drv_imu_quat_to_euler(s_sampling.instance);

    s_pose.euler.yaw = lib_deg_to_rad(s_sampling.instance->euler.yaw);

    s_pose.euler.pitch = lib_deg_to_rad(s_sampling.instance->euler.roll)
        - pitch_encoder_angle();
    /* 此处 euler 已消除固定竖装角：等效水平 IMU x 对应云台 y，y 对应云台 -x。
     * 默认 IMU 欧拉角分解映射到云台 ZYX 后，roll = -IMU pitch。
     * pitch 按平台倾角减去俯仰关节编码器相对零位角进行补偿。 */
    s_pose.euler.roll = -lib_deg_to_rad(s_sampling.instance->euler.pitch);

    drv_imu_real_t body_gyro;
    drv_imu_get_body_gyro(s_sampling.instance, &body_gyro);
    s_pose.yaw_rate = lib_lpf_update(&s_sampling.yaw_rate_lpf, body_gyro.z, 0.001f);

    drv_imu_real_t body_acc;
    drv_imu_get_body_acc(s_sampling.instance, &body_acc);
    const float acc_y_raw = body_acc.y;
    const float acc_bias = lib_lpf_update(
        &s_pitch_ff.bias_lpf, acc_y_raw, 0.001f);
    const float acc_dynamic = acc_y_raw - acc_bias;
    const float acc_smoothed = lib_lpf_update(
        &s_pitch_ff.dynamic_lpf, acc_dynamic, 0.001f);

    const float acc_deadband = isfinite(s_pitch_ff.deadband)
        ? lib_clamp(s_pitch_ff.deadband, 0.0f, 1.0f)
        : 0.0f;
    float acc_shaped;
    if (fabsf(acc_smoothed) <= acc_deadband) {
        acc_shaped = 0.0f;
    } else {
        acc_shaped = copysignf(
            fabsf(acc_smoothed) - acc_deadband,
            acc_smoothed);
    }

    const float acc_ff_gain = isfinite(s_pitch_ff.gain)
        ? lib_clamp(s_pitch_ff.gain, -10000.0f, 10000.0f)
        : 0.0f;
    const float acc_ff_limit = isfinite(s_pitch_ff.limit)
        ? lib_clamp(s_pitch_ff.limit, 0.0f, 7000.0f)
        : 0.0f;
    s_pitch_ff.current = lib_clamp(
        acc_ff_gain * acc_shaped,
        -acc_ff_limit, acc_ff_limit);

}

static void yaw_pid_control(void)
{
    float yaw_cur_angle = s_pose.euler.yaw;
    float small_yaw_cur_angle = SENTRY_GIMBAL_S_YAW_DIR * lib_enc13_relative_rad(
        s_motor_state.feedback[MOTOR_YAW_S].angle, SENTRY_GIMBAL_S_YAW_ZERO);
    float small_yaw_tar_angle = yaw_cur_angle + lib_get_shortest_path(
        s_command_state.yaw_tar_angle, yaw_cur_angle);
    float large_yaw_tar_angle = yaw_cur_angle + lib_get_shortest_path(
        lib_rad_norm(s_command_state.yaw_tar_angle + small_yaw_cur_angle), yaw_cur_angle);
    float large_yaw_cur_speed = SENTRY_GIMBAL_L_YAW_DIR * lib_rpm_to_rad_s(
        (float)s_motor_state.feedback[MOTOR_YAW_L].speed);
    float small_yaw_cur_speed = SENTRY_GIMBAL_S_YAW_DIR * lib_rpm_to_rad_s(
        (float)s_motor_state.feedback[MOTOR_YAW_S].speed);
    float chassis_yaw_speed = 0.0f;
    float large_yaw_ground_speed;
    float small_yaw_ground_speed;
    float chassis_yaw_feedback;
    uint32_t chassis_omega_rx_tick_ms;
    uint32_t irq_state;
    uint8_t has_chassis_omega;
    uint8_t chassis_omega_is_valid;
    float small_yaw_current;
    float limit_weight = 0.0f;

    irq_state = __get_PRIMASK();
    __disable_irq();
    chassis_yaw_feedback = s_chassis_feedback.vw_cur_speed;
    chassis_omega_rx_tick_ms = s_chassis_feedback.vw_rx_tick_ms;
    chassis_omega_is_valid = s_chassis_feedback.vw_is_valid;
    __set_PRIMASK(irq_state);

    has_chassis_omega = chassis_omega_is_valid
        && (uint32_t)(bsp_tim_get_tick_ms() - chassis_omega_rx_tick_ms)
            <= s_chassis_feedback.timeout_ms;
    if (has_chassis_omega) {
        chassis_yaw_speed = chassis_yaw_feedback;
    }
    if (has_chassis_omega) {
        large_yaw_ground_speed = chassis_yaw_speed + large_yaw_cur_speed;
        small_yaw_ground_speed = large_yaw_ground_speed + small_yaw_cur_speed;
    } else {
        /* 没有可信底盘速度时，维持原来的相对速度阻尼。 */
        large_yaw_ground_speed = large_yaw_cur_speed;
        small_yaw_ground_speed = small_yaw_cur_speed;
    }

    small_yaw_current = lib_pid_pos_calc(
        &s_pid_yaw_small, small_yaw_tar_angle, yaw_cur_angle,
        0.0f, 0.0f, small_yaw_ground_speed, 0.001f);
    s_motor_state.current[MOTOR_YAW_L] = (int16_t)(
        SENTRY_GIMBAL_L_YAW_DIR * lib_pid_pos_calc(
            &s_pid_yaw_large, large_yaw_tar_angle, yaw_cur_angle,
            0.0f, 0.0f, large_yaw_ground_speed, 0.001f));

    if (small_yaw_cur_angle > SENTRY_GIMBAL_S_YAW_LIMIT - SENTRY_GIMBAL_S_YAW_SOFT_ZONE) {
        limit_weight = lib_remap_clamp(small_yaw_cur_angle,
            SENTRY_GIMBAL_S_YAW_LIMIT - SENTRY_GIMBAL_S_YAW_SOFT_ZONE,
            SENTRY_GIMBAL_S_YAW_LIMIT, 0.0f, 1.0f);
        if (small_yaw_current > 0.0f) {
            small_yaw_current *= 1.0f - limit_weight;
            s_pid_yaw_small.integral = 0.0f;
        }
        if (small_yaw_cur_speed > 0.0f) {
            small_yaw_current -= SENTRY_GIMBAL_S_YAW_BRAKE_GAIN
                               * limit_weight * small_yaw_cur_speed;
            s_pid_yaw_small.integral = 0.0f;
        }
    } else if (small_yaw_cur_angle < -SENTRY_GIMBAL_S_YAW_LIMIT + SENTRY_GIMBAL_S_YAW_SOFT_ZONE) {
        limit_weight = lib_remap_clamp(small_yaw_cur_angle,
            -SENTRY_GIMBAL_S_YAW_LIMIT,
            -SENTRY_GIMBAL_S_YAW_LIMIT + SENTRY_GIMBAL_S_YAW_SOFT_ZONE, 1.0f, 0.0f);
        if (small_yaw_current < 0.0f) {
            small_yaw_current *= 1.0f - limit_weight;
            s_pid_yaw_small.integral = 0.0f;
        }
        if (small_yaw_cur_speed < 0.0f) {
            small_yaw_current -= SENTRY_GIMBAL_S_YAW_BRAKE_GAIN
                               * limit_weight * small_yaw_cur_speed;
            s_pid_yaw_small.integral = 0.0f;
        }
    }

    small_yaw_current = lib_clamp(small_yaw_current,
        DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX);
    s_pid_yaw_small.out = small_yaw_current;
    s_motor_state.current[MOTOR_YAW_S] = (int16_t)(
        SENTRY_GIMBAL_S_YAW_DIR * small_yaw_current);
}
static void pitch_control(void)
{
    float pitch_cur_angle = s_pose.euler.pitch;
    float pitch_joint_angle = pitch_encoder_angle();
    float platform_pitch = pitch_cur_angle + pitch_joint_angle;
    float pitch_tar_angle;
    float pitch_current;

    s_command_state.pitch_tar_angle = lib_clamp(s_command_state.pitch_tar_angle,
        SENTRY_GIMBAL_PITCH_MIN, SENTRY_GIMBAL_PITCH_MAX);
    /* pitch = 平台倾角 - 编码器角；编码器上限对应 pitch 下限。 */
    pitch_tar_angle = lib_clamp(s_command_state.pitch_tar_angle,
        platform_pitch - SENTRY_GIMBAL_PITCH_JOINT_MAX,
        platform_pitch - SENTRY_GIMBAL_PITCH_JOINT_MIN);

    /* 位置环输出反向映射到电机电流；重力前馈保持原电机补偿方向。 */
    pitch_current = -lib_pid_pos_calc(
        &s_pid_pitch, pitch_tar_angle, pitch_cur_angle,
        -cosf(pitch_cur_angle), 0.0f, s_motor_state.pitch_cur_speed, 0.001f);

    pitch_current = lib_clamp(
        pitch_current + s_pitch_ff.current,
        DRV_MOTOR_GM6020_CURRENT_MIN, DRV_MOTOR_GM6020_CURRENT_MAX);

    /* 机械软限位只看关节角；仅削减继续向外的电流，保留向内恢复能力。 */
    if (pitch_current > 0.0f
        && pitch_joint_angle > SENTRY_GIMBAL_PITCH_JOINT_MAX - SENTRY_GIMBAL_PITCH_SOFT_ZONE) {
        pitch_current *= lib_remap_clamp(pitch_joint_angle,
            SENTRY_GIMBAL_PITCH_JOINT_MAX - SENTRY_GIMBAL_PITCH_SOFT_ZONE,
            SENTRY_GIMBAL_PITCH_JOINT_MAX, 1.0f, 0.0f);
        s_pid_pitch.integral = 0.0f;
    } else if (pitch_current < 0.0f
               && pitch_joint_angle < SENTRY_GIMBAL_PITCH_JOINT_MIN + SENTRY_GIMBAL_PITCH_SOFT_ZONE) {
        pitch_current *= lib_remap_clamp(pitch_joint_angle,
            SENTRY_GIMBAL_PITCH_JOINT_MIN,
            SENTRY_GIMBAL_PITCH_JOINT_MIN + SENTRY_GIMBAL_PITCH_SOFT_ZONE, 0.0f, 1.0f);
        s_pid_pitch.integral = 0.0f;
    }

    s_pid_pitch.out = pitch_current;
    s_motor_state.current[MOTOR_PITCH] = (int16_t)pitch_current;
}

static void launch_control(void)
{
    switch (s_command_state.data.fire) {
    case APP_SENTRY_FIRE_OFF:
        s_motor_state.current[MOTOR_DISC] = 0;
        lib_pid_reset(&s_pid_disc);
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, 0.0f,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, 0.0f,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);

        break;
    default:
        launcher_reset();

        break;

    case APP_SENTRY_FIRE_ON:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, -SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);
        s_motor_state.current[MOTOR_DISC] = (int16_t)lib_pid_ff_calc(
            &s_pid_disc, SENTRY_DISC_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed),
            3000.0f, 0.0f, 0.005f);
        break;

    case APP_SENTRY_FIRE_FIR:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, -SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);
        s_motor_state.current[MOTOR_DISC] = 0;
        lib_pid_reset(&s_pid_disc);
        break;

    case APP_SENTRY_FIRE_REVERSE:
        s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
            &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed), 0.005f);
        s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
            &s_pid_fric_r, -SENTRY_FRICTION_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed), 0.005f);
        s_motor_state.current[MOTOR_DISC] = (int16_t)lib_pid_ff_calc(
            &s_pid_disc, -SENTRY_DISC_TAR_SPEED,
            lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed),
            -3000.0f, 0.0f, 0.005f);
        break;

    }
}

static void gimbal_can2_tx(void)
{
    uint8_t frame[8];

    /* 0x1FE: [YL_H,YL_L, YS_H,YS_L, P_H,P_L, 0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_state.current[MOTOR_YAW_L]);
    frame[1] = LIB_LO_BYTE(s_motor_state.current[MOTOR_YAW_L]);
    frame[2] = LIB_HI_BYTE(s_motor_state.current[MOTOR_YAW_S]);
    frame[3] = LIB_LO_BYTE(s_motor_state.current[MOTOR_YAW_S]);
    frame[4] = LIB_HI_BYTE(s_motor_state.current[MOTOR_PITCH]);
    frame[5] = LIB_LO_BYTE(s_motor_state.current[MOTOR_PITCH]);
    (void)bsp_can_tx(&hcan2, SENTRY_CAN_GIMBAL_TX_YAW, frame);
}

static void launcher_can2_tx(void)
{
    uint8_t frame[8];

    /* 0x200: [FL_H,FL_L, DL_H,DL_L, FR_H,FR_L, 0,0] */
    memset(frame, 0, sizeof(frame));
    frame[0] = LIB_HI_BYTE(s_motor_state.current[MOTOR_FRIC_L]);
    frame[1] = LIB_LO_BYTE(s_motor_state.current[MOTOR_FRIC_L]);
    frame[2] = LIB_HI_BYTE(s_motor_state.current[MOTOR_DISC]);
    frame[3] = LIB_LO_BYTE(s_motor_state.current[MOTOR_DISC]);
    frame[4] = LIB_HI_BYTE(s_motor_state.current[MOTOR_FRIC_R]);
    frame[5] = LIB_LO_BYTE(s_motor_state.current[MOTOR_FRIC_R]);
    (void)bsp_can_tx(&hcan2, SENTRY_CAN_GIMBAL_TX_LAUNCH, frame);
}

static void gimbal_can1_tx(void)
{
    const float yaw = s_pose.euler.yaw;
    const float pitch = s_pose.euler.pitch;
    const float roll = s_pose.euler.roll;
    drv_imu_quat_t *control_quat = &s_pose.quat;

    if (!s_sampling.instance || !isfinite(yaw) || !isfinite(pitch) || !isfinite(roll)) {
        return;
    }

    /* 用控制反馈的同一组角度重建云台到世界的四元数。
     * ZYX：q = qz(yaw) * qy(pitch) * qx(roll)，标量在前。
     * 不直接发送传感器四元数：它不含 pitch 关节编码器补偿。 */
    const float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    control_quat->q0 = cy * cp * cr + sy * sp * sr;
    control_quat->q1 = cy * cp * sr - sy * sp * cr;
    control_quat->q2 = cy * sp * cr + sy * cp * sr;
    control_quat->q3 = sy * cp * cr - cy * sp * sr;

    /* 0x122：发送 yaw/pitch 角速度，单位 rad/s。 */
    app_gimbal_comm_gyro_tx(
        s_pose.yaw_rate,
        s_motor_state.pitch_cur_speed);

    /* 0x124：发送 yaw/pitch 当前角度，单位 rad。 */
    app_gimbal_comm_angle_tx(
        yaw,
        pitch);

    app_gimbal_comm_quat_tx(
        (int16_t)(control_quat->q0 * 30000.0f), (int16_t)(control_quat->q1 * 30000.0f),
        (int16_t)(control_quat->q2 * 30000.0f), (int16_t)(control_quat->q3 * 30000.0f));

    /* 0x139 与 0x124、PID 使用完全相同的 yaw/pitch，仅单位改为度。 */
    app_gimbal_comm_fused_yaw_pitch_tx(
        lib_rad_to_deg(yaw),
        lib_rad_to_deg(pitch));
}

static void gimbal_vofa_send(void)
{
    /* 按调试需求选择 data[] 的数据，未使用通道保持为 0。
     * 当前 I0~I2 为 pitch 目标角、实际角、角速度；I3~I5 为前馈、命令、反馈电流；
     * I6~I8 为前馈慢基线、滤波加速度、yaw 角速度。 */
    float data[DRV_VOFA_CHANNEL_COUNT] = {0};
    data[0] = lib_rad_to_deg(s_command_state.pitch_tar_angle);
    data[1] = lib_rad_to_deg(s_pose.euler.pitch);
    data[2] = s_motor_state.pitch_cur_speed;
    data[3] = s_pitch_ff.current;
    data[4] = (float)s_motor_state.current[MOTOR_PITCH];
    data[5] = (float)s_motor_state.feedback[MOTOR_PITCH].current;
    data[6] = s_pitch_ff.bias_lpf.out;
    data[7] = s_pitch_ff.dynamic_lpf.out;
    data[8] = s_pose.yaw_rate;
    (void)drv_vofa_port_send(data);
}
