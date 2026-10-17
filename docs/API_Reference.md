# BNGU Control System API 参考

本文整理仓库 `Self_core` 的主要公共接口及其使用约定。函数签名以对应 `.h` 文件为准；HAL 和 STM32CubeMX 接口不在此重复。如何选择板型、编译和运行见 [使用文档](USAGE.md)。

## 1. 模块边界

| 层 | 目录 | 职责 |
| --- | --- | --- |
| APP | `Self_core/App` | 控制策略、系统调度和板间协议 |
| DRV | `Self_core/Drv` | 传感器、电机、遥控器、裁判系统和调试输出 |
| BSP | `Self_core/Bsp` | 将 CAN、UART、定时器、SPI 等 HAL 外设封装给驱动 |
| LIB | `Self_core/Lib` | 不依赖硬件的数学、滤波和 PID 算法 |

APP 组织系统调度和机器人控制；公共通信模块会通过 BSP CAN 接口收发板间报文，设备驱动则通过 BSP 适配硬件。LIB 是纯算法模块。应用配置由 `Self_core/project_cfg.h` 选择。

## 2. 系统与应用接口

头文件：`app_init.h`、`app_control.h`。

```c
void app_init(void);
void app_control_process(void);
void app_control_1khz(void);
```

`app_init()` 初始化公共外设并按板型选择应用。TIM14 的 1 ms 回调调用 `app_control_1khz()`；主循环调用 `app_control_process()` 执行低频后台工作。哨兵云台的 AHRS 和云台控制运行在 1 kHz，板间转发和发射机构控制运行在 200 Hz。

### 2.1 底盘通信

头文件：`app_chassis_comm.h`。CAN1 应用报文最长为 8 字节；多字节字段按当前 STM32 小端布局发送，`0x129` 发射命令使用 DLC=4。完整 CAN1/CAN2 ID、帧布局和电机组帧顺序见 [CAN 总线 ID 与通信协议](CAN_Protocol.md)。

| ID | 方向 | 数据 |
| --- | --- | --- |
| `0x111` | 底盘接收 | `app_chassis_speed_cmd_t`：`vx`、`vy`，单位 mm/s；`vz`，以 0.001 rad/s 为单位；`power_pct` 为保留字段。 |
| `0x112` | 底盘发送 | 实际功率 `int16_t`，缩放值为功率 × 100，其余字节为 0。 |
| `0x113` | 底盘接收 | `app_chassis_ackermann_cmd_t`：速度和转向角两个 `float`。 |
| `0x115` | 底盘接收 | `app_chassis_follow_cmd_t`：`vx`、`vy`、云台角度 × 1000、保留字段。 |
| `0x119` | 底盘发送 | z 轴角速度 `float`，单位 rad/s。 |

```c
void app_chassis_comm_init(void);
uint8_t app_chassis_comm_read_rx(app_chassis_comm_rx_t *rx);
uint8_t app_chassis_comm_power_tx(int16_t power_x100);
uint8_t app_chassis_comm_omega_tx(float omega_z);
```

接收函数返回 `1` 表示输出快照有效，空指针返回 `0`。两个发送函数返回 `bsp_can_tx()` 的状态，`0` 表示发送成功。

### 2.2 云台通信

头文件：`app_gimbal_comm.h`。本板 CAN1 接收上位机/底盘指令并发送云台反馈。接收回调只更新快照；由控制循环读取，避免在 CAN 中断里运行控制逻辑。

| ID | 方向 | 数据 |
| --- | --- | --- |
| `0x120` | 云台接收并转发 | 4 个 `int16_t`：底盘 x/y 速度、z 角速度量化值、保留值。仅 CAN 控制挡位下转发到 `0x111`。 |
| `0x121`、`0x125` | 云台接收 | 两个 `float`：yaw/pitch 角度增量，单位 rad；后者为带射击指令。 |
| `0x123`、`0x127` | 云台接收 | 两个 `float`：yaw/pitch 绝对角度，单位 rad；后者为带射击指令。 |
| `0x129` | 云台接收 | 4 字节控制命令；前两字节为 `shoot_switch`、`retreat`。 |
| `0x122` | 云台发送 | 两个 `float`：yaw/pitch 角速度，单位 rad/s。 |
| `0x124` | 云台发送 | 两个 `float`：yaw/pitch 角度，单位 rad。 |
| `0x126` | 云台发送 | 8 字节发射机构状态数据。 |
| `0x130` | 云台发送 | 4 个 `uint16_t` 的预留角度反馈格式。 |
| `0x139` | 云台发送 | 融合后的 yaw、pitch；两个 `float32`，单位度，小端序；不包含 roll。 |
| `0x233` | 云台发送 | 4 个 `int16_t` 四元数分量；调用者按 ×30000 缩放。 |

```c
void app_gimbal_comm_init(void);
uint8_t app_gimbal_comm_read_rx(app_gimbal_comm_rx_t *rx,
                                uint8_t *updated_mask);
uint8_t app_gimbal_comm_dbus_rx(app_gimbal_dbus_input_t *input);
void app_gimbal_comm_process(void);
void app_gimbal_comm_gyro_tx(float yaw_tar_speed, float pitch_tar_speed);
void app_gimbal_comm_angle_tx(float yaw_angle, float pitch_angle);
void app_gimbal_comm_fused_yaw_pitch_tx(float yaw_deg, float pitch_deg);
void app_gimbal_comm_angle_tx_v2(uint16_t yaw, uint16_t pitch,
                                 uint16_t roll, uint16_t interval);
void app_gimbal_comm_shoot_tx(const uint8_t data[8]);
void app_gimbal_comm_quat_tx(int16_t q0, int16_t q1,
                             int16_t q2, int16_t q3);
```

`app_gimbal_comm_read_rx()` 原子复制最近一次收到的命令，并返回自上次读取后各命令的更新位；读取后更新位清零。掩码为 `APP_GIMBAL_COMM_UPDATE_*`。`app_gimbal_comm_dbus_rx()` 将摇杆量归一化到 `[-1, 1]`，并将 S2 挡位转换成 DBUS、CAN 或急停控制源；无有效遥控数据时返回 `0`。

### 2.3 哨兵控制入口

```c
/* app_sentry_gimbal.h */
void app_sentry_gimbal_init(drv_imu_t *imu);
void app_gimbal_ahrs_update(float dt);
void app_sentry_gimbal_ctrl_1khz(void);
void app_sentry_launcher_ctrl_200hz(void);

/* app_sentry_chassis.h */
void app_sentry_chassis_init(void);
void app_sentry_chassis_ctrl(void);
const app_sentry_chassis_state_t *app_chassis_get_state(void);
```

`app_gimbal_ahrs_update()` 接受秒为单位的控制周期；当前调度传入 `0.001f`。没有新 DMA 快照时累计周期，拿到新采样后按累计时间更新所选姿态算法，最大积分间隔为 100 ms。底盘状态指针只读，指向模块维护的最新快照。哨兵控制模式、底盘参数和电机 CAN ID 定义在 `app_sentry_common.h`。

## 3. 驱动接口

### 3.1 DJI、达妙和翎控电机

头文件：`drv_motor.h`、`drv_motor_dm_reg.h`。`drv_motor_data_t` 保存通用反馈中的命令 ID、转速、编码器角度、温度和电流。DJI 电机反馈字段按大端解析；DJI 电流帧最多有 4 个槽位。

```c
void drv_motor_solve_dji(const uint8_t *data, drv_motor_data_t *cur);
void drv_motor_reset_frame(uint8_t *frame);
void drv_motor_set_dji(uint8_t *frame, uint8_t slot, int16_t current);

uint8_t drv_motor_init_dm(drv_motor_dm_t *motor, drv_motor_dm_mode_t mode,
                          uint16_t can_id, uint16_t Master_id,
                          float p_max, float v_max, float t_max);
void drv_motor_solve_dm(const uint8_t *data, drv_motor_dm_t *cur);
uint8_t drv_motor_dm_MIT_frame_config(uint8_t *frame,
    const drv_motor_dm_config_t *config, float p_des, float v_des,
    float t_ff, float kp, float kd);
uint8_t drv_motor_dm_position_velocity_frame_config(uint8_t *frame,
    const drv_motor_dm_config_t *config, float p_des, float v_des);
uint8_t drv_motor_dm_velocity_frame_config(uint8_t *frame,
    const drv_motor_dm_config_t *config, float v_des);
uint8_t drv_motor_dm_PVT_frame_config(uint8_t *frame,
    const drv_motor_dm_config_t *config, float p_des,
    uint16_t v_des, uint16_t i_des);
uint8_t drv_motor_dm_command_frame(uint8_t *frame, drv_motor_dm_cmd_t command);
uint8_t drv_motor_dm_read_reg_frame_config(uint8_t *frame, uint16_t cmd_id,
    const drv_motor_dm_reg_t reg);
uint8_t drv_motor_dm_read_reg_response_parse(drv_motor_dm_reg_info_t *reg_info,
    const uint8_t *data, uint8_t len);
uint8_t drv_motor_dm_write_reg_frame_config(uint8_t *frame, uint16_t cmd_id,
    const drv_motor_dm_reg_t reg, drv_motor_dm_w_reg_value_t value);
uint8_t drv_motor_dm_write_reg_response_parse(drv_motor_dm_reg_info_t *reg_info,
    const uint8_t *data, uint8_t len);
uint8_t drv_motor_dm_save_params_frame_config(uint8_t *frame, uint16_t cmd_id);
uint8_t drv_motor_dm_save_params_response_parse(uint16_t cmd_id,
    const uint8_t *data, uint8_t len);
```

达妙配置中的位置、速度、扭矩范围单位分别是 rad、rad/s、N·m。初始化和帧构建函数返回 `1` 表示成功、`0` 表示参数无效；初始化只保存本地配置，不会发送命令或使能电机。模式和错误状态使用 `drv_motor_dm_mode_t`、`drv_motor_dm_err_status_t`。寄存器读写、回复解析和参数保存接口及寄存器枚举见对应头文件。翎控函数目前属于未完整适配接口，使用前需自行核对硬件协议。

达妙寄存器请求和回复解析函数只组帧或检查接收到的数据，不负责发 CAN 或等待回复。保存参数命令会写入电机 Flash；仅在明确需要时调用，避免在周期任务里重复保存。

### 3.2 IMU

头文件：`drv_imu.h`。`drv_imu_t` 保存原始与物理量传感器数据、四元数、欧拉角和滤波状态。加速度单位为 g，角速度单位为 rad/s，欧拉角单位为度。

```c
void drv_imu_init(drv_imu_t *imu, const drv_imu_bus_t *bus);
void drv_imu_start(drv_imu_t *imu);
void drv_imu_set_euler_mode(drv_imu_t *imu, drv_imu_euler_mode_t mode);
uint8_t drv_imu_set_algorithm(drv_imu_t *imu, drv_imu_algorithm_t algorithm);
void drv_imu_reset_filter(drv_imu_t *imu);
void drv_imu_read_acc_raw(drv_imu_t *imu);
void drv_imu_read_gyro_raw(drv_imu_t *imu);
void drv_imu_data_convert(drv_imu_t *imu);
void drv_imu_read_temp(drv_imu_t *imu);
void drv_imu_update(drv_imu_t *imu, float dt);
void drv_imu_mahony_update(drv_imu_t *imu, float dt);
void drv_imu_ekf_update(drv_imu_t *imu, float dt);
void drv_imu_vqf_update(drv_imu_t *imu, float dt);
void drv_imu_ekf_default_config(drv_imu_ekf_config_t *config);
void drv_imu_vqf_default_config(drv_imu_vqf_config_t *config);
uint8_t drv_imu_ekf_configure(drv_imu_t *imu, const drv_imu_ekf_config_t *config);
uint8_t drv_imu_vqf_configure(drv_imu_t *imu, const drv_imu_vqf_config_t *config);
void drv_imu_quat_to_euler(drv_imu_t *imu);
void drv_imu_calibrate_gyro(drv_imu_t *imu, uint16_t sample_count);
void drv_imu_initial_alignment(drv_imu_t *imu);
void drv_imu_calibrate_pose(drv_imu_t *imu);
void drv_imu_restart(drv_imu_t *imu);

/* BMI088/SPI1 硬件适配接口 */
void drv_imu_port_init(drv_imu_t *imu);
uint8_t drv_imu_port_async_start(void);
uint8_t drv_imu_port_snapshot_update(drv_imu_t *imu);
uint8_t drv_imu_port_is_online(uint32_t timeout_ms);
uint32_t drv_imu_port_get_busy_count(void);
uint32_t drv_imu_port_get_error_count(void);
```

端口采样分为异步启动和快照复制；`snapshot_update()` 返回 `1` 表示拿到了新的一组完整采样。在线状态由最近一次完整快照的时间决定。

姿态解算使用 `drv_imu_update()`，按 `imu->algorithm` 分发至 Mahony、EKF 或 VQF。初始化默认使用 Mahony；云台启动时读取 `project_cfg.h` 的 `IMU_AHRS_ALGORITHM`。三个专用 `*_update()` 入口会自动切换算法。`dt` 为两次有效采样之间的秒数，有效范围为 `(0, 0.1]`；非有限或越界周期、非有限陀螺数据会跳过更新，加速度为零或非有限时仍进行陀螺积分。更新不读取硬件、不换算原始数据、不转换欧拉角。

四元数为标量在前的传感器系到世界系旋转，水平静止时重力测量沿 `+Z`。全部算法先扣除 `gyro_offset`、`acc_offset`；EKF/VQF 在线估计额外的残余陀螺零偏，单位 rad/s，保存在 `imu->filter.ekf.bias` / `imu->filter.vqf.bias`，不修改原始角速度或静态标定值。六轴传感器无法提供绝对航向；运动中绕重力方向的零偏不可观，yaw 仍可能长期漂移。

| 算法 | 实现与默认参数 | 调试状态 |
| --- | --- | --- |
| `DRV_IMU_ALGO_MAHONY` | 原有重力方向 PI 反馈。 | `integral_fb` |
| `DRV_IMU_ALGO_EKF` | 右乘四元数误差状态 EKF；状态为三维姿态误差和三维陀螺零偏；解析旋转积分、`F P Fᵀ + Q` 预测、重力量测及 Joseph 协方差更新。默认陀螺噪声密度 `0.015 rad/s/√Hz`、零偏随机游走 `0.0001 rad/s/√s`、归一化重力噪声 `0.035`、加速度模长偏离门限 `0.2g`、创新距离平方门限 `11.345`。 | `filter.ekf.covariance`、`acc_used`、`innovation_squared`、`bias` |
| `DRV_IMU_ALGO_VQF` | 作者在线 VQF 的六轴 C 移植，保留惯性系二阶 Butterworth 加速度滤波、倾角修正、运动/静止零偏估计。默认 `tau_acc=3s`、静止检测滤波 `0.5s`、静止持续 `1.5s`、角速度偏差 `2deg/s`、加速度偏差 `0.5m/s²`；两种零偏估计均开启。 | `filter.vqf.rest_detected`、`rest_deviation`、`bias` |

`*_default_config()` 填充默认参数。`*_configure()` 先校验再应用，成功返回 `1`，无效配置返回 `0` 且不修改实例；传 `NULL` 恢复默认参数。EKF 噪声参数均为标准差。VQF 的静止加速度门限使用 **m/s²**，驱动内部将 g 转为 m/s² 后处理；角速度相关配置沿用上游 **deg/s** 单位。VQF 在 `dt` 改变时更新系数并转换滤波历史，初始化平均时间按实际有效采样时间累计；其低通系数及历史采用 double，避免 1 kHz、低截止频率时的精度损失。具体字段及单位见 `drv_imu.h`。

算法切换会保留当前四元数并清除所选算法历史；重复选择同一算法不会重置。`drv_imu_reset_filter()`、重标定陀螺零偏、初始重力对准都会重置在线零偏和协方差；配置和静态零偏仍保留。`drv_imu_initial_alignment()` 以当前加速度建立倾角并令初始 yaw 为零，建议在启动 EKF 前完成，以免较大的初始姿态误差被创新门限拒绝。`filter` 是联合体，只能读取当前算法对应成员。配置、切换、对准及重置应在同一 AHRS 执行上下文中调用，或暂停更新后调用。

代码来源及完整 MIT 许可见 `Self_core/Drv/Drv_imu/VQF_LICENSE.txt`，上游为 [dlaidig/vqf](https://github.com/dlaidig/vqf)，EKF 误差定义参考 [Solà 的误差状态卡尔曼滤波推导](https://arxiv.org/abs/1711.02508)。

### 3.3 DBUS、VOFA、裁判系统与功率计

| 模块 | 主要接口 | 说明 |
| --- | --- | --- |
| DBUS (`drv_dbus.h`) | `drv_dbus_decode(const uint8_t buffer[DRV_DBUS_BUFFER_SIZE], drv_dbus_data_t *data)`、`drv_dbus_port_init()`、`drv_dbus_port_get_data()` | 解码遥控器 18 字节数据；端口负责 USART3 DMA 接收并返回最新有效快照。 |
| VOFA (`drv_vofa.h`) | `drv_vofa_pack(data, frame)`、`drv_vofa_port_init()`、`drv_vofa_port_send(data)` | 8 个 `float` 打包为 JustFloat 帧；通过 USART1 非阻塞发送。 |
| 裁判系统 (`drv_referee.h`) | `drv_referee_init()`、`drv_referee_process(data, len)`、`drv_referee_get_data()`、`drv_referee_port_init()`、`drv_referee_port_process()` | 解析增量字节流、检查 CRC 并提供最新数据快照；端口运行在 USART6。 |
| 功率计 (`drv_power_measure.h`) | `drv_power_solve(can_data, data)`、`drv_power_port_can_init(hcan, rx_cb)` | 解析 8 字节 CAN 数据；电压和电流以 0.01 V、0.01 A 为单位，`power` 为 W。 |

VOFA 定义：`DRV_VOFA_CHANNEL_COUNT == 8`，`DRV_VOFA_FRAME_SIZE == 36`。帧内容是 8 个 IEEE-754 `float` 的原始小端字节，后接 `00 00 80 7F`。发送状态为 `DRV_VOFA_OK`、`DRV_VOFA_ERR_BUSY`、`DRV_VOFA_ERR_ARGUMENT` 或 `DRV_VOFA_ERR_UART`。端口持有发送缓冲区；调用方只需提供至少 8 个 `float`。

`drv_referee_get_data()` 返回模块持有的只读快照；不要在调用方释放或改写。`drv_referee_get_and_clear_flags()` 返回并清除更新标志，标志定义为 `REFEREE_FLAG_*`。

### 3.4 指示灯、蜂鸣器和乐谱

- `drv_led.h`：`drv_led_port_init()` 初始化板载 RGB LED；`drv_led_set(color, state)`、`drv_led_toggle(color)` 和 `drv_led_rgb(r, g, b)` 控制颜色。核心层也可通过 `drv_led_init()` 注入 GPIO 回调。
- `drv_buzzer.h`：`drv_buzzer_port_init()` 绑定蜂鸣器 PWM；`drv_buzzer_on/off/toggle()` 控制输出，`drv_buzzer_set_freq(freq_hz)` 和 `drv_buzzer_set_duty(duty)` 更新音调及占空比。
- `drv_melody.h`：`drv_melody_play(notes, loop)` 播放以 `{0, 0}` 结尾的音符数组；需周期调用 `drv_melody_update()` 推进播放状态。此模块标注为未测试。

## 4. 数学、滤波与 PID

### 4.1 数学（`lib_math.h`）

| 函数 | 用途 |
| --- | --- |
| `lib_clamp(value, min, max)`、`lib_remap_clamp(...)` | 限幅、区间线性映射。 |
| `lib_get_shortest_path(target, measure)`、`lib_rad_norm(rad)` | 角度最短差和弧度归一化，单位 rad。 |
| `lib_deg_to_rad(deg)`、`lib_rad_to_deg(rad)` | 度与弧度转换。 |
| `lib_rpm_to_rad_s(rpm)`、`lib_rad_s_to_rpm(rad_s)` | RPM 与 rad/s 转换。 |
| `lib_motor_rpm_to_mm_s(...)`、`lib_motor_mm_s_to_rpm(...)` | 电机转速与轮缘线速度转换，半径单位 mm。 |
| `lib_enc13_relative_rad(raw_enc, zero_enc)`、`lib_enc_conv(value, dir)` | 13 位编码器相对角和 13/16 位计数换算。 |
| `lib_encode_symmetric(...)`、`lib_encode_unsigned(...)` | 将有符号或非负浮点范围映射到 `uint16_t` 编码。 |
| `lib_fast_sigmoid(x)` | 快速 Sigmoid 近似。 |

编码器转换方向常量为 `LIB_ENC13_TO_RAD`、`LIB_RAD_TO_ENC13`、`LIB_ENC16_TO_RAD`、`LIB_RAD_TO_ENC16`。

### 4.2 滤波（`lib_filter.h`）

```c
void lib_lpf_init(lib_lpf_t *lpf, float cutoff_hz);
float lib_lpf_update(lib_lpf_t *lpf, float input, float dt);
void lib_swf_init(lib_swf_t *swf, float *buf, uint16_t len);
float lib_swf_update(lib_swf_t *swf, float input);
```

低通滤波 `dt` 以秒为单位且必须大于 0；滑动窗口缓冲区由调用方提供，长度至少为 `len`。

### 4.3 PID（`lib_pid.h`）

所有计算接口显式接收周期 `dt`（秒）。每个控制环应持有独立的 `lib_pid_t` 状态，输出限幅、积分限幅和前馈参数由 `lib_pid_init()` 配置。

```c
void lib_pid_init(lib_pid_t *pid, float kp, float ki, float kd,
    float kff_g, float kff_y, float max_ff_g, float max_ff_y,
    float min_out, float max_out, float max_iout);
void lib_pid_reset(lib_pid_t *pid);
float lib_pid_calc(lib_pid_t *pid, float target, float measure, float dt);
float lib_pid_ff_calc(lib_pid_t *pid, float target, float measure,
    float ff_g, float ff_y, float dt);
float lib_pid_pos_calc(lib_pid_t *pid, float target, float measure,
    float ff_g, float ff_y, float speed, float dt);
void lib_pid_set_2dof_weight(lib_pid_t *pid, float weight_p, float weight_d);
float lib_pid_deriv_first_calc(lib_pid_t *pid, float target, float measure,
    float dt);
float lib_pid_2dof_calc(lib_pid_t *pid, float target, float measure, float dt);
```

模糊 PID 使用 `lib_pid_fuzzy_t`，由 `lib_pid_fuzzy_init()` 初始化，`lib_pid_fuzzy_cfg()` 配置量化因子，可用 `lib_pid_fuzzy_load_default_rules()` 加载默认规则，最后以 `lib_pid_fuzzy_calc(pid, target, measure, dt)` 计算。

## 5. BSP 常用接口

应用代码通常不直接调用 BSP；下列接口主要供 DRV 的硬件适配层使用。句柄类型由 `bsp_cfg.h` 导入。

| 头文件 | API |
| --- | --- |
| `bsp_can.h` | `bsp_can_start(hcan, filter_bank, slave_filter_bank)`、`bsp_can_tx(hcan, std_id, data[8])`、`bsp_can_rx_reg(hcan, std_id, callback)`、`bsp_can_rx_irq_handler(hcan)` |
| `bsp_uart.h` | `bsp_uart_tx(huart, data, len)`、`bsp_uart_rx_dma(huart, data, len)`、`bsp_uart_reg_tx_cplt_callback(huart, callback)`、`bsp_uart_reg_rx_idle_callback(huart, callback)`、`bsp_uart_irq_handler(huart)` |
| `bsp_tim.h` | `bsp_tim_pwm_start/stop(htim, channel)`、`bsp_tim_pwm_set_freq(htim, channel, freq_hz)`、`bsp_tim_pwm_set_compare(htim, channel, compare)`、`bsp_tim_reg_callback(htim, callback)`、`bsp_tim_get_tick_ms()`、`bsp_tim_delay_ms(ms)` |
| `bsp_spi.h` | `bsp_spi_transceive(hspi, tx, rx, size)`、`bsp_spi_transceive_dma(hspi, tx, rx, size)`、`bsp_spi_reg_dma_callbacks(hspi, complete, error)`、`bsp_spi_abort_dma(hspi)` |

其他板级 API 见 `bsp_adc.h`、`bsp_flash.h`、`bsp_gpio.h`、`bsp_i2c.h`。HAL 中断应通过相应 BSP/DRV 回调入口分发，不要从应用逻辑绕过驱动直接操作 HAL 状态。
