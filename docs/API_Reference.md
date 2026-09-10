# API Reference — BNGU Control System

> STM32F407 四层架构 (LIB / BSP / DRV / APP)
> 依赖方向: `LIB ← BSP ← DRV ← APP`

---

## 目录

1. [架构与依赖规则](#1-架构与依赖规则)
2. [目录结构](#2-目录结构)
3. [LIB 层](#3-lib-层)
4. [BSP 层](#4-bsp-层)
5. [DRV 层](#5-drv-层)
6. [APP 层](#6-app-层)
7. [层间依赖矩阵](#7-层间依赖矩阵)
8. [头文件索引](#8-头文件索引)

---

## 1. 架构与依赖规则

```
┌──────────────────────────────────────────────────────┐
│                      APP 层                           │
│            (应用逻辑：状态管理 / 业务编排)               │
├──────────┬──────────┬───────────────────────────────┤
│ BSP 层   │ DRV 层   │    LIB 层                      │
│ (硬件抽象) │ (器件驱动) │    (算法基础库)                  │
│ CAN/UART/ │ 电机/裁判 │    PID/滤波器/数学             │
│ TIM/SPI   │ 系统/IMU  │                               │
├──────────┴──────────┴───────────────────────────────┤
│                    HAL / CMSIS                        │
│              (STM32F4xx HAL + 寄存器层)                │
└──────────────────────────────────────────────────────┘
```

### 依赖规则

| 方向 | 规则 |
|------|------|
| **APP → DRV / LIB** | ✅ 允许 — APP 是编排者 |
| **APP → BSP** | ❌ 禁止 — App 不应直接操作硬件，通过 DRV 间接访问 |
| **DRV → BSP** | ✅ 允许 — 驱动负责硬件挂接和适配 |
| **BSP → LIB** | ❌ 禁止 — BSP 通过 bsp_cfg.h 获取 HAL 类型, 不依赖 LIB |
| **DRV → LIB** | ✅ 允许 — 驱动可使用数学/滤波工具 |
| **LIB → 任何层** | ❌ 禁止 — LIB 必须纯软件，无任何依赖 |
| **任何层 → APP** | ❌ 禁止 — 严禁反向依赖 |

### DRV 内部结构

每个 Drv 模块分为两部分:

```
┌─────────────────────────────────────┐
│  Core: 纯逻辑/协议层                  │
│  不依赖 BSP/HAL, 通过函数指针解耦      │
├─────────────────────────────────────┤
│  Port: BSP 适配层                    │
│  包含 BSP/HAL 调用, 提供 *_port_* 函数 │
│  将硬件操作挂接到 Core 的函数指针       │
└─────────────────────────────────────┘
```

---

## 2. 目录结构

```
Self_core/
├── project_cfg.h              # 唯一配置入口：选择板型 + 车组
│
├── Lib/                        # ──── LIB 层：纯软件 ────
│   ├── Lib_Typedef/lib_typedef.h
│   ├── Lib_Math/lib_math.h/.c
│   ├── Lib_Filter/lib_filter.h/.c
│   └── Lib_Pid/lib_pid.h/.c
│
├── Bsp/                        # ──── BSP 层：硬件抽象 ────
│   ├── bsp_cfg.h               # 外设句柄映射 (extern)
│   ├── bsp_can.h/.c
│   ├── bsp_uart.h/.c
│   ├── bsp_tim.h/.c
│   ├── bsp_spi.h/.c
│   ├── bsp_gpio.h/.c
│   ├── bsp_i2c.h/.c
│   ├── bsp_adc.h/.c
│   └── bsp_flash.h/.c
│
├── Drv/                        # ──── DRV 层：器件驱动 ────
│   ├── Drv_buzzer/
│   ├── Drv_dbus/
│   ├── Drv_imu/
│   ├── Drv_led/
│   ├── Drv_motor/
│   ├── Drv_power_measure/
│   └── Drv_vofa/
│
└── App/                        # ──── APP 层：业务逻辑 ────
    └── common/
        ├── app_chassis_comm.h/.c  # 底盘 CAN 通信 (0x111-0x115)
        ├── app_gimbal_comm.h/.c   # 云台 CAN 通信 (0x120-0x130, 0x233)
        ├── app_referee.h/.c       # 裁判系统 UART 协议 (USART6)
        ├── app_control.h/.c       # 1kHz 控制循环调度
        └── app_diagnostic.h/.c    # 在线诊断告警
```

---

## 3. LIB 层

> 纯软件层，不依赖任何 MCU 头文件或硬件。

### lib_typedef

**文件:** `Self_core/Lib/Lib_Typedef/lib_typedef.h`

通用类型别名 (`stdint.h` / `stdbool.h` 的 include 封装)。

### lib_filter

**文件:** `Self_core/Lib/Lib_filter/lib_filter.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `lib_filter_lpf_init` | `void lib_filter_lpf_init(lib_filter_lpf_t *lpf, float alpha)` | 初始化一阶低通滤波器 |
| `lib_filter_lpf_update` | `float lib_filter_lpf_update(lib_filter_lpf_t *lpf, float input)` | 更新低通滤波 |
| `lib_filter_swf_init` | `void lib_filter_swf_init(lib_filter_swf_t *swf, float *buf, uint16_t len)` | 初始化滑动窗口滤波器 |
| `lib_filter_swf_update` | `float lib_filter_swf_update(lib_filter_swf_t *swf, float input)` | 更新滑动窗口滤波 |

### lib_math

**文件:** `Self_core/Lib/Lib_math/lib_math.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `lib_math_clamp` | `float lib_math_clamp(float value, float min, float max)` | 限幅 |
| `lib_math_get_shortest_path` | `float lib_math_get_shortest_path(float target, float measure)` | 角度最短路径误差 (rad) |
| `lib_math_rad_normalize` | `float lib_math_rad_normalize(float rad)` | 弧度归一化到 (-PI, PI] |
| `lib_math_deg2rad` | `float lib_math_deg2rad(float deg)` | 角度转弧度 |
| `lib_math_rad2deg` | `float lib_math_rad2deg(float rad)` | 弧度转角度 |
| `lib_math_fast_sigmoid` | `float lib_math_fast_sigmoid(float x)` | 快速 Sigmoid 近似 |
| `lib_math_enc_convert` | `float lib_math_enc_convert(float value, uint8_t dir)` | 编码器值与弧度互转 |

### lib_pid

**文件:** `Self_core/Lib/Lib_pid/lib_pid.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `lib_pid_init` | `void lib_pid_init(lib_pid_t *p, float kp, float ki, float kd, float kff_g, float kff_y, float max_ff_g, float max_ff_y, float min_out, float max_out, float max_iout)` | PID 初始化 |
| `lib_pid_calc` | `float lib_pid_calc(lib_pid_t *pid, float target, float measure)` | 标准 PID 计算 |
| `lib_pid_ff_calc` | `float lib_pid_ff_calc(lib_pid_t *pid, float target, float measure, float ff_g, float ff_y)` | 前馈 PID 计算 |
| `lib_pid_pos_calc` | `float lib_pid_pos_calc(lib_pid_t *pid, float target, float measure, float ff_g, float ff_y, float speed)` | 位置环 PID (含速度微分 + 前馈) |
| `lib_pid_set_2dof_weight` | `void lib_pid_set_2dof_weight(lib_pid_t *pid, float weight_p, float weight_d)` | 设置二自由度权重 |
| `lib_pid_deriv_first_calc` | `float lib_pid_deriv_first_calc(lib_pid_t *pid, float target, float measure)` | 微分先行 PID |
| `lib_pid_2dof_calc` | `float lib_pid_2dof_calc(lib_pid_t *pid, float target, float measure)` | 二自由度 PID |
| `lib_pid_fuzzy_init` | `void lib_pid_fuzzy_init(lib_pid_fuzzy_t *fpid, float kp, float ki, float kd, float min_out, float max_out, float max_iout)` | 模糊 PID 初始化 |
| `lib_pid_fuzzy_cfg` | `void lib_pid_fuzzy_cfg(lib_pid_fuzzy_t *fpid, float ke, float kec, float kp_delta, float ki_delta, float kd_delta)` | 配置模糊 PID 量化因子 |
| `lib_pid_fuzzy_load_default_rules` | `void lib_pid_fuzzy_load_default_rules(lib_pid_fuzzy_t *fpid)` | 载入默认模糊规则表 |
| `lib_pid_fuzzy_calc` | `float lib_pid_fuzzy_calc(lib_pid_fuzzy_t *fpid, float target, float measure)` | 模糊 PID 计算 |

---

## 4. BSP 层

> 硬件抽象层，封装 STM32 HAL 调用。所有函数直接操作外设句柄。

### bsp_cfg

**文件:** `Self_core/Bsp/bsp_cfg.h`

外设句柄 extern 声明（由 CubeMX 生成）。

| 符号 | 类型 | 说明 |
|------|------|------|
| `hcan1` | `CAN_HandleTypeDef` | CAN1 句柄 |
| `hcan2` | `CAN_HandleTypeDef` | CAN2 句柄 |
| `huart1` | `UART_HandleTypeDef` | USART1 句柄 |
| `huart3` | `UART_HandleTypeDef` | USART3 句柄 (DBUS) |
| `huart6` | `UART_HandleTypeDef` | USART6 句柄 |
| `hspi1` | `SPI_HandleTypeDef` | SPI1 句柄 (BMI088) |
| `htim4` | `TIM_HandleTypeDef` | TIM4 句柄 (蜂鸣器 PWM) |
| `htim10` | `TIM_HandleTypeDef` | TIM10 句柄 (IMU 加热 PWM) |
| `htim14` | `TIM_HandleTypeDef` | TIM14 句柄 |

### bsp_can

**文件:** `Self_core/Bsp/bsp_can.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_can_start` | `HAL_StatusTypeDef bsp_can_start(CAN_HandleTypeDef *hcan, uint8_t filter_bank, uint8_t slave_filter_bank)` | 启动 CAN (滤波器 + 中断) |
| `bsp_can_send` | `bsp_can_tx_status_t bsp_can_send(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t data[8])` | 非阻塞发送 (8 字节) |
| `bsp_can_register_rx_callback` | `void bsp_can_register_rx_callback(CAN_HandleTypeDef *hcan, uint32_t std_id, bsp_can_rx_callback_t callback)` | 注册接收回调 |
| `bsp_can_rx_irq_handler` | `void bsp_can_rx_irq_handler(CAN_HandleTypeDef *hcan)` | CAN 接收中断入口 |

### bsp_uart

**文件:** `Self_core/Bsp/bsp_uart.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_uart_send` | `HAL_StatusTypeDef bsp_uart_send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)` | 非阻塞发送 (中断方式) |
| `bsp_uart_register_rx_callback` | `void bsp_uart_register_rx_callback(UART_HandleTypeDef *huart, bsp_uart_rx_callback_t callback)` | 注册接收回调 |
| `bsp_uart_register_tx_callback` | `void bsp_uart_register_tx_callback(UART_HandleTypeDef *huart, bsp_uart_tx_callback_t callback)` | 注册发送完成回调 |
| `bsp_uart_rx_irq_handler` | `void bsp_uart_rx_irq_handler(UART_HandleTypeDef *huart)` | UART 接收中断入口 |
| `bsp_uart_tx_irq_handler` | `void bsp_uart_tx_irq_handler(UART_HandleTypeDef *huart)` | UART 发送中断入口 |

### bsp_tim

**文件:** `Self_core/Bsp/bsp_tim.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_tim_pwm_start` | `void bsp_tim_pwm_start(TIM_HandleTypeDef *htim, uint32_t channel)` | 启动 PWM 输出 |
| `bsp_tim_pwm_stop` | `void bsp_tim_pwm_stop(TIM_HandleTypeDef *htim, uint32_t channel)` | 停止 PWM 输出 |
| `bsp_tim_pwm_set_freq` | `void bsp_tim_pwm_set_freq(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz)` | 设置 PWM 频率 |
| `bsp_tim_pwm_set_compare` | `void bsp_tim_pwm_set_compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t compare)` | 设置 PWM 占空比 |
| `bsp_tim_it_start` | `void bsp_tim_it_start(TIM_HandleTypeDef *htim)` | 启动定时器中断 |
| `bsp_tim_it_stop` | `void bsp_tim_it_stop(TIM_HandleTypeDef *htim)` | 停止定时器中断 |

### bsp_gpio

**文件:** `Self_core/Bsp/bsp_gpio.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_gpio_write_pin` | `void bsp_gpio_write_pin(GPIO_TypeDef *port, uint16_t pin, uint8_t state)` | 写 GPIO 电平 |
| `bsp_gpio_toggle_pin` | `void bsp_gpio_toggle_pin(GPIO_TypeDef *port, uint16_t pin)` | 翻转 GPIO 电平 |

### bsp_spi

**文件:** `Self_core/Bsp/bsp_spi.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_spi_transceive` | `void bsp_spi_transceive(SPI_HandleTypeDef *hspi, const uint8_t *tx, uint8_t *rx, uint16_t size)` | SPI 同步全双工收发 |

### bsp_adc / bsp_flash / bsp_i2c

**文件:** `Self_core/Bsp/bsp_adc.h` / `bsp_flash.h` / `bsp_i2c.h`

预留接口，当前仅有头文件声明，函数实现待补充。

---

## 5. DRV 层

> 每个 Drv 模块分为 Core（纯逻辑）+ Port（BSP 适配）。

### drv_buzzer

**文件:** `Self_core/Drv/Drv_buzzer/drv_buzzer.h`
**依赖:** `lib_typedef.h`
**Port 依赖:** `bsp_cfg.h`, `bsp_tim.h`

#### Core (纯逻辑)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_buzzer_init` | `void drv_buzzer_init(drv_buzzer_set_fn_t set, drv_buzzer_freq_fn_t set_freq, drv_buzzer_duty_fn_t set_duty)` | 初始化 (注册回调) |
| `drv_buzzer_on` | `void drv_buzzer_on(void)` | 打开蜂鸣器 |
| `drv_buzzer_off` | `void drv_buzzer_off(void)` | 关闭蜂鸣器 |
| `drv_buzzer_toggle` | `void drv_buzzer_toggle(void)` | 翻转状态 |
| `drv_buzzer_set_freq` | `void drv_buzzer_set_freq(uint16_t freq_hz)` | 设置音调频率 |
| `drv_buzzer_set_duty` | `void drv_buzzer_set_duty(uint16_t duty)` | 设置占空比 |

#### Port (BSP 适配)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_buzzer_port_init` | `void drv_buzzer_port_init(void)` | 挂接 TIM4_CH3, 调 drv_buzzer_init |

---

### drv_led

**文件:** `Self_core/Drv/Drv_led/drv_led.h`
**依赖:** `lib_typedef.h`
**Port 依赖:** `bsp_cfg.h`, `bsp_gpio.h`

#### Core (纯逻辑)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_led_init` | `void drv_led_init(drv_led_set_fn_t set_r, drv_led_set_fn_t set_g, drv_led_set_fn_t set_b)` | 初始化 (注册回调) |
| `drv_led_set` | `void drv_led_set(drv_led_color_t color, uint8_t state)` | 设置单色亮灭 |
| `drv_led_toggle` | `void drv_led_toggle(drv_led_color_t color)` | 翻转单色 |
| `drv_led_rgb` | `void drv_led_rgb(uint8_t r, uint8_t g, uint8_t b)` | 同时设置 RGB |

#### Port (BSP 适配)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_led_port_init` | `void drv_led_port_init(void)` | 挂接 GPIOH 10/11/12, 调 drv_led_init |

---

### drv_dbus

**文件:** `Self_core/Drv/Drv_dbus/drv_dbus.h`
**依赖:** `lib_typedef.h`
**Port 依赖:** `bsp_cfg.h`

#### Core (纯逻辑)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_dbus_decode` | `void drv_dbus_decode(const uint8_t buffer[18], drv_dbus_data_t *data)` | 解码 18 字节 DBUS 原始数据 |

#### Port (BSP 适配)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_dbus_port_init` | `void drv_dbus_port_init(void)` | 初始化 USART3 DMA + IDLE 中断 |
| `drv_dbus_port_irq_handler` | `void drv_dbus_port_irq_handler(void)` | UART IDLE 中断入口 |
| `drv_dbus_port_get_data` | `const drv_dbus_data_t *drv_dbus_port_get_data(void)` | 获取最新解码数据 |

---

### drv_imu

**文件:** `Self_core/Drv/Drv_imu/drv_imu.h`
**依赖:** `lib_typedef.h`, `math.h`
**Port 依赖:** `bsp_cfg.h`, `bsp_spi.h`, `bsp_gpio.h`, `bsp_tim.h`

#### Core (纯逻辑)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_imu_init` | `void drv_imu_init(drv_imu_t *imu, const drv_imu_bus_t *bus)` | IMU 初始化 |
| `drv_imu_start` | `void drv_imu_start(drv_imu_t *imu)` | 启动 BMI088 (SPI 配置) |
| `drv_imu_set_euler_mode` | `void drv_imu_set_euler_mode(drv_imu_t *imu, drv_imu_euler_mode_t mode)` | 设置欧拉角模式 |
| `drv_imu_read_acc_raw` | `void drv_imu_read_acc_raw(drv_imu_t *imu)` | 读加速度计原始值 |
| `drv_imu_read_gyro_raw` | `void drv_imu_read_gyro_raw(drv_imu_t *imu)` | 读陀螺仪原始值 |
| `drv_imu_data_convert` | `void drv_imu_data_convert(drv_imu_t *imu)` | 原始值转物理量 |
| `drv_imu_read_temp` | `void drv_imu_read_temp(drv_imu_t *imu)` | 读温度 |
| `drv_imu_mahony_update` | `void drv_imu_mahony_update(drv_imu_t *imu, float dt)` | Mahony 姿态解算 |
| `drv_imu_quat_to_euler` | `void drv_imu_quat_to_euler(drv_imu_t *imu)` | 四元数转欧拉角 |
| `drv_imu_calibrate_gyro` | `void drv_imu_calibrate_gyro(drv_imu_t *imu, uint16_t sample_count)` | 陀螺零偏校准 |
| `drv_imu_initial_alignment` | `void drv_imu_initial_alignment(drv_imu_t *imu)` | 初始对准 (加速度计) |
| `drv_imu_calibrate_pose` | `void drv_imu_calibrate_pose(drv_imu_t *imu)` | 记录零位欧拉角 |
| `drv_imu_restart` | `void drv_imu_restart(drv_imu_t *imu)` | 软复位 + 重新启动 |

#### Port (BSP 适配)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_imu_port_init` | `void drv_imu_port_init(drv_imu_t *imu)` | 挂接 SPI1/PA4/PB0, 调 drv_imu_init + drv_imu_start |
| `drv_imu_port_async_start` | `uint8_t drv_imu_port_async_start(void)` | 启动一轮 ACC→GYRO SPI DMA 采样链 |
| `drv_imu_port_snapshot_update` | `uint8_t drv_imu_port_snapshot_update(drv_imu_t *imu)` | 读取最近的完整双缓冲快照 |
| `drv_imu_port_is_online` | `uint8_t drv_imu_port_is_online(uint32_t timeout_ms)` | 检查完整快照是否超时 |
| `drv_imu_port_get_busy_count` | `uint32_t drv_imu_port_get_busy_count(void)` | 获取采样忙跳过次数 |
| `drv_imu_port_get_error_count` | `uint32_t drv_imu_port_get_error_count(void)` | 获取 DMA 启动、超时及传输错误次数 |
| `drv_imu_port_heater_start` | `void drv_imu_port_heater_start(void)` | 启动 TIM10_CH1 加热 PWM |
| `drv_imu_port_heater_set` | `void drv_imu_port_heater_set(uint16_t val)` | 设置加热 PWM 比较值 |
| `drv_imu_port_delay_ms` | `void drv_imu_port_delay_ms(uint32_t ms)` | 毫秒延时 |

---
### drv_led
...
---

### drv_melody

**文件:** `Self_core/Drv/Drv_melody/drv_melody.h`
**依赖:** `lib_typedef.h`
**Port 依赖:** `drv_buzzer.h` (无需独立 port 文件)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_melody_init` | `void drv_melody_init(void)` | 初始化旋律播放器 |
| `drv_melody_play` | `void drv_melody_play(const drv_melody_note_t *notes, uint8_t loop)` | 播放旋律 (loop=1 循环) |
| `drv_melody_stop` | `void drv_melody_stop(void)` | 停止播放 |
| `drv_melody_pause` | `void drv_melody_pause(void)` | 暂停播放 |
| `drv_melody_resume` | `void drv_melody_resume(void)` | 恢复播放 |
| `drv_melody_update` | `void drv_melody_update(void)` | 状态机更新 (每 1ms 调用) |
| `drv_melody_get_state` | `drv_melody_state_t drv_melody_get_state(void)` | 获取播放状态 |

音符定义: `C4=262, D4=294, E4=330, F4=349, G4=392, A4=440, B4=494, C5=523, C6=1047, NOTE_REST=0`

---
### drv_motor

**文件:** `Self_core/Drv/Drv_motor/drv_motor.h`
**依赖:** `lib_typedef.h`
**Port 依赖:** `bsp_cfg.h`, `bsp_can.h`

#### Core (纯逻辑)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_motor_solve_dji_data` | `void drv_motor_solve_dji_data(const uint8_t *data, drv_motor_data_t *cur)` | 解析 DJI 回传数据 (大端) |
| `drv_motor_build_dji_frame_init` | `void drv_motor_build_dji_frame_init(uint8_t *frame)` | 初始化 DJI 电流帧 (全0) |
| `drv_motor_build_dji_frame_set` | `void drv_motor_build_dji_frame_set(uint8_t *frame, uint8_t slot, int16_t current)` | 设置 DJI 帧某路电流 |
| `drv_motor_solve_lk_data` | `void drv_motor_solve_lk_data(const uint8_t *data, drv_motor_data_t *cur)` | 解析翎控回传数据 (小端) |
| `drv_motor_build_lk_read_frame` | `void drv_motor_build_lk_read_frame(uint8_t *frame)` | 构建翎控读取命令帧 |
| `drv_motor_build_lk_frame` | `void drv_motor_build_lk_frame(uint8_t *frame, int16_t current)` | 构建翎控电流指令帧 |

#### Port (BSP 适配)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_motor_port_can_init` | `void drv_motor_port_can_init(void *hcan, uint32_t can_id, void (*rx_cb)(...))` | 注册 CAN 接收回调 |
| `drv_motor_port_can_send` | `void drv_motor_port_can_send(void *hcan, uint32_t std_id, uint8_t *data)` | CAN 非阻塞发送 |
| `drv_motor_port_get_tick` | `uint32_t drv_motor_port_get_tick(void)` | 获取系统 tick (ms) |

---

### drv_power_measure

**文件:** `Self_core/Drv/Drv_power_measure/drv_power_measure.h`
**依赖:** `lib_typedef.h`
**Port 依赖:** `bsp_cfg.h`, `bsp_can.h`

#### Core (纯逻辑)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_power_solve` | `void drv_power_solve(const uint8_t can_data[8], drv_power_data_t *data)` | 解析功率计 CAN 数据 (ID 0x212) |

#### Port (BSP 适配)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_power_port_can_init` | `void drv_power_port_can_init(void *hcan, void (*rx_cb)(...))` | 注册 CAN 接收回调 |
| `drv_power_port_get_tick` | `uint32_t drv_power_port_get_tick(void)` | 获取系统 tick (ms) |

---

### drv_vofa

**文件:** `Self_core/Drv/Drv_vofa/drv_vofa.h`
**依赖:** `lib_typedef.h`
**Port 依赖:** `bsp_cfg.h`, `bsp_uart.h`

#### Core (纯逻辑)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_vofa_init` | `void drv_vofa_init(uint8_t ch_count)` | 初始化 VOFA 通道数 |
| `drv_vofa_pack` | `int drv_vofa_pack(float *fdata, uint8_t *buf, uint16_t *len)` | 打包一帧数据 (float[] + tail) |
| `drv_vofa_tx_complete` | `void drv_vofa_tx_complete(void)` | 发送完成通知 |

#### Port (BSP 适配)

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_vofa_port_init` | `void drv_vofa_port_init(void *huart, void (*tx_cb)(void), uint8_t ch_count)` | 初始化 + 注册 TX 回调 |
| `drv_vofa_port_send` | `void drv_vofa_port_send(void *huart, uint8_t *data, uint16_t len)` | UART 非阻塞发送 |

---

## 6. APP 层

> 业务逻辑层，负责状态管理和模块编排。
> 不直接调用 BSP/HAL，所有硬件访问通过 DRV Port 完成。
> 例外: `app_init.c` 作为系统初始化编排器允许直接包含 BSP。

### app_chassis_comm

**文件:** `Self_core/App/common/app_chassis_comm.h`
**依赖:** `lib_typedef.h`, `bsp_can.h`, `project_cfg.h`

底盘CAN通信协议 (板间通信)。

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_chassis_comm_init` | `void app_chassis_comm_init(void)` | 初始化 (注册 CAN 回调, BOARD_CHASSIS only) |
| `app_chassis_comm_get_speed_cmd` | `const app_chassis_speed_cmd_t *app_chassis_comm_get_speed_cmd(void)` | 获取最新速度指令 (0x111) |
| `app_chassis_comm_get_ackermann_cmd` | `const app_chassis_ackermann_cmd_t *app_chassis_comm_get_ackermann_cmd(void)` | 获取阿克曼指令 (0x113) |
| `app_chassis_comm_get_follow_cmd` | `const app_chassis_follow_cmd_t *app_chassis_comm_get_follow_cmd(void)` | 获取跟随指令 (0x115) |
| `app_chassis_comm_send_power_feedback` | `void app_chassis_comm_send_power_feedback(int16_t power_x100)` | 发送功率反馈 (0x112) |
| `app_chassis_comm_send_speed_cmd` | `void app_chassis_comm_send_speed_cmd(int16_t vx, int16_t vy, int16_t vz, int16_t power_pct)` | 发送速度指令 (0x111, 云台转发用) |

### app_gimbal_comm

**文件:** `Self_core/App/common/app_gimbal_comm.h`
**依赖:** `lib_typedef.h`, `bsp_can.h`, `app_chassis_comm.h`, `project_cfg.h`

云台CAN通信协议 (板间通信)。收到 0x120 自动转发到 0x111。

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_gimbal_comm_init` | `void app_gimbal_comm_init(void)` | 初始化 (注册 CAN 回调, BOARD_GIMBAL only) |
| `app_gimbal_comm_get_radar_speed` | `const app_gimbal_radar_speed_cmd_t *...` | 获取雷达速度指令 (0x120) |
| `app_gimbal_comm_get_speed_no_shoot` | `const app_gimbal_speed_cmd_t *...` | 获取速度不发射指令 (0x121) |
| `app_gimbal_comm_get_angle_no_shoot` | `const app_gimbal_angle_cmd_t *...` | 获取角度不发射指令 (0x123) |
| `app_gimbal_comm_get_speed_shoot` | `const app_gimbal_speed_cmd_t *...` | 获取速度发射指令 (0x125) |
| `app_gimbal_comm_get_angle_shoot` | `const app_gimbal_angle_cmd_t *...` | 获取角度发射指令 (0x127) |
| `app_gimbal_comm_get_control` | `const app_gimbal_control_cmd_t *...` | 获取控制指令 (0x129, DLC=4) |
| `app_gimbal_comm_send_speed_feedback` | `void app_gimbal_comm_send_speed_feedback(float yaw, float pitch)` | 发送速度反馈 (0x122) |
| `app_gimbal_comm_send_angle_feedback` | `void app_gimbal_comm_send_angle_feedback(float yaw, float pitch)` | 发送角度反馈 (0x124, rad) |
| `app_gimbal_comm_send_angle_feedback_v2` | `void app_gimbal_comm_send_angle_feedback_v2(uint16_t yaw, uint16_t pitch, uint16_t roll, uint16_t interval)` | 发送角度反馈v2 (0x130, deg, 65536/rev) |
| `app_gimbal_comm_send_shoot_feedback` | `void app_gimbal_comm_send_shoot_feedback(const uint8_t data[8])` | 发送射击反馈 (0x126, 自定义) |
| `app_gimbal_comm_send_imu_quaternion` | `void app_gimbal_comm_send_imu_quaternion(int16_t q0, int16_t q1, int16_t q2, int16_t q3)` | 发送IMU四元数 (0x233, 除30000后用) |

### app_referee

**文件:** `Self_core/App/common/app_referee.h`
**依赖:** `lib_typedef.h`, `main.h`, `usart.h`

裁判系统串口通信协议 (底盘C板 USART6 直连)。UART DMA+IDLE 接收，SOF=0xA5 帧格式。

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_referee_init` | `void app_referee_init(void)` | 初始化 DMA+IDLE 接收 (BOARD_CHASSIS only) |
| `app_referee_uart_idle_handler` | `void app_referee_uart_idle_handler(void)` | USART6 空闲中断处理 (在 stm32f4xx_it.c 中调用) |
| `app_referee_get_data` | `const referee_global_t *app_referee_get_data(void)` | 获取全局裁判数据 |
| `app_referee_get_and_clear_flags` | `uint32_t app_referee_get_and_clear_flags(void)` | 获取并清除更新标志 |
| `app_referee_is_data_updated` | `int app_referee_is_data_updated(uint32_t flag)` | 检查指定数据是否更新 |

### app_control

**文件:** `Self_core/App/common/app_control.h`
**依赖:** `project_cfg.h`, `app_diagnostic.h`

1kHz 控制循环调度器。

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_control_1khz` | `void app_control_1khz(void)` | 主控制循环 (由 TIM 中断回调调用) |

调度顺序: diagnostic → monitor(TODO) → referee(TODO) → chassis/gimbal/shoot(TODO) → robot-specific(TODO)

### app_diagnostic

**文件:** `Self_core/App/common/app_diagnostic.h`
**依赖:** `lib_typedef.h`, `drv_motor.h`, `drv_led.h`, `drv_buzzer.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_diagnostic_init` | `void app_diagnostic_init(void)` | 初始化诊断模块 |
| `app_diagnostic_register` | `int app_diagnostic_register(app_diagnostic_device_type_t type, uint8_t index, uint32_t timeout_ms)` | 注册设备 (返回 0=成功) |
| `app_diagnostic_heartbeat` | `void app_diagnostic_heartbeat(app_diagnostic_device_type_t type, uint8_t index)` | 喂狗更新心跳 |
| `app_diagnostic_is_online` | `uint8_t app_diagnostic_is_online(app_diagnostic_device_type_t type, uint8_t index)` | 查询在线状态 |
| `app_diagnostic_update` | `void app_diagnostic_update(app_diagnostic_result_t *result)` | 更新诊断 (在线检测+LED/蜂鸣器告警) |

---

## 7. 层间依赖矩阵

### 7.1 文件依赖关系

| 文件 | include 的外部文件 |
|------|-------------------|
| **LIB/lib_typedef.h** | `stdint.h`, `stdbool.h` (C 标准库) |
| **LIB/lib_filter.h** | `lib_typedef.h` |
| **LIB/lib_math.h** | `lib_typedef.h` |
| **LIB/lib_pid.h** | `lib_typedef.h`, `lib_filter.h` |
| **BSP/bsp_cfg.h** | `stm32f4xx_hal.h` (CubeMX) |
| **BSP/bsp_can.h** | `bsp_cfg.h` |
| **BSP/bsp_uart.h** | `bsp_cfg.h` |
| **BSP/bsp_tim.h** | `bsp_cfg.h` |
| **BSP/bsp_gpio.h** | `bsp_cfg.h` |
| **BSP/bsp_spi.h** | `bsp_cfg.h` |
| **BSP/bsp_adc.h** | `bsp_cfg.h` |
| **BSP/bsp_flash.h** | `bsp_cfg.h` |
| **BSP/bsp_i2c.h** | `bsp_cfg.h` |
| **DRV/drv_buzzer.h** | `lib_typedef.h` |
| **DRV/drv_buzzer.c** (port) | `bsp_cfg.h`, `bsp_tim.h` |
| **DRV/drv_led.h** | `lib_typedef.h` |
| **DRV/drv_led.c** (port) | `bsp_cfg.h`, `bsp_gpio.h` |
| **DRV/drv_dbus.h** | `lib_typedef.h` |
| **DRV/drv_dbus.c** (port) | `bsp_cfg.h` |
| **DRV/drv_imu.h** | `lib_typedef.h` |
| **DRV/drv_imu.c** (port) | `bsp_cfg.h`, `bsp_spi.h`, `bsp_gpio.h`, `bsp_tim.h` |
| **DRV/drv_melody.h** | `lib_typedef.h`, `drv_buzzer.h` |
| **DRV/drv_motor.h** | `lib_typedef.h` |
| **DRV/drv_motor.c** (port) | `bsp_cfg.h`, `bsp_can.h` |
| **DRV/drv_power_measure.h** | `lib_typedef.h` |
| **DRV/drv_power_measure.c** (port) | `bsp_cfg.h`, `bsp_can.h` |
| **DRV/drv_vofa.h** | `lib_typedef.h` |
| **DRV/drv_vofa.c** (port) | `bsp_cfg.h`, `bsp_uart.h` |
| **APP/app_chassis_comm.h** | `lib_typedef.h`, `bsp_can.h`, `project_cfg.h` |
| **APP/app_gimbal_comm.h** | `lib_typedef.h`, `bsp_can.h`, `app_chassis_comm.h`, `project_cfg.h` |
| **APP/app_referee.h** | `lib_typedef.h`, `main.h`, `usart.h` |
| **APP/app_control.h** | `project_cfg.h`, `app_diagnostic.h` |
| **APP/app_diagnostic.h** | `lib_typedef.h`, `drv_motor.h`, `drv_led.h`, `drv_buzzer.h` |

### 7.2 层间依赖总结

```
LIB  → 无 (纯 C 标准库)
BSP  → 无 (仅 HAL/CMSIS)
DRV  → BSP (port 部分) + LIB
APP  → DRV + LIB (禁止 → BSP)
Core/ → DRV (stm32f4xx_it.c → drv_dbus_port_irq_handler)
```

### 7.3 外部调用点

| 位置 | 调用 | 说明 |
|------|------|------|
| `Core/Src/stm32f4xx_it.c` | `drv_dbus_port_irq_handler()` | USART3 IDLE 中断 → DBUS 解码 |
| `Core/Src/stm32f4xx_it.c` | `app_referee_uart_idle_handler()` | USART6 IDLE 中断 → 裁判系统解析 |

---

## 8. 头文件索引

| 层级 | 路径 |
|------|------|
| LIB 类型 | `Self_core/Lib/Lib_Typedef/lib_typedef.h` |
| LIB 滤波 | `Self_core/Lib/Lib_filter/lib_filter.h` |
| LIB 数学 | `Self_core/Lib/Lib_math/lib_math.h` |
| LIB PID  | `Self_core/Lib/Lib_pid/lib_pid.h` |
| BSP 配置 | `Self_core/Bsp/bsp_cfg.h` |
| BSP CAN  | `Self_core/Bsp/bsp_can.h` |
| BSP UART | `Self_core/Bsp/bsp_uart.h` |
| BSP TIM  | `Self_core/Bsp/bsp_tim.h` |
| BSP GPIO | `Self_core/Bsp/bsp_gpio.h` |
| BSP SPI  | `Self_core/Bsp/bsp_spi.h` |
| BSP ADC  | `Self_core/Bsp/bsp_adc.h` |
| BSP I2C  | `Self_core/Bsp/bsp_i2c.h` |
| BSP FLASH | `Self_core/Bsp/bsp_flash.h` |
| DRV 蜂鸣器 | `Self_core/Drv/Drv_buzzer/drv_buzzer.h` |
| DRV DBUS | `Self_core/Drv/Drv_dbus/drv_dbus.h` |
| DRV IMU  | `Self_core/Drv/Drv_imu/drv_imu.h` |
| DRV LED  | `Self_core/Drv/Drv_led/drv_led.h` |
| DRV 旋律 | `Self_core/Drv/Drv_melody/drv_melody.h` |
| DRV 电机 | `Self_core/Drv/Drv_motor/drv_motor.h` |
| DRV 功率计 | `Self_core/Drv/Drv_power_measure/drv_power_measure.h` |
| DRV VOFA | `Self_core/Drv/Drv_vofa/drv_vofa.h` |
| APP 底盘通信 | `Self_core/App/common/app_chassis_comm.h` |
| APP 云台通信 | `Self_core/App/common/app_gimbal_comm.h` |
| APP 裁判系统 | `Self_core/App/common/app_referee.h` |
| APP 控制调度 | `Self_core/App/common/app_control.h` |
| APP 诊断 | `Self_core/App/common/app_diagnostic.h` |
| APP 初始化 | `Self_core/App/common/app_init.h` |
| 项目配置 | `Self_core/project_cfg.h` |
