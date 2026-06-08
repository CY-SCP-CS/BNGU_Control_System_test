# API Reference — BNGU Control System

> 本文档列出 APP 层以下（BSP / DRV / LIB / APP Bridge）所有公开函数签名，按模块组织。

---

## 目录

- [BSP 层](#bsp-层)
  - [bsp_can](#bsp_can)
  - [bsp_gpio](#bsp_gpio)
  - [bsp_spi](#bsp_spi)
  - [bsp_tim](#bsp_tim)
  - [bsp_uart](#bsp_uart)
  - [bsp_adc / bsp_flash / bsp_i2c](#bsp_adc--bsp_flash--bsp_i2c-占位)
- [DRV 层](#drv-层)
  - [drv_buzzer](#drv_buzzer)
  - [drv_dbus](#drv_dbus)
  - [drv_imu](#drv_imu)
  - [drv_led](#drv_led)
  - [drv_motor](#drv_motor)
  - [drv_power_measure](#drv_power_measure)
  - [drv_vofa](#drv_vofa)
- [LIB 层](#lib-层)
  - [lib_filter](#lib_filter)
  - [lib_math](#lib_math)
  - [lib_pid](#lib_pid)
- [APP Bridge 层](#app-bridge-层)
  - [app_buzzer](#app_buzzer)
  - [app_dbus](#app_dbus)
  - [app_imu](#app_imu)
  - [app_led](#app_led)
  - [app_vofa](#app_vofa)

---

## BSP 层

### bsp_can

**头文件:** `Self_core/Bsp/bsp_can.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_can_start` | `HAL_StatusTypeDef bsp_can_start(CAN_HandleTypeDef *hcan, uint8_t filter_bank)` | 启动 CAN：配置 16-bit IDMASK 滤波器（全通），开启中断 |
| `bsp_can_send` | `bsp_can_tx_status_t bsp_can_send(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t data[8])` | 非阻塞发送 8 字节 CAN 帧 |
| `bsp_can_register_rx_callback` | `void bsp_can_register_rx_callback(CAN_HandleTypeDef *hcan, uint32_t std_id, bsp_can_rx_callback_t callback)` | 注册接收回调（按 ID 分发） |
| `bsp_can_rx_irq_handler` | `void bsp_can_rx_irq_handler(CAN_HandleTypeDef *hcan)` | 中断入口，分发到已注册的回调 |

**类型：**
- `bsp_can_rx_callback_t` = `void (*)(uint8_t *data, uint8_t len)`
- `bsp_can_tx_status_t` 枚举: `BSP_CAN_TX_OK`, `BSP_CAN_TX_BUSY`, `BSP_CAN_TX_ERROR`

---

### bsp_gpio

**头文件:** `Self_core/Bsp/bsp_gpio.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_gpio_write_pin` | `void bsp_gpio_write_pin(GPIO_TypeDef *port, uint16_t pin, uint8_t state)` | 写 GPIO 电平（0/1） |
| `bsp_gpio_toggle_pin` | `void bsp_gpio_toggle_pin(GPIO_TypeDef *port, uint16_t pin)` | 翻转 GPIO 电平 |

---

### bsp_spi

**头文件:** `Self_core/Bsp/bsp_spi.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_spi_transceive` | `void bsp_spi_transceive(SPI_HandleTypeDef *hspi, const uint8_t *tx, uint8_t *rx, uint16_t size)` | SPI 同步全双工收发 |

---

### bsp_tim

**头文件:** `Self_core/Bsp/bsp_tim.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_tim_pwm_start` | `void bsp_tim_pwm_start(TIM_HandleTypeDef *htim, uint32_t channel)` | 启动 PWM 输出 |
| `bsp_tim_pwm_stop` | `void bsp_tim_pwm_stop(TIM_HandleTypeDef *htim, uint32_t channel)` | 停止 PWM 输出 |
| `bsp_tim_pwm_set_freq` | `void bsp_tim_pwm_set_freq(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t freq_hz)` | 设 PWM 频率（自动计算 PSC+ARR，占空比 50%） |
| `bsp_tim_pwm_set_compare` | `void bsp_tim_pwm_set_compare(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t compare)` | 设 PWM 比较值（占空比） |
| `bsp_tim_it_start` | `void bsp_tim_it_start(TIM_HandleTypeDef *htim)` | 启动定时器基础定时中断 |
| `bsp_tim_it_stop` | `void bsp_tim_it_stop(TIM_HandleTypeDef *htim)` | 停止定时器基础定时中断 |

---

### bsp_uart

**头文件:** `Self_core/Bsp/bsp_uart.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `bsp_uart_send` | `HAL_StatusTypeDef bsp_uart_send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)` | 非阻塞中断发送 |
| `bsp_uart_register_rx_callback` | `void bsp_uart_register_rx_callback(UART_HandleTypeDef *huart, bsp_uart_rx_callback_t callback)` | 注册接收回调 |
| `bsp_uart_register_tx_callback` | `void bsp_uart_register_tx_callback(UART_HandleTypeDef *huart, bsp_uart_tx_callback_t callback)` | 注册发送完成回调 |
| `bsp_uart_rx_irq_handler` | `void bsp_uart_rx_irq_handler(UART_HandleTypeDef *huart)` | 接收中断入口 |
| `bsp_uart_tx_irq_handler` | `void bsp_uart_tx_irq_handler(UART_HandleTypeDef *huart)` | 发送中断入口 |

**类型：**
- `bsp_uart_rx_callback_t` = `void (*)(uint8_t *data, uint16_t len)`
- `bsp_uart_tx_callback_t` = `void (*)(void)`

---

### bsp_adc / bsp_flash / bsp_i2c (占位)

当前仅有空的 `.h` / `.c` 文件，无公开函数声明，待实现。

---

## DRV 层

### drv_buzzer

**头文件:** `Self_core/Drv/Drv_buzzer/drv_buzzer.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_buzzer_init` | `void drv_buzzer_init(drv_buzzer_set_fn_t set, drv_buzzer_freq_fn_t set_freq, drv_buzzer_duty_fn_t set_duty)` | 初始化（注册函数指针，默认关闭） |
| `drv_buzzer_on` | `void drv_buzzer_on(void)` | 开启蜂鸣器 |
| `drv_buzzer_off` | `void drv_buzzer_off(void)` | 关闭蜂鸣器 |
| `drv_buzzer_toggle` | `void drv_buzzer_toggle(void)` | 翻转蜂鸣器状态 |
| `drv_buzzer_set_freq` | `void drv_buzzer_set_freq(uint16_t freq_hz)` | 设置频率 |
| `drv_buzzer_set_duty` | `void drv_buzzer_set_duty(uint16_t duty)` | 设置占空比比较值 |

**类型：**
- `drv_buzzer_set_fn_t` = `void (*)(uint8_t state)`
- `drv_buzzer_freq_fn_t` = `void (*)(uint16_t freq_hz)`
- `drv_buzzer_duty_fn_t` = `void (*)(uint16_t duty)`

---

### drv_dbus

**头文件:** `Self_core/Drv/Drv_dbus/drv_dbus.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_dbus_decode` | `void drv_dbus_decode(const uint8_t buffer[18], drv_dbus_data_t *data)` | 解码 18 字节 DBUS 原始数据 |

**数据结构：** `drv_dbus_data_t` 含：
- `rc.ch[4]` — 摇杆通道 (0~1684)
- `rc.s1`, `rc.s2` — 拨轮开关 (1/2/3)
- `rc.rolling_wheel` — 滚轮
- `mouse.x/y/z` — 鼠标位移
- `mouse.left/right` — 鼠标按键
- `keyboard.w/s/a/d/q/e/shift/ctrl` — 键盘按键

---

### drv_imu

**头文件:** `Self_core/Drv/Drv_imu/drv_imu.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_imu_init` | `void drv_imu_init(drv_imu_t *imu, const drv_imu_bus_t *bus)` | IMU 句柄初始化（参数置零，默认欧拉模式 XYZ） |
| `drv_imu_start` | `void drv_imu_start(drv_imu_t *imu)` | 启动 BMI088（ACC+GYRO 配置，软复位） |
| `drv_imu_set_euler_mode` | `void drv_imu_set_euler_mode(drv_imu_t *imu, drv_imu_euler_mode_t mode)` | 设置欧拉角模式 |
| `drv_imu_read_acc_raw` | `void drv_imu_read_acc_raw(drv_imu_t *imu)` | 读取 ACC 原始数据（6 字节突发） |
| `drv_imu_read_gyro_raw` | `void drv_imu_read_gyro_raw(drv_imu_t *imu)` | 读取 GYRO 原始数据（6 字节突发） |
| `drv_imu_data_convert` | `void drv_imu_data_convert(drv_imu_t *imu)` | 原始值转物理量（g, °/s） |
| `drv_imu_read_temp` | `void drv_imu_read_temp(drv_imu_t *imu)` | 读取 ACC 温度 |
| `drv_imu_mahony_update` | `void drv_imu_mahony_update(drv_imu_t *imu, float dt)` | Mahony 滤波更新（融合 acc+gyro，更新四元数） |
| `drv_imu_quat_to_euler` | `void drv_imu_quat_to_euler(drv_imu_t *imu)` | 四元数 → 欧拉角（按当前模式） |
| `drv_imu_calibrate_gyro` | `void drv_imu_calibrate_gyro(drv_imu_t *imu, uint16_t sample_count)` | 陀螺零偏校准 |
| `drv_imu_initial_alignment` | `void drv_imu_initial_alignment(drv_imu_t *imu)` | 基于加速度计的初始对准 |
| `drv_imu_calibrate_pose` | `void drv_imu_calibrate_pose(drv_imu_t *imu)` | 记录当前位置为零位偏置 |
| `drv_imu_restart` | `void drv_imu_restart(drv_imu_t *imu)` | 软复位 + 重新启动 |

**类型：**
- `drv_imu_euler_mode_t` 枚举: `DRV_IMU_EULER_XYZ`(锁 pitch), `ZYX`(锁 roll), `YXZ`(锁 yaw)
- `drv_imu_bus_t` — 总线函数指针结构体（spi_xfer, acc_cs, gyro_cs, delay_ms）
- `drv_imu_t` — 主句柄（含 raw/real/quat/euler/offset/temperature 等）

---

### drv_led

**头文件:** `Self_core/Drv/Drv_led/drv_led.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_led_init` | `void drv_led_init(drv_led_set_fn_t set_r, drv_led_set_fn_t set_g, drv_led_set_fn_t set_b)` | 初始化（注册函数指针，默认全灭） |
| `drv_led_set` | `void drv_led_set(drv_led_color_t color, uint8_t state)` | 设置单色亮灭 |
| `drv_led_toggle` | `void drv_led_toggle(drv_led_color_t color)` | 翻转单色 |
| `drv_led_rgb` | `void drv_led_rgb(uint8_t r, uint8_t g, uint8_t b)` | 同时设置 RGB |

**类型：**
- `drv_led_color_t` 枚举: `DRV_LED_R`, `DRV_LED_G`, `DRV_LED_B`
- `drv_led_set_fn_t` = `void (*)(uint8_t state)`

---

### drv_motor

**头文件:** `Self_core/Drv/Drv_motor/drv_motor.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_motor_solve_dji_data` | `void drv_motor_solve_dji_data(const uint8_t *data, drv_motor_data_t *cur)` | 解析 DJI 回传（大端） |
| `drv_motor_build_dji_frame_init` | `void drv_motor_build_dji_frame_init(uint8_t *frame)` | 初始化 DJI 电流帧（全零） |
| `drv_motor_build_dji_frame_set` | `void drv_motor_build_dji_frame_set(uint8_t *frame, uint8_t slot, int16_t current)` | 设置 DJI 帧某路电流 |
| `drv_motor_solve_lk_data` | `void drv_motor_solve_lk_data(const uint8_t *data, drv_motor_data_t *cur)` | 解析翎控回传（小端） |
| `drv_motor_build_lk_read_frame` | `void drv_motor_build_lk_read_frame(uint8_t *frame)` | 构建翎控读取命令帧 |
| `drv_motor_build_lk_frame` | `void drv_motor_build_lk_frame(uint8_t *frame, int16_t current)` | 构建翎控电流指令帧 |

**数据结构：** `drv_motor_data_t` 含 `cmd_id`, `speed`, `angle`, `temperature`, `current`, `power`

---

### drv_power_measure

**头文件:** `Self_core/Drv/Drv_power_measure/drv_power_measure.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_power_solve` | `void drv_power_solve(const uint8_t can_data[8], drv_power_data_t *data)` | 解析功率计 CAN 数据（ID 0x212） |

**数据结构：** `drv_power_data_t` 含 `bat_v`(0.01V), `bat_i`(0.01A), `cap_v`(0.01V), `ch_i`(0.01A), `power`(W)

---

### drv_vofa

**头文件:** `Self_core/Drv/Drv_vofa/drv_vofa.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `drv_vofa_init` | `void drv_vofa_init(uint8_t ch_count)` | 初始化（设置通道数） |
| `drv_vofa_pack` | `int drv_vofa_pack(float *fdata, uint8_t *buf, uint16_t *len)` | 打包一帧 float 数据 + 帧尾；返回 0=成功，-1=忙 |
| `drv_vofa_tx_complete` | `void drv_vofa_tx_complete(void)` | 发送完成通知（释放忙标志） |

---

## LIB 层

### lib_filter

**头文件:** `Self_core/Lib/Lib_filter/lib_filter.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `lib_filter_lpf_init` | `void lib_filter_lpf_init(lib_filter_lpf_t *lpf, float alpha)` | 一阶低通初始化 |
| `lib_filter_lpf_update` | `float lib_filter_lpf_update(lib_filter_lpf_t *lpf, float input)` | 低通滤波更新 |
| `lib_filter_swf_init` | `void lib_filter_swf_init(lib_filter_swf_t *swf, float *buf, uint16_t len)` | 滑动窗口初始化 |
| `lib_filter_swf_update` | `float lib_filter_swf_update(lib_filter_swf_t *swf, float input)` | 滑窗滤波更新（返回均值） |

---

### lib_math

**头文件:** `Self_core/Lib/Lib_math/lib_math.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `lib_math_clamp` | `float lib_math_clamp(float value, float min, float max)` | 限幅 |
| `lib_math_get_shortest_path` | `float lib_math_get_shortest_path(float target, float measure)` | 角度最短路径误差 (rad) |
| `lib_math_rad_normalize` | `float lib_math_rad_normalize(float rad)` | 弧度归一化到 (-PI, PI] |
| `lib_math_deg2rad` | `float lib_math_deg2rad(float deg)` | 角度转弧度 |
| `lib_math_rad2deg` | `float lib_math_rad2deg(float rad)` | 弧度转角度 |
| `lib_math_fast_sigmoid` | `float lib_math_fast_sigmoid(float x)` | 快速 Sigmoid 近似 |
| `lib_math_enc_convert` | `float lib_math_enc_convert(float value, uint8_t dir)` | 编码器值 ↔ 弧度互转 |

转换方向宏: `LIB_MATH_ENC13_TO_RAD`, `RAD_TO_ENC13`, `ENC16_TO_RAD`, `RAD_TO_ENC16`

---

### lib_pid

**头文件:** `Self_core/Lib/Lib_pid/lib_pid.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `lib_pid_init` | `void lib_pid_init(lib_pid_t *p, float kp, float ki, float kd, float kff_g, float kff_y, float max_ff_g, float max_ff_y, float min_out, float max_out, float max_iout)` | PID 初始化 |
| `lib_pid_calc` | `float lib_pid_calc(lib_pid_t *pid, float target, float measure)` | 标准 PID 计算 |
| `lib_pid_ff_calc` | `float lib_pid_ff_calc(lib_pid_t *pid, float target, float measure, float ff_g, float ff_y)` | 前馈 PID 计算 |
| `lib_pid_pos_calc` | `float lib_pid_pos_calc(lib_pid_t *pid, float target, float measure, float ff_g, float ff_y, float speed)` | 位置环 PID（微分对速度求导） |
| `lib_pid_set_2dof_weight` | `void lib_pid_set_2dof_weight(lib_pid_t *pid, float weight_p, float weight_d)` | 设二自由度加权系数 |
| `lib_pid_deriv_first_calc` | `float lib_pid_deriv_first_calc(lib_pid_t *pid, float target, float measure)` | 微分先行 PID |
| `lib_pid_2dof_calc` | `float lib_pid_2dof_calc(lib_pid_t *pid, float target, float measure)` | 二自由度 PID |
| `lib_pid_fuzzy_init` | `void lib_pid_fuzzy_init(lib_pid_fuzzy_t *fpid, float kp, float ki, float kd, float min_out, float max_out, float max_iout)` | 模糊 PID 初始化 |
| `lib_pid_fuzzy_cfg` | `void lib_pid_fuzzy_cfg(lib_pid_fuzzy_t *fpid, float ke, float kec, float kp_delta, float ki_delta, float kd_delta)` | 配置模糊量化/比例因子 |
| `lib_pid_fuzzy_load_default_rules` | `void lib_pid_fuzzy_load_default_rules(lib_pid_fuzzy_t *fpid)` | 载入默认模糊规则表 |
| `lib_pid_fuzzy_calc` | `float lib_pid_fuzzy_calc(lib_pid_fuzzy_t *fpid, float target, float measure)` | 模糊 PID 计算 |

---

## APP Bridge 层

### app_buzzer

**头文件:** `Self_core/App/common/bridge/app_buzzer.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_buzzer_init` | `void app_buzzer_init(void)` | 初始化蜂鸣器（挂接 TIM4_CH3 PWM → drv_buzzer） |

---

### app_dbus

**头文件:** `Self_core/App/common/bridge/app_dbus.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_dbus_init` | `void app_dbus_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)` | 初始化 DBUS（UART DMA + IDLE 中断） |
| `app_dbus_irq_handler` | `void app_dbus_irq_handler(void)` | IDLE 中断入口（在 USART_IRQHandler 中调用） |
| `app_dbus_get_data` | `const drv_dbus_data_t *app_dbus_get_data(void)` | 获取最新解码数据（只读指针） |

---

### app_imu

**头文件:** `Self_core/App/common/bridge/app_imu.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_imu_init` | `void app_imu_init(drv_imu_t *imu)` | 初始化 BMI088（挂接 SPI1 + CS PA4/ PB0 + 启动芯片） |
| `app_imu_calibrate` | `int app_imu_calibrate(drv_imu_t *imu)` | 开机自校准（加热→陀螺零偏→初始对准→Mahony 收敛→记录零位） |

---

### app_led

**头文件:** `Self_core/App/common/bridge/app_led.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_led_init` | `void app_led_init(void)` | 初始化 LED（PH10-R, PH11-G, PH12-B → drv_led） |

---

### app_vofa

**头文件:** `Self_core/App/common/bridge/app_vofa.h`

| 函数 | 签名 | 说明 |
|------|------|------|
| `app_vofa_init` | `void app_vofa_init(UART_HandleTypeDef *huart, uint8_t ch_count)` | 初始化 VOFA 发送通道 |
| `app_vofa_send` | `void app_vofa_send(float *fdata)` | 非阻塞发送一帧数据 |

---

## 头文件索引

| 层级 | 路径 |
|------|------|
| BSP 配置 | `Self_core/Bsp/bsp_cfg.h` |
| BSP CAN | `Self_core/Bsp/bsp_can.h` |
| BSP GPIO | `Self_core/Bsp/bsp_gpio.h` |
| BSP SPI  | `Self_core/Bsp/bsp_spi.h` |
| BSP TIM  | `Self_core/Bsp/bsp_tim.h` |
| BSP UART | `Self_core/Bsp/bsp_uart.h` |
| DRV 蜂鸣器 | `Self_core/Drv/Drv_buzzer/drv_buzzer.h` |
| DRV DBUS | `Self_core/Drv/Drv_dbus/drv_dbus.h` |
| DRV IMU  | `Self_core/Drv/Drv_imu/drv_imu.h` |
| DRV LED  | `Self_core/Drv/Drv_led/drv_led.h` |
| DRV 电机 | `Self_core/Drv/Drv_motor/drv_motor.h` |
| DRV 功率计 | `Self_core/Drv/Drv_power_measure/drv_power_measure.h` |
| DRV VOFA | `Self_core/Drv/Drv_vofa/drv_vofa.h` |
| LIB 类型 | `Self_core/Lib/Lib_Typedef/lib_typedef.h` |
| LIB 滤波 | `Self_core/Lib/Lib_filter/lib_filter.h` |
| LIB 数学 | `Self_core/Lib/Lib_math/lib_math.h` |
| LIB PID  | `Self_core/Lib/Lib_pid/lib_pid.h` |
| APP 蜂鸣器 | `Self_core/App/common/bridge/app_buzzer.h` |
| APP DBUS | `Self_core/App/common/bridge/app_dbus.h` |
| APP IMU  | `Self_core/App/common/bridge/app_imu.h` |
| APP LED  | `Self_core/App/common/bridge/app_led.h` |
| APP VOFA | `Self_core/App/common/bridge/app_vofa.h` |
