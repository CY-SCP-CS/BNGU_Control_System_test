# 实现逻辑详解 — 分层架构与解耦模式

> 基于 STM32F407 (C板) · APP/BSP/DRV/LIB 四层架构
> 核心目标：App 层零 BSP 依赖，通过 DRV 间接访问硬件

---

## 目录

1. [总体分层架构](#1-总体分层架构)
2. [DRV 内部设计：Core + Port 模式](#2-drv-内部设计core--port-模式)
3. [函数指针解耦详解](#3-函数指针解耦详解)
4. [void* 句柄模式](#4-void-句柄模式)
5. [依赖规则与检查方式](#5-依赖规则与检查方式)
6. [典型数据流](#6-典型数据流)

---

## 1. 总体分层架构

### 1.1 四层

```
┌──────────────────────────────────────────────────────┐
│                      APP 层                           │
│            (业务逻辑：电机管理/诊断告警)                 │
├──────────┬──────────┬───────────────────────────────┤
│ BSP 层   │ DRV 层   │    LIB 层                      │
│ (硬件抽象) │ (器件驱动) │    (算法基础库)                  │
│ CAN/UART/ │ 电机/DBUS │    PID/滤波器/数学             │
│ TIM/SPI   │ /IMU/...  │                               │
├──────────┴──────────┴───────────────────────────────┤
│                    HAL / CMSIS                        │
│              (STM32F4xx HAL + 寄存器层)                │
└──────────────────────────────────────────────────────┘
```

### 1.2 依赖规则

| 方向 | 规则 |
|------|------|
| **APP → DRV / LIB** | ✅ 允许 — APP 是业务编排者 |
| **APP → BSP** | ❌ **禁止** — App 通过 DRV 间接访问硬件 |
| **DRV → BSP** | ✅ 允许 — 驱动负责硬件挂接和适配 |
| **BSP → LIB** | ❌ **禁止** — BSP 通过 `bsp_cfg.h → stm32f4xx_hal.h` 获取类型，不依赖 LIB |
| **DRV → LIB** | ✅ 允许 — 驱动可使用数学/滤波工具 |
| **LIB → 任何层** | ❌ **禁止** — LIB 必须纯软件，零依赖 |
| **任何层 → APP** | ❌ **禁止** — 严禁反向依赖 |

### 1.3 关键设计理念

1. **DRV 是 App 和 BSP 之间的唯一桥梁**。App 不直接 include 任何 `bsp_*.h` 或 HAL 头文件。
2. **LIB 是纯 C 标准库软件**。不 include 任何 STM32 或项目头文件（`stdint.h` / `stdbool.h` 除外）。
3. **BSP 不依赖 LIB**。BSP 通过 `bsp_cfg.h` include `stm32f4xx_hal.h` 获取 `stdint.h` 类型，不需要 LIB。
4. **条件编译是唯二的差异化手段**。`project_cfg.h` 中的 `CURRENT_BOARD` 和 `CURRENT_ROBOT` 宏控制 App 层行为。

---

## 2. DRV 内部设计：Core + Port 模式

每个 DRV 模块在物理上是一个 `.h` / `.c` 文件对，但在逻辑上分为两部分：

```
┌─────────────────────────────────────────────────────┐
│  Core (同一 .c 文件的上半部分)                        │
│  ├── 纯算法/协议逻辑                                  │
│  ├── 零 BSP/HAL 依赖                                 │
│  ├── 通过函数指针调用硬件操作                          │
│  └── 被 APP 和 Port 共同使用                          │
├─────────────────────────────────────────────────────┤
│  Port (同一 .c 文件的下半部分, #if 0 风格隔离)        │
│  ├── 包含 bsp_*.h / HAL 头文件                        │
│  ├── 实现函数指针指向的 static 函数                    │
│  ├── 提供 drv_*_port_init() 挂接 Core                 │
│  └── 提供 drv_*_port_*() 便捷入口 (可选)              │
└─────────────────────────────────────────────────────┘
```

### 为什么要放在同一文件？

- **编译单元内聚**：Core 和 Port 属于同一模块（都是 "蜂鸣器驱动"），放在同一文件便于维护。
- **条件编译的未来选项**：如果需要同一模块支持多平台，可以用 `#if` 切换 Port 实现，Core 完全不变。
- **减少文件数量**：每个驱动一个 `.h` / `.c` 对，不拆分为四个文件。

### 当前模块的 Core/Port 划分

| 模块 | Core 功能 | Port 适配内容 |
|------|-----------|--------------|
| `drv_buzzer` | 开关状态管理、频率/占空比缓存 | TIM4_CH3 PWM 启停 (PD14) |
| `drv_led` | RGB 三色状态管理 | GPIOH 10/11/12 写操作 |
| `drv_dbus` | 18 字节 DBUS 协议解码 | USART3 DMA + IDLE 中断 |
| `drv_imu` | BMI088 寄存器读写、姿态解算 | SPI1 + PA4/PB0 CS 引脚 |
| `drv_motor` | DJI/翎控协议解析、电流帧构建 | CAN 收发回调注册 |
| `drv_power_measure` | 功率计 CAN 数据解析 | CAN 接收回调注册 |
| `drv_vofa` | float[] 打包、发送完成通知 | UART 非阻塞发送 |

---

## 3. 函数指针解耦详解

### 3.1 模式说明

函数指针是 Core 与 Port 之间的"接口契约"。Core 定义函数指针类型（即"我需要什么样的硬件操作"），Port 提供具体实现并通过 `drv_*_init()` 注册到 Core。

```
Core (纯逻辑)           Port (BSP 适配)
    │                        │
    │  定义回调类型            │
    │  typedef void (*set_fn_t)(uint8_t);  │
    │                        │
    │                        │  实现回调
    │                        │  static void port_set(uint8_t s) {
    │                        │      bsp_tim_pwm_start(...);
    │                        │  }
    │                        │
    │  drv_init(set_fn_t) ───┤ 注册回调
    │                        │
    │  调用: s_set_fn(1) ────┼──→ port_set(1) → bsp_tim_pwm_start()
```

### 3.2 示例：蜂鸣器驱动 (`drv_buzzer`)

这是最简单的函数指针用法，适合理解基本模式。

**Step 1 — Core 定义回调类型** (`drv_buzzer.h`)

```c
typedef void (*drv_buzzer_set_fn_t)(uint8_t state);   /* 开关 */
typedef void (*drv_buzzer_freq_fn_t)(uint16_t freq_hz); /* 设频率 */
typedef void (*drv_buzzer_duty_fn_t)(uint16_t duty);    /* 设占空比 */

void drv_buzzer_init(drv_buzzer_set_fn_t set,
                     drv_buzzer_freq_fn_t set_freq,
                     drv_buzzer_duty_fn_t set_duty);
```

**Step 2 — Core 保存回调并调用** (`drv_buzzer.c`)

```c
static drv_buzzer_set_fn_t  s_set;      /* 保存函数指针 */
static drv_buzzer_freq_fn_t s_set_freq;
static uint8_t              s_state;    /* 状态缓存 */

void drv_buzzer_init(drv_buzzer_set_fn_t set, ...) {
    s_set      = set;      /* 把函数指针存起来 */
    s_set_freq = set_freq;
    s_state    = 0;
    s_set(0);              /* 初始关闭 */
}

void drv_buzzer_on(void) {
    s_state = 1;
    s_set(1);              /* 通过函数指针调用 → 实际操控 TIM4 PWM */
}

void drv_buzzer_set_freq(uint16_t freq_hz) {
    s_freq_hz = freq_hz;
    if (s_set_freq)
        s_set_freq(freq_hz);  /* 通过函数指针调用 → 修改 TIM4 ARR */
}
```

**Step 3 — Port 实现具体硬件操作** (`drv_buzzer.c` 下半部分)

```c
#include "bsp_cfg.h"
#include "bsp_tim.h"

static TIM_HandleTypeDef *s_port_htim = &htim4;

static void port_buzzer_set(uint8_t state) {
    if (state)
        bsp_tim_pwm_start(s_port_htim, TIM_CHANNEL_3);
    else
        bsp_tim_pwm_stop(s_port_htim, TIM_CHANNEL_3);
}

static void port_buzzer_set_freq(uint16_t freq_hz) {
    bsp_tim_pwm_set_freq(s_port_htim, TIM_CHANNEL_3, freq_hz);
}

void drv_buzzer_port_init(void) {
    drv_buzzer_init(port_buzzer_set, port_buzzer_set_freq, port_buzzer_set_duty);
}
```

**Step 4 — App 调用**

```c
// app_init.c
#include "drv_buzzer.h"

void app_init(void) {
    drv_buzzer_port_init();  // 一行初始化, 内部挂接所有回调
}

void some_logic(void) {
    drv_buzzer_on();         // App 层完全不知道 TIM4 的存在
    drv_buzzer_set_freq(2000);
}
```

App 层调用 `drv_buzzer_on()` 时，执行链为：

```
drv_buzzer_on()
  → s_set(1)
    → port_buzzer_set(1)
      → bsp_tim_pwm_start(&htim4, TIM_CHANNEL_3)
        → HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3)
```

App 全程不知道 `htim4`、`TIM_CHANNEL_3`、`PD14` 的存在。

### 3.3 示例：IMU 总线抽象 (`drv_imu`)

IMU 需要 SPI 收发 + 两个 CS 引脚控制 + 延时，共 4 个操作。用一个结构体打包传递：

```c
// drv_imu.h — 定义总线操作类型
typedef void (*drv_imu_spi_xfer_t)(void *ctx, const uint8_t *tx,
                                   uint8_t *rx, uint16_t len);
typedef void (*drv_imu_gpio_write_t)(void *ctx, uint8_t state);
typedef void (*drv_imu_delay_t)(uint32_t ms);

typedef struct {
    void               *spi_ctx;      /* SPI 上下文 (本平台传 NULL) */
    drv_imu_spi_xfer_t  spi_xfer;     /* SPI 收发函数             */

    void               *acc_cs_ctx;   /* ACC CS 上下文            */
    drv_imu_gpio_write_t acc_cs;      /* ACC CS 写函数            */

    void               *gyro_cs_ctx;  /* GYRO CS 上下文           */
    drv_imu_gpio_write_t gyro_cs;     /* GYRO CS 写函数           */

    drv_imu_delay_t      delay_ms;    /* 毫秒延时                  */
} drv_imu_bus_t;
```

Core 内通过 `imu->bus.spi_xfer(ctx, tx, rx, len)` 调用 SPI：

```c
// Core 读加速度计原始值 — 纯逻辑, 没有 BSP/HAL
void drv_imu_read_acc_raw(drv_imu_t *imu) {
    uint8_t tx_buf[8] = {0x12 | 0x80, 0, 0, 0, 0, 0, 0, 0};
    uint8_t rx_buf[8];

    BMI088_ACC_CS_LOW(imu, imu->bus.acc_cs_ctx);         // CS 拉低
    imu->bus.spi_xfer(imu->bus.spi_ctx, tx_buf, rx_buf, 8); // SPI 收发
    BMI088_ACC_CS_HIGH(imu, imu->bus.acc_cs_ctx);        // CS 拉高

    imu->acc_raw.x = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    imu->acc_raw.y = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
    imu->acc_raw.z = (int16_t)((rx_buf[7] << 8) | rx_buf[6]);
}
```

Port 填充结构体：

```c
#include "bsp_cfg.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"

static void port_imu_spi_xfer(void *ctx, const uint8_t *tx,
                               uint8_t *rx, uint16_t len) {
    (void)ctx;
    bsp_spi_transceive(&hspi1, tx, rx, len);  // 实际 SPI1 收发
}

static void port_acc_cs_write(void *ctx, uint8_t state) {
    (void)ctx;
    bsp_gpio_write_pin(GPIOA, GPIO_PIN_4, state);  // PA4 = ACC CS
}

void drv_imu_port_init(drv_imu_t *imu) {
    drv_imu_bus_t bus = {
        .spi_xfer    = port_imu_spi_xfer,
        .acc_cs      = port_acc_cs_write,
        .gyro_cs     = port_gyro_cs_write,
        .delay_ms    = drv_imu_port_delay_ms,
    };
    drv_imu_init(imu, &bus);   // 注册所有回调
    drv_imu_start(imu);        // 启动 BMI088
}
```

这种模式的优势：IMU Core 中的 `drv_imu_read_acc_raw()` 是**纯算法代码**，可以在 PC 上编译测试（只需 mock 掉 `drv_imu_bus_t` 中的函数指针）。

### 3.4 函数指针类型表

| 模块 | 回调类型 | 数量 | 用途 |
|------|---------|------|------|
| `drv_buzzer` | `drv_buzzer_set_fn_t` | 1 | PWM 启停 |
| | `drv_buzzer_freq_fn_t` | 1 | PWM 频率设置 |
| | `drv_buzzer_duty_fn_t` | 1 | PWM 占空比设置 |
| `drv_led` | `drv_led_set_fn_t` | 3 | R/G/B 三路 GPIO 写 |
| `drv_imu` | `drv_imu_spi_xfer_t` | 1 | SPI 收发 |
| | `drv_imu_gpio_write_t` | 2 | ACC/GYRO CS 引脚控制 |
| | `drv_imu_delay_t` | 1 | 毫秒延时 |

---

## 4. void\* 句柄模式

### 4.1 解决的问题

App 层需要将 CAN/UART 句柄传递给 DRV Port，但如果 App include `CAN_HandleTypeDef` 就违反了 "App 不依赖 BSP/HAL" 规则。

### 4.2 解决方案：App 层用 `void*`，Port 内部强转

```
App 层:     void *hcan           ← 不关心具体类型
               │
Port 内部:  (CAN_HandleTypeDef *)hcan   ← 在 Port 内强转
               │
               ↓
          bsp_can_send(hcan, ...)   ← BSP 调用
```

### 4.3 示例：电机管理

App 层接口：

```c
// app_motor.h  — App 层头文件, 不 include 任何 BSP/HAL
void app_motor_init(void *hcan, const app_motor_cfg_t *cfgs, uint8_t count);
```

App 层初始化：

```c
// 调用方 (app_init.c, 在 #if CURRENT_BOARD 分支中)
app_motor_init(&hcan1, motor_configs, 4);
//              ^^^^^^
//         传入具体句柄的地址, 类型为 CAN_HandleTypeDef*
//         app_motor_init 将其视为 void*
```

DRV Port 内强转：

```c
void drv_motor_port_can_send(void *hcan, uint32_t std_id, uint8_t *data) {
    bsp_can_send((CAN_HandleTypeDef *)hcan, std_id, data);  // 强转回真实类型
}
```

### 4.4 适用范围

| 模块 | void* 参数 | 内部强转目标 |
|------|-----------|-------------|
| `drv_motor` | `hcan` | `CAN_HandleTypeDef*` |
| `drv_power_measure` | `hcan` | `CAN_HandleTypeDef*` |
| `drv_vofa` | `huart` | `UART_HandleTypeDef*` |
| `drv_imu` | `spi_ctx`, `acc_cs_ctx`, `gyro_cs_ctx` | 本平台传 NULL (单实例) |

> `drv_imu` 的 `void *ctx` 是为多实例预留：如果系统中有两片 BMI088 挂在不同 SPI 总线上，`ctx` 可指向不同的 SPI 句柄。

---

## 5. 依赖规则与检查方式

### 5.1 禁止的依赖链

```
APP → BSP       ❌  App 代码中不能出现 #include "bsp_*.h"
APP → HAL       ❌  App 代码中不能出现 #include "stm32f4xx_hal.h"
BSP → LIB       ❌  BSP 代码中不能出现 #include "lib_*.h"
LIB → 任何项目  ❌  LIB 代码中只能出现 C 标准库头文件
任何 → APP      ❌  非 App 层的代码不能 include "app_*.h"
```

### 5.2 人工检查方法

**检查 App 层是否引用了 BSP**：

```bash
# 在 Self_core/App/ 目录下搜索 BSP/HAL 包含
grep -rn '#include.*bsp_' App/
grep -rn '#include.*stm32' App/
# 期望输出: 空 (无匹配)
```

**检查 BSP 层是否引用了 LIB**：

```bash
# 在 Self_core/Bsp/ 目录下搜索 LIB 包含
grep -rn '#include.*lib_' Bsp/
# 期望输出: 空 (无匹配)
```

**检查 DRV Core 是否引用了 BSP**：

```bash
# 在 Drv 目录下搜索 bsp_ 包含, 排除 Port 部分
# Port 部分包含 BSP 是允许的
```

### 5.3 当前依赖全貌

```
LIB  →  <stdint.h>, <stdbool.h>, <math.h>, <string.h>  (C 标准库)
BSP  →  stm32f4xx_hal.h (通过 bsp_cfg.h)
DRV  →  BSP (仅 Port 部分) + LIB (Core 和 Port 都可)
APP  →  DRV + LIB
      →  ❌BSP  ❌HAL
```

详细依赖矩阵见 `API_Reference.md → 第 7 节`。

---

## 6. 典型数据流

### 6.1 DBUS 遥控数据 (中断驱动)

```
USART3 接收完成
    │
    ↓
USART3_IDLE_IRQHandler()          [Core/Src/stm32f4xx_it.c]
    │  drv_dbus_port_irq_handler()
    ↓
drv_dbus_port_irq_handler()        [Drv/Drv_dbus/drv_dbus.c Port]
    │  __HAL_UART_GET_FLAG, HAL_UART_DMAStop
    │  drv_dbus_decode(buffer, &data)
    │  HAL_UART_Receive_DMA  (重新启动 DMA)
    ↓
drv_dbus_decode(buffer, &data)     [Drv/Drv_dbus/drv_dbus.c Core]
    │  解析 18 字节 DBUS 协议
    ↓
APP 通过 drv_dbus_port_get_data()  读取数据
    │  const drv_dbus_data_t *data = drv_dbus_port_get_data();
    │  data->ch[0], data->sw[0], ...
    ↓
App 控制逻辑使用遥控值
```

### 6.2 电机控制 (1kHz 循环)

```
app_control_1khz()                  [App/common/system/app_control.c]
    │
    ├── app_motor_refresh_online()
    │     → drv_motor_port_get_tick()     [Drv/Drv_motor Port]
    │         → HAL_GetTick()
    │
    ├── app_motor_set_current(0, 500)    设置电流
    │
    ├── app_motor_send_frame(0x200)      发送控制帧
    │     → drv_motor_build_dji_frame_init(frame)   [Core, 纯逻辑]
    │     → drv_motor_build_dji_frame_set(frame, ...) [Core, 纯逻辑]
    │     → drv_motor_port_can_send(hcan, 0x200, frame) [Port]
    │         → bsp_can_send(hcan, 0x200, frame)   [BSP]
    │             → HAL_CAN_AddTxMessage(...)       [HAL]
    │
    └── 电机回传触发 CAN RX 中断
          → bsp_can_rx_irq_handler()   [BSP]
          → motor_rx_callback()        [App, 按 can_id 匹配]
              → drv_motor_solve_dji_data()  [DRV Core, 大端解析]
              → drv_motor_port_get_tick()   [DRV Port, 记录时间戳]
```

### 6.3 诊断告警 (LED + 蜂鸣器)

```
app_diagnostic_update(&result)       [App/common/diagnostic/app_diagnostic.c]
    │  遍历电机/功率计在线状态
    │
    ├── 检测到离线:
    │   → drv_led_rgb(1, 0, 0)      [Drv/Drv_led Core]
    │       → s_set[0](1)           [函数指针]
    │           → port_led_r_set(1) [Port]
    │               → bsp_gpio_write_pin(GPIOH, GPIO_PIN_10, 1) [BSP]
    │
    ├── 全部在线:
    │   → drv_led_rgb(0, 1, 0)      [绿色]
    │
    └── 严重故障:
        → drv_buzzer_on()           [Drv/Drv_buzzer Core]
            → s_set(1)              [函数指针]
                → port_buzzer_set(1) [Port]
                    → bsp_tim_pwm_start(&htim4, TIM_CHANNEL_3) [BSP]
```

---

## 附录：模式总结

| 模式 | 适用场景 | 示例 | 优点 |
|------|---------|------|------|
| **函数指针注册** | 设备有明确的操作集 (on/off/set_freq) | `drv_buzzer`、`drv_led` | 简单直观，头文件可见全部回调类型 |
| **总线结构体** | 设备需要多个关联操作 (SPI+CS+延时) | `drv_imu` | 结构体打包，一次性注册 |
| **void\* 句柄** | 需要传递 HAL 句柄但不暴露类型 | `drv_motor`、`drv_vofa` | 在 App 层隐藏 BSP 类型 |
| **Port 便捷函数** | 纯 BSP 转发，无状态管理 | `drv_motor_port_can_send` | 保持接口一致性 |
