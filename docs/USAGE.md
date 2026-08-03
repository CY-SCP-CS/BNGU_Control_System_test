# BNGU Control System — 架构使用指南

> STM32F407 四层架构 (LIB / BSP / DRV / APP)
> 适用车型: 英雄 / 步兵 / 哨兵   |   板型: 底盘 / 云台

---

## 目录

1. [快速开始](#1-快速开始)
2. [项目配置](#2-项目配置)
3. [启动流程](#3-启动流程)
4. [控制循环](#4-控制循环)
5. [新增 DRV 模块](#5-新增-drv-模块)
6. [CAN 通信协议](#6-can-通信协议)
7. [调试与诊断](#7-调试与诊断)
8. [PID 调参指南](#8-pid-调参指南)
9. [常见问题](#9-常见问题)

---

## 1. 快速开始

### 1.1 开发环境

| 工具 | 版本 |
|------|------|
| CubeMX | STM32CubeMX 6.x |
| 工具链 | STM32CubeCLT (GCC arm-none-eabi 14.x) |
| 构建 | CMake 3.22+ / Ninja |
| 调试 | OpenOCD + GDB, 或 Keil MDK |
| 串口调试 | VOFA+ (波特率 115200) |

### 1.2 首次编译

```bash
# 配置 (仅需一次)
cmake --preset Debug

# 编译
cmake --build build/Debug -j

# 烧录 (OpenOCD)
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Debug/BNGU_Control_System.elf verify reset exit"
```

---

## 2. 项目配置

### 2.1 唯一配置入口: `Self_core/project_cfg.h`

```c
// 选择主板类型
#define CURRENT_BOARD   BOARD_CHASSIS   // BOARD_CHASSIS(0) 或 BOARD_GIMBAL(1)

// 选择车组类型
#define CURRENT_ROBOT   ROBOT_HERO      // ROBOT_HERO(0) / ROBOT_INFANTRY(1) / ROBOT_SENTRY(2)
```

编译时 `#if CURRENT_BOARD` / `#if CURRENT_ROBOT` 决定哪些代码被包含。

### 2.2 CubeMX 外设分配

| 外设 | 引脚 | 用途 | 板型 |
|------|------|------|------|
| CAN1 | PD0(RX), PD1(TX) | 主CAN总线 (电机/板间通信) | 两板共用 |
| CAN2 | PB5(RX), PB6(TX) | 辅CAN总线 | 两板共用 |
| SPI1 | PB3/PB4/PA7 | BMI088 IMU | 两板共用 |
| USART1 | PA9(TX), PB7(RX) | VOFA+ 调试输出 (115200) | 两板共用 |
| USART3 | PC10(TX), PC11(RX) | DBUS 遥控器 (100000, 9E1) | 两板共用 |
| USART6 | PG9(RX), PG14(TX) | 裁判系统 (115200, 底盘专用) | 底盘 |
| TIM4_CH3 | PD14 | 蜂鸣器 PWM | 两板共用 |
| TIM10_CH1 | PF6 | IMU 加热 PWM | 两板共用 |
| TIM14 | (无引脚) | 1kHz 控制周期定时器 | 两板共用 |
| GPIOH 10/11/12 | PH10/11/12 | RGB LED (R/G/B) | 两板共用 |
| GPIOA 4 | PA4 | BMI088 ACC CS | 两板共用 |
| GPIOB 0 | PB0 | BMI088 GYRO CS | 两板共用 |

### 2.3 中断优先级

| 中断 | 优先级 | 用途 |
|------|--------|------|
| USART3 | 0,0 | DBUS DMA 接收 |
| USART6 | 0,0 | 裁判系统 DMA 接收 |
| USART1 | 0,0 | VOFA 发送完成 |
| CAN1/2 | 0,0 | 电机/板间通信 |
| TIM14 | - | 1kHz 控制循环 |

---

## 3. 启动流程

```
上电
  │
  ▼
HAL_Init()                    ← CubeMX 生成
SystemClock_Config()          ← 168MHz (HSE 12MHz × PLL)
MX_GPIO / DMA / CAN / SPI
  / TIM / USART _Init()       ← 外设初始化
  │
  ▼
app_init()                    ← 应用层初始化
  ├── bsp_can_start() ×2      ← CAN1/CAN2 滤波+启动
  ├── drv_*_port_init() ×7    ← LED/蜂鸣器/DBUS/IMU/VOFA/旋律/电机
  ├── app_diagnostic_init()   ← 设备诊断注册
  ├── app_chassis_comm_init() ← 底盘CAN RX (BOARD_CHASSIS only)
  ├── app_gimbal_comm_init()  ← 云台CAN RX (BOARD_GIMBAL only)
  └── app_referee_init()      ← 裁判系统UART (BOARD_CHASSIS only)
  │
  ▼
用户需要手动添加:
  bsp_tim_register_period_callback(&htim14, app_timer_1khz_cb);
  bsp_tim_it_start(&htim14);
  │
  ▼
while (1) { }                 ← 主循环空闲，所有逻辑在中断中执行
```

**注意**: `main.c` 的 `while(1)` 当前为空。需要在 CubeMX 的 main.c USER CODE 区域添加:
```c
/* USER CODE BEGIN 2 */
app_init();
bsp_tim_register_period_callback(&htim14, app_timer_1khz_cb);
bsp_tim_it_start(&htim14);
/* USER CODE END 2 */
```

---

## 4. 控制循环

### 4.1 调度架构

TIM14 每 1ms 触发一次中断 → `app_control_1khz()`:

```
app_control_1khz()  @1kHz
  │
  ├── app_diagnostic_update()         ← 设备心跳检测 + LED告警
  ├── app_monitor_update()            ← TODO: CPU负载/总线负载
  ├── app_referee_update()            ← TODO: 裁判数据处理
  │
  ├── #if BOARD_CHASSIS
  │     └── app_chassis_control()      ← TODO: 底盘控制
  │
  ├── #else (BOARD_GIMBAL)
  │     ├── app_gimbal_control()       ← TODO: 云台控制
  │     └── app_shoot_control()        ← TODO: 射击控制
  │
  └── #if ROBOT_HERO / INFANTRY / SENTRY
        └── 车型特殊逻辑              ← TODO
```

### 4.2 典型底盘控制循环实现 (伪代码)

```c
void app_chassis_control(void)
{
    // 1. 读取 DBUS 遥控器数据
    const drv_dbus_data_t *dbus = drv_dbus_port_get_data();

    // 2. 读取 CAN 速度指令 (可能来自云台转发)
    const app_chassis_speed_cmd_t *cmd = app_chassis_comm_get_speed_cmd();

    // 3. 读取 IMU 姿态
    drv_imu_read_acc_raw(&imu);
    drv_imu_read_gyro_raw(&imu);
    drv_imu_data_convert(&imu);
    drv_imu_mahony_update(&imu, 0.001f);
    drv_imu_quat_to_euler(&imu);

    // 4. PID 计算 → 电流输出
    float vx_target = dbus->rc.ch[2] / 660.0f * MAX_SPEED;
    float vy_target = dbus->rc.ch[3] / 660.0f * MAX_SPEED;
    float vz_target = dbus->rc.ch[0] / 660.0f * MAX_OMEGA;

    // 麦轮解算 (4轮独立)
    float currents[4];
    currents[0] = vx_target - vy_target - vz_target * WHEEL_BASE;
    currents[1] = vx_target + vy_target + vz_target * WHEEL_BASE;
    currents[2] = vx_target + vy_target - vz_target * WHEEL_BASE;
    currents[3] = vx_target - vy_target + vz_target * WHEEL_BASE;

    // 5. 功率限制
    int16_t power_limit = get_power_limit();
    scale_by_power(currents, 4, power_limit);

    // 6. 发送 CAN 电流帧
    uint8_t frame[8];
    drv_motor_build_dji_frame_init(frame);
    for (int i = 0; i < 4; i++) {
        drv_motor_build_dji_frame_set(frame, i, (int16_t)currents[i]);
    }
    bsp_can_send(&hcan1, 0x200, frame);

    // 7. 发送功率反馈
    app_chassis_comm_send_power_feedback(actual_power);
}
```

---

## 5. 新增 DRV 模块

### 5.1 文件结构

```
Self_core/Drv/Drv_xxx/
├── drv_xxx.h        ← Core 头文件 (结构体 + API 声明, 只依赖 LIB)
├── drv_xxx.c        ← Core 实现 (纯逻辑, 不依赖 HAL/BSP)
└── drv_xxx_port.c   ← Port 适配层 (依赖 BSP, 给 Core 注入硬件操作)
```

### 5.2 Core 模式: 函数指针解耦

```c
// drv_xxx.h
typedef void (*drv_xxx_write_fn_t)(uint8_t *data, uint16_t len);
typedef void (*drv_xxx_read_fn_t)(uint8_t *data, uint16_t len);

typedef struct {
    drv_xxx_write_fn_t write;
    drv_xxx_read_fn_t  read;
} drv_xxx_bus_t;

typedef struct {
    drv_xxx_bus_t bus;    // 函数指针表 ← Port 负责填充
    float data;
} drv_xxx_t;

// API
void drv_xxx_init(drv_xxx_t *dev, const drv_xxx_bus_t *bus);
float drv_xxx_get_data(drv_xxx_t *dev);
```

### 5.3 Port 实现: 绑定 BSP

```c
// drv_xxx_port.c
#include "drv_xxx.h"
#include "bsp_cfg.h"
#include "bsp_spi.h"

static void port_xxx_write(uint8_t *data, uint16_t len) {
    bsp_spi_transceive(&hspi1, data, NULL, len);
}

void drv_xxx_port_init(drv_xxx_t *dev) {
    drv_xxx_bus_t bus = {
        .write = port_xxx_write,
        .read  = NULL,
    };
    drv_xxx_init(dev, &bus);
}
```

### 5.4 检查清单

- [ ] Core 头文件只 `#include "lib_typedef.h"`
- [ ] Core 实现只包含 Core 头 + `<math.h>` / `<string.h>`
- [ ] Port 文件只包含 `"bsp_cfg.h"` + 所需 BSP 头文件
- [ ] 所有公开符号加 `drv_` 前缀
- [ ] 静态函数和宏加 `drv_模块名_` 前缀
- [ ] 在 `CMakeLists.txt` 添加 .c 文件, include path 无需改

---

## 6. CAN 通信协议

### 6.1 CAN 总线拓扑

```
                  CAN1 (1Mbps)
┌─────────┐    ┌──────────┐    ┌──────────┐
│ 小电脑   │    │ 底盘 C板  │    │ 云台 C板  │
│ (MiniPC) │    │ CHASSIS   │    │ GIMBAL    │
└────┬─────┘    └─────┬─────┘    └─────┬─────┘
     │                │                │
     ├── 0x111~0x115 →│  ← 底盘指令    │
     │← 0x112 ────────│  → 功率反馈    │
     │                │                │
     │                │  ← 0x120~0x129 │ ← 小电脑→云台
     │                │                │ → 0x122/124/130/233
     │                │                │
     │  ← 0x151~0x157 │ ← 裁判系统     │
     │                │                │
     └── 0x200+ ──────┴── 电机控制 ────┘

                  CAN2 (1Mbps)
              (预留, 当前未使用)
```

### 6.2 数据格式速查

#### 底盘接收 (CAN1)

| ID | 名称 | 格式 | 说明 |
|----|------|------|------|
| 0x111 | 速度指令 | int16×4 LE | vx, vy, vz, power_pct |
| 0x113 | 阿克曼指令 | float×2 LE | speed[-100,100], steer[-PI,PI] |
| 0x115 | 跟随指令 | int16×4 LE | vx, vy, gimbal_angle×1000, custom |

#### 底盘发送 (CAN1)

| ID | 名称 | 格式 | 说明 |
|----|------|------|------|
| 0x112 | 功率反馈 | int16 LE | 实际功率×100 (瓦特) |

#### 云台接收 (CAN1)

| ID | 名称 | 格式 | 说明 |
|----|------|------|------|
| 0x120 | 雷达速度 | int16×4 LE | 同0x111, 云台转发到0x111 |
| 0x121 | 速度不发射 | float×2 LE | yaw_inc, pitch_inc [rad] |
| 0x123 | 角度不发射 | float×2 LE | yaw_abs, pitch_abs [rad] |
| 0x125 | 速度发射 | float×2 LE | yaw_inc, pitch_inc [rad] |
| 0x127 | 角度发射 | float×2 LE | yaw_abs, pitch_abs [rad]; 全0xFFFF=检测到目标 |
| 0x129 | 控制指令 | uint8×4 (DLC=4) | shoot, retreat, -, - |

#### 云台发送 (CAN1)

| ID | 名称 | 格式 | 说明 |
|----|------|------|------|
| 0x122 | 速度反馈 | float×2 LE | yaw_speed, pitch_speed [rad/s] |
| 0x124 | 角度反馈 | float×2 LE | yaw, pitch [rad] |
| 0x130 | 角度反馈v2 | uint16×4 LE | yaw, pitch, roll [deg×65536/360], interval [1/65536s] |
| 0x126 | 射击反馈 | uint8×8 | 自定义内容 |
| 0x233 | IMU四元数 | int16×4 LE | q0, q1, q2, q3 (除30000后用) |

---

## 7. 调试与诊断

### 7.1 VOFA+ 实时数据波形

连接 USART1 (PA9 TX, 115200 8N1)。在 VOFA+ 中选 `FireWater` 协议。

```c
// 在控制循环中发送调试数据
float vofa_data[APP_INIT_VOFA_CH_COUNT];
vofa_data[0] = target_speed;
vofa_data[1] = actual_speed;
vofa_data[2] = pid_output;
vofa_data[3] = imu.euler.yaw;
// ... 最多 10 通道
drv_vofa_port_send(&huart1, (uint8_t *)vofa_data, sizeof(vofa_data));
```

VOFA+ 下载: https://www.vofa-plus.com/

### 7.2 LED 诊断信号

| 状态 | LED |
|------|-----|
| 全部设备在线 | 绿色常亮 |
| 有设备离线 | 红色闪烁 (250ms周期) |
| 程序启动 | LED 初始化全灭 |

### 7.3 裁判系统调试

裁判系统数据在底盘板上通过 USART6 接收。可用另一根 USB-TTL 监听:

```
底盘C板 PG14(TX) → USB-TTL RX → PC 串口助手 (115200 8N1)
```

解析函数: `app_referee_get_data()` 返回全部裁判数据。

---

## 8. PID 调参指南

### 8.1 可用 PID 类型

| 函数 | 适用场景 |
|------|---------|
| `lib_pid_calc()` | 基础 PID, 调试入门 |
| `lib_pid_ff_calc()` | 前馈 PID, 云台重力补偿 |
| `lib_pid_pos_calc()` | 位置环 PID, 云台角度控制 |
| `lib_pid_deriv_first_calc()` | 微分先行, 目标突变不抖动 |
| `lib_pid_fuzzy_calc()` | 模糊 PID, 非线性负载 |

### 8.2 调参顺序

```
1. 先调 Kp (Ki=Kd=0), 调到轻微震荡
2. 加 Kd 抑制震荡 (通常 Kd = Kp/100 ~ Kp/10)
3. 最后加 Ki 消除稳态误差 (Ki = Kp/10 ~ Kp/2)
4. 如有前馈需求, 最后调 Kff
```

### 8.3 代码示例

```c
lib_pid_t pid_speed;

// 初始化: Kp, Ki, Kd, Kff_g, Kff_y, max_ff_g, max_ff_y, min_out, max_out, max_iout
lib_pid_init(&pid_speed, 10.0f, 0.5f, 0.1f, 0, 0, 0, 0, -16000, 16000, 5000);

// 每 1ms 计算
float current = lib_pid_calc(&pid_speed, target_rpm, actual_rpm);
drv_motor_build_dji_frame_set(frame, slot, (int16_t)current);
```

---

## 9. 常见问题

### Q: 编译报错 `expected ';' before 'typedef'`

**A:** 文件开头有 UTF-8 BOM。编辑器设置编码为 "UTF-8 without BOM"，或运行:
```bash
sed -i '1s/^\xEF\xBB\xBF//' <filename>
```

### Q: 如何切换底盘/云台固件

**A:** 修改 `Self_core/project_cfg.h` 中的 `CURRENT_BOARD` 宏，重新编译即可。同一份代码编译出两个固件。

### Q: CAN 通信不上

**A:** 检查步骤:
1. CAN 总线终端电阻 (120Ω) 是否焊接
2. `bsp_can_start()` 是否在 `app_init()` 中调用
3. CAN 波特率是否匹配 (当前 1Mbps: APB1=42MHz / Prescaler=3 / BS1=10 / BS2=3)
4. CAN ID 是否匹配 (`app_chassis_comm.h` / `app_gimbal_comm.h` 中定义)

### Q: DBUS 遥控器没数据

**A:** 检查:
1. DBUS 接收器接线: PC11(RX)
2. USART3 配置: 100000 baud, 9-bit, Even parity
3. DMA1_Stream1 Channel 4 是否在 CubeMX 中启用
4. `drv_dbus_port_irq_handler()` 是否在 `stm32f4xx_it.c` 的 USART3_IRQHandler 中被调用

### Q: 如何添加新的 CAN 消息

**A:** 在对应的 `app_*_comm.c` 中:
1. 定义结构体 (在 .h)
2. 写 `static void on_xxx(uint32_t std_id, uint8_t *data, uint8_t len)` 回调
3. 在 `init()` 中 `bsp_can_register_rx_callback(&hcan1, ID, on_xxx)`
4. 如需发送: 写 `void app_xxx_comm_send_xxx(...)` 函数, 内部调 `bsp_can_send()`