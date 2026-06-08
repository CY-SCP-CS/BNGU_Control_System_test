# RoboMaster 电控框架架构设计 v3

> 基于 STM32F407 (C板) · APP/BSP/DRV/LIB 四层架构
> 每车双主控：底盘(C板) + 云台(C板) · CAN2 互联

---

## 1. 架构总览

### 1.1 四层依赖规则

```
┌──────────────────────────────────────────────────────┐
│                      APP 层                           │  ← 可以依赖 BSP、DRV、LIB
│            (应用逻辑：底盘控制/云台控制/发射控制)        │
├──────────┬──────────┬───────────────────────────────┤
│ BSP 层   │ DRV 层   │    LIB 层                      │
│ (硬件抽象) │ (器件驱动) │    (算法基础库)                  │
│ CAN/UART/ │ 电机/裁判 │    PID/滤波器/数学/CRC          │
│ TIM/SPI   │ 系统/IMU  │                               │
├──────────┴──────────┴───────────────────────────────┤
│                    HAL / CMSIS                        │
│              (STM32F4xx HAL + 寄存器层)                │
└──────────────────────────────────────────────────────┘
```

**依赖约束（核心）**：

| 方向 | 规则 |
|------|------|
| **APP → DRV / LIB** | ✅ 允许 — APP 是编排者 |
| **APP → BSP** | ❌ 禁止 — App 不应直接操作硬件，通过 DRV 间接访问 |
| **DRV → BSP** | ✅ 允许 — 驱动负责硬件挂接和适配 |
| **BSP → LIB** | ✅ 允许 — BSP 可使用基础类型和工具 |
| **LIB → 任何** | ❌ 禁止 — LIB 必须纯软件 |
| **DRV → LIB** | ✅ 允许 — 驱动可使用数学/滤波工具 |
| **任何层 → APP** | ❌ 禁止 — 严禁反向依赖 |


## 2. 目录结构

```
Self_core/
│
├── project_cfg.h                  # [入口] 唯一配置：选择底盘/云台 + 车组
│
├── Lib/                           # ──── LIB 层：纯软件，无条件编译 ────
│   ├── Lib_Math/lib_math.h/.c     # 限幅/角度归一化/插值/编码器转换/角度弧度转换
│   ├── Lib_Filter/lib_filter.h/.c # 一阶低通/滑动窗口滤波
│   ├── Lib_Pid/lib_pid.h/.c       # PID 控制器(标准/前馈/位置/微分先行/二自由度/模糊)
│   └── Lib_Typedef/lib_typedef.h  # 通用类型（不含 MCU 头文件）
│
├── Bsp/                           # ──── BSP 层：硬件外设抽象，无条件编译 ────
│   ├── bsp_can.h/.c               # CAN 初始化/滤波器/非阻塞发送/回调订阅
│   ├── bsp_uart.h/.c              # UART DMA 收发
│   ├── bsp_tim.h/.c               # 定时器 PWM/编码器/定时中断
│   ├── bsp_spi.h/.c               # SPI 收发
│   ├── bsp_i2c.h/.c               # I2C 收发
│   ├── bsp_adc.h/.c               # ADC 采样
│   ├── bsp_gpio.h/.c              # GPIO 控制
│   ├── bsp_flash.h/.c             # Flash 读写
│   └── bsp_cfg.h                  # 外设句柄映射 (extern hcan1, huart1...)
│
├── Drv/                           # ──── DRV 层：器件驱动，无条件编译 ────
│   ├── Drv_motor/drv_motor.h/.c   # 电机协议层 (DJI/翎控/MF7010)
│   ├── Drv_imu/drv_imu.h/.c       # BMI088 驱动
│   ├── Drv_power_measure/drv_power_measure.h/.c
│   ├── Drv_led/drv_led.h/.c       # RGB LED (PH10/11/12)
│   ├── Drv_dbus/drv_dbus.h/.c     # DBUS 遥控器
│   └── Drv_buzzer/drv_buzzer.h/.c # 蜂鸣器 (PD14)
│
└── App/                           # ──── APP 层：条件编译区分板型 + 车组 ────
    └── common/                    # ← 全车共用，内含 #if 区分
        ├── app_init.c             #   统一初始化入口
        ├── app_control.c          #   统一调度框架
        ├── app_monitor.c          #   系统监控
        └── app_referee.c          #   裁判系统响应

```

---

## 3. 条件编译策略

### 3.1 核心原则

```
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  LIB / BSP / DRV →  零条件编译                              │
│                     所有文件全编译，不关心板型和车组            │
│                                                             │
│  App/common/*     →  使用 #if CURRENT_BOARD / #if CURRENT_   │
│                      ROBOT 区分板型和车组                     │
│                                                             │
│  条件编译范围：                                             │
│    - CURRENT_BOARD  →  0=底盘, 1=云台                        │
│    - CURRENT_ROBOT  →  1=英雄, 3=步兵, 7=哨兵                │
│                                                             │
│  预处理器条件编译 = 唯一的差异化手段，不用 CMake 管理车型      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 project_cfg.h — 唯一配置头文件

**每个工程目录下仅一个 `project_cfg.h`，作为唯一配置入口**：

```c
// project_cfg.h
#ifndef PROJECT_CFG_H
#define PROJECT_CFG_H

// ========== [必改] 选择主板 ==========
#define BOARD_CHASSIS       0
#define BOARD_GIMBAL        1

#define CURRENT_BOARD       BOARD_CHASSIS      // 可选: BOARD_CHASSIS / BOARD_GIMBAL

// ========== [必改] 选择车组 ==========
#define ROBOT_HERO          1
#define ROBOT_INFANTRY      3
#define ROBOT_SENTRY        7

#define CURRENT_ROBOT       ROBOT_HERO         // 可选: ROBOT_HERO / ROBOT_INFANTRY / ROBOT_SENTRY

#endif
```

> **注意**：编译前必须检查 `CURRENT_BOARD` 和 `CURRENT_ROBOT` 是否正确。不同车组共用同一份代码，差异由预处理指令处理。

### 3.3 条件编译在项目中的完整分布

| 文件 | 条件 | 说明 |
|------|------|------|
| `project_cfg.h` | 无 | 仅定义 `CURRENT_BOARD` + `CURRENT_ROBOT` |
| `Lib/*` | 无 | 纯算法，与硬件/车型/板型无关 |
| `Bsp/*` | 无 | 同一颗 F407，所有外设全编译 |
| `Drv/*` | 无 | 只提供接口，不关心谁调用 |
| `app/common/app_init.c` | `CURRENT_BOARD` + `CURRENT_ROBOT` | 初始化对应板型和车组的模块 |
| `app/common/app_control.c` | `CURRENT_BOARD` + `CURRENT_ROBOT` | 运行对应的控制逻辑 |
| `app/common/app_monitor.c` | 无 | 每台车都需要 |
| `app/common/app_referee.c` | 无 | 每台车都需要 |

---

## 4. APP 层文件详细内容

### 4.1 `app/common/app_init.c` — 统一初始化入口

**原理**：所有 target 共用此文件，通过 `project_cfg.h` 中的宏决定初始化哪些模块。

```c
/**
 * @file    app_init.c
 * @brief   统一初始化入口 — 所有机器人共用
 * @note    #if 用于区分板型 (底盘/云台) 和车组 (英雄/步兵/哨兵)
 */
#include "project_cfg.h"            // ← 必须第一个 include

#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_tim.h"

#include "app_chassis.h"
#include "app_gimbal.h"
#include "app_shoot.h"
#include "app_monitor.h"
#include "app_referee.h"

// ─── 统一初始化入口 ─────────────────────
void app_init(void)
{
    // ── ① BSP 层基础初始化（无条件执行） ──
    bsp_can_start(&hcan1, 0);
    bsp_can_start(&hcan2, 14);
    // ...

    // ── ② 通用模块初始化（无条件执行） ──
    app_monitor_init();
    app_referee_init();

    // ── ③ 按板分支 ─────────────────────
    #if CURRENT_BOARD == BOARD_CHASSIS
        app_chassis_init();
    #else
        app_gimbal_init();
        app_shoot_init();
    #endif

    // ── ④ 按车组分支 ───────────────────
    #if CURRENT_ROBOT == ROBOT_HERO
        // 英雄特殊初始化
    #elif CURRENT_ROBOT == ROBOT_INFANTRY
        // 步兵特殊初始化
    #elif CURRENT_ROBOT == ROBOT_SENTRY
        // 哨兵特殊初始化
    #endif
}
```

### 4.2 `app/common/app_control.c` — 统一调度框架

```c
/**
 * @file    app_control.c
 * @brief   1kHz 控制循环 — 所有机器人共用
 */
#include "project_cfg.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "app_shoot.h"

void app_control_1khz(void)
{
    // ── 底盘/云台共用部分 ──
    app_referee_update();
    app_monitor_update();

    // ── 按板分支 ─────────────────────
    #if CURRENT_BOARD == BOARD_CHASSIS
        app_chassis_control();
    #else
        app_gimbal_control();
        app_shoot_control();
    #endif

    // ── 按车组分支（若有特殊处理） ────
    #if CURRENT_ROBOT == ROBOT_HERO
        // 英雄特殊逻辑
    #elif CURRENT_ROBOT == ROBOT_SENTRY
        // 哨兵特殊逻辑（如自动瞄准）
    #endif
}
```

---

## 5. 配置与使用的流程

```
1. 复制模板工程到一个新目录
2. 修改 project_cfg.h：
   #define CURRENT_BOARD   BOARD_GIMBAL    // 选板
   #define CURRENT_ROBOT   ROBOT_INFANTRY  // 选车
3. 编译 → 自动得到对应的固件
4. 一个工程 = 一个固件；不同车 = 不同 project_cfg.h 配置
```

---

## 6. 总结

v3 架构的核心思想：

```
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  「预处理器条件编译是唯一的差异化手段」                     │
│                                                         │
│  project_cfg.h 中的两个宏控制一切：                       │
│    - CURRENT_BOARD : 0=底盘, 1=云台                      │
│    - CURRENT_ROBOT : 1=英雄, 3=步兵, 7=哨兵              │
│                                                         │
│  LIB / BSP / DRV → 零条件编译，全都编译                   │
│  App/common/*    → 用 #if 按需初始化和控制                │
│                                                         │
│  结果是：                                                │
│  - 一套代码，任意配置                                     │
│  - 改 project_cfg.h 即可切换目标                          │
│  - 不需要 CMake 管理车型，不需要多重构建配置                │
│                                                         │
└─────────────────────────────────────────────────────────┘
```
