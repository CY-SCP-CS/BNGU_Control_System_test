# RoboMaster 电控编码规范

> v3 架构配套编码规范 — 统一所有层级命名风格，降低团队认知开销

---

## 1. 分层命名前缀

| 层级 | 前缀 | 示例 |
|------|------|------|
| **LIB** | `lib_` | `lib_pid_calc()` |
| **BSP** | `bsp_` | `bsp_can_send()` |
| **DRV** | `drv_` | `drv_motor_init()` |
| **APP** | `app_` | `app_chassis_ctrl()` |

**规则**：所有全局可见的函数/变量必须以层级前缀开头。

---

## 2. 函数命名

### 2.1 通用格式

```
<层级前缀>_<模块名>_<动作>
```

| 部分 | 规范 | 示例 |
|------|------|------|
| 层级前缀 | `lib_` `bsp_` `drv_` `app_` | 同上 |
| 模块名 | 小写下划线 | `motor` `can` `pid` |
| 动作 | 小写下划线动词 | `init` `calc` `send` `update` |

### 2.2 常用动作词

| 动作词 | 使用场景 | 示例 |
|--------|---------|------|
| `init` | 初始化模块 | `bsp_can_init()` |
| `calc` | 计算/运算 | `lib_pid_calc()` |
| `send` | 发送数据 | `bsp_can_send()` |
| `read` | 读取数据（主动） | `bsp_adc_read()` |
| `update` | 更新状态（周期调用） | `drv_imu_update()` |
| `process` | 处理数据流 | `drv_referee_process()` |
| `register` | 注册回调 | `bsp_can_register_rx_callback()` |
| `start` / `stop` | 启动/停止外设 | `bsp_tim_start()` |
| `set` / `get` | 设置/获取参数 | `app_chassis_set_power_limit()` |
| `toggle` | 切换状态 | `bsp_gpio_toggle()` |
| `on_` | 事件响应 | `drv_motor_on_can_rx()` |

### 2.3 不同机器人的相同功能用同一函数名

不同机器人的 chassis.c 中，底盘控制的函数名必须一致：

```c
// ✅ 正确：所有机器人的 chassis.c 都实现这两个函数
void app_chassis_init(void);
void app_chassis_ctrl(void);

// ❌ 错误：函数名跟机器人绑定
void hero_chassis_init(void);
void infantry_chassis_ctrl(void);
```

---

## 3. 变量命名

### 3.1 全局变量

```
g_<模块>_<名称>
```

```c
// ✅ 正确
extern drv_referee_data_t g_referee;
extern drv_imu_data_t     g_imu;
extern monitor_data_t     g_monitor;

// ❌ 错误
extern drv_referee_data_t referee_data;  // 缺少 g_ 前缀
extern drv_imu_data_t     imu;           // 缺少 g_ 前缀
```

`g_` 表示全局可见（`extern`），仅 DRV 层的核心数据实例使用，**APP 层尽量避免全局变量**。

### 3.2 文件内全局变量（static）

```
s_<模块>_<名称>
```

```c
// ✅ 正确
static drv_motor_t s_motors[4];
static lib_pid_t   s_pid[4];
static uint32_t    s_tick_count;
```

规则：所有 `static` 文件作用域变量以 `s_` 开头。

### 3.3 局部变量

```
小写下划线
```

```c
// ✅ 正确
float rpm_target[4];
int16_t currents[4];
uint8_t power_limit;
uint32_t timeout_count;

// ❌ 错误
float rpmTarget[4];    // 驼峰
int16_t a, b, c, d;   // 无意义单字母（循环变量 i/j/k 除外）
int16_t m1, m2;        // 魔数命名
```

**例外**：循环计数器可以用 `i` `j` `k`。

### 3.4 结构体成员

```
小写下划线
```

```c
typedef struct {
    float kp, ki, kd;        // ✅ 小写下划线
    float max_out;            // ✅
    float integral;           // ✅
    uint16_t remain_hp;       // ✅
    uint16_t remain_bullet;   // ✅
} drv_referee_data_t;
```

---

## 4. 类型命名

### 4.1 格式

```
<层级前缀>_<模块>_<描述>_t
```

```c
// ✅ 正确
typedef struct {
    float kp, ki, kd;
    float max_out, max_iout;
    float integral, last_err;
    float out;
} lib_pid_t;                     // lib + pid + _t

typedef struct {
    drv_motor_type_t type;
    bsp_can_ch_t     can_ch;
    uint16_t         can_id;
    int16_t          rpm;
} drv_motor_t;                   // drv + motor + _t

typedef enum {
    BSP_CAN_1 = 0,
    BSP_CAN_2,
    BSP_CAN_NUM
} bsp_can_ch_t;                  // bsp + can_ch + _t

// ❌ 错误
typedef struct {
    ...
} PID_t;                         // 缺少层级前缀，风格不对
```

### 4.2 枚举类型

枚举值用全大写下划线，枚举名用层级前缀 + 描述 + `_t`：

```c
typedef enum {
    MOTOR_TYPE_M3508  = 0,
    MOTOR_TYPE_M2006  = 1,
    MOTOR_TYPE_GM6020 = 2,
} drv_motor_type_t;

typedef enum {
    BSP_UART_1 = 0,
    BSP_UART_2,
    BSP_UART_3,
    BSP_UART_NUM
} bsp_uart_ch_t;
```

---

## 5. 宏定义

### 5.1 格式

```
全大写下划线
```

```c
// ✅ 正确
#define CURRENT_BOARD       BOARD_CHASSIS
#define HAS_CHASSIS         (CURRENT_BOARD == BOARD_CHASSIS)
#define CANID_MOTOR_1       0x201
#define WHEEL_BASE          0.40f
#define MAX_SPEED           3.5f
#define PID_KP              18.0f

// ❌ 错误
#define current_board       BOARD_CHASSIS    // 小写
#define WheelBase           0.40f            // 驼峰
```

### 5.2 条件编译宏用全大写下划线

```c
#if CURRENT_BOARD == BOARD_CHASSIS
    app_chassis_init();
#endif
```

---

## 6. 文件结构规范

### 6.1 文件头模板

```c
/**
 * @file    app_chassis.c
 * @brief   底盘运动控制
 * @note    英雄车底盘，四轮 M3508 + 全向轮
 */
#include "project_cfg.h"

#include "app_chassis.h"
#include "drv_motor.h"
#include "drv_imu.h"
#include "drv_can_protocol.h"
#include "lib_pid.h"
#include "lib_math.h"

// ─── 私有宏 ─────────────────────────
#define WHEEL_BASE      0.40f
#define MAX_SPEED       3.5f

// ─── 私有类型 ───────────────────────
typedef struct {
    float vx_target;
    float vy_target;
    float wz_target;
} chassis_cmd_t;

// ─── 私有变量 ───────────────────────
static drv_motor_t s_motors[4];
static lib_pid_t   s_pid[4];

// ─── 私有函数声明 ───────────────────
static void kinematics(float vx, float vy, float wz, float *rpm);

// ─── 接口实现 ───────────────────────
void app_chassis_init(void) { ... }
void app_chassis_ctrl(void) { ... }

// ─── 私有函数定义 ───────────────────
static void kinematics(float vx, float vy, float wz, float *rpm) { ... }
```

### 6.2 文件内段落顺序

```
1. @file 注释块
2. #include
3. 私有宏定义
4. 私有类型定义（typedef / struct / enum）
5. 私有变量（static）
6. extern 引用（若有）
7. 私有函数声明（static）
8. 公有接口实现
9. 私有函数实现
```

### 6.3 分段分隔

段落之间用 `// ─── 段落名 ─────────────────────────` 分隔，行长度控制在 120 列以内：

```c
// ─── PID 参数 ──────────────────────────
#define YAW_KP      18.0f
#define YAW_KI      0.8f

// ─── 私有变量 ──────────────────────────
static lib_pid_t s_pid_yaw;
```

---

## 7. Include 顺序

按层级从低到高排列，空行分隔层级：

```c
// ① 同层头文件（自己的头文件放最前）
#include "app_chassis.h"

// ② 下层头文件（LIB → BSP → DRV → APP）
#include "lib_pid.h"
#include "lib_math.h"
#include "bsp_can.h"
#include "drv_motor.h"
#include "drv_imu.h"

// ③ 系统/CubeMX 头文件（外部库）
#include "stm32f4xx_hal.h"
#include <string.h>
```

---

## 8. 注释规范

### 8.1 函数注释

公有接口函数必须有注释：

```c
/**
 * @brief   计算 PID 输出
 * @param   pid   PID 控制器实例指针
 * @param   ref   目标值
 * @param   fdb   反馈值
 * @return  PID 计算输出
 */
float lib_pid_calc(lib_pid_t *pid, float ref, float fdb);
```

### 8.2 关键逻辑注释

```c
// 运动解算：vx, vy, wz → 四轮 RPM（全向轮）
kinematics(cmd.target_vx, 0.0f, cmd.target_wz, rpm_target);

// PID 闭环
for (int i = 0; i < 4; i++) {
    float out = lib_pid_calc(&s_pid[i], rpm_target[i], s_motors[i].rpm);
}
```

### 8.3 不写废话注释

```c
// ❌ 废注释：代码本身就说明了一切
int i = 0;     // 把 i 设为 0

// ✅ 好注释：解释了"为什么"这么做
// 全向轮运动学要求左前轮和右后轮转速方向相反
rpm[0] = ( vx - vy - wz * WHEEL_BASE) / WHEEL_RADIUS;
```

---

## 9. 头文件规范

### 9.1 防止重复包含

统一用 `#ifndef` 方案（STM32CubeMX 默认风格，与 HAL 保持一致）：

```c
#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

// ... 内容 ...

#endif
```

### 9.2 头文件只放声明，不放定义

```c
// ✅ 正确：.h 放声明
void app_chassis_init(void);
void app_chassis_ctrl(void);
extern drv_motor_t g_motors[];      // 如有必要

// ❌ 错误：.h 不要放变量定义
drv_motor_t s_motors[4];            // 应该在 .c 中用 static
```

---

## 10. 格式化规则

### 10.1 大括号

```c
// 函数定义：大括号另起一行
void app_chassis_init(void)
{
    // ...
}

// 控制语句：大括号不另起一行
for (int i = 0; i < 4; i++) {
    drv_motor_init(&s_motors[i], MOTOR_TYPE_M3508, BSP_CAN_1, 0x201 + i);
}

if (voltage < 22000) {
    bsp_gpio_set(BSP_GPIO_BUZZER, 1);
} else {
    bsp_gpio_set(BSP_GPIO_BUZZER, 0);
}
```

### 10.2 缩进

4 空格缩进，不使用 Tab。

```c
void app_chassis_ctrl(void) {
····can_msg_gimbal2chassis_t cmd;        // 4 空格
····can_proto_get_gimbal_cmd(&cmd);
····
····drv_imu_update();
····
····float rpm_target[4];
····kinematics(cmd.target_vx, 0.0f, cmd.target_wz, rpm_target);
}
```

### 10.3 行宽

每行不超过 120 个字符。

---

## 11. 布尔/状态命名惯例

| 类型 | 命名习惯 | 示例 |
|------|---------|------|
| 布尔变量 | 用 `is_` `has_` 开头 | `is_online`, `has_error`, `is_ready` |
| 状态枚举 | 枚举值以模块前缀开头 | `GAME_STAGE_INIT`, `MOTOR_STATE_STOP` |
| 错误码/标志位 | 位运算建议用 `#define` 宏 | `#define ERR_CAN_OFFLINE (1 << 0)` |

---

## 12. 快速参考卡

```
┌────────────────────────────────────────────────────────────┐
│                    编码规范速查                             │
├────────────────────────────────────────────────────────────┤
│ 全局函数          lib_pid_calc()       层级_模块_动作      │
│ 全局变量          g_referee            g_模块_名称          │
│ 文件内静态变量     s_motors             s_模块_名称          │
│ 局部变量          rpm_target           小写下划线           │
│ 类型名             lib_pid_t            层级_描述_t          │
│ 枚举值             BSP_CAN_1           全大写下划线          │
│ 宏                 WHEEL_BASE          全大写下划线          │
│ 头文件守卫         APP_CHASSIS_H       {模块名}_H           │
└────────────────────────────────────────────────────────────┘
```
