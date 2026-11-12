# 哨兵底盘速度斜坡（sen 版本）

在哨兵底盘控制链路中接入速度斜坡。本次同时调整两个转向角度环的 PID 参数：Kp 从 10000 改为 20000，Ki 从 10 改为 20，积分限幅从 0 改为 1000；默认板型从 BOARD_GIMBAL 切换为 BOARD_CHASSIS。转向算法、动力学分配和遥控映射保持原有实现。

## 参数

在 `Self_core/App/sentry/app_sentry_common.h` 中修改：

```c
#define SENTRY_CHASSIS_RAMP_ENABLE 1
#define SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2 3000.0f
#define SENTRY_CHASSIS_ANGULAR_ACCEL_RAD_S2 9.0f
```

默认启用；当前平移 3000 mm/s²（3 m/s²），旋转 9 rad/s²。两项加速度须大于零，减小更柔和，增大更跟手。数值仍需实车整定。

## 行为

CAN1 指令限幅后，在逆运动学和原控制环之前，每 1 ms 更新目标。平移按二维速度增量的长度统一缩放，避免斜向变化率增大到 sqrt(2) 倍；旋转单独限制角速度变化量。

从静止到 1000 mm/s 的直线目标约需 0.33 秒，到 3000 mm/s 约需 1 秒。普通松杆减速、反向也经过斜坡；松杆不等于瞬间停止。急停分支直接清零输出并复位斜坡，解除后从零起步。

Watch 变量：

```text
app_sentry_chassis_debug.motion.cmd_speed  // 原始限幅指令
app_sentry_chassis_debug.motion.tar_speed  // 斜坡后的目标
```

x/y 单位 mm/s，w 单位 rad/s。斜坡不消除遥控中位偏置，也不提供航向锁定。这是车体坐标系的目标变化率限制，不是实际加速度反馈或 S 曲线，不能保证任何工况下都不打滑。

## 验证

`tests/chassis/run_tests.ps1` 使用本机 GCC 编译执行真实底盘 C 代码，CAN 和中断通过主机桩替代。覆盖斜坡开/关、直线/斜向加速、减速、换向、到达目标、5000 次变向的变化率上限、真实控制入口接入、急停与解除后起步、初始化复位。未进行实车运行和烧录。
