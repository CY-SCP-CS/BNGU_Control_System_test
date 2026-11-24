# 重装底盘状态机框架

当前默认配置为 `BOARD_CHASSIS + ROBOT_HERO`，系统调度运行的是临时调参入口。正式入口已能更新 IMU，行为分发和四毫秒控制调度仍待接入。下面分别画出正式状态机和当前执行路径。

## 正式状态机

实线表示状态执行函数里已有的切换条件。虚线表示模式归属或待接入的切换，具体含义标在箭头上。

```mermaid
flowchart TB
    INIT["正式初始化"]
    NORMAL["NORMAL · 日常模式"]
    ACTION["ACTION · 额外动作<br/>VMC_SHOW / MIT_SHOW 已定义，执行逻辑预留"]
    INIT --> NORMAL
    NORMAL -. "行为切换待实现" .-> ACTION

    subgraph VMC["JOINT_MODE_VMC · 高度和姿态控制"]
        REST["REST · 休息<br/>发送零扭矩"]
        RISING["RISING · 起立<br/>跟踪正常高度"]
        STANDING["STANDING · 站立<br/>控制高度、俯仰和横滚"]
        LOWERING["LOWERING · 坐下<br/>跟踪休息高度"]

        REST -->|"有非零速度目标，1 s 高度请求受理"| RISING
        RISING -->|"高度误差 ≤ 5 mm，且轨迹已结束"| STANDING
        STANDING -->|"空闲计时 ≥ 15 s，1 s 高度请求受理"| LOWERING
        LOWERING -->|"高度误差 ≤ 5 mm，且轨迹已结束"| REST
    end

    subgraph MIT["JOINT_MODE_POSITION · MIT 关节位置控制"]
        FRONT_LIFT["FRONT_LIFT · 抬前腿，后腿同步回到 0 rad"]
        FRONT_FOLD["FRONT_REAR_FOLD · 收前后腿"]
        REAR_LIFT["REAR_LIFT · 前后腿到最后一步目标"]
        DONE["DONE · 等待切回 VMC"]

        FRONT_LIFT -.-> FRONT_FOLD
        FRONT_FOLD -.-> REAR_LIFT
        REAR_LIFT -.-> DONE
    end

    NORMAL -->|"关节默认进入 VMC 休息"| REST
    NORMAL -. "位置模式入口待接入" .-> FRONT_LIFT
    DONE -->|"MIT 轨迹结束，姿态和高度数值有效，1 s 高度请求受理"| RISING
```

- **VMC** 在起立和站立时使用 `target_height_mm`，坐下时使用 `rest_height_mm`。有速度目标会清零站立空闲计时，但累计计时的调度还没接入。正常高度目前是 `0 mm` 占位值，休息高度在正式初始化时尝试从关节反馈计算。
- **MIT** 的三个步骤共用一个执行函数。临时模式下第一次按 A 直接进入 `FRONT_LIFT`。当前用轨迹完成判断可否切步，没有用实测角度判定到位。
- **MIT → VMC** 在 `DONE` 中已有实现。等待期间继续运行 MIT 并保留现有目标。切换成功后进入 `RISING`，由 VMC 从当前高度平滑过渡到正常高度。
- **麦轮行走** 是独立的控制维度，已定义车体坐标系和世界坐标系。正式入口中的遥控输入映射、坐标系切换和麦轮调度仍待接入。

## 当前执行路径

```mermaid
flowchart TD
    INIT["底盘初始化<br/>CAN2、底盘 IMU、临时麦轮和关节控制"] --> TICK["TIM14 · 1 kHz"]
    TICK --> TEMP["app_armored_ctrl_1khz_temp"]
    TEMP --> AHRS["每 1 ms 检查 IMU DMA 快照<br/>新样本到达后更新四元数和欧拉角"]
    AHRS --> DIV{"检查 1 ms 错峰计数"}
    DIV -->|"其余节拍"| END["结束本周期"]
    DIV -->|"关节节拍"| JOINT["更新关节 P/D 与 MIT 位置控制<br/>A 键推进动作，最后一步结束后再按 A 回定角<br/>左右两侧交替发帧"]
    DIV -->|"关节节拍后 2 ms"| DRIVE["更新轮速 PID，默认由手柄控制麦轮"]
    DRIVE --> VOFA["VOFA CH0 · 功率计实测电池侧功率（W）"]
```

关节和麦轮控制各每 4 ms 执行一次。麦轮在关节之后 2 ms 发送，VOFA 紧随麦轮。关节左右两侧交替发送，所以每个关节每 8 ms 获得一次指令发送机会。IMU 更新独立于这一分频。正式入口 `app_armored_ctrl_1khz()` 和临时入口共用 AHRS 更新函数。

源码对应 [重装控制](chassis/app_Armored_ctrl.c)、[关节控制](chassis/app_Armored_joint.c)、[系统初始化](../common/system/app_init.c) 和 [周期调度](../common/system/app_control.c)。
