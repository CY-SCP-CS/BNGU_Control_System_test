# BNGU Control System 使用文档

本文面向首次编译、配置和运行本项目的开发者。代码接口和 CAN 数据结构见 [API 参考](API_Reference.md)。

## 1. 项目组成

固件运行在 STM32F407 上。`Core` 和 `Drivers` 提供 CubeMX/HAL 基础；`Self_core/Bsp` 封装板级外设，`Self_core/Drv` 实现传感器、电机和协议驱动，`Self_core/App` 运行底盘、云台和系统控制，`Self_core/Lib` 提供通用算法。

仓库当前默认配置为哨兵云台板：

```c
#define CURRENT_BOARD BOARD_GIMBAL
#define CURRENT_ROBOT ROBOT_SENTRY
```

在 `Self_core/project_cfg.h` 中选择 `BOARD_CHASSIS` 或 `BOARD_GIMBAL`，以及 `ROBOT_HERO`、`ROBOT_INFANTRY` 或 `ROBOT_SENTRY`。更换板型或车组后重新编译并烧录对应控制板。当前已接入的机器人应用是哨兵底盘和哨兵云台；英雄、步兵分支仍为预留位置。

## 2. 编译

### 工具

- CMake 3.22 或更新版本。
- Ninja。
- GNU Arm Embedded 工具链，命令行可找到 `arm-none-eabi-gcc`。
- Windows 可使用已安装的 STM32CubeCLT；也可使用工程附带的 MDK-ARM 工程。

### CMake 构建

从仓库根目录运行：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel 4
```

Release 配置：

```powershell
cmake --preset Release
cmake --build --preset Release --parallel 4
```

生成目录分别为 `build/Debug` 和 `build/Release`，固件文件为 `BNGU_Control_System.elf`。若 CMake 找不到 ARM 编译器，先将工具链的 `bin` 目录加入 `PATH`，再清除对应构建目录中的 CMake 缓存并重新配置。不要复用为另一个板型配置过的旧构建目录。

### CubeMX 重新生成代码

打开 `BNGU_Control_System.ioc` 后，保留 `Keep User Code when re-generating`，并确认栈大小为 `0x1000`（4 KiB）。栈大小应在 CubeMX 配置中设置，以便生成后的 Keil 启动文件和 CMake 链接脚本保持一致。

Keil 工程中，CubeMX 生成的源文件统一放在 `Application/User/Core`、`Drivers/STM32F4xx_HAL_Driver` 和 `Drivers/CMSIS` 分组；`05_Core` 只保留头文件，`06_Startup` 保留启动汇编。`Self_core` 的应用和驱动保留在原有自定义分组中。不要把同一个 Core 或 HAL 源文件再次添加到其他分组，否则会产生大量 `L6200E: Symbol ... multiply defined` 链接错误。

当前 `.ioc` 选择 `MDK-ARM V5.32`；需要更新 CMake 工程时，在 Project Manager 中切换到 CMake 后生成。生成完成后在 Keil 中重新加载工程并执行 `Rebuild all target files`，确保使用新的源文件列表。

### 烧录

可在 MDK-ARM 工程中构建并通过已配置的调试器下载，也可将对应 `.elf` 交给本机 STM32 烧录工具。请确认下载目标是正在调试的板型，并使用该板型刚生成的固件。

## 3. 上电与启动

启动顺序由 `Core/Src/main.c` 管理：HAL 和时钟初始化，初始化 CubeMX 外设，调用 `app_init()` 注册驱动及控制回调，再启动 TIM14 更新中断。主循环持续调用 `app_control_process()`。

TIM14 的控制周期为 1 ms。哨兵云台每周期先更新 IMU AHRS，再运行 yaw/pitch 控制；每 5 个周期运行一次 CAN 命令处理、底盘速度转发和发射机构控制（200 Hz）。哨兵底盘的控制入口为 1 kHz。裁判系统接收数据在底盘板的主循环处理。

云台板上电时会校准陀螺仪零偏并建立初始姿态。校准期间保持云台和 IMU 静止；上电后若姿态跳动或零点不稳定，先检查安装牢固度和校准时的振动。

### IMU 姿态算法

在 `Self_core/project_cfg.h` 中选择姿态算法后重新编译。当前云台默认使用 VQF，另外支持 EKF 和 Mahony：

```c
#define IMU_AHRS_ALGORITHM DRV_IMU_ALGO_EKF
/* 使用 VQF 时改为 DRV_IMU_ALGO_VQF。 */
```

云台已经通过统一入口 `drv_imu_update()` 调用所选算法。DMA 未提供新快照时会累计经过的控制周期，在下次有效快照到来时一起积分；最长积分间隔限制为 100 ms。其他应用可按以下顺序使用；`dt` 必须为两次有效采样的实际间隔（秒），不应重复融合同一组采样：

```c
drv_imu_init(&imu, &bus);
drv_imu_start(&imu);
drv_imu_calibrate_gyro(&imu, 1000); /* 保持静止，阻塞标定 */
drv_imu_read_acc_raw(&imu);
drv_imu_data_convert(&imu);
drv_imu_initial_alignment(&imu);
drv_imu_set_algorithm(&imu, DRV_IMU_ALGO_VQF);

/* 每次收到一组新采样后： */
drv_imu_data_convert(&imu);
drv_imu_update(&imu, 0.001f);      /* 例如 1 kHz 采样 */
drv_imu_quat_to_euler(&imu);
```

EKF 使用姿态误差和陀螺零偏共六维状态，异常加速度会被模长/创新门限拒绝。VQF 使用作者算法的六轴版本，包含运动与静止零偏估计；默认加速度时间常数为 3 秒，减小 `tau_acc` 会加快倾角响应，也会降低对运动加速度的抑制。所有算法的加速度输入为 g，陀螺输入为 rad/s，输出欧拉角为度。BMI088 没有磁力计，无法确定绝对航向或完全消除 yaw 长期漂移。

可在暂停更新后配置参数。配置接口返回 `1` 表示成功；无效配置返回 `0`，保留已有设置：

```c
drv_imu_vqf_config_t config;
drv_imu_vqf_default_config(&config);
config.tau_acc = 1.0f;
drv_imu_vqf_configure(&imu, &config);
/* EKF 对应 drv_imu_ekf_config_t / drv_imu_ekf_configure()。 */
```

配置当前算法会保留姿态并重置其历史；切换算法也会保留姿态。独立实例各自保存状态，无堆内存分配。CMake 和 MDK-ARM 已加入两个算法源文件；两种构建方式的栈预留均增至 4 KiB，以容纳协方差计算与中断调用。主机模拟测试执行 `python tests/imu/run_tests.py`，参考数值对照的运行方式见 `tests/imu/README.md`；实际板上仍需验证安装方向、噪声参数和 1 ms 周期内的执行耗时。

### IMU 竖装的水平零位

当前板子竖直安装。根据板端原始四元数，传感器相对等效水平机体系的固定旋转约为绕 Y 轴 `-90°`。哨兵云台在初始化时应用 `Self_core/project_cfg.h` 中的固定安装角：

```c
#define IMU_MOUNT_ROLL_DEG    0.0f
#define IMU_MOUNT_PITCH_DEG   -90.0f
#define IMU_MOUNT_YAW_DEG     0.0f
```

这些角度定义传感器系到等效水平机体系的 ZYX 旋转。驱动先用四元数消除固定安装旋转，再计算欧拉角；车体水平时查看 `app_gimbal_imu.euler.roll` 和 `pitch` 应接近 `0°`。其他安装方向应根据静止时原始四元数与重力轴确定对应旋转，不要只根据某一种欧拉角读数猜安装轴。配置后重新编译烧录。

Mahony/EKF/VQF 的内部 `imu->quat` 保留传感器姿态，不能将其直接当作补偿后的输出；需要四元数时调用 `drv_imu_get_body_quat()`。独立驱动实例默认无补偿，可调用 `drv_imu_set_mounting(&imu, 0.0f, -90.0f, 0.0f)` 配置。安装角是固定机械参数，不应在每次上电时将当前姿态自动记为水平，否则斜坡上的真实倾角也会被消除。

### 上位机 roll 偏置与 yaw 漂移的板端排查

先烧录本次构建的固件，在 Watch 查看 `app_gimbal_quat_tx_debug.build_time` 确认版本，再看 `app_sentry_gimbal_debug.imu.body_quat`、`tx_quat` 和 `app_gimbal_quat_tx_debug.payload`。这三处分别对应安装补偿后的 IMU 姿态、含关节补偿的云台姿态和实际提交发送的字节，不需要另一台上位机提供源码。具体变量、协议解码和 pitch 接近 ±90° 时的欧拉角退化说明见 [CAN 协议](CAN_Protocol.md#云台姿态的统一基准)。

当前绕 Y 轴 -90° 安装时，航向的静止残余零偏主要落在传感器 x 轴。使用 VQF 时同时查看 `filter.vqf.rest_detected`、`filter.vqf.bias[0]` 和 `app_sentry_gimbal_debug.imu.body_gyro.z`。前者判断是否持续静止，后两者分别是在线估计的残余零偏和补偿后的机体角速度。刚上电温度变化时二者仍可能在收敛；持续漂移需结合静止判定、实际采样周期和温度检查，六轴数据仍不提供绝对航向参考。`0x122` 的 yaw 角速度现已应用静态及在线零偏补偿，四元数滤波仍在传感器坐标中独立执行原有补偿。

VQF 低通初始化已改为显式就绪标记，避免依赖 Keil 默认浮点模式不保证支持的 NaN 哨兵；默认参数保持不变。主机测试除原有运动/零偏验证外，还覆盖竖装下静止 180 秒、CAN 打包、收敛后 yaw 变化和角速度输出；这些合成数据测试不能替代实机安装、温漂与接收端验证。

## 4. 控制板、外设与连线

以下列出当前 `BNGU_Control_System.ioc` 和固件初始化中配置的 MCU 引脚。它们是芯片管脚名；具体排针/连接器编号需以所用 PCB 原理图和线束为准。

| 外设/信号 | MCU 引脚 | 复用/方向 | 当前用途及连接说明 |
| --- | --- | --- | --- |
| CAN1_RX / CAN1_TX | PD0 / PD1 | AF9；RX / TX | CAN1 板间通信，两块控制板互连 |
| CAN2_RX / CAN2_TX | PB5 / PB6 | AF9；RX / TX | CAN2 本板电机和功率计总线 |
| SPI1_SCK / MISO / MOSI | PB3 / PB4 / PA7 | AF5 | BMI088 共用 SPI 总线 |
| IST8310 I2C3 SCL / SDA | PA8 / PC9 | AF4，开漏 | 板载磁力计，7 位地址 `0x0E`，HAL 地址 `0x1C` |
| IST8310 复位 / DRDY | PG6 / PG3 | GPIO 输出 / EXTI3 下降沿 | 复位低有效；DRDY 记录样本时间戳，I2C 在后台中断异步传输 |
| BMI088 加速度计 CS | PA4 | GPIO 输出，低有效 | 独立片选；不是硬件 NSS |
| BMI088 陀螺仪 CS | PB0 | GPIO 输出，低有效 | 独立片选；不是硬件 NSS |
| USART1_TX / RX | PA9 / PB7 | AF7；TX / RX | VOFA+ 从 PA9 发出；PB7 当前配置但此应用未用于接收 |
| USART3_TX / RX | PC10 / PC11 | AF7；TX / RX | DBUS 接收使用 PC11；PC10 已配置但当前应用不发送 |
| USART6_TX / RX | PG14 / PG9 | AF8；TX / RX | 裁判系统接收使用 PG9；PG14 已配置但当前应用不发送 |
| 蜂鸣器 PWM | PD14 | TIM4_CH3，AF2 | 蜂鸣器驱动输出；运行时由 BSP 设置 PWM 频率 |
| IMU 加热 PWM | PF6 | TIM10_CH1，AF3 | BMI088 加热器输出；TIM10 同时配置基础定时中断 |
| 板载 RGB LED | PH10 / PH11 / PH12 | GPIO 推挽输出 | 红 / 绿 / 蓝；无上下拉 |
| GPIO 外部中断输入 | PC4 / PC5 | GPIO 输入，EXTI 上升沿 | CubeMX 已配置，无上下拉；当前应用未见对应的板级事件处理回调 |
| HSE 外部晶振 | PH0 / PH1 | OSC_IN / OSC_OUT | 12 MHz 外部高速时钟输入/输出 |
| SWD 调试 | PA13 / PA14 | SWDIO / SWCLK | 下载与在线调试接口；烧录时保留 SWD 连接 |

### 串行总线与 DMA

| 外设 | 参数 | DMA 配置 | 中断优先级（抢占, 子优先级） |
| --- | --- | --- | --- |
| CAN1 / CAN2 | 两者均为 1 Mbps；正常模式；自动总线关闭开启，自动重发关闭。CAN1 使用过滤器 bank 0 起始；CAN2 从 bank 14 起始，过滤器分界为 bank 14。 | 无 | CAN1/2 TX、RX0、RX1：均为 (0, 0) |
| SPI1 | 主机、双线全双工、8 bit、模式 3（CPOL 高、第二边沿采样）、软件 NSS、MSB first、分频 16；当前时钟配置下 SCK 为 5.25 MHz。 | DMA2_Stream0 / Channel 3：RX，外设到内存；DMA2_Stream3 / Channel 3：TX，内存到外设；均为高优先级、普通模式。 | SPI1：(1, 0)；RX DMA：(1, 0)；TX DMA：(0, 0) |
| I2C3 | 400 kHz、7 位地址；IST8310 使用阻塞式寄存器读写。 | 无 | EXTI3：(0, 0) |
| USART1 | 115200 baud，8N1；VOFA+ 使用 TX。 | 当前应用不使用 DMA | USART1：(0, 0) |
| USART3 | 100000 baud，9 位字长、偶校验、1 停止位；DBUS 使用 RX。 | DMA1_Stream1 / Channel 4：RX，外设到内存，字节对齐、普通模式、低优先级。 | USART3 与 RX DMA：均为 (0, 0) |
| USART6 | 115200 baud，8N1；裁判系统使用 RX。 | DMA2_Stream1 / Channel 5：RX，外设到内存，字节对齐、普通模式、低优先级。 | USART6：(0, 0) |

USART3 的“9 位字长 + 偶校验”是 STM32 HAL 的配置写法，其中字长包含校验位，线上格式为 8 数据位 + 偶校验 + 1 停止位（8E1）。接线时使用 DBUS 信号输出接 PC11（USART3_RX），并确认遥控器接收机与控制板共地。SPI 片选分别控制 BMI088 加速度计与陀螺仪，调试时不要把两路 CS 短接。

### 定时器与时钟

| 定时器 | 配置 | 当前用途 |
| --- | --- | --- |
| TIM14 | PSC=83，ARR=999；APB1 定时器时钟 84 MHz，因此更新频率为 1 kHz。 | 主控制节拍；无外接引脚；更新中断优先级 (0, 0)。 |
| TIM4_CH3 | PWM1，PSC=0，ARR=65535。 | PD14 蜂鸣器输出；应用/BSP 设置目标频率。 |
| TIM10_CH1 | PWM1，PSC=0，ARR=2000；另启用基础定时中断。 | PF6 IMU 加热器；更新中断优先级 (0, 0)。 |

系统时钟由 12 MHz HSE 经 PLL 配置为 168 MHz；APB1 为 42 MHz、APB2 为 84 MHz。若改动时钟树、定时器分频或 CubeMX 配置，应重新核算 CAN 波特率、SPI 频率和控制周期，并同步更新本文档。当前已启用 I2C3，本配置没有启用 ADC。

连 CAN 时确认两端 CANH/CANL 对应、总线有合适终端电阻且波特率一致。项目 CAN 初始化使用标准帧；APP 板间 ID 和数据布局见 [API 参考的通信章节](API_Reference.md#21-底盘通信)。

### 板载磁力计运行与检查

云台板启动时，`app_init.c` 调用 `drv_mag_port_init()`；TIM14 的 1 kHz 控制回调每周期调用 `drv_mag_port_process()`，与 IMU 快照更新处于同一调度周期。状态机只发起中断式 I2C 读写，不等待总线；I2C 完成中断只记录结果，由后续 1 kHz 周期推进配置和采样。磁力计转换周期至少 6 ms，速率低于 IMU；快照保留 DRDY 时间戳，控制算法可据此判断样本新鲜度并与 IMU 样本配对，不会每毫秒生成一条磁场数据。`main.c` 负责 `MX_I2C3_Init()` 等外设初始化并调度应用，无须包含磁力计驱动头文件。底盘板不启用磁力计采样。硬件资源如下，修改接线时同步修改 CubeMX 配置和 `drv_mag_port.c`。

| 资源 | 连接与配置 |
| --- | --- |
| I2C3 | SCL=PA8、SDA=PC9，AF4 开漏，400 kHz；7 位地址 `0x0E`，仅端口层左移为 HAL 地址 `0x1C`。 |
| PG6 | RSTN，低有效；后台状态机拉低 50 ms、释放后等待 50 ms，再检查 ID 和配置。 |
| PG3 / EXTI3 | DRDY，下降沿。BSP 分发到端口，只置就绪标记，中断内不读 I2C。 |
| TIM14 1 kHz | `drv_mag_port_process()` 按状态异步推进复位、配置、转换及读取；I2C IRQ 完成回调不会执行驱动逻辑。转换至少等待 6 ms，DRDY 丢失时轮询状态位；20 ms 未就绪报错，500 ms 后复位重试。 |

需要磁场数据时，包含 `drv_mag_port.h` 并复制快照；时钟由端口内部提供：

```c
drv_mag_snapshot_t mag;
if (drv_mag_port_read_snapshot(&mag) && mag.online) {
    /* mag.sample.x_uT / y_uT / z_uT: 未标定的传感器系磁场，单位 µT。 */
}
```

消费者不要访问驱动实例或改写调试数据。调试器 Watch 可以查看 `s_magnetometer.published`（必要时指定 `drv_mag_port.c` 文件作用域）：`device_id` 应为 `0x10`；`sample_count` 持续增加；`error_count` 正常保持不变。`last_error` 保存最近一次历史故障，是否已恢复应结合 `online` 和采样计数判断。快照 API 还会按当前时间检查 100 ms 过期阈值；Watch 中的存储字段不会自行随时间过期。

磁力计缺失或错误不会阻断云台控制。磁场数据当前仅完成独立采集与健康管理，尚未接入姿态融合；控制角度和 CAN 反馈的六轴姿态来源保持一致。当前默认取消启动时自动极性自检；需要诊断时，先停止后台采样，再单独调用核心自检接口，完成后重新初始化驱动。

## 5. 哨兵双板运行

底盘板与云台板运行同一仓库的固件，但需分别选择板型、分别编译和烧录。

| 板型 | 主要职责 | CAN2 反馈 ID | 电流发送 ID |
| --- | --- | --- | --- |
| `BOARD_CHASSIS` | 双舵轮底盘、功率管理、CAN1 底盘反馈 | 右/左驱动 `0x201/0x202`，左/右转向 `0x205/0x206`，功率计 `0x212` | 驱动 `0x200`，转向 `0x1FF` |
| `BOARD_GIMBAL` | 双 yaw、pitch、摩擦轮、拨弹轮和 IMU | 左/右摩擦轮 `0x201/0x203`，拨弹轮 `0x202`，yaw/pitch `0x205/0x206/0x207` | yaw/pitch `0x1FE`，发射机构 `0x200` |

CAN1 负责底盘与云台之间的控制和状态交换；CAN2 连接本板电机和功率计。修改电机 ID、方向或机械零位时，需同时核对 `app_sentry_common.h` 中的配置与线束实际连接。

## 6. 云台控制与遥控器

云台输入源由遥控器 S2 决定：

| S2 挡位 | 输入源 | 行为 |
| --- | --- | --- |
| 上挡 | DBUS | 摇杆控制底盘速度和云台 yaw/pitch 角速度。 |
| 中挡 | CAN | 使用 CAN1 收到的雷达/上位机命令。 |
| 下挡 | 急停 | 云台和发射机构输出复位。 |

四个摇杆通道和拨轮会按 DBUS 中心值 1024、范围 660 归一化并限幅到 `[-1, 1]`，之后乘以 `app_sentry_common.h` 中配置的速度上限。没有有效 DBUS 数据时，控制输入默认选择 CAN；确认 CAN 控制端和输入数据符合预期。

S1 的挡位值直接用于发射机构模式：`1` 停止，`2` 摩擦轮和拨弹轮运行，`3` 仅摩擦轮运行。CAN 命令还可选择拨弹轮反转模式。停止模式将拨弹轮电流清零，并让摩擦轮速度环以零为目标减速；急停或 IMU 未就绪时则复位控制输出。

## 7. CAN 控制数据速查

APP 板间数据均使用 CAN1 标准帧。下表列出常用哨兵命令；结构体、方向定义及各反馈字段见 [API 参考](API_Reference.md)。

| ID | 接收端 | 含义 |
| --- | --- | --- |
| `0x111` | 底盘 | 底盘速度命令：`int16_t vx, vy, vz, power_pct`。 |
| `0x112` | 云台 | 底盘功率反馈：`int16_t`，功率 × 100。 |
| `0x119` | 云台 | 底盘 z 轴角速度反馈：`float`，rad/s。 |
| `0x120` | 云台 | 雷达/上位机底盘速度命令；CAN 挡位时转发为 `0x111`。 |
| `0x121` / `0x125` | 云台 | yaw/pitch 角度增量，两个 `float`，rad；分别为不射击/射击模式。 |
| `0x123` / `0x127` | 云台 | yaw/pitch 绝对角度，两个 `float`，rad；分别为不射击/射击模式。 |
| `0x129` | 云台 | 发射命令，DLC 4；前两字节是 `shoot_switch` 和 `retreat`。 |
| `0x122` / `0x124` | 底盘 | 云台角速度/角度反馈，各两个 `float`。 |
| `0x126` / `0x233` | 底盘 | 发射机构反馈 / 云台控制姿态四元数（ZYX，w/x/y/z ×30000）。 |
| `0x139` | CAN1 监听端 | 与控制和 `0x124` 同源的 yaw、pitch，两个小端序 `float32`，单位度；不含 roll。 |

多字节字段按 STM32 小端序布局。CAN 接收回调只保存最新值；云台 200 Hz 调度中处理待转发的 `0x120`，且仅在 S2 位于 CAN 挡时转发。

IMU 位于小 yaw 平台，不随 pitch 转动，先消除固定竖装旋转，再沿用等效水平 IMU 的云台轴映射：控制 pitch 为补偿后的 IMU roll 加上 pitch 编码器相对机械零位的角度；yaw 为补偿后的 IMU yaw，云台 roll 为负的补偿后 IMU pitch。`0x124`（rad）、`0x139`（度）和 `0x233`（ZYX 四元数）均取同一组控制姿态，状态发送始终按 200 Hz 调度，不要求接入小电脑或选择 CAN 控制源。四元数解码须除以 30000、归一化并按 ZYX 反解。坐标定义、编码器补偿模型及帧布局详见 [CAN 协议](CAN_Protocol.md#云台姿态的统一基准)。

## 8. VOFA+ 调试

云台板通过 USART1 每个 200 Hz 发射周期发送 8 个 `float`。帧尾为 JustFloat 标记 `00 00 80 7F`，帧长 36 字节。通道定义如下：

| 通道 | 内容 | 单位 |
| --- | --- | --- |
| 0 / 1 | 拨弹轮目标 / 实际转速 | rad/s |
| 2 / 3 | 左摩擦轮目标 / 实际转速 | rad/s |
| 4 / 5 | 右摩擦轮目标 / 实际转速 | rad/s；目标为负值 |
| 6 | 右摩擦轮反馈电流原始值 | 驱动反馈单位 |
| 7 | 云台控制 pitch（与 0x139 pitch 相同，含编码器补偿） | 度 |

VOFA 端串口设为 115200 baud、8N1，数据格式选择 JustFloat 并添加 8 个浮点通道。发送接口是非阻塞的：前一帧尚未完成时，新帧会以 `DRV_VOFA_ERR_BUSY` 返回并被跳过。

## 9. 排查顺序

1. **固件构建失败**：检查 CMake、Ninja 和 `arm-none-eabi-gcc` 是否可执行；切换板型后使用干净的构建缓存。
2. **CAN 无反馈**：确认 CAN1/CAN2 接线、终端电阻、波特率及电机反馈 ID；检查板型对应的 CAN 过滤器和总线选择。
3. **云台 IMU 无更新**：检查 BMI088 的 SPI1 连线、PA4/PB0 片选、DMA 配置，以及校准期间是否保持静止。
4. **DBUS 无输入**：检查 USART3 RX 和接收 DMA；S2 下挡会触发急停，上挡使用 DBUS，中挡使用 CAN。
5. **两板控制不通**：先确认 CAN1 物理链路，再按 `0x120`/`0x111`、`0x119` 和云台反馈 ID 检查报文方向与格式。
6. **VOFA 曲线无数据**：检查 USART1 TX 到上位机 RX 的连接；串口设为 115200 baud、8N1，解析选择 JustFloat，并配置 8 个通道。

调试时可在 `app_control_1khz()`、哨兵控制入口、CAN RX 回调和 VOFA 发送结果处设置断点。底盘和云台的电机 ID、电机方向、编码器零位、PID 参数及限位均以 `Self_core/App/sentry/app_sentry_common.h` 和相应控制文件为准。

遥控器控制底盘时，断点应放在 `gimbal_send_chassis_cmd()`；`gimbal_forward_chassis_speed_cmd()` 专用于小电脑 `0x120` 转发，S2 上挡不会进入该分支。云台每 5 ms 先提交底盘命令，再提交姿态反馈；CAN1 软件缓冲处理三个硬件邮箱不足的情况。

可在 Watch 中检查以下字段（累计计数应让程序运行一段时间后再暂停比较）：

| Watch 表达式 | 判断依据 |
| --- | --- |
| `app_sentry_gimbal_debug.chassis.input_source` | `1`=DBUS、`3`=CAN、`2`=急停。遥控器正常控制时应为 `1`。 |
| `app_sentry_gimbal_debug.chassis.vx_tar_speed` / `vy_tar_speed` / `vw_tar_speed` | 摇杆/拨轮变化时目标应随之变化；旋转字段单位为 0.001 rad/s。 |
| `app_sentry_gimbal_debug.chassis.tx_attempt_count` / `tx_accept_count` / `tx_fail_count` | DBUS/急停模式下尝试与接受计数应增加；失败计数正常应为 0。 |
| `app_sentry_gimbal_debug.chassis.tx_can_skip_count` | 增加说明选择了 CAN 控制源，遥控器发送路径被跳过。 |
