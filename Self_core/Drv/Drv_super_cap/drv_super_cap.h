/**
 * @file    drv_super_cap.h
 * @brief   超级电容控制帧构建、反馈解析及 CAN 端口接口。
 */
#ifndef DRV_SUPER_CAP_H
#define DRV_SUPER_CAP_H

#include "lib_typedef.h"

/* 底盘视角的标准 CAN ID；数据场固定为 8 字节，多字节字段采用小端序。 */
#define DRV_SUPER_CAP_CAN_ID_FEEDBACK  0x211U
#define DRV_SUPER_CAP_CAN_ID_CONTROL   0x213U

/** 超级电容反馈物理量。 */
typedef struct {
    float battery_voltage; // 电池电压，V。
    float battery_current; // 电池电流，A，保留协议有符号值。
    float cap_voltage; // 电容电压，V。
    float max_output_power; // 电容与电池合计的最大输出功率，W。
} drv_super_cap_feedback_t;

/** 底盘下发的超级电容控制参数。 */
typedef struct {
    float power_limit; // 裁判系统功率限制，W，范围 0～655.35，分辨率 0.01 W。
    uint8_t discharge_strategy; // 放电策略，0～255；当前控制器固件尚未实现。
    uint8_t enabled; // 请求输出使能：0=关闭，1=开启。
} drv_super_cap_control_t;

/** 超级电容反馈实例。 */
typedef struct {
    drv_super_cap_feedback_t feedback;
    uint32_t rx_tick_ms; // 最近一次有效反馈的接收时间，ms。
    uint8_t has_feedback; // 是否已接收有效反馈，0=否，1=是。
} drv_super_cap_t;

/**
 * @brief  清零超级电容反馈及接收状态。
 * @param  super_cap 超级电容实例；空指针时不执行操作。
 * @note   在接收启动前调用。
 */
void drv_super_cap_init(drv_super_cap_t *super_cap);

/**
 * @brief  解析 0x211 反馈，将协议单位转换为 V、A、W。
 * @param  super_cap 超级电容实例。
 * @param  std_id 标准 CAN ID，应为 DRV_SUPER_CAP_CAN_ID_FEEDBACK。
 * @param  data CAN 数据场，8 字节，小端序。
 * @param  len 数据长度，必须为 8。
 * @param  now_ms 当前毫秒时间戳。
 * @return 1=解析成功；0=参数、ID 或长度错误，原实例保持不变。
 * @note   反馈不包含实际使能状态，也不包含电容输出电流。
 *         若在中断中更新，调用方应在临界区内复制反馈快照后使用。
 */
uint8_t drv_super_cap_solve(drv_super_cap_t *super_cap, uint32_t std_id,
                            const uint8_t *data, uint8_t len, uint32_t now_ms);

/**
 * @brief  构建 0x213 控制帧，保留字节全部置零。
 * @param  control 控制参数；enabled 仅接受 0 或 1。
 * @param  frame 至少 8 字节的输出缓冲区。
 * @return 1=构建成功；0=空指针、功率非有限数或参数越界，输出保持不变。
 * @note   功率四舍五入到 0.01 W；字节 4 的 bit0 为低有效使能位。
 *         由 App 约每 100 ms 发送一次；控制器连续 1 s 未收到控制帧将关闭输出。
 */
uint8_t drv_super_cap_pack(const drv_super_cap_control_t *control, uint8_t frame[8]);

/**
 * @brief  判断反馈是否在线，时间差采用无符号减法处理毫秒计数回绕。
 * @param  super_cap 超级电容实例。
 * @param  now_ms 当前毫秒时间戳。
 * @param  timeout_ms 反馈有效期，ms；必须大于 0。
 * @return 1=已收到反馈且时间差小于有效期；0=未收到、超时或参数无效。
 * @note   调用方应保证接收状态读取与中断更新互斥；在线不代表输出已使能。
 */
uint8_t drv_super_cap_is_online(const drv_super_cap_t *super_cap,
                                uint32_t now_ms, uint32_t timeout_ms);

/**
 * @brief  为指定 CAN 总线注册 0x211 反馈回调。
 * @param  hcan CAN 句柄，类型为 CAN_HandleTypeDef *。
 * @param  rx_cb 接收回调，在 CAN 中断中执行。
 * @note   不启动总线；同一总线同 ID 的重复注册会覆盖旧回调。
 */
void drv_super_cap_port_can_init(void *hcan,
                                 void (*rx_cb)(uint32_t, uint8_t *, uint8_t));

/**
 * @brief  构建并非阻塞发送标准 ID 0x213、DLC=8 的控制帧。
 * @param  hcan CAN 句柄，类型为 CAN_HandleTypeDef *。
 * @param  control 控制参数。
 * @return 1=发送已接受；0=参数无效或发送被拒绝。
 * @note   返回成功不代表已得到总线 ACK；发送周期由 App 调度。
 */
uint8_t drv_super_cap_port_can_tx(void *hcan, const drv_super_cap_control_t *control);

#endif
