//
// Created by 33140 on 2026/11/4.
//

#ifndef BNGU_CONTROL_SYSTEM_NAVIGATION_CTRL_H
#define BNGU_CONTROL_SYSTEM_NAVIGATION_CTRL_H

#include <stdint.h>
#include "drv_imu.h"

/**
 * @brief 绑定 IMU，初始化底盘、遥控接收及导航控制状态。
 * @param imu_instance 已完成基础初始化的 IMU 实例，需在运行期间保持有效。
 */
void navigation_ctrl_init(drv_imu_t *imu_instance);


/**
 * @brief 每 1 ms 调用：更新 IMU，并按节拍执行底盘控制。
 * @param count 外部 1 ms 节拍计数，count % 5 == 0 时执行底盘控制。
 * @note 调用前须完成 navigation_ctrl_init()。
 */
void navigation_ctrl_1khz(uint8_t count);


#endif // BNGU_CONTROL_SYSTEM_NAVIGATION_CTRL_H
