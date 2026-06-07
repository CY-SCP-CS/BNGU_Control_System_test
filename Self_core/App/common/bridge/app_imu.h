/**
 * @file    app_imu.h
 * @brief   BMI088 上层胶水 — 挂接 BSP SPI/GPIO/TIM 与 DRV IMU
 */
#ifndef APP_IMU_H
#define APP_IMU_H

#include "drv_imu.h"

/**
 * @brief  初始化 BMI088 (SPI + CS 引脚挂接, 芯片启动)
 * @param  imu  IMU 句柄
 */
void app_imu_init(drv_imu_t *imu);

/**
 * @brief  开机自校准流程 (加热 → 陀螺零偏 → 初始对准 → 收敛 → 记录零位)
 * @param  imu  IMU 句柄
 * @retval 0  成功
 */
int app_imu_calibrate(drv_imu_t *imu);

#endif
