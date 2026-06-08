/**
 * @file    app_motor.h
 * @brief   电机管理桥接层 — CAN 回调分发 / 在线检测 / 失控保护
 * @note    被动监听模式: 电机主动回传数据, 超时未收到即判离线
 *          支持 DJI (大端) 和翎控 (小端) 两种协议
 */
#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include "lib_typedef.h"
#include "drv_motor.h"
#include "bsp_can.h"

#define APP_MOTOR_MAX              8
#define APP_MOTOR_TIMEOUT_MS       10      /* 电机超时阈值 (DJI ~1kHz) */

/* DJI 控制帧 CAN ID */
#define APP_MOTOR_CTRL_GROUP0      0x200   /* 索引 0~3 */
#define APP_MOTOR_CTRL_GROUP1      0x1FF   /* 索引 4~7 */

/**
 * @brief  电机协议类型
 */
typedef enum {
    APP_MOTOR_TYPE_DJI = 0,         /* 大端, DJI 协议 */
    APP_MOTOR_TYPE_LINGKONG,        /* 小端, 翎控协议 */
} app_motor_type_t;

/**
 * @brief  电机配置项 (初始化用)
 */
typedef struct {
    uint32_t          can_id;       /* 回传 CAN ID (如 0x201) */
    app_motor_type_t  type;         /* 协议类型 */
} app_motor_cfg_t;

/**
 * @brief  初始化电机管理
 * @param  hcan    CAN 句柄
 * @param  cfgs    电机配置数组
 * @param  count   电机数量 (≤ APP_MOTOR_MAX)
 * @note   注册单个 CAN 回调, 收到数据后按 can_id 匹配索引,
 *         按 type 选择 DJI / 翎控 解析函数
 */
void app_motor_init(CAN_HandleTypeDef *hcan,
                    const app_motor_cfg_t *cfgs, uint8_t count);

/**
 * @brief  设置某路电机电流指令 (离线自动归零)
 * @param  idx      电机索引 [0, count)
 * @param  current  电流值
 * @return 0=正常, -1=电机已离线 (指令已强制置 0)
 */
int app_motor_set_current(uint8_t idx, int16_t current);

/**
 * @brief  构建并发送一组电机的 DJI 电流帧 (CAN 0x200 / 0x1FF)
 * @param  group  0=索引 0~3, 1=索引 4~7
 * @note   离线电机自动填 0; 每路 2 字节, 大端编码
 */
void app_motor_send_frame(uint8_t group);

/**
 * @brief  获取电机数据
 */
int  app_motor_get_data(uint8_t idx, drv_motor_data_t *out);

/**
 * @brief  查询电机在线状态
 */
uint8_t app_motor_is_online(uint8_t idx);

/**
 * @brief  获取电机总数
 */
uint8_t app_motor_get_count(void);

/**
 * @brief  更新所有电机的在线状态 (需周期性调用)
 */
void app_motor_refresh_online(void);

#endif
