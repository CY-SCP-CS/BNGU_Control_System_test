/**
 * @file    drv_motor.h
 * @brief   电机协议层: DJI / 翎控 数据解析与命令构建
 */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "lib_typedef.h"
//翎控电机没有做完整适配，达妙没写过，均待补充

typedef struct {
    uint8_t  cmd_id;//命令ID
    int16_t  speed;//转速
    uint16_t angle;//编码器值
    uint8_t  temperature;//温度
    int16_t  current;//当前电流
} drv_motor_data_t;

#define DRV_MOTOR_DJI_FRAME_MAX  4   /* 每帧最多 4 路电机 */


//外部接口
/**
 * @brief  解析 DJI 电机  回传数据
 * @param  data   CAN 数据场 (8 字节, 大端)
 * @param  cur    电机数据结构体指针
 */
void drv_motor_solve_dji_data(const uint8_t *data, drv_motor_data_t *cur);

/**
 * @brief  初始化 DJI 电流帧 (全部置零)
 * @param  frame  输出缓冲区 (8 字节)
 */
void drv_motor_build_dji_frame_init(uint8_t *frame);

/**
 * @brief  设置 DJI 电流帧中某一路电流
 * @param  frame    帧缓冲区 (8 字节)
 * @param  slot     槽位 [0..3], 对应帧内位置
 * @param  current  电流值 (大端编码)
 */
void drv_motor_build_dji_frame_set(uint8_t *frame, uint8_t slot,
                                   int16_t current);

/**
 * @brief  解析翎控电机回传数据
 * @param  data   CAN 数据场 (8 字节, 小端)
 * @param  cur    电机数据结构体指针
 */
void drv_motor_solve_lk_data(const uint8_t *data, drv_motor_data_t *cur);

/**
 * @brief  构建翎控电机读取命令帧
 * @param  frame  输出缓冲区 (8 字节)
 */
void drv_motor_build_lk_read_frame(uint8_t *frame);

/**
 * @brief  构建 翎控 电流指令帧 (小端)
 * @param  frame    输出缓冲区 (8 字节)
 * @param  current  电流值
 */
void drv_motor_build_lk_frame(uint8_t *frame, int16_t current);

//Port层

/**
 * @brief  CAN 初始化 (注册接收回调)
 * @param  hcan    CAN 句柄 (void*)
 * @param  can_id  要监听的 CAN ID
 * @param  rx_cb   接收回调
 */
void drv_motor_port_can_init(void *hcan, uint32_t can_id,
                              void (*rx_cb)(uint32_t, uint8_t*, uint8_t));

/**
 * @brief  CAN 发送 (非阻塞)
 * @param  hcan    CAN 句柄 (void*)
 * @param  std_id  标准 ID
 * @param  data    数据 (8 字节)
 */
void drv_motor_port_can_send(void *hcan, uint32_t std_id, uint8_t *data);

/**
 * @brief  获取系统 tick (毫秒)
 * @return uint32_t  当前 tick 值
 */
uint32_t drv_motor_port_get_tick(void);

#endif
