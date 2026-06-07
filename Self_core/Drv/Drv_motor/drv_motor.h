/**
 * @file    drv_motor.h
 * @brief   电机协议层: DJI / 翎控 数据解析与命令构建
 */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "lib_typedef.h"

// ─── 电机数据结构体 ─────────────────────────────

typedef struct {
    uint8_t  cmd_id;        /* 命令 ID (翎控)       */
    int16_t  speed;         /* 转速 (rpm)           */
    uint16_t angle;         /* 编码器值             */
    uint8_t  temperature;   /* 温度                 */
    int16_t  current;       /* 电流                 */
    int16_t  power;         /* 功率 (翎控)          */
} drv_motor_data_t;

// ─── DJI 电流帧参数 ─────────────────────────────

#define DRV_MOTOR_DJI_FRAME_MAX  4   /* 每帧最多 4 路电机 */

// ─── 接口声明 ─────────────────────────────────────

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

#endif
