/**
 * @file    drv_dbus.h
 * @brief   DBUS 遥控器数据解码 (纯逻辑层)
 * @note    不依赖 BSP / HAL, 输入 18 字节原始数据, 输出解码后的遥控数据
 */
#ifndef DRV_DBUS_H
#define DRV_DBUS_H

#include "lib_typedef.h"

#define DRV_DBUS_BUFFER_SIZE  18

/**
 * @brief  DBUS 解码数据结构
 */
typedef struct {
    struct {
        uint16_t ch[4];         /* 摇杆通道 (0~1684, 中点≈1024) */
        uint8_t  s1;            /* 拨轮开关 S1 (1=上, 2=中, 3=下) */
        uint8_t  s2;            /* 拨轮开关 S2 */
        uint16_t rolling_wheel; /* 滚轮 */
    } rc;

    struct {
        int16_t  x;             /* 鼠标 X 位移 (像素) */
        int16_t  y;             /* 鼠标 Y 位移 */
        int16_t  z;             /* 鼠标滚轮 */
        uint8_t  left;          /* 鼠标左键 (0=松开, 1=按下) */
        uint8_t  right;         /* 鼠标右键 */
    } mouse;

    struct {
        uint8_t w;              /* W 键 */
        uint8_t s;              /* S 键 */
        uint8_t a;              /* A 键 */
        uint8_t d;              /* D 键 */
        uint8_t q;              /* Q 键 */
        uint8_t e;              /* E 键 */
        uint8_t shift;          /* Shift 键 */
        uint8_t ctrl;           /* Ctrl 键 */
    } keyboard;
} drv_dbus_data_t;

/**
 * @brief  解码 DBUS 18 字节原始数据
 * @param  buffer  串口 DMA 接收缓冲区 (18 bytes)
 * @param  data    解码结果输出
 */
void drv_dbus_decode(const uint8_t buffer[DRV_DBUS_BUFFER_SIZE],
                     drv_dbus_data_t *data);

// ─── Port: BSP 适配 ─────────────────────────────

/**
 * @brief  初始化 DBUS (UART DMA + IDLE 中断)
 * @note   挂接 USART3 + DMA, 硬编码硬件映射
 */
void drv_dbus_port_init(void);

/**
 * @brief  UART IDLE 中断入口
 */
void drv_dbus_port_irq_handler(void);

/**
 * @brief  获取最新解码的 DBUS 数据
 * @return const drv_dbus_data_t*  只读指针
 */
const drv_dbus_data_t *drv_dbus_port_get_data(void);

#endif
