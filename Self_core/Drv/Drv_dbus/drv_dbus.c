/**
 * @file    drv_dbus.c
 * @brief   DBUS 遥控器数据解码实现 (纯逻辑层)
 */
#include "drv_dbus.h"
#include <string.h>

void drv_dbus_decode(const uint8_t buffer[DRV_DBUS_BUFFER_SIZE],
                     drv_dbus_data_t *data)
{
    memset(data, 0, sizeof(*data));

    /* ── 摇杆通道 ── */
    data->rc.ch[0] = buffer[0] | ((buffer[1] & 0x07) << 8);
    data->rc.ch[1] = ((buffer[1] & 0xF8) >> 3) | ((buffer[2] & 0x3F) << 5);
    data->rc.ch[2] = ((buffer[2] & 0xC0) >> 6) | (buffer[3] << 2)
                   | ((buffer[4] & 0x01) << 10);
    data->rc.ch[3] = ((buffer[4] & 0xFE) >> 1) | ((buffer[5] & 0x0F) << 7);

    /* ── 拨轮开关 ── */
    data->rc.s1 = (buffer[5] >> 6) & 0x03;
    data->rc.s2 = (buffer[5] >> 4) & 0x03;

    /* ── 鼠标 ── */
    data->mouse.x     = (int16_t)(buffer[6] | (buffer[7] << 8));
    data->mouse.y     = (int16_t)(buffer[8] | (buffer[9] << 8));
    data->mouse.z     = (int16_t)(buffer[10] | (buffer[11] << 8));
    data->mouse.left  = buffer[12];
    data->mouse.right = buffer[13];

    /* ── 键盘 ── */
    data->keyboard.w     = (buffer[14] >> 0) & 0x01;
    data->keyboard.s     = (buffer[14] >> 1) & 0x01;
    data->keyboard.a     = (buffer[14] >> 2) & 0x01;
    data->keyboard.d     = (buffer[14] >> 3) & 0x01;
    data->keyboard.q     = (buffer[14] >> 4) & 0x01;
    data->keyboard.e     = (buffer[14] >> 5) & 0x01;
    data->keyboard.shift = (buffer[14] >> 6) & 0x01;
    data->keyboard.ctrl  = (buffer[14] >> 7) & 0x01;

    /* ── 滚轮 ── */
    data->rc.rolling_wheel = (buffer[16] | (buffer[17] << 8));
}