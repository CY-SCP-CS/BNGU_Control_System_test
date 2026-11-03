//
// Created by bismarckkk on 2025/3/22.
// Dynamic Edition
//

#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include "ui_types.h"
#include "drv_referee.h"

extern int ui_self_id;

void print_message(const uint8_t* message, int length);

// User Code Begin             //把这里与drv层的裁判系统发送函数对接
#define SEND_MESSAGE(message, length)  drv_referee_port_send_frame((message), (length))
// User Code End

#define ui_set(obj, field, value) { \
    obj->field = value; \
    obj##_dirty = obj##_max_send_count; \
}

void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);
void ui_proc_delete_frame(ui_delete_frame_t *msg);

void ui_delete_layer(const uint8_t delete_type, const uint8_t layer);

/**
 * @brief 每次扫描最多提交一帧，调用方负责控制发送周期
 * @param ui_now_figures
 * @param ui_dirty_figure
 * @param ui_now_strings
 * @param ui_dirty_string
 * @param total_figures
 * @param total_strings
 * @note 发送成功后才减少dirty；添加完成后将对应对象切换为修改。
 */
void ui_scan_and_send(ui_interface_figure_t* ui_now_figures, uint8_t* ui_dirty_figure, ui_interface_string_t* ui_now_strings, uint8_t* ui_dirty_string, int total_figures, int total_strings);

#endif //UI_INTERFACE_H
