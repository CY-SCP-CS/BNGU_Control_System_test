//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 3
#define TOTAL_STRING 4

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

uint8_t ui_g_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
    1,
    1,
    1,
    1,
    1,
    1,
    1,
};

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_g() {
    memset(ui_g_now_figures, 0, sizeof(ui_g_now_figures));
    memset(ui_g_now_strings, 0, sizeof(ui_g_now_strings));
    ui_g_Ungroup_NewRound->figure_type = 2;
    ui_g_Ungroup_NewRound->operate_type = 1;
    ui_g_Ungroup_NewRound->layer = 0;
    ui_g_Ungroup_NewRound->color = 8;
    ui_g_Ungroup_NewRound->start_x = 428;
    ui_g_Ungroup_NewRound->start_y = 490;
    ui_g_Ungroup_NewRound->width = 10;
    ui_g_Ungroup_NewRound->r = 18;

    ui_g_Ungroup_NewRect->figure_type = 1;
    ui_g_Ungroup_NewRect->operate_type = 1;
    ui_g_Ungroup_NewRect->layer = 0;
    ui_g_Ungroup_NewRect->color = 8;
    ui_g_Ungroup_NewRect->start_x = 418;
    ui_g_Ungroup_NewRect->start_y = 572;
    ui_g_Ungroup_NewRect->width = 1;
    ui_g_Ungroup_NewRect->end_x = 553;
    ui_g_Ungroup_NewRect->end_y = 622;

    ui_g_Ungroup_NewLine->figure_type = 0;
    ui_g_Ungroup_NewLine->operate_type = 1;
    ui_g_Ungroup_NewLine->layer = 0;
    ui_g_Ungroup_NewLine->color = 2;
    ui_g_Ungroup_NewLine->start_x = 424;
    ui_g_Ungroup_NewLine->start_y = 577;
    ui_g_Ungroup_NewLine->width = 40;
    ui_g_Ungroup_NewLine->end_x = 490;
    ui_g_Ungroup_NewLine->end_y = 577;

    ui_g_Ungroup_friction_state_Text->figure_type = 7;
    ui_g_Ungroup_friction_state_Text->operate_type = 1;
    ui_g_Ungroup_friction_state_Text->layer = 0;
    ui_g_Ungroup_friction_state_Text->color = 0;
    ui_g_Ungroup_friction_state_Text->start_x = 466;
    ui_g_Ungroup_friction_state_Text->start_y = 506;
    ui_g_Ungroup_friction_state_Text->width = 2;
    ui_g_Ungroup_friction_state_Text->font_size = 18;
    ui_g_Ungroup_friction_state_Text->str_length = 3;
    strcpy(ui_g_Ungroup_friction_state_Text->string, "OFF");

    ui_g_Ungroup_HeatText->figure_type = 7;
    ui_g_Ungroup_HeatText->operate_type = 1;
    ui_g_Ungroup_HeatText->layer = 0;
    ui_g_Ungroup_HeatText->color = 8;
    ui_g_Ungroup_HeatText->start_x = 418;
    ui_g_Ungroup_HeatText->start_y = 644;
    ui_g_Ungroup_HeatText->width = 1;
    ui_g_Ungroup_HeatText->font_size = 14;
    ui_g_Ungroup_HeatText->str_length = 4;
    strcpy(ui_g_Ungroup_HeatText->string, "HEAT");

    ui_g_Ungroup_HEAT_Number_ext->figure_type = 7;
    ui_g_Ungroup_HEAT_Number_ext->operate_type = 1;
    ui_g_Ungroup_HEAT_Number_ext->layer = 0;
    ui_g_Ungroup_HEAT_Number_ext->color = 2;
    ui_g_Ungroup_HEAT_Number_ext->start_x = 480;
    ui_g_Ungroup_HEAT_Number_ext->start_y = 643;
    ui_g_Ungroup_HEAT_Number_ext->width = 1;
    ui_g_Ungroup_HEAT_Number_ext->font_size = 14;
    ui_g_Ungroup_HEAT_Number_ext->str_length = 5;
    strcpy(ui_g_Ungroup_HEAT_Number_ext->string, "--/--");

    ui_g_Ungroup_pitch_Text->figure_type = 7;
    ui_g_Ungroup_pitch_Text->operate_type = 1;
    ui_g_Ungroup_pitch_Text->layer = 0;
    ui_g_Ungroup_pitch_Text->color = 8;
    ui_g_Ungroup_pitch_Text->start_x = 414;
    ui_g_Ungroup_pitch_Text->start_y = 551;
    ui_g_Ungroup_pitch_Text->width = 1;
    ui_g_Ungroup_pitch_Text->font_size = 14;
    ui_g_Ungroup_pitch_Text->str_length = 10;
    strcpy(ui_g_Ungroup_pitch_Text->string, "PITCH: ---");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    // 仅准备添加操作，由后续周期调用ui_update_g()逐帧发送。
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = ui_g_max_send_count[i];
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
            ui_g_dirty_string[i] = ui_g_max_send_count[TOTAL_FIGURE + i];
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
#ifndef MANUAL_DIRTY
    // 发送器内部将添加切换为修改，不把这个变化当成新的数据更新。
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_last_figures[i].operate_type = ui_g_now_figures[i].operate_type;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_last_strings[i].operate_type = ui_g_now_strings[i].operate_type;
    }
#endif
}
