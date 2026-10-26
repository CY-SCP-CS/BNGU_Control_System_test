//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[3];
extern uint8_t ui_g_dirty_figure[3];
extern ui_interface_string_t ui_g_now_strings[4];
extern uint8_t ui_g_dirty_string[4];

extern uint8_t ui_g_max_send_count[7];

#define ui_g_Ungroup_NewRound ((ui_interface_round_t*)&(ui_g_now_figures[0]))
#define ui_g_Ungroup_NewRect ((ui_interface_rect_t*)&(ui_g_now_figures[1]))
#define ui_g_Ungroup_NewLine ((ui_interface_line_t*)&(ui_g_now_figures[2]))

#define ui_g_Ungroup_friction_state_Text (&(ui_g_now_strings[0]))
#define ui_g_Ungroup_HeatText (&(ui_g_now_strings[1]))
#define ui_g_Ungroup_HEAT_Number_ext (&(ui_g_now_strings[2]))
#define ui_g_Ungroup_pitch_Text (&(ui_g_now_strings[3]))

#define ui_g_Ungroup_NewRound_max_send_count (ui_g_max_send_count[0])
#define ui_g_Ungroup_NewRect_max_send_count (ui_g_max_send_count[1])
#define ui_g_Ungroup_NewLine_max_send_count (ui_g_max_send_count[2])

#define ui_g_Ungroup_friction_state_Text_max_send_count (ui_g_max_send_count[3])
#define ui_g_Ungroup_HeatText_max_send_count (ui_g_max_send_count[4])
#define ui_g_Ungroup_HEAT_Number_ext_max_send_count (ui_g_max_send_count[5])
#define ui_g_Ungroup_pitch_Text_max_send_count (ui_g_max_send_count[6])

#ifdef MANUAL_DIRTY
#define ui_g_Ungroup_NewRound_dirty (ui_g_dirty_figure[0])
#define ui_g_Ungroup_NewRect_dirty (ui_g_dirty_figure[1])
#define ui_g_Ungroup_NewLine_dirty (ui_g_dirty_figure[2])

#define ui_g_Ungroup_friction_state_Text_dirty (ui_g_dirty_string[0])
#define ui_g_Ungroup_HeatText_dirty (ui_g_dirty_string[1])
#define ui_g_Ungroup_HEAT_Number_ext_dirty (ui_g_dirty_string[2])
#define ui_g_Ungroup_pitch_Text_dirty (ui_g_dirty_string[3])
#endif

void ui_init_g();
void ui_update_g();

#endif // UI_g_H
