//
// Created by 33140 on 2026/10/13.
//

#include "app_Armored_ui.h"
#include "drv_referee.h"
#include "Bsp/bsp_tim.h"
#include "ui.h"
#include <stdio.h>
#include <string.h>

static app_Armored_ui_data_t s_ui_data; //要画UI的数据

typedef struct {
    drv_referee_interaction_figure_t figure; // 图形配置
    uint8_t sent;                           // 0=尚未启动添加帧发送，1=已启动
} app_ui_object_t;

static app_ui_object_t s_ammo_number;
static app_ui_object_t s_pitch_number;

void app_Armored_ui_init(uint8_t* friction_state , uint16_t* heat , uint16_t* heat_limit ,
                  uint16_t* projectile_allowance, float* pitch_deg, uint8_t* control_mode)
{
    s_ui_data.initial = 0U;

    if (friction_state == NULL || heat == NULL ||heat_limit == NULL || projectile_allowance == NULL
                                              ||pitch_deg == NULL || control_mode == NULL) {return;}

    s_ui_data.friction_state = friction_state;
    s_ui_data.heat_info.heat = heat;
    s_ui_data.heat_info.heat_limit = heat_limit;
    s_ui_data.projectile_allowance = projectile_allowance;
    s_ui_data.pitch_deg = pitch_deg;
    s_ui_data.control_mode = control_mode;

     ui_init_g();  //调用生成代码中的初始化函数

    s_ui_data.initial = 1;
}

// 摩擦轮状态显示：0是圆形指示灯，1是状态文字。
// 屏幕左下角为坐标原点，x向右增加，y向上增加；坐标、半径和线宽以像素计。
// 颜色和文字长度由 friction_state_config() 根据当前状态设置。
static app_ui_object_t friction_state[2] = {
    // friction_state[0]：圆形状态指示灯。
    {
        .figure = {
            .figure_name = {'F', 'R', 'C'}, // 3字节图形名称，后续修改或删除使用同一名称。
            .operate_type = 1U,            // 图形操作：1=添加，2=修改，3=删除。
            .figure_type = 2U,             // 图形类型：2=正圆。
            .layer = 0U,                   // 位于第0层，协议支持图层0~9。
            .width = 25U,                  // 圆形轮廓线宽。
            .start_x = 125U,               // 圆心x坐标。
            .start_y = 250U,               // 圆心y坐标。
            .details_c = 100U,             // 正圆类型下，details_c表示半径。
        },
        .sent = 0U,
    },
    // friction_state[1]：OFF、SPINUP、READY或FAULT等状态文字。
    {
        .figure = {
            .figure_name = {'F', 'R', 'T'}, // 文字图形名称，与圆形名称区分。
            .operate_type = 1U,            // 首次发送时添加文字图形。
            .figure_type = 7U,             // 图形类型：7=字符，实际文字存放在30字节字符区。
            .layer = 0U,                   // 文字位于第0层。
            .details_a = 20U,              // 字符类型下，details_a表示字体大小。
            .width = 2U,                  // 文字笔画线宽。
            .start_x = 275U,               // 文字起点x坐标。
            .start_y = 250U,               // 文字起点y坐标。
        },
        .sent = 0U,
    },
};

typedef struct {
    uint8_t color;
    uint8_t text[30];
} friction_state_style_t;

// 状态0、1、2依次对应关闭、加速、就绪；最后一项作为异常状态。
static const friction_state_style_t s_friction_styles[] = {
    {8U, "OFF"},
    {1U, "SPINUP"},
    {2U, "READY"},
    {4U, "FAULT"},
};

static const uint8_t *c_text_data;

/**
 * @brief 摩擦轮状态ui发送配置
 * @return 0配置失败 1配置成功
 */
static uint8_t friction_state_config(void)
{
    if (s_ui_data.friction_state == NULL) {
        return 0U;
    }

    uint8_t state = *s_ui_data.friction_state;
    const size_t style_count = sizeof(s_friction_styles) / sizeof(s_friction_styles[0]);
    if (state >= style_count) {
        state = (uint8_t)(style_count - 1U);
    }

    const friction_state_style_t *style = &s_friction_styles[state];
    friction_state[0].figure.color = style->color;
    friction_state[1].figure.color = style->color;
    c_text_data = style->text;
    friction_state[1].figure.details_b = (uint32_t)strlen((const char *)c_text_data);

    return 1U;
}

int friction_state_send(uint16_t sender_id,uint16_t receiver_id)
{
    if (!friction_state_config()) return -1;

    static uint8_t kind;

    app_ui_object_t *obj = &friction_state[kind % 2U];
    obj->figure.operate_type = obj->sent ? 2U : 1U; // 首次添加，后续修改。

    uint16_t payload_len;

    drv_referee_interaction_data_t inter_data = {
        .sender_id = sender_id,
        .receiver_id = receiver_id,
    };

    if (kind%2U == 0) {
        inter_data.data_cmd_id = 0x0101;
        memcpy(inter_data.user_data, &obj->figure, sizeof(obj->figure));
        payload_len = 6U + sizeof(obj->figure);
    }
   else {

       inter_data.data_cmd_id = 0x0110;
       memcpy(inter_data.user_data, &obj->figure, sizeof(obj->figure));
       memcpy(inter_data.user_data + sizeof(obj->figure),c_text_data ,30U);
       payload_len = 6U + sizeof(obj->figure) + 30U;
   }

    int result = drv_referee_port_send(CMD_ROBOT_INTERACTION,&inter_data, payload_len);

    if (result == 0) {
        obj->sent = 1U;
        kind++;
    }

    return result;

}

/**
 * @brief 热量上限矩形
 */
static app_ui_object_t s_heat_border = {
    .figure = {
        .figure_name = {'S', 'H', 'B'},
        .operate_type = 1U,
        .figure_type = 1U,
        .layer = 0U,
        .color = 8U,
        .width = 2U,             // 边框线宽；矩形高度由两个顶点的y坐标决定。
        .start_x = 350U,
        .start_y = 250U,
        .details_d = 650U,
        .details_e = 125U,
    },
    .sent = 0U,
};

/**
 * @brief 填充线信息
 */
static app_ui_object_t s_heat_fill = {
    .figure = {
        .figure_name = {'S', 'H', 'F'},
        .operate_type = 1U,
        .figure_type = 0U,
        .layer = 0U,
        .width = 100U,           // 填充线厚度，小于矩形高度125。
        .start_x = 355U,
        .start_y = 188U,         // 矩形竖直中心约为(125 + 250) / 2。
        .details_d = 355U,       // 终点x由后续热量配置函数按比例更新。
        .details_e = 188U,       // 终点y与起点相同，绘制水平线。
    },
    .sent = 0U,
};
#define FILL_LENGTH_MAX 290  //填充线最大长度

static app_ui_object_t s_heat_text = {
    .figure = {
        .figure_name = {'S', 'H', 'T'},
        .operate_type = 1U,
        .figure_type = 7U,
        .layer = 0U,
        .color = 8U,
        .details_a = 20U,
        .width = 2U,
        .start_x = 350U,         // HEAT 当前值 / 上限：放在矩形上方。
        .start_y = 280U,
    },
    .sent = 0U,
};
static char s_heat_text_data[30];

// 热量百分比文字：放在矩形右侧，例如60%。
static app_ui_object_t s_heat_percent_text = {
    .figure = {
        .figure_name = {'S', 'H', 'P'},
        .operate_type = 1U,
        .figure_type = 7U,
        .layer = 0U,
        .color = 8U,
        .details_a = 20U,
        .width = 2U,
        .start_x = 680U,
        .start_y = 200U,
    },
    .sent = 0U,
};
static char s_heat_percent_text_data[30];
static uint32_t percent;

/**
 * @brief 热量限制文字构建
 * @return 0成功 1失败
 */
static uint8_t s_heat_text_config()
{
    if (s_ui_data.heat_info.heat == NULL ||s_ui_data.heat_info.heat_limit == NULL) {return 1U;}

    uint16_t heat = *s_ui_data.heat_info.heat;
    uint16_t limit = *s_ui_data.heat_info.heat_limit;

    int len1 = snprintf(s_heat_text_data, sizeof(s_heat_text_data),"HEAT %u/%u",
    (unsigned int)heat,(unsigned int)limit);

    int len2 = 0;
    if (limit!=0) {
        percent =(uint32_t) ((heat*100U/ limit));
        len2 = snprintf(s_heat_percent_text_data, sizeof(s_heat_percent_text_data),"%u%%",(unsigned int)percent);
    }
    else{percent = 0U;len2 = snprintf(s_heat_percent_text_data, sizeof(s_heat_percent_text_data),"---%%");}

    if (len1 < 0 || (size_t)len1 >= sizeof(s_heat_text_data) || len2 < 0 ||
    (size_t)len2 >= sizeof(s_heat_percent_text_data)) {return 1U;}

    s_heat_text.figure.details_b = (uint32_t)len1;
    s_heat_percent_text.figure.details_b = (uint32_t)len2;

    return 0U;
}

/**
 * @brief 热量限制填充线构建
 * @param s_heat_text_config 热量限制文字构建
 * @return 0成功 1失败
 */
static uint8_t s_heat_fill_config(uint8_t s_heat_text_config)
{
   if (s_heat_text_config){return 1U;}

    uint32_t fill_percent = percent;
    if (fill_percent > 100U) {fill_percent = 100U;}

    uint16_t length = (FILL_LENGTH_MAX*fill_percent)/100U;

    s_heat_fill.figure.details_d = 355 + length;

    return 0U;
}

int heat_info_send(uint16_t sender_id,uint16_t receiver_id)
{
    if (s_heat_fill_config(s_heat_text_config())) {return -1;}

    // text为NULL表示普通图形，否则附带30字节文字。
    static const struct {
        app_ui_object_t *obj;
        const char *text;
    } items[] = {
        {&s_heat_border, NULL},
        {&s_heat_fill, NULL},
        {&s_heat_text, s_heat_text_data},
        {&s_heat_percent_text, s_heat_percent_text_data},
    };
    static uint8_t kind;
    app_ui_object_t *obj = items[kind].obj;
    const char *text = items[kind].text;

    obj->figure.operate_type = obj->sent ? 2U : 1U; // 首次添加，后续修改。

    drv_referee_interaction_data_t inter_data = {
        .data_cmd_id = text != NULL ? 0x0110U : 0x0101U,
        .sender_id = sender_id,
        .receiver_id = receiver_id,
    };
    memcpy(inter_data.user_data, &obj->figure, sizeof(obj->figure));
    uint16_t payload_len = 6U + sizeof(obj->figure);

    if (text != NULL) {
        memcpy(inter_data.user_data + sizeof(obj->figure), text, 30U);
        payload_len += 30U;
    }

    int result = drv_referee_port_send(CMD_ROBOT_INTERACTION, &inter_data, payload_len);
    if (result == 0) {
        obj->sent = 1U;
        kind++;
        // 边框只发送一次，之后循环更新填充线和两段文字。
        if (kind >= sizeof(items) / sizeof(items[0])) {
            kind = 1U;
        }
    }

    return result;
}

//------------------------------------------------------------------------------------------
/**
 *调用生成ui所使用的代码
 */
//------------------------------------------------------------------------------------------

/**
 * @brief 配置生成代码中的摩擦轮圆形和状态文字
 * @return 0配置失败，1配置成功
 */
static uint8_t friction_state_generated_config(void)
{
    if (s_ui_data.friction_state == NULL) {return 0U;}

    uint8_t state = *s_ui_data.friction_state;
    const size_t style_count = sizeof(s_friction_styles) / sizeof(s_friction_styles[0]);
    if (state >= style_count) {state = (uint8_t)(style_count - 1U);}

    const friction_state_style_t *style = &s_friction_styles[state];
    ui_interface_string_t *text = ui_g_Ungroup_friction_state_Text;

    ui_g_Ungroup_NewRound->color = style->color;
    text->color = style->color;
    memcpy(text->string, style->text, sizeof(text->string));
    text->str_length = (uint32_t)strlen(text->string);

    return 1U;
}

// 设计器中的填充线起点x为424，满热终点x为547。
#define GENERATED_HEAT_FILL_LENGTH_MAX 123U

/**
 * @brief 配置生成代码中的热量文字和填充线
 * @return 0配置失败，1配置成功
 */
static uint8_t heat_info_generated_config(void)
{
    if (s_ui_data.heat_info.heat == NULL || s_ui_data.heat_info.heat_limit == NULL) {return 0U;}

    uint16_t heat = *s_ui_data.heat_info.heat;
    uint16_t limit = *s_ui_data.heat_info.heat_limit;
    ui_interface_string_t *text = ui_g_Ungroup_HEAT_Number_ext;

    char text_data[30] = {0};
    int len = snprintf(text_data, sizeof(text_data), "%u/%u",
                       (unsigned int)heat, (unsigned int)limit);
    if (len < 0 || (size_t)len >= sizeof(text_data)) {return 0U;}

    memcpy(text->string, text_data, sizeof(text->string));
    text->str_length = (uint32_t)len;

    uint32_t fill_percent = 0U;
    if (limit != 0U) {fill_percent = (uint32_t)heat * 100U / limit;}
    if (fill_percent > 100U) {fill_percent = 100U;}

    // 30%为绿色，达到60%变黄，达到90%使用己方主色（红方为红色）。
    if (fill_percent >= 90U) {
        ui_g_Ungroup_NewLine->color = 0U;
    } else if (fill_percent >= 60U) {
        ui_g_Ungroup_NewLine->color = 1U;
    } else {
        ui_g_Ungroup_NewLine->color = 2U;
    }

    uint32_t length = GENERATED_HEAT_FILL_LENGTH_MAX * fill_percent / 100U;
    ui_g_Ungroup_NewLine->end_x = ui_g_Ungroup_NewLine->start_x + length;

    return 1U;
}

/**
 * @brief 配置生成代码中的俯仰角文字，固定白色
 * @return 0配置失败，1配置成功
 */
static uint8_t pitch_generated_config(void)
{
    if (s_ui_data.pitch_deg == NULL) {return 0U;}

    float pitch = *s_ui_data.pitch_deg;
    ui_interface_string_t *text = ui_g_Ungroup_pitch_Text;

    char text_data[30] = {0};
    int len = snprintf(text_data, sizeof(text_data), "PITCH: %.1f", (double)pitch);
    if (len < 0 || (size_t)len >= sizeof(text_data)) {return 0U;}

    text->color = 8U;
    memcpy(text->string, text_data, sizeof(text->string));
    text->str_length = (uint32_t)len;

    return 1U;
}

void app_Armored_ui_process(void)
{

    static uint32_t s_ui_last_tick;
    uint32_t now = bsp_tim_get_tick_ms();

    if ((uint32_t)(now - s_ui_last_tick) >= 40U) {
        s_ui_last_tick = now;
        if (s_ui_data.initial == 0U) {return;}

            friction_state_generated_config();
            heat_info_generated_config();
            pitch_generated_config();

            ui_update_g();
        }

}

struct {
    uint8_t friction_state;
    uint16_t heat;
    uint16_t heat_limit;
    uint16_t projectile_allowance; // 42mm允许发弹量
    float pitch_deg;             // 云台俯仰角，单位度
    uint8_t control_mode;
}fake_data = {
    .friction_state = 1U,
    .heat = 45U,
    .heat_limit = 100U,
    .pitch_deg = 20.0,
    .control_mode = 0,
    .projectile_allowance = 0,
};

void app_control_ui_init(void)
{
    app_Armored_ui_init(&fake_data.friction_state,&fake_data.heat,&fake_data.heat_limit,
            &fake_data.projectile_allowance,&fake_data.pitch_deg,&fake_data.control_mode);

}