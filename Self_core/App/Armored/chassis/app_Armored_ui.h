//
// Created by 33140 on 2026/10/13.
//

#ifndef BNGU_CONTROL_SYSTEM_APP_ARMORED_UI_H
#define BNGU_CONTROL_SYSTEM_APP_ARMORED_UI_H

#include <stdint.h>

typedef struct {
    uint8_t* friction_state;      //  摩擦轮状态 0=关闭，1=加速，2=就绪
    struct {
        uint16_t* heat;               // 当前42mm枪口热量
        uint16_t* heat_limit;          // 热量上限
    }heat_info;
    uint16_t* projectile_allowance; // 42mm允许发弹量
    float* pitch_deg;             // 云台俯仰角，单位度
    uint8_t* control_mode;        // 与控制模块约定的模式编号

    uint8_t initial;      //是否初始化完成 0未完成 1已完成
} app_Armored_ui_data_t;  // 要画UI的数据



/**
 * @brief UI绘图初始化 绑定UI数据源
 * @param friction_state 摩擦轮状态地址
 * @param heat 当前42mm枪口热量地址
 * @param heat_limit 热量上限地址
 * @param projectile_allowance 42mm允许发弹量地址
 * @param pitch_deg 云台俯仰角地址（度）
 * @param control_mode 控制模式地址
 * @note 传入变量须在UI使用期间保持有效。
 */
void app_Armored_ui_init(uint8_t* friction_state , uint16_t* heat , uint16_t* heat_limit ,
                  uint16_t* projectile_allowance, float* pitch_deg, uint8_t* control_mode);

/**
 * @brief 摩擦轮状态UI发送
 * @param sender_id 本机器人的裁判系统ID
 * @param receiver_id 对应选手端ID
 * @return 0=成功启动发送，1=发送忙，-1=配置或发送启动失败
 */
int friction_state_send(uint16_t sender_id,uint16_t receiver_id);

/**
 * @brief 热量UI发送，每次发送一个图形或一段文字
 * @param sender_id 本机器人的裁判系统ID
 * @param receiver_id 对应选手端ID
 * @return 0=成功启动发送，1=发送忙，-1=配置或发送启动失败
 */
int heat_info_send(uint16_t sender_id,uint16_t receiver_id);

/**
 * @brief 放入app_control中主循环调用的ui更新函数
 * @note 现在分析因为ui_scan_and_send没有检查串口中断发送的返回值可能会出现丢包的情况
 */
void app_Armored_ui_process(void);

/**
 * @brief 临时调试用的
 */
void app_control_ui_init(void);
#endif // BNGU_CONTROL_SYSTEM_APP_ARMORED_UI_H
