/**
 * @file    drv_melody.h
 * @brief   蜂鸣器乐谱播放 — 音符定义 + 播放器状态机
 * @note    依赖 drv_buzzer 提供 PWM 调频和开关
 */
#ifndef DRV_MELODY_H
#define DRV_MELODY_H

#include "lib_typedef.h"

// ─── 音符频率 (Hz) — 标准 12 平均律 ──────────────

#define NOTE_C4     262
#define NOTE_CS4    277
#define NOTE_D4     294
#define NOTE_DS4    311
#define NOTE_E4     330
#define NOTE_F4     349
#define NOTE_FS4    370
#define NOTE_G4     392
#define NOTE_GS4    415
#define NOTE_A4     440
#define NOTE_AS4    466
#define NOTE_B4     494
#define NOTE_C5     523
#define NOTE_CS5    554
#define NOTE_D5     587
#define NOTE_DS5    622
#define NOTE_E5     659
#define NOTE_F5     698
#define NOTE_FS5    740
#define NOTE_G5     784
#define NOTE_GS5    831
#define NOTE_A5     880
#define NOTE_AS5    932
#define NOTE_B5     988
#define NOTE_C6     1047
#define NOTE_REST   0       /* 休止符 */

// ─── 乐谱条目 ────────────────────────────────────

typedef struct {
    uint16_t freq_hz;       /* 频率, NOTE_REST = 休止 */
    uint16_t duration_ms;   /* 持续时长 (ms)          */
} drv_melody_note_t;

// ─── 播放器状态 ──────────────────────────────────

typedef enum {
    DRV_MELODY_STATE_IDLE = 0,
    DRV_MELODY_STATE_PLAYING,
    DRV_MELODY_STATE_PAUSED,
    DRV_MELODY_STATE_DONE,
} drv_melody_state_t;

// ─── 接口 ────────────────────────────────────────

void drv_melody_init(void);

/**
 * @brief  开始播放乐谱
 * @param  notes     乐谱数组 (以 {0, 0} 结尾)
 * @param  loop      0=单次, 1=循环
 */
void drv_melody_play(const drv_melody_note_t *notes, uint8_t loop);

void drv_melody_stop(void);
void drv_melody_pause(void);
void drv_melody_resume(void);

/**
 * @brief  播放器 tick (需周期性调用, 建议 1ms)
 * @note   内部自动切换音符、启停蜂鸣器
 */
void drv_melody_update(void);

drv_melody_state_t drv_melody_get_state(void);

#endif
