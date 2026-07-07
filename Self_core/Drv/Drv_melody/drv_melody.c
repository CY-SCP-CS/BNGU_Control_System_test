/**
 * @file    drv_melody.c
 * @brief   蜂鸣器乐谱播放器实现
 */
#include "drv_melody.h"

#include "drv_buzzer.h"

// 鈹€鈹€鈹€ 绉佹湁绫诲瀷 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

// 鈹€鈹€鈹€ 绉佹湁瀹?鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€



// 鈹€鈹€鈹€ 绉佹湁鍙橀噺 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

static const drv_melody_note_t *s_melody_notes;
static uint16_t                 s_melody_index;
static uint16_t                 s_melody_elapsed_ms;
static uint8_t                  s_melody_loop;
static volatile drv_melody_state_t s_melody_state;

// 鈹€鈹€鈹€ 绉佹湁鍑芥暟澹版槑 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

static void melody_set_note(uint16_t freq_hz);

// 鈹€鈹€鈹€ 鎺ュ彛瀹炵幇 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

void drv_melody_init(void)
{
    s_melody_notes      = NULL;
    s_melody_index      = 0;
    s_melody_elapsed_ms = 0;
    s_melody_loop       = 0;
    s_melody_state      = DRV_MELODY_STATE_IDLE;
}

void drv_melody_play(const drv_melody_note_t *notes, uint8_t loop)
{
    if (!notes) return;

    s_melody_notes      = notes;
    s_melody_index      = 0;
    s_melody_elapsed_ms = 0;
    s_melody_loop       = loop;
    s_melody_state      = DRV_MELODY_STATE_PLAYING;

    /* 绔嬪嵆鎾斁绗竴涓煶绗?*/
    melody_set_note(notes[0].freq_hz);
}

void drv_melody_stop(void)
{
    drv_buzzer_off();
    s_melody_state = DRV_MELODY_STATE_IDLE;
}

void drv_melody_pause(void)
{
    if (s_melody_state == DRV_MELODY_STATE_PLAYING) {
        drv_buzzer_off();
        s_melody_state = DRV_MELODY_STATE_PAUSED;
    }
}

void drv_melody_resume(void)
{
    if (s_melody_state == DRV_MELODY_STATE_PAUSED) {
        s_melody_state = DRV_MELODY_STATE_PLAYING;
        melody_set_note(s_melody_notes[s_melody_index].freq_hz);
    }
}

void drv_melody_update(void)
{
    const drv_melody_note_t *note;

    if (s_melody_state != DRV_MELODY_STATE_PLAYING) return;
    if (!s_melody_notes) return;

    note = &s_melody_notes[s_melody_index];

    /* 鏈熬鍝ㄥ叺 {0, 0} 鈫?缁撴潫鎴栧惊鐜?*/
    if (note->freq_hz == 0 && note->duration_ms == 0) {
        if (s_melody_loop) {
            s_melody_index      = 0;
            s_melody_elapsed_ms = 0;
            melody_set_note(s_melody_notes[0].freq_hz);
            return;
        }
        drv_buzzer_off();
        s_melody_state = DRV_MELODY_STATE_DONE;
        return;
    }

    s_melody_elapsed_ms++;

    /* 褰撳墠闊崇鍒版椂 鈫?鍒囦笅涓€涓?*/
    if (s_melody_elapsed_ms >= note->duration_ms) {
        s_melody_elapsed_ms = 0;
        s_melody_index++;
        melody_set_note(s_melody_notes[s_melody_index].freq_hz);
    }
}

drv_melody_state_t drv_melody_get_state(void)
{
    return s_melody_state;
}

// 鈹€鈹€鈹€ 绉佹湁鍑芥暟瀹氫箟 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

static void melody_set_note(uint16_t freq_hz)
{
    if (freq_hz == NOTE_REST) {
        drv_buzzer_off();
    } else {
        drv_buzzer_set_freq(freq_hz);
        drv_buzzer_on();
    }
}


