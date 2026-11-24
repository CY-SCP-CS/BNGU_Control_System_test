//
// Created by 33140 on 2026/11/7.
//

#include "Drv/Drv_game_controller/drv_game_controller.h"
#include <stdio.h>
#include <math.h>

uint8_t drv_game_controller_decode(const uint8_t *buffer, drv_game_controller_buttons_t *data)
{
    if (!buffer || !data) {
        return 0U;
    }

    drv_game_controller_buttons_t decoded = {0};
    uint8_t *const buttons[] = {
        &decoded.lb, &decoded.rb,
        &decoded.a, &decoded.b, &decoded.x, &decoded.y,
        &decoded.up, &decoded.down, &decoded.left, &decoded.right,
        &decoded.view, &decoded.menu, &decoded.share, &decoded.xbox
    };
    char raw_buttons[sizeof(buttons) / sizeof(buttons[0])];
    char trailing;

    /* 最后的 %c 如果读到数据，说明第 20 个字段之后还有内容。 */
    int fields = sscanf((const char *)buffer,
                        "JOY,%f,%f,%f,%f"
                        ",%f,%c,%f,%c"
                        ",%c,%c,%c,%c"
                        ",%c,%c,%c,%c"
                        ",%c,%c,%c,%c%c",
                        &decoded.lx, &decoded.ly, &decoded.rx, &decoded.ry,
                        &decoded.lt, &raw_buttons[0], &decoded.rt, &raw_buttons[1],
                        &raw_buttons[2], &raw_buttons[3], &raw_buttons[4], &raw_buttons[5],
                        &raw_buttons[6], &raw_buttons[7], &raw_buttons[8], &raw_buttons[9],
                        &raw_buttons[10], &raw_buttons[11], &raw_buttons[12], &raw_buttons[13],
                        &trailing);
    if (fields != 20) {
        return 0U;
    }

    if (!isfinite(decoded.lx) || !isfinite(decoded.ly) ||
        !isfinite(decoded.rx) || !isfinite(decoded.ry) ||fabsf(decoded.lx) > 1.0f || fabsf(decoded.ly) > 1.0f ||
        fabsf(decoded.rx) > 1.0f || fabsf(decoded.ry) > 1.0f ||!isfinite(decoded.lt) || !isfinite(decoded.rt)) {
        return 0U;
    }

    for (size_t i = 0U; i < sizeof(raw_buttons); ++i) {
        if (raw_buttons[i] != '0' && raw_buttons[i] != '1') {
            return 0U;
        }
        *buttons[i] = (uint8_t)(raw_buttons[i] - '0');
    }

    *data = decoded;
    return 1U;
}
