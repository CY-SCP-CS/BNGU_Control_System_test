//
// Created by 33140 on 2026/11/7.
//

#include "Drv/Drv_game_controller/drv_game_controller.h"
#include <stdio.h>
#include <math.h>

uint8_t drv_game_controller_decode(const uint8_t *buffer,drv_game_controller_buttons_t *data)
{
    if (!buffer || !data) {return 0U;}

    float lx, ly, rx, ry;

    if (sscanf((const char *)buffer, "JOY,%f,%f,%f,%f",&lx, &ly, &rx, &ry) != 4) {return 0U;}

    if (!isfinite(lx) || !isfinite(ly) ||!isfinite(rx) || !isfinite(ry) ||
       fabsf(lx) > 1.0f || fabsf(ly) > 1.0f ||fabsf(rx) > 1.0f || fabsf(ry) > 1.0f) {return 0U;}

    data->lx = lx;
    data->ly = ly;
    data->rx = rx;
    data->ry = ry;

    return 1U;
}