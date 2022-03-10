#pragma once

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ARR_VAL 65535

    typedef enum
    {
        RED = 3,
        GREEN = 1,
        BLUE = 2
    } LED_COLOR;

    void set_led_intensity(uint8_t color, float intensity);
    void start_timers();

#ifdef __cplusplus
}
#endif