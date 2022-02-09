#pragma once

#include <stdint.h>
#include "main.h"

#define ARR_VAL 65535

typedef enum
{
    RED = 3,
    GREEN = 1,
    BLUE = 2
} LED_COLOR;

void set_led_intensity(uint8_t color, float intensity);
void start_timers();
void delay_us(uint16_t us); // uses timer 2 as a 1us counter