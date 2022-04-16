#pragma once

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BLUE    = 3,
    GREEN   = 2,
    RED     = 1
} LED_COLOR;

void set_LED_intensity(LED_COLOR colour, float intensity);
void start_rgb_pwm();

void start_motor_pwm();
void stop_motor_pwm();


#ifdef __cplusplus
}
#endif
