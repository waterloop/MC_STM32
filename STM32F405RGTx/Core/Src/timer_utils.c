#include <stdint.h>
#include "main.h"
#include "timer_utils.h"

void set_led_intensity(uint8_t color, float intensity)
{
    uint32_t ccr_val = (uint32_t)(((100 - intensity) * ARR_VAL) / 100);
    switch (color)
    {
    case 1:
        htim3.Instance->CCR1 = ccr_val;
        break;
    case 2:
        htim3.Instance->CCR2 = ccr_val;
        break;        
    case 3:
        htim3.Instance->CCR3 = ccr_val;
        break;
    }
}

void start_timers()
{
    HAL_TIM_Base_Start(&htim3);

    set_led_intensity(RED, 0);
    set_led_intensity(GREEN, 0);
    set_led_intensity(BLUE, 0);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}
