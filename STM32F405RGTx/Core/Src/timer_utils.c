#include <stdint.h>
#include "main.h"
#include "timer_utils.h"

void set_led_intensity(LED_COLOR color, float intensity) {
    uint32_t ccr_val = (uint32_t)( ((100 - intensity)*htim3.Instance->ARR)/100 );
    switch (color) {
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
void start_rgb_pwm() {
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void start_motor_pwm() {
    HAL_TIM_Base_Start(&htim1);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}
void stop_motor_pwm() {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    HAL_TIM_Base_Stop(&htim1);
}


