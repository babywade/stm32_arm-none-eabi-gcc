#include "foc.h"

void pwm_test() {
    HAL_TIM_PWM_START(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(htim1, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 400);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 5600);
}