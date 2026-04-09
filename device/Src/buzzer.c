#include "buzzer.h"
#include "tim.h"
#include "bsp_dwt.h"

void buzzer_beep_ms(uint8_t ms)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 2000);
    dwt_delay_ms(ms);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
}