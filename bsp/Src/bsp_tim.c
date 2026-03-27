#include "tim.h"

void tim_init(void)
{
    // enable tim5, tim12, tim15
    HAL_TIM_Base_Start_IT(&htim5);  // neck control, 1000hz
    HAL_TIM_Base_Start_IT(&htim12); // head control, 125hz
    HAL_TIM_Base_Start_IT(&htim15); // chassis control, 125hz
}