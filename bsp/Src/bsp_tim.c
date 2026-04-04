#include "tim.h"

void tim_init(void)
{
    /* enable tim2, tim5, tim12, tim15 */
    HAL_TIM_Base_Start_IT(&htim2); /* for chores, like ui and vision */
    HAL_TIM_Base_Start_IT(&htim5);  /* for gimbal task, 1000 hz */
    HAL_TIM_Base_Start_IT(&htim12); /* for ammo task, 125 hz */
    HAL_TIM_Base_Start_IT(&htim15); /* for chassis task, 125 hz */
}