#include "bsp_pwm.h"
#include "main.h"

#define PWM_MAX_DUTY 999


extern TIM_HandleTypeDef htim3;

void BSP_PWM_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    BSP_PWM_SetDuty(0);
}

void BSP_PWM_SetDuty(uint16_t duty)
{
    if (duty > PWM_MAX_DUTY)
    {
        duty = PWM_MAX_DUTY;
    }

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_1,
        duty
    );
}