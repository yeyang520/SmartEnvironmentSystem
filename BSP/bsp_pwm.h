#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

// 启动pwm输出
void BSP_PWM_Init(void);

// 动态修改pwm占空比
void BSP_PWM_SetDuty(uint16_t duty);

#endif