//
// Created by gaoxu on 2026/7/12.
//

#include "app_solenoid.h"

#include "stm32f4xx_hal_tim.h"
#include "tim.h"
static uint64_t arr_tim1 = 0;
//
//@brief:初始化电磁铁
//
void App_Solenoid_Init(void) {
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//开启pwm
    arr_tim1 = __HAL_TIM_GET_AUTORELOAD(&htim1);//读取定时器1的arr
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);//把通道一的占空比先设为0
}
//
//@brief:设置占空比
//
void App_Solenoid_Set_PWM(uint64_t duty) {
    if (duty > 100) { duty = 100; }
    if (duty < 0) { duty = 0; }
    uint64_t ccr = duty * (arr_tim1 + 1) / 100.0f;
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,ccr);
}
