//
// Created by gaoxu on 2026/7/12.
//

#include "app_task.h"

#include "app_button.h"
#include "app_QD4310_PID.h"
#include "app_solenoid.h"
#include "QD4310.h"
#include "tim.h"
#include "App_Emm_V5/app_Emm_V5.h"

void App_Task_Init(void) {
    CAN_InterfaceInit();
    QD4310_PID_Init();//初始化无刷电机
    App_Button_Init();//初始化按键
    App_Solenoid_Init();//初始化电磁铁，默认不开
    Emm_V5_PID_Init();//初始化步进电机
    HAL_TIM_Base_Start_IT(&htim12);//开启定时器12中断，用来扫描按键
    HAL_TIM_Base_Start_IT(&htim11);//开启定时器11中断，用来微秒级计时

    ///////////////////////开启三色灯pwm
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);//蓝
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);//绿
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);//红
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);//开启蜂鸣器pwm
    uint64_t arr3 = __HAL_TIM_GET_AUTORELOAD(&htim4);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (arr3+1)/5);//把蜂鸣器占空比设为0
    HAL_Delay(200);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);//把蜂鸣器占空比设为0
}
