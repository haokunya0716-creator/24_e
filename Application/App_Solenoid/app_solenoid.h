//
// Created by gaoxu on 2026/7/12.
//

#ifndef QD4310_APP_SOLENOID_H
#define QD4310_APP_SOLENOID_H

#include "stm32f4xx_hal.h"
void App_Solenoid_Init(void);
void App_Solenoid_Set_PWM(uint64_t duty);

#endif //QD4310_APP_SOLENOID_H