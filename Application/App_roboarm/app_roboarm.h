//
// Created by gaoxu on 2026/7/14.
//

#ifndef QD4310_APP_ROBOARM_H
#define QD4310_APP_ROBOARM_H
#include "stm32f4xx_hal.h"

void Robo_Compute(float x,float y,float z);
void Robo_Set_YawDeg(float deg);
void Robo_Set_YawRad(float rad);
#endif //QD4310_APP_ROBOARM_H