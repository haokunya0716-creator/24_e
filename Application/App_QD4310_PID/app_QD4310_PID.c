//
// Created by gaoxu on 2026/4/5.
//

#include "app_QD4310_PID.h"
#include <math.h>

#include "app_usart.h"
#include "QD4310.h"
#include "task.h"

static float yaw_speed = 0;
static float pitch_speed = 0;

extern volatile uint8_t buzzer_flag;
extern uint32_t buzzer_time;


void QD4310_PID_Init(void) {
    QD4310_Init(&SmallMotor, &hcan1, 0x00);
    QD4310_Init(&BigMotor, &hcan1, 0x01);
    HAL_Delay(100);
    QD4310_Enable(&SmallMotor);//默认为使能状态
    QD4310_Enable(&BigMotor);
    HAL_Delay(20);
}
//大臂：向上正，向下负
void Set_Big_Rad(float rad) {
    if (rad >= 0.0f) {
        QD4310_SetAngle(&BigMotor, rad);
    }else if (rad < 0.0f) {
        QD4310_SetAngle(&BigMotor, (rad + 6.283));
    }
}
//小臂：向外正，向内负
void Set_Small_Rad(float rad) {
    if (rad >= 0.0f) {
        QD4310_SetAngle(&SmallMotor, rad);
    }else if (rad < 0.0f) {
        QD4310_SetAngle(&SmallMotor, (rad + 6.283));
    }
}
