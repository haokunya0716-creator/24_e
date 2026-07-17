//
// Created by gaoxu on 2026/4/5.
//

#include "app_QD4310_PID.h"
#include <math.h>
#include "pid.h"

#include "app_usart.h"
#include "QD4310.h"
#include "task.h"

static float bigMotor_ref_angle = 0;
static float smallMotor_ref_angle = 0;


static PID_TypeDef smallMotor_pid;
static PID_TypeDef bigMotor_pid;

void QD4310_PID_Init(void) {
    QD4310_Init(&SmallMotor, &hcan1, 0x00);
    QD4310_Init(&BigMotor, &hcan1, 0x01);
    HAL_Delay(100);
    QD4310_Enable(&SmallMotor);//默认为使能状态
    QD4310_Enable(&BigMotor);
    HAL_Delay(20);
    PID_Init(&smallMotor_pid,0,0,0);
    PID_Init(&bigMotor_pid,0,0,0);
    PID_LimitConfig(&smallMotor_pid,+150,-150);
    PID_LimitConfig(&bigMotor_pid,+150,-150);
}

void AD4310_PID_Reset(void) {
    PID_Reset(&smallMotor_pid);
    PID_Reset(&bigMotor_pid);
}

void QD4310_PID_Pro(void) {
    float speed_big = PID_Compute(&bigMotor_pid, bigMotor_ref_angle);
    float speed_small = PID_Compute(&smallMotor_pid, smallMotor_ref_angle);

    QD4310_SetSpeed(&BigMotor, speed_big);
    QD4310_SetSpeed(&SmallMotor, speed_small);

}

//大臂：向上正，向下负
void Set_Big_Rad(float rad) {
    if (rad >= 0.0f) {
        bigMotor_ref_angle = rad;
    }else if (rad < 0.0f) {
        bigMotor_ref_angle = (rad + 6.283);
    }
}
//小臂：向外正，向内负
void Set_Small_Rad(float rad) {
    if (rad >= 0.0f) {
        smallMotor_ref_angle = rad;
    }else if (rad < 0.0f) {
        smallMotor_ref_angle = (rad + 6.283);
    }
}