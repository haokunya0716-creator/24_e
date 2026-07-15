//
// Created by gaoxu on 2026/4/5.
//

#ifndef QD4310_APP_QD4310_PID_H
#define QD4310_APP_QD4310_PID_H

#include "main.h"
#include "pid.h"
#include "delay.h"

/* ================== 全局变量声明区 ================== */
// 【修改】将PID参数和前馈系数暴露出来，方便你用按键、串口或调试器在线调参

extern float Kp_pitch;
extern float Ki_pitch;
extern float Kd_pitch;


/* ================== 函数声明区 ================== */
void QD4310_PID_Init(void);
void QD4310_PID_Pro(void);
void QD4310_PID_Reset(void);




#endif //QD4310_APP_QD4310_PID_H