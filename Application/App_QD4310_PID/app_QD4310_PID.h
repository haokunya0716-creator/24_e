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
extern uint8_t valid;

extern float Kp_pitch;
extern float Ki_pitch;
extern float Kd_pitch;
extern float Kf_pitch;      // 【新增】Pitch轴前馈系数
extern float speed_pitch_f; // Pitch轴前馈计算结果暂存

extern float Kp_yaw;
extern float Ki_yaw;
extern float Kd_yaw;
extern float Kf_yaw;        // 【新增】Yaw轴前馈系数
extern float speed_yaw_f;   // Yaw轴前馈计算结果暂存

extern volatile float laser_current_yaw;
extern volatile float laser_current_yaw_target;
extern volatile float laser_current_pitch;
extern volatile float laser_current_pitch_target;

extern uint8_t has_beeped_pro;

/* ================== 函数声明区 ================== */
void QD4310_PID_Init(void);
void QD4310_PID_Pro(void);
void QD4310_PID_Pro2(void);
void QD4310_PID_Pro_Extend(void);
void QD4310_PID_Pro_Extend5(void);
void QD4310_PID_Reset(void);
void QD4310_Vaild_Update(void);
float PID_Compute_YAW(PID_TypeDef *PID, float FB);
float PID_Compute_Pitch(PID_TypeDef *PID, float FB);
void QD4310_PID_Update_yaw(PID_TypeDef *PID);
void QD4310_PID_Update_pitch(PID_TypeDef *PID);



#endif //QD4310_APP_QD4310_PID_H