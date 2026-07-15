//
// Created by gaoxu on 2026/4/5.
//

#include "app_QD4310_PID.h"
#include <math.h>

#include "app_usart.h"
#include "QD4310.h"
#include "task.h"

uint8_t valid = 0;

float Kp_pitch = 0;
float Ki_pitch = 0;
float Kd_pitch = 0;
float Kf_pitch = 0; // 【新增】Pitch前馈系数
float speed_pitch_f = 0;

float Kp_yaw = 0;
float Ki_yaw = 0;
float Kd_yaw = 0;

static float yaw_speed = 0;
static float pitch_speed = 0;

static PID_TypeDef pid_motor_pitch;
static PID_TypeDef pid_motor_yaw;



extern volatile uint8_t buzzer_flag;
extern uint32_t buzzer_time;


void QD4310_PID_Reset(void) {
    PID_Reset(&pid_motor_pitch);
    PID_Reset(&pid_motor_yaw);
}

void QD4310_PID_Init(void) {
    QD4310_Init(&YawMotor, &hcan1, 0x00);
    QD4310_Init(&PitchMotor, &hcan1, 0x01);
    HAL_Delay(100);
    QD4310_Enable(&YawMotor);//默认为使能状态
    QD4310_Enable(&PitchMotor);
    HAL_Delay(20);

    PID_Init(&pid_motor_pitch, Kp_pitch, Ki_pitch, Kd_pitch);
    PID_LimitConfig(&pid_motor_pitch, +120.0f, -120.0f);
    PID_Init(&pid_motor_yaw, Kp_yaw, Ki_yaw, Kd_yaw); // 【修改】传入Kd_yaw
    PID_LimitConfig(&pid_motor_yaw, +120.0f, -120.0f);


}

// ============================
// ============================
void QD4310_PID_Pro(void) {
    // 增加一个静态变量，记录是否已经叫过
    static uint8_t has_beeped_pro = 0;

    PERIODIC_START(QD4310_PID,2)
        buzzer_flag = 0;
        has_beeped_pro = 0; // 正常追踪状态，复位锁
        yaw_speed = PID_Compute(&pid_motor_yaw, 0);
        QD4310_SetSpeed(&YawMotor,yaw_speed);
        pitch_speed = PID_Compute(&pid_motor_pitch,0);
        QD4310_SetSpeed(&PitchMotor,pitch_speed);

    PERIODIC_END
}
