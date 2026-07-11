//
// Created by gaoxu on 2026/4/5.
//

#include "app_QD4310_PID.h"
#include <math.h>

#include "app_usart.h"
#include "irqHandlers.h"
#include "task.h"
#include "QD4310.h"
#include "vision_protocol.h"
#include "App_Kalman/app_kalman.h" // 【新增】引入你刚刚创建的卡尔曼头文件

// 【新增】定义全局的卡尔曼滤波器实例
KalmanFilter_t kf_yaw;
KalmanFilter_t kf_pitch;

// 视觉测速全局变量
volatile float vision_vx = 0.0f; // yaw轴目标运动速度 (像素/秒)
volatile float vision_vy = 0.0f; // pitch轴目标运动速度 (像素/秒)

// 视觉更新周期 (根据你的相机帧率决定，比如 40Hz 就是 25ms)
#define VISION_FRAME_MS 30

uint8_t valid = 0;

float Kp_pitch = 0;
float Ki_pitch = 0;
float Kd_pitch = 0;
float Kf_pitch = 0; // 【新增】Pitch前馈系数
float speed_pitch_f = 0;

float Kp_yaw = 0;
float Ki_yaw = 0;
float Kd_yaw = 0;
float Kf_yaw = 0;   // 【新增】Yaw前馈系数
float speed_yaw_f = 0; // Yaw前馈计算

float __Kp_yaw = 0;
float __Ki_yaw = 0;
float __Kd_yaw = 0;
float __Kf_yaw = 0;
float __Kp_pitch = 0;
float __Ki_pitch = 0;
float __Kd_pitch = 0;
float __Kf_pitch = 0;

volatile float laser_current_yaw = 0;
volatile float laser_current_yaw_target = 0;
volatile float laser_current_pitch = 0;
volatile float laser_current_pitch_target = 0;

static float yaw_speed = 0;
static float pitch_speed = 0;

static PID_TypeDef pid_motor_pitch;
static PID_TypeDef pid_motor_yaw;

static PID_TypeDef __pid_motor_pitch;
static PID_TypeDef __pid_motor_yaw;

static PID_TypeDef __pid_motor_pitch5;
static PID_TypeDef __pid_motor_yaw5;

extern volatile uint8_t buzzer_flag;
extern uint32_t buzzer_time;


void QD4310_PID_Reset(void) {
    PID_Reset(&pid_motor_pitch);
    PID_Reset(&pid_motor_yaw);
    PID_Reset(&__pid_motor_yaw);
    PID_Reset(&__pid_motor_pitch);

    // 【新增】复位卡尔曼滤波器
    Kalman_Reset(&kf_yaw);
    Kalman_Reset(&kf_pitch);
}

void QD4310_PID_Init(void) {
    CAN_InterfaceInit();
    QD4310_Init(&YawMotor, &hcan1, 0x00);
    QD4310_Init(&PitchMotor, &hcan1, 0x01);
    HAL_Delay(100);
    // QD4310_Enable(&YawMotor);
    // QD4310_Enable(&PitchMotor);
    HAL_Delay(20);

    // 【新增】初始化卡尔曼滤波器 (参数可自行微调)
    Kalman_Init(&kf_yaw,   0.005f, 5.0f, 2.0f);
    Kalman_Init(&kf_pitch, 0.005f, 5.0f, 2.0f);

    // 【调参提示】加入滤波后：
    // 1. Kp可以大胆给大(响应加快)
    // 2. 必须给Kd(抑制超调震荡)
    // 3. Kf前馈系数根据测试逐渐增加
    Kp_yaw = 0.09;
    Ki_yaw = 0.0;
    Kd_yaw = 0.00; // 尝试给一点Kd作为刹车
    Kf_yaw = 0.0;  // 刚开始先设为0，等PD调稳了再加前馈

    Kp_pitch = 0.08;
    Ki_pitch = 0.0;
    Kd_pitch = 0.0;
    Kf_pitch = 0.0;

    //适合普通PID的参数
    __Kp_yaw = 0.20;
    __Ki_yaw = 1;
    __Kd_yaw = 0.0041; // 尝试给一点Kd作为刹车
    __Kf_yaw = 0.0;  // 刚开始先设为0，等PD调稳了再加前馈

    __Kp_pitch = 0.23;
    __Ki_pitch = 1.2;
    __Kd_pitch = 0.005;
    __Kf_pitch = 0.0;

    //适合卡尔曼滤波的参数
    // __Kp_yaw = 0.20;
    // __Ki_yaw = 0.00;
    // __Kd_yaw = 0.0128; // 尝试给一点Kd作为刹车
    // __Kf_yaw = 0.0;  // 刚开始先设为0，等PD调稳了再加前馈
    //
    // __Kp_pitch = 0.23;
    // __Ki_pitch = 0.0;
    // __Kd_pitch = 0.01;
    // __Kf_pitch = 0.0;
    //任务1，2，3的
    PID_Init(&pid_motor_pitch, Kp_pitch, Ki_pitch, Kd_pitch);
    PID_LimitConfig(&pid_motor_pitch, +120.0f, -120.0f);
    PID_Init(&pid_motor_yaw, Kp_yaw, Ki_yaw, Kd_yaw); // 【修改】传入Kd_yaw
    PID_LimitConfig(&pid_motor_yaw, +120.0f, -120.0f);
    //下面是任务四
    PID_Init(&__pid_motor_pitch, __Kp_pitch, __Ki_pitch, __Kd_pitch);
    PID_LimitConfig(&__pid_motor_pitch, +120.0f, -120.0f);
    PID_Init(&__pid_motor_yaw, __Kp_yaw, __Ki_yaw, __Kd_yaw); // 【修改】传入Kd_yaw
    PID_LimitConfig(&__pid_motor_yaw, +120.0f, -120.0f);
    //下面是任务五
    PID_Init(&__pid_motor_pitch5, 0.23f,1.2f, 0.005f);
    PID_LimitConfig(&__pid_motor_pitch5, +120.0f, -120.0f);
    PID_Init(&__pid_motor_yaw5, 0.2f, 1.0f, 0.0041f); // 【修改】传入Kd_yaw
    PID_LimitConfig(&__pid_motor_yaw5, +120.0f, -120.0f);

}
// 建议定义在全局或静态
static float last_vision_dx = 0.0f;
static float last_vision_dy = 0.0f;
static uint64_t last_vision_time = 0;
static uint8_t is_first_frame = 1; // 记录是否是丢失后第一帧

/**
 * @brief  视觉目标运动速度估算函数 (改进版)
 */
void Vision_Velocity_Update_Improved(void) {
    // 如果目标丢失，清空速度缓存并重置首帧标志
    if (valid == 0) {
        vision_vx = 0.0f;
        vision_vy = 0.0f;
        is_first_frame = 1;
        return;
    }

    uint64_t current_time = gx_GetUs();
    // 计算两次有效视觉帧之间的时间差 (秒)
    float dt = (float)(current_time - last_vision_time) / 1000000.0f;

    // 只有当时间跨度接近相机帧周期(>25ms)时，才认为视觉更新了一帧
    if (dt >= 0.030f) {
        __disable_irq(); // 开启原子操作，防止读取 vision 数据时被串口中断打断
        float current_dx = g_vision.dx;
        float current_dy = g_vision.dy;
        __enable_irq();

        if (is_first_frame) {
            is_first_frame = 0;
        } else {
            float raw_vx = (current_dx - last_vision_dx) / dt;
            float raw_vy = (current_dy - last_vision_dy) / dt;

            float alpha = 0.35f;
            vision_vx = vision_vx * (1.0f - alpha) + raw_vx * alpha;
            vision_vy = vision_vy * (1.0f - alpha) + raw_vy * alpha;
        }

        // 记录历史值用于下次差分
        last_vision_dx = current_dx;
        last_vision_dy = current_dy;
        last_vision_time = current_time;
    }
}

// 数据更新函数
void QD4310_Vaild_Update(void) {
    __disable_irq();
    valid = g_vision.valid;//有效值，0代表相机未识别到图形
    __enable_irq();
}

void QD4310_PID_Update_pitch(PID_TypeDef *PID) {
    __disable_irq();
    laser_current_pitch = g_vision.dy;
    __enable_irq();

    laser_current_pitch_target = 0.0f;

    PID->SP = laser_current_pitch_target;
}

void QD4310_PID_Update_yaw(PID_TypeDef *PID) {
    __disable_irq();
    laser_current_yaw = g_vision.dx;
    __enable_irq();
    laser_current_yaw_target = 0.0f;//视觉发送的坐标

    PID->SP = laser_current_yaw_target;
}



// ============== 任务 1,2,3 保持你的原始逻辑完全不变 ==============
// ============== 任务 1,2,3 保持你的原始逻辑完全不变 ==============
void QD4310_PID_Pro(void) {
    // 增加一个静态变量，记录是否已经叫过
    static uint8_t has_beeped_pro = 0;

    PERIODIC(2)
    QD4310_Vaild_Update();
    QD4310_PID_Update_yaw(&pid_motor_yaw);
    QD4310_PID_Update_pitch(&pid_motor_pitch);

    // ⚠️删除原代码这里的 buzzer_flag = 0;
    // buzzer_flag = 0;

    //再进行pid计算
    App_USART6_Printf("%d\n",valid);
    if (valid == 0) {
        buzzer_flag = 0;
        has_beeped_pro = 0; // 目标丢失，复位锁
        QD4310_SetSpeed(&YawMotor,0);
        QD4310_SetSpeed(&PitchMotor,0);
        PID_Reset(&pid_motor_yaw);
        PID_Reset(&pid_motor_pitch);
    }else if (valid == 1){
        buzzer_flag = 0;
        has_beeped_pro = 0; // 正常追踪状态，复位锁
        yaw_speed = PID_Compute(&pid_motor_yaw, laser_current_yaw);
        QD4310_SetSpeed(&YawMotor,yaw_speed);
        pitch_speed = PID_Compute(&pid_motor_pitch, laser_current_pitch);
        QD4310_SetSpeed(&PitchMotor,pitch_speed);
    }else if (valid == 2){
        buzzer_flag = 1;
        // // 目标满足(valid==2)时，判断是否还没叫过
        // if (has_beeped_pro == 0) {
        //     buzzer_flag = 1;    // 触发 main.c 响铃
        //     has_beeped_pro = 1; // 叫过了，上锁（直到变成0或1才会解锁）
        // }
    }
}

void QD4310_PID_Pro2(void) {
    QD4310_Vaild_Update();
    QD4310_PID_Update_yaw(&pid_motor_yaw);
    QD4310_PID_Update_pitch(&pid_motor_pitch);

    // 激光在目标内
    if (valid == 1 && fabsf(laser_current_yaw) <= 5 && fabsf(laser_current_pitch) <= 5) {
        if (buzzer_flag == 0) {
            buzzer_flag = 1;
            buzzer_time = HAL_GetTick(); // 锁定进入时刻
        }
    }
    else {
        // 激光离开了目标：如果还没响完，就复位，允许下次重新计时
        if (buzzer_flag == 1 && (HAL_GetTick() - buzzer_time < 1900)) {
            buzzer_flag = 0;
        }
    }

    //再进行pid计算
    if (valid == 0) {
        QD4310_SetSpeed(&YawMotor,0);
        QD4310_SetSpeed(&PitchMotor,0);
        PID_Reset(&pid_motor_yaw);
        PID_Reset(&pid_motor_pitch);
    }else if (valid == 1){
        yaw_speed = PID_Compute_YAW(&pid_motor_yaw, laser_current_yaw);
        QD4310_SetSpeed(&YawMotor,yaw_speed);

        // 【修正】补齐缺失的_Pitch后缀，防止编译报错
        pitch_speed = PID_Compute_Pitch(&pid_motor_pitch, laser_current_pitch);
        QD4310_SetSpeed(&PitchMotor,pitch_speed);
    }
}

void QD4310_PID_Pro_Extend(void) {
    // 1. 更新视觉有效位
    QD4310_Vaild_Update();
    yaw_speed = 0;
    pitch_speed = 0;
    // 2. 【核心更新】调用测速函数，更新 vision_vx 和 vision_vy
    // 必须在 PID 计算前调用，确保前馈使用的是最新速度
    //Vision_Velocity_Update();

    QD4310_PID_Update_yaw(&__pid_motor_yaw); //先更新yaw轴和pitch轴的数据
    QD4310_PID_Update_pitch(&__pid_motor_pitch);

    buzzer_flag = 0;
    //再进行pid计算
    if (valid == 0) {
        //如果没识别到矩形框，视为无效。在此时将电机速度设为0并且复位pid，防止之前的积分项影响下一次目标识别
        QD4310_SetSpeed(&YawMotor, 0);
        QD4310_SetSpeed(&PitchMotor, 0);
        PID_Reset(&__pid_motor_yaw);
        PID_Reset(&__pid_motor_pitch);
    } else if (valid == 1) {
        yaw_speed = PID_Compute(&__pid_motor_yaw, laser_current_yaw); //这个的pid加上了输出限幅
        QD4310_SetSpeed(&YawMotor, yaw_speed);

        pitch_speed = PID_Compute(&__pid_motor_pitch, laser_current_pitch);
        QD4310_SetSpeed(&PitchMotor, pitch_speed);

    }
}
void QD4310_PID_Pro_Extend5(void) {
    // 1. 更新视觉有效位
    QD4310_Vaild_Update();
    yaw_speed = 0;
    pitch_speed = 0;
    // 2. 【核心更新】调用测速函数，更新 vision_vx 和 vision_vy
    // 必须在 PID 计算前调用，确保前馈使用的是最新速度
    //Vision_Velocity_Update();

    QD4310_PID_Update_yaw(&__pid_motor_yaw5); //先更新yaw轴和pitch轴的数据
    QD4310_PID_Update_pitch(&__pid_motor_pitch5);

    buzzer_flag = 0;
    //再进行pid计算
    if (valid == 0) {
        //如果没识别到矩形框，视为无效。在此时将电机速度设为0并且复位pid，防止之前的积分项影响下一次目标识别
        QD4310_SetSpeed(&YawMotor, 0);
        QD4310_SetSpeed(&PitchMotor, 0);
        PID_Reset(&__pid_motor_yaw5);
        PID_Reset(&__pid_motor_pitch5);
    } else if (valid == 1) {
        yaw_speed = PID_Compute_YAW(&__pid_motor_yaw5, laser_current_yaw); //这个的pid加上了输出限幅
        QD4310_SetSpeed(&YawMotor, yaw_speed);

        pitch_speed = PID_Compute_Pitch(&__pid_motor_pitch5, laser_current_pitch);
        QD4310_SetSpeed(&PitchMotor, pitch_speed);

    }
}

// // ============== 任务 4,5 (发挥题) 独占的卡尔曼+前馈 ==============
// void QD4310_PID_Pro_Extend5(void) {
//     // 1. 更新视觉有效位
//     QD4310_Vaild_Update();
//     yaw_speed = 0;
//     pitch_speed = 0;
//
//     // if (fabsf(laser_current_yaw) >= 30.0f || fabsf(laser_current_pitch) >= 30.0f) {
//     //     __pid_motor_pitch.Kp = Kp_pitch;
//     //     __pid_motor_pitch.Ki = Ki_pitch;
//     //     __pid_motor_pitch.Kd = Kd_pitch;
//     //     __pid_motor_yaw.Kp =  Kp_yaw;
//     //     __pid_motor_yaw.Ki =  Ki_yaw;
//     //     __pid_motor_yaw.Kd =  Kd_yaw;
//     //
//     // }else {
//     //     __pid_motor_pitch.Kp = __Kp_pitch;
//     //     __pid_motor_pitch.Ki = __Ki_pitch;
//     //     __pid_motor_pitch.Kd = __Kd_pitch;
//     //     __pid_motor_yaw.Kp =  __Kp_yaw;
//     //     __pid_motor_yaw.Ki =  __Ki_yaw;
//     //     __pid_motor_yaw.Kd =  __Kd_yaw;
//     // }
//
//     // 2. 先获取原始的像素数据
//     QD4310_PID_Update_yaw(&__pid_motor_yaw);
//     QD4310_PID_Update_pitch(&__pid_motor_pitch);
//
//     buzzer_flag = 0;
//
//     //再进行pid计算
//     if (valid == 0) {
//         // 目标丢失
//         QD4310_SetSpeed(&YawMotor, 0);
//         QD4310_SetSpeed(&PitchMotor, 0);
//         PID_Reset(&__pid_motor_yaw);
//         PID_Reset(&__pid_motor_pitch);
//
//         // 【新增】如果目标丢了，重置卡尔曼
//         Kalman_Reset(&kf_yaw);
//         Kalman_Reset(&kf_pitch);
//     } else if (valid == 1) {
//         // 【新增】对读取到的原始像素偏差进行卡尔曼滤波平滑
//         float smoothed_yaw = Kalman_Update(&kf_yaw, laser_current_yaw, gx_GetUs());
//         float smoothed_pitch = Kalman_Update(&kf_pitch, laser_current_pitch, gx_GetUs());
//
//         // 【新增】直接提取卡尔曼算出来的运动速度，用来做前馈！
//         vision_vx = kf_yaw.v;
//         vision_vy = kf_pitch.v;
//
//         //【新增】修改了这里：PID计算时传入平滑后的偏差
//         float yaw_pid_out = PID_Compute(&__pid_motor_yaw, smoothed_yaw);
//         float pitch_pid_out = PID_Compute(&__pid_motor_pitch, smoothed_pitch);
//
//         // 【新增】这里你可以调前馈系数 Kf，弥补动态追踪时的落后！
//         __Kf_yaw = 0.0f;   // 你可以从 0.001 开始慢慢往上加测试
//         __Kf_pitch = 0.0f;
//         static float yaw_ff = 0;
//         static float pitch_ff = 0;
//         yaw_ff = vision_vx * __Kf_yaw;
//         pitch_ff = vision_vy * __Kf_pitch;
//         // //限制前馈输出
//         // if (yaw_ff > 100.0f) {
//         //     yaw_ff = 100.0f;
//         // }else if (yaw_ff < -100.0f) {
//         //     yaw_ff = -100.0f;
//         // }else {}
//         //
//         // if (pitch_ff > 100.0f) {
//         //     pitch_ff = 100.0f;
//         // }else if (pitch_ff < -100.0f) {
//         //     pitch_ff = -100.0f;
//         // }else {}
//         //
//         // 最终输出 = PID输出 + 前馈输出
//         yaw_speed = yaw_pid_out - yaw_ff;
//         pitch_speed = pitch_pid_out - pitch_ff;
//         //
//         // if (yaw_speed > 100.0f) {
//         //     yaw_speed = 100.0f;
//         // }else if (yaw_speed < -100.0f) {
//         //     yaw_speed = -100.0f;
//         // }else {
//         // }
//         // if (pitch_speed > 100.0f) {
//         //     pitch_speed = 100.0f;
//         // }else if (pitch_speed < -100.0f) {
//         //     pitch_speed = -100.0f;
//         // }else{}
//         //
//         QD4310_SetSpeed(&YawMotor, yaw_speed);
//         QD4310_SetSpeed(&PitchMotor, pitch_speed);
//     }
// }

//
// @简介：执行一次PID运算
// @参数 FB - 反馈的值，也就是传感器采回的值
//
float PID_Compute_Pitch(PID_TypeDef *PID, float FB)
{
    float err = PID->SP - FB;

    uint64_t t_k = gx_GetUs();

    float deltaT = (t_k - PID->t_k_1)* 1.0e-6f;
    if(deltaT < 1.0e-6f) deltaT = 1.0e-6f;

    float err_dev = 0.0f;
    float err_int = 0.0f;

    if(PID->t_k_1 != 0)
    {
        err_dev = (err - PID->err_k_1) / deltaT;
        err_int = PID->err_int_k_1 + (err + PID->err_k_1) * deltaT * 0.5f;
    }

    float COp = PID->Kp * err;
    float COi = PID->Ki * err_int;
    float COd = (1 - PID->alpha) * (PID->Kd * err_dev) + PID->alpha * PID->COd_1;
    float CO = COp + COi + COd;

    // 更新
    PID->COd_1 = COd;
    PID->t_k_1 = t_k;
    PID->err_k_1 = err;
    PID->err_int_k_1 = err_int;

    // 输出限幅
    if(CO > PID->UpperLimit) CO = PID->UpperLimit;
    if(CO < PID->LowerLimit) CO = PID->LowerLimit;

    if(PID->err_int_k_1 > 20.0f) PID->err_int_k_1 = 20.0f;
    if(PID->err_int_k_1 < -20.0f) PID->err_int_k_1 = -20.0f;

    // // 当误差小于2的时候不输出
    // if (fabsf(err) <= 2.0f) {
    //     CO = 0.0f;
    // }

    return CO;
}


//
// @简介：执行一次PID运算
// @参数 FB - 反馈的值，也就是传感器采回的值
//
float PID_Compute_YAW(PID_TypeDef *PID, float FB)
{
    float err = PID->SP - FB;

    uint64_t t_k = gx_GetUs();

    float deltaT = (t_k - PID->t_k_1)* 1.0e-6f;
    if(deltaT < 1.0e-6f) deltaT = 1.0e-6f;

    float err_dev = 0.0f;
    float err_int = 0.0f;

    if(PID->t_k_1 != 0)
    {
        err_dev = (err - PID->err_k_1) / deltaT;
        err_int = PID->err_int_k_1 + (err + PID->err_k_1) * deltaT * 0.5f;
    }

    float COp = PID->Kp * err;
    float COi = PID->Ki * err_int;
    float COd = (1 - PID->alpha) * (PID->Kd * err_dev) + PID->alpha * PID->COd_1;
    float CO = COp + COi + COd;

    // 更新
    PID->COd_1 = COd;
    PID->t_k_1 = t_k;
    PID->err_k_1 = err;
    PID->err_int_k_1 = err_int;

    // 输出限幅
    if(CO > PID->UpperLimit) CO = PID->UpperLimit;
    if(CO < PID->LowerLimit) CO = PID->LowerLimit;

    // 积分限幅
    if(PID->err_int_k_1 > 10.0f) PID->err_int_k_1 = 10.0f;
    if(PID->err_int_k_1 < -10.0f) PID->err_int_k_1 = -10.0f;
    // // 当误差小于2的时候不输出
    // if (fabsf(err) <= 1.5f) {
    //     CO = 0.0f;
    // }

    return CO;
}