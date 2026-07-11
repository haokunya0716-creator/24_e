#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include <stdint.h>

/* 卡尔曼滤波器结构体 (二维：位置 + 速度) */
typedef struct {
    // 状态量
    float p; // 滤波后的位置 (如像素偏差)
    float v; // 滤波后的速度 (像素/秒)

    // 协方差矩阵 P (2x2)
    float P[2][2];

    // 过程噪声 Q (相信模型的程度)
    float Q_p; // 位置过程噪声
    float Q_v; // 速度过程噪声

    // 测量噪声 R (相信传感器的程度)
    float R;

    // 时间记录
    uint64_t last_time;
    uint8_t is_initialized;
} KalmanFilter_t;

// 函数声明
void Kalman_Init(KalmanFilter_t *kf, float Q_p, float Q_v, float R);
void Kalman_Reset(KalmanFilter_t *kf);
float Kalman_Update(KalmanFilter_t *kf, float measure_pos, uint64_t current_time_us);

#endif