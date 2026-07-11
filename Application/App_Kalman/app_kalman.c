#include "app_kalman.h"

/**
 * @brief 初始化卡尔曼滤波器
 * @param Q_p 位置过程噪声 (越小越平滑，建议0.01~0.1)
 * @param Q_v 速度过程噪声 (目标运动越剧烈给得越大，建议0.001~0.01)
 * @param R   测量噪声 (相机抖动越大给得越大，建议1.0~5.0)
 */
void Kalman_Init(KalmanFilter_t *kf, float Q_p, float Q_v, float R) {
    kf->Q_p = Q_p;
    kf->Q_v = Q_v;
    kf->R = R;
    Kalman_Reset(kf);
}

/**
 * @brief 目标丢失时复位滤波器，防止重获瞬间产生跳变拉扯
 */
void Kalman_Reset(KalmanFilter_t *kf) {
    kf->p = 0.0f;
    kf->v = 0.0f;
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
    kf->is_initialized = 0;
}

/**
 * @brief 执行一次二维卡尔曼滤波迭代
 * @return 滤波后的平滑位置偏差
 */
float Kalman_Update(KalmanFilter_t *kf, float measure_pos, uint64_t current_time_us) {
    if (kf->is_initialized == 0) {
        kf->p = measure_pos;
        kf->v = 0.0f;
        kf->last_time = current_time_us;
        kf->is_initialized = 1;
        return kf->p;
    }

    // 计算 dt (秒)
    float dt = (current_time_us - kf->last_time) / 1000000.0f;
    kf->last_time = current_time_us;

    // 防御性编程：如果两次调用间隔异常，限制dt防飞车
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.001f;
    }

    // ====== 1. 预测步 ======
    float p_pred = kf->p + kf->v * dt;
    float v_pred = kf->v;

    float P00_pred = kf->P[0][0] + dt * (kf->P[1][0] + kf->P[0][1]) + dt * dt * kf->P[1][1] + kf->Q_p;
    float P01_pred = kf->P[0][1] + dt * kf->P[1][1];
    float P10_pred = kf->P[1][0] + dt * kf->P[1][1];
    float P11_pred = kf->P[1][1] + kf->Q_v;

    // ====== 2. 更新步 ======
    float y = measure_pos - p_pred; // 测量残差
    float S = P00_pred + kf->R;     // 创新协方差

    float K0 = P00_pred / S;        // 卡尔曼增益
    float K1 = P10_pred / S;

    kf->p = p_pred + K0 * y;
    kf->v = v_pred + K1 * y;

    kf->P[0][0] = (1.0f - K0) * P00_pred;
    kf->P[0][1] = (1.0f - K0) * P01_pred;
    kf->P[1][0] = P10_pred - K1 * P00_pred;
    kf->P[1][1] = P11_pred - K1 * P01_pred;

    return kf->p; // 返回滤波后的平滑位置
}