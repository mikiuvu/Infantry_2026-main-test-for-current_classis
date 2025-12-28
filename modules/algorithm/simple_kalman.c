/**
 ******************************************************************************
 * @file    simple_kalman.c
 * @brief   轻量级一维卡尔曼滤波器实现
 * @note    计算复杂度 O(1)
 ******************************************************************************
 */
#include "simple_kalman.h"

/**
 * @brief 一维卡尔曼滤波器初始化
 */
void SimpleKalman1D_Init(SimpleKalman1D_t *kf, float Q, float R, float init_x)
{
    kf->x = init_x;
    kf->P = 1.0f;   // 初始协方差，表示初始不确定性
    kf->Q = Q;      // 过程噪声 (预测误差累积)
    kf->R = R;      // 测量噪声 (传感器不确定性)
}

/**
 * @brief 一维卡尔曼滤波器更新 (预测 + 校正)
 * 
 * 算法步骤:
 * 1. 预测: x_pred = x + accel * dt
 *          P_pred = P + Q
 * 2. 校正: K = P_pred / (P_pred + R)
 *          x = x_pred + K * (measure - x_pred)
 *          P = (1 - K) * P_pred
 */
float SimpleKalman1D_Update(SimpleKalman1D_t *kf, float accel, float measure, float dt)
{
    // ========== 预测步骤 ==========
    // 状态预测: x(k|k-1) = x(k-1|k-1) + a * dt
    float x_pred = kf->x + accel * dt;
    
    // 协方差预测: P(k|k-1) = P(k-1|k-1) + Q
    float P_pred = kf->P + kf->Q;
    
    // ========== 校正步骤 ==========
    // 卡尔曼增益: K = P(k|k-1) / (P(k|k-1) + R)
    float K = P_pred / (P_pred + kf->R);
    
    // 状态更新: x(k|k) = x(k|k-1) + K * (z - x(k|k-1))
    kf->x = x_pred + K * (measure - x_pred);
    
    // 协方差更新: P(k|k) = (1 - K) * P(k|k-1)
    kf->P = (1.0f - K) * P_pred;
    
    return kf->x;
}

/**
 * @brief 仅执行预测步骤
 * @note  当没有测量值可用时使用 (如传感器离线)
 */
float SimpleKalman1D_Predict(SimpleKalman1D_t *kf, float accel, float dt)
{
    // 状态预测
    kf->x = kf->x + accel * dt;
    
    // 协方差预测 (不确定性增加)
    kf->P = kf->P + kf->Q;
    
    return kf->x;
}

/**
 * @brief 仅执行校正步骤
 * @note  当没有过程模型时使用 (纯测量滤波)
 */
float SimpleKalman1D_Correct(SimpleKalman1D_t *kf, float measure)
{
    // 卡尔曼增益
    float K = kf->P / (kf->P + kf->R);
    
    // 状态更新
    kf->x = kf->x + K * (measure - kf->x);
    
    // 协方差更新
    kf->P = (1.0f - K) * kf->P;
    
    return kf->x;
}

/**
 * @brief 动态调整测量噪声
 */
void SimpleKalman1D_SetR(SimpleKalman1D_t *kf, float R)
{
    kf->R = R;
}

/**
 * @brief 动态调整过程噪声
 */
void SimpleKalman1D_SetQ(SimpleKalman1D_t *kf, float Q)
{
    kf->Q = Q;
}

/**
 * @brief 重置滤波器状态
 */
void SimpleKalman1D_Reset(SimpleKalman1D_t *kf, float init_x)
{
    kf->x = init_x;
    kf->P = 1.0f;
}

/**
 * @brief 获取当前状态估计值
 */
float SimpleKalman1D_GetState(SimpleKalman1D_t *kf)
{
    return kf->x;
}
