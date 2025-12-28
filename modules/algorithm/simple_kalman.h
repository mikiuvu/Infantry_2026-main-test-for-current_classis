/**
 ******************************************************************************
 * @file    simple_kalman.h
 * @brief   轻量级一维卡尔曼滤波器，用于传感器融合
 * @note    适用于IMU加速度积分与轮速测量的速度融合等场景
 ******************************************************************************
 */
#ifndef __SIMPLE_KALMAN_H
#define __SIMPLE_KALMAN_H

#include <stdint.h>

/**
 * @brief 一维卡尔曼滤波器结构体
 */
typedef struct
{
    float x;      // 状态估计值
    float P;      // 估计协方差
    float Q;      // 过程噪声协方差 (预测不确定性，如IMU积分误差)
    float R;      // 测量噪声协方差 (测量不确定性，如轮速打滑)
} SimpleKalman1D_t;

/**
 * @brief 一维卡尔曼滤波器初始化
 * @param kf      滤波器实例指针
 * @param Q       过程噪声协方差 (越大表示预测模型越不可信)
 * @param R       测量噪声协方差 (越大表示测量值越不可信)
 * @param init_x  初始状态估计值
 */
void SimpleKalman1D_Init(SimpleKalman1D_t *kf, float Q, float R, float init_x);

/**
 * @brief 一维卡尔曼滤波器更新 (预测+校正)
 * @param kf        滤波器实例指针
 * @param accel     加速度/变化率 (用于预测: x_pred = x + accel * dt)
 * @param measure   测量值 (用于校正)
 * @param dt        时间间隔 (秒)
 * @return          融合后的状态估计值
 * 
 * @note 典型应用:
 *       - accel: IMU加速度 (m/s² 或 mm/s²)
 *       - measure: 轮速计算的速度 (m/s 或 mm/s)
 *       - 返回: 融合后的速度估计
 */
float SimpleKalman1D_Update(SimpleKalman1D_t *kf, float accel, float measure, float dt);

/**
 * @brief 仅执行预测步骤 (无测量值时使用)
 * @param kf      滤波器实例指针
 * @param accel   加速度/变化率
 * @param dt      时间间隔 (秒)
 * @return        预测后的状态估计值
 */
float SimpleKalman1D_Predict(SimpleKalman1D_t *kf, float accel, float dt);

/**
 * @brief 仅执行校正步骤 (无过程模型时使用)
 * @param kf        滤波器实例指针
 * @param measure   测量值
 * @return          校正后的状态估计值
 */
float SimpleKalman1D_Correct(SimpleKalman1D_t *kf, float measure);

/**
 * @brief 动态调整测量噪声 (用于打滑检测等场景)
 * @param kf    滤波器实例指针
 * @param R     新的测量噪声协方差
 */
void SimpleKalman1D_SetR(SimpleKalman1D_t *kf, float R);

/**
 * @brief 动态调整过程噪声
 * @param kf    滤波器实例指针
 * @param Q     新的过程噪声协方差
 */
void SimpleKalman1D_SetQ(SimpleKalman1D_t *kf, float Q);

/**
 * @brief 重置滤波器状态
 * @param kf      滤波器实例指针
 * @param init_x  重置后的状态值
 */
void SimpleKalman1D_Reset(SimpleKalman1D_t *kf, float init_x);

/**
 * @brief 获取当前状态估计值
 * @param kf    滤波器实例指针
 * @return      当前状态估计值
 */
float SimpleKalman1D_GetState(SimpleKalman1D_t *kf);

#endif /* __SIMPLE_KALMAN_H */
