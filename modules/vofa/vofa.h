/**
 * @file vofa.h
 * @brief VOFA+调试模块
 * 
 * 【用法】
 *   VOFA(0, yaw, pitch);           // 发送到通道0,1，上位机数据引擎选justfloat
 *   VOFA(2, speed, accel);         // 发送到通道2,3
 * 
 *   VOFA_BIND(0, &kp, &ki);        // 绑定通道0,1
 *   VOFA_BIND(2, &kd);             // 绑定通道2
 * 
 * 【指定串口】
 *   VofaSend(&huart6, 0, data, 3); // 使用huart6发送
 *   VofaBind(&huart6, 0, ptrs, 3); // 使用huart6接收
 */

#ifndef VOFA_H
#define VOFA_H

#include "usart.h"
#include <stdint.h>

/* ======================== 配置 ======================== */
#define VOFA_UART           huart1      // 默认串口
#define VOFA_MAX_CH         16          // 最大通道数
#define VOFA_MAX_BIND_GROUPS 8          // 最大接收绑定组数，可根据需要调整

/**
 * @brief 发送数据（指定起始通道）
 */
#define VOFA(start, ...) do { \
    float _v[] = {__VA_ARGS__}; \
    VofaSend(NULL, start, _v, sizeof(_v)/sizeof(float)); \
} while(0)

/**
 * @brief 绑定接收变量（指定起始通道）
 */
#define VOFA_BIND(start, ...) do { \
    static float *_p[] = {__VA_ARGS__}; \
    VofaBind(NULL, start, _p, sizeof(_p)/sizeof(float*)); \
} while(0)

/**
 * @brief 发送数据到VOFA
 * @param uart 串口句柄，NULL则使用默认的VOFA_UART
 * @param start_ch 起始通道号(0-15)
 * @param data 数据数组
 * @param len 数据个数
 */
void VofaSend(UART_HandleTypeDef *uart, uint8_t start_ch, float *data, uint8_t len);

/**
 * @brief 绑定接收变量
 * @param uart 串口句柄，NULL则使用默认的VOFA_UART
 * @param start_ch 起始通道号
 * @param ptrs 变量指针数组
 * @param len 变量个数
 */
void VofaBind(UART_HandleTypeDef *uart, uint8_t start_ch, float **ptrs, uint8_t len);

#endif
