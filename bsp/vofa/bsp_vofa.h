/**
 * @file bsp_vofa.h
 * @brief VOFA+调试模块 - 极简版，一行代码收发数据
 * 
 * @note 【发送】VOFA(speed, angle, current);
 * @note 【接收】VOFA_BIND(&kp, &ki, &kd);
 * @note 无需初始化！无需修改bsp_usart！
 */

#ifndef BSP_VOFA_H
#define BSP_VOFA_H

#include "usart.h"
#include <stdint.h>

/* ======================== 配置 ======================== */
#define VOFA_UART           huart1
#define VOFA_MAX_CH         16
#define VOFA_MIN_INTERVAL   2

/* ======================== API ======================== */

// 发送: VOFA(v1, v2, v3, ...);
#define VOFA(...) do { \
    float _vofa_tmp[] = {__VA_ARGS__}; \
    VofaSend(_vofa_tmp, sizeof(_vofa_tmp)/sizeof(float)); \
} while(0)

// 接收: VOFA_BIND(&a, &b, &c);
#define VOFA_BIND(...) do { \
    static float *_vofa_ptrs[] = {__VA_ARGS__}; \
    VofaBind(_vofa_ptrs, sizeof(_vofa_ptrs)/sizeof(float*)); \
} while(0)

// 检查更新: if (VOFA_UPDATED()) { ... }
#define VOFA_UPDATED()  VofaIsUpdated()

// 获取通道: float val = VOFA_GET(0);
#define VOFA_GET(idx)   VofaGetChannel(idx)

/* ======================== 底层函数 ======================== */
void VofaSend(float *data, uint8_t len);
void VofaSendForce(float *data, uint8_t len);
void VofaBind(float **ptrs, uint8_t len);
uint8_t VofaIsUpdated(void);
float VofaGetChannel(uint8_t idx);
uint8_t VofaGetCount(void);

#endif
