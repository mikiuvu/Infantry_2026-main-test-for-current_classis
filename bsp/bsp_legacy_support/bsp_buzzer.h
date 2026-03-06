/**
 * @file bsp_buzzer.h
 * @brief 蜂鸣器BSP驱动 - 纯硬件抽象层
 * @note BSP层只负责硬件开关,所有定时/状态逻辑由上层Module管理
 */

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

/**
 * @brief 初始化蜂鸣器PWM
 */
void BuzzerInit(void);

/**
 * @brief 打开蜂鸣器
 * @param freq 频率等级 (1=最高音, 10=最低音), 对应TIM prescaler
 */
void BuzzerOn(uint8_t freq);

/**
 * @brief 打开蜂鸣器 (直接设置prescaler和compare, 用于降调等特殊音效)
 * @param prescaler TIM预分频值
 * @param compare   TIM比较值
 */
void BuzzerOnRaw(uint16_t prescaler, uint16_t compare);

/**
 * @brief 关闭蜂鸣器
 */
void BuzzerOff(void);

#endif // BSP_BUZZER_H
