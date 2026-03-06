/**
 * @file bsp_buzzer.c
 * @brief 蜂鸣器BSP驱动 - 纯硬件抽象层
 * @note BSP层只负责硬件开关,所有定时/状态逻辑由上层Module管理
 */

#include "bsp_buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

void BuzzerInit(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    BuzzerOff();
}

void BuzzerOn(uint8_t freq)
{
    if (freq < 1)  freq = 1;
    if (freq > 10) freq = 10;
    __HAL_TIM_PRESCALER(&htim4, freq);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 5000);
}

void BuzzerOnRaw(uint16_t prescaler, uint16_t compare)
{
    if (prescaler < 1) prescaler = 1;
    __HAL_TIM_PRESCALER(&htim4, (uint32_t)prescaler);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, compare);
}

void BuzzerOff(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
