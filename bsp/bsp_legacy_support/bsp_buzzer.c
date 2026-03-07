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

void BuzzerOn(uint16_t freq_hz)
{
    if (freq_hz < 100)  freq_hz = 100;
    if (freq_hz > 5000) freq_hz = 5000;
    // TIM4: 84MHz, ARR=65535, PWM频率 = 84MHz / (PSC+1) / 65536
    // PSC = 84000000 / (freq_hz * 65536) - 1
    uint32_t psc = 84000000 / ((uint32_t)freq_hz * 65536);
    if (psc < 1) psc = 1;
    __HAL_TIM_PRESCALER(&htim4, (uint16_t)psc);
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
