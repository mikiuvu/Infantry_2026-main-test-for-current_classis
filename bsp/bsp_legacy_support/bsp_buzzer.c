#include "bsp_buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

// 状态机变量
static uint8_t  beep_times = 0;      // 剩余蜂鸣次数
static uint16_t beep_on_ms = 0;      // 每声持续时间
static uint16_t beep_off_ms = 0;     // 间隔时间
static uint8_t  beep_freq = 4;       // 频率等级
static uint16_t beep_timer = 0;      // 计时器
static uint8_t  is_on = 0;           // 当前是否在响

void BuzzerInit(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void BuzzerBeep(uint8_t times, uint16_t on_ms, uint16_t off_ms, uint8_t freq)
{
    if (times == 0) { BuzzerOff(); return; }
    if (freq < 1) freq = 1;
    if (freq > 10) freq = 10;
    
    beep_times = times;
    beep_on_ms = on_ms;
    beep_off_ms = off_ms;
    beep_freq = freq;
    beep_timer = 0;
    is_on = 1;
    
    // 立即开始响
    __HAL_TIM_PRESCALER(&htim4, beep_freq);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 5000);
}

void BuzzerTask(void)
{
    if (beep_times == 0) return;
    
    beep_timer++;
    
    if (is_on) {
        if (beep_timer >= beep_on_ms) {
            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
            beep_times--;
            beep_timer = 0;
            is_on = 0;
        }
    } else {
        if (beep_timer >= beep_off_ms && beep_times > 0) {
            __HAL_TIM_PRESCALER(&htim4, beep_freq);
            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 5000);
            beep_timer = 0;
            is_on = 1;
        }
    }
}

uint8_t BuzzerIsBusy(void)
{
    return (beep_times > 0) ? 1 : 0;
}

void BuzzerOff(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
    beep_times = 0;
    beep_timer = 0;
    is_on = 0;
}
