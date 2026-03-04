/**
 * @file bsp_buzzer.c
 * @brief 蜂鸣器BSP驱动 (DWT精确计时版)
 * @note 修复: 使用DWT时间戳替代调用计数,消除计时精度对调用频率的依赖
 *       新增: 降调蜂鸣模式(全部离线警报)、就绪检测(含尾部静音)
 */

#include "bsp_buzzer.h"
#include "bsp_dwt.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

// ======================== 内部状态 ========================

static BuzzerMode_e buzzer_mode = BUZZER_MODE_IDLE;
static float last_transition_ms = 0;  // 上次状态转换的DWT时间戳(ms)

// --- 普通蜂鸣状态 ---
static uint8_t  beep_times  = 0;     // 剩余蜂鸣次数
static uint16_t beep_on_ms  = 0;     // 每声持续时间(ms)
static uint16_t beep_off_ms = 0;     // 间隔时间(ms)
static uint8_t  beep_freq   = 4;     // 频率等级
static uint8_t  is_on       = 0;     // 当前是否在响

// --- 尾部静音 (蜂鸣完毕后短暂保持busy,确保听觉间隔) ---
#define BUZZER_TAIL_SILENCE_MS  80
static uint8_t tail_silence = 0;

// --- 降调蜂鸣状态 ---
static uint8_t  desc_total_steps   = 0;
static uint8_t  desc_current_step  = 0;
static uint16_t desc_step_ms       = 0;
static uint8_t  desc_freq_start    = 1;
static uint8_t  desc_freq_end      = 10;

// ======================== 硬件操作 ========================

static void _BuzzerHwOn(uint8_t freq)
{
    if (freq < 1)  freq = 1;
    if (freq > 10) freq = 10;
    __HAL_TIM_PRESCALER(&htim4, freq);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 5000);
}

static void _BuzzerHwOff(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

// ======================== 公开接口 ========================

void BuzzerInit(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    _BuzzerHwOff();
}

void BuzzerBeep(uint8_t times, uint16_t on_ms, uint16_t off_ms, uint8_t freq)
{
    if (times == 0) { BuzzerOff(); return; }
    if (freq < 1)  freq = 1;
    if (freq > 10) freq = 10;

    buzzer_mode = BUZZER_MODE_BEEP;
    beep_times  = times;
    beep_on_ms  = on_ms;
    beep_off_ms = off_ms;
    beep_freq   = freq;
    is_on        = 1;
    tail_silence = 0;
    last_transition_ms = DWT_GetTimeline_ms();

    _BuzzerHwOn(freq);
}

void BuzzerPlayDescending(uint8_t steps, uint16_t step_ms, uint8_t freq_start, uint8_t freq_end)
{
    if (steps == 0) return;

    buzzer_mode       = BUZZER_MODE_DESCENDING;
    desc_total_steps  = steps;
    desc_current_step = 0;
    desc_step_ms      = step_ms;
    desc_freq_start   = (freq_start < 1)  ? 1  : freq_start;
    desc_freq_end     = (freq_end   > 10) ? 10 : freq_end;
    tail_silence      = 0;
    last_transition_ms = DWT_GetTimeline_ms();

    _BuzzerHwOn(desc_freq_start);
}

void BuzzerTask(void)
{
    float now = DWT_GetTimeline_ms();
    float elapsed = now - last_transition_ms;

    // --- 尾部静音: 等待间隔结束后回到IDLE ---
    if (tail_silence) {
        if (elapsed >= (float)BUZZER_TAIL_SILENCE_MS) {
            tail_silence = 0;
            buzzer_mode  = BUZZER_MODE_IDLE;
        }
        return;
    }

    if (buzzer_mode == BUZZER_MODE_IDLE) return;

    // --- 普通蜂鸣模式 ---
    if (buzzer_mode == BUZZER_MODE_BEEP) {
        if (is_on) {
            if (elapsed >= (float)beep_on_ms) {
                _BuzzerHwOff();
                beep_times--;
                last_transition_ms = now;
                is_on = 0;
                if (beep_times == 0) {
                    tail_silence = 1;  // 最后一声结束,进入尾部静音
                }
            }
        } else {
            if (elapsed >= (float)beep_off_ms && beep_times > 0) {
                _BuzzerHwOn(beep_freq);
                last_transition_ms = now;
                is_on = 1;
            }
        }
    }
    // --- 降调蜂鸣模式 ---
    else if (buzzer_mode == BUZZER_MODE_DESCENDING) {
        if (elapsed >= (float)desc_step_ms) {
            desc_current_step++;
            if (desc_current_step >= desc_total_steps) {
                // 降调播放完毕
                _BuzzerHwOff();
                last_transition_ms = now;
                tail_silence = 1;
            } else {
                // 线性插值: 从freq_start渐变到freq_end
                uint8_t freq = desc_freq_start +
                    (uint8_t)((float)(desc_freq_end - desc_freq_start) *
                              (float)desc_current_step / (float)(desc_total_steps - 1));
                _BuzzerHwOn(freq);
                last_transition_ms = now;
            }
        }
    }
}

uint8_t BuzzerIsBusy(void)
{
    return (buzzer_mode != BUZZER_MODE_IDLE || tail_silence) ? 1 : 0;
}

uint8_t BuzzerIsReady(void)
{
    return (buzzer_mode == BUZZER_MODE_IDLE && !tail_silence) ? 1 : 0;
}

BuzzerMode_e BuzzerGetMode(void)
{
    return buzzer_mode;
}

void BuzzerOff(void)
{
    _BuzzerHwOff();
    buzzer_mode  = BUZZER_MODE_IDLE;
    beep_times   = 0;
    is_on        = 0;
    tail_silence = 0;
    desc_total_steps  = 0;
    desc_current_step = 0;
}
