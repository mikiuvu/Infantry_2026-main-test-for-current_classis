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

// --- 尾部静音 (蜂鸣完毕后保持busy,确保两轮报警间隔均匀) ---
static uint8_t  tail_silence    = 0;
static uint16_t tail_silence_ms = 0;  // 动态尾部静音时长(ms)
static uint16_t beep_tail_ms = 0;     // 整轮蜂鸣结束后的静音间隔(ms)

// --- 降调蜂鸣状态 (连续线性递减) ---
static int16_t  desc_start_val   = 4000;  // 起始计数
static int16_t  desc_end_val     = 1000;  // 停止阈值
static uint16_t desc_duration_ms = 1500;  // 总时长(ms)

// ======================== 硬件操作 ========================

static void _BuzzerHwOn(uint8_t freq)
{
    if (freq < 1)  freq = 1;
    if (freq > 10) freq = 10;
    __HAL_TIM_PRESCALER(&htim4, freq);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 5000);
}

/**
 * @brief 直接设置prescaler+compare (降调模式专用)
 */
static void _BuzzerHwOnRaw(int16_t prescaler, uint16_t compare)
{
    if (prescaler < 1) prescaler = 1;
    __HAL_TIM_PRESCALER(&htim4, (uint32_t)prescaler);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, compare);
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

void BuzzerBeep(uint8_t times, uint16_t on_ms, uint16_t off_ms, uint16_t tail_ms, uint8_t freq)
{
    if (times == 0) { BuzzerOff(); return; }
    if (freq < 1)  freq = 1;
    if (freq > 10) freq = 10;

    buzzer_mode = BUZZER_MODE_BEEP;
    beep_times  = times;
    beep_on_ms  = on_ms;
    beep_off_ms = off_ms;
    beep_tail_ms = tail_ms;
    beep_freq   = freq;
    is_on        = 1;
    tail_silence = 0;
    last_transition_ms = DWT_GetTimeline_ms();

    _BuzzerHwOn(freq);
}

void BuzzerPlayDescending(int16_t start_val, int16_t end_val, uint16_t duration_ms)
{
    if (duration_ms == 0 || start_val <= end_val) return;

    buzzer_mode      = BUZZER_MODE_DESCENDING;
    desc_start_val   = start_val;
    desc_end_val     = end_val;
    desc_duration_ms = duration_ms;
    tail_silence     = 0;
    last_transition_ms = DWT_GetTimeline_ms();

    // 立即以起始值开始响
    _BuzzerHwOnRaw((int16_t)(start_val / 1000), 10000);
}

void BuzzerTask(void)
{
    float now = DWT_GetTimeline_ms();
    float elapsed = now - last_transition_ms;

    // --- 尾部静音: 等待间隔结束后回到IDLE ---
    if (tail_silence) {
        if (elapsed >= (float)tail_silence_ms) {
            tail_silence    = 0;
            tail_silence_ms = 0;
            buzzer_mode     = BUZZER_MODE_IDLE;
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
                    tail_silence    = 1;
                    tail_silence_ms = beep_tail_ms;  // 使用独立的两轮间隔时长
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
    // --- 降调蜂鸣模式 (连续线性递减, 参照BuzzerOn()) ---
    else if (buzzer_mode == BUZZER_MODE_DESCENDING) {
        if (elapsed >= (float)desc_duration_ms) {
            // 降调播放完毕
            _BuzzerHwOff();
            last_transition_ms = now;
            tail_silence    = 1;
            tail_silence_ms = 200;  // 降调模式尾部静音(ms)
        } else {
            // 线性插值: start_val → end_val, prescaler = val / 1000
            float progress = elapsed / (float)desc_duration_ms;
            int16_t val = desc_start_val -
                (int16_t)(progress * (float)(desc_start_val - desc_end_val));
            if (val < desc_end_val) val = desc_end_val;
            _BuzzerHwOnRaw((int16_t)(val / 1000), 10000);
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
    buzzer_mode     = BUZZER_MODE_IDLE;
    beep_times      = 0;
    is_on           = 0;
    tail_silence    = 0;
    tail_silence_ms = 0;
    beep_tail_ms    = 0;
    desc_start_val  = 4000;
    desc_end_val    = 1000;
}
