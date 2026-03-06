#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

/**
 * @brief 蜂鸣器工作模式
 */
typedef enum {
    BUZZER_MODE_IDLE = 0,       // 空闲
    BUZZER_MODE_BEEP,           // 普通蜂鸣 (固定频率,指定次数)
    BUZZER_MODE_DESCENDING,     // 降调蜂鸣 (全部离线警报)
} BuzzerMode_e;

/**
 * @brief 蜂鸣器初始化
 */
void BuzzerInit(void);

/**
 * @brief 设置蜂鸣 (非阻塞, 需周期调用BuzzerTask)
 * @param times 蜂鸣次数 (1-255)
 * @param on_ms 每声持续时间(ms)
 * @param off_ms 声音间隔时间(ms)
 * @param tail_ms 整轮蜂鸣结束后的静音间隔(ms), 用于控制两轮报警之间的间隔
 * @param freq 频率等级 (1-10, 1最高音, 10最低音)
 * @note 内部使用DWT计时,调用频率不限
 */
void BuzzerBeep(uint8_t times, uint16_t on_ms, uint16_t off_ms, uint16_t tail_ms, uint8_t freq);

/**
 * @brief 播放连续降调蜂鸣 (用于全部离线警报)
 * @param start_val  起始分频计数 (如4000, 对应prescaler=4, 低音)
 * @param end_val    停止阈值 (如1000, 计数低于此值时结束)
 * @param duration_ms 整个降调过程持续时间(ms)
 * @note prescaler = counter / 1000, compare = 10000
 *       参照BuzzerOn()线性递减模式, DWT精确计时
 */
void BuzzerPlayDescending(int16_t start_val, int16_t end_val, uint16_t duration_ms);

/**
 * @brief 蜂鸣器状态机驱动 - 需周期调用(调用频率不限,内部DWT计时)
 */
void BuzzerTask(void);

/**
 * @brief 蜂鸣器是否忙 (正在播放任何音效)
 * @return 1:忙, 0:空闲
 */
uint8_t BuzzerIsBusy(void);

/**
 * @brief 蜂鸣器是否就绪 (完全空闲,包括尾部静音间隔)
 * @return 1:就绪可接受新指令, 0:仍在播放或尾部静音中
 * @note 比BuzzerIsBusy更严格: 尾部静音结束后才返回1
 */
uint8_t BuzzerIsReady(void);

/**
 * @brief 获取当前工作模式
 */
BuzzerMode_e BuzzerGetMode(void);

/**
 * @brief 强制关闭蜂鸣器并重置所有状态
 */
void BuzzerOff(void);

#endif // !BSP_BUZZER_H
