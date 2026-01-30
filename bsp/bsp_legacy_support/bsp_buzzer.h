#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

/**
 * @brief 蜂鸣器初始化
 */
void BuzzerInit(void);

/**
 * @brief 设置蜂鸣 (非阻塞, 需周期调用BuzzerTask)
 * @param times 蜂鸣次数 (1-255)
 * @param on_ms 每声持续时间(ms)
 * @param off_ms 声音间隔时间(ms)
 * @param freq 频率等级 (1-10, 1最高音, 10最低音)
 */
void BuzzerBeep(uint8_t times, uint16_t on_ms, uint16_t off_ms, uint8_t freq);

/**
 * @brief 蜂鸣器状态机 (每1ms调用)
 */
void BuzzerTask(void);

/**
 * @brief 检查蜂鸣器是否忙
 * @return 1:正在蜂鸣, 0:空闲
 */
uint8_t BuzzerIsBusy(void);

/**
 * @brief 关闭蜂鸣器
 */
void BuzzerOff(void);

#endif // !BSP_BUZZER_H
