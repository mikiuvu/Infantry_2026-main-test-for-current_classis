/**
 * @file dji_motor_offline_alarm.c
 * @brief 大疆电机离线蜂鸣器报警模块实现
 * @note 功能:
 *       1. 支持多组电机的离线检测 (底盘/云台/发射)
 *       2. 轮流检测机制,确保多电机离线时依次报警
 *       3. 每组电机可设置不同音调以区分来源
 *       4. 蜂鸣次数对应电机编号,便于定位故障
 * 
 * 使用方法见 dji_motor_offline_alarm.h 头文件注释
 */

#include "dji_motor_offline_alarm.h"
#include "bsp_buzzer.h"
#include <string.h>

// 电机离线检测组实例池
static MotorOfflineAlarmInstance alarm_instances[MOTOR_GROUP_MAX_COUNT];
static uint8_t alarm_instance_count = 0;

/**
 * @brief 初始化电机离线检测组
 */
MotorOfflineAlarmInstance* MotorOfflineAlarmRegister(MotorOfflineAlarmConfig_t *config)
{
    if (config == NULL || config->motor_count == 0 || 
        config->motor_count > MOTOR_GROUP_MAX_SIZE ||
        alarm_instance_count >= MOTOR_GROUP_MAX_COUNT) {
        return NULL;
    }
    
    MotorOfflineAlarmInstance *instance = &alarm_instances[alarm_instance_count++];
    
    // 复制配置参数 (0值使用默认)
    instance->motor_count = config->motor_count;
    instance->check_period = config->check_period ? config->check_period : ALARM_DEFAULT_CHECK_PERIOD;
    instance->beep_on_ms = config->beep_on_ms ? config->beep_on_ms : ALARM_DEFAULT_BEEP_ON_MS;
    instance->beep_off_ms = config->beep_off_ms ? config->beep_off_ms : ALARM_DEFAULT_BEEP_OFF_MS;
    instance->buzzer_freq = config->buzzer_freq ? config->buzzer_freq : ALARM_FREQ_HIGH;
    instance->run_buzzer_task = config->run_buzzer_task;
    
    // 直接从数组复制电机指针和蜂鸣次数
    for (uint8_t i = 0; i < config->motor_count; i++) {
        instance->motors[i] = config->motors[i];
        instance->beep_times[i] = config->beep_times[i];
    }
    
    // 初始化内部状态
    instance->check_cnt = 0;
    instance->motor_index = 0;
    
    return instance;
}

/**
 * @brief 电机离线检测任务 - 轮流检测机制
 */
void MotorOfflineAlarmTask(MotorOfflineAlarmInstance *instance)
{
    if (instance == NULL) return;
    
    // 根据配置决定是否执行BuzzerTask
    if (instance->run_buzzer_task) {
        BuzzerTask();
    }
    
    // 检测周期计数
    if (++instance->check_cnt < instance->check_period) {
        return;
    }
    instance->check_cnt = 0;
    
    // 蜂鸣器忙则跳过
    if (BuzzerIsBusy()) return;
    
    // 轮流检测: 从当前索引开始,找到第一个离线的电机
    for (uint8_t i = 0; i < instance->motor_count; i++) {
        uint8_t idx = (instance->motor_index + i) % instance->motor_count;
        
        if (!DJIMotorIsOnline(instance->motors[idx])) {
            // 发出报警: 蜂鸣次数 = 电机对应的编号
            BuzzerBeep(instance->beep_times[idx], 
                       instance->beep_on_ms, 
                       instance->beep_off_ms, 
                       instance->buzzer_freq);
            
            // 下次从下一个电机开始检测 (轮流机制)
            instance->motor_index = (idx + 1) % instance->motor_count;
            return;
        }
    }
    
    // 没有电机离线,重置索引
    instance->motor_index = 0;
}

/**
 * @brief 获取电机组中离线电机数量
 */
uint8_t MotorOfflineAlarmGetOfflineCount(MotorOfflineAlarmInstance *instance)
{
    if (instance == NULL) return 0;
    
    uint8_t count = 0;
    for (uint8_t i = 0; i < instance->motor_count; i++) {
        if (!DJIMotorIsOnline(instance->motors[i])) {
            count++;
        }
    }
    return count;
}
