/**
 * @file dji_motor_offline_alarm.h
 * @brief 大疆电机离线蜂鸣器报警模块 (v3.0 - 状态机版)
 * @note 特性:
 *       1. BSP层纯硬件抽象, 所有定时/调度逻辑在本模块
 *       2. 7状态机: SILENCE → IDLE → BEEP_ON/OFF → TAIL → DESC → DESC_COOL
 *       3. 上电静默期 + 全部离线降调警报 + 优先级调度
 *
 * ======================= 使用方法 =======================
 *
 * 1. 在Init函数中注册:
 *    MotorOfflineAlarmConfig_t cfg = {
 *        .motors = {yaw_motor, pitch_motor},
 *        .beep_times = {1, 2},       // 响声次数=优先级(少的先响)
 *        .motor_count = 2,
 *        .buzzer_freq = ALARM_FREQ_HIGH,
 *        .run_buzzer_task = 1,       // 主模块负责状态机调度
 *    };
 *    alarm_inst = MotorOfflineAlarmRegister(&cfg);
 *
 * 2. 在Task函数中调用:
 *    MotorOfflineAlarmTask(alarm_inst);
 *
 * ===========================================================
 */

#ifndef DJI_MOTOR_OFFLINE_ALARM_H
#define DJI_MOTOR_OFFLINE_ALARM_H

#include <stdint.h>
#include "dji_motor.h"

// ======================== 容量限制 ========================
#define MOTOR_GROUP_MAX_SIZE    8   // 每组最多电机数量
#define MOTOR_GROUP_MAX_COUNT   4   // 最多电机组数量

// ======================== 默认参数 ========================
#define ALARM_DEFAULT_CHECK_PERIOD_MS  200   // 默认检测间隔(ms)
#define ALARM_DEFAULT_BEEP_ON_MS       100    // 默认响持续时间(ms)
#define ALARM_DEFAULT_BEEP_OFF_MS      300   // 默认声间间隔时间(ms)
#define ALARM_DEFAULT_BEEP_TAIL_MS     1000  // 默认两轮报警间隔(ms)
#define ALARM_FREQ_HIGH                5     // 高音调 (云台/发射)
#define ALARM_FREQ_LOW                 8     // 低音调 (底盘)

/**
 * @brief 电机离线检测组配置 (初始化时填写)
 */
typedef struct {
    DJIMotorInstance *motors[MOTOR_GROUP_MAX_SIZE];  // 电机指针数组
    uint8_t beep_times[MOTOR_GROUP_MAX_SIZE];        // 各电机蜂鸣次数 (=优先级, 越少越优先)
    uint8_t motor_count;          // 电机数量 (必填)
    uint16_t check_period_ms;     // 检测间隔(ms), 0=使用默认200ms
    uint16_t beep_on_ms;          // 蜂鸣响持续时间(ms), 0=使用默认100ms
    uint16_t beep_off_ms;         // 声间间隔时间(ms), 0=使用默认300ms
    uint16_t beep_tail_ms;        // 两轮报警间隔(ms), 0=使用默认1000ms
    uint8_t buzzer_freq;          // 音调 (ALARM_FREQ_HIGH / ALARM_FREQ_LOW)
    uint8_t run_buzzer_task;      // 1=主调度器(驱动状态机), 0=仅扫描
} MotorOfflineAlarmConfig_t;

/**
 * @brief 电机离线检测组实例
 */
typedef struct {
    DJIMotorInstance *motors[MOTOR_GROUP_MAX_SIZE];
    uint8_t beep_times[MOTOR_GROUP_MAX_SIZE];
    uint8_t motor_count;
    uint16_t check_period_ms;
    uint16_t beep_on_ms;
    uint16_t beep_off_ms;
    uint16_t beep_tail_ms;
    uint8_t buzzer_freq;
    uint8_t run_buzzer_task;
    uint8_t offline_count;        // 当前离线电机数
    float last_check_ms;          // 上次检测DWT时间戳(ms)
} MotorOfflineAlarmInstance;

/**
 * @brief 注册电机离线检测组
 * @param config 配置参数
 * @return 实例指针, 失败返回NULL
 */
MotorOfflineAlarmInstance* MotorOfflineAlarmRegister(MotorOfflineAlarmConfig_t *config);

/**
 * @brief 电机离线报警任务 - 需周期调用
 * @param instance 实例指针
 * @note run_buzzer_task=1的实例负责驱动状态机
 *       run_buzzer_task=0的实例仅扫描自身电机状态
 */
void MotorOfflineAlarmTask(MotorOfflineAlarmInstance *instance);

/**
 * @brief 获取电机组中离线电机数量
 */
uint8_t MotorOfflineAlarmGetOfflineCount(MotorOfflineAlarmInstance *instance);

/**
 * @brief 获取所有注册组的离线电机总数
 */
uint8_t MotorOfflineAlarmGetTotalOfflineCount(void);

#endif // DJI_MOTOR_OFFLINE_ALARM_H
