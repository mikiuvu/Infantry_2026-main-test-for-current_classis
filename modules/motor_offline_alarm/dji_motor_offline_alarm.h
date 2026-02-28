/**
 * @file dji_motor_offline_alarm.h
 * @brief 大疆电机离线蜂鸣器报警模块
 * @note 支持多组电机的离线检测,轮流报警机制
 *       每组电机可设置不同音调以区分
 * 
 * ======================= 使用方法 =======================
 * 
 * 1. 包含头文件:
 *    #include "dji_motor_offline_alarm.h"
 * 
 * 2. 声明实例指针 (文件顶部静态变量区):
 *    static MotorOfflineAlarmInstance *gimbal_offline_alarm = NULL;
 * 
 * 3. 在Init函数中配置并注册 (类似PID配置风格):
 *    MotorOfflineAlarmConfig_t gimbal_alarm_cfg = {
 *        .motors = {yaw_motor, pitch_motor},   // 电机指针数组
 *        .beep_times = {1, 2},                 // 各电机蜂鸣次数 (用于区分)
 *        .motor_count = 2,                     // 电机数量 (必填)
 *        .buzzer_freq = ALARM_FREQ_HIGH,       // 音调: HIGH=2高音, LOW=8低音
 *        .run_buzzer_task = 1,                 // 是否执行BuzzerTask
 *        // 以下可选, 不填则使用默认值:
 *        // .check_period = 500,               // 检测周期 (调用次数)
 *        // .beep_on_ms = 100,                 // 响持续时间 (ms)
 *        // .beep_off_ms = 150,                // 间隔时间 (ms)
 *    };
 *    gimbal_offline_alarm = MotorOfflineAlarmRegister(&gimbal_alarm_cfg);
 * 
 * 4. 在Task函数中调用检测:
 *    void GimbalTask() {
 *        MotorOfflineAlarmTask(gimbal_offline_alarm);  // 放在Task开头
 *        // ... 其他代码 ...
 *    }
 * 
 * ======================= 参数说明 =======================
 * 
 * - run_buzzer_task: 蜂鸣器驱动需要周期调用BuzzerTask()
 *   - = 1: 该模块负责调用 (主模块)
 *   - = 0: 不调用, 由其他模块负责 (避免重复调用)
 *   - 同一Task中多个模块只需一个设为1
 * 
 * - buzzer_freq: 音调 (1-10, 数字越小越高)
 *   - ALARM_FREQ_HIGH = 2 (高音, 推荐云台/发射)
 *   - ALARM_FREQ_LOW  = 8 (低音, 推荐底盘)
 * 
 * ======================= 配置示例 =======================
 * 
 * 云台 (高音):
 *    .motors = {yaw_motor, pitch_motor},
 *    .beep_times = {1, 2},  // yaw离线响1声, pitch离线响2声
 *    .buzzer_freq = ALARM_FREQ_HIGH,
 *    .run_buzzer_task = 1,
 * 
 * 底盘 (低音):
 *    .motors = {motor_rf, motor_lb, motor_rb, motor_lf},
 *    .beep_times = {1, 2, 3, 4},
 *    .buzzer_freq = ALARM_FREQ_LOW,
 *    .run_buzzer_task = 1,
 * 
 * 发射 (高音, 与云台同Task):
 *    .motors = {loader, friction_l, friction_r},
 *    .beep_times = {3, 4, 5},  // 与云台编号错开
 *    .buzzer_freq = ALARM_FREQ_HIGH,
 *    .run_buzzer_task = 0,  // 云台已调用BuzzerTask
 * 
 * ===========================================================
 */

#ifndef DJI_MOTOR_OFFLINE_ALARM_H
#define DJI_MOTOR_OFFLINE_ALARM_H

#include <stdint.h>
#include "dji_motor.h"

#define MOTOR_GROUP_MAX_SIZE    8   // 每组最多支持的电机数量
#define MOTOR_GROUP_MAX_COUNT   4   // 最多支持的电机组数量

// 默认参数
#define ALARM_DEFAULT_CHECK_PERIOD  500   // 默认检测周期 (500次调用)
#define ALARM_DEFAULT_BEEP_ON_MS    50   // 默认响持续时间
#define ALARM_DEFAULT_BEEP_OFF_MS   150   // 默认间隔时间
#define ALARM_FREQ_HIGH             2     // 高音调 (云台/发射)
#define ALARM_FREQ_LOW              8     // 低音调 (底盘)

/**
 * @brief 电机离线检测组配置 (支持结构体初始化风格)
 */
typedef struct {
    DJIMotorInstance *motors[MOTOR_GROUP_MAX_SIZE];  // 电机指针数组 (直接初始化)
    uint8_t beep_times[MOTOR_GROUP_MAX_SIZE];        // 每个电机对应的蜂鸣次数
    uint8_t motor_count;          // 电机数量 (必填)
    uint16_t check_period;        // 检测周期 (0=使用默认500)
    uint16_t beep_on_ms;          // 蜂鸣响持续时间 (0=使用默认50ms)
    uint16_t beep_off_ms;         // 蜂鸣间隔时间 (0=使用默认150ms)
    uint8_t buzzer_freq;          // 蜂鸣器音调 (推荐: ALARM_FREQ_HIGH/LOW)
    uint8_t run_buzzer_task;      // 是否执行BuzzerTask (1:主模块, 0:其他模块已调用)
} MotorOfflineAlarmConfig_t;

/**
 * @brief 电机离线检测组实例
 */
typedef struct {
    DJIMotorInstance *motors[MOTOR_GROUP_MAX_SIZE];  // 电机指针数组
    uint8_t beep_times[MOTOR_GROUP_MAX_SIZE];        // 每个电机对应的蜂鸣次数
    uint8_t motor_count;          // 电机数量
    uint16_t check_period;        // 检测周期
    uint16_t beep_on_ms;          // 蜂鸣响持续时间
    uint16_t beep_off_ms;         // 蜂鸣间隔时间
    uint8_t buzzer_freq;          // 蜂鸣器音调
    uint8_t run_buzzer_task;      // 是否执行BuzzerTask
    
    // 内部状态
    uint32_t check_cnt;           // 检测计数器
    uint8_t motor_index;          // 当前检测的电机索引
} MotorOfflineAlarmInstance;

/**
 * @brief 初始化电机离线检测组
 * @param config 配置参数
 * @return 电机离线检测组实例指针, 失败返回NULL
 */
MotorOfflineAlarmInstance* MotorOfflineAlarmRegister(MotorOfflineAlarmConfig_t *config);

/**
 * @brief 电机离线检测任务 - 需要周期性调用
 * @param instance 电机离线检测组实例
 */
void MotorOfflineAlarmTask(MotorOfflineAlarmInstance *instance);

/**
 * @brief 获取电机组中离线电机数量
 * @param instance 电机离线检测组实例
 * @return 离线电机数量
 */
uint8_t MotorOfflineAlarmGetOfflineCount(MotorOfflineAlarmInstance *instance);

#endif // DJI_MOTOR_OFFLINE_ALARM_H
