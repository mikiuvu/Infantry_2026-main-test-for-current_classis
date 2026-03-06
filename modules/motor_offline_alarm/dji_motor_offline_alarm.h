/**
 * @file dji_motor_offline_alarm.h
 * @brief 大疆电机离线蜂鸣器报警模块 (v2.0 - 优先级调度版)
 * @note 特性:
 *       1. 多组电机离线检测 + 集中优先级调度 (蜂鸣次数少的优先)
 *       2. 上电静默期 (避免初始化瞬间误报)
 *       3. 全部离线降调警报 (所有注册电机均离线时)
 *       4. 基于DWT精确计时,不依赖调用频率
 * 
 * ======================= 板级拓扑 =======================
 * 
 *   GIMBAL_BOARD: 同时运行 GimbalTask + ShootTask
 *     → 云台组 (run_buzzer_task=1, 负责调度)
 *     → 发射组 (run_buzzer_task=0, 仅扫描)
 * 
 *   CHASSIS_BOARD / FORCE_CONTROL_CHASSIS_BOARD:
 *     → 底盘组 (run_buzzer_task=1, 负责调度)
 * 
 * ======================= 使用方法 =======================
 * 
 * 1. 在Init函数中配置并注册:
 *    MotorOfflineAlarmConfig_t cfg = {
 *        .motors = {yaw_motor, pitch_motor},
 *        .beep_times = {1, 2},       // 响声次数=优先级(少的先响)
 *        .motor_count = 2,
 *        .buzzer_freq = ALARM_FREQ_HIGH,
 *        .run_buzzer_task = 1,       // 主模块负责调度
 *    };
 *    alarm_inst = MotorOfflineAlarmRegister(&cfg);
 * 
 * 2. 在Task函数中调用:
 *    MotorOfflineAlarmTask(alarm_inst);
 * 
 * ======================= 优先级规则 =======================
 * 
 * - 蜂鸣器空闲时,从所有组中选取 beep_times 最小的待报警电机
 * - 同一组内按轮流顺序扫描,避免某个电机被饥饿
 * - 全部离线时,跳过单电机报警,改为降调警报
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
#define ALARM_DEFAULT_BEEP_ON_MS       80    // 默认响持续时间(ms)
#define ALARM_DEFAULT_BEEP_OFF_MS      500   // 默认声间间隔时间(ms)
#define ALARM_DEFAULT_BEEP_TAIL_MS     1200  // 默认两轮报警之间的间隔(ms)
#define ALARM_FREQ_HIGH                2     // 高音调 (云台/发射)
#define ALARM_FREQ_LOW                 8     // 低音调 (底盘)

// ======================== 全局行为配置 ========================
#define ALARM_POWER_ON_SILENCE_MS      3000  // 上电静默期(ms) - 期间不报警
#define ALARM_ALL_OFFLINE_START_VAL    4000  // 全离线降调: 起始分频计数 (psc=4, 低音)
#define ALARM_ALL_OFFLINE_END_VAL      1000  // 全离线降调: 停止阈值 (psc=1)
#define ALARM_ALL_OFFLINE_DURATION_MS  1500  // 全离线降调: 总时长(ms)
#define ALARM_ALL_OFFLINE_COOLDOWN_MS  2000  // 全离线降调: 播放后冷却(ms)

/**
 * @brief 单个离线电机的报警请求 (内部使用)
 */
typedef struct {
    uint8_t  beep_times;     // 蜂鸣次数 (也是优先级: 越小越优先)
    uint16_t beep_on_ms;
    uint16_t beep_off_ms;
    uint16_t beep_tail_ms;   // 两轮报警之间的间隔(ms)
    uint8_t  buzzer_freq;
    uint8_t  valid;          // 请求是否有效
} AlarmRequest_t;

/**
 * @brief 电机离线检测组配置 (初始化时填写)
 */
typedef struct {
    DJIMotorInstance *motors[MOTOR_GROUP_MAX_SIZE];  // 电机指针数组
    uint8_t beep_times[MOTOR_GROUP_MAX_SIZE];        // 各电机蜂鸣次数 (=优先级)
    uint8_t motor_count;          // 电机数量 (必填)
    uint16_t check_period_ms;     // 检测间隔(ms), 0=使用默认200ms
    uint16_t beep_on_ms;          // 蜂鸣响持续时间(ms), 0=使用默认80ms
    uint16_t beep_off_ms;         // 声间间隔时间(ms), 0=使用默认500ms
    uint16_t beep_tail_ms;        // 两轮报警间隔(ms), 0=使用默认1200ms
    uint8_t buzzer_freq;          // 音调 (ALARM_FREQ_HIGH / ALARM_FREQ_LOW)
    uint8_t run_buzzer_task;      // 1=主调度器(执行BuzzerTask+优先级调度), 0=仅扫描
} MotorOfflineAlarmConfig_t;

/**
 * @brief 电机离线检测组实例 (运行时状态)
 */
typedef struct {
    // 配置 (注册时复制)
    DJIMotorInstance *motors[MOTOR_GROUP_MAX_SIZE];
    uint8_t beep_times[MOTOR_GROUP_MAX_SIZE];
    uint8_t motor_count;
    uint16_t check_period_ms;
    uint16_t beep_on_ms;
    uint16_t beep_off_ms;
    uint16_t beep_tail_ms;
    uint8_t buzzer_freq;
    uint8_t run_buzzer_task;

    // 内部状态
    float last_check_ms;          // 上次检测的DWT时间戳(ms)
    uint8_t motor_index;          // 轮流扫描索引
    uint8_t offline_flags[MOTOR_GROUP_MAX_SIZE]; // 各电机离线状态缓存
    uint8_t offline_count;        // 当前离线电机数
    AlarmRequest_t pending;       // 当前待播放的报警请求
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
 * @note run_buzzer_task=1的实例负责驱动BuzzerTask和全局调度
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
