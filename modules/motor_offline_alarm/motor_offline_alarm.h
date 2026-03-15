/**
 * @file motor_offline_alarm.h
 * @brief 通用电机离线蜂鸣器报警模块 (v4.0 - 多电机类型支持)
 * @note 特性:
 *       1. BSP层纯硬件抽象, 所有定时/调度逻辑在本模块
 *       2. 状态机: IDLE → BEEP_ON/OFF → TAIL → DESC → DESC_COOL
 *       3. 全部离线降调警报 + 优先级调度
 *       4. 通过回调函数支持任意电机类型
 *
 * ======================= 使用方法 =======================
 *
 * 1. DJI电机示例:
 *    MotorOfflineAlarmConfig_t cfg = {
 *        .motors = {yaw_motor, pitch_motor}, //填入电机实例指针
 *        .is_online = DJIMotorIsOnline,  // 填入电机在线检测函数
 *        .beep_times = {1, 2},  //填入对应电机的蜂鸣次数(优先级), 越少越优先
 *        .motor_count = 2,      //电机数量, 不填自动计数非NULL项
 *        .buzzer_freq = ALARM_FREQ_HIGH, // 云台/发射用高音调, 底盘用低音调,范围100~5000Hz
 *        .run_buzzer_task = 1,         //同时有多组电机配置在同一块板中时,
 *  //仅其中一组配置run_buzzer_task=1来驱动状态机,其他组配置为0仅扫描状态即可
 *          //其他参数可根据需要调整,如检测间隔、蜂鸣时长、两轮间隔等，不填为默认值
 *    };
 *    alarm_inst = MotorOfflineAlarmRegister(&cfg);
 *
 * 2. 其他电机类型 (需自行编写统一回调):

 *    MotorOfflineAlarmConfig_t cfg = {
 *        .motors = {my_motor1, my_motor2},
 *        .is_online = MyMotorIsOnline,
 *        ...
 *    };
 *
 * 3. 在Task函数中调用:
 *    MotorOfflineAlarmTask(alarm_inst);
 *
 * ===========================================================
 */

#ifndef MOTOR_OFFLINE_ALARM_H
#define MOTOR_OFFLINE_ALARM_H

#include <stdint.h>

// ======================== 容量限制 ========================
#define MOTOR_GROUP_MAX_SIZE    8   // 每组最多电机数量
#define MOTOR_GROUP_MAX_COUNT   4   // 最多电机组数量

// ======================== 默认参数 ========================
#define ALARM_DEFAULT_CHECK_PERIOD_MS  200   // 默认检测间隔(ms)
#define ALARM_DEFAULT_BEEP_ON_MS       100   // 默认响持续时间(ms)
#define ALARM_DEFAULT_BEEP_OFF_MS      300   // 默认声间间隔时间(ms)
#define ALARM_DEFAULT_BEEP_TAIL_MS     1000  // 默认两轮报警间隔(ms)
#define ALARM_FREQ_HIGH                400   // 高音调 800Hz (云台/发射)
#define ALARM_FREQ_LOW                 400   // 低音调 400Hz (底盘)

/**
 * @brief 电机在线检测回调函数类型
 * @param motor_ptr 电机实例指针 (void* 以支持任意电机类型)
 * @return 1=在线, 0=离线
 */
typedef uint8_t (*MotorIsOnlineFn)(void *motor_ptr);

/**
 * @brief 电机离线检测组配置 (初始化时填写)
 */
typedef struct {
    void *motors[MOTOR_GROUP_MAX_SIZE];              // 电机指针数组 (任意类型, 自动计数非NULL项)
    MotorIsOnlineFn is_online;                       // 在线检测回调 (必填)
    uint8_t beep_times[MOTOR_GROUP_MAX_SIZE];        // 各电机蜂鸣次数 (=优先级, 越少越优先)
    uint8_t motor_count;          // 电机数量, 0=自动计数motors[]中非NULL项
    uint16_t check_period_ms;     // 检测间隔(ms), 0=使用默认200ms
    uint16_t beep_on_ms;          // 蜂鸣响持续时间(ms), 0=使用默认100ms
    uint16_t beep_off_ms;         // 声间间隔时间(ms), 0=使用默认300ms
    uint16_t beep_tail_ms;        // 两轮报警间隔(ms), 0=使用默认1000ms
    uint16_t buzzer_freq;         // 蜂鸣器频率(Hz), 值越大音越高, 范围100~5000
    uint8_t run_buzzer_task;      // 1=主调度器(驱动状态机), 0=仅扫描
} MotorOfflineAlarmConfig_t;

/**
 * @brief 电机离线检测组实例
 */
typedef struct {
    void *motors[MOTOR_GROUP_MAX_SIZE];
    MotorIsOnlineFn is_online;
    uint8_t beep_times[MOTOR_GROUP_MAX_SIZE];
    uint8_t motor_count;
    uint16_t check_period_ms;
    uint16_t beep_on_ms;
    uint16_t beep_off_ms;
    uint16_t beep_tail_ms;
    uint16_t buzzer_freq;
    uint8_t run_buzzer_task;
    uint8_t offline_count;        // 当前离线电机数
    float last_check_ms;          // 上次检测DWT时间戳(ms)
} MotorOfflineAlarmInstance;

/**
 * @brief 注册电机离线检测组
 * @param config 配置参数 (is_online回调必须非NULL)
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

#endif // MOTOR_OFFLINE_ALARM_H
