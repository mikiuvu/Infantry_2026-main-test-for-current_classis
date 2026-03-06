/**
 * @file dji_motor_offline_alarm.c
 * @brief 大疆电机离线蜂鸣器报警模块 (v2.0 - 优先级调度版)
 * @note 架构:
 *       1. 每个组的 MotorOfflineAlarmTask() 扫描自身电机,生成 pending 报警请求
 *       2. run_buzzer_task=1 的组额外负责:
 *          - 驱动 BuzzerTask() 状态机
 *          - 上电静默期判断
 *          - 全部离线降调警报
 *          - 跨组优先级调度 (beep_times 最少的先播放)
 *
 * 板级部署:
 *       GIMBAL_BOARD:  云台组(主调度) + 发射组(仅扫描) → 共享蜂鸣器
 *       CHASSIS_BOARD: 底盘组(主调度) → 独占蜂鸣器
 */

#include "dji_motor_offline_alarm.h"
#include "bsp_buzzer.h"
#include "bsp_dwt.h"
#include <string.h>

// ======================== 实例池 ========================

static MotorOfflineAlarmInstance _alarm_pool[MOTOR_GROUP_MAX_COUNT];
static uint8_t _alarm_count = 0;

// ======================== 全局状态 ========================

static float    _power_on_ms          = 0;     // 首次注册时的DWT时间戳
static uint8_t  _power_on_recorded    = 0;     // 是否已记录上电时间
static float    _all_offline_cooldown = 0;     // 全离线降调冷却结束时间(ms)

// ======================== 注册 ========================

MotorOfflineAlarmInstance* MotorOfflineAlarmRegister(MotorOfflineAlarmConfig_t *config)
{
    if (!config || config->motor_count == 0 ||
        config->motor_count > MOTOR_GROUP_MAX_SIZE ||
        _alarm_count >= MOTOR_GROUP_MAX_COUNT) {
        return NULL;
    }

    // 记录上电时间 (首次注册时)
    if (!_power_on_recorded) {
        _power_on_ms       = DWT_GetTimeline_ms();
        _power_on_recorded = 1;
    }

    MotorOfflineAlarmInstance *inst = &_alarm_pool[_alarm_count++];
    memset(inst, 0, sizeof(*inst));

    // 复制配置
    inst->motor_count    = config->motor_count;
    inst->check_period_ms = config->check_period_ms ? config->check_period_ms : ALARM_DEFAULT_CHECK_PERIOD_MS;
    inst->beep_on_ms     = config->beep_on_ms  ? config->beep_on_ms  : ALARM_DEFAULT_BEEP_ON_MS;
    inst->beep_off_ms    = config->beep_off_ms  ? config->beep_off_ms  : ALARM_DEFAULT_BEEP_OFF_MS;
    inst->beep_tail_ms   = config->beep_tail_ms ? config->beep_tail_ms : ALARM_DEFAULT_BEEP_TAIL_MS;
    inst->buzzer_freq    = config->buzzer_freq  ? config->buzzer_freq  : ALARM_FREQ_HIGH;
    inst->run_buzzer_task = config->run_buzzer_task;

    for (uint8_t i = 0; i < config->motor_count; i++) {
        inst->motors[i]     = config->motors[i];
        inst->beep_times[i] = config->beep_times[i];
    }

    inst->last_check_ms = DWT_GetTimeline_ms();
    return inst;
}

// ======================== 内部函数 ========================

/**
 * @brief 检查所有注册的电机是否全部离线
 * @return 1=全部离线, 0=至少有一个在线
 * @note 直接调用 DJIMotorIsOnline(),不使用缓存
 */
static uint8_t _AllMotorsOffline(void)
{
    uint8_t total = 0, offline = 0;
    for (uint8_t g = 0; g < _alarm_count; g++) {
        MotorOfflineAlarmInstance *inst = &_alarm_pool[g];
        for (uint8_t i = 0; i < inst->motor_count; i++) {
            total++;
            if (!DJIMotorIsOnline(inst->motors[i])) {
                offline++;
            }
        }
    }
    return (total > 0 && offline == total) ? 1 : 0;
}

/**
 * @brief 从所有组中选取最高优先级的待播放报警
 * @return 选中的组索引, -1表示无待播放请求
 * @note 优先级规则: beep_times 越少越优先
 */
static int8_t _PickHighestPriorityGroup(void)
{
    int8_t  best_group = -1;
    uint8_t best_times = 255;

    for (uint8_t g = 0; g < _alarm_count; g++) {
        if (_alarm_pool[g].pending.valid &&
            _alarm_pool[g].pending.beep_times < best_times) {
            best_times = _alarm_pool[g].pending.beep_times;
            best_group = (int8_t)g;
        }
    }
    return best_group;
}

/**
 * @brief 扫描电机离线状态,更新 pending 报警请求
 * @param inst 组实例
 * @note 检测间隔使用DWT精确计时 (ms)
 */
static void _ScanAndUpdatePending(MotorOfflineAlarmInstance *inst)
{
    float now = DWT_GetTimeline_ms();

    // 检测间隔未到 → 跳过
    if (now - inst->last_check_ms < (float)inst->check_period_ms) return;
    inst->last_check_ms = now;

    // 已有待播放请求 → 不覆盖 (等调度器消费后再扫描)
    if (inst->pending.valid) return;

    // 更新离线状态
    inst->offline_count = 0;
    for (uint8_t i = 0; i < inst->motor_count; i++) {
        inst->offline_flags[i] = DJIMotorIsOnline(inst->motors[i]) ? 0 : 1;
        if (inst->offline_flags[i]) inst->offline_count++;
    }

    if (inst->offline_count == 0) {
        inst->motor_index = 0; // 全部在线,重置轮流索引
        return;
    }

    // 轮流: 从当前索引找下一个离线电机,生成报警请求
    for (uint8_t i = 0; i < inst->motor_count; i++) {
        uint8_t idx = (inst->motor_index + i) % inst->motor_count;
        if (inst->offline_flags[idx]) {
            inst->pending.beep_times = inst->beep_times[idx];
            inst->pending.beep_on_ms = inst->beep_on_ms;
            inst->pending.beep_off_ms = inst->beep_off_ms;
            inst->pending.beep_tail_ms = inst->beep_tail_ms;
            inst->pending.buzzer_freq = inst->buzzer_freq;
            inst->pending.valid       = 1;
            inst->motor_index = (idx + 1) % inst->motor_count;
            return;
        }
    }
}

// ======================== 公开接口 ========================

void MotorOfflineAlarmTask(MotorOfflineAlarmInstance *instance)
{
    if (!instance) return;

    // ---- 所有实例都要扫描自身电机 ----
    _ScanAndUpdatePending(instance);

    // ---- 仅主调度器执行以下逻辑 ----
    if (!instance->run_buzzer_task) return;

    // 驱动蜂鸣器状态机
    BuzzerTask();

    float now = DWT_GetTimeline_ms();

    // ---- 上电静默期: 不发出任何报警 ----
    if (now - _power_on_ms < (float)ALARM_POWER_ON_SILENCE_MS) return;

    // ---- 蜂鸣器忙: 等待当前音效播完 ----
    if (!BuzzerIsReady()) return;

    // ---- 全部离线 → 降调警报 (优先于单电机报警) ----
    if (_AllMotorsOffline()) {
        // 清除所有组的 pending (全离线模式下不播放单电机报警)
        for (uint8_t g = 0; g < _alarm_count; g++) {
            _alarm_pool[g].pending.valid = 0;
        }

        if (now >= _all_offline_cooldown) {
            BuzzerPlayDescending(
                ALARM_ALL_OFFLINE_START_VAL,
                ALARM_ALL_OFFLINE_END_VAL,
                ALARM_ALL_OFFLINE_DURATION_MS);

            // 设置冷却: 降调时长 + 冷却间隔
            _all_offline_cooldown = now +
                (float)ALARM_ALL_OFFLINE_DURATION_MS +
                (float)ALARM_ALL_OFFLINE_COOLDOWN_MS;
        }
        return;  // 全部离线时始终跳过单电机报警
    }

    // ---- 优先级调度: 选取 beep_times 最少的报警播放 ----
    int8_t best = _PickHighestPriorityGroup();
    if (best >= 0) {
        AlarmRequest_t *req = &_alarm_pool[best].pending;
        BuzzerBeep(req->beep_times, req->beep_on_ms, req->beep_off_ms, req->beep_tail_ms, req->buzzer_freq);
        req->valid = 0;  // 已消费
    }
}

uint8_t MotorOfflineAlarmGetOfflineCount(MotorOfflineAlarmInstance *instance)
{
    if (!instance) return 0;
    return instance->offline_count;
}

uint8_t MotorOfflineAlarmGetTotalOfflineCount(void)
{
    uint8_t total = 0;
    for (uint8_t g = 0; g < _alarm_count; g++) {
        total += _alarm_pool[g].offline_count;
    }
    return total;
}
