/**
 * @file dji_motor_offline_alarm.c
 * @brief 大疆电机离线蜂鸣器报警模块 (v3.0 - 状态机版)
 * @note 状态机流程:
 *       SILENCE → IDLE ⇄ BEEP_ON → BEEP_OFF → (循环或TAIL) → IDLE
 *                      ↘ DESC → DESC_COOL → IDLE
 */

#include "dji_motor_offline_alarm.h"
#include "bsp_buzzer.h"
#include "bsp_dwt.h"
#include <string.h>

// ======================== 全局时间参数 ========================
#define DESC_START_VAL      4000  // 全离线降调: 起始分频计数
#define DESC_END_VAL        1000  // 全离线降调: 停止阈值
#define DESC_DURATION_MS    1500  // 全离线降调: 总时长(ms)
#define DESC_COOLDOWN_MS    2000  // 全离线降调: 冷却(ms)

// ======================== 实例池 ========================
static MotorOfflineAlarmInstance _pool[MOTOR_GROUP_MAX_COUNT];
static uint8_t _pool_count = 0;

// ======================== 状态机 ========================
typedef enum {
    ST_IDLE,        // 空闲,扫描电机
    ST_BEEP_ON,     // 蜂鸣器响
    ST_BEEP_OFF,    // 声间间隔
    ST_TAIL,        // 两轮报警间隔
    ST_DESC,        // 全离线降调
    ST_DESC_COOL,   // 降调冷却
} _State_e;

static _State_e _state = ST_IDLE;
static float    _state_ts = 0;        // 进入当前状态的时间戳(ms)

// 当前蜂鸣上下文
static uint8_t  _beep_remaining = 0;  // 剩余蜂鸣次数
static uint8_t  _beep_freq = ALARM_FREQ_HIGH;
static uint16_t _beep_on_ms  = ALARM_DEFAULT_BEEP_ON_MS;
static uint16_t _beep_off_ms = ALARM_DEFAULT_BEEP_OFF_MS;
static uint16_t _beep_tail_ms = ALARM_DEFAULT_BEEP_TAIL_MS;

// 轮询索引 (跨组扁平化: group * MAX_SIZE + motor)
static uint8_t _rr_index = 0;

static void _EnterState(_State_e st, float now)
{
    _state = st;
    _state_ts = now;
}

// ======================== 注册 ========================

MotorOfflineAlarmInstance* MotorOfflineAlarmRegister(MotorOfflineAlarmConfig_t *config)
{
    if (!config || config->motor_count == 0 ||
        config->motor_count > MOTOR_GROUP_MAX_SIZE ||
        _pool_count >= MOTOR_GROUP_MAX_COUNT)
        return NULL;

    MotorOfflineAlarmInstance *inst = &_pool[_pool_count++];
    memset(inst, 0, sizeof(*inst));
    inst->motor_count     = config->motor_count;
    inst->check_period_ms = config->check_period_ms ? config->check_period_ms : ALARM_DEFAULT_CHECK_PERIOD_MS;
    inst->beep_on_ms      = config->beep_on_ms  ? config->beep_on_ms  : ALARM_DEFAULT_BEEP_ON_MS;
    inst->beep_off_ms     = config->beep_off_ms  ? config->beep_off_ms  : ALARM_DEFAULT_BEEP_OFF_MS;
    inst->beep_tail_ms    = config->beep_tail_ms ? config->beep_tail_ms : ALARM_DEFAULT_BEEP_TAIL_MS;
    inst->buzzer_freq     = config->buzzer_freq  ? config->buzzer_freq  : ALARM_FREQ_HIGH;
    inst->run_buzzer_task = config->run_buzzer_task;
    inst->last_check_ms   = DWT_GetTimeline_ms();
    for (uint8_t i = 0; i < config->motor_count; i++) {
        inst->motors[i]     = config->motors[i];
        inst->beep_times[i] = config->beep_times[i];
    }
    return inst;
}

// ======================== 内部: 扫描 ========================

static void _UpdateOfflineCount(MotorOfflineAlarmInstance *inst)
{
    float now = DWT_GetTimeline_ms();
    if (now - inst->last_check_ms < (float)inst->check_period_ms)
        return;
    inst->last_check_ms = now;

    inst->offline_count = 0;
    for (uint8_t i = 0; i < inst->motor_count; i++) {
        if (!DJIMotorIsOnline(inst->motors[i]))
            inst->offline_count++;
    }
}

static uint8_t _AllMotorsOffline(void)
{
    uint8_t total = 0, offline = 0;
    for (uint8_t g = 0; g < _pool_count; g++) {
        for (uint8_t i = 0; i < _pool[g].motor_count; i++) {
            total++;
            if (!DJIMotorIsOnline(_pool[g].motors[i]))
                offline++;
        }
    }
    return (total > 0 && offline == total) ? 1 : 0;
}

/**
 * @brief 轮询选取下一个离线电机,并输出其组的时间参数
 * @note 从 _rr_index 开始扫描,找到后将 _rr_index 推进到下一个位置
 */
static uint8_t _FindNextOffline(uint8_t *out_freq, uint8_t *out_times,
                                 uint16_t *out_on, uint16_t *out_off, uint16_t *out_tail)
{
    // 计算所有组的电机总数
    uint8_t total = 0;
    for (uint8_t g = 0; g < _pool_count; g++)
        total += _pool[g].motor_count;
    if (total == 0) return 0;

    // 从 _rr_index 开始,扫描一圈
    for (uint8_t n = 0; n < total; n++) {
        uint8_t pos = (_rr_index + n) % total;
        // 将扁平索引映射到 (group, motor)
        uint8_t g = 0, offset = pos;
        while (g < _pool_count && offset >= _pool[g].motor_count) {
            offset -= _pool[g].motor_count;
            g++;
        }
        if (g >= _pool_count) continue;
        MotorOfflineAlarmInstance *inst = &_pool[g];
        if (!DJIMotorIsOnline(inst->motors[offset])) {
            *out_freq  = inst->buzzer_freq;
            *out_times = inst->beep_times[offset];
            *out_on    = inst->beep_on_ms;
            *out_off   = inst->beep_off_ms;
            *out_tail  = inst->beep_tail_ms;
            _rr_index  = (pos + 1) % total;  // 下次从这个电机之后开始
            return 1;
        }
    }
    return 0;
}

// ======================== 状态机驱动 ========================

static void _RunStateMachine(void)
{
    float now = DWT_GetTimeline_ms();
    float elapsed = now - _state_ts;

    switch (_state) {

    case ST_IDLE: {
        // 全部离线 → 降调
        if (_AllMotorsOffline()) {
            BuzzerOnRaw(DESC_START_VAL / 1000, 10000);
            _EnterState(ST_DESC, now);
            break;
        }
        // 轮询找下一个离线电机
        uint8_t freq, times;
        uint16_t on_ms, off_ms, tail_ms;
        if (_FindNextOffline(&freq, &times, &on_ms, &off_ms, &tail_ms)) {
            _beep_remaining = times;
            _beep_freq    = freq;
            _beep_on_ms   = on_ms;
            _beep_off_ms  = off_ms;
            _beep_tail_ms = tail_ms;
            BuzzerOn(_beep_freq);
            _EnterState(ST_BEEP_ON, now);
        }
        break;
    }

    case ST_BEEP_ON:
        if (elapsed >= (float)_beep_on_ms) {
            BuzzerOff();
            _beep_remaining--;
            if (_beep_remaining == 0)
                _EnterState(ST_TAIL, now);
            else
                _EnterState(ST_BEEP_OFF, now);
        }
        break;

    case ST_BEEP_OFF:
        if (elapsed >= (float)_beep_off_ms) {
            BuzzerOn(_beep_freq);
            _EnterState(ST_BEEP_ON, now);
        }
        break;

    case ST_TAIL:
        if (elapsed >= (float)_beep_tail_ms)
            _EnterState(ST_IDLE, now);
        break;

    case ST_DESC: {
        if (elapsed >= (float)DESC_DURATION_MS) {
            BuzzerOff();
            _EnterState(ST_DESC_COOL, now);
        } else {
            float progress = elapsed / (float)DESC_DURATION_MS;
            int16_t val = DESC_START_VAL -
                (int16_t)(progress * (float)(DESC_START_VAL - DESC_END_VAL));
            if (val < DESC_END_VAL) val = DESC_END_VAL;
            BuzzerOnRaw((uint16_t)(val / 1000), 10000);
        }
        break;
    }

    case ST_DESC_COOL:
        if (elapsed >= (float)DESC_COOLDOWN_MS)
            _EnterState(ST_IDLE, now);
        break;
    }
}

// ======================== 公开接口 ========================

void MotorOfflineAlarmTask(MotorOfflineAlarmInstance *instance)
{
    if (!instance) return;

    _UpdateOfflineCount(instance);

    if (instance->run_buzzer_task)
        _RunStateMachine();
}

uint8_t MotorOfflineAlarmGetOfflineCount(MotorOfflineAlarmInstance *instance)
{
    return instance ? instance->offline_count : 0;
}

uint8_t MotorOfflineAlarmGetTotalOfflineCount(void)
{
    uint8_t total = 0;
    for (uint8_t g = 0; g < _pool_count; g++)
        total += _pool[g].offline_count;
    return total;
}
