# `DJIMotorIsOnline()` 实现说明

## 函数

```c
uint8_t DJIMotorIsOnline(void *motor);
// 参数为 void* 以兼容通用离线报警模块 (motor_offline_alarm)
// 返回 1=在线, 0=离线
```

## 涉及的代码（共 4 处）

### 1. 电机结构体中的 daemon 指针

在 `dji_motor.h` 的 `DJIMotorInstance` 中：

```c
#include "daemon.h"

typedef struct {
    // ... 其他成员 ...
    DaemonInstance *motor_daemon;    // 电机离线检测 daemon
} DJIMotorInstance;
```

### 2. 初始化时注册 daemon

在 `DJIMotorInit()` 中，CAN 注册之后：

```c
Daemon_Init_Config_s daemon_conf = {
    .reload_count = 200,       // 200ms 超时（DaemonTask 以 1ms 运行）
    .callback = NULL,          // 不需要离线回调，由用户主动查询
    .owner_id = instance,
};
instance->motor_daemon = DaemonRegister(&daemon_conf);
```

### 3. CAN 回调中喂狗

在 `DecodeDJIMotor()`（CAN 接收回调）中：

```c
static void DecodeDJIMotor(CANInstance *_instance)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;

    // 收到 CAN 反馈报文 → 喂狗
    if (motor->motor_daemon != NULL) {
        DaemonReload(motor->motor_daemon);
    }

    // ... 解析数据 ...
}
```

### 4. 函数本体

在 `dji_motor.c` 中：

```c
uint8_t DJIMotorIsOnline(void *motor)
{
    DJIMotorInstance *m = (DJIMotorInstance *)motor;
    if (m == NULL || m->motor_daemon == NULL)
        return 0;
    return DaemonIsOnline(m->motor_daemon);
}
```

在 `dji_motor.h` 中声明：

```c
uint8_t DJIMotorIsOnline(void *motor);
```

> **为什么参数是 `void*` 而不是 `DJIMotorInstance*`？**
>
> 离线报警模块 `motor_offline_alarm` 的回调类型为 `uint8_t (*MotorIsOnlineFn)(void*)`,
> 使用 `void*` 签名后可直接赋值 `.is_online = DJIMotorIsOnline`，无需强制类型转换。
> 代价是失去编译期类型检查——如果在其他地方直接调用时误传了错误类型的指针，编译器不会报错。

## 与通用离线报警模块的配合

```c
#include "motor_offline_alarm.h"
#include "dji_motor.h"

MotorOfflineAlarmConfig_t cfg = {
    .motors = {yaw_motor, pitch_motor},
    .is_online = DJIMotorIsOnline,
    .beep_times = {1, 2},
    .buzzer_freq = ALARM_FREQ_HIGH,
    .run_buzzer_task = 1,
};
alarm_inst = MotorOfflineAlarmRegister(&cfg);
```

## 工作原理

```
电机反馈报文(CAN RX) ──→ DecodeDJIMotor() ──→ DaemonReload() ──→ 重置计数器=200
                                                                       ↑
DaemonTask(1ms周期) ──→ temp_count-- ──→ 到0则标记 OFFLINE              │
                                                                       │
DJIMotorIsOnline() ──→ DaemonIsOnline() ──→ 读取 is_online 标志 ←──────┘
```

每次收到 CAN 报文就把计数器重置为 200。`DaemonTask` 每 1ms 将计数器减 1，如果 200ms 内没有新报文，计数器归零，标记离线。
