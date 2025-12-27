# VOFA+ 极简调试模块

## 简介

一行代码发送/接收调试数据，无需初始化，即插即用！

## 快速上手

### 发送数据（观察变量）

```c
#include "bsp_vofa.h"

void YourTask(void)
{
    // 直接发送，想发几个发几个（最多16个）
    VOFA(motor_speed, target_speed, error);
    VOFA(pitch, yaw, roll, accel_x, accel_y, accel_z);
}
```

### 接收数据（远程调参）

```c
#include "bsp_vofa.h"

// 定义要调的参数
float kp = 1.0f, ki = 0.1f, kd = 0.5f;

void YourInit(void)
{
    // 绑定变量，VOFA+发送数据时自动更新
    VOFA_BIND(&kp, &ki, &kd);
}

void YourTask(void)
{
    // kp, ki, kd 会被VOFA+发来的数据自动更新
    output = kp * error + ki * integral + kd * derivative;
    
    // 可选：检查是否有新数据
    if (VOFA_UPDATED()) {
        // 参数已更新
    }
}
```

## API 说明

### 发送相关

| 宏/函数 | 说明 |
|--------|------|
| `VOFA(v1, v2, ...)` | 发送多个float到VOFA+，最多16个 |
| `VofaSendForce(data, len)` | 强制立即发送（忽略间隔限制） |

### 接收相关

| 宏/函数 | 说明 |
|--------|------|
| `VOFA_BIND(&a, &b, ...)` | 绑定变量指针，接收数据自动更新 |
| `VOFA_UPDATED()` | 检查是否有新数据到达（读后清除） |
| `VOFA_GET(idx)` | 获取指定通道的值（不绑定模式） |
| `VofaGetCount()` | 获取最近接收的通道数 |

## VOFA+ 上位机配置

### 接收数据（查看曲线）
1. 选择正确的串口和波特率
2. 协议选择 **JustFloat**
3. 点击连接，即可看到波形

### 发送数据（远程调参）
1. 打开 **控件** 面板
2. 添加滑块或输入框控件
3. 控件协议设置为 **JustFloat**
4. 拖动滑块，MCU端绑定的变量自动更新

## 配置选项

在 `bsp_vofa.h` 中可修改：

```c
#define VOFA_UART           huart1   // 使用的串口
#define VOFA_MAX_CH         16       // 最大通道数
#define VOFA_MIN_INTERVAL   2        // 最小发送间隔(ms)
```

## 应用示例：力控底盘调参

```c
static float force_kp = 10.0f, force_ki = 0.1f;
static float tcs_threshold = 0.5f;

void ChassisInit(void)
{
    // 绑定力控参数
    VOFA_BIND(&force_kp, &force_ki, &tcs_threshold);
}

void ChassisTask(void)
{
    // 发送力控状态观察
    VOFA(target_force, actual_force, torque_efficiency, 
         wheel_speed_lf, wheel_speed_rf, slip_factor);
}
```

## 注意事项

1. **串口冲突**：默认USART1，如有冲突请修改`VOFA_UART`
2. **波特率**：由CubeMX配置，常用115200或921600
3. **绑定时机**：`VOFA_BIND`只需调用一次
4. **协议格式**：JustFloat (小端float + 帧尾 `0x00 0x00 0x80 0x7f`)
