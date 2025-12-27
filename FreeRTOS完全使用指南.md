# FreeRTOS 完全使用指南 - 从 CubeMX 到实战

## 📚 目录
1. [FreeRTOS 基础概念](#1-freertos-基础概念)
2. [STM32CubeMX 配置详解](#2-stm32cubemx-配置详解)
3. [任务创建与管理](#3-任务创建与管理)
4. [任务间通信](#4-任务间通信)
5. [同步与互斥](#5-同步与互斥)
6. [内存管理](#6-内存管理)
7. [中断管理](#7-中断管理)
8. [实战案例](#8-实战案例-云台控制系统)
9. [调试技巧](#9-调试技巧)
10. [常见问题](#10-常见问题)

---

## 1. FreeRTOS 基础概念

### 1.1 什么是 RTOS?

**RTOS (Real-Time Operating System)** = 实时操作系统

```
裸机程序 (Bare Metal):           FreeRTOS 程序:
┌─────────────────┐              ┌─────────────────┐
│   while(1) {    │              │   任务调度器     │
│     task1();    │              │   ┌─────────┐   │
│     task2();    │              │   │ Task 1  │   │
│     task3();    │    VS        │   ├─────────┤   │
│   }             │              │   │ Task 2  │   │
└─────────────────┘              │   ├─────────┤   │
                                 │   │ Task 3  │   │
❌ 问题:                         │   └─────────┘   │
- 任务无法并行                   └─────────────────┘
- 响应时间不可控                 ✅ 优势:
- 代码耦合严重                   - 任务独立运行
- 难以扩展                       - 优先级管理
                                 - 实时响应
                                 - 易于维护
```

### 1.2 核心概念

#### 任务 (Task)
```c
// 任务就是一个无限循环的函数
void MyTask(void *argument)
{
    // 初始化代码
    init();
    
    for(;;)  // 无限循环
    {
        // 任务主体
        do_something();
        
        // 延时或等待
        osDelay(10);  // 让出 CPU
    }
}
```

**关键点:**
- 每个任务有独立的栈空间
- 每个任务有优先级 (0-56,数字越大优先级越高)
- 任务必须让出 CPU (通过延时、等待等)

#### 任务状态
```
                  ┌──────────┐
    创建 ────────>│  就绪态   │<─────── 被唤醒
                  │ (Ready)  │
                  └──────────┘
                       │ ▲
              调度器选中│ │时间片用完/
                       │ │被抢占
                       ▼ │
                  ┌──────────┐
                  │  运行态   │
                  │(Running) │
                  └──────────┘
                       │ ▲
              等待事件 │ │事件发生
                       ▼ │
                  ┌──────────┐
                  │  阻塞态   │────── 延时到期
                  │(Blocked) │
                  └──────────┘
```

#### 调度器 (Scheduler)
- **抢占式调度**: 高优先级任务立即抢占低优先级任务
- **时间片轮转**: 相同优先级任务轮流执行
- **空闲任务**: 优先级 0,所有任务阻塞时运行

---

## 2. STM32CubeMX 配置详解

### 2.1 打开 FreeRTOS 配置

**步骤 1**: 打开 STM32CubeMX,加载你的 `.ioc` 文件

**步骤 2**: 点击左侧 `Middleware and Software Packs` → `FREERTOS`

**步骤 3**: 选择 `Interface` = **CMSIS_V2**

```
为什么选择 CMSIS_V2?
- CMSIS_V2 是 ARM 官方标准 API
- 代码可移植性更好
- 函数命名更规范 (osXxx 开头)
- 支持更多高级特性
```

### 2.2 基础配置 (Configuration 选项卡)

#### 2.2.1 Kernel settings (内核设置)

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| **USE_PREEMPTION** | Enabled | 启用抢占式调度(必选) |
| **CPU_CLOCK_HZ** | 168000000 | CPU 频率 168MHz |
| **TICK_RATE_HZ** | 1000 | 系统节拍 1000Hz (1ms 精度) |
| **MAX_PRIORITIES** | 56 | 最大优先级数 (0-55) |
| **MINIMAL_STACK_SIZE** | 128 | 最小栈大小 128 Words = 512 Bytes |
| **MAX_TASK_NAME_LEN** | 16 | 任务名最大长度 |
| **USE_16_BIT_TICKS** | Disabled | 使用 32 位计数器(推荐) |
| **IDLE_SHOULD_YIELD** | Enabled | 空闲任务主动让出 CPU |
| **USE_MUTEXES** | Enabled | 启用互斥锁 |
| **USE_RECURSIVE_MUTEXES** | Enabled | 启用递归互斥锁 |
| **USE_COUNTING_SEMAPHORES** | Enabled | 启用计数信号量 |
| **QUEUE_REGISTRY_SIZE** | 8 | 队列注册表大小 |
| **USE_TASK_NOTIFICATIONS** | Enabled | 启用任务通知(高效) |

#### 2.2.2 Memory management settings (内存管理)

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| **Memory Allocation** | Dynamic/Static | 动态分配(简单)/静态分配(可控) |
| **TOTAL_HEAP_SIZE** | 20480 | 堆大小 20KB (根据 RAM 调整) |
| **Memory Management scheme** | heap_4 | 推荐方案,支持碎片合并 |

**heap 方案对比:**
```
heap_1: 只分配,不释放 ❌ (不推荐)
heap_2: 可释放,但有碎片 ⚠️
heap_3: 使用 malloc/free ⚠️ (线程不安全)
heap_4: 支持碎片合并 ✅ (推荐)
heap_5: 支持多内存区域 ✅ (高级)
```

#### 2.2.3 Hook function settings (钩子函数)

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| **USE_IDLE_HOOK** | Disabled | 空闲任务钩子(用于低功耗) |
| **USE_TICK_HOOK** | Disabled | 节拍钩子(用于定时任务) |
| **USE_MALLOC_FAILED_HOOK** | Enabled | 内存分配失败钩子 ✅ 调试用 |
| **USE_DAEMON_TASK_STARTUP_HOOK** | Disabled | 守护任务启动钩子 |
| **CHECK_FOR_STACK_OVERFLOW** | Option2 | 栈溢出检测 ✅ 调试必备 |

### 2.3 任务和队列配置 (Tasks and Queues 选项卡)

#### 2.3.1 创建任务示例

点击 `Add` 按钮添加任务:

**任务 1: LED 闪烁任务**
```
Task Name:          LEDTask
Priority:           osPriorityLow (24)
Stack Size (Words): 128
Entry Function:     LED_Task_Entry
Code Generation:    Default (As weak)
Parameter:          NULL
Allocation:         Dynamic
```

**任务 2: 电机控制任务**
```
Task Name:          MotorTask
Priority:           osPriorityHigh (48)
Stack Size (Words): 512
Entry Function:     Motor_Task_Entry
Code Generation:    Default (As weak)
Parameter:          NULL
Allocation:         Dynamic
```

**任务 3: 机器人主任务**
```
Task Name:          RobotTask
Priority:           osPriorityNormal (24)
Stack Size (Words): 512
Entry Function:     Robot_Task_Entry
Code Generation:    Default (As weak)
Parameter:          NULL
Allocation:         Dynamic
```

**优先级建议:**
```
osPriorityIdle        = 0   (空闲任务,系统占用)
osPriorityLow         = 8   (LED, 日志等不重要任务)
osPriorityBelowNormal = 16  (数据处理)
osPriorityNormal      = 24  (普通控制任务)
osPriorityAboveNormal = 32  (重要控制任务)
osPriorityHigh        = 40  (电机控制、通信)
osPriorityRealtime    = 48  (姿态解算、IMU)
```

#### 2.3.2 创建队列 (Queues)

点击 `Add` → `Queue` 添加队列:

**示例: 传感器数据队列**
```
Queue Name:         SensorDataQueue
Queue Size:         10              (队列长度,可存 10 条消息)
Item Size:          16              (每条消息 16 字节)
```

#### 2.3.3 创建信号量 (Semaphores)

**二值信号量 (Binary Semaphore):**
```
Semaphore Name:     UART_TxSemaphore
```

**计数信号量 (Counting Semaphore):**
```
Semaphore Name:     BufferSemaphore
Count:              5               (初始计数值)
```

#### 2.3.4 创建互斥锁 (Mutexes)

```
Mutex Name:         CAN_Mutex       (保护 CAN 发送)
```

### 2.4 中断优先级配置 (NVIC Settings)

**关键配置:**
```
中断优先级分组: 4 bits for pre-emption priority

规则:
- 优先级数字越小,优先级越高
- 抢占优先级 < 5 的中断可以调用 FreeRTOS API
- 抢占优先级 >= 5 的中断不能调用 FreeRTOS API
```

**推荐配置:**
```
SysTick               : 抢占 15, 子优先级 0  (最低,FreeRTOS 心跳)
PendSV                : 抢占 15, 子优先级 0  (任务切换)
USART3 (遥控器)       : 抢占  4, 子优先级 0  (可调用 FromISR API)
CAN1_RX0 (电机)       : 抢占  5, 子优先级 0
DMA 中断              : 抢占  5, 子优先级 1
TIM 中断              : 抢占  6, 子优先级 0
```

### 2.5 生成代码

点击 `Project` → `Generate Code` → 等待生成完成

---

## 3. 任务创建与管理

### 3.1 自动生成的任务代码

生成代码后,在 `Core/Src/freertos.c` 中可以看到:

```c
/* USER CODE BEGIN Header_LED_Task_Entry */
/**
  * @brief  LED 闪烁任务函数
  * @param  argument: 传入参数(未使用)
  * @retval None
  */
/* USER CODE END Header_LED_Task_Entry */
void LED_Task_Entry(void *argument)
{
  /* USER CODE BEGIN LED_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);  // 默认延时 1ms
  }
  /* USER CODE END LED_Task_Entry */
}
```

### 3.2 编写任务代码

#### 示例 1: LED 闪烁任务

```c
void LED_Task_Entry(void *argument)
{
    /* USER CODE BEGIN LED_Task_Entry */
    
    // 任务初始化(只执行一次)
    uint8_t led_state = 0;
    
    // 任务主循环
    for(;;)
    {
        // 翻转 LED 状态
        led_state = !led_state;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, led_state);
        
        // 延时 500ms
        osDelay(500);
    }
    
    /* USER CODE END LED_Task_Entry */
}
```

#### 示例 2: 电机控制任务

```c
void Motor_Task_Entry(void *argument)
{
    /* USER CODE BEGIN Motor_Task_Entry */
    
    // 等待系统初始化完成
    osDelay(100);
    
    for(;;)
    {
        // 执行电机控制
        DJIMotorControl();
        
        // 500Hz 控制频率
        osDelay(2);  // 2ms
    }
    
    /* USER CODE END Motor_Task_Entry */
}
```

#### 示例 3: 传感器数据处理任务

```c
void Sensor_Task_Entry(void *argument)
{
    /* USER CODE BEGIN Sensor_Task_Entry */
    
    uint32_t tick = 0;
    SensorData_t sensor_data;
    
    for(;;)
    {
        // 读取传感器数据
        sensor_data.timestamp = tick++;
        sensor_data.temperature = Read_Temperature();
        sensor_data.pressure = Read_Pressure();
        
        // 发送到队列(非阻塞)
        osMessageQueuePut(SensorDataQueueHandle, &sensor_data, 0, 0);
        
        // 100Hz 采样频率
        osDelay(10);
    }
    
    /* USER CODE END Sensor_Task_Entry */
}
```

### 3.3 任务管理 API

#### 创建任务 (动态)
```c
osThreadId_t myTaskHandle;

const osThreadAttr_t myTask_attributes = {
    .name = "myTask",
    .stack_size = 512 * 4,  // 512 Words = 2048 Bytes
    .priority = (osPriorityNormal),
};

myTaskHandle = osThreadNew(MyTask_Entry, NULL, &myTask_attributes);
```

#### 删除任务
```c
osThreadTerminate(myTaskHandle);  // 删除指定任务
osThreadTerminate(NULL);          // 删除自己
```

#### 挂起/恢复任务
```c
osThreadSuspend(myTaskHandle);   // 挂起任务
osThreadResume(myTaskHandle);    // 恢复任务
```

#### 获取任务信息
```c
osThreadState_t state = osThreadGetState(myTaskHandle);  // 获取状态
const char *name = osThreadGetName(myTaskHandle);        // 获取名称
osPriority_t priority = osThreadGetPriority(myTaskHandle); // 获取优先级
```

#### 修改任务优先级
```c
osThreadSetPriority(myTaskHandle, osPriorityHigh);
```

---

## 4. 任务间通信

### 4.1 队列 (Queue) - 最常用

#### 4.1.1 在 CubeMX 中创建队列

```
Queue Name:  DataQueue
Queue Size:  10
Item Size:   sizeof(MyData_t)
```

#### 4.1.2 定义数据结构

```c
// 队列传递的数据结构
typedef struct {
    uint32_t timestamp;
    float value1;
    float value2;
    uint8_t status;
} MyData_t;
```

#### 4.1.3 发送数据到队列

```c
void Producer_Task(void *argument)
{
    MyData_t data;
    
    for(;;)
    {
        // 准备数据
        data.timestamp = osKernelGetTickCount();
        data.value1 = 3.14f;
        data.value2 = 2.71f;
        data.status = 1;
        
        // 发送到队列(等待 100ms)
        osStatus_t status = osMessageQueuePut(
            DataQueueHandle,  // 队列句柄
            &data,            // 数据指针
            0,                // 优先级(0 = 普通)
            100               // 超时时间(ms), osWaitForever = 永久等待
        );
        
        if (status == osOK) {
            printf("Data sent successfully\n");
        } else {
            printf("Queue full!\n");
        }
        
        osDelay(100);
    }
}
```

#### 4.1.4 从队列接收数据

```c
void Consumer_Task(void *argument)
{
    MyData_t received_data;
    
    for(;;)
    {
        // 从队列接收(永久等待)
        osStatus_t status = osMessageQueueGet(
            DataQueueHandle,     // 队列句柄
            &received_data,      // 接收缓冲区
            NULL,                // 优先级(输出参数,可为 NULL)
            osWaitForever        // 永久等待
        );
        
        if (status == osOK) {
            printf("Received: %lu, %.2f, %.2f, %d\n",
                   received_data.timestamp,
                   received_data.value1,
                   received_data.value2,
                   received_data.status);
        }
    }
}
```

#### 4.1.5 队列 API 汇总

```c
// 创建队列
osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size, const osMessageQueueAttr_t *attr);

// 发送消息
osStatus_t osMessageQueuePut(osMessageQueueId_t mq_id, const void *msg_ptr, uint8_t msg_prio, uint32_t timeout);

// 接收消息
osStatus_t osMessageQueueGet(osMessageQueueId_t mq_id, void *msg_ptr, uint8_t *msg_prio, uint32_t timeout);

// 获取队列信息
uint32_t osMessageQueueGetCapacity(osMessageQueueId_t mq_id);  // 队列容量
uint32_t osMessageQueueGetCount(osMessageQueueId_t mq_id);     // 当前消息数
uint32_t osMessageQueueGetSpace(osMessageQueueId_t mq_id);     // 剩余空间

// 重置队列
osStatus_t osMessageQueueReset(osMessageQueueId_t mq_id);

// 删除队列
osStatus_t osMessageQueueDelete(osMessageQueueId_t mq_id);
```

### 4.2 信号量 (Semaphore)

#### 4.2.1 二值信号量 (Binary Semaphore) - 同步用

**使用场景:** 中断通知任务

```c
// 在 CubeMX 中创建: UART_RxSemaphore (Binary Semaphore)

// 中断服务函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        // 释放信号量,通知任务
        osSemaphoreRelease(UART_RxSemaphoreHandle);
    }
}

// 任务函数
void UART_Process_Task(void *argument)
{
    for(;;)
    {
        // 等待信号量(阻塞)
        osSemaphoreAcquire(UART_RxSemaphoreHandle, osWaitForever);
        
        // 处理接收到的数据
        Process_UART_Data();
    }
}
```

#### 4.2.2 计数信号量 (Counting Semaphore) - 资源计数

**使用场景:** 缓冲区管理

```c
// 在 CubeMX 中创建: BufferSemaphore (Counting Semaphore, Count=5)

void Producer_Task(void *argument)
{
    for(;;)
    {
        // 等待空闲缓冲区
        if (osSemaphoreAcquire(BufferSemaphoreHandle, 1000) == osOK) {
            // 填充缓冲区
            Fill_Buffer();
        }
        osDelay(10);
    }
}

void Consumer_Task(void *argument)
{
    for(;;)
    {
        // 消费缓冲区
        Process_Buffer();
        
        // 释放缓冲区
        osSemaphoreRelease(BufferSemaphoreHandle);
        
        osDelay(20);
    }
}
```

#### 4.2.3 信号量 API

```c
// 创建二值信号量
osSemaphoreId_t osSemaphoreNew(uint32_t max_count, uint32_t initial_count, const osSemaphoreAttr_t *attr);

// 获取信号量(P 操作)
osStatus_t osSemaphoreAcquire(osSemaphoreId_t semaphore_id, uint32_t timeout);

// 释放信号量(V 操作)
osStatus_t osSemaphoreRelease(osSemaphoreId_t semaphore_id);

// 获取计数值
uint32_t osSemaphoreGetCount(osSemaphoreId_t semaphore_id);

// 删除信号量
osStatus_t osSemaphoreDelete(osSemaphoreId_t semaphore_id);
```

### 4.3 互斥锁 (Mutex) - 资源保护

#### 4.3.1 使用场景

**问题:** 多个任务访问共享资源(如 CAN 总线)导致冲突

```c
// ❌ 错误示例 - 没有保护
void Task1(void *argument)
{
    CAN_Send(motor1_data);  // 任务 1 发送
}

void Task2(void *argument)
{
    CAN_Send(motor2_data);  // 任务 2 同时发送 → 冲突!
}
```

**解决方案:** 使用互斥锁

```c
// 在 CubeMX 中创建: CAN_Mutex

// ✅ 正确示例 - 互斥保护
void Task1(void *argument)
{
    for(;;)
    {
        // 获取互斥锁
        osMutexAcquire(CAN_MutexHandle, osWaitForever);
        
        // 临界区 - 只有一个任务能执行
        CAN_Send(motor1_data);
        
        // 释放互斥锁
        osMutexRelease(CAN_MutexHandle);
        
        osDelay(10);
    }
}

void Task2(void *argument)
{
    for(;;)
    {
        osMutexAcquire(CAN_MutexHandle, osWaitForever);
        CAN_Send(motor2_data);
        osMutexRelease(CAN_MutexHandle);
        
        osDelay(10);
    }
}
```

#### 4.3.2 互斥锁 vs 信号量

| 特性 | 互斥锁 (Mutex) | 二值信号量 (Binary Semaphore) |
|------|---------------|------------------------------|
| 用途 | 资源互斥访问 | 任务同步 |
| 优先级继承 | ✅ 支持(防止优先级翻转) | ❌ 不支持 |
| 递归锁定 | ✅ 支持 | ❌ 不支持 |
| 谁能释放 | 只有持有者 | 任何任务 |

**优先级继承示例:**
```
高优先级任务 A 等待 Mutex
  ↓
中优先级任务 B 抢占
  ↓
低优先级任务 C 持有 Mutex
  ↓
问题: A 被 B 阻塞,即使 C 的优先级更低!

解决: Mutex 自动提升 C 的优先级到 A,
      让 C 尽快完成并释放 Mutex
```

#### 4.3.3 互斥锁 API

```c
// 创建互斥锁
osMutexId_t osMutexNew(const osMutexAttr_t *attr);

// 获取互斥锁
osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout);

// 释放互斥锁
osStatus_t osMutexRelease(osMutexId_t mutex_id);

// 获取持有者
osThreadId_t osMutexGetOwner(osMutexId_t mutex_id);

// 删除互斥锁
osStatus_t osMutexDelete(osMutexId_t mutex_id);
```

### 4.4 事件标志 (Event Flags)

#### 4.4.1 使用场景

等待多个事件同时或任一发生

```c
// 在 CubeMX 中创建: SystemEventFlags

#define EVENT_MOTOR_READY   (1 << 0)  // 0x01
#define EVENT_SENSOR_READY  (1 << 1)  // 0x02
#define EVENT_COMM_READY    (1 << 2)  // 0x04

// 初始化任务
void Init_Task(void *argument)
{
    Motor_Init();
    osEventFlagsSet(SystemEventFlagsHandle, EVENT_MOTOR_READY);
    
    Sensor_Init();
    osEventFlagsSet(SystemEventFlagsHandle, EVENT_SENSOR_READY);
    
    Comm_Init();
    osEventFlagsSet(SystemEventFlagsHandle, EVENT_COMM_READY);
    
    osThreadTerminate(NULL);  // 初始化完成,删除自己
}

// 主控制任务
void Control_Task(void *argument)
{
    // 等待所有设备就绪
    uint32_t flags = osEventFlagsWait(
        SystemEventFlagsHandle,
        EVENT_MOTOR_READY | EVENT_SENSOR_READY | EVENT_COMM_READY,
        osFlagsWaitAll,   // 等待所有标志
        osWaitForever
    );
    
    printf("All systems ready! Flags = 0x%02X\n", flags);
    
    // 开始控制循环
    for(;;)
    {
        // 控制逻辑
        osDelay(10);
    }
}
```

#### 4.4.2 事件标志 API

```c
// 创建事件标志组
osEventFlagsId_t osEventFlagsNew(const osEventFlagsAttr_t *attr);

// 设置标志位
uint32_t osEventFlagsSet(osEventFlagsId_t ef_id, uint32_t flags);

// 清除标志位
uint32_t osEventFlagsClear(osEventFlagsId_t ef_id, uint32_t flags);

// 等待标志位
uint32_t osEventFlagsWait(osEventFlagsId_t ef_id, uint32_t flags, uint32_t options, uint32_t timeout);
// options: osFlagsWaitAny  - 任一标志满足即返回
//          osFlagsWaitAll  - 所有标志满足才返回
//          osFlagsNoClear  - 不自动清除标志

// 获取当前标志
uint32_t osEventFlagsGet(osEventFlagsId_t ef_id);
```

### 4.5 任务通知 (Task Notification) - 最高效

#### 4.5.1 特点

- **最快**: 比队列快 45%,比信号量快 20%
- **最省内存**: 不需要额外对象
- **限制**: 只能通知一个任务

#### 4.5.2 使用示例

```c
osThreadId_t TargetTaskHandle;

// 发送通知(从中断)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BUTTON_Pin) {
        osThreadFlagsSet(TargetTaskHandle, 0x01);
    }
}

// 接收通知
void Target_Task(void *argument)
{
    for(;;)
    {
        // 等待通知
        uint32_t flags = osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        
        if (flags & 0x01) {
            printf("Button pressed!\n");
        }
    }
}
```

---

## 5. 同步与互斥

### 5.1 临界区 (Critical Section)

**使用场景:** 保护非常短的代码段(微秒级)

```c
void My_Task(void *argument)
{
    for(;;)
    {
        // 进入临界区 - 关闭中断
        taskENTER_CRITICAL();
        
        // 临界代码 - 必须非常短!
        shared_variable++;
        
        // 退出临界区 - 恢复中断
        taskEXIT_CRITICAL();
        
        osDelay(10);
    }
}
```

**⚠️ 注意:**
- 临界区内不能调用任何 FreeRTOS API
- 临界区内不能调用会阻塞的函数
- 临界区越短越好(< 10us)

### 5.2 调度器锁

**使用场景:** 临时禁止任务切换

```c
void My_Task(void *argument)
{
    for(;;)
    {
        // 挂起调度器 - 不会切换任务,但中断仍然响应
        osKernelLock();
        
        // 执行一系列操作
        operation1();
        operation2();
        operation3();
        
        // 恢复调度器
        osKernelUnlock();
        
        osDelay(10);
    }
}
```

---

## 6. 内存管理

### 6.1 动态内存分配

```c
// 分配内存
void *ptr = pvPortMalloc(100);  // 分配 100 字节

if (ptr != NULL) {
    // 使用内存
    memset(ptr, 0, 100);
    
    // 释放内存
    vPortFree(ptr);
}
```

### 6.2 静态内存分配

```c
// 定义静态缓冲区
static StaticTask_t TaskBuffer;
static StackType_t TaskStack[512];

// 创建任务(静态)
osThreadId_t handle = osThreadNew(
    MyTask_Entry,
    NULL,
    &(osThreadAttr_t){
        .name = "MyTask",
        .cb_mem = &TaskBuffer,
        .cb_size = sizeof(TaskBuffer),
        .stack_mem = TaskStack,
        .stack_size = sizeof(TaskStack),
        .priority = osPriorityNormal,
    }
);
```

### 6.3 内存使用查询

```c
// 获取剩余堆空间
size_t free_heap = xPortGetFreeHeapSize();
printf("Free heap: %u bytes\n", free_heap);

// 获取历史最小剩余堆空间
size_t min_free_heap = xPortGetMinimumEverFreeHeapSize();
printf("Minimum free heap: %u bytes\n", min_free_heap);
```

---

## 7. 中断管理

### 7.1 中断优先级规则

```c
// 在 FreeRTOSConfig.h 中定义
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5

规则:
1. 优先级 0-4:   不能调用 FreeRTOS API (非托管中断)
2. 优先级 5-15:  可以调用 FromISR API (托管中断)
3. SysTick = 15: 最低优先级
```

### 7.2 中断安全 API

**在中断中必须使用 `FromISR` 版本:**

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    CAN_RxData_t rx_data;
    
    // 读取 CAN 数据
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data.data);
    
    // 发送到队列(中断版本)
    xQueueSendFromISR(CANRxQueueHandle, &rx_data, &xHigherPriorityTaskWoken);
    
    // 如果有更高优先级任务被唤醒,立即切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

### 7.3 常用 FromISR API

| 任务版本 | 中断版本 |
|---------|---------|
| `osMessageQueuePut()` | `xQueueSendFromISR()` |
| `osMessageQueueGet()` | `xQueueReceiveFromISR()` |
| `osSemaphoreRelease()` | `xSemaphoreGiveFromISR()` |
| `osThreadFlagsSet()` | `xTaskNotifyFromISR()` |

---

## 8. 实战案例: 云台控制系统

### 8.1 系统架构

```
┌──────────────────────────────────────────────────┐
│                  FreeRTOS 内核                    │
├──────────────┬──────────────┬────────────────────┤
│ INS_Task     │ Motor_Task   │ Robot_Task         │
│ (1000Hz)     │ (500Hz)      │ (200Hz)            │
│              │              │                    │
│ - 姿态解算    │ - 电机控制    │ - 遥控器处理       │
│ - IMU 融合   │ - PID 计算   │ - 视觉数据处理     │
│              │              │ - 指令下发         │
└──────────────┴──────────────┴────────────────────┘
       │              │              │
       ▼              ▼              ▼
┌──────────────────────────────────────────────────┐
│              消息队列 & 信号量                     │
│  - GimbalCmdQueue                                │
│  - GimbalFeedbackQueue                           │
│  - CAN_RxSemaphore                               │
└──────────────────────────────────────────────────┘
```

### 8.2 任务定义

#### 在 CubeMX 中创建任务

```
任务 1: INSTask
  Priority: osPriorityRealtime (48)
  Stack: 1024 Words
  Entry: INS_Task_Entry

任务 2: MotorTask
  Priority: osPriorityHigh (40)
  Stack: 512 Words
  Entry: Motor_Task_Entry

任务 3: RobotTask
  Priority: osPriorityNormal (24)
  Stack: 512 Words
  Entry: Robot_Task_Entry
```

### 8.3 完整代码实现

#### 8.3.1 在 `freertos.c` 中实现任务

```c
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "robot.h"
#include "ins_task.h"
#include "dji_motor.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Definitions for INSTask */
osThreadId_t INSTaskHandle;
const osThreadAttr_t INSTask_attributes = {
  .name = "INSTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for RobotTask */
osThreadId_t RobotTaskHandle;
const osThreadAttr_t RobotTask_attributes = {
  .name = "RobotTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/**
  * @brief  FreeRTOS initialization
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of INSTask */
  INSTaskHandle = osThreadNew(INS_Task_Entry, NULL, &INSTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(Motor_Task_Entry, NULL, &MotorTask_attributes);

  /* creation of RobotTask */
  RobotTaskHandle = osThreadNew(Robot_Task_Entry, NULL, &RobotTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_INS_Task_Entry */
/**
  * @brief  姿态解算任务 - 1000Hz
  * @param  argument: 未使用
  * @retval None
  */
/* USER CODE END Header_INS_Task_Entry */
void INS_Task_Entry(void *argument)
{
  /* USER CODE BEGIN INS_Task_Entry */
  
  // 等待系统启动
  osDelay(100);
  
  /* Infinite loop */
  for(;;)
  {
    // 执行姿态解算
    INS_Task();
    
    // 1000Hz 频率
    osDelay(1);
  }
  
  /* USER CODE END INS_Task_Entry */
}

/* USER CODE BEGIN Header_Motor_Task_Entry */
/**
  * @brief  电机控制任务 - 500Hz
  * @param  argument: 未使用
  * @retval None
  */
/* USER CODE END Header_Motor_Task_Entry */
void Motor_Task_Entry(void *argument)
{
  /* USER CODE BEGIN Motor_Task_Entry */
  
  // 等待系统启动
  osDelay(200);
  
  /* Infinite loop */
  for(;;)
  {
    // 执行电机控制
    DJIMotorControl();
    
    // 500Hz 频率
    osDelay(2);
  }
  
  /* USER CODE END Motor_Task_Entry */
}

/* USER CODE BEGIN Header_Robot_Task_Entry */
/**
  * @brief  机器人主任务 - 200Hz
  * @param  argument: 未使用
  * @retval None
  */
/* USER CODE END Header_Robot_Task_Entry */
void Robot_Task_Entry(void *argument)
{
  /* USER CODE BEGIN Robot_Task_Entry */
  
  // 机器人初始化(只执行一次)
  RobotInit();
  
  /* Infinite loop */
  for(;;)
  {
    // 执行机器人控制逻辑
    RobotTask();
    
    // 200Hz 频率
    osDelay(5);
  }
  
  /* USER CODE END Robot_Task_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  内存分配失败钩子函数
  */
void vApplicationMallocFailedHook(void)
{
    printf("ERROR: Malloc failed!\n");
    while(1);  // 停止运行,等待调试
}

/**
  * @brief  栈溢出检测钩子函数
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    printf("ERROR: Stack overflow in task: %s\n", pcTaskName);
    while(1);  // 停止运行,等待调试
}

/* USER CODE END Application */
```

---

## 9. 调试技巧

### 9.1 打印任务信息

```c
#include "task.h"

void Print_Task_Info(void)
{
    char buffer[512];
    
    printf("=== Task List ===\n");
    printf("Name          State  Priority  Stack  Num\n");
    printf("-------------------------------------------\n");
    
    vTaskList(buffer);
    printf("%s\n", buffer);
    
    printf("\n=== Runtime Stats ===\n");
    printf("Task            Abs Time      %% Time\n");
    printf("-------------------------------------------\n");
    
    vTaskGetRunTimeStats(buffer);
    printf("%s\n", buffer);
}
```

**输出示例:**
```
=== Task List ===
Name          State  Priority  Stack  Num
-------------------------------------------
INSTask       X      48        512    3
MotorTask     B      40        256    2
RobotTask     R      24        128    1
IDLE          R      0         64     4

=== Runtime Stats ===
Task            Abs Time      % Time
-------------------------------------------
INSTask         1250000       50%
MotorTask       625000        25%
RobotTask       500000        20%
IDLE            125000        5%
```

### 9.2 监控堆栈使用

```c
void Monitor_Stack_Usage(void)
{
    // 获取任务剩余栈空间
    UBaseType_t stack_left = uxTaskGetStackHighWaterMark(INSTaskHandle);
    printf("INSTask stack left: %u words\n", stack_left);
    
    // 监控所有任务
    TaskStatus_t task_status[10];
    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    
    uxTaskGetSystemState(task_status, task_count, NULL);
    
    for (int i = 0; i < task_count; i++) {
        printf("%s: %u words left\n", 
               task_status[i].pcTaskName,
               task_status[i].usStackHighWaterMark);
    }
}
```

### 9.3 使用断言

```c
// 在 FreeRTOSConfig.h 中启用
#define configASSERT(x) if((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;); }

// 使用示例
void My_Task(void *argument)
{
    osStatus_t status = osMessageQueueGet(QueueHandle, &data, NULL, 100);
    
    // 断言检查
    configASSERT(status == osOK);
    
    // 继续处理
}
```

---

## 10. 常见问题

### Q1: 任务卡死不运行

**原因:**
- 没有调用 `osDelay()` 让出 CPU
- 被更高优先级任务阻塞

**解决:**
```c
// ❌ 错误 - 任务不让出 CPU
void Bad_Task(void *argument)
{
    for(;;)
    {
        do_something();
        // 没有 osDelay() !
    }
}

// ✅ 正确 - 任务主动让出 CPU
void Good_Task(void *argument)
{
    for(;;)
    {
        do_something();
        osDelay(1);  // 让出 CPU
    }
}
```

### Q2: HardFault 崩溃

**常见原因:**
1. **栈溢出** - 增大任务栈大小
2. **空指针访问** - 检查指针初始化
3. **中断优先级错误** - 检查 NVIC 配置
4. **在中断中调用了非 FromISR API**

**调试方法:**
```c
void HardFault_Handler(void)
{
    printf("HardFault!\n");
    printf("PC = 0x%08X\n", __get_PSP());
    
    while(1);  // 停止等待调试器
}
```

### Q3: 队列满/信号量获取失败

**原因:** 生产者太快,消费者太慢

**解决:**
```c
// 方案 1: 增大队列
osMessageQueueNew(20, sizeof(Data_t), NULL);  // 从 10 增大到 20

// 方案 2: 检查返回值
osStatus_t status = osMessageQueuePut(queue, &data, 0, 100);
if (status == osErrorTimeout) {
    printf("Queue full, data lost!\n");
}

// 方案 3: 加快消费速度
void Consumer_Task(void *argument)
{
    osDelay(5);  // 从 10ms 改为 5ms
}
```

### Q4: 优先级翻转

**现象:** 高优先级任务等待低优先级任务释放资源,被中优先级任务抢占

**解决:** 使用互斥锁(Mutex)而不是信号量

```c
// ❌ 错误 - 使用二值信号量
osSemaphoreAcquire(ResourceSemaphore, osWaitForever);

// ✅ 正确 - 使用互斥锁(自动优先级继承)
osMutexAcquire(ResourceMutex, osWaitForever);
```

### Q5: 内存不足

**检查方法:**
```c
size_t free_heap = xPortGetFreeHeapSize();
printf("Free heap: %u bytes\n", free_heap);

if (free_heap < 1024) {
    printf("WARNING: Low memory!\n");
}
```

**解决方案:**
1. 增大 `TOTAL_HEAP_SIZE` (在 CubeMX 中)
2. 减小任务栈大小
3. 使用静态内存分配
4. 检查内存泄漏

---

## 📚 总结

### 最佳实践

1. **任务设计**
   - ✅ 每个任务只做一件事
   - ✅ 任务必须有延时(`osDelay`)
   - ✅ 合理分配优先级
   - ✅ 栈大小留有余量(至少 20%)

2. **通信机制选择**
   - 数据传递 → 队列 (Queue)
   - 同步通知 → 信号量 (Semaphore)
   - 资源保护 → 互斥锁 (Mutex)
   - 简单通知 → 任务通知 (Task Notification)

3. **中断处理**
   - ✅ 中断要短要快
   - ✅ 使用 FromISR API
   - ✅ 正确配置优先级

4. **调试技巧**
   - ✅ 启用栈溢出检测
   - ✅ 启用内存分配失败钩子
   - ✅ 定期监控栈和堆使用情况
   - ✅ 使用断言检查

### 学习路径

```
第 1 阶段: 基础入门
  ├─ 创建简单任务(LED 闪烁)
  ├─ 学习 osDelay()
  └─ 理解任务优先级

第 2 阶段: 任务通信
  ├─ 使用队列传递数据
  ├─ 使用信号量同步
  └─ 使用互斥锁保护资源

第 3 阶段: 中断处理
  ├─ 中断配置优先级
  ├─ 使用 FromISR API
  └─ 中断与任务协作

第 4 阶段: 实战项目
  ├─ 设计任务架构
  ├─ 优化性能
  └─ 调试和优化
```

---

## 🎯 下一步

现在你已经掌握了 FreeRTOS 的完整使用方法,可以:

1. **实践练习** - 创建 LED 闪烁任务
2. **移植云台控制** - 按照云台移植指南操作
3. **深入学习** - 阅读 FreeRTOS 官方文档
4. **性能优化** - 分析任务执行时间和 CPU 占用率

祝学习顺利! 🚀
