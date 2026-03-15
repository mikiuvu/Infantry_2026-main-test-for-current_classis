# CODEX Project Guide

这是一份面向后续代码协作的项目速读文档，目标是让我在再次进入这个仓库时，能快速恢复上下文、定位入口，并优先找到常改模块。

## 1. 项目定位

- 项目类型：STM32F407 RoboMaster 电控固件
- 运行环境：HAL + FreeRTOS
- 代码分层：`bsp` / `modules` / `application`
- 当前主控配置：`application/robot_def.h` 中启用了 `GIMBAL_BOARD`
- 当前视觉协议：`VISION_USE_VCP`

这意味着当前编译目标偏向云台板逻辑；若要切到底盘板、底盘单板调试或力控底盘，需要先改 `robot_def.h` 里的板级宏，再重新编译。

## 2. 程序主入口

程序启动主链路如下：

1. `Src/main.c`
2. `HAL_Init() / 时钟初始化 / 外设初始化`
3. `RobotInit()`
4. `MX_FREERTOS_Init()`
5. `osKernelStart()`

其中真正的应用层总入口是：

- `application/robot.c`
  - `RobotInit()`：统一初始化 BSP 和各 APP
  - `RobotTask()`：统一调度各 APP 的周期任务

## 3. FreeRTOS 任务分工

在 `Src/freertos.c` 中，当前关键任务如下：

- `StartINSTASK()`：1 kHz，IMU 解算，并周期发送视觉数据
- `StartMOTORTASK()`：500 Hz，执行 `MotorControlTask()`
- `StartDAEMONTASK()`：100 Hz，执行离线监测等守护逻辑
- `StartROBOTTASK()`：500 Hz，执行 `RobotTask()`
- `StartUITASK()`：裁判系统交互/UI 刷新

理解这个工程时，可以把它看成两条主线：

- 控制计算主线：`RobotTask()`
- 执行器闭环主线：`MotorControlTask()`

## 4. 分层理解

### `bsp`

对 HAL 外设做二次封装，提供更稳定的板级接口，例如：

- `bsp/can`
- `bsp/usart`
- `bsp/spi`
- `bsp/gpio`
- `bsp/dwt`

如果只是改机器人行为逻辑，通常不应先动 `bsp`。

### `modules`

提供可复用功能模块，典型包括：

- 电机：`modules/motor`
- IMU：`modules/imu`
- 消息中心：`modules/message_center`
- 裁判系统：`modules/referee`
- 遥控器：`modules/remote`
- 上下位机通信：`modules/master_machine`
- 超级电容：`modules/super_cap`

应用层大量依赖这些模块，但尽量不直接碰 HAL。

### `application`

机器人具体行为定义层，当前最关键的应用有：

- `application/cmd`：控制指令汇总与分发
- `application/gimbal`：云台控制
- `application/chassis`：底盘控制
- `application/shoot`：发射机构
- `application/robot.c`：总装配入口

## 5. 应用层数据流

建议把应用层理解成“并列应用 + 消息中心”结构，而不是彼此直接调用。

核心关系如下：

1. `robot_cmd` 读取遥控器、键鼠、视觉等输入
2. `robot_cmd` 整理为底盘/云台/发射控制指令
3. 通过 `message_center` 发布给 `chassis`、`gimbal`、`shoot`
4. 各应用完成本地解算后，再发布反馈数据

因此后续排查控制异常时，优先分清问题属于哪一层：

- 输入源异常
- `robot_cmd` 模式/映射异常
- 对应应用自身解算异常
- motor task 闭环输出异常

## 6. 板级配置重点

核心配置文件是：

- `application/robot_def.h`

这里集中定义了：

- 开发板类型
- 视觉协议
- 机器人类型
- 云台机械参数
- 底盘几何参数
- IMU 安装偏移与方向校正
- 功率、自旋、平移等控制参数

后续如果出现“行为不对但代码流程没问题”的情况，优先检查这里，例如：

- `YAW_CHASSIS_ALIGN_ECD`
- `WHEEL_BASE`
- `TRACK_WIDTH`
- `RADIUS_WHEEL`
- `CHASSIS_IMU_OFFSET_X`
- `CHASSIS_IMU_OFFSET_Y`

## 7. 底盘模块速读

当前底盘核心文件：

- `application/chassis/chassis.c`
- `application/chassis/chassis.h`
- `application/chassis/chassis.md`
- `application/chassis/力控底盘技术方案.md`
- `application/chassis/chassis_force_ctrl.c`

`chassis.c` 当前可归纳为几部分：

1. 底盘电机初始化
2. 超级电容与裁判系统接入
3. IMU 数据接入
4. `message_center` 或双板 CAN 通信接入
5. 麦轮/底盘运动学解算
6. 功率限制
7. 速度估计与反馈上报
8. 一些增强控制逻辑

从源码上看，这个底盘文件已经不仅是“基础麦轮控制”，还叠加了较多实验性/增强型逻辑，包括：

- 速度融合 Kalman
- SMC / ESO / DOB 相关控制变量
- 打滑检测相关变量
- 电流前馈
- IMU 偏移补偿

所以后续修改 `chassis.c` 时，建议先分清是在改哪一层：

- 控制输入映射
- 运动学
- 估计器
- 控制器
- 功率/安全限制
- 反馈发布

## 8. 当前我建议的阅读顺序

如果后续继续改这个仓库，我会优先按下面顺序读：

1. `application/robot_def.h`
2. `application/robot.c`
3. `Src/freertos.c`
4. `application/cmd/robot_cmd.c`
5. `application/chassis/chassis.c`
6. `application/gimbal/gimbal.c`
7. `application/shoot/shoot.c`
8. `modules/message_center/message_center.c`
9. `modules/motor/motor_task.c`
10. 对应模块的 `.md` 说明文件

## 9. 常用定位入口

### 想改底盘行为

先看：

- `application/cmd/robot_cmd.c`
- `application/chassis/chassis.c`
- `application/robot_def.h`

### 想改云台行为

先看：

- `application/cmd/robot_cmd.c`
- `application/gimbal/gimbal.c`
- `application/robot_def.h`

### 想改发射机构

先看：

- `application/cmd/robot_cmd.c`
- `application/shoot/shoot.c`
- `application/robot_def.h`

### 想排查“电机不转 / 输出不对”

先看：

- `application/...` 对应应用是否写入目标值
- `modules/motor/motor_task.c`
- `modules/motor/...`
- `modules/motor_offline_alarm/...`
- `bsp/can/...`

## 10. 仓库内已有高价值文档

除了这份文档，后续继续接手时优先参考这些已有说明：

- `README.md`
- `application/application.md`
- `application/cmd/robot_cmd.md`
- `application/chassis/chassis.md`
- `application/gimbal/gimbal.md`
- `application/shoot/shoot.md`
- `application/APP层应用编写指引.md`
- `VSCode+Ozone使用方法.md`

## 11. 协作备注

- 当前工作区不是干净状态，已有若干未提交改动，尤其包括 `application/chassis/chassis.c`、`application/robot_def.h` 等核心文件。
- 后续继续修改前，需要先注意不要误覆盖这些在研内容。
- 若要让我继续协作，直接告诉我“基于这份 guide，从底盘/云台/发射继续看”，我可以按这份文档继续往下接。
