/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
//#define CHASSIS_BOARD                 // 底盘板 (速控底盘)
//#define CHASSIS_ONLY                  // 底盘调试模式: 无云台,只有底盘+超级电容+遥控器 
//#define FORCE_CONTROL_CHASSIS_BOARD   // 力控底盘板
#define GIMBAL_BOARD                    // 云台板

/* 遥控器类型选择: 定义USE_IMAGE_REMOTE使用图传遥控器(UART6), 注释掉则使用原DJI遥控器(USART3/DBUS) */
//#define USE_IMAGE_REMOTE

// 视觉通信协议选择,只能开一个

#define VISION_USE_VCP          // USB虚拟串口
// #define VISION_USE_UART         // 串口+seasky协议
// #define VISION_USE_SERIALPORT   // HUST上位机
//#define VISION_USE_SP               // SP协议,支持前馈

// 协议兼容性检查 - 确保只定义了一个协议
#if (defined(VISION_USE_VCP) + defined(VISION_USE_UART) + defined(VISION_USE_SERIALPORT) + defined(VISION_USE_SP)) > 1
#error "Error: Multiple vision protocols defined! Please select only ONE protocol."
#endif

#if !defined(VISION_USE_VCP) && !defined(VISION_USE_UART) && !defined(VISION_USE_SERIALPORT) && !defined(VISION_USE_SP)
#error "Error: No vision protocol defined! Please select one protocol."
#endif

//#define USE_LASER_POINTER  // 启用激光笔模式,注释掉此行则使用完整发射机构

// @todo: 增加机器人类型定义,后续是否要兼容所有机器人?(只兼容步兵英雄哨兵似乎就够了)
// 通过该宏,你可以直接将所有机器人的参数保存在一处,然后每次只需要修改这个宏就可以替换所有参数
/* 机器人类型定义 */
// #define ROBOT_HERO 1     // 英雄机器人
// #define ROBOT_ENINEER 2  // 工程机器人
#define ROBOT_INFANTRY 3 // 步兵机器人3
// #define ROBOT_INFANTRY 4 // 步兵机器人4
// #define ROBOT_INFANTRY 5 // 步兵机器人5
// #define ROBOT_SENTRY 6   // 哨兵机器人

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_ALIGN_ECD         0 //0    //云台和底盘对齐指向相同方向时的yaw的差值,需要测量
#define YAW_CHASSIS_ALIGN_ECD 2730  //步兵一  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
//#define YAW_CHASSIS_ALIGN_ECD 2030  //步兵二
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3625       // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 0 //云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度) 
#define PITCH_MIN_ANGLE 0 //云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

/* ======================== 云台软件限位 ======================== */
#define PITCH_MIN_LIMIT         -20.0f  // pitch最小角度(°)
#define PITCH_MAX_LIMIT         12.0f   // pitch最大角度(°)

// 拨盘堵转检测与反转参数
#define BLOCK_DETECT_THRESHOLD 0.80f   // 堵转判定误差阈值(误差/目标值)
#define BLOCK_DETECT_COUNT 200         // 堵转判定次数
#define BLOCK_TIME_RECORD_COUNT 20     // 记录堵转时间戳的计数阈值，不建议修改 20
#define REVERSE_DURATION_MS 500        // 反转持续时间(ms)
#define REVERSE_ANGLE_RATIO 1.0f       // 反转角度系数，不建议修改 1.0

// 发射参数
#define ONE_BULLET_DELTA_ANGLE 45   // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 36.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 8             // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 340              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 340             // 横向轮距(左右平移方向)
#define WHEEL_DIAGONAL (sqrt((double)(WHEEL_BASE^2) + (double)(TRACK_WIDTH^2))/2)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 75             // 轮子半径
#define REDUCTION_RATIO_WHEEL 15.7f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

/* ======================== 底盘IMU配置 ======================== */
// IMU安装位置偏移 (相对于底盘几何中心, 单位mm)
// 正方向: X轴指向机器人前方, Y轴指向机器人左侧
#define CHASSIS_IMU_OFFSET_X     -185.0f   // IMU相对底盘中心的X偏移 (mm), 正值表示IMU在前
#define CHASSIS_IMU_OFFSET_Y     4.1f   // IMU相对底盘中心的Y偏移 (mm), 正值表示IMU在左

/* ======================== IMU安装方向校正 ======================== */
// 使用ins_task中的IMU_Param_Correction进行方向校正
// Yaw/Pitch/Roll: IMU安装相对于机体系的偏角 (度)
// scale: 陀螺仪标度因数校正 (默认1.0)

#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY) || defined(FORCE_CONTROL_CHASSIS_BOARD)
// 底盘板IMU安装方向校正 (速控/力控/调试模式通用)
#define IMU_PARAM_YAW        0.0f    // IMU Yaw偏角 (°)
#define IMU_PARAM_PITCH      0.0f    // IMU Pitch偏角 (°)
#define IMU_PARAM_ROLL       0.0f    // IMU Roll偏角 (°)
#define IMU_PARAM_SCALE_X    1.0f    // 陀螺仪 X轴标度因数
#define IMU_PARAM_SCALE_Y    1.0f    // 陀螺仪 Y轴标度因数
#define IMU_PARAM_SCALE_Z    1.0f    // 陀螺仪 Z轴标度因数
#endif

#ifdef GIMBAL_BOARD
// 云台板IMU安装方向校正
#define IMU_PARAM_YAW        0.0f    // IMU Yaw偏角 (°)
#define IMU_PARAM_PITCH      0.0f    // IMU Pitch偏角 (°)
#define IMU_PARAM_ROLL       0.0f    // IMU Roll偏角 (°)
#define IMU_PARAM_SCALE_X    1.0f    // 陀螺仪 X轴标度因数
#define IMU_PARAM_SCALE_Y    1.0f    // 陀螺仪 Y轴标度因数
#define IMU_PARAM_SCALE_Z    1.0f    // 陀螺仪 Z轴标度因数
#endif


/* ======================== 云台前馈参数 ======================== */
// 前馈在gimbal中本地计算
// 加速度-电流转换系数
#define YAW_ACC_TO_CURRENT      1.0f    // yaw加速度(°/s²)转电流系数
#define PITCH_ACC_TO_CURRENT    1.0f    // pitch加速度(°/s²)转电流系数

/* ======================== Pitch重力补偿参数 ======================== */
#define GRAVITY_COMP_MAX        -9200.0f  // 最大重力补偿电流值(水平时)
#define PITCH_HORIZONTAL_ANGLE  -1.2f      // pitch水平时的IMU角度(°)

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(CHASSIS_BOARD) + defined(GIMBAL_BOARD) + defined(CHASSIS_ONLY) + defined(FORCE_CONTROL_CHASSIS_BOARD)) != 1
#error Conflict board definition! You must define exactly ONE board type (CHASSIS_BOARD, GIMBAL_BOARD, CHASSIS_ONLY, or FORCE_CONTROL_CHASSIS_BOARD).
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 1, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_4_BULLET, // 四发
    LOAD_5_BULLET, // 五发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

typedef enum
{
    UI_KEEP,
    UI_REFRESH,
} ui_mode_e;
// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;
typedef enum
{ // 超电启停
    CAP_OFF,
    CAP_ON,
} SuperCap_Mode_e;

// 自瞄模式设置
typedef enum
{
    AIM_OFF = 0,  // 开启自瞄模式
    AIM_ON,   
} aim_mode_e;

typedef enum
{
    DASH_OFF = 0,  // 开启自瞄模式
    DASH_ON,   
} dash_mode_e;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度, 单位: degree/s (电机转子角速度)
    float vy;           // 横移方向速度, 单位: degree/s (电机转子角速度)
    float wz;           // 旋转速度, 单位: degree/s
    float offset_angle; // 底盘和归中位置的夹角, 单位: degree
    chassis_mode_e chassis_mode;
    float chassis_power_buff;
    dash_mode_e dash_mode; // 冲刺模式

    // UI部分
    ui_mode_e ui_mode;
    gimbal_mode_e gimbal_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    loader_mode_e load_mode;
    SuperCap_Mode_e super_cap;
    float pitch_angle;
    aim_mode_e aim_mode;

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
// 前馈现在在gimbal中本地计算 (微分目标角度), 不再通过消息传递
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

/* @todo : 对于平衡底盘,需要新增控制模式和控制数据 */
typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    attitude_t chassis_imu_data;
#endif
    // 底盘真实速度 (用于速度融合和打滑检测), 单位: mm/s (轮子线速度)
     float real_vx;
     float real_vy;
     float real_wz;  // 单位: degree/s

    uint8_t rest_heat;           // 剩余枪口热量
    float bullet_speed;          // 实时弹速
    Detect_Color_e self_color;   // 0 for blue, 1 for red
    uint8_t robot_level;

} Chassis_Upload_Data_s;

/* @todo : 对于平衡底盘,需要不同的反馈数据 */
typedef struct
{
    attitude_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct
{
    
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
