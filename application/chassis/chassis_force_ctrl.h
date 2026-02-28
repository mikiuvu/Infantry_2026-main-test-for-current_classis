/**
 * @file chassis_force_ctrl.h
 * @brief 力控底盘头文件
 * @note 通过在robot_def.h中定义USE_FORCE_CONTROL_CHASSIS宏启用
 * 
 * 力控底盘特性:
 * - 力矩前馈 + 软速度环兜底
 * - 在线自适应参数估计
 * - 滑模控制 + 扰动观测器(DOB)
 * - 摩擦系数在线估计
 * 
 * @version 1.0
 * @date 2026-03-01
 */

#ifndef CHASSIS_FORCE_CTRL_H
#define CHASSIS_FORCE_CTRL_H

#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "bsp_buzzer.h"

/* ===================== 车辆物理参数 ===================== */
// 这些参数需要根据实际机器人测量/标定
#define FC_CHASSIS_MASS          15.0f      // 整车质量 (kg)
#define FC_CHASSIS_INERTIA_Z     0.8f       // Z轴转动惯量 (kg*m²)
#define FC_WHEEL_RADIUS          0.076f     // 轮子半径 (m)
#define FC_WHEEL_BASE_L          0.2f       // 轮距/2 (m), 轮子到中心距离

/* ===================== 电机力矩参数 ===================== */
// M3508: k_t = 0.3 N*m/A, 减速比19.2, 轮径0.076m
// k_total = k_t / 减速比 / R_wheel = 0.3 / 19.2 / 0.076 ≈ 0.00206 N/A
// 考虑效率损失(~90%), 初始值取 0.00185 N/A
#define FC_KT_INITIAL            0.00185f   // 初始力矩常数 (N/A)
#define FC_KT_MIN                0.0010f    // 最小允许值 (约55%标称)
#define FC_KT_MAX                0.0025f    // 最大允许值 (约135%标称)

/* ===================== 电机摩擦模型参数 ===================== */
// 电机模型: τ_m = K_t * I - τ_c * sign(ω) - b * ω
// 这些参数需要通过实验标定:
// 1. τ_c: 使用拉力计测量启动电流, τ_c = K_t * I_startup
// 2. b: 使用降速法测量或稳态运行测量
// 注意: 这里的参数已换算到轮子输出力 (N)
#define FC_TAU_C_INITIAL         0.8f       // 库仑摩擦力 (N), 需实测标定
#define FC_VISCOUS_B_INITIAL     0.02f      // 粘性摩擦系数 (N/(rad/s)), 需实测标定
#define FC_FRICTION_COMP_ENABLE  1          // 1=启用摩擦补偿, 0=禁用

/* ===================== 自适应控制参数 ===================== */
#define FC_ADAPTIVE_GAMMA        0.00001f   // 自适应增益
#define FC_ADAPTIVE_I_THRESHOLD  2.0f       // 电流阈值 (A), 低于此值不更新

/* ===================== 滑模控制参数 ===================== */
#define FC_SMC_LAMBDA            2.0f       // 滑模面斜率
#define FC_SMC_ETA               50.0f      // 趋近律增益
#define FC_SMC_PHI               0.3f       // 边界层厚度 (m/s)

/* ===================== 扰动观测器参数 ===================== */
#define FC_DOB_L                 50.0f      // 观测器增益/带宽
#define FC_DOB_D_MAX             75.0f      // 最大扰动估计 (N), 约0.5g

/* ===================== 摩擦估计参数 ===================== */
#define FC_MU_INITIAL            0.85f      // 初始摩擦效率
#define FC_MU_MIN                0.30f      // 最小摩擦效率 (湿滑地面)
#define FC_MU_FILTER_ALPHA       0.02f      // 滤波系数

/* ===================== 软速度环参数 ===================== */
#define FC_SOFT_SPEED_KP         1.0f       // 软速度环比例增益
#define FC_SOFT_SPEED_KI         0.1f       // 软速度环积分增益
#define FC_SOFT_SPEED_MAX_OUT    5000.0f    // 软速度环最大输出

/* ===================== 数据结构定义 ===================== */

/**
 * @brief 底盘力指令结构体
 */
typedef struct {
    float Fx;       // X方向目标力 (N)
    float Fy;       // Y方向目标力 (N)
    float Mz;       // Z轴目标力矩 (N*m)
} ChassisForceCmd_t;

/**
 * @brief 在线自适应参数估计结构体
 */
typedef struct {
    float kt_estimate;      // 估计的力矩常数 (N/A)
    float kt_min;           // 最小允许值
    float kt_max;           // 最大允许值
    float gamma;            // 自适应增益
} AdaptiveParam_t;

/**
 * @brief 电机摩擦模型参数结构体
 * @note 模型: τ_m = K_t * I - τ_c * sign(ω) - b * ω
 */
typedef struct {
    float tau_c;            // 库仑摩擦力 (N)
    float viscous_b;        // 粘性摩擦系数 (N/(rad/s))
    uint8_t enable;         // 是否启用摩擦补偿
} MotorFrictionModel_t;

/**
 * @brief 滑模控制器结构体
 */
typedef struct {
    float lambda;           // 滑模面斜率
    float eta;              // 趋近律增益
    float phi;              // 边界层厚度
    float disturbance;      // 估计的扰动 (由DOB提供)
    float v_err_last;       // 上一周期速度误差 (用于微分)
} SlidingModeCtrl_t;

/**
 * @brief 扰动观测器结构体
 */
typedef struct {
    float z;                // 观测器内部状态
    float d_hat;            // 估计扰动 (N)
    float L;                // 观测器增益
} DisturbanceObserver_t;

/**
 * @brief 摩擦系数在线估计结构体
 */
typedef struct {
    float mu_estimate;      // 当前估计摩擦效率
    float mu_nominal;       // 标称值
    float mu_min;           // 最小值
    float filter_alpha;     // 滤波系数
    uint32_t sample_count;  // 有效样本计数
} FrictionEstimator_t;

/**
 * @brief 力控调试数据结构体 (用于VOFA输出)
 */
typedef struct {
    // 速度跟踪
    float vx_ref;           // 目标速度X (m/s)
    float vy_ref;           // 目标速度Y (m/s)
    float vx_act;           // 实际速度X (m/s)
    float vy_act;           // 实际速度Y (m/s)
    
    // 力控输出
    float Fx_cmd;           // 目标力X (N)
    float Fy_cmd;           // 目标力Y (N)
    
    // 自适应
    float kt_estimate;      // kt估计值
    
    // 扰动
    float dx_hat;           // X扰动估计 (N)
    float dy_hat;           // Y扰动估计 (N)
    
    // 摩擦
    float mu_estimate;      // 摩擦效率
    
    // 摩擦补偿
    float friction_comp[4]; // 四轮摩擦补偿电流 (LF, RF, LB, RB)
    
    // 电流前馈
    float current_ff[4];    // 四轮电流前馈 (LF, RF, LB, RB)
} ForceCtrlDebug_t;

/* ===================== 函数声明 ===================== */

/**
 * @brief 力控底盘初始化
 * @note 在RobotInit()中调用, 替代ChassisInit()
 */
void ChassisForceCtrlInit(void);

/**
 * @brief 力控底盘任务
 * @note 在FreeRTOS任务中周期调用, 替代ChassisTask()
 */
void ChassisForceCtrlTask(void);

/**
 * @brief 获取力控调试数据
 * @return 调试数据结构体指针
 */
const ForceCtrlDebug_t* GetForceCtrlDebugData(void);

/**
 * @brief 获取自适应参数
 * @return 自适应参数结构体指针
 */
const AdaptiveParam_t* GetAdaptiveParam(void);

/**
 * @brief 获取摩擦估计数据
 * @return 摩擦估计结构体指针
 */
const FrictionEstimator_t* GetFrictionEstimator(void);

/**
 * @brief 获取电机摩擦模型参数
 * @return 电机摩擦模型结构体指针
 */
const MotorFrictionModel_t* GetMotorFrictionModel(void);

/**
 * @brief 设置电机摩擦模型参数 (用于标定后更新)
 * @param tau_c 库仑摩擦力 (N)
 * @param viscous_b 粘性摩擦系数 (N/(rad/s))
 */
void SetMotorFrictionModel(float tau_c, float viscous_b);

/**
 * @brief 重置自适应参数到初始值
 */
void ResetAdaptiveParam(void);

/**
 * @brief 重置摩擦估计到初始值
 */
void ResetFrictionEstimator(void);

#endif // CHASSIS_FORCE_CTRL_H
