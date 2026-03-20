/**
 * @file chassis_force_ctrl.h
 * @brief 纯力矩控制底盘 (Super-Twisting SMC + ESO + 阻力前馈 + 加速度前馈)
 * @note 通过在robot_def.h中定义FORCE_CONTROL_CHASSIS_BOARD宏启用
 * 
 * 控制架构 (每轮独立, 无PID, 无软速度环):
 * 
 *   u = u_ff + u_drag + u_smc + u_eso
 * 
 *   u_ff   = k_ff * dv_ref/dt                         (加速度前馈, 提升响应)
 *   u_drag = (a0 + a1|w| + a2|w|² + a3|w|³) * sign(w)  (阻力前馈, 抵消损耗)
 *   u_smc  = Super-Twisting(s)                         (二阶滑模, 连续输出无抖振)
 *   u_eso  = -d_hat                                    (ESO扰动补偿)
 *   s      = v_ref - w                                (滑模面 = 速度误差)
 * 
 * Super-Twisting算法:
 *   u1 = -k1 * sqrt(|s|) * sign(s)   (比例项)
 *   σ̇  = -k2 * sign(s)              (积分项动态)
 *   u_smc = u1 + σ                   (连续控制输出)
 * 
 * ESO (扩张状态观测器):
 *   e = w - z1
 *   ż1 = b0 * (u_applied - u_drag) + z2 + β1 * e
 *   ż2 = β2 * e
 *   d_hat = z2 / b0                  (估计扰动→控制输入补偿)
 * 
 * 底盘速度观测: IMU加速度 + 轮速卡尔曼融合, 打滑检测动态调整R值.
 * 电机配置为 OPEN_LOOP, DJIMotorSetRef() 直接输出 C620 电流指令.
 * 
 * @version 3.0
 * @date 2026-03-02
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
#include "ui_interface.h"
#include "arm_math.h"
#include "bsp_buzzer.h"

/* ===================== Super-Twisting 滑模参数 ===================== */
// Super-Twisting: u1 = -k1*|s|^0.5*sign(s), dσ/dt = -k2*sign(s)
// u_smc = u1 + σ
// 稳定性条件: k2 > 0, k1 > sqrt(2*k2)
// sign(s) 使用 SmoothSign(s, phi) 平滑, phi 越大越平滑
#define FC_ST_K1                 15.0f      // ST比例增益 (决定收敛速度)
#define FC_ST_K2                 30.0f      // ST积分增益 (补偿不确定性)
#define FC_ST_PHI                0.3f       // SmoothSign边界层 (deg/s)
#define FC_ST_SIGMA_MAX          5000.0f    // 积分项限幅 (防止饱和)

/* ===================== ESO 扩张状态观测器参数 ===================== */
// 二阶ESO: 观测 [速度, 总扰动]
// 带宽 ω_o 决定观测速度, β1 = 2*ω_o, β2 = ω_o^2
// b0: 控制效能系数, 表示单位控制输入(C620单位)产生多少 deg/s² 的加速度
// b0 估算: 20A对应16384单位, kt=0.3Nm/A, 减速比15.7, 轮惯量J_wheel
//          b0 = (kt * 减速比) / (J_eq * 16384/20) ≈ 需要实测调节
#define FC_ESO_OMEGA             50.0f      // ESO带宽 (rad/s), 越大响应越快但噪声越大
#define FC_ESO_BETA1             (2.0f * FC_ESO_OMEGA)           // = 100
#define FC_ESO_BETA2             (FC_ESO_OMEGA * FC_ESO_OMEGA)   // = 2500
#define FC_ESO_B0                0.5f       // 控制效能系数 (C620单位 → deg/s²), 需实测调节
#define FC_ESO_D_MAX             10000.0f   // 最大扰动估计限幅 (防止发散)

/* ===================== 加速度前馈参数 ===================== */
// u_ff = k_ff * (v_ref(k) - v_ref(k-1)) / dt
#define FC_K_FF                  1.2f       // 前馈增益

/* ===================== 阻力前馈模型参数 ===================== */
// 模型: drag(|w|) = a0 + a1*|w| + a2*|w|^2 + a3*|w|^3
// w 单位: 电机轴速度 speed_aps (deg/s)
// drag 输出单位: C620 电流指令
//
// 标定方法:
//   1. 使用普通速度PID底盘(CHASSIS_BOARD), 让底盘稳定平动
//   2. 记录各电机 speed_aps 与 real_current (稳态电流)
//   3. 多组速度点拟合: I_steady = a0 + a1*|w| + a2*|w|^2 + a3*|w|^3
//   4. 将拟合参数填入下方宏, 切换到力控模式
//
// 物理意义:
//   a0: 库仑摩擦 (静摩擦 + 低速阻力)
//   a1: 粘滞摩擦系数
//   a2: 二次阻力系数 (风阻等)
//   a3: 三次阻力系数 (高速非线性)
#define FC_A0                    0.15f      // 库仑摩擦 (C620单位)
#define FC_A1                    0.02f      // 粘滞摩擦系数
#define FC_A2                    0.0005f    // 二次阻力系数
#define FC_A3                    0.0f       // 三次阻力系数 (标定后填入)

/* ===================== 输出限幅 ===================== */
#define FC_MAX_TORQUE            16000.0f   // 最大输出 (C620: ±16384)

/* ===================== 打滑检测参数 ===================== */
#define FC_SLIP_THRESHOLD        30.0f      // 打滑判定阈值 (mm/s 速度变化差)
#define FC_SLIP_SPEED_MIN        100.0f     // 最小轮速才检测 (mm/s, 避免静止误判)

/* ===================== 底盘IMU偏移 ===================== */
#define FC_IMU_OFFSET_X          CHASSIS_IMU_OFFSET_X   // mm
#define FC_IMU_OFFSET_Y          CHASSIS_IMU_OFFSET_Y   // mm

/* ===================== 单位转换 ===================== */
#define WHEEL_MMPS_TO_MOTOR_APS  (REDUCTION_RATIO_WHEEL / ((float)RADIUS_WHEEL * DEGREE_2_RAD))

/* ===================== 数据结构定义 ===================== */

/**
 * @brief 单轮ESO状态
 */
typedef struct {
    float z1;               // 观测速度 (deg/s)
    float z2;               // 观测扰动 (内部单位: deg/s²)
    float d_hat;            // 输出扰动补偿 (C620单位): = z2 / b0
} WheelESO_t;

/**
 * @brief 单轮Super-Twisting滑模控制器状态
 */
typedef struct {
    float sigma;            // 积分项状态
} WheelST_SMC_t;

/**
 * @brief 单轮力矩控制器完整状态
 */
typedef struct {
    // Super-Twisting SMC
    WheelST_SMC_t  st;     // ST状态
    // ESO
    WheelESO_t     eso;    // ESO状态
    // 前馈
    float v_ref_last;       // 上一时刻目标速度 (deg/s)
    // 各项输出 (用于调试)
    float u_ff;             // 加速度前馈
    float u_drag;           // 阻力前馈
    float u_smc;            // Super-Twisting输出
    float u_eso;            // ESO补偿
    float u_total;          // 总输出 (限幅后)
    float s;                // 滑模面
} WheelTorqueCtrl_t;

/**
 * @brief 打滑检测 (每轮)
 */
typedef struct {
    uint8_t is_slipping;
    float deviation;        // 打滑指标 (mm/s)
    float expected_dv;      // IMU预测速度变化 (mm/s)
    float actual_dv;        // 实际速度变化 (mm/s)
    float last_speed;       // 上一帧轮速 (mm/s)
} WheelSlip_t;

/**
 * @brief 力矩控制调试数据 (用于VOFA输出)
 * @note 数组下标: [0]=LF, [1]=RF, [2]=LB, [3]=RB
 */
typedef struct {
    // 每轮速度跟踪
    float v_ref[4];         // 目标速度 (deg/s)
    float v_act[4];         // 实际速度 (deg/s)
    float s[4];             // 滑模面 (deg/s)
    // 每轮控制输出分量
    float u_ff[4];          // 加速度前馈
    float u_drag[4];        // 阻力前馈
    float u_smc[4];         // Super-Twisting输出
    float u_eso[4];         // ESO扰动补偿
    float u_total[4];       // 总输出 (限幅后)
    // ESO状态
    float eso_d_hat[4];     // 各轮ESO扰动估计
    // 底盘状态
    float chassis_vx;       // 卡尔曼融合速度X (mm/s)
    float chassis_vy;       // 卡尔曼融合速度Y (mm/s)
} ForceCtrlDebug_t;

/* ===================== 函数声明 ===================== */

void ChassisForceCtrlInit(void);
void ChassisForceCtrlTask(void);
const ForceCtrlDebug_t* GetForceCtrlDebugData(void);

#endif // CHASSIS_FORCE_CTRL_H
