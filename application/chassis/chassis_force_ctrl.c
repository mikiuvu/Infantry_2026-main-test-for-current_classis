/**
 * @file chassis_force_ctrl.c
 * @brief 力控底盘实现
 * @note 通过在robot_def.h中定义USE_FORCE_CONTROL_CHASSIS宏启用
 * 
 * 控制架构:
 *   目标速度 ──> 鲁棒力控制器 ──> 目标力 ──> 摩擦限幅 ──> 力分配 ──> 电流前馈 ──┐
 *                (滑模+DOB)             (自适应)    (逆运动学)  (自适应kt)    │
 *                                                                           │
 *   目标速度 ──> 逆运动学 ──> 各轮目标速度 ──> 软速度环 ──────────────────────┤
 *                                                                           │
 *                                              电流指令 = 速度环输出 + 电流前馈 ──> 电机
 * 
 * @version 1.0
 * @date 2026-03-01
 */

#include "chassis_force_ctrl.h"
#include "user_lib.h"
#include "bsp_rng.h"
#include "bsp_dwt.h"
#include "controller.h"
#include "vofa.h"
#include "simple_kalman.h"
#include "dji_motor_offline_alarm.h"

#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
#include "ins_task.h"
#endif

#ifdef CHASSIS_BOARD
#include "can_comm.h"
#endif

/* ===================== 模块实例 ===================== */
#ifdef CHASSIS_BOARD
static CANCommInstance *chasiss_can_comm;
#endif

#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
static INS_t *Chassis_INS_data;
#endif

#ifdef CHASSIS_ONLY
static Publisher_t *chassis_pub;
static Subscriber_t *chassis_sub;
#endif

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;
static Chassis_Upload_Data_s chassis_feedback_data;

static referee_info_t* referee_data;
static Referee_Interactive_info_t ui_data;

static SuperCapInstance *cap;
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;
static MotorOfflineAlarmInstance *chassis_offline_alarm = NULL;

/* ===================== 速度融合卡尔曼滤波器 ===================== */
static SimpleKalman1D_t kf_vx, kf_vy;

/* ===================== 力控核心数据结构 ===================== */
// 力指令
static ChassisForceCmd_t force_cmd = {0};

// 电流前馈值
static float current_ff_lf = 0.0f;
static float current_ff_rf = 0.0f;
static float current_ff_lb = 0.0f;
static float current_ff_rb = 0.0f;

// 在线自适应参数
static AdaptiveParam_t adaptive = {
    .kt_estimate = FC_KT_INITIAL,
    .kt_min = FC_KT_MIN,
    .kt_max = FC_KT_MAX,
    .gamma = FC_ADAPTIVE_GAMMA,
};

// 二阶滑模控制器 (X/Y方向各一个, Super-Twisting算法)
static SlidingModeCtrl_t smc_vx = {
    .lambda = FC_SMC_LAMBDA,
    .st_k1 = FC_SMC_K1,
    .st_k2 = FC_SMC_K2,
    .phi = FC_SMC_PHI,
    .v_integral = 0.0f,
    .v_max = FC_SMC_V_MAX,
    .disturbance = 0.0f,
    .v_err_last = 0.0f,
    .s_last = 0.0f,
};
static SlidingModeCtrl_t smc_vy = {
    .lambda = FC_SMC_LAMBDA,
    .st_k1 = FC_SMC_K1,
    .st_k2 = FC_SMC_K2,
    .phi = FC_SMC_PHI,
    .v_integral = 0.0f,
    .v_max = FC_SMC_V_MAX,
    .disturbance = 0.0f,
    .v_err_last = 0.0f,
    .s_last = 0.0f,
};

// 扰动观测器 (X/Y方向各一个)
static DisturbanceObserver_t dob_x = {.z = 0, .d_hat = 0, .L = FC_DOB_L};
static DisturbanceObserver_t dob_y = {.z = 0, .d_hat = 0, .L = FC_DOB_L};

// 摩擦系数估计
static FrictionEstimator_t friction = {
    .mu_estimate = FC_MU_INITIAL,
    .mu_nominal = FC_MU_INITIAL,
    .mu_min = FC_MU_MIN,
    .filter_alpha = FC_MU_FILTER_ALPHA,
    .sample_count = 0,
};

// 电机摩擦模型参数
static MotorFrictionModel_t motor_friction = {
    .tau_c = FC_TAU_C_INITIAL,
    .viscous_b = FC_VISCOUS_B_INITIAL,
    .enable = FC_FRICTION_COMP_ENABLE,
};

// 调试数据
static ForceCtrlDebug_t debug_data = {0};

/* ===================== TCS打滑检测 ===================== */
typedef struct {
    uint8_t is_slipping;
    float deviation;           // 打滑指标 (mm/s)
    float expected_dv;         // 期望速度变化 (mm/s)
    float actual_dv;           // 实际速度变化 (mm/s)
    float last_speed;          // 上一帧轮速 (mm/s)
} WheelSlip_t;

#define WHEEL_LF 0
#define WHEEL_RF 1
#define WHEEL_LB 2
#define WHEEL_RB 3

static WheelSlip_t wheel_slip[4] = {0};

#define TCS_SLIP_THRESHOLD       30.0f   // 打滑判定阈值 (mm/s 速度变化差)

/* ===================== 其他参数 ===================== */
static float L = 200.0f;  // 轮轴距中心距离 (mm)
#define CAP_RESTART_COOLDOWN 2000

// IMU偏移
static float imu_offset_x = CHASSIS_IMU_OFFSET_X;
static float imu_offset_y = CHASSIS_IMU_OFFSET_Y;

// 自旋变速
static float pos_variable_rotate_speed[8] = {1.3, 1.4, 1.5, 1.6, 1.7, 1.75, 1.8, 1.85};
static float rotate_speed_buff = 0;

// 底盘速度（电机角速度 degree/s）
static float chassis_vx, chassis_vy;
static float vt_lf, vt_rf, vt_lb, vt_rb;

// 初始化标志
static uint8_t force_ctrl_inited = 0;

/* ===================== 内部函数声明 ===================== */
static void EstimateSpeed(float dt);
static void MecanumCalculate(void);
static void ForceControlCalculate(float dt);
static void LimitChassisOutput(void);

// 力控核心函数
static float Saturation(float s, float phi);
static float SlidingModeControl(SlidingModeCtrl_t *smc, float v_err, float dt);
static float DOBUpdate(DisturbanceObserver_t *dob, float v, float u, float dt);
static void RobustForceCalculate(float vx_ref, float vy_ref, float vx_act, float vy_act, float dt);
static void AdaptiveUpdate(float I_x, float I_y, float ax_actual, float ay_actual, float dt);
static void ForceAllocation(float kt);
static void GetEquivalentCurrent(float *I_x, float *I_y);
static void FrictionEstimatorUpdate(float Fx_cmd, float Fy_cmd, float ax_actual, float ay_actual, float slip_severity);
static float GetMaxGroundForce(void);
static void FrictionAdaptiveLimiter(float *Fx, float *Fy);

/* ===================== 宏定义辅助计算 ===================== */
#define CENTER1 ((L + CENTER_GIMBAL_OFFSET_X - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define CENTER2 ((L - CENTER_GIMBAL_OFFSET_X - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define CENTER3 ((L + CENTER_GIMBAL_OFFSET_X + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define CENTER4 ((L - CENTER_GIMBAL_OFFSET_X + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

/* ===================== 二阶滑模控制实现 (Super-Twisting) ===================== */

/**
 * @brief 平滑符号函数 (边界层法)
 * @param s 输入值
 * @param phi 边界层厚度
 * @return 平滑后的符号值 [-1, 1]
 */
static float SmoothSign(float s, float phi)
{
    if (phi < 0.001f) phi = 0.001f;  // 防止除零
    
    if (fabsf(s) < phi) {
        return s / phi;  // 边界层内线性插值
    } else {
        return (s > 0) ? 1.0f : -1.0f;  // 边界层外饱和
    }
}

/**
 * @brief 二阶滑模控制器计算 (Super-Twisting算法)
 * @param smc 控制器实例
 * @param v_err 速度误差 (m/s)
 * @param dt 时间步长 (s)
 * @return 控制力 (N)
 * 
 * @note Super-Twisting算法:
 *       u = u1 + u2 + disturbance
 *       u1 = -k1 * |s|^0.5 * sign(s)   (比例项, 提供快速收敛)
 *       u2 = v_integral                 (积分项, 补偿不确定性)
 *       dv_integral/dt = -k2 * sign(s)  (积分项动态)
 *       
 *       优点: 控制输出连续, 天然抑制抖振, 有限时间收敛
 */
static float SlidingModeControl(SlidingModeCtrl_t *smc, float v_err, float dt)
{
    // 计算速度误差导数 (数值微分)
    float v_err_dot = 0.0f;
    if (dt > 0.0001f) {
        v_err_dot = (v_err - smc->v_err_last) / dt;
    }
    smc->v_err_last = v_err;
    
    // 滑模面: s = v̇_err + λ * v_err
    float s = v_err_dot + smc->lambda * v_err;
    smc->s_last = s;  // 保存用于调试
    
    // ======== Super-Twisting算法 ========
    // 比例项: u1 = -k1 * |s|^0.5 * sign(s)
    float abs_s = fabsf(s);
    float sqrt_abs_s = sqrtf(abs_s + 0.0001f);  // 加小量防止数值问题
    float sign_s = SmoothSign(s, smc->phi);
    
    float u1 = -smc->st_k1 * sqrt_abs_s * sign_s;
    
    // 积分项动态: dv/dt = -k2 * sign(s)
    smc->v_integral += -smc->st_k2 * sign_s * dt;
    
    // 积分项限幅 (防止积分饱和)
    if (smc->v_integral > smc->v_max) {
        smc->v_integral = smc->v_max;
    } else if (smc->v_integral < -smc->v_max) {
        smc->v_integral = -smc->v_max;
    }
    
    float u2 = smc->v_integral;
    
    // 总控制力: F = m * (u1 + u2) + 扰动补偿
    float u = FC_CHASSIS_MASS * (u1 + u2);
    
    // 加入扰动补偿 (由DOB提供)
    u += smc->disturbance;
    
    return u;
}

/* ===================== 扰动观测器实现 ===================== */

/**
 * @brief 扰动观测器更新
 * @param dob 观测器实例
 * @param v 当前速度 (m/s)
 * @param u 控制输入力 (N) - 上一周期的
 * @param dt 时间步长 (s)
 * @return 估计扰动 (N)
 */
static float DOBUpdate(DisturbanceObserver_t *dob, float v, float u, float dt)
{
    float Lmv = dob->L * FC_CHASSIS_MASS * v;
    
    // 观测器动态: ż = -L*z - L*(L*m*v + u)
    float z_dot = -dob->L * dob->z - dob->L * (Lmv + u);
    dob->z += z_dot * dt;
    
    // 扰动估计: d̂ = z + L*m*v
    dob->d_hat = dob->z + Lmv;
    
    // 限幅防止发散
    dob->d_hat = fmaxf(-FC_DOB_D_MAX, fminf(FC_DOB_D_MAX, dob->d_hat));
    
    return dob->d_hat;
}

/* ===================== 鲁棒力控制 ===================== */

/**
 * @brief 鲁棒力控制计算 (滑模 + 扰动观测)
 */
static void RobustForceCalculate(float vx_ref, float vy_ref, 
                                  float vx_act, float vy_act, float dt)
{
    // 速度误差
    float vx_err = vx_ref - vx_act;
    float vy_err = vy_ref - vy_act;
    
    // 滑模控制计算力
    float Fx = SlidingModeControl(&smc_vx, vx_err, dt);
    float Fy = SlidingModeControl(&smc_vy, vy_err, dt);
    
    // 更新扰动观测器 (使用上一周期的力)
    float dx = DOBUpdate(&dob_x, vx_act, force_cmd.Fx, dt);
    float dy = DOBUpdate(&dob_y, vy_act, force_cmd.Fy, dt);
    
    // 将扰动补偿传给滑模控制器 (下一周期使用)
    smc_vx.disturbance = -dx;  // 负号: 补偿要抵消扰动
    smc_vy.disturbance = -dy;
    
    // 更新力指令
    force_cmd.Fx = Fx;
    force_cmd.Fy = Fy;
    
    // 更新调试数据
    debug_data.dx_hat = dx;
    debug_data.dy_hat = dy;
}

/* ===================== 在线自适应 ===================== */

/**
 * @brief 计算等效电流 (正运动学)
 */
static void GetEquivalentCurrent(float *I_x, float *I_y)
{
    // 正运动学: 各轮电流 -> XY方向等效电流
    *I_x = (-current_ff_lf + current_ff_rf - current_ff_lb + current_ff_rb) 
           / (2.0f * Sqrt(2));
    *I_y = (-current_ff_lf - current_ff_rf + current_ff_lb + current_ff_rb) 
           / (2.0f * Sqrt(2));
}

/**
 * @brief 在线自适应更新力矩常数
 */
static void AdaptiveUpdate(float I_x, float I_y, 
                            float ax_actual, float ay_actual, float dt)
{
    // 电调单位 -> 安培 (M3508: 1A ≈ 819.2 电调单位)
    float I_x_amp = I_x / 819.2f;
    float I_y_amp = I_y / 819.2f;
    float I_norm = sqrtf(I_x_amp * I_x_amp + I_y_amp * I_y_amp);
    
    // 只在有足够激励时更新
    if (I_norm < FC_ADAPTIVE_I_THRESHOLD) return;
    
    // 模型预测加速度
    float ax_expect = adaptive.kt_estimate * I_x_amp / FC_CHASSIS_MASS;
    float ay_expect = adaptive.kt_estimate * I_y_amp / FC_CHASSIS_MASS;
    
    // 预测误差
    float error_x = ax_actual - ax_expect;
    float error_y = ay_actual - ay_expect;
    
    // 归一化自适应律 (MIT Rule)
    float norm_factor = 1.0f / (1.0f + I_norm * I_norm);
    float delta_kt = adaptive.gamma * (error_x * I_x_amp + error_y * I_y_amp) 
                     * norm_factor * dt;
    
    // 更新估计值并限幅
    adaptive.kt_estimate += delta_kt;
    adaptive.kt_estimate = fmaxf(adaptive.kt_min, 
                                  fminf(adaptive.kt_max, adaptive.kt_estimate));
    
    // 更新调试数据
    debug_data.kt_estimate = adaptive.kt_estimate;
}

/* ===================== 力分配与电流映射 ===================== */

/**
 * @brief 计算单轮摩擦补偿电流
 * @param omega_wheel 轮子角速度 (rad/s)
 * @param kt_esc kt转换到电调单位 (N/电调单位)
 * @return 摩擦补偿电流 (电调单位)
 * @note 电机模型: τ_m = K_t * I - τ_c * sign(ω) - b * ω
 *       要产生目标力F, 需要: I = (F + τ_c * sign(ω) + b * ω) / K_t
 */
static float CalcFrictionCompensation(float omega_wheel, float kt_esc)
{
    if (!motor_friction.enable || kt_esc < 0.001f) {
        return 0.0f;
    }
    
    // 库仑摩擦补偿: τ_c * sign(ω)
    float coulomb_comp = 0.0f;
    if (omega_wheel > 0.1f) {
        coulomb_comp = motor_friction.tau_c;
    } else if (omega_wheel < -0.1f) {
        coulomb_comp = -motor_friction.tau_c;
    } else {
        // 死区内使用线性插值避免抖动
        coulomb_comp = motor_friction.tau_c * (omega_wheel / 0.1f);
    }
    
    // 粘性摩擦补偿: b * ω
    float viscous_comp = motor_friction.viscous_b * omega_wheel;
    
    // 总摩擦补偿电流
    return (coulomb_comp + viscous_comp) / kt_esc;
}

/**
 * @brief 力分配与电流映射
 * @param kt 当前力矩常数估计值 (N/A)
 */
static void ForceAllocation(float kt)
{
    float Fx = force_cmd.Fx;
    float Fy = force_cmd.Fy;
    float Mz = force_cmd.Mz;
    
    // 力分配 (X型全向轮逆运动学)
    // 注意: FC_WHEEL_BASE_L 单位是m, 这里统一使用m
    float F_lf = (-Fx - Fy) / (2.0f * Sqrt(2)) + Mz / (4.0f * FC_WHEEL_BASE_L);
    float F_rf = ( Fx - Fy) / (2.0f * Sqrt(2)) - Mz / (4.0f * FC_WHEEL_BASE_L);
    float F_lb = (-Fx + Fy) / (2.0f * Sqrt(2)) - Mz / (4.0f * FC_WHEEL_BASE_L);
    float F_rb = ( Fx + Fy) / (2.0f * Sqrt(2)) + Mz / (4.0f * FC_WHEEL_BASE_L);
    
    // 力 -> 电流前馈 (电调单位)
    // kt 单位: N/A, 需要转换到电调单位
    // M3508: 1A = 819.2 电调单位
    float kt_esc = kt * 819.2f;  // N/电调单位
    
    // 获取轮子角速度 (rad/s)
    // speed_aps: 度/秒, 需要转换到 rad/s 并考虑减速比
    float omega_lf = motor_lf->measure.speed_aps * DEGREE_2_RAD / REDUCTION_RATIO_WHEEL;
    float omega_rf = motor_rf->measure.speed_aps * DEGREE_2_RAD / REDUCTION_RATIO_WHEEL;
    float omega_lb = motor_lb->measure.speed_aps * DEGREE_2_RAD / REDUCTION_RATIO_WHEEL;
    float omega_rb = motor_rb->measure.speed_aps * DEGREE_2_RAD / REDUCTION_RATIO_WHEEL;
    
    // 计算摩擦补偿电流
    float friction_comp_lf = CalcFrictionCompensation(omega_lf, kt_esc);
    float friction_comp_rf = CalcFrictionCompensation(omega_rf, kt_esc);
    float friction_comp_lb = CalcFrictionCompensation(omega_lb, kt_esc);
    float friction_comp_rb = CalcFrictionCompensation(omega_rb, kt_esc);
    
    if (kt_esc > 0.001f) {  // 避免除零
        // 目标力电流 + 摩擦补偿电流
        current_ff_lf = F_lf / kt_esc + friction_comp_lf;
        current_ff_rf = F_rf / kt_esc + friction_comp_rf;
        current_ff_lb = F_lb / kt_esc + friction_comp_lb;
        current_ff_rb = F_rb / kt_esc + friction_comp_rb;
    }
    
    // 更新调试数据
    debug_data.current_ff[0] = current_ff_lf;
    debug_data.current_ff[1] = current_ff_rf;
    debug_data.current_ff[2] = current_ff_lb;
    debug_data.current_ff[3] = current_ff_rb;
    debug_data.friction_comp[0] = friction_comp_lf;
    debug_data.friction_comp[1] = friction_comp_rf;
    debug_data.friction_comp[2] = friction_comp_lb;
    debug_data.friction_comp[3] = friction_comp_rb;
}

/* ===================== 摩擦系数估计 ===================== */

/**
 * @brief 摩擦系数在线估计
 */
static void FrictionEstimatorUpdate(float Fx_cmd, float Fy_cmd,
                                     float ax_actual, float ay_actual,
                                     float slip_severity)
{
    float F_cmd_norm = sqrtf(Fx_cmd * Fx_cmd + Fy_cmd * Fy_cmd);
    float a_actual_norm = sqrtf(ax_actual * ax_actual + ay_actual * ay_actual);
    
    // 有效性检查
    // 1. 指令力足够大
    if (F_cmd_norm < 15.0f) return;
    
    // 2. 不在严重打滑状态 (超过一半轮子打滑)
    if (slip_severity > 0.5f) return;
    
    // 3. 方向一致性检查
    if (F_cmd_norm > 0.01f && a_actual_norm > 0.01f) {
        float cos_angle = (Fx_cmd * ax_actual + Fy_cmd * ay_actual) 
                          / (F_cmd_norm * a_actual_norm);
        if (cos_angle < 0.7f) return;
    }
    
    // 摩擦效率计算
    float mu_measured = (FC_CHASSIS_MASS * a_actual_norm) / F_cmd_norm;
    mu_measured = fmaxf(friction.mu_min, fminf(1.0f, mu_measured));
    
    // 低通滤波更新
    friction.mu_estimate = friction.mu_estimate * (1.0f - friction.filter_alpha)
                         + mu_measured * friction.filter_alpha;
    
    friction.sample_count++;
    
    // 更新调试数据
    debug_data.mu_estimate = friction.mu_estimate;
}

/**
 * @brief 获取当前最大可用地面力
 */
static float GetMaxGroundForce(void)
{
    // 单轮法向力 ≈ mg/4
    float normal_force_per_wheel = FC_CHASSIS_MASS * 9.81f / 4.0f;
    
    // 最大总摩擦力 = μ * N * 4 * 安全系数
    float safety_factor = 0.75f;
    return friction.mu_estimate * normal_force_per_wheel * 4.0f * safety_factor;
}

/**
 * @brief 摩擦自适应力限幅
 */
static void FrictionAdaptiveLimiter(float *Fx, float *Fy)
{
    float F_max = GetMaxGroundForce();
    float F_cmd = sqrtf((*Fx) * (*Fx) + (*Fy) * (*Fy));
    
    if (F_cmd > F_max && F_cmd > 0.01f) {
        float scale = F_max / F_cmd;
        *Fx *= scale;
        *Fy *= scale;
    }
}

/* ===================== 速度估计与打滑检测 ===================== */

/**
 * @brief 速度估计与打滑检测
 */
static void EstimateSpeed(float dt)
{
    // 获取四个轮子的线速度: 电机角速度(degree/s) -> 轮子线速度(mm/s)
    float wheel_lf = motor_lf->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    float wheel_rf = motor_rf->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    float wheel_lb = motor_lb->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    float wheel_rb = motor_rb->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    
    // 正运动学: 轮速(mm/s) -> 底盘速度(mm/s)
    float wheel_vx = (-wheel_lf + wheel_rf - wheel_lb + wheel_rb) / (2.0f * Sqrt(2));
    float wheel_vy = (-wheel_lf - wheel_rf + wheel_lb + wheel_rb) / (2.0f * Sqrt(2));
    
    // 角速度wz
    chassis_feedback_data.real_wz = (wheel_lf + wheel_rf + wheel_lb + wheel_rb) 
                                    / (4.0f * L * DEGREE_2_RAD);

#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
    // 使用已去重力的运动加速度 (m/s^2 -> mm/s^2)
    float imu_accel_x = -Chassis_INS_data->MotionAccel_b[0] * 1000.0f;
    float imu_accel_y = Chassis_INS_data->MotionAccel_b[1] * 1000.0f;
    
    // IMU安装偏移补偿
    float omega_yaw = Chassis_INS_data->Gyro[2];
    float alpha_yaw = Chassis_INS_data->GyroAlpha[2];
    float omega_yaw_sq = omega_yaw * omega_yaw;
    
    // 向心加速度补偿
    imu_accel_x -= omega_yaw_sq * imu_offset_x;
    imu_accel_y -= omega_yaw_sq * imu_offset_y;
    
    // 切向加速度补偿
    imu_accel_x -= (-alpha_yaw * imu_offset_y);
    imu_accel_y -= (alpha_yaw * imu_offset_x);

    // 卡尔曼滤波融合
    chassis_feedback_data.real_vx = SimpleKalman1D_Update(&kf_vx, imu_accel_x, wheel_vx, dt);
    chassis_feedback_data.real_vy = SimpleKalman1D_Update(&kf_vy, imu_accel_y, wheel_vy, dt);
    
    // ========== 基于加速度的打滑检测 ==========
    // 优点: 直接使用IMU加速度预测, 不依赖卡尔曼输出, 打破循环依赖
    
    // 1. 用IMU加速度预测底盘速度变化 (mm/s)
    float dv_x = imu_accel_x * dt;  // 底盘X方向速度变化
    float dv_y = imu_accel_y * dt;  // 底盘Y方向速度变化
    float dv_wz = alpha_yaw * dt;   // 角速度变化
    
    // 2. 逆运动学: 底盘速度变化 → 各轮期望速度变化 (mm/s)
    float expected_dv_lf = (-dv_x - dv_y) / Sqrt(2) + dv_wz * L;
    float expected_dv_rf = ( dv_x - dv_y) / Sqrt(2) - dv_wz * L;
    float expected_dv_lb = (-dv_x + dv_y) / Sqrt(2) - dv_wz * L;
    float expected_dv_rb = ( dv_x + dv_y) / Sqrt(2) + dv_wz * L;
    
    // 3. 实际轮速变化 (当前 - 上一帧)
    float actual_dv_lf = wheel_lf - wheel_slip[WHEEL_LF].last_speed;
    float actual_dv_rf = wheel_rf - wheel_slip[WHEEL_RF].last_speed;
    float actual_dv_lb = wheel_lb - wheel_slip[WHEEL_LB].last_speed;
    float actual_dv_rb = wheel_rb - wheel_slip[WHEEL_RB].last_speed;
    
    // 保存当前轮速供下次使用
    wheel_slip[WHEEL_LF].last_speed = wheel_lf;
    wheel_slip[WHEEL_RF].last_speed = wheel_rf;
    wheel_slip[WHEEL_LB].last_speed = wheel_lb;
    wheel_slip[WHEEL_RB].last_speed = wheel_rb;
    
    // 4. 打滑检测: 比较期望速度变化与实际速度变化
    float expected_dv[4] = {expected_dv_lf, expected_dv_rf, expected_dv_lb, expected_dv_rb};
    float actual_dv[4] = {actual_dv_lf, actual_dv_rf, actual_dv_lb, actual_dv_rb};
    
    uint8_t any_wheel_slipping = 0;
    for (int i = 0; i < 4; i++) {
        wheel_slip[i].expected_dv = expected_dv[i];
        wheel_slip[i].actual_dv = actual_dv[i];
        
        // 打滑指标: 期望与实际速度变化的差异
        float slip_indicator = fabsf(actual_dv[i] - expected_dv[i]);
        wheel_slip[i].deviation = slip_indicator;
        
        // 打滑判定: 差异超过阈值且轮速足够大(避免静止时误判)
        float wheel_speed = fabsf((i == 0) ? wheel_lf : 
                                  (i == 1) ? wheel_rf : 
                                  (i == 2) ? wheel_lb : wheel_rb);
        
        if (slip_indicator > TCS_SLIP_THRESHOLD && wheel_speed > 100.0f) {
            wheel_slip[i].is_slipping = 1;
            any_wheel_slipping = 1;
        } else {
            wheel_slip[i].is_slipping = 0;
        }
    }
    
    // 动态调整卡尔曼R值 (打滑时更信任IMU)
    if (any_wheel_slipping) {
        uint8_t slip_wheel_count = 0;
        for (int i = 0; i < 4; i++) {
            if (wheel_slip[i].is_slipping) slip_wheel_count++;
        }
        float slip_R = 0.8f + 0.7f * slip_wheel_count;
        kf_vx.R = slip_R;
        kf_vy.R = slip_R;
    } else {
        kf_vx.R = 0.8f;
        kf_vy.R = 0.8f;
    }
#else
    chassis_feedback_data.real_vx = wheel_vx;
    chassis_feedback_data.real_vy = wheel_vy;
#endif
}

/* ===================== 麦轮逆运动学 ===================== */

/**
 * @brief 计算各轮目标速度 (送入软速度环)
 */
static void MecanumCalculate(void)
{   
    chassis_vx *= 10;
    chassis_vy *= 10;
    
    // X型全向轮逆运动学
    vt_lf = (-chassis_vx - chassis_vy) / Sqrt(2) + chassis_cmd_recv.wz * CENTER2;
    vt_rf = (chassis_vx - chassis_vy) / Sqrt(2) - chassis_cmd_recv.wz * CENTER1;
    vt_lb = (-chassis_vx + chassis_vy) / Sqrt(2) - chassis_cmd_recv.wz * CENTER3;
    vt_rb = (chassis_vx + chassis_vy) / Sqrt(2) + chassis_cmd_recv.wz * CENTER4;
}

/* ===================== 力控核心计算 ===================== */

/**
 * @brief 力控主函数
 */
static void ForceControlCalculate(float dt)
{
#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
    if (!force_ctrl_inited) return;
    
    // 1. 获取状态 (单位转换: mm/s -> m/s)
    float vx_ref = chassis_cmd_recv.vx / 1000.0f;
    float vy_ref = chassis_cmd_recv.vy / 1000.0f;
    float wz_ref = chassis_cmd_recv.wz * DEGREE_2_RAD;
    
    float vx_act = chassis_feedback_data.real_vx / 1000.0f;
    float vy_act = chassis_feedback_data.real_vy / 1000.0f;
    float wz_act = Chassis_INS_data->Gyro[2];
    
    float ax_act = Chassis_INS_data->MotionAccel_b[0];
    float ay_act = Chassis_INS_data->MotionAccel_b[1];
    
    // 更新调试数据
    debug_data.vx_ref = vx_ref;
    debug_data.vy_ref = vy_ref;
    debug_data.vx_act = vx_act;
    debug_data.vy_act = vy_act;
    
    // 2. 鲁棒力控制 (滑模 + DOB)
    RobustForceCalculate(vx_ref, vy_ref, vx_act, vy_act, dt);
    
    // 简化的Mz控制 (可扩展为单独的滑模)
    force_cmd.Mz = FC_CHASSIS_INERTIA_Z * 5.0f * (wz_ref - wz_act);
    
    // 3. 摩擦自适应估计与限幅
    // 统计打滑轮数量
    uint8_t slip_count = 0;
    for (int i = 0; i < 4; i++) {
        if (wheel_slip[i].is_slipping) slip_count++;
    }
    float slip_severity = (float)slip_count / 4.0f;  // 0~1
    
    FrictionEstimatorUpdate(force_cmd.Fx, force_cmd.Fy, ax_act, ay_act, slip_severity);
    FrictionAdaptiveLimiter(&force_cmd.Fx, &force_cmd.Fy);
    
    // 更新调试数据
    debug_data.Fx_cmd = force_cmd.Fx;
    debug_data.Fy_cmd = force_cmd.Fy;
    
    // 4. 力分配与电流映射
    ForceAllocation(adaptive.kt_estimate);
    
    // 5. 在线自适应更新kt
    float I_x, I_y;
    GetEquivalentCurrent(&I_x, &I_y);
    AdaptiveUpdate(I_x, I_y, ax_act, ay_act, dt);
    
    // 6. VOFA调试输出
    VOFA(0, 
         vx_ref, vx_act, vy_ref, vy_act,
         force_cmd.Fx, force_cmd.Fy,
         adaptive.kt_estimate * 1000.0f,  // 放大显示
         dob_x.d_hat, dob_y.d_hat,
         friction.mu_estimate,
         current_ff_lf, current_ff_rf);
    
#else
    // 无IMU模式: 清零前馈
    current_ff_lf = current_ff_rf = current_ff_lb = current_ff_rb = 0;
#endif
}

/* ===================== 输出限制与电机设置 ===================== */

/**
 * @brief 电容消息结构
 */
struct CapTxMsg
{
    uint8_t enableDCDC : 1;
    uint8_t systemRestart : 1;
    uint8_t resv0 : 6;
    uint16_t RefereePowerLimit;
    uint16_t RefereeEnergyBuffer;
    uint8_t resv1[3];
} __attribute__((packed));

static float chassis_powerlimit = 0;

/**
 * @brief 输出限制与电机设置
 */
static void LimitChassisOutput(void)
{   
    // 超级电容控制
    uint8_t enableDCDC = (cap->cap_msg.capEnergy >= 65) ? 1 : 0;
    
    if (enableDCDC) {
        chassis_powerlimit = 40 + cap->cap_msg.chassisPowerLimit;
    } else {
        chassis_powerlimit = 40.0f; 
    }
    ChassisPowerSet(chassis_powerlimit);
    
    static uint32_t cnt = 0;
    if ((cnt++) % 125 == 0) {
        rotate_speed_buff = pos_variable_rotate_speed[GetRandomInt(0, 7)];
    }
    rotate_speed_buff = 1.5;
    
    // 设置电机参考值 (软速度环)
    if(chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE) {
        DJIMotorSetRef(motor_lf, vt_lf * 1.25f);
        DJIMotorSetRef(motor_rf, vt_rf * 1.25f);
        DJIMotorSetRef(motor_lb, vt_lb * 1.25f);
        DJIMotorSetRef(motor_rb, vt_rb * 1.25f);
    } else {
        DJIMotorSetRef(motor_lf, vt_lf);
        DJIMotorSetRef(motor_rf, vt_rf);
        DJIMotorSetRef(motor_lb, vt_lb);
        DJIMotorSetRef(motor_rb, vt_rb);
    }

    // 超电错误检测与重启
    static uint32_t cooldown = 0;
    uint8_t restart = 0;
    
    if (cooldown > 0) {
        cooldown--;
    } else if (cap->cap_msg.errorCoad != 0) {
        restart = 1;
        cooldown = CAP_RESTART_COOLDOWN;
    }
    
    struct CapTxMsg cap_msg = {
        .enableDCDC = enableDCDC,
        .systemRestart = restart,
        .RefereePowerLimit = 100,
        .RefereeEnergyBuffer = 60,
    };
    SuperCapSend(cap, (uint8_t*)&cap_msg);
}

/* ===================== 公共接口实现 ===================== */

/**
 * @brief 力控底盘初始化
 */
void ChassisForceCtrlInit(void)
{
    // 电机配置 - 软速度环 + 电流前馈
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            // 软速度环 (低增益)
            .speed_PID = {
                .Kp = FC_SOFT_SPEED_KP,
                .Ki = FC_SOFT_SPEED_KI,
                .Kd = 0.0f,
                .IntegralLimit = 2000,
                .Improve = PID_Integral_Limit,
                .MaxOut = FC_SOFT_SPEED_MAX_OUT,
            },
            // 电流环
            .current_PID = {
                .Kp = 0.5f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Integral_Limit,
                .MaxOut = 16000,
            },
            .current_feedforward_ptr = NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
    
    // LF电机
    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &current_ff_lf;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    // RF电机
    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &current_ff_rf;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    // LB电机
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &current_ff_lb;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    // RB电机
    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &current_ff_rb;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    // 注册功率限制
    DJIMotorSetPowerLimitMotors(motor_lf, M3508);
    DJIMotorSetPowerLimitMotors(motor_rf, M3508);
    DJIMotorSetPowerLimitMotors(motor_lb, M3508);
    DJIMotorSetPowerLimitMotors(motor_rb, M3508);

    // 裁判系统初始化
    referee_data = Referee_Interactive_init(&huart6, &ui_data); 

    // 超级电容初始化
    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x061,
            .rx_id = 0x051,
        }
    };
    cap = SuperCapInit(&cap_conf);
    
#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
    INS_Init();
    Chassis_INS_data = INS_GetDataPtr();
    
    // 卡尔曼滤波器初始化
    SimpleKalman1D_Init(&kf_vx, 0.1f, 0.8f, 0.0f);
    SimpleKalman1D_Init(&kf_vy, 0.1f, 0.8f, 0.0f);
    
    // 初始化TCS状态
    for (int i = 0; i < 4; i++) {
        wheel_slip[i].is_slipping = 0;
    }
    
    // 力控系统初始化
    adaptive.kt_estimate = FC_KT_INITIAL;
    
    smc_vx.lambda = FC_SMC_LAMBDA;
    smc_vx.st_k1 = FC_SMC_K1;
    smc_vx.st_k2 = FC_SMC_K2;
    smc_vx.phi = FC_SMC_PHI;
    smc_vx.v_integral = 0;
    smc_vx.v_max = FC_SMC_V_MAX;
    smc_vx.disturbance = 0;
    smc_vx.v_err_last = 0;
    smc_vx.s_last = 0;
    
    smc_vy = smc_vx;
    
    dob_x.L = FC_DOB_L;
    dob_x.z = dob_x.d_hat = 0;
    dob_y = dob_x;
    
    friction.mu_estimate = FC_MU_INITIAL;
    friction.sample_count = 0;
    
    // VOFA绑定
    VOFA_BIND(0, &imu_offset_x, &imu_offset_y);
    
    force_ctrl_inited = 1;
#endif

#ifdef CHASSIS_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x311,
            .rx_id = 0x312,
         },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf);
#endif

#ifdef CHASSIS_ONLY
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif

    // 电机离线检测
    MotorOfflineAlarmConfig_t chassis_alarm_cfg = {
        .motors = {motor_rf, motor_lb, motor_rb, motor_lf},
        .beep_times = {1, 2, 3, 4},
        .motor_count = 4,
        .buzzer_freq = ALARM_FREQ_LOW,
        .run_buzzer_task = 1,
    };
    chassis_offline_alarm = MotorOfflineAlarmRegister(&chassis_alarm_cfg);
}

/**
 * @brief 力控底盘任务
 */
void ChassisForceCtrlTask(void)
{
    // 电机离线报警
    MotorOfflineAlarmTask(chassis_offline_alarm);
    
    // 获取控制指令
#ifdef CHASSIS_ONLY
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif

    // 急停处理
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    } else {
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 控制模式处理
    switch (chassis_cmd_recv.chassis_mode) {
    case CHASSIS_NO_FOLLOW:
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW:
#ifdef CHASSIS_ONLY
        break;
#else
        chassis_cmd_recv.wz = 0.5 * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
        break;
#endif
    case CHASSIS_ROTATE:
        chassis_cmd_recv.wz = 1000 * rotate_speed_buff;
        break;
    default:
        break;
    }
    
    // 坐标变换
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

    chassis_vx = (chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta);
    chassis_vy = (chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta);

    // 获取dt
    static uint32_t force_ctrl_cnt = 0;
    float dt = DWT_GetDeltaT(&force_ctrl_cnt);
    if (dt > 0.1f) dt = 0.002f;
    
    // 速度估计
    EstimateSpeed(dt);
    
    // 麦轮逆运动学 (软速度环)
    MecanumCalculate();
    
    // 力控核心计算
    ForceControlCalculate(dt);

    // 输出限制
    LimitChassisOutput();

    // 裁判系统数据
    chassis_feedback_data.self_color = referee_data->GameRobotState.robot_id > 7 ? COLOR_BLUE : COLOR_RED;
    chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;
    chassis_feedback_data.robot_level = referee_data->GameRobotState.robot_level;
    chassis_feedback_data.bullet_speed = referee_data->ShootData.bullet_speed;
    
    ui_data.chassis_mode = chassis_cmd_recv.chassis_mode;
    ui_data.friction_mode = chassis_cmd_recv.friction_mode;
    ui_data.lid_mode = chassis_cmd_recv.lid_mode;
    ui_data.shoot_mode = chassis_cmd_recv.load_mode;
    ui_data.ui_mode = chassis_cmd_recv.ui_mode;
    ui_data.Pitch_angle = -chassis_cmd_recv.pitch_angle;
    ui_data.offset_angle = chassis_cmd_recv.offset_angle;
    ui_data.aim_mode = chassis_cmd_recv.aim_mode;
    ui_data.capEnergy = cap->cap_msg.capEnergy;

    // 推送反馈消息
#ifdef CHASSIS_ONLY
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif
}

/* ===================== 辅助接口 ===================== */

const ForceCtrlDebug_t* GetForceCtrlDebugData(void)
{
    return &debug_data;
}

const AdaptiveParam_t* GetAdaptiveParam(void)
{
    return &adaptive;
}

const FrictionEstimator_t* GetFrictionEstimator(void)
{
    return &friction;
}

void ResetAdaptiveParam(void)
{
    adaptive.kt_estimate = FC_KT_INITIAL;
}

void ResetFrictionEstimator(void)
{
    friction.mu_estimate = FC_MU_INITIAL;
    friction.sample_count = 0;
}

const MotorFrictionModel_t* GetMotorFrictionModel(void)
{
    return &motor_friction;
}

void SetMotorFrictionModel(float tau_c, float viscous_b)
{
    motor_friction.tau_c = tau_c;
    motor_friction.viscous_b = viscous_b;
}
