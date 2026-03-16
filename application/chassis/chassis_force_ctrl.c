/**
 * @file chassis_force_ctrl.c
 * @brief 纯力矩控制底盘 (Super-Twisting SMC + ESO + Kalman融合 + 阻力前馈)
 * @note 每轮独立力矩控制, 电机配置为OPEN_LOOP, DJIMotorSetRef直接输出C620电流
 *
 * 控制架构 (每轮独立):
 *   u = u_ff + u_drag + u_smc + u_eso
 *
 * 底盘速度观测: IMU加速度 + 轮速卡尔曼融合 (移植自chassis.c)
 * 打滑检测: IMU加速度预测 vs 轮速变化, 动态调整Kalman R
 *
 * @version 3.0
 * @date 2026-03-04
 */

#include "chassis_force_ctrl.h"
#include "user_lib.h"
#include "bsp_rng.h"
#include "controller.h"
#include "vofa.h"
#include "simple_kalman.h"
#include "motor_offline_alarm.h"

/* ======================== 轮子索引 ======================== */
#define W_LF 0
#define W_RF 1
#define W_LB 2
#define W_RB 3

/* ======================== 模块实例 ======================== */
// 电机 (OPEN_LOOP, DJIMotorSetRef直接输出C620电流)
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;
static DJIMotorInstance *motors[4]; // 指针数组方便遍历

// 通信
#ifdef FORCE_CONTROL_CHASSIS_BOARD
#include "can_comm.h"
static CANCommInstance *chassis_can_comm;
#endif
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;
static Chassis_Upload_Data_s chassis_feedback_data;

// IMU
#include "ins_task.h"
static INS_t *Chassis_INS;

// 裁判系统 & 超电
static referee_info_t *referee_data;
static Referee_Interactive_info_t ui_data;
static SuperCapInstance *cap;

// pub/sub (CHASSIS_ONLY用, FORCE_CONTROL_CHASSIS_BOARD走CAN)
// 力控底盘暂时不走pub/sub, 只走CAN comm

// 离线报警
static MotorOfflineAlarmInstance *fc_offline_alarm = NULL;

/* ======================== 卡尔曼融合 ======================== */
static SimpleKalman1D_t kf_vx, kf_vy;

/* ======================== 每轮控制状态 ======================== */
static WheelTorqueCtrl_t wheel_ctrl[4] = {0};
static WheelSlip_t       wheel_slip[4] = {0};

/* ======================== 调试数据 ======================== */
static ForceCtrlDebug_t fc_debug = {0};

/* ======================== 私有变量 ======================== */
static float chassis_vx, chassis_vy;  // 云台系→底盘系, 单位: deg/s (电机转子角速度)
static float vt_lf, vt_rf, vt_lb, vt_rb; // 各轮目标速度 deg/s

// IMU偏移 (VOFA可调)
static float imu_offset_x = FC_IMU_OFFSET_X;
static float imu_offset_y = FC_IMU_OFFSET_Y;

// 自旋变速
static float rotate_speed_buff = 1.5f;
static float pos_variable_rotate_speed[8] = {1.3f, 1.4f, 1.5f, 1.6f, 1.7f, 1.75f, 1.8f, 1.85f};

// 静止检测 (用于重置控制器状态)
#define FC_ZERO_INPUT_THRESHOLD  30   // 30 * 2ms = 60ms
static uint32_t zero_input_cnt = 0;

// 轮轴距中心
static float L = 200.0f;
#define Sqrt2 1.41421356f

#define CENTER1 ((L + CENTER_GIMBAL_OFFSET_X - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define CENTER2 ((L - CENTER_GIMBAL_OFFSET_X - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define CENTER3 ((L + CENTER_GIMBAL_OFFSET_X + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define CENTER4 ((L - CENTER_GIMBAL_OFFSET_X + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

/* ======================== 辅助函数 ======================== */



/**
 * @brief 浮点限幅
 */
static float _Clampf(float val, float lo, float hi)
{
    if (val > hi) return hi;
    if (val < lo) return lo;
    return val;
}

/* ======================== 麦轮逆运动学 ======================== */

/**
 * @brief 底盘速度 → 各轮目标速度 (X型全向轮)
 * @note 输入输出均为 deg/s (电机转子角速度)
 */
static void _MecanumCalculate(void)
{
    vt_lf = (-chassis_vx - chassis_vy) / Sqrt2 + chassis_cmd_recv.wz * CENTER2;
    vt_rf = ( chassis_vx - chassis_vy) / Sqrt2 - chassis_cmd_recv.wz * CENTER1;
    vt_lb = (-chassis_vx + chassis_vy) / Sqrt2 - chassis_cmd_recv.wz * CENTER3;
    vt_rb = ( chassis_vx + chassis_vy) / Sqrt2 + chassis_cmd_recv.wz * CENTER4;
}

/* ======================== 阻力前馈模型 ======================== */

/**
 * @brief 计算单轮阻力前馈
 * @param w 电机轴角速度 speed_aps (deg/s), 已含方向符号
 * @return 阻力前馈 u_drag (C620电流单位), 含方向
 * @note T = (a0 + a1*|w| + a2*|w|^2 + a3*|w|^3) * sign(w)
 */
static float _CalcDragFF(float w)
{
    float abs_w = fabsf(w);
    float drag_mag = FC_A0 + FC_A1 * abs_w + FC_A2 * abs_w * abs_w
                           + FC_A3 * abs_w * abs_w * abs_w;
    return (w >= 0.0f) ? drag_mag : -drag_mag;
}

/* ======================== Super-Twisting SMC ======================== */

/**
 * @brief 单轮Super-Twisting滑模控制计算
 * @param st  ST控制器状态
 * @param s   滑模面 = v_ref - v_act (deg/s)
 * @param dt  控制周期 (s)
 * @return u_smc (C620电流单位)
 *
 * 算法:
 *   u1 = -k1 * sqrt(|s|) * sign(s)
 *   dσ/dt = -k2 * sign(s)
 *   u_smc = u1 + σ
 */
static float _STSMCCalc(WheelST_SMC_t *st, float s, float dt)
{
    float sign_s = SmoothSign(s, FC_ST_PHI);
    float abs_s  = fabsf(s);

    // 比例项
    float u1 = -FC_ST_K1 * sqrtf(abs_s + 1e-6f) * sign_s;

    // 积分项动态更新: dσ/dt = -k2 * sign(s)
    st->sigma += -FC_ST_K2 * sign_s * dt;
    st->sigma  = _Clampf(st->sigma, -FC_ST_SIGMA_MAX, FC_ST_SIGMA_MAX);

    return u1 + st->sigma;
}

/* ======================== ESO 扩张状态观测器 ======================== */

/**
 * @brief 单轮ESO更新
 * @param eso    ESO状态
 * @param w_meas 测量角速度 speed_aps (deg/s)
 * @param u_applied 上一周期施加的电流 (已扣除阻力前馈后的净控制)
 * @param u_drag 阻力前馈
 * @param dt     控制周期 (s)
 *
 * 状态方程:
 *   e = w_meas - z1
 *   ż1 = b0 * (u_applied - u_drag) + z2 + β1 * e
 *   ż2 = β2 * e
 *   d_hat = z2 / b0
 */
static void _ESOUpdate(WheelESO_t *eso, float w_meas, float u_applied, float u_drag, float dt)
{
    float e = w_meas - eso->z1;

    float z1_dot = FC_ESO_B0 * (u_applied - u_drag) + eso->z2 + FC_ESO_BETA1 * e;
    float z2_dot = FC_ESO_BETA2 * e;

    eso->z1 += z1_dot * dt;
    eso->z2 += z2_dot * dt;
    eso->z2  = _Clampf(eso->z2, -FC_ESO_D_MAX * FC_ESO_B0, FC_ESO_D_MAX * FC_ESO_B0);

    eso->d_hat = eso->z2 / FC_ESO_B0;
    eso->d_hat = _Clampf(eso->d_hat, -FC_ESO_D_MAX, FC_ESO_D_MAX);
}

/* ======================== 速度估计 (Kalman + 打滑检测) ======================== */

/**
 * @brief 速度估计: IMU加速度+轮速卡尔曼融合, 打滑检测调整R
 * @return dt (s)
 * @note 移植自 chassis.c EstimateSpeed(), 保持相同逻辑
 */
static float _EstimateSpeed(void)
{
    // 各轮线速度: motor deg/s → wheel mm/s
    float wheel_speed[4];
    wheel_speed[W_LF] = motor_lf->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    wheel_speed[W_RF] = motor_rf->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    wheel_speed[W_LB] = motor_lb->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    wheel_speed[W_RB] = motor_rb->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;

    // 正运动学: wheel mm/s → chassis mm/s
    float wheel_vx = (-wheel_speed[W_LF] + wheel_speed[W_RF] - wheel_speed[W_LB] + wheel_speed[W_RB]) / (2.0f * Sqrt2);
    float wheel_vy = (-wheel_speed[W_LF] - wheel_speed[W_RF] + wheel_speed[W_LB] + wheel_speed[W_RB]) / (2.0f * Sqrt2);

    chassis_feedback_data.real_wz = (wheel_speed[W_LF] + wheel_speed[W_RF] 
                                   + wheel_speed[W_LB] + wheel_speed[W_RB])
                                    / (4.0f * L * DEGREE_2_RAD);

    // DWT精确dt
    static uint32_t dwt_cnt = 0;
    float dt = DWT_GetDeltaT(&dwt_cnt);
    if (dt > 0.1f) dt = 0.002f;

    // IMU运动加速度 (m/s² → mm/s²)
    float imu_accel_x = -Chassis_INS->MotionAccel_b[0] * 1000.0f;
    float imu_accel_y =  Chassis_INS->MotionAccel_b[1] * 1000.0f;

    // IMU安装偏移补偿
    float omega_yaw   = Chassis_INS->Gyro[2];
    float alpha_yaw   = Chassis_INS->GyroAlpha[2];
    float omega_yaw_sq = omega_yaw * omega_yaw;

    // 向心加速度补偿
    imu_accel_x -= omega_yaw_sq * imu_offset_x;
    imu_accel_y -= omega_yaw_sq * imu_offset_y;

    // 切向加速度补偿
    imu_accel_x -= (-alpha_yaw * imu_offset_y);
    imu_accel_y -= ( alpha_yaw * imu_offset_x);

    // 卡尔曼融合
    chassis_feedback_data.real_vx = SimpleKalman1D_Update(&kf_vx, imu_accel_x, wheel_vx, dt);
    chassis_feedback_data.real_vy = SimpleKalman1D_Update(&kf_vy, imu_accel_y, wheel_vy, dt);

    // =============== 打滑检测 ===============
    float dv_x  = imu_accel_x * dt;
    float dv_y  = imu_accel_y * dt;
    float dv_wz = alpha_yaw * dt;

    // 逆运动学: 底盘速度变化 → 各轮期望速度变化 (mm/s)
    float expected_dv[4];
    expected_dv[W_LF] = (-dv_x - dv_y) / Sqrt2 + dv_wz * L;
    expected_dv[W_RF] = ( dv_x - dv_y) / Sqrt2 - dv_wz * L;
    expected_dv[W_LB] = (-dv_x + dv_y) / Sqrt2 - dv_wz * L;
    expected_dv[W_RB] = ( dv_x + dv_y) / Sqrt2 + dv_wz * L;

    uint8_t any_slipping = 0;
    uint8_t slip_count   = 0;

    for (int i = 0; i < 4; i++) {
        float actual_dv = wheel_speed[i] - wheel_slip[i].last_speed;
        wheel_slip[i].last_speed  = wheel_speed[i];
        wheel_slip[i].expected_dv = expected_dv[i];
        wheel_slip[i].actual_dv   = actual_dv;
        wheel_slip[i].deviation   = fabsf(actual_dv - expected_dv[i]);

        if (wheel_slip[i].deviation > FC_SLIP_THRESHOLD && fabsf(wheel_speed[i]) > FC_SLIP_SPEED_MIN) {
            wheel_slip[i].is_slipping = 1;
            any_slipping = 1;
            slip_count++;
        } else {
            wheel_slip[i].is_slipping = 0;
        }
    }

    // 卡尔曼R值调整
    if (any_slipping) {
        float slip_R = 0.8f + 0.7f * (float)slip_count;
        kf_vx.R = slip_R;
        kf_vy.R = slip_R;
    } else {
        kf_vx.R = 0.8f;
        kf_vy.R = 0.8f;
    }

    // 调试
    fc_debug.chassis_vx = chassis_feedback_data.real_vx;
    fc_debug.chassis_vy = chassis_feedback_data.real_vy;

    return dt;
}

/* ======================== 控制器状态重置 ======================== */

static void _ResetAllCtrl(void)
{
    for (int i = 0; i < 4; i++) {
        wheel_ctrl[i].st.sigma  = 0;
        wheel_ctrl[i].eso.z1    = 0;
        wheel_ctrl[i].eso.z2    = 0;
        wheel_ctrl[i].eso.d_hat = 0;
        wheel_ctrl[i].v_ref_last = 0;
        wheel_ctrl[i].u_ff   = 0;
        wheel_ctrl[i].u_drag = 0;
        wheel_ctrl[i].u_smc  = 0;
        wheel_ctrl[i].u_eso  = 0;
        wheel_ctrl[i].u_total = 0;
        wheel_ctrl[i].s       = 0;
    }
}

/* ======================== 单轮力矩控制 ======================== */

/**
 * @brief 计算单轮力矩控制输出
 * @param idx  轮子索引 (W_LF..W_RB)
 * @param v_ref  目标速度 (deg/s, 电机转子)
 * @param w_meas 实际速度 speed_aps (deg/s, 电机转子)
 * @param dt     控制周期 (s)
 * @return 总输出 (C620电流单位, ±16000)
 */
static float _WheelForceCtrl(int idx, float v_ref, float w_meas, float dt)
{
    WheelTorqueCtrl_t *wc = &wheel_ctrl[idx];

    // 滑模面: s = v_ref - w
    float s = v_ref - w_meas;
    wc->s = s;

    // 1) 加速度前馈: u_ff = k_ff * (v_ref - v_ref_last) / dt
    float dv_ref = v_ref - wc->v_ref_last;
    wc->v_ref_last = v_ref;
    wc->u_ff = (dt > 1e-6f) ? FC_K_FF * dv_ref / dt : 0.0f;

    // 2) 阻力前馈: T(w) = (a0 + a1|w| + a2|w|^2 + a3|w|^3) * sign(w)
    wc->u_drag = _CalcDragFF(w_meas);

    // 3) Super-Twisting SMC
    wc->u_smc = _STSMCCalc(&wc->st, s, dt);

    // 4) ESO扰动补偿 (用上一周期输出更新)
    _ESOUpdate(&wc->eso, w_meas, wc->u_total, wc->u_drag, dt);
    wc->u_eso = -wc->eso.d_hat;

    // 合成输出
    float u = wc->u_ff + wc->u_drag + wc->u_smc + wc->u_eso;
    u = _Clampf(u, -FC_MAX_TORQUE, FC_MAX_TORQUE);
    wc->u_total = u;

    // 填充调试数据
    fc_debug.v_ref[idx]    = v_ref;
    fc_debug.v_act[idx]    = w_meas;
    fc_debug.s[idx]        = s;
    fc_debug.u_ff[idx]     = wc->u_ff;
    fc_debug.u_drag[idx]   = wc->u_drag;
    fc_debug.u_smc[idx]    = wc->u_smc;
    fc_debug.u_eso[idx]    = wc->u_eso;
    fc_debug.u_total[idx]  = u;
    fc_debug.eso_d_hat[idx] = wc->eso.d_hat;

    return u;
}

/* ======================== 功率限制 & 超电 ======================== */

// 超电通信结构体
struct FCCapTxMsg {
    uint8_t enableDCDC    : 1;
    uint8_t systemRestart : 1;
    uint8_t resv0         : 6;
    uint16_t RefereePowerLimit;
    uint16_t RefereeEnergyBuffer;
    uint8_t resv1[3];
} __attribute__((packed));

static void _LimitAndSend(void)
{
    // 功率限制
    static float chassis_powerlimit = 0;
    uint8_t enableDCDC = (cap->cap_msg.capEnergy >= 65) ? 1 : 0;
    if (enableDCDC) {
        chassis_powerlimit = 40.0f + cap->cap_msg.chassisPowerLimit;
    } else {
        chassis_powerlimit = 40.0f;
    }
    ChassisPowerSet(chassis_powerlimit);

    // 自旋变速
    static uint32_t rot_cnt = 0;
    if ((rot_cnt++) % 125 == 0) {
        rotate_speed_buff = pos_variable_rotate_speed[GetRandomInt(0, 7)];
    }
    rotate_speed_buff = 1.5f;

    // 超电控制
    struct FCCapTxMsg cap_msg = {
        .enableDCDC = enableDCDC,
        .systemRestart = 0,
        .RefereePowerLimit = 100,
        .RefereeEnergyBuffer = 60,
    };
    SuperCapSend(cap, (uint8_t *)&cap_msg);
}

/* ======================== 公开接口 ======================== */

void ChassisForceCtrlInit(void)
{
    // ---- 电机初始化 (OPEN_LOOP, 无PID) ----
    Motor_Init_Config_s motor_cfg = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {0},  // OPEN_LOOP无需PID参数
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type  = OPEN_LOOP,
            .close_loop_type  = OPEN_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
        .motor_type = M3508,
    };

    // LF: tx_id=2, NORMAL
    motor_cfg.can_init_config.tx_id = 2;
    motor_cfg.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&motor_cfg);

    // RF: tx_id=3, REVERSE
    motor_cfg.can_init_config.tx_id = 3;
    motor_cfg.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&motor_cfg);

    // LB: tx_id=1, REVERSE
    motor_cfg.can_init_config.tx_id = 1;
    motor_cfg.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&motor_cfg);

    // RB: tx_id=4, NORMAL
    motor_cfg.can_init_config.tx_id = 4;
    motor_cfg.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = DJIMotorInit(&motor_cfg);

    motors[W_LF] = motor_lf;
    motors[W_RF] = motor_rf;
    motors[W_LB] = motor_lb;
    motors[W_RB] = motor_rb;

    // 功率限制注册
    DJIMotorSetPowerLimitMotors(motor_lf, M3508);
    DJIMotorSetPowerLimitMotors(motor_rf, M3508);
    DJIMotorSetPowerLimitMotors(motor_lb, M3508);
    DJIMotorSetPowerLimitMotors(motor_rb, M3508);

    // ---- 裁判系统 ----
    referee_data = Referee_Interactive_init(&huart6, &ui_data);

    // ---- 超级电容 ----
    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x061,
            .rx_id = 0x051,
        }
    };
    cap = SuperCapInit(&cap_conf);

    // ---- IMU ----
    INS_Init();
    Chassis_INS = INS_GetDataPtr();

    // ---- 卡尔曼滤波器 (与chassis.c相同参数) ----
    SimpleKalman1D_Init(&kf_vx, 0.1f, 0.8f, 0.0f);
    SimpleKalman1D_Init(&kf_vy, 0.1f, 0.8f, 0.0f);

    // ---- CAN通信 (双板) ----
#ifdef FORCE_CONTROL_CHASSIS_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chassis_can_comm = CANCommInit(&comm_conf);
#endif

    // ---- VOFA接收绑定 (在线调IMU偏移) ----
    VOFA_BIND(0, &imu_offset_x, &imu_offset_y);

    // ---- 离线报警 ----
    MotorOfflineAlarmConfig_t alarm_cfg = {
        .motors = {motor_rf, motor_lb, motor_rb, motor_lf},
        .beep_times = {1, 2, 3, 4},
        .motor_count = 4,
        .buzzer_freq = ALARM_FREQ_LOW,
        .run_buzzer_task = 1,
    };
    fc_offline_alarm = MotorOfflineAlarmRegister(&alarm_cfg);
}

void ChassisForceCtrlTask(void)
{
    // ---- 离线报警 ----
    MotorOfflineAlarmTask(fc_offline_alarm);

    // ---- 接收控制指令 ----
#ifdef FORCE_CONTROL_CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chassis_can_comm);
#endif

    // ---- 零力模式: 停止所有电机 ----
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        _ResetAllCtrl();
        zero_input_cnt = 0;
    } else {
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // ---- 底盘模式处理 ----
    switch (chassis_cmd_recv.chassis_mode) {
    case CHASSIS_NO_FOLLOW:
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        chassis_cmd_recv.wz = 0.5f * chassis_cmd_recv.offset_angle
                            * fabsf(chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_ROTATE:
        chassis_cmd_recv.wz = 1000.0f * rotate_speed_buff;
        break;
    default:
        break;
    }

    // ---- 云台系→底盘系坐标变换 ----
    float cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    float sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // ---- 速度估计 (Kalman + 打滑检测) ----
    float dt = _EstimateSpeed();

    // ---- 麦轮逆运动学: 底盘速度 → 各轮目标速度 (deg/s) ----
    _MecanumCalculate();

    // ---- 静止检测: 全零输入时重置控制器 ----
    if (fabsf(chassis_cmd_recv.vx) < 10.0f &&
        fabsf(chassis_cmd_recv.vy) < 10.0f &&
        fabsf(chassis_cmd_recv.wz) < 10.0f) {
        zero_input_cnt++;
        if (zero_input_cnt >= FC_ZERO_INPUT_THRESHOLD) {
            _ResetAllCtrl();
            zero_input_cnt = FC_ZERO_INPUT_THRESHOLD;
            // 静止: 所有电机输出0
            DJIMotorSetRef(motor_lf, 0);
            DJIMotorSetRef(motor_rf, 0);
            DJIMotorSetRef(motor_lb, 0);
            DJIMotorSetRef(motor_rb, 0);
            goto feedback; // 跳过力矩计算
        }
    } else {
        zero_input_cnt = 0;
    }

    // ---- 每轮独立力矩控制 ----
    {
        float vt[4] = {vt_lf, vt_rf, vt_lb, vt_rb};
        for (int i = 0; i < 4; i++) {
            float w_meas = motors[i]->measure.speed_aps;
            float u = _WheelForceCtrl(i, vt[i], w_meas, dt);
            DJIMotorSetRef(motors[i], u);
        }
    }

    // ---- 功率限制 & 超电 ----
    _LimitAndSend();

    // ---- VOFA调试输出 ----
    VOFA(0, fc_debug.v_ref[0], fc_debug.v_act[0], fc_debug.u_total[0], fc_debug.eso_d_hat[0],
            fc_debug.v_ref[1], fc_debug.v_act[1], fc_debug.u_total[1], fc_debug.eso_d_hat[1],
            fc_debug.chassis_vx, fc_debug.chassis_vy,
            (float)wheel_slip[0].is_slipping, (float)wheel_slip[1].is_slipping);

feedback:
    // ---- 反馈数据 ----
    chassis_feedback_data.self_color  = referee_data->GameRobotState.robot_id > 7 ? COLOR_BLUE : COLOR_RED;
    chassis_feedback_data.rest_heat   = referee_data->PowerHeatData.shooter_heat0;
    chassis_feedback_data.robot_level = referee_data->GameRobotState.robot_level;
    chassis_feedback_data.bullet_speed = referee_data->ShootData.bullet_speed;

    ui_data.chassis_mode  = chassis_cmd_recv.chassis_mode;
    ui_data.friction_mode = chassis_cmd_recv.friction_mode;
    ui_data.shoot_mode    = chassis_cmd_recv.load_mode;
    ui_data.ui_mode       = chassis_cmd_recv.ui_mode;
    ui_data.Pitch_angle   = -chassis_cmd_recv.pitch_angle;
    ui_data.offset_angle  = chassis_cmd_recv.offset_angle;
    ui_data.aim_mode      = chassis_cmd_recv.aim_mode;
    ui_data.fire_mode     = chassis_cmd_recv.fire_mode;
    ui_data.capEnergy     = cap->cap_msg.capEnergy;

#ifdef FORCE_CONTROL_CHASSIS_BOARD
    CANCommSend(chassis_can_comm, (void *)&chassis_feedback_data);
#endif
}

const ForceCtrlDebug_t* GetForceCtrlDebugData(void)
{
    return &fc_debug;
}
