/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.2
 * @date 2022-12-04
 * @note 速度-力矩混合控制模式: 
 *       - 电机内部速度闭环
 *       - 应用层计算力矩前馈 
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "user_lib.h"
#include "bsp_rng.h"
#include "bsp_dwt.h"
#include "controller.h"
#include "vofa.h"
#include "simple_kalman.h"
#include "dji_motor_offline_alarm.h" // 电机离线检测

// 速度融合卡尔曼滤波器实例
static SimpleKalman1D_t kf_vx, kf_vy;

/* ===================== 二阶滑模控制参数 (Super-Twisting) ===================== */
// Super-Twisting算法: u = -k1*|s|^0.5*sign(s) + v, dv/dt = -k2*sign(s)
#define SMC_LAMBDA            3.0f       // 滑模面斜率
#define SMC_K1                30.0f      // Super-Twisting增益k1 (比例项)
#define SMC_K2                60.0f     // Super-Twisting增益k2 (积分项)
#define SMC_PHI               0.03f      // 边界层厚度 (sign函数平滑)
#define SMC_V_MAX             150.0f     // 积分项最大值限幅
#define SMC_MAX_OUT           0.0f    // 滑模控制器最大输出 (电流)

// 二阶滑模控制器结构体 (Super-Twisting算法)
typedef struct {
    float lambda;           // 滑模面斜率
    float st_k1;            // Super-Twisting增益k1 (比例项)
    float st_k2;            // Super-Twisting增益k2 (积分项)
    float phi;              // 边界层厚度
    float v_integral;       // Super-Twisting积分项状态
    float v_max;            // 积分项限幅
    float disturbance;      // 估计的扰动 (由DOB提供)
    float v_err_last;       // 上一周期速度误差 (用于微分)
    float s_last;           // 上一周期滑模面值 (调试用)
} SMC_Controller_t;

/* ===================== 扩展状态观测器参数 (ESO) ===================== */
// LESO参数: 只需调整带宽ω0，观测器增益自动计算
// β1 = 2*ω0, β2 = ω0² (极点配置法)
#define ESO_OMEGA0            20.0f      // ESO带宽 (核心参数, 越大响应越快但噪声敏感)
#define ESO_B0                1.0f       // 控制增益 (归一化为1, 简化调参)
#define ESO_D_MAX             0.0f    // 最大扰动估计限幅 (电流单位) 5000

// 扩展状态观测器结构体 (LESO - 线性ESO)
typedef struct {
    float z1;               // 状态估计 (速度)
    float z2;               // 总扰动估计 (包含模型不确定性+外部扰动)
    float omega0;           // 观测器带宽
    float beta1;            // 观测器增益1 (= 2*ω0)
    float beta2;            // 观测器增益2 (= ω0²)
    float b0;               // 控制增益
} ESO_t;

// X/Y方向滑模控制器和ESO
static SMC_Controller_t smc_vx = {0};
static SMC_Controller_t smc_vy = {0};
static ESO_t eso_x = {0};
static ESO_t eso_y = {0};

/* ===================== 打滑检测与牵引力控制 ===================== */
// 单轮打滑检测结构 (基于加速度的方法，解决循环依赖)
typedef struct {
    uint8_t is_slipping;     // 是否打滑
    float deviation;         // 打滑指标 (mm/s)
    float expected_dv;       // 期望速度变化 (IMU预测)
    float actual_dv;         // 实际速度变化 (轮速差分)
    float last_speed;        // 上一帧轮速 (mm/s)
} WheelSlip_t;

// 轮子索引定义
#define WHEEL_LF 0
#define WHEEL_RF 1
#define WHEEL_LB 2
#define WHEEL_RB 3

static WheelSlip_t wheel_slip[4] = {0};

// TCS参数 (基于加速度差异的打滑检测，仅用于调整卡尔曼R)
#define TCS_SLIP_THRESHOLD       30.0f   // 打滑判定阈值 (mm/s 速度变化差)
/* ===================== VOFA可调参数 ===================== */
// IMU安装偏移 
static float imu_offset_x = CHASSIS_IMU_OFFSET_X;  // X方向偏移 (mm)
static float imu_offset_y = CHASSIS_IMU_OFFSET_Y;  // Y方向偏移 (mm)
/* ===================== 力矩前馈变量 ===================== */
// 每个轮子的电流前馈值
static float current_ff_lf = 0.0f;
static float current_ff_rf = 0.0f;
static float current_ff_lb = 0.0f;
static float current_ff_rb = 0.0f;

// 全局观测器计算的补偿电流 (X/Y方向)
static float global_Ix = 0.0f;  // X方向补偿电流
static float global_Iy = 0.0f;  // Y方向补偿电流
static float global_Iwz = 0.0f; // 旋转补偿电流

// 上一周期的补偿电流 (用于DOB)
static float last_Ix = 0.0f;
static float last_Iy = 0.0f;

/* ===================== SMC+DOB 控制 ===================== */
static uint8_t observer_inited = 0;

// 静止检测: 用于清零SMC和ESO状态
#define SMC_ESO_RESET_THRESHOLD  30   // 静止检测阈值 (30*2ms=60ms)
static uint32_t zero_input_cnt = 0;    // 零输入计数器

// 超电重启冷却时间 (500*2ms=1s)
#define CAP_RESTART_COOLDOWN 2000

/* 根据robot_def.h中的macro自动计算的参数 */
float L=200; //轮轴距中心距离
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用双板CAN通信
#include "can_comm.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
#endif // CHASSIS_BOARD

#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
#include "ins_task.h"
static INS_t *Chassis_INS_data;           // 完整IMU数据（包含MotionAccel_b）
#endif

#ifdef CHASSIS_ONLY
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // CHASSIS_ONLY
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据


static referee_info_t* referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

// 底盘电机离线检测实例
static MotorOfflineAlarmInstance *chassis_offline_alarm = NULL;

/* 用于自旋变速策略的时间变量 */
static float pos_variable_rotate_speed[8] = {1.3, 1.4, 1.5, 1.6, 1.7, 1.75, 1.8, 1.85};

static uint16_t DataSend2Cap[4] = {0,0,0,0};
static float chassis_SPEED_PID[3]={2,0,0};
static float chassis_CURRENT_PID[3]={0.5,0,0};

/* 用于自旋变速策略的时间变量,后续考虑查表加速 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘, 单位: degree/s (电机转子角速度)
static float vt_lf, vt_rf, vt_lb, vt_rb; // 混合控制: 各轮目标速度, 单位: degree/s (电机转子角速度)

static float rotate_speed_buff = 0;
void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    // 混合控制模式: 速度环 + 电流环，启用电流前馈
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            // 速度环 (主控制)
            .speed_PID = {
                .Kp = 3.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Integral_Limit,
                .MaxOut = 15000,
            },
            // 电流环 (内环)
            .current_PID = {
                .Kp = 0.5f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Integral_Limit,
                .MaxOut = 16000,
            },
            // 电流前馈指针 (后面单独设置)
            .current_feedforward_ptr = NULL,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,                    // 外环为速度环
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,     // 速度+电流双闭环
            .feedforward_flag = CURRENT_FEEDFORWARD,          // 启用电流前馈
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

    // 注册底盘电机用于功率限制
    DJIMotorSetPowerLimitMotors(motor_lf, M3508);
    DJIMotorSetPowerLimitMotors(motor_rf, M3508);
    DJIMotorSetPowerLimitMotors(motor_lb, M3508);
    DJIMotorSetPowerLimitMotors(motor_rb, M3508);

    // 裁判系统初始化 - 如果没有连接裁判系统可以注释掉
    // 注意: 如果注释掉,需要在LimitChassisOutput()中处理referee_data的空指针问题
    referee_data = Referee_Interactive_init(&huart6,&ui_data); 

    // 超级电容初始化 - 如果没有超级电容可以注释掉
    // 注意: 如果注释掉,需要在LimitChassisOutput()中注释掉SuperCapSend()调用
    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x061, // 超级电容默认接收id通信can1
            .rx_id = 0x051, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // ww超级电容初始化
    
    // 发布订阅初始化,如果为双板,则需要can comm来传递消息    
#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
    INS_Init(); // 底盘IMU初始化
    Chassis_INS_data = INS_GetDataPtr(); // 获取完整IMU数据指针（包含MotionAccel_b）
    
    // 初始化速度融合卡尔曼滤波器 (混合控制参数)
    SimpleKalman1D_Init(&kf_vx, 0.1f, 0.8f, 0.0f);
    SimpleKalman1D_Init(&kf_vy, 0.1f, 0.8f, 0.0f);
    
    // 初始化二阶滑模控制器 (Super-Twisting算法)
    smc_vx.lambda = SMC_LAMBDA;
    smc_vx.st_k1 = SMC_K1;
    smc_vx.st_k2 = SMC_K2;
    smc_vx.phi = SMC_PHI;
    smc_vx.v_integral = 0;
    smc_vx.v_max = SMC_V_MAX;
    smc_vx.disturbance = 0;
    smc_vx.v_err_last = 0;
    smc_vx.s_last = 0;
    
    smc_vy = smc_vx;  // Y方向使用相同参数
    
    // 初始化扩展状态观测器 (ESO)
    // 增益配置: β1 = 2*ω0, β2 = ω0² (极点配置法)
    eso_x.omega0 = ESO_OMEGA0;
    eso_x.beta1 = 2.0f * ESO_OMEGA0;
    eso_x.beta2 = ESO_OMEGA0 * ESO_OMEGA0;
    eso_x.b0 = ESO_B0;
    eso_x.z1 = 0;
    eso_x.z2 = 0;
    
    eso_y = eso_x;  // Y方向使用相同参数
    
    // 标记观测器初始化完成
    observer_inited = 1;
    
    // 初始化打滑检测状态
    for (int i = 0; i < 4; i++) {
        wheel_slip[i].is_slipping = 0;
    }
    
    // VOFA接收绑定 - 用于实时调参
    // CH0: imu_offset_x, CH1: imu_offset_y
    VOFA_BIND(0, &imu_offset_x, &imu_offset_y);
#endif // CHASSIS_BOARD || CHASSIS_ONLY

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
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef CHASSIS_ONLY // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // CHASSIS_ONLY

    // 底盘电机离线检测配置 
    MotorOfflineAlarmConfig_t chassis_alarm_cfg = {
        .motors = {motor_rf, motor_lb, motor_rb, motor_lf},
        .beep_times = {1, 2, 3, 4},     // RF=1声, LB=2声, RB=3声, LF=4声
        .motor_count = 4,
        .buzzer_freq = ALARM_FREQ_LOW,  // 低音调
        .run_buzzer_task = 1,           // 主模块执行BuzzerTask
    };
    chassis_offline_alarm = MotorOfflineAlarmRegister(&chassis_alarm_cfg);
}

#define CENTER1 ((L + CENTER_GIMBAL_OFFSET_X - CENTER_GIMBAL_OFFSET_Y) *DEGREE_2_RAD)
#define CENTER2 ((L - CENTER_GIMBAL_OFFSET_X - CENTER_GIMBAL_OFFSET_Y) *DEGREE_2_RAD)   //数字与轮子的对应关系参考坐标系象限
#define CENTER3 ((L + CENTER_GIMBAL_OFFSET_X + CENTER_GIMBAL_OFFSET_Y) *DEGREE_2_RAD)
#define CENTER4 ((L - CENTER_GIMBAL_OFFSET_X + CENTER_GIMBAL_OFFSET_Y) *DEGREE_2_RAD)

/**
 * @brief 计算各轮目标速度 (麦轮/全向轮逆运动学)
 * @note 输入输出单位均为 degree/s (电机转子角速度)
 */
static void MecanumCalculate()
{   
    // X型全向轮逆运动学: 底盘速度 -> 各轮速度 (单位: degree/s)
    vt_lf = (-chassis_vx - chassis_vy) / Sqrt(2) + chassis_cmd_recv.wz * CENTER2;
    vt_rf = (chassis_vx - chassis_vy) / Sqrt(2) - chassis_cmd_recv.wz * CENTER1;
    vt_lb = (-chassis_vx + chassis_vy) / Sqrt(2) - chassis_cmd_recv.wz * CENTER3;
    vt_rb = (chassis_vx + chassis_vy) / Sqrt(2) + chassis_cmd_recv.wz * CENTER4;
}

/**
 * @brief 平滑符号函数 (边界层法)
 * @param s 输入值
 * @param phi 边界层厚度
 * @return 平滑后的符号值 [-1, 1]
 */
static float SmoothSign(float s, float phi)
{
    if (phi < 0.001f) phi = 0.001f;
    if (fabsf(s) < phi) {
        return s / phi;
    } else {
        return (s > 0) ? 1.0f : -1.0f;
    }
}

/**
 * @brief 二阶滑模控制器计算 (Super-Twisting算法)
 * @param smc 控制器实例
 * @param v_err 速度误差 (mm/s)
 * @param dt 时间步长 (s)
 * @return 补偿电流
 */
static float SMC_Calculate(SMC_Controller_t *smc, float v_err, float dt)
{
    // 速度误差导数 (数值微分)
    float v_err_dot = 0.0f;
    if (dt > 0.0001f) {
        v_err_dot = (v_err - smc->v_err_last) / dt;
    }
    smc->v_err_last = v_err;
    
    // 滑模面: s = v̇_err + λ * v_err
    float s = v_err_dot + smc->lambda * v_err;
    smc->s_last = s;
    
    // Super-Twisting算法
    // 比例项: u1 = -k1 * |s|^0.5 * sign(s)
    float abs_s = fabsf(s);
    float sqrt_abs_s = sqrtf(abs_s + 0.0001f);
    float sign_s = SmoothSign(s, smc->phi);
    
    float u1 = -smc->st_k1 * sqrt_abs_s * sign_s;
    
    // 积分项动态: dv/dt = -k2 * sign(s)
    smc->v_integral += -smc->st_k2 * sign_s * dt;
    
    // 积分项限幅
    if (smc->v_integral > smc->v_max) {
        smc->v_integral = smc->v_max;
    } else if (smc->v_integral < -smc->v_max) {
        smc->v_integral = -smc->v_max;
    }
    
    float u2 = smc->v_integral;
    
    // 总控制输出 + 扰动补偿
    float u = u1 + u2 + smc->disturbance;
    
    // 限幅
    if (u > SMC_MAX_OUT) u = SMC_MAX_OUT;
    else if (u < -SMC_MAX_OUT) u = -SMC_MAX_OUT;
    
    return u;
}

/**
 * @brief 扩展状态观测器更新 (LESO - 线性ESO)
 * @param eso 观测器实例
 * @param y 测量速度 (mm/s)
 * @param u 控制输入 (电流)
 * @param dt 时间步长 (s)
 * @return 总扰动估计
 * 
 * @note ESO状态方程:
 *       e = z1 - y
 *       ż1 = z2 - β1*e + b0*u
 *       ż2 = -β2*e
 *       
 *       优点: 
 *       1. 不需要精确模型，将所有不确定性视为“总扰动”
 *       2. 只需调一个参数ω0 (带宽)
 *       3. 同时估计状态和扰动
 */
static float ESO_Update(ESO_t *eso, float y, float u, float dt)
{
    // 观测误差
    float e = eso->z1 - y;
    
    // ESO状态更新
    // ż1 = z2 - β1*e + b0*u
    float z1_dot = eso->z2 - eso->beta1 * e + eso->b0 * u;
    // ż2 = -β2*e
    float z2_dot = -eso->beta2 * e;
    
    // 欧拉积分
    eso->z1 += z1_dot * dt;
    eso->z2 += z2_dot * dt;
    
    // 扰动估计限幅 (防止发散)
    if (eso->z2 > ESO_D_MAX) eso->z2 = ESO_D_MAX;
    else if (eso->z2 < -ESO_D_MAX) eso->z2 = -ESO_D_MAX;
    
    return eso->z2;  // 返回总扰动估计
}

/**
 * @brief 全局观测器: 二阶滑模控制 + 扩展状态观测器(ESO)
 * 
 * 控制架构:
 *    目标速度 ──> 逆运动学 ──> 各轮目标速度 ──> 电机速度环 ──┬──> 电机电流环 ──> 输出
 *                                                        │
 *    速度误差 ──> SMC(Super-Twisting) + ESO ──> 电流前馈 ──┘
 */
/**
 * @brief 重置SMC和ESO状态 (静止时调用)
 */
static void ResetSMC_ESO(void)
{
    // 重置SMC积分和状态
    smc_vx.v_integral = 0;
    smc_vx.v_err_last = 0;
    smc_vx.s_last = 0;
    smc_vx.disturbance = 0;
    
    smc_vy.v_integral = 0;
    smc_vy.v_err_last = 0;
    smc_vy.s_last = 0;
    smc_vy.disturbance = 0;
    
    // 重置ESO状态
    eso_x.z1 = 0;
    eso_x.z2 = 0;
    eso_y.z1 = 0;
    eso_y.z2 = 0;
    
    // 重置输出
    global_Ix = 0;
    global_Iy = 0;
    last_Ix = 0;
    last_Iy = 0;
    
    // 重置前馈
    current_ff_lf = 0;
    current_ff_rf = 0;
    current_ff_lb = 0;
    current_ff_rb = 0;
}

static void GlobalObserverCalculate(float dt)
{
#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
    if (!observer_inited) return;
    
    // ==================== 静止检测: 清零SMC和ESO ====================
    // 当遥控器vx/vy都接近0时开始计数，达到阈值后重置状态
    if (fabsf(chassis_cmd_recv.vx) < 10.0f && fabsf(chassis_cmd_recv.vy) < 10.0f) {
        zero_input_cnt++;
        if (zero_input_cnt >= SMC_ESO_RESET_THRESHOLD) {
            ResetSMC_ESO();
            zero_input_cnt = SMC_ESO_RESET_THRESHOLD;  // 防止溢出
            return;  // 静止时不计算
        }
    } else {
        zero_input_cnt = 0;  // 有输入时重置计数器
    }
    
    // ==================== 获取速度误差 ==
    // 目标速度 (底盘速度 -> 轮子线速度 mm/s)
    float target_vx_wheel = chassis_vx * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    float target_vy_wheel = chassis_vy * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    
    // 实际速度 (融合后的底盘速度 mm/s)
    float actual_vx = chassis_feedback_data.real_vx;
    float actual_vy = chassis_feedback_data.real_vy;
    
    // 速度误差
    float vx_err = target_vx_wheel - actual_vx;
    float vy_err = target_vy_wheel - actual_vy;
    
    // ==================== 更新ESO ====================
    // 使用上一周期的控制量更新扰动估计
    float dx = ESO_Update(&eso_x, actual_vx, last_Ix, dt);
    float dy = ESO_Update(&eso_y, actual_vy, last_Iy, dt);
    
    // 将扰动补偿传给滑模控制器 (负号: 补偿要抵消扰动)
    smc_vx.disturbance = -dx;
    smc_vy.disturbance = -dy;
    
    // ==================== SMC计算 ====================
    global_Ix = SMC_Calculate(&smc_vx, vx_err, dt);
    global_Iy = SMC_Calculate(&smc_vy, vy_err, dt);
    
    // 保存本周期控制量供下次ESO使用
    last_Ix = global_Ix;
    last_Iy = global_Iy;
    
    // ==================== 分配到各轮 ====================
    // 逆运动学: XY方向电流 -> 各轮电流前馈
    current_ff_lf = (-global_Ix - global_Iy) / (2.0f * Sqrt(2));
    current_ff_rf = ( global_Ix - global_Iy) / (2.0f * Sqrt(2));
    current_ff_lb = (-global_Ix + global_Iy) / (2.0f * Sqrt(2));
    current_ff_rb = ( global_Ix + global_Iy) / (2.0f * Sqrt(2));
    
#else
    // 无IMU模式
    current_ff_lf = 0;
    current_ff_rf = 0;
    current_ff_lb = 0;
    current_ff_rb = 0;
#endif
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 */
struct CapTxMsg
{
    uint8_t enableDCDC : 1;       // 是否启动DC-DC转换器
    uint8_t systemRestart : 1;    // 系统重启指令，正常为0
    uint8_t resv0 : 6;            // 保留6字节方便对齐
    uint16_t RefereePowerLimit;   // 裁判系统功率限制
    uint16_t RefereeEnergyBuffer; // 裁判系统能量缓冲区
    uint8_t resv1[3];             // 保留3个字节
} __attribute__((packed));
static float chassis_powerlimit = 0;
static void LimitChassisOutput()
{   
    static float chassis_power_buff = 1;

    // 超级电容控制: 当电容能量低于65%时关闭DC-DC，避免过放
    uint8_t enableDCDC = (cap->cap_msg.capEnergy >= 65) ? 1 : 0;
    
    // 功率限制: DC-DC开启时使用超电输出功率，关闭时固定100W
    if (enableDCDC) {
        chassis_powerlimit = 40 + cap->cap_msg.chassisPowerLimit;
    } else {
        chassis_powerlimit = 40.0f; 
    }
        ChassisPowerSet(chassis_powerlimit);
    static uint32_t cnt = 0;
    if ((cnt++) % 125 == 0)  // 125*2ms=250ms
    {
        rotate_speed_buff = pos_variable_rotate_speed[GetRandomInt(0, 7)];
    }
    
    rotate_speed_buff = 1.5;
    chassis_power_buff = 3.0;

    // 混合控制模式: 设置目标速度，前馈已在GlobalObserverCalculate中更新
    // 电机模块会自动使用速度环+电流前馈
    // 注: 打滑检测仅用于调整卡尔曼R值，不再降低速度目标
    //     SMC+ESO会自动补偿打滑造成的扰动
    if(chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE)
    {
        // 自旋模式: 速度增强
        DJIMotorSetRef(motor_lf, vt_lf * 1.25f);
        DJIMotorSetRef(motor_rf, vt_rf * 1.25f);
        DJIMotorSetRef(motor_lb, vt_lb * 1.25f);
        DJIMotorSetRef(motor_rb, vt_rb * 1.25f);
    }
    else
    {
        DJIMotorSetRef(motor_lf, vt_lf);
        DJIMotorSetRef(motor_rf, vt_rf);
        DJIMotorSetRef(motor_lb, vt_lb);
        DJIMotorSetRef(motor_rb, vt_rb);
    }


    // 超电错误检测与重启: 检测到错误时发送重启指令，冷却后再次检测
    static uint32_t cooldown = 0;
    uint8_t restart = 0;
    
    if (cooldown > 0) {
        cooldown--;
    } else if (cap->cap_msg.errorCoad != 0) {
        restart = 1;
        cooldown = CAP_RESTART_COOLDOWN;
    }
    
    // 超级电容控制: 发送裁判系统功率限制和能量缓冲区  
    struct CapTxMsg cap_msg = {
        .enableDCDC = enableDCDC,
        .systemRestart = 0, //restart,
        .RefereePowerLimit = 100,
        .RefereeEnergyBuffer = 60,
    };
    SuperCapSend(cap, (uint8_t*)&cap_msg);

}

/**
 * @brief 速度估计与打滑检测 (混合控制模式)
 *        使用基于加速度的打滑检测，解决循环依赖问题
 * @note 内部计算使用 mm/s (轮子线速度) 进行融合和打滑检测
 * @return 时间步长dt (s)
 */
static float EstimateSpeed()
{
    // 获取四个轮子的线速度: 电机角速度(degree/s) -> 轮子线速度(mm/s)
    float wheel_lf = motor_lf->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    float wheel_rf = motor_rf->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    float wheel_lb = motor_lb->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    float wheel_rb = motor_rb->measure.speed_aps * DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
    
    // 正运动学: 轮速(mm/s) -> 底盘速度(mm/s), 用于速度融合和打滑检测
    float wheel_vx = (-wheel_lf + wheel_rf - wheel_lb + wheel_rb) / (2.0f * Sqrt(2));
    float wheel_vy = (-wheel_lf - wheel_rf + wheel_lb + wheel_rb) / (2.0f * Sqrt(2));
    
    // 角速度wz
    chassis_feedback_data.real_wz = (wheel_lf + wheel_rf + wheel_lb + wheel_rb) 
                                    / (4.0f * L * DEGREE_2_RAD);

#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
    static uint32_t dwt_cnt = 0;
    float dt = DWT_GetDeltaT(&dwt_cnt);
    if (dt > 0.1f) dt = 0.001f;
    
    // 使用已去重力的运动加速度 MotionAccel_b (m/s^2 -> mm/s^2)
    float imu_accel_x = -Chassis_INS_data->MotionAccel_b[0] * 1000.0f;
    float imu_accel_y = Chassis_INS_data->MotionAccel_b[1] * 1000.0f;
    
    // IMU安装偏移补偿: a_center = a_imu - omega × (omega × r) - alpha × r
    // 需要补偿两种加速度:
    // 1. 向心加速度: a_cent = omega^2 * r (指向旋转中心)
    // 2. 切向加速度: a_tang = alpha * r (角加速度引起)
    
    // 获取Yaw轴角速度和角加速度
    float omega_yaw = Chassis_INS_data->Gyro[2];       // Z轴角速度 (rad/s)
    float alpha_yaw = Chassis_INS_data->GyroAlpha[2];  // Z轴角加速度 (rad/s^2)
    
    float omega_yaw_sq = omega_yaw * omega_yaw;
    
    // ========== 1. 向心加速度补偿 ==========
    // Yaw旋转补偿 (绕Z轴): X/Y方向偏移产生X/Y方向向心加速度
    imu_accel_x -= omega_yaw_sq * imu_offset_x;
    imu_accel_y -= omega_yaw_sq * imu_offset_y;
    
    // ========== 2. 切向加速度补偿 ==========
    // 切向加速度公式: a_tang = alpha × r
    // Yaw角加速度 (绕Z轴): a_tang_x = -alpha_z * r_y, a_tang_y = alpha_z * r_x
    imu_accel_x -= (-alpha_yaw * imu_offset_y);  // 注意符号
    imu_accel_y -= (alpha_yaw * imu_offset_x);   // 不补偿会导致自旋时y轴突变

    // ==================== IMU原始积分速度（开环，用于调试对比，正式上车时删除） ====================
    static float imu_raw_vx = 0.0f;  // IMU开环积分速度X
    static float imu_raw_vy = 0.0f;  // IMU开环积分速度Y
    static uint32_t zero_speed_cnt = 0;  // 零速计数器
    #define ZERO_SPEED_HOLD_TIME 250  // 保持静止的时间阈值 (250*2ms=500ms)
    
    imu_raw_vx += imu_accel_x * dt;
    imu_raw_vy += imu_accel_y * dt;
    
    // 静止检测：轮速接近0并保持一段时间后才重置IMU积分
    if (fabsf(wheel_vx) < 2.0f && fabsf(wheel_vy) < 2.0f) {
        zero_speed_cnt++;
        if (zero_speed_cnt >= ZERO_SPEED_HOLD_TIME) {
            imu_raw_vx = 0.0f;
            imu_raw_vy = 0.0f;
            zero_speed_cnt = ZERO_SPEED_HOLD_TIME;  // 防止溢出
        }
    } else {
        zero_speed_cnt = 0;  // 运动时重置计数器
    }

    // 卡尔曼滤波融合: 用IMU加速度预测, 用轮速校正
    chassis_feedback_data.real_vx = SimpleKalman1D_Update(&kf_vx, imu_accel_x, wheel_vx, dt);
    chassis_feedback_data.real_vy = SimpleKalman1D_Update(&kf_vy, imu_accel_y, wheel_vy, dt);
    
    // ==================== 基于加速度的打滑检测 ====================
    // 用IMU加速度直接预测各轮期望速度变化，与实际轮速变化对比
    
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
    float wheel_speeds[4] = {wheel_lf, wheel_rf, wheel_lb, wheel_rb};
    
    uint8_t any_wheel_slipping = 0;
    for (int i = 0; i < 4; i++) {
        wheel_slip[i].expected_dv = expected_dv[i];
        wheel_slip[i].actual_dv = actual_dv[i];
        
        // 打滑指标: 期望与实际速度变化的差异
        float slip_indicator = fabsf(actual_dv[i] - expected_dv[i]);
        wheel_slip[i].deviation = slip_indicator;
        
        // 打滑判定: 差异超过阈值且轮速足够大(避免静止时误判)
        float wheel_speed_abs = fabsf(wheel_speeds[i]);
        
        if (slip_indicator > TCS_SLIP_THRESHOLD && wheel_speed_abs > 100.0f) {
            wheel_slip[i].is_slipping = 1;
            any_wheel_slipping = 1;
        } else {
            wheel_slip[i].is_slipping = 0;
        }
    }
    
    // VOFA+调试输出
    VOFA(0, imu_raw_vx, imu_raw_vy, wheel_vx, wheel_vy,
         chassis_feedback_data.real_vx, chassis_feedback_data.real_vy, 
         imu_accel_x, imu_accel_y, 
         (float)wheel_slip[0].is_slipping, (float)wheel_slip[1].is_slipping,
         (float)wheel_slip[2].is_slipping, (float)wheel_slip[3].is_slipping);
    
    // ==================== 卡尔曼R值调整 ====================
    // 根据打滑轮数量调整: 打滑轮越多，轮速可信度越低
    if (any_wheel_slipping) {
        uint8_t slip_wheel_count = 0;
        for (int i = 0; i < 4; i++) {
            if (wheel_slip[i].is_slipping) slip_wheel_count++;
        }
        // 1轮: R=1.5, 2轮: R=2.2, 3轮: R=3.0, 4轮: R=4.0
        float slip_R = 0.8f + 0.7f * slip_wheel_count;
        kf_vx.R = slip_R;
        kf_vy.R = slip_R;
    } else {
        // 恢复正常卡尔曼R
        kf_vx.R = 0.8f;
        kf_vy.R = 0.8f;
    }
    
    return dt;
#else
    chassis_feedback_data.real_vx = wheel_vx;
    chassis_feedback_data.real_vy = wheel_vy;
    return 0.002f;  // 默认时间步长
#endif
}
/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 电机离线报警: RF=1声, LB=2声, RB=3声, LF=4声
    
    //MotorOfflineAlarmTask(chassis_offline_alarm);
    
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef CHASSIS_ONLY
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
#ifdef CHASSIS_ONLY
        // CHASSIS_ONLY模式: 保留cmd发来的wz值(左摇杆控制)
        break;
#else
        chassis_cmd_recv.wz = 0.5*chassis_cmd_recv.offset_angle*abs(chassis_cmd_recv.offset_angle);
        break;
#endif
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 1000*rotate_speed_buff;
        break;
    default:
        break;
    }
  
    
    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘***逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)*
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

    chassis_vx = (chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta);
    chassis_vy = (chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta);

    // 先计算真实速度(用于速度外环和打滑检测)
    // 注意: 必须在GlobalObserverCalculate之前调用!
    float dt = EstimateSpeed();
    
    // 计算各轮目标速度 (送入电机速度环)
    MecanumCalculate();
    
    // 全局观测器计算前馈补偿 (送入电机电流前馈)
    GlobalObserverCalculate(dt);

    // 设置电机参考值
    LimitChassisOutput();

    // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // 我方颜色id小于7是红色,大于7是蓝色, 0:red , 1:blue
    chassis_feedback_data.self_color = referee_data->GameRobotState.robot_id > 7 ? COLOR_BLUE : COLOR_RED;
    //当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
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
    ui_data.capEnergy = cap->cap_msg.capEnergy;  // 超级电容未启用时注释掉

    // 推送反馈消息
#ifdef CHASSIS_ONLY
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}
