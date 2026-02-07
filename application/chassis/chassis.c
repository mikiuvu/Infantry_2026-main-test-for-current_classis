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

// 速度融合卡尔曼滤波器实例
static SimpleKalman1D_t kf_vx, kf_vy;

/* ===================== 打滑检测与牵引力控制 ===================== */
// 单轮打滑检测结构
typedef struct {
    uint8_t is_slipping;     // 是否打滑
    float deviation;         // 实际轮速与期望轮速的偏差 (mm/s)
    float tcs_factor;        // 该轮的TCS系数 
    float expected_speed;    // 期望轮速 (根据融合速度计算)
    float actual_speed;      // 实际轮速
} WheelSlip_t;

// 轮子索引定义
#define WHEEL_LF 0
#define WHEEL_RF 1
#define WHEEL_LB 2
#define WHEEL_RB 3

static WheelSlip_t wheel_slip[4] = {0};

// TCS参数 (滑移率控制)
#define TCS_TARGET_SLIP_RATIO    0.20f   // 目标滑移率上限 (20%)
#define TCS_SLIP_DEADBAND        0.10f   // 滑移率死区 (10%)，低于此值不干预
#define TCS_MIN_FACTOR           0.3f    // TCS最小输出系数
#define TCS_RECOVERY_RATE        0.008f  // TCS恢复速率 (每周期, 500Hz)
#define TCS_RESPONSE_GAIN        4.0f    // 滑移率超限时的响应增益
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

// 全局观测器计算的补偿力
static float global_Fx = 0.0f;  // X方向补偿力 (N)
static float global_Fy = 0.0f;  // Y方向补偿力 (N)
static float global_Mz = 0.0f;  // 旋转补偿力矩 (N*m)

/* ===================== 单轮补偿PID ===================== */
// 各轮独立的速度误差补偿PID (期望轮速 vs 实际轮速)
static PIDInstance wheel_ff_pid[4];  // LF, RF, LB, RB
static uint8_t observer_inited = 0;


/* 根据robot_def.h中的macro自动计算的参数 */
float L=200; //轮轴距中心距离
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
static INS_t *Chassis_INS_data;           // 完整IMU数据（包含MotionAccel_b）
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据


static referee_info_t* referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

/* ===================== 电机离线蜂鸣器报警 ===================== */
#define MOTOR_OFFLINE_CHECK_PERIOD  1000  // 离线检测周期(2ms*1000=2s)

/**
 * @brief 电机离线报警函数 - 离线电机响对应CAN ID次数
 * @note LF=4声(ID4), RF=1声(ID1), LB=2声(ID2), RB=3声(ID3)
 *       BuzzerTask周期=2ms, 参数单位为周期数
 */
static void MotorOfflineBeepAlarm(void)
{
    static uint32_t check_cnt = 0;
    
    BuzzerTask();  // 执行蜂鸣器状态机
    
    if (++check_cnt >= MOTOR_OFFLINE_CHECK_PERIOD) {
        check_cnt = 0;
        if (BuzzerIsBusy()) return;
        
        // BuzzerBeep(次数, 响周期数, 间隔周期数, 音调)
        // 50周期*2ms=100ms响, 75周期*2ms=150ms间隔
        // 响声次数 = CAN ID
        if (!DJIMotorIsOnline(motor_rf))       // ID=1
            BuzzerBeep(1, 50, 75, 4);
        else if (!DJIMotorIsOnline(motor_lb))  // ID=2
            BuzzerBeep(2, 50, 75, 4);
        else if (!DJIMotorIsOnline(motor_rb))  // ID=3
            BuzzerBeep(3, 50, 75, 4);
        else if (!DJIMotorIsOnline(motor_lf))  // ID=4
            BuzzerBeep(4, 50, 75, 4);
    }
}

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
    // referee_data = Referee_Interactive_init(&huart6,&ui_data); 

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
#ifdef CHASSIS_BOARD
    INS_Init(); // 底盘IMU初始化
    Chassis_INS_data = INS_GetDataPtr(); // 获取完整IMU数据指针（包含MotionAccel_b）
    
    // 初始化速度融合卡尔曼滤波器 (混合控制参数)
    SimpleKalman1D_Init(&kf_vx, 0.1f, 0.8f, 0.0f);
    SimpleKalman1D_Init(&kf_vy, 0.1f, 0.8f, 0.0f);
    
    // 初始化单轮补偿PID (速度误差 -> 电流补偿)
    // 用于：期望轮速 vs 实际轮速 -> 补偿电流
    PID_Init_Config_s wheel_ff_pid_conf = {
        .Kp = 5.0f,
        .Ki = 0.5f,
        .Kd = 0.1f,
        .MaxOut = 3000.0f,  // 最大补偿电流
        .IntegralLimit = 1000.0f,
        .DeadBand = 10.0f,  // 死区 10mm/s
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    };
    for (int i = 0; i < 4; i++) {
        PIDInit(&wheel_ff_pid[i], &wheel_ff_pid_conf);
    }
    // 标记观测器初始化完成
    observer_inited = 1;
    
    // 初始化单轮TCS系数
    for (int i = 0; i < 4; i++) {
        wheel_slip[i].tcs_factor = 1.0f;
        wheel_slip[i].is_slipping = 0;
    }
    
    // VOFA接收绑定 - 用于实时调参
    // CH0: imu_offset_x, CH1: imu_offset_y
    VOFA_BIND(0, &imu_offset_x, &imu_offset_y);

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

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
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
 * @brief 全局观测器: 根据融合速度与目标速度的误差计算补偿力矩
 *        补偿力矩通过电流前馈作用于电机
 * 
 * 控制架构 (单轮独立补偿):
 *    目标速度 ──> 逆运动学 ──> 各轮目标速度 ──> 电机速度环 ──┬──> 电机电流环 ──> 输出
 *                                                        │
 *    融合速度 ──> 逆运动学 ──> 各轮期望速度 ──┐             │
 *                                          ├─> 单轮PID ──> 电流前馈 ──────┘
 *    各轮实际速度 ─────────────────────────┘
 * 
 */
static void GlobalObserverCalculate()
{
#ifdef CHASSIS_BOARD
    if (!observer_inited) return;
    
    // ==================== 单轮独立补偿 ====================
    // 使用打滑检测中已计算的期望轮速和实际轮速
    // expected_speed: 根据融合速度(IMU+轮速)计算的期望轮速
    // actual_speed: 电机反馈的实际轮速
    
    // 判断是否为卡住状态 (期望 > 实际)
    // 卡住: 期望 > 实际 → 需要PID补偿
    // 空转: 期望 < 实际 → 不补偿(由TCS降低)
    uint8_t is_stuck[4];
    is_stuck[WHEEL_LF] = (wheel_slip[WHEEL_LF].expected_speed > wheel_slip[WHEEL_LF].actual_speed);
    is_stuck[WHEEL_RF] = (wheel_slip[WHEEL_RF].expected_speed > wheel_slip[WHEEL_RF].actual_speed);
    is_stuck[WHEEL_LB] = (wheel_slip[WHEEL_LB].expected_speed > wheel_slip[WHEEL_LB].actual_speed);
    is_stuck[WHEEL_RB] = (wheel_slip[WHEEL_RB].expected_speed > wheel_slip[WHEEL_RB].actual_speed);
    
    // 使用PID计算补偿电流 (PIDCalculate自带限幅)
    // 注意: PIDCalculate(pid, measure, ref) = ref - measure 的PID输出
    float ff_lf = PIDCalculate(&wheel_ff_pid[WHEEL_LF], 
                               wheel_slip[WHEEL_LF].actual_speed,
                               wheel_slip[WHEEL_LF].expected_speed);
    float ff_rf = PIDCalculate(&wheel_ff_pid[WHEEL_RF],
                               wheel_slip[WHEEL_RF].actual_speed,
                               wheel_slip[WHEEL_RF].expected_speed);
    float ff_lb = PIDCalculate(&wheel_ff_pid[WHEEL_LB],
                               wheel_slip[WHEEL_LB].actual_speed,
                               wheel_slip[WHEEL_LB].expected_speed);
    float ff_rb = PIDCalculate(&wheel_ff_pid[WHEEL_RB],
                               wheel_slip[WHEEL_RB].actual_speed,
                               wheel_slip[WHEEL_RB].expected_speed);
    
    // 只对"卡住"方向补偿，空转由TCS处理
    // 卡住时保留正向补偿，空转时清零
    ff_lf = is_stuck[WHEEL_LF] ? fmaxf(ff_lf, 0.0f) : 0.0f;
    ff_rf = is_stuck[WHEEL_RF] ? fmaxf(ff_rf, 0.0f) : 0.0f;
    ff_lb = is_stuck[WHEEL_LB] ? fmaxf(ff_lb, 0.0f) : 0.0f;
    ff_rb = is_stuck[WHEEL_RB] ? fmaxf(ff_rb, 0.0f) : 0.0f;
    
    // 应用力补偿
    current_ff_lf = ff_lf;
    current_ff_rf = ff_rf;
    current_ff_lb = ff_lb;
    current_ff_rb = ff_rb;
    
#else
    // 单板模式
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

static void LimitChassisOutput()
{   
    static float chassis_power_buff = 1;

    ChassisPowerSet(30);
    
    static uint32_t cnt = 0;
    if ((cnt++) % 125 == 0)  // 125*2ms=250ms
    {
        rotate_speed_buff = pos_variable_rotate_speed[GetRandomInt(0, 7)];
    }
    
    rotate_speed_buff = 1.5;
    chassis_power_buff = 3.0;

    // 混合控制模式: 设置目标速度，前馈已在GlobalObserverCalculate中更新
    // 电机模块会自动使用速度环+电流前馈
    // TCS: 根据滑移率降低速度参考值，限制打滑
    float tcs_lf = wheel_slip[WHEEL_LF].tcs_factor;
    float tcs_rf = wheel_slip[WHEEL_RF].tcs_factor;
    float tcs_lb = wheel_slip[WHEEL_LB].tcs_factor;
    float tcs_rb = wheel_slip[WHEEL_RB].tcs_factor;
    
    if(chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE)
    {
        // 自旋模式: 速度增强，同时应用TCS
        DJIMotorSetRef(motor_lf, vt_lf * 1.25f * tcs_lf);
        DJIMotorSetRef(motor_rf, vt_rf * 1.25f * tcs_rf);
        DJIMotorSetRef(motor_lb, vt_lb * 1.25f * tcs_lb);
        DJIMotorSetRef(motor_rb, vt_rb * 1.25f * tcs_rb);
    }
    else
    {
        DJIMotorSetRef(motor_lf, vt_lf * tcs_lf);
        DJIMotorSetRef(motor_rf, vt_rf * tcs_rf);
        DJIMotorSetRef(motor_lb, vt_lb * tcs_lb);
        DJIMotorSetRef(motor_rb, vt_rb * tcs_rb);
    }
}

/**
 * @brief 速度估计与打滑检测 (混合控制模式)
 *        使用轮速与融合速度的偏差来检测打滑
 * @note 内部计算使用 mm/s (轮子线速度) 进行融合和打滑检测
 */
static void EstimateSpeed()
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

#ifdef CHASSIS_BOARD
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
    
    // ==================== 单轮打滑检测 ====================
    // 原理: 根据融合速度计算每个轮子的期望速度，与实际轮速对比
    // 如果某个轮子偏差大，则认为该轮打滑
    
    // 使用融合速度和IMU角速度计算期望轮速 (逆运动学)
    float real_vx = chassis_feedback_data.real_vx;  // 融合速度 (mm/s)
    float real_vy = chassis_feedback_data.real_vy;  // 融合速度 (mm/s)
    float real_wz = omega_yaw;  // 使用IMU角速度 (rad/s) 用轮速计算的怎么样？
    
    // 期望轮速计算 (X型全向轮逆运动学) - 单位: mm/s (轮子线速度)
    float expected_lf = (-real_vx - real_vy) / Sqrt(2) + real_wz * L;
    float expected_rf = (real_vx - real_vy) / Sqrt(2) - real_wz * L;
    float expected_lb = (-real_vx + real_vy) / Sqrt(2) - real_wz * L;
    float expected_rb = (real_vx + real_vy) / Sqrt(2) + real_wz * L;
    
    // 实际轮速 (mm/s)
    wheel_slip[WHEEL_LF].actual_speed = wheel_lf;
    wheel_slip[WHEEL_RF].actual_speed = wheel_rf;
    wheel_slip[WHEEL_LB].actual_speed = wheel_lb;
    wheel_slip[WHEEL_RB].actual_speed = wheel_rb;
    
    wheel_slip[WHEEL_LF].expected_speed = expected_lf;
    wheel_slip[WHEEL_RF].expected_speed = expected_rf;
    wheel_slip[WHEEL_LB].expected_speed = expected_lb;
    wheel_slip[WHEEL_RB].expected_speed = expected_rb;
    
    // 单轮滑移率检测与TCS控制
    // 目标: 限制滑移率不超过 TCS_TARGET_SLIP_RATIO (20%)
    uint8_t any_wheel_slipping = 0;
    float wheel_actual[4] = {wheel_lf, wheel_rf, wheel_lb, wheel_rb};
    float wheel_expected[4] = {expected_lf, expected_rf, expected_lb, expected_rb};
    
    for (int i = 0; i < 4; i++) {
        float actual_abs = fabsf(wheel_actual[i]);
        float expected_abs = fabsf(wheel_expected[i]);
        
        // 计算滑移率: slip_ratio = (|actual| - |expected|) / |actual|
        // 正值表示空转(打滑)，负值表示卡住
        float slip_ratio = 0.0f;
        if (actual_abs > 50.0f) {  // 避免低速时除零和噪声
            slip_ratio = (actual_abs - expected_abs) / actual_abs;
        }
        
        // 保存偏差用于调试
        wheel_slip[i].deviation = slip_ratio * 100.0f;  // 转为百分比
        
        // 判断是否需要TCS干预
        // 只在空转(slip_ratio > 0)且超过死区时干预
        if (slip_ratio > TCS_SLIP_DEADBAND) {
            wheel_slip[i].is_slipping = 1;
            any_wheel_slipping = 1;
            
            // 超过目标滑移率时，降低tcs_factor
            if (slip_ratio > TCS_TARGET_SLIP_RATIO) {
                // 超限量越大，降低越多
                float excess = slip_ratio - TCS_TARGET_SLIP_RATIO;
                float reduction = excess * TCS_RESPONSE_GAIN;
                wheel_slip[i].tcs_factor = fmaxf(1.0f - reduction, TCS_MIN_FACTOR);
            } else {
                // 在死区和目标之间，逐渐恢复
                if (wheel_slip[i].tcs_factor < 1.0f) {
                    wheel_slip[i].tcs_factor += TCS_RECOVERY_RATE;
                    if (wheel_slip[i].tcs_factor > 1.0f)
                        wheel_slip[i].tcs_factor = 1.0f;
                }
            }
        } 
        else {
            // 滑移率在死区内或卡住状态，不干预
            wheel_slip[i].is_slipping = 0;
            
            // 逐渐恢复tcs_factor
            if (wheel_slip[i].tcs_factor < 1.0f) {
                wheel_slip[i].tcs_factor += TCS_RECOVERY_RATE;
                if (wheel_slip[i].tcs_factor > 1.0f) 
                    wheel_slip[i].tcs_factor = 1.0f;
            }
        }
    }
    
    // VOFA+调试输出
    VOFA(0, imu_raw_vx, imu_raw_vy, wheel_vx, wheel_vy,
         chassis_feedback_data.real_vx, chassis_feedback_data.real_vy, 
         imu_accel_x, imu_accel_y, wheel_slip[0].tcs_factor, wheel_slip[1].tcs_factor,
         wheel_slip[2].tcs_factor, wheel_slip[3].tcs_factor);
    
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
#else
    chassis_feedback_data.real_vx = wheel_vx;
    chassis_feedback_data.real_vy = wheel_vy;
#endif
}
/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 电机离线报警: 几号电机离线就响几声
    MotorOfflineBeepAlarm();
    
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
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
        chassis_cmd_recv.wz = 0.5*chassis_cmd_recv.offset_angle*abs(chassis_cmd_recv.offset_angle);
        break;
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
    EstimateSpeed();
    
    // 计算各轮目标速度 (送入电机速度环)
    MecanumCalculate();
    
    // 全局观测器计算前馈补偿 (送入电机电流前馈)
    GlobalObserverCalculate();

    // 设置电机参考值
    LimitChassisOutput();

    // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // 我方颜色id小于7是红色,大于7是蓝色, 0:red , 1:blue
    // chassis_feedback_data.self_color = referee_data->GameRobotState.robot_id > 7 ? COLOR_BLUE : COLOR_RED;
    // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;
    // chassis_feedback_data.robot_level = referee_data->GameRobotState.robot_level;
    // chassis_feedback_data.bullet_speed = referee_data->ShootData.bullet_speed;
    
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
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}
