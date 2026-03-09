#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "vofa.h" // VOFA+数据发送模块
#include "bmi088.h"
#include "bsp_dwt.h"  // 用于获取精确时间间隔
#include "motor_offline_alarm.h" // 电机离线检测

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static Chassis_Upload_Data_s chassis_real_speed;
static Subscriber_t *chassis_speed_sub; // 底盘反馈信息订阅者

static float yaw_speed_feedforward = 0.0f;    // yaw速度前馈 (本地微分计算)
static float pitch_speed_feedforward = 0.0f;  // pitch速度前馈 (本地微分计算)

static float yaw_current_feedforward = 0.0f;    // yaw电流前馈
static float pitch_current_feedforward = 0.0f;  // pitch电流前馈

static float yaw_ref_prev = 0.0f;           // 上一次yaw目标角度
static float pitch_ref_prev = 0.0f;         // 上一次pitch目标角度
static float yaw_speed_raw_prev = 0.0f;     // 上一次yaw速度 (用于计算加速度)
static float pitch_speed_raw_prev = 0.0f;   // 上一次pitch速度 (用于计算加速度)
static float yaw_speed_filtered = 0.0f;     // 滤波后的yaw速度
static float pitch_speed_filtered = 0.0f;   // 滤波后的pitch速度
static float yaw_acc_filtered = 0.0f;       // 滤波后的yaw加速度
static float pitch_acc_filtered = 0.0f;     // 滤波后的pitch加速度
static uint32_t last_feedforward_time = 0;  // 上次计算时间戳(us)
static uint8_t feedforward_initialized = 0; // 初始化标志

// 前馈计算参数
#define FEEDFORWARD_SPEED_LPF_RC    0.02f   // 速度前馈低通滤波时间常数(s)
#define FEEDFORWARD_ACC_LPF_RC      0.03f   // 加速度前馈低通滤波时间常数(s)
#define FEEDFORWARD_JUMP_THRESHOLD  50.0f   // 角度跳变阈值(°), 超过则视为模式切换
#define FEEDFORWARD_RAMP_TIME       0.2f    // 跳变后渐变恢复时间(s)
static float feedforward_ramp_factor = 1.0f; // 渐变系数 (0~1)

// 云台电机离线检测实例
static MotorOfflineAlarmInstance *gimbal_offline_alarm = NULL;

void GimbalInit()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源,
    
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.55, //0.55
                .Ki = 0.04,//0.05
                .Kd = 0.035,//0.02
                .CoefA =0.5,//0.5
                .CoefB = 0.6,//0.6
                .Output_LPF_RC = 0,
                .DeadBand = 0,//0.02
                .Derivative_LPF_RC=0.008,//0.008
                .Improve = PID_Trapezoid_Intergral |PID_ChangingIntegrationRate| PID_Integral_Limit |PID_Derivative_On_Measurement |  PID_OutputFilter |PID_DerivativeFilter,
                .IntegralLimit = 0.5,

                .MaxOut = 400,
            },
            .speed_PID = {
                .Kp = 3300,//1300
                .Ki = 0,//
                .Kd =0.005,//0.01
                 .CoefA = 0.8,
                 .CoefB = 0.1,
                .Output_LPF_RC = 0.002,//0.002
                .Improve = PID_Trapezoid_Intergral |PID_Integral_Limit |PID_Derivative_On_Measurement |  PID_OutputFilter,
                .IntegralLimit = 5000,
                .MaxOut = 16000,//16000
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr=&gimba_IMU_data->Gyro[2],
            .speed_feedforward_ptr = &yaw_speed_feedforward,
            .current_feedforward_ptr = &yaw_current_feedforward,  // 电流前馈 (加速度前馈)
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,  
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD,  // 启用速度+电流前馈
        },
        .motor_type = GM6020_CURRENT};
        
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp =0.45,//0.45
                .Ki = 0.4,//0.4
                .Kd = 0.045,//0.045
                .CoefA = 0.7,//0.7
                .CoefB = 0.6,//0.6
                .DeadBand = 0,//0
                .Output_LPF_RC = 0.01,//0.01
                .Improve = PID_Trapezoid_Intergral |PID_ChangingIntegrationRate | PID_Integral_Limit| PID_OutputFilter|PID_DerivativeFilter,
                .IntegralLimit =5,//5
                .MaxOut = 600,//600
            },
            .speed_PID = {
                .Kp=3500,//3500
                .Ki =0,//0
                .Kd =0.0005,//0.0005
                .CoefA =1500,//1500
                .CoefB =2000,//2000
                .Output_LPF_RC = 0.005,//0.005
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit  | PID_OutputFilter,
                .IntegralLimit =3000,//3000
                .MaxOut = 20000,//20000
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[1]),
            // pitch前馈指针
            .speed_feedforward_ptr = &pitch_speed_feedforward,      // 速度前馈 (来自摇杆)
            .current_feedforward_ptr = &pitch_current_feedforward,  // 电流前馈 (加速度前馈+重力补偿)
        },
            .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,// OTHER_FEED
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,  
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag = SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD, 
        },
        .motor_type = GM6020,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    chassis_speed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    
    // 云台电机离线检测配置
    MotorOfflineAlarmConfig_t gimbal_alarm_cfg = {
        .motors = {yaw_motor, pitch_motor},
        .is_online = DJIMotorIsOnline,
        .beep_times = {1, 2},       // yaw=1声, pitch=2声
        .motor_count = 2,
        .buzzer_freq = ALARM_FREQ_HIGH,
        .run_buzzer_task = 1,       // 主模块执行BuzzerTask
    };
    gimbal_offline_alarm = MotorOfflineAlarmRegister(&gimbal_alarm_cfg);
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 电机离线报警: yaw=1声, pitch=2声
    MotorOfflineAlarmTask(gimbal_offline_alarm);
    
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    SubGetMessage(chassis_speed_sub, &chassis_real_speed);
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
       // DJIMotorSetFeedfoward(yaw_motor,SPEED_FEEDFORWARD);
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorSetFeedfoward(yaw_motor,FEEDFORWARD_NONE);
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }

    // ======================== 前馈计算 (微分目标角度) ========================
    uint32_t current_time = DWT_GetTimeline_us();
    float dt = (current_time - last_feedforward_time) * 1e-6f;  // 转换为秒
    
    // 防止dt异常 (首次运行或时间溢出)
    if (dt <= 0 || dt > 0.1f || !feedforward_initialized) {
        // 初始化历史值
        yaw_ref_prev = gimbal_cmd_recv.yaw;
        pitch_ref_prev = gimbal_cmd_recv.pitch;
        yaw_speed_raw_prev = 0;
        pitch_speed_raw_prev = 0;
        yaw_speed_filtered = 0;
        pitch_speed_filtered = 0;
        yaw_acc_filtered = 0;
        pitch_acc_filtered = 0;
        last_feedforward_time = current_time;
        feedforward_initialized = 1;
        feedforward_ramp_factor = 1.0f;
    } else {
        // 计算角度变化量
        float yaw_delta = gimbal_cmd_recv.yaw - yaw_ref_prev;
        float pitch_delta = gimbal_cmd_recv.pitch - pitch_ref_prev;
        
        // 跳变检测: 角度突变时抑制前馈输出
        if (fabsf(yaw_delta) > FEEDFORWARD_JUMP_THRESHOLD || 
            fabsf(pitch_delta) > FEEDFORWARD_JUMP_THRESHOLD) {
            // 检测到跳变, 重置前馈并开始渐变恢复
            feedforward_ramp_factor = 0.0f;
            yaw_speed_filtered = 0;
            pitch_speed_filtered = 0;
            yaw_acc_filtered = 0;
            pitch_acc_filtered = 0;
        } else {
            // 计算原始速度 (一阶导数)
            float yaw_speed_raw = yaw_delta / dt;
            float pitch_speed_raw = pitch_delta / dt;
            
            // 计算原始加速度 (二阶导数)
            float yaw_acc_raw = (yaw_speed_raw - yaw_speed_raw_prev) / dt;
            float pitch_acc_raw = (pitch_speed_raw - pitch_speed_raw_prev) / dt;
            
            // 低通滤波 (一阶RC滤波器)
            float speed_alpha = dt / (FEEDFORWARD_SPEED_LPF_RC + dt);
            float acc_alpha = dt / (FEEDFORWARD_ACC_LPF_RC + dt);
            
            yaw_speed_filtered = yaw_speed_filtered + speed_alpha * (yaw_speed_raw - yaw_speed_filtered);
            pitch_speed_filtered = pitch_speed_filtered + speed_alpha * (pitch_speed_raw - pitch_speed_filtered);
            yaw_acc_filtered = yaw_acc_filtered + acc_alpha * (yaw_acc_raw - yaw_acc_filtered);
            pitch_acc_filtered = pitch_acc_filtered + acc_alpha * (pitch_acc_raw - pitch_acc_filtered);
            
            // 保存当前速度用于下次加速度计算
            yaw_speed_raw_prev = yaw_speed_raw;
            pitch_speed_raw_prev = pitch_speed_raw;
        }
        
        // 渐变恢复
        if (feedforward_ramp_factor < 1.0f) {
            feedforward_ramp_factor += dt / FEEDFORWARD_RAMP_TIME;
            if (feedforward_ramp_factor > 1.0f) feedforward_ramp_factor = 1.0f;
        }
        
        // 更新历史值
        yaw_ref_prev = gimbal_cmd_recv.yaw;
        pitch_ref_prev = gimbal_cmd_recv.pitch;
        last_feedforward_time = current_time;
    }
    
    // 应用渐变系数和前馈系数到前馈输出
    // 注意: yaw_speed_filtered 单位是 °/s, 但 Gyro 反馈是 rad/s, 需要转换
    yaw_speed_feedforward = yaw_speed_filtered * feedforward_ramp_factor * YAW_SPEED_FF_COEF * DEGREE_2_RAD;
    pitch_speed_feedforward = pitch_speed_filtered * feedforward_ramp_factor * PITCH_SPEED_FF_COEF * DEGREE_2_RAD;
    
    // 电流前馈: 加速度(°/s²) → 力矩(N·m) → CAN原始值
    // τ_acc = J * α, α需要从°/s²转rad/s², 再通过NM_TO_GM6020_RAW转为CAN值
    float yaw_acc_torque = YAW_INERTIA * yaw_acc_filtered * DEGREE_2_RAD;  // N·m
    float yaw_acc_current = yaw_acc_torque * NM_TO_GM6020_RAW * feedforward_ramp_factor;
    float pitch_acc_torque = PITCH_INERTIA * pitch_acc_filtered * DEGREE_2_RAD;  // N·m
    float pitch_acc_current = pitch_acc_torque * NM_TO_GM6020_RAW * feedforward_ramp_factor;
    // ======================== 小陀螺自稳前馈 (三参数模型) ========================
    // τ = b*ω + τ_f*sign(ω)
    // 所有参数均为物理量 (N·m), 最后统一乘NM_TO_GM6020_RAW转为CAN原始值
    float chassis_wz_rad = chassis_real_speed.chassis_imu_data.Gyro[2];  // rad/s
    float chassis_wz_dps = chassis_wz_rad * RAD_2_DEGREE;               // °/s (用于SmoothSign)
    float smooth_sign_val = SmoothSign(chassis_wz_dps, YAW_COULOMB_DEADZONE);
    float viscous_torque = YAW_VISCOUS_DAMPING * chassis_wz_rad;  // N·m
    float friction_torque = YAW_FRICTION_TORQUE * smooth_sign_val;  // N·m
    float yaw_total_torque = viscous_torque + friction_torque;  // N·m
    yaw_current_feedforward = yaw_acc_current + yaw_total_torque * NM_TO_GM6020_RAW;
    
    // 使用IMU的Pitch角度计算重力补偿: feedforward = MAX * cos(pitch_angle)
    float pitch_angle_rad = (gimba_IMU_data->Pitch - PITCH_HORIZONTAL_ANGLE) * DEGREE_2_RAD;
    float pitch_gravity_comp = GRAVITY_COMP_MAX * arm_cos_f32(pitch_angle_rad);
    // 合并加速度前馈和重力补偿
    pitch_current_feedforward = pitch_acc_current + pitch_gravity_comp;
    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    
    // CH0:yaw角度  CH1:yaw角速度(rad/s)  CH2:底盘wz(°/s)  CH3:yaw电流前馈  CH4:实际电流
    VOFA(0, gimba_IMU_data->Yaw,gimba_IMU_data->Gyro[2],(float)yaw_motor->measure.real_current);
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
