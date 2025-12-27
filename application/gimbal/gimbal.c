#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_vofa.h" // VOFA+数据发送模块
#include "bmi088.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static Chassis_Upload_Data_s chassis_real_speed;
static Subscriber_t *chassis_speed_sub; // 底盘反馈信息订阅者

// ======================== 速度前馈 ========================
static float yaw_speed_feedforward = 0.0f;    // yaw速度前馈 (来自cmd)
static float pitch_speed_feedforward = 0.0f;  // pitch速度前馈 (来自cmd)

// ======================== 电流前馈 (加速度前馈 + 重力补偿) ========================
static float yaw_current_feedforward = 0.0f;    // yaw电流前馈
static float pitch_current_feedforward = 0.0f;  // pitch电流前馈
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
                .MaxOut = 20000,
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
        .motor_type = GM6020};
        
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp =0.45,//0.45
                .Ki = 0.4,//0.5
                .Kd = 0.045,//0.005
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
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
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
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
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

    //更新速度前馈
    yaw_speed_feedforward = gimbal_cmd_recv.yaw_speed_feedforward;
    pitch_speed_feedforward = gimbal_cmd_recv.pitch_speed_feedforward;
    //更新电流前馈
    yaw_current_feedforward = YAW_ACC_TO_CURRENT * gimbal_cmd_recv.yaw_acc_feedforward;// 将角加速度转换为电流前馈

    float pitch_acc_current = PITCH_ACC_TO_CURRENT * gimbal_cmd_recv.pitch_acc_feedforward;// Pitch电流前馈 = 加速度前馈 + 重力补偿
    
    // ======================== Pitch重力补偿计算 ========================
    // 使用IMU的Pitch角度计算重力补偿: feedforward = MAX * cos(pitch_angle)
    float pitch_angle_rad = (gimba_IMU_data->Pitch - PITCH_HORIZONTAL_ANGLE) * DEGREE_2_RAD;
    float pitch_gravity_comp = GRAVITY_COMP_MAX * arm_cos_f32(pitch_angle_rad);
    // 合并加速度前馈和重力补偿
    pitch_current_feedforward = pitch_acc_current + pitch_gravity_comp;
    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
