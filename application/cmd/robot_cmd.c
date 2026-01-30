// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "vofa.h"
// ======================== 视觉通信协议条件编译 ========================
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
    #include "master_process.h"  // VCP/UART协议 (ROS2上位机)
#elif defined(VISION_USE_SERIALPORT)
    #include "serialport_protocol.h"  // SerialPort协议 (HUST上位机)
#elif defined(VISION_USE_SP)
    #include "master_process.h"  // SP协议 (gimbal_aim_vision上位机)
#endif

#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* gimbal_cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

//#define OUTBREAK_PRIORITY
#define COOLING_PRIORITY

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static float yaw_diff;

// ======================== 视觉数据指针 - 根据协议类型条件编译 ========================
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
    static Vision_Recv_s *vision_recv_data; // VCP/UART协议视觉接收数据
    static Vision_Send_s vision_send_data;  // VCP/UART协议视觉发送数据
#elif defined(VISION_USE_SERIALPORT)
    static SerialPort_Recv_s *vision_recv_data_sp; // SerialPort协议接收数据
    static SerialPort_Send_s vision_send_data_sp;  // SerialPort协议发送数据
#elif defined(VISION_USE_SP)
    static Vision_SP_Recv_s *vision_recv_sp; // SP协议接收数据
#endif

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

static float chassis_speed_buff;
static Work_Mode_e vision_work_mode;


//用于计算角加速度
static int16_t last_rocker_l_ = 0;
static int16_t last_rocker_l1 = 0;

/**
 * @brief 二段灵敏度计算函数
 * @param rocker_value 摇杆值 (死区处理后)
 * @param low_sens 低灵敏度系数
 * @param high_sens 高灵敏度系数
 * @param threshold 灵敏度切换阈值
 * @return 角度调整量
 */
static float TwoStageSensitivity(int16_t rocker_value, float low_sens, float high_sens, int16_t threshold)
{
    if (rocker_value > threshold)
        return low_sens * threshold + high_sens * (rocker_value - threshold);
    else if (rocker_value < -threshold)
        return -low_sens * threshold + high_sens * (rocker_value + threshold);
    else
        return low_sens * rocker_value;
}

/**
 * @brief 视觉模式下摇杆微调 (二段灵敏度)
 * @note 小幅度精细控制，大幅度快速切换目标
 */
static void VisionRockerAdjust(void)
{
    // 死区处理
    int16_t rocker_l_filtered = rc_data[TEMP].rc.rocker_l_;
    int16_t rocker_l1_filtered = rc_data[TEMP].rc.rocker_l1;
    if (rocker_l_filtered <= 10 && rocker_l_filtered >= -10)
        rocker_l_filtered = 0;
    if (rocker_l1_filtered <= 10 && rocker_l1_filtered >= -10)
        rocker_l1_filtered = 0;
    
    // 二段灵敏度参数: 阈值300, 低灵敏度/高灵敏度系数
    gimbal_cmd_send.yaw -= TwoStageSensitivity(rocker_l_filtered, 0.0003f, 0.0012f, 300);
    gimbal_cmd_send.pitch -= TwoStageSensitivity(rocker_l1_filtered, 0.00025f, 0.001f, 300);
}

#ifdef OUTBREAK_PRIORITY
static void Limitshoot()
{
    switch (chassis_fetch_data.bullet_speed)
    {
    case 200:
        shoot_cmd_send.shoot_rate = 6;
        break;
    case 250:
        shoot_cmd_send.shoot_rate = 7;
        break;
    case 300:
        shoot_cmd_send.shoot_rate = 8;
        break;
    case 350:
        shoot_cmd_send.shoot_rate = 9;
        break;    
    default:
        shoot_cmd_send.shoot_rate = 10;
        break;
    }
}
#endif // DEBUG

#ifdef COOLING_PRIORITY
static void Limitshoot()
{
    // switch (chassis_fetch_data.bullet_speed)
    // {
    // case 50:
    //     shoot_cmd_send.shoot_rate = 4;
    //     break;
    // case 85:
    //     shoot_cmd_send.shoot_rate = 5;
    //     break;
    // case 120:
    //     shoot_cmd_send.shoot_rate = 6;
    //     break;
    // case 155:
    //     shoot_cmd_send.shoot_rate = 7;
    //     break;    
    // default:
    //     shoot_cmd_send.shoot_rate = 5;
    //     break;
    // }
    shoot_cmd_send.shoot_rate = 20;
}
#endif // DEBUG

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    
    // ======================== 视觉通信初始化 - 根据协议类型 ========================
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
    vision_recv_data = VisionInit(&huart1); // VCP/UART协议
#elif defined(VISION_USE_SERIALPORT)
    vision_recv_data_sp = SerialPortInit(&huart1); // SerialPort协议
#elif defined(VISION_USE_SP)
    vision_recv_sp = VisionSPInit(&huart1); // SP协议
#endif

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD通信can1
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    chassis_cmd_send.chassis_power_buff = 1;
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],进入小陀螺模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; //全改成云台gyro反馈
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))  // 右侧开关状态[上],进入跟头模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; //全改成云台gyro反馈
    }
    

    // 云台参数,确定云台控制数据


    uint8_t no_target = 0;
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
    no_target = (vision_recv_data->tracking == NO_TARGET);
#elif defined(VISION_USE_SERIALPORT)
    // SerialPort协议没有tracking字段,根据shootStatus或state判断
    no_target = (vision_recv_data_sp->shootStatus == 0);
#elif defined(VISION_USE_SP)
    // SP协议: mode==0表示不控制(无目标或空闲)
    no_target = (vision_recv_sp->mode == 0);
#endif
    if (switch_is_mid(rc_data[TEMP].rc.switch_left) && !no_target) // 左侧开关状态为[中]且有目标,视觉模式
    {   
        // ======================== 根据协议类型获取视觉数据 ========================
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
        // VCP/UART协议
        if (vision_recv_data->aimYaw > 180.0f)
            vision_recv_data->aimYaw -= 360.0f; 
        yaw_diff =(vision_recv_data->aimYaw == 0 ? 0 : vision_recv_data->aimYaw - gimbal_cmd_send.yaw);
        yaw_diff = loop_float_constrain(yaw_diff, -45.0f, 45.0f); 
        gimbal_cmd_send.yaw += yaw_diff;
        gimbal_cmd_send.pitch = vision_recv_data->aimPitch;
        // 视觉模式下遥控器微调 (二段灵敏度, 方便快速更换目标)  
        //VisionRockerAdjust();
#elif defined(VISION_USE_SERIALPORT)
        // SerialPort协议 - 需要转换整数到浮点数
        // 上位机发送的是放大后的整数值,需要根据实际编码规则转换
        // 假设: yaw和pitch都是角度*100的整数形式
        if (vision_recv_data_sp->yaw != 0 || vision_recv_data_sp->pitch != 0) {
            gimbal_cmd_send.yaw = (float)vision_recv_data_sp->yaw / 100.0f;
            gimbal_cmd_send.pitch = (float)vision_recv_data_sp->pitch / 100.0f;
        }
        
        // 视觉模式下遥控器微调 (二段灵敏度, 方便快速更换目标)
        if (vision_recv_data_sp->shootStatus != 0) {
            VisionRockerAdjust();
        }
#elif defined(VISION_USE_SP)
        // SP协议, 视觉返回目标角+前馈
        if (vision_recv_sp->mode != 0) {
            gimbal_cmd_send.yaw = vision_recv_sp->yaw;
            gimbal_cmd_send.pitch = vision_recv_sp->pitch;
            gimbal_cmd_send.yaw_speed_feedforward = vision_recv_sp->yaw_vel;
            gimbal_cmd_send.pitch_speed_feedforward = vision_recv_sp->pitch_vel;
            gimbal_cmd_send.yaw_acc_feedforward = vision_recv_sp->yaw_acc;
            gimbal_cmd_send.pitch_acc_feedforward = vision_recv_sp->pitch_acc;
            
            // 二段灵敏度微调
            VisionRockerAdjust();
        }
#endif
        
        // 软件限位 - 到达限位时清零对应方向的前馈，防止越界
        if (gimbal_cmd_send.pitch <= PITCH_MIN_LIMIT)
         {
             gimbal_cmd_send.pitch = PITCH_MIN_LIMIT;
             // 清零向下的前馈（负方向）
             if (gimbal_cmd_send.pitch_speed_feedforward < 0)
                 gimbal_cmd_send.pitch_speed_feedforward = 0;
             if (gimbal_cmd_send.pitch_acc_feedforward < 0)
                 gimbal_cmd_send.pitch_acc_feedforward = 0;
         }
         if (gimbal_cmd_send.pitch >= PITCH_MAX_LIMIT)
         {
             gimbal_cmd_send.pitch = PITCH_MAX_LIMIT;
             // 清零向上的前馈（正方向）
             if (gimbal_cmd_send.pitch_speed_feedforward > 0)
                 gimbal_cmd_send.pitch_speed_feedforward = 0;
             if (gimbal_cmd_send.pitch_acc_feedforward > 0)
                 gimbal_cmd_send.pitch_acc_feedforward = 0;
         }
        //  if (gimbal_cmd_send.yaw >= 30)
        //  {
        //      gimbal_cmd_send.yaw = 30;
        //  }
        //  if (gimbal_cmd_send.yaw <= -30)
        //  {
        //      gimbal_cmd_send.yaw = -30;
        //  }
        
#if !defined(VISION_USE_SP)
        // 非SP协议清零前馈
        gimbal_cmd_send.yaw_speed_feedforward = 0;
        gimbal_cmd_send.pitch_speed_feedforward = 0;
        gimbal_cmd_send.yaw_acc_feedforward = 0;
        gimbal_cmd_send.pitch_acc_feedforward = 0;
#endif
         
        chassis_cmd_send.aim_mode = AIM_ON;
    }
    else
    {
        chassis_cmd_send.aim_mode = AIM_OFF;
    }

    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    // ======================== 根据协议判断是否有目标 ========================
    
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || no_target)
    { // 按照摇杆的输出大小进行角度增量,增益系数需调整
        if (rc_data[TEMP].rc.rocker_l_ <=10 && rc_data[TEMP].rc.rocker_l_ >=-10)
        {
            rc_data[TEMP].rc.rocker_l_ =0;
        }
        else if (rc_data[TEMP].rc.rocker_l1 <=10 && rc_data[TEMP].rc.rocker_l1 >=-10)
        {
            rc_data[TEMP].rc.rocker_l1 =0;
        }
        gimbal_cmd_send.yaw -= 0.002f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch -= 0.0015f * (float)rc_data[TEMP].rc.rocker_l1;
        
        // 速度前馈
        gimbal_cmd_send.yaw_speed_feedforward = -RC_YAW_SPEED_COEF * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch_speed_feedforward = -RC_PITCH_SPEED_COEF * (float)rc_data[TEMP].rc.rocker_l1;
        
        // 加速度前馈
        gimbal_cmd_send.yaw_acc_feedforward = -RC_YAW_ACC_COEF * (float)(rc_data[TEMP].rc.rocker_l_ - last_rocker_l_);
        gimbal_cmd_send.pitch_acc_feedforward = -RC_PITCH_ACC_COEF * (float)(rc_data[TEMP].rc.rocker_l1 - last_rocker_l1);
        
        // 更新历史摇杆值
        last_rocker_l_ = rc_data[TEMP].rc.rocker_l_;
        last_rocker_l1 = rc_data[TEMP].rc.rocker_l1;
        
        // 摇杆控制的软件限位 - 到达限位时清零对应方向的前馈，防止越界
         if (gimbal_cmd_send.pitch <= PITCH_MIN_LIMIT)
         {
             gimbal_cmd_send.pitch = PITCH_MIN_LIMIT;
             if (gimbal_cmd_send.pitch_speed_feedforward < 0)
                 gimbal_cmd_send.pitch_speed_feedforward = 0;
             if (gimbal_cmd_send.pitch_acc_feedforward < 0)
                 gimbal_cmd_send.pitch_acc_feedforward = 0;
         }
         if (gimbal_cmd_send.pitch >= PITCH_MAX_LIMIT)
         {
             gimbal_cmd_send.pitch = PITCH_MAX_LIMIT;
             if (gimbal_cmd_send.pitch_speed_feedforward > 0)
                 gimbal_cmd_send.pitch_speed_feedforward = 0;
             if (gimbal_cmd_send.pitch_acc_feedforward > 0)
                 gimbal_cmd_send.pitch_acc_feedforward = 0;
         }
        //  if (gimbal_cmd_send.yaw >= 30)
        //  {
        //      gimbal_cmd_send.yaw = 30;
        //  }
        //  if (gimbal_cmd_send.yaw <= -30)
        //  {
        //      gimbal_cmd_send.yaw = -30;
        //  }
    }

    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    // 单位: degree/s (电机转子角速度), 摇杆范围±660, 系数×660=满杆转速
    // M3508最高约54000 degree/s, 45×660≈29700, 留有余量
    chassis_cmd_send.vx = 45.0f * (float)rc_data[TEMP].rc.rocker_r_; // degree/s
    chassis_cmd_send.vy = 45.0f * (float)rc_data[TEMP].rc.rocker_r1; // degree/s
    chassis_cmd_send.dash_mode = DASH_OFF;
    //chassis_cmd_send.wz=5.0f*(float)rc_data[TEMP].rc.rocker_l_;
    
    
    // 发射参数
    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (rc_data[TEMP].rc.dial < -100 ) // 向上超过100,打开摩擦轮
        shoot_cmd_send.friction_mode = FRICTION_ON;
    else
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (rc_data[TEMP].rc.dial < -300)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else if (rc_data[TEMP].rc.dial > 300)
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
    // 视觉模式时根据上位机fire/shootStatus字段控制发射


    if (switch_is_mid(rc_data[TEMP].rc.switch_left))
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;  // 摩擦轮常开
         if (rc_data[TEMP].rc.dial < -300)
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        else if (rc_data[TEMP].rc.dial > 300)
                shoot_cmd_send.load_mode = LOAD_1_BULLET;
        else
                shoot_cmd_send.load_mode = LOAD_STOP;
        // ======================== 根据协议类型获取视觉发射指令 ========================
        // 只有跟踪和开火标志位都为1时才开启摩擦轮，并允许通过拨轮控制拨弹盘
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
        // // VCP/UART协议: 跟踪(tracking==1)且开火(fire==1)时才允许开火
        // if (vision_recv_data->tracking == 1 && vision_recv_data->fire == 1)
        // {
        //     shoot_cmd_send.friction_mode = FRICTION_ON;  // 摩擦轮常开
        //     // 通过拨轮控制拨弹盘: 拨轮向上(<-300)连发, 向下(>300)单发
        //     if (rc_data[TEMP].rc.dial < -300)
        //         shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        //     else if (rc_data[TEMP].rc.dial > 300)
        //         shoot_cmd_send.load_mode = LOAD_1_BULLET;
        //     else
        //         shoot_cmd_send.load_mode = LOAD_STOP;
        // }
        // else
        // {
        //     shoot_cmd_send.friction_mode = FRICTION_OFF;
        //     shoot_cmd_send.load_mode = LOAD_STOP;
        // }
#elif defined(VISION_USE_SERIALPORT)
        // SerialPort协议: 使用shootStatus字段 (非0=跟踪且开火, 0=停火)
        if (vision_recv_data_sp->shootStatus != 0)
        {
            shoot_cmd_send.friction_mode = FRICTION_ON;
            if (rc_data[TEMP].rc.dial < -300)
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            else if (rc_data[TEMP].rc.dial > 300)
                shoot_cmd_send.load_mode = LOAD_1_BULLET;
            else
                shoot_cmd_send.load_mode = LOAD_STOP;
        }
        else
        {
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
#elif defined(VISION_USE_SP)
        // SP协议: mode==2 表示控制云台且开火
        if (vision_recv_sp->mode == 2)
        {
            shoot_cmd_send.friction_mode = FRICTION_ON;
            if (rc_data[TEMP].rc.dial < -300)
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            else if (rc_data[TEMP].rc.dial > 300)
                shoot_cmd_send.load_mode = LOAD_1_BULLET;
            else
                shoot_cmd_send.load_mode = LOAD_STOP;
        }
        else
        {
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
#endif
    }
     if (rc_data[TEMP].rc.dial > 1000)
        shoot_cmd_send.lid_mode = LID_OPEN;
    else
       shoot_cmd_send.lid_mode = LID_CLOSE;
    Limitshoot();
}

static float time, dead_time;

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    /**
    if(chassis_fetch_data.robot_level>=0 )
    {
        shoot_cmd_send.shoot_rate = 8;
    }
    else{
        shoot_cmd_send.shoot_rate = 5;
    }
    **/
    Limitshoot();
    // 单位: degree/s (电机转子角速度)
    // 30000 degree/s ≈ 2500 mm/s 轮速, M3508最高约54000 degree/s
    chassis_speed_buff = 30000;
    chassis_cmd_send.chassis_power_buff = 1;
    gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].w * chassis_speed_buff - rc_data[TEMP].key[KEY_PRESS].s * chassis_speed_buff; // degree/s
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].a * chassis_speed_buff - rc_data[TEMP].key[KEY_PRESS].d * chassis_speed_buff; // degree/s
    switch (rc_data[TEMP].key[KEY_PRESS].x) // X键刷新UI
    {
    case 1:
        chassis_cmd_send.ui_mode = UI_REFRESH;
        break;
    default:    
        chassis_cmd_send.ui_mode = UI_KEEP;
        break;
    }

    switch (rc_data[TEMP].mouse.press_r) // 鼠标右键开启自瞄
    {
    case 0:
        gimbal_cmd_send.yaw -= (float)rc_data[TEMP].mouse.x / 660 * 16; // 系数待测
        gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 16;
        if (gimbal_cmd_send.pitch <= PITCH_MIN_LIMIT)
        {
             gimbal_cmd_send.pitch = PITCH_MIN_LIMIT;
        }
        else if (gimbal_cmd_send.pitch >= PITCH_MAX_LIMIT)
        {
             gimbal_cmd_send.pitch = PITCH_MAX_LIMIT;
        }
        chassis_cmd_send.aim_mode = AIM_OFF;
        break;
        

    default:
        // ======================== 根据协议类型获取视觉数据 ========================
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
        // VCP/UART协议
        gimbal_cmd_send.yaw += loop_float_constrain(vision_recv_data->aimYaw - gimbal_cmd_send.yaw, -45, 45);
        gimbal_cmd_send.pitch = (vision_recv_data->aimPitch == 0 ? gimbal_cmd_send.pitch : vision_recv_data->aimPitch);
#elif defined(VISION_USE_SERIALPORT)
        // SerialPort协议 - 整数转浮点数
        if (vision_recv_data_sp->yaw != 0 || vision_recv_data_sp->pitch != 0) {
            float aim_yaw = (float)vision_recv_data_sp->yaw / 100.0f;
            float aim_pitch = (float)vision_recv_data_sp->pitch / 100.0f;
            gimbal_cmd_send.yaw += loop_float_constrain(aim_yaw - gimbal_cmd_send.yaw, -45, 45);
            gimbal_cmd_send.pitch = aim_pitch;
        }
#endif
        if (gimbal_cmd_send.pitch <= PITCH_MIN_LIMIT)
        {
             gimbal_cmd_send.pitch = PITCH_MIN_LIMIT;
        }
        else if (gimbal_cmd_send.pitch >= PITCH_MAX_LIMIT)
        {
             gimbal_cmd_send.pitch = PITCH_MAX_LIMIT;
        }
        chassis_cmd_send.aim_mode = AIM_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2) // E键设置发射模式 
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data[TEMP].mouse.press_l) // 鼠标左键射击
    {
    case 0:
         shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    default:
        break;
    }
    
    // ======================== 视觉自瞄模式下的自动开火 ========================
    // 当鼠标右键开启自瞄,且视觉发送开火指令时,覆盖鼠标左键的控制
    if (rc_data[TEMP].mouse.press_r) 
    {
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
        // VCP/UART协议: 使用fire字段
        if (vision_recv_data->fire == 1)
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        else if (!rc_data[TEMP].mouse.press_l) // 如果视觉不开火且鼠标左键也没按,则停止
            shoot_cmd_send.load_mode = LOAD_STOP;
#elif defined(VISION_USE_SERIALPORT)
        // SerialPort协议: 使用shootStatus字段
        if (vision_recv_data_sp->shootStatus != 0)
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        else if (!rc_data[TEMP].mouse.press_l)
            shoot_cmd_send.load_mode = LOAD_STOP;
#endif
    }
    
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    /**
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] ) // z键退弹
    {
    case 0:
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_REVERSE;
        time = DWT_GetTimeline_ms();  // 记录触发指令的时间
        dead_time = 500; 
        while(time + dead_time > DWT_GetTimeline_ms())
        ;
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    }
    **/
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q] % 2) // Q键设置底盘运动模式
    {
    case 0:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    default:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 按shift使用超级电容
    {
    case 1:
        chassis_cmd_send.dash_mode = DASH_ON;
        chassis_cmd_send.super_cap = CAP_ON;
        break;

    default:
        chassis_cmd_send.dash_mode = DASH_OFF;
        chassis_cmd_send.super_cap = CAP_OFF;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 3) // G键切换视觉击打目标
    {
    case 0:
        vision_work_mode = VISION_MODE_AIM;
        break;
    case 1:
        vision_work_mode = VISION_MODE_SMALL_BUFF;
        break;
    default:
        vision_work_mode = VISION_MODE_BIG_BUFF;
        break;
    }
    if (shoot_cmd_send.friction_mode == FRICTION_OFF && shoot_cmd_send.load_mode != LOAD_REVERSE)
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{

    if (switch_is_down(rc_data[TEMP].rc.switch_left) || switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下/中],遥控器控制
    {
        if (robot_state == ROBOT_STOP || !RemoteControlIsOnline()) // 还需添加重要应用和模块离线的判断
        {
            robot_state = ROBOT_STOP;
            gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.load_mode = LOAD_STOP;

            gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle; // 急停时设定值保持与实际值同步，避免恢复时疯转
            gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        }
        // 右拨杆为[中],恢复正常运行
        if (switch_is_mid(rc_data[TEMP].rc.switch_right) && RemoteControlIsOnline())
        {
            robot_state = ROBOT_READY;
            shoot_cmd_send.shoot_mode = SHOOT_ON;
        }
    }
    // else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[中],视觉模式
    // {
    //     // 视觉模式下允许发射(激光笔需要)
    //     if (robot_state == ROBOT_STOP || rc_data[TEMP].rc.dial >= 640 || !RemoteControlIsOnline())
    //     {
    //         robot_state = ROBOT_STOP;
    //         shoot_cmd_send.shoot_mode = SHOOT_OFF;
    //     }
    //     else
    //     {
    //         robot_state = ROBOT_READY;
    //         shoot_cmd_send.shoot_mode = SHOOT_ON;  // 视觉模式下允许发射
    //     }
    // }
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
    {
        if (rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL][Key_C] % 2 == 0 || !RemoteControlIsOnline()) // ctrl+c 进入急停
        {
            robot_state = ROBOT_READY;
            shoot_cmd_send.shoot_mode = SHOOT_ON;
        }
        else
        {
            robot_state = ROBOT_STOP;
            gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.load_mode = LOAD_STOP;

            gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle; // 急停时设定值保持与实际值同步，避免恢复时疯转
            gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        }
    }
    
}
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{

    chassis_cmd_send.ui_mode = UI_KEEP;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    // chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
        RemoteControlSet();
        VOFA(vision_recv_data->aimPitch,yaw_diff,gimbal_cmd_send.pitch
            ,gimbal_fetch_data.gimbal_imu_data.Pitch,gimbal_fetch_data.gimbal_imu_data.Yaw); 
    if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        MouseKeySet();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    shoot_cmd_send.bullet_speed = chassis_fetch_data.bullet_speed;
    
    // ======================== 视觉发送数据准备 - 根据协议类型 ========================
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
    // VCP/UART协议 - 设置标志位和姿态角度
    VisionSetFlag(chassis_fetch_data.self_color, vision_work_mode, chassis_fetch_data.bullet_speed);
    VisionSetAltitude(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle, 
                      gimbal_fetch_data.gimbal_imu_data.Pitch, 
                      gimbal_fetch_data.gimbal_imu_data.Roll);
#elif defined(VISION_USE_SERIALPORT)
    // SerialPort协议 - 准备发送数据
    vision_send_data_sp.startflag = '!';
    vision_send_data_sp.flag = 0x01;  // 状态标志
    
    // 转换角度为整数(放大100倍)
    vision_send_data_sp.yaw = (int32_t)(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle * 100.0f);
    vision_send_data_sp.pitch = (int16_t)(gimbal_fetch_data.gimbal_imu_data.Pitch * 100.0f);
    vision_send_data_sp.roll = (int16_t)(gimbal_fetch_data.gimbal_imu_data.Roll * 100.0f);
    
    // 颜色: 0=蓝色, 1=红色
    vision_send_data_sp.color = chassis_fetch_data.self_color;
    
    // 时间戳
    vision_send_data_sp.time_stamp = DWT_GetTimeline_ms();
    
    // 右击状态 (鼠标右键或其他准备状态)
    vision_send_data_sp.right_clicked = (rc_data[TEMP].mouse.press_r ? 1 : 0);
    
    // 用户时间偏差/弹速
    vision_send_data_sp.user_time_bias = (int8_t)chassis_fetch_data.bullet_speed;
#elif defined(VISION_USE_SP)
    // SP协议发送数据
    {
        uint8_t sp_mode = (uint8_t)(vision_work_mode + 1);
        if (chassis_cmd_send.aim_mode == AIM_OFF) sp_mode = 0;
        
        // 欧拉角转四元数
        float roll = gimbal_fetch_data.gimbal_imu_data.Roll * DEGREE_2_RAD;
        float pitch = gimbal_fetch_data.gimbal_imu_data.Pitch * DEGREE_2_RAD;
        float yaw = gimbal_fetch_data.gimbal_imu_data.Yaw * DEGREE_2_RAD;
        
        float cy = arm_cos_f32(yaw * 0.5f);
        float sy = arm_sin_f32(yaw * 0.5f);
        float cp = arm_cos_f32(pitch * 0.5f);
        float sp = arm_sin_f32(pitch * 0.5f);
        float cr = arm_cos_f32(roll * 0.5f);
        float sr = arm_sin_f32(roll * 0.5f);
        
        float q[4] = {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
        
        float yaw_vel = gimbal_fetch_data.gimbal_imu_data.Gyro[2] * RAD_2_DEGREE;
        float pitch_vel = gimbal_fetch_data.gimbal_imu_data.Gyro[1] * RAD_2_DEGREE;
        static uint16_t bullet_count = 0;
        
        VisionSPSetState(sp_mode, q, 
                         gimbal_fetch_data.gimbal_imu_data.YawTotalAngle, yaw_vel,
                         gimbal_fetch_data.gimbal_imu_data.Pitch, pitch_vel,
                         (float)chassis_fetch_data.bullet_speed, bullet_count);
    }0, 
#endif
    
    chassis_cmd_send.friction_mode = shoot_cmd_send.friction_mode;
    chassis_cmd_send.gimbal_mode = gimbal_cmd_send.gimbal_mode;
    chassis_cmd_send.lid_mode = shoot_cmd_send.lid_mode;
    chassis_cmd_send.load_mode = shoot_cmd_send.load_mode;
    chassis_cmd_send.pitch_angle = ((int)(gimbal_fetch_data.gimbal_imu_data.Pitch * 10 + 0.5)) / 10.0;

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    
    // 发送视觉数据
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
    // VisionSend(&vision_send_data);
#elif defined(VISION_USE_SERIALPORT)
    static uint32_t serialport_send_counter = 0;
    if (++serialport_send_counter % 5 == 0)
        SerialPortSend(&vision_send_data_sp);
#elif defined(VISION_USE_SP)
    static uint32_t sp_send_counter = 0;
    if (++sp_send_counter % 2 == 0)  // 100Hz
        VisionSPSend();
#endif
}
