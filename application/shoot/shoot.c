#include "shoot.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "servo_motor.h"
#include "dji_motor_offline_alarm.h" // 电机离线检测

#ifdef USE_LASER_POINTER
/* ======================== 激光笔模式 ======================== */
#include "bsp_gpio.h"

static GPIOInstance *laser_gpio;  // 激光笔GPIO实例
static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data;
/**
 * @brief 激光笔控制函数
 * @param enable 1-打开激光笔, 0-关闭激光笔
 */
void LaserControl(uint8_t enable)
{
    if (enable)
        GPIOSet(laser_gpio);    // 输出高电平,激光笔打开
    else
        GPIOReset(laser_gpio);  // 输出低电平,激光笔关闭
}

void ShootInit()
{
    // 初始化PC8作为激光笔控制引脚
    GPIO_Init_Config_s laser_gpio_config = {
        .GPIOx = GPIOC,
        .GPIO_Pin = GPIO_PIN_8,
        .pin_state = GPIO_PIN_RESET,     // 初始状态为关闭
        .exti_mode = GPIO_EXTI_MODE_NONE, // 不使用中断
        .gpio_model_callback = NULL,
        .id = NULL,
    };
    laser_gpio = GPIORegister(&laser_gpio_config);
    
    // 初始化消息订阅和发布
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    
    // 确保激光笔初始为关闭状态
    LaserControl(0);
}

/* 激光笔控制任务-(云台调试用) */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    
 
    // 云台调试专用: 只根据load_mode判断是否开火
    
    if (shoot_cmd_recv.load_mode == LOAD_BURSTFIRE || 
        shoot_cmd_recv.load_mode == LOAD_1_BULLET ||
        shoot_cmd_recv.load_mode == LOAD_3_BULLET)
    {
        LaserControl(1);  // 打开激光笔
    }
    else
    {
        LaserControl(0);  // 关闭激光笔
    }
    
    // 反馈数据
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

#else
/* ======================== 完整发射机构模式 ======================== */

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l; // 左摩擦轮
static DJIMotorInstance *friction_r; // 右摩擦轮
static DJIMotorInstance *loader;     // 拨盘电机
static ServoInstance *lid_L;         // 需要增加弹舱盖
static ServoInstance *lid_R;         // 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自gimbal_cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自gimbal_cmd的发射控制信息

// 拨弹盘前馈
static float loader_current_forward = 0;

// dwt定时
static float hibernate_time = 0, dead_time = 0, blocked_num = 0, blocked_time = 0;
static float load_angle_set = 0;

// 发射电机离线检测实例
static MotorOfflineAlarmInstance *shoot_offline_alarm = NULL;

void ShootInit()
{
    // 左摩擦轮,两个can2
    Motor_Init_Config_s friction_config_r = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 5, //20
                .Ki = 1, //1
                .Kd = 0,
                .Derivative_LPF_RC = 0.02,
                .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_DerivativeFilter,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
                .DeadBand = 25,
            },
            .current_PID = {
                .Kp = 1.5, //0.7
                .Ki = 0,   //0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508,
    };

    Motor_Init_Config_s friction_config_l = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, //20
                .Ki = 1, //1
                .Kd = 0,
                .Derivative_LPF_RC = 0.02,
                .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_DerivativeFilter,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
                .DeadBand = 25
            },
            .current_PID = {
                .Kp = 1.5, //0.7
                .Ki = 0,   //0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };

        friction_r = DJIMotorInit(&friction_config_r);
        friction_l = DJIMotorInit(&friction_config_l); 
    
    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 4,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 18, //10
                .Ki = 0.8,
                .Kd = 0,
                .MaxOut = 20000,
            },
            .speed_PID = {
                .Kp = 1.0, //0
                .Ki = 0.1, //0
                .Kd = 0.002,
                .Improve = PID_Integral_Limit | PID_ErrorHandle,
                .IntegralLimit = 400,//200
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 1.5, //1.5
                .Ki = 0.0,   //0
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 10000,
            },
            .current_feedforward_ptr = &loader_current_forward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, 
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);
    Servo_Init_Config_s lid_L_config = {
        .htim = &htim1,
        .Channel = TIM_CHANNEL_1,
        //舵机的初始化模式和类型
        .Servo_Angle_Type = Start_mode,
        .Servo_type = Servo270,
    };
    // Servo_Init_Config_s lid_R_config={
    //     .htim=&htim1,
    //     .Channel=TIM_CHANNEL_2,
    //     //舵机的初始化模式和类型
    //     .Servo_Angle_Type=Start_mode,
    //     .Servo_type=Servo180,
    // };
    lid_L = ServoInit(&lid_L_config);
    // lid_R=ServoInit(&lid_R_config);
    Servo_Motor_Type_Select(lid_L, Start_mode);
    //Servo_Motor_Type_Select(lid_R, Start_mode);
    //Servo_Motor_StartSTOP_Angle_Set(lid_L ,5,120);   //步兵二
    Servo_Motor_StartSTOP_Angle_Set(lid_L, 160, 270); //步兵一
    //Servo_Motor_StartSTOP_Angle_Set(lid_R,-5,66);
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    
    // 发射电机离线检测配置 
    MotorOfflineAlarmConfig_t shoot_alarm_cfg = {
        .motors = {loader, friction_l, friction_r},
        .beep_times = {3, 4, 5},        // loader=3声, friction_l=4声, friction_r=5声
        .motor_count = 3,
        .buzzer_freq = ALARM_FREQ_HIGH, // 高音调
        .run_buzzer_task = 0,           // 云台已执行BuzzerTask
    };
    shoot_offline_alarm = MotorOfflineAlarmRegister(&shoot_alarm_cfg);
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 电机离线报警: loader=3声, friction_l=4声, friction_r=5声
    MotorOfflineAlarmTask(shoot_offline_alarm);
    
    // 从 cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        // switch (shoot_cmd_recv.bullet_speed)
        // {
        // case SMALL_AMU_15:
        //     DJIMotorSetRef(friction_l, 26800);
        //     DJIMotorSetRef(friction_r, 26800);
        //     break;
        // case SMALL_AMU_18:
        //     DJIMotorSetRef(friction_l, 30000);
        //     DJIMotorSetRef(friction_r, 30000);
        //     break;
        // case SMALL_AMU_30:
        //     DJIMotorSetRef(friction_l, 46500);
        //     DJIMotorSetRef(friction_r, 46500);
        //     break;
        // default: // 当前为了调试设定4000
        //     DJIMotorSetRef(friction_l, 40500);
        //     DJIMotorSetRef(friction_r, 40000);
        //     break;
        // } 

        DJIMotorSetRef(friction_l, 5000);//40000
        DJIMotorSetRef(friction_r, 5000);
    }
    else // 关闭摩擦轮/
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }
    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;

    if (shoot_cmd_recv.bullet_speed >= shoot_cmd_recv.rest_heat + 10 || shoot_cmd_recv.bullet_speed == 0 || shoot_cmd_recv.rest_heat == 0)
    {
        if (loader->motor_controller.speed_PID.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR)
        {
            blocked_num++;
            if (blocked_num <= BLOCK_TIME_RECORD_COUNT)
            {
                blocked_time = DWT_GetTimeline_ms();
            }
        }
        else
        {
            blocked_num = 0;
        }
        
        if (DWT_GetTimeline_ms() - blocked_time < REVERSE_DURATION_MS)
        {
            shoot_cmd_recv.load_mode = LOAD_REVERSE;
        }
        
        
        switch (shoot_cmd_recv.load_mode)
        {
        case LOAD_STOP:
            DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
            DJIMotorSetRef(loader, 0);
            break;
        // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
        case LOAD_1_BULLET: // 激活能量机关/干扰对方用,英雄用.
            load_angle_set = loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER;
            DJIMotorOuterLoop(loader, ANGLE_LOOP);  // 切换到角度环
            DJIMotorSetRef(loader, load_angle_set); // 控制量增加一发弹丸的角度
            hibernate_time = DWT_GetTimeline_ms();  // 记录触发指令的时间
            dead_time = 500;                        // 完成1发弹丸发射的时间
            break;
        // 三连发
        case LOAD_3_BULLET:
            load_angle_set = loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER * 3;
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorSetRef(loader, load_angle_set);
            hibernate_time = DWT_GetTimeline_ms();
            dead_time = 1800;  // 完成3发弹丸发射的时间 (约600ms/发)
            break;
        // 四连发
        case LOAD_4_BULLET:
            load_angle_set = loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER * 4;
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorSetRef(loader, load_angle_set);
            hibernate_time = DWT_GetTimeline_ms();
            dead_time = 1000;  // 完成4发弹丸发射的时间 (约600ms/发)
            break;
        // 五连发
        case LOAD_5_BULLET:
            load_angle_set = loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER * 5;
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorSetRef(loader, load_angle_set);
            hibernate_time = DWT_GetTimeline_ms();
            dead_time = 3000;  // 完成5发弹丸发射的时间 (约600ms/发)
            break;
        // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
        case LOAD_BURSTFIRE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE);
            // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
            break;
        // 拨盘反转,对速度闭环
        // 也有可能需要从switch-case中独立出来
        case LOAD_REVERSE:
            load_angle_set = loader->measure.total_angle - ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER * REVERSE_ANGLE_RATIO;
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorSetRef(loader, load_angle_set);
            // ...
            break;
        default:
            while (1)
                ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
        }
    }
    else
    {
        DJIMotorOuterLoop(loader, ANGLE_LOOP);
        DJIMotorSetRef(loader, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        Servo_Motor_Type_Select(lid_L, Start_mode);
       // Servo_Motor_Type_Select(lid_R,Final_mode);  //.
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
       Servo_Motor_Type_Select(lid_L, Final_mode);
        // Servo_Motor_Type_Select(lid_R,Start_mode);   //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弈反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

#endif // USE_LASER_POINTER
