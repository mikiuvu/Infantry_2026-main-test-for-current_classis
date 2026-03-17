#include "shoot.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "servo_motor.h"
#include "motor_offline_alarm.h" // 电机离线检测
#include "vofa.h"
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

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自gimbal_cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自gimbal_cmd的发射控制信息

// 摩擦轮弹速闭环
static float last_bullet_speed_feedback = 0.0f;
static float friction_ref = SHOOT_FRICTION_BASE_REF;
static friction_mode_e last_friction_mode = FRICTION_OFF;
static uint8_t friction_soft_stopping = 0;
static float friction_soft_stop_start_ms = 0.0f;
// 拨弹盘前馈
static float loader_current_forward = 0;

// dwt定时
static float hibernate_time = 0, dead_time = 0, blocked_num = 0, blocked_time = 0;
static float load_angle_set = 0;

static uint8_t LoaderBlockedDetected(void)
{
    if (loader == NULL)
    {
        return 0;
    }

    return loader->motor_controller.speed_PID.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR &&
           fabsf(loader->measure.speed_aps) < BLOCK_SPEED_THRESHOLD &&
           abs(loader->measure.real_current) > BLOCK_CURRENT_THRESHOLD;
}

static float ClampFrictionRef(float ref)
{
    if (ref < SHOOT_FRICTION_REF_MIN)
        return SHOOT_FRICTION_REF_MIN;
    if (ref > SHOOT_FRICTION_REF_MAX)
        return SHOOT_FRICTION_REF_MAX;
    return ref;
}

static void UpdateFrictionRefByBulletSpeed(float bullet_speed)
{
    float speed_error;

    if (bullet_speed < 5.0f || bullet_speed > 40.0f)
        return;

    if (fabsf(bullet_speed - last_bullet_speed_feedback) < 0.05f)
        return;

    last_bullet_speed_feedback = bullet_speed;
    speed_error = SHOOT_BULLET_SPEED_TARGET - bullet_speed;

    if (fabsf(speed_error) < SHOOT_BULLET_SPEED_DEADBAND)
        return;

    friction_ref += SHOOT_FRICTION_SPEED_KP * speed_error;
    friction_ref = ClampFrictionRef(friction_ref);
}

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
                .Kp = 10, //20
                .Ki = 3, //1
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
                .Ki = 3, //1
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
                .Kp = 38, //10
                .Ki = 2.8,
                .Kd = 0,
                .MaxOut = 20000,
            },
            .speed_PID = {
                .Kp = 1.5, //0
                .Ki = 2.0, //02.0
                .Kd = 0.0, //
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
                .MaxOut = 10000, //10000
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
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    
    // 发射电机离线检测配置 
    MotorOfflineAlarmConfig_t shoot_alarm_cfg = {
        .motors = {loader, friction_l, friction_r},
        .is_online = DJIMotorIsOnline,
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
    //VOFA(0,loader->measure.angle_single_round,loader->measure.speed_aps,loader->measure.real_current,shoot_cmd_recv.load_mode == LOAD_REVERSE);
    //VOFA(0,friction_l->measure.speed_aps, friction_r->measure.speed_aps, loader->measure.angle_single_round);
    // 从 cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
        friction_ref = SHOOT_FRICTION_BASE_REF;
        last_bullet_speed_feedback = 0.0f;
        friction_soft_stopping = 0;
        last_friction_mode = FRICTION_OFF;
    }
    else // 恢复运行
    {
         DJIMotorEnable(friction_l);
         DJIMotorEnable(friction_r);
         DJIMotorEnable(loader);
        // DJIMotorStop(friction_l);
        // DJIMotorStop(friction_r);
        // DJIMotorStop(loader);
    }
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        friction_soft_stopping = 0;
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        UpdateFrictionRefByBulletSpeed(shoot_cmd_recv.bullet_speed);
        DJIMotorOuterLoop(friction_l, SPEED_LOOP);
        DJIMotorOuterLoop(friction_r, SPEED_LOOP);
        DJIMotorSetRef(friction_l, friction_ref);
        DJIMotorSetRef(friction_r, friction_ref);

    }
    else // 关闭摩擦轮
    {
        if (last_friction_mode == FRICTION_ON && !friction_soft_stopping)
        {
            friction_soft_stopping = 1;
            friction_soft_stop_start_ms = DWT_GetTimeline_ms();
        }

        if (friction_soft_stopping &&
            (DWT_GetTimeline_ms() - friction_soft_stop_start_ms) < SHOOT_FRICTION_SOFT_STOP_MS)
        {
            DJIMotorEnable(friction_l);
            DJIMotorEnable(friction_r);
            DJIMotorOuterLoop(friction_l, SPEED_LOOP);
            DJIMotorOuterLoop(friction_r, SPEED_LOOP);
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
        }
        else
        {
            friction_soft_stopping = 0;
            DJIMotorStop(friction_l);
            DJIMotorStop(friction_r);
        }
        friction_ref = SHOOT_FRICTION_BASE_REF;
        last_bullet_speed_feedback = 0.0f;
    }
    last_friction_mode = shoot_cmd_recv.friction_mode;
    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;

    if (shoot_cmd_recv.bullet_speed >= shoot_cmd_recv.rest_heat + 10 || shoot_cmd_recv.bullet_speed == 0 || shoot_cmd_recv.rest_heat == 0)
    {
        if (LoaderBlockedDetected())
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
        // 拨盘反转
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

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弈反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

#endif // USE_LASER_POINTER
