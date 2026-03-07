#include "dji_motor.h"
#include "general_def.h"
#include "bsp_log.h"

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用

/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL};

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 * GM6020电流控制模式:
 * can1: [6]:0x1FE,[7]:0x2FE
 * can2: [8]:0x1FE,[9]:0x2FE
 */
static CANInstance sender_assignment[10] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [6] = {.can_handle = &hcan1, .txconf.StdId = 0x1fe, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},  // GM6020 id 1-4 CAN1
    [7] = {.can_handle = &hcan1, .txconf.StdId = 0x2fe, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},  // GM6020 id 5-7 CAN1
    [8] = {.can_handle = &hcan2, .txconf.StdId = 0x1fe, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},  // GM6020 id 1-4 CAN2
    [9] = {.can_handle = &hcan2, .txconf.StdId = 0x2fe, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},  // GM6020 id 5-7 CAN2
};

/**
 * @brief 10个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 *
 */
static uint8_t sender_enable_flag[10] = {0};

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 *
 * @param config
 */
static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 1 : 4;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }

        // 计算接收id并设置分组发送id
        config->rx_id = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        // 检查是否发生id冲突
        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information."); // 后续可以把id和CAN打印出来
                while (1)
                    ; // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
            }
        }
        break;

    case GM6020:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 2 : 5;
        }

        config->rx_id = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                while (1)
                    ; // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8)
            }
        }
        break;

    case GM6020_CURRENT: // GM6020电流控制模式,发送帧为0x1FE/0x2FE
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 6 : 8;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 7 : 9;
        }

        config->rx_id = 0x204 + motor_id + 1;   // 反馈报文与电压控制模式相同
        sender_enable_flag[motor_grouping] = 1;
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                while (1)
                    ;
            }
        }
        break;

    default: // other motors should not be registered here
        LOGERROR("You must not register other motors using the API of DJI motor.");
        while (1)
            ; // 其他电机不应该在这里注册
    }
}

// 底盘电机指针数组(用于功率限制)
static DJIMotorInstance *dji_motor_powerLimit_3508[POWER_LIMIT_3508_CNT] = {NULL};
static uint8_t idx_powerLimit_3508 = 0;

// 功率限制参数
static float Chassis_Power_Max = 30.0f;  // 底盘功率上限(W),应从裁判系统获取
static float limit_coef = 1.0f;          // 电流衰减系数(0~1)
static float total_predicted_power = 0;  // 总预测功率

/**
 * @brief 注册需要进行功率限制的电机
 * @param motor 电机实例指针
 * @param motor_type 电机类型
 */
void DJIMotorSetPowerLimitMotors(DJIMotorInstance *motor, Motor_Type_e motor_type)
{
    if (motor == NULL) return;
    
    if (motor_type == M3508 && idx_powerLimit_3508 < POWER_LIMIT_3508_CNT)
    {
        // 检查是否已注册
        for (uint8_t i = 0; i < idx_powerLimit_3508; i++)
        {
            if (dji_motor_powerLimit_3508[i] == motor) return;
        }
        dji_motor_powerLimit_3508[idx_powerLimit_3508++] = motor;
    }
}

/**
 * @brief 功率预测函数 - 根据电流和转速预测单个电机功率
 * @param motor 电机实例指针
 * @note 使用6参数模型: P = K0 + K1*I + K2*ω + K3*I*ω + K4*I² + K5*ω²
 */
static void DJIMotorPowerPredict(DJIMotorInstance *motor)
{
    if (motor == NULL) return;
    
    // 电流转换: real_current是原始值,需要转换为安培
    // M3508: real_current * 20 / 16384 = 实际电流(A)
    float current = motor->measure.real_current * 20.0f / 16384.0f;
    float speed = motor->measure.speed_aps * DEGREE_2_RAD;  // 转为弧度每秒
    
    // 使用绝对值,功率与方向无关
    // float abs_speed = (speed > 0) ? speed : -speed;
    
    motor->measure.predicted_power = k0 + k1 * current + k2 * speed 
                                   + k3 * current * speed 
                                   + k4 * current * current 
                                   + k5 * speed * speed;
}

/**
 * @brief 功率限制函数 - 计算电流衰减系数
 * @note 当预测总功率超过限制时,通过求解二次方程得到衰减系数
 *       二次方程: a*x² + b*x + c = 0, 其中x为电流衰减系数
 */
static void DJIMotorPowerLimit(void)
{
    total_predicted_power = 0;
    
    // 1. 计算所有电机的预测功率
    for (uint8_t i = 0; i < POWER_LIMIT_3508_CNT; i++)
    {
        if (dji_motor_powerLimit_3508[i] != NULL)
        {
            DJIMotorPowerPredict(dji_motor_powerLimit_3508[i]);
            total_predicted_power += dji_motor_powerLimit_3508[i]->measure.predicted_power;
        }
    }
    
    // 2. 判断是否超功率
    if (total_predicted_power <= Chassis_Power_Max)
    {
        limit_coef = 1.0f;  // 未超功率,不限制
        return;
    }
    
    // 3. 超功率,求解衰减系数
    // 建立二次方程: Σ(K0 + K1*I*x + K2*ω + K3*I*x*ω + K4*(I*x)² + K5*ω²) = P_max
    // 整理为: a*x² + b*x + c = 0
    float a = 0, b = 0, c = 0;
    
    for (uint8_t i = 0; i < POWER_LIMIT_3508_CNT; i++)
    {
        if (dji_motor_powerLimit_3508[i] != NULL)
        {
            float current = dji_motor_powerLimit_3508[i]->measure.real_current * 20.0f / 16384.0f;
            float speed = dji_motor_powerLimit_3508[i]->measure.speed_aps * DEGREE_2_RAD;
            // float abs_speed = (speed > 0) ? speed : -speed;
            
            // a = Σ K4 * I²
            a += k4 * current * current;
            // b = Σ (K1*I + K3*I*ω)
            b += k1 * current + k3 * current * speed;
            // c = Σ (K0 + K2*ω + K5*ω²)
            c += k0 + k2 * speed + k5 * speed * speed;
        }
    }
    
    // c需要减去目标功率
    c -= Chassis_Power_Max;
    
    // 4. 求解二次方程
    float delta = b * b - 4 * a * c;
    
    if (delta < 0)
    {
        // 无解,直接置零(极端情况)
        limit_coef = 0;
        return;
    }
    
    float sqrt_delta = Sqrt(delta);
    float x1 = (-b + sqrt_delta) / (2 * a);
    float x2 = (-b - sqrt_delta) / (2 * a);
    
    // 选择在(0,1)区间内的解
    if((x1 > 0 && x1 < 1) || (x2 > 0 && x2 < 1))
        {
            // 取有效解（优先取较大的，减少性能损失）
            limit_coef = (x1 > 0 && x1 < 1) ? x1 : x2;
        }
    else
    {
        // 两个解都不在有效区间,置零
        limit_coef = 0;
    }
}

/**
 * @brief 获取当前功率限制系数
 * @return float 限制系数(0~1)
 */
float DJIMotorGetPowerLimitCoef(void)
{
    return limit_coef;
}

/**
 * @todo  是否可以简化多圈角度的计算？
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DecodeDJIMotor(CANInstance *_instance)
{
    // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
    uint8_t *rxbuff = _instance->rx_buff;
    DJI_Motor_Measure_s *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

    // 喂狗 - 收到CAN报文表示电机在线
    if (motor->motor_daemon != NULL) {
        DaemonReload(motor->motor_daemon);
    }

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperate = rxbuff[6];

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

// 电机初始化,返回一个电机实例
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    memset(instance, 0, sizeof(DJIMotorInstance));

    // motor basic setting 电机基本设置
    instance->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

    // motor controller init 电机控制器初始化
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    // 后续增加电机前馈控制器(速度和电流)
    instance->motor_controller.speed_foward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    instance->motor_controller.current_foward_ptr = config->controller_param_init_config.current_feedforward_ptr;

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    MotorSenderGrouping(instance, &config->can_init_config);

    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = DecodeDJIMotor; // set callback
    config->can_init_config.id = instance;                        // set id,eq to address(it is identity)
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    // 注册电机离线检测 daemon (200ms超时, DaemonTask以1ms运行)
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 200,
        .callback = NULL,  // 电机离线时不执行额外回调，由用户主动查询
        .owner_id = instance,
    };
    instance->motor_daemon = DaemonRegister(&daemon_conf);

    DJIMotorEnable(instance);
    dji_motor_instance[idx++] = instance;
    return instance;
}

/* 电流只能通过电机自带传感器监测,后续考虑加入力矩传感器应变片等 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
    {
        motor->motor_settings.angle_feedback_source = type;
    }
    else if (loop == SPEED_LOOP)
    {
        motor->motor_settings.speed_feedback_source = type;
    }
    else
    {
        LOGERROR("[dji_motor] loop type error, check memory access and func param"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
    }
}

void DJIMotorStop(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 修改电机的实际闭环对象 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

/* 修改电机的前馈标志 */
void DJIMotorSetFeedfoward(DJIMotorInstance *motor, Feedfoward_Type_e feedfoward_loop)
{
    motor->motor_settings.feedforward_flag = feedfoward_loop;
}

// 设置参考值
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

// 为所有电机实例计算三环PID,发送控制报文
void DJIMotorControl()
{
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // 电机控制CAN发送设定值
    DJIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    DJI_Motor_Measure_s *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机PID测量值和设定值

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i)
    { // 减小访存开销,先保存指针引用
        motor = dji_motor_instance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;
        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->outer_loop_type & ANGLE_LOOP))
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->outer_loop_type & SPEED_LOOP) || (motor_setting->close_loop_type & SPEED_LOOP))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor->motor_controller.speed_foward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed_aps;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor->motor_controller.current_foward_ptr;

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if ((motor_setting->outer_loop_type & CURRENT_LOOP) || (motor_setting->close_loop_type & CURRENT_LOOP))
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
        }

        // 获取最终输出
        set = (int16_t)pid_ref;
        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            set *= -1;
        // 分组填入发送数据
        group = motor->sender_group;
        num = motor->message_num;
        motor->measure.target_current = set;
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)((int16_t)motor->measure.target_current >> 8);         // 低八位
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)((int16_t)motor->measure.target_current & 0x00ff); // 高八位

        // 电机是否停止运行
        if (motor->stop_flag == MOTOR_STOP)
        { // 若该电机处于停止状态,直接将buff置零
            memset(sender_assignment[group].tx_buff + 2 * num, 0, 16u);
            motor->real_output = motor->measure.target_current = 0;
        }
    }
    
    // ==================== 功率限制处理 ====================
#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
    // 计算功率限制系数
    DJIMotorPowerLimit();
    
    // 如果需要限制功率(limit_coef < 1),则对底盘电机电流进行衰减
    if (limit_coef < 1.0f)
    {
        for (uint8_t i = 0; i < POWER_LIMIT_3508_CNT; i++)
        {
            if (dji_motor_powerLimit_3508[i] != NULL)
            {
                DJIMotorInstance *chassis_motor = dji_motor_powerLimit_3508[i];
                
                // 应用衰减系数
                chassis_motor->real_output = (int16_t)(chassis_motor->measure.target_current * limit_coef);
                
                // 更新发送缓冲区
                uint8_t grp = chassis_motor->sender_group;
                uint8_t msg_num = chassis_motor->message_num;
                sender_assignment[grp].tx_buff[2 * msg_num] = (uint8_t)(((int16_t)chassis_motor->real_output) >> 8);
                sender_assignment[grp].tx_buff[2 * msg_num + 1] = (uint8_t)(((int16_t)chassis_motor->real_output) & 0x00ff);
            }
        }
    }
#endif

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 10; ++i)
    {
        if (sender_enable_flag[i])
        {
            if (!CANTransmit(&sender_assignment[i], 1))
            {
                // 发送失败，下次再试
                // 避免持续堵塞
            }
        }
    }
}

/**
 * @brief 检查电机是否在线
 * @param motor 电机实例指针 (void*以兼容通用离线报警模块)
 * @return uint8_t 1=在线, 0=离线
 */
uint8_t DJIMotorIsOnline(void *motor)
{
    DJIMotorInstance *m = (DJIMotorInstance *)motor;
    if (m == NULL || m->motor_daemon == NULL)
        return 0;
    return DaemonIsOnline(m->motor_daemon);
}

void ChassisPowerSet(float power)
{
    Chassis_Power_Max = power;
}