// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "transfer_image.h"
#include <string.h>
#include "ins_task.h"
#include "vofa.h"
#include "bsp_dwt.h"
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
#include <math.h>

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* gimbal_cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif

#ifdef CHASSIS_ONLY
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // CHASSIS_ONLY

//#define OUTBREAK_PRIORITY
#define COOLING_PRIORITY

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // DJI遥控器数据
static Image_RC_ctrl_t *image_rc_data;  // 图传遥控器数据
static float yaw_diff;

// ======================== 视觉数据指针 - 根据协议类型条件编译 ========================
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
    static Vision_Recv_s *vision_recv_data; // VCP/UART协议视觉接收数据
    static Vision_Recv_s vision_recv_cache; // 任务内使用的视觉快照,避免和中断回调并发读写
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
static uint8_t yaw_ident_active;
static float yaw_ident_center_yaw;
static float yaw_ident_hold_pitch;
static float yaw_ident_start_ms;
static uint32_t yaw_ident_round_trip_count;
static float pitch_ident_center_yaw;
static float pitch_ident_center_pitch;
static fire_mode_e vision_fire_mode = FIRE_ON;

#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
static uint8_t vision_interp_initialized;
static float vision_interp_start_yaw;
static float vision_interp_target_yaw;
static float vision_interp_start_pitch;
static float vision_interp_target_pitch;
static float vision_interp_start_ms;
static uint8_t vision_interp_pitch_valid;
static uint8_t vision_interp_last_tracking;
#endif

uint8_t remote_flag = 0;  // 遥控器类型选择: 0=未选择, 1=DJI遥控(键鼠), 2=图传遥控
uint8_t stop_flag = 0;    // 急停锁存: 0=运行, 1=急停 (key_stop上升沿切换)
uint8_t spin_flag = 0;    // 图传右侧按键自旋锁存: 0=关闭, 1=开启

#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
static float GetVisionInterpValue(float start, float target, float now_ms)
{
    float duration_ms = VISION_INTERP_DURATION_MS;
    float progress;

    if (duration_ms <= 1e-3f)
        return target;

    progress = (now_ms - vision_interp_start_ms) / duration_ms;
    progress = loop_float_constrain(progress, 0.0f, 1.0f);
    return start + (target - start) * progress;
}

static void GetVisionInterpolatedAim(float *yaw, float *pitch, uint8_t *pitch_valid)
{
    float now_ms = DWT_GetTimeline_ms();

    if (!vision_interp_initialized)
    {
        *yaw = vision_recv_cache.aimYaw;
        *pitch = vision_recv_cache.aimPitch;
        *pitch_valid = (fabsf(vision_recv_cache.aimPitch) > 1e-3f);
        return;
    }

    *yaw = GetVisionInterpValue(vision_interp_start_yaw, vision_interp_target_yaw, now_ms);
    *pitch = GetVisionInterpValue(vision_interp_start_pitch, vision_interp_target_pitch, now_ms);
    *pitch_valid = vision_interp_pitch_valid;
}

static void UpdateVisionInterpolation(void)
{
    float now_ms = DWT_GetTimeline_ms();
    float current_yaw;
    float current_pitch;
    uint8_t tracking_valid = (vision_recv_cache.tracking != NO_TARGET);
    uint8_t pitch_valid = (fabsf(vision_recv_cache.aimPitch) > 1e-3f);
    uint8_t target_changed;

    if (!vision_interp_initialized)
    {
        vision_interp_initialized = 1;
        vision_interp_start_yaw = vision_recv_cache.aimYaw;
        vision_interp_target_yaw = vision_recv_cache.aimYaw;
        vision_interp_start_pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        vision_interp_target_pitch = pitch_valid ? vision_recv_cache.aimPitch : gimbal_fetch_data.gimbal_imu_data.Pitch;
        vision_interp_start_ms = now_ms;
        vision_interp_pitch_valid = pitch_valid;
        vision_interp_last_tracking = tracking_valid;
        return;
    }

    current_yaw = GetVisionInterpValue(vision_interp_start_yaw, vision_interp_target_yaw, now_ms);
    current_pitch = GetVisionInterpValue(vision_interp_start_pitch, vision_interp_target_pitch, now_ms);

    target_changed =
        (fabsf(vision_recv_cache.aimYaw - vision_interp_target_yaw) > VISION_INTERP_EPSILON_DEG) ||
        (pitch_valid && fabsf(vision_recv_cache.aimPitch - vision_interp_target_pitch) > VISION_INTERP_EPSILON_DEG) ||
        (vision_interp_pitch_valid != pitch_valid) ||
        (vision_interp_last_tracking != tracking_valid);

    if (!target_changed)
        return;

    vision_interp_start_yaw = current_yaw;
    vision_interp_target_yaw = vision_recv_cache.aimYaw;
    vision_interp_start_pitch = current_pitch;
    vision_interp_target_pitch = pitch_valid ? vision_recv_cache.aimPitch : current_pitch;
    vision_interp_start_ms = now_ms;
    vision_interp_pitch_valid = pitch_valid;
    vision_interp_last_tracking = tracking_valid;
}

static void RefreshVisionRecvCache(void)
{
    uint32_t irq_state = __get_PRIMASK();

    __disable_irq();
    vision_recv_cache = *vision_recv_data;
    __set_PRIMASK(irq_state);

    vision_recv_cache.aimYaw = loop_float_constrain(vision_recv_cache.aimYaw, -180.0f, 180.0f);
}

static Bullet_Speed_e GetVisionBulletSpeedFlag(float bullet_speed)
{
    if (bullet_speed >= 24.0f)
        return SMALL_AMU_30;
    if (bullet_speed >= 17.0f)
        return SMALL_AMU_18;
    if (bullet_speed >= 12.0f)
        return SMALL_AMU_15;
    return BULLET_SPEED_NONE;
}

static Detect_Color_e GetEnemyDetectColor(Detect_Color_e self_color)
{
    if (self_color == COLOR_RED)
        return COLOR_BLUE;
    if (self_color == COLOR_BLUE)
        return COLOR_RED;
    return COLOR_NONE;
}
#endif

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

static void ResetYawIdentificationMode(void)
{
    yaw_ident_active = 0;
}

static float ClampYawIdentSpeed(float speed_dps)
{
    if (speed_dps < 0.0f)
        return 0.0f;
    if (speed_dps > YAW_IDENT_MAX_SPEED_DPS)
        return YAW_IDENT_MAX_SPEED_DPS;
    return speed_dps;
}

static float ClampPitchIdentSpeed(float speed_dps)
{
    if (speed_dps < 0.0f)
        return 0.0f;
    if (speed_dps > PITCH_IDENT_MAX_SPEED_DPS)
        return PITCH_IDENT_MAX_SPEED_DPS;
    return speed_dps;
}

static float ClampSpinIdentSpeed(float speed_dps)
{
    if (speed_dps < 0.0f)
        return 0.0f;
    if (speed_dps > SPIN_IDENT_MAX_SPEED_DPS)
        return SPIN_IDENT_MAX_SPEED_DPS;
    return speed_dps;
}

static float CalcYawIdentTargetOffset(float elapsed_ms, float speed_dps)
{
    float hold_ms = YAW_IDENT_HOLD_MS;
    float platform_ms = YAW_IDENT_PLATFORM_MS;
    float cycle_ms = 4.0f * hold_ms + 4.0f * platform_ms;
    float phase_ms = elapsed_ms;
    float amplitude_deg = speed_dps * platform_ms / 1000.0f;

    while (phase_ms >= cycle_ms)
        phase_ms -= cycle_ms;

    if (phase_ms < hold_ms)
        return 0.0f;
    phase_ms -= hold_ms;

    if (phase_ms < platform_ms)
        return speed_dps * phase_ms / 1000.0f;
    phase_ms -= platform_ms;

    if (phase_ms < hold_ms)
        return amplitude_deg;
    phase_ms -= hold_ms;

    if (phase_ms < 2.0f * platform_ms)
        return amplitude_deg - speed_dps * phase_ms / 1000.0f;
    phase_ms -= 2.0f * platform_ms;

    if (phase_ms < hold_ms)
        return -amplitude_deg;
    phase_ms -= hold_ms;

    if (phase_ms < platform_ms)
        return -amplitude_deg + speed_dps * phase_ms / 1000.0f;

    return 0.0f;
}

static float CalcPitchIdentTargetOffset(float elapsed_ms, float speed_dps)
{
    float hold_ms = PITCH_IDENT_HOLD_MS;
    float platform_ms = PITCH_IDENT_PLATFORM_MS;
    float cycle_ms = 4.0f * hold_ms + 4.0f * platform_ms;
    float phase_ms = elapsed_ms;
    float amplitude_deg = speed_dps * platform_ms / 1000.0f;

    while (phase_ms >= cycle_ms)
        phase_ms -= cycle_ms;

    if (phase_ms < hold_ms)
        return 0.0f;
    phase_ms -= hold_ms;

    if (phase_ms < platform_ms)
        return speed_dps * phase_ms / 1000.0f;
    phase_ms -= platform_ms;

    if (phase_ms < hold_ms)
        return amplitude_deg;
    phase_ms -= hold_ms;

    if (phase_ms < 2.0f * platform_ms)
        return amplitude_deg - speed_dps * phase_ms / 1000.0f;
    phase_ms -= 2.0f * platform_ms;

    if (phase_ms < hold_ms)
        return -amplitude_deg;
    phase_ms -= hold_ms;

    if (phase_ms < platform_ms)
        return -amplitude_deg + speed_dps * phase_ms / 1000.0f;

    return 0.0f;
}

static float CalcSpinIdentTargetWz(float elapsed_ms, float speed_dps)
{
    float hold_ms = SPIN_IDENT_HOLD_MS;
    float platform_ms = SPIN_IDENT_PLATFORM_MS;
    float cycle_ms = 4.0f * hold_ms + 4.0f * platform_ms;
    float phase_ms = elapsed_ms;

    while (phase_ms >= cycle_ms)
        phase_ms -= cycle_ms;

    if (phase_ms < hold_ms)
        return 0.0f;
    phase_ms -= hold_ms;

    if (phase_ms < platform_ms)
        return speed_dps;
    phase_ms -= platform_ms;

    if (phase_ms < hold_ms)
        return speed_dps;
    phase_ms -= hold_ms;

    if (phase_ms < platform_ms)
        return 0.0f;
    phase_ms -= platform_ms;

    if (phase_ms < hold_ms)
        return 0.0f;
    phase_ms -= hold_ms;

    if (phase_ms < platform_ms)
        return -speed_dps;
    phase_ms -= platform_ms;

    if (phase_ms < hold_ms)
        return -speed_dps;
    phase_ms -= hold_ms;

    if (phase_ms < platform_ms)
        return 0.0f;

    return 0.0f;
}

static void ApplyYawIdentificationMode(void)
{
    float elapsed_ms;
    float speed_dps;
    float hold_ms = YAW_IDENT_HOLD_MS;
    float platform_ms = YAW_IDENT_PLATFORM_MS;
    float cycle_ms = 4.0f * hold_ms + 4.0f * platform_ms;
    float target_offset_deg;

    if (!yaw_ident_active)
    {
        yaw_ident_active = 1;
        yaw_ident_center_yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        yaw_ident_hold_pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        yaw_ident_start_ms = DWT_GetTimeline_ms();
        yaw_ident_round_trip_count = 0;
    }

    elapsed_ms = DWT_GetTimeline_ms() - yaw_ident_start_ms;
    yaw_ident_round_trip_count = (uint32_t)(elapsed_ms / cycle_ms);
    speed_dps = YAW_IDENT_START_SPEED_DPS + yaw_ident_round_trip_count * YAW_IDENT_SPEED_STEP_DPS;
    speed_dps = ClampYawIdentSpeed(speed_dps);
    target_offset_deg = CalcYawIdentTargetOffset(elapsed_ms, speed_dps);

    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.dash_mode = DASH_OFF;
    chassis_cmd_send.aim_mode = AIM_OFF;

    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.yaw = yaw_ident_center_yaw + target_offset_deg;
    gimbal_cmd_send.pitch = yaw_ident_hold_pitch;

    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
}

static void ApplyPitchIdentificationMode(void)
{
    float elapsed_ms;
    float speed_dps;
    float hold_ms = PITCH_IDENT_HOLD_MS;
    float platform_ms = PITCH_IDENT_PLATFORM_MS;
    float cycle_ms = 4.0f * hold_ms + 4.0f * platform_ms;
    float target_offset_deg;

    if (!yaw_ident_active)
    {
        yaw_ident_active = 1;
        pitch_ident_center_yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        pitch_ident_center_pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        yaw_ident_start_ms = DWT_GetTimeline_ms();
        yaw_ident_round_trip_count = 0;
    }

    elapsed_ms = DWT_GetTimeline_ms() - yaw_ident_start_ms;
    yaw_ident_round_trip_count = (uint32_t)(elapsed_ms / cycle_ms);
    speed_dps = PITCH_IDENT_START_SPEED_DPS + yaw_ident_round_trip_count * PITCH_IDENT_SPEED_STEP_DPS;
    speed_dps = ClampPitchIdentSpeed(speed_dps);
    target_offset_deg = CalcPitchIdentTargetOffset(elapsed_ms, speed_dps);

    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.dash_mode = DASH_OFF;
    chassis_cmd_send.aim_mode = AIM_OFF;

    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.yaw = pitch_ident_center_yaw;
    gimbal_cmd_send.pitch = pitch_ident_center_pitch + target_offset_deg;

    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
}

static void ApplySpinIdentificationMode(void)
{
    float elapsed_ms;
    float speed_dps;
    float hold_ms = SPIN_IDENT_HOLD_MS;
    float platform_ms = SPIN_IDENT_PLATFORM_MS;
    float cycle_ms = 4.0f * hold_ms + 4.0f * platform_ms;
    float target_wz_dps;

    if (!yaw_ident_active)
    {
        yaw_ident_active = 1;
        yaw_ident_center_yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        yaw_ident_hold_pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        yaw_ident_start_ms = DWT_GetTimeline_ms();
        yaw_ident_round_trip_count = 0;
    }

    elapsed_ms = DWT_GetTimeline_ms() - yaw_ident_start_ms;
    yaw_ident_round_trip_count = (uint32_t)(elapsed_ms / cycle_ms);
    speed_dps = SPIN_IDENT_START_SPEED_DPS + yaw_ident_round_trip_count * SPIN_IDENT_SPEED_STEP_DPS;
    speed_dps = ClampSpinIdentSpeed(speed_dps);
    target_wz_dps = CalcSpinIdentTargetWz(elapsed_ms, speed_dps);

    chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = target_wz_dps;
    chassis_cmd_send.dash_mode = DASH_OFF;
    chassis_cmd_send.aim_mode = AIM_OFF;

    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.yaw = yaw_ident_center_yaw;
    gimbal_cmd_send.pitch = yaw_ident_hold_pitch;

    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
}

static void ApplyYawSineIdentificationMode(void)
{
    float elapsed_ms;
    float sample_time_ms;
    float target_offset_deg;

    if (!yaw_ident_active)
    {
        yaw_ident_active = 1;
        yaw_ident_center_yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        yaw_ident_hold_pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        yaw_ident_start_ms = DWT_GetTimeline_ms();
        yaw_ident_round_trip_count = 0;
    }

    elapsed_ms = DWT_GetTimeline_ms() - yaw_ident_start_ms;
    sample_time_ms = floorf(elapsed_ms / YAW_SINE_IDENT_UPDATE_MS) * YAW_SINE_IDENT_UPDATE_MS;
    target_offset_deg =
        YAW_SINE_IDENT_AMPLITUDE_DEG *
        sinf(2.0f * 3.1415926f * YAW_SINE_IDENT_FREQ_HZ * sample_time_ms / 1000.0f);

    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.dash_mode = DASH_OFF;
    chassis_cmd_send.aim_mode = AIM_OFF;

    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.yaw = yaw_ident_center_yaw + target_offset_deg;
    gimbal_cmd_send.pitch = yaw_ident_hold_pitch;

    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
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
    rc_data = RemoteControlInit(&huart3);       // DJI遥控器: USART3 (DBUS)
    image_rc_data = TransferImageInit(&huart6); // 图传遥控器: USART6
    
#ifdef CHASSIS_ONLY
    // ===================== CHASSIS_ONLY模式: 只初始化底盘 =====================
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    robot_state = ROBOT_READY;
#else
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
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.fire_mode = FIRE_ON;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    robot_state = ROBOT_STOP; // 上电默认停止, 需遥控器在线且拨杆正确才能恢复
#endif // CHASSIS_ONLY
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
    chassis_cmd_send.fire_mode = vision_fire_mode;
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    
#ifdef CHASSIS_ONLY
    // ===================== CHASSIS_ONLY模式: 仅底盘控制 =====================
    // 右侧开关控制底盘模式
    if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;  // 小陀螺
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;  // 自由平移
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;  // 跟头模式(左摇杆转向)
    }

    // 底盘速度控制 - 右摇杆
    chassis_cmd_send.vx = 45.0f * (float)rc_data[TEMP].rc.rocker_r_; // degree/s
    chassis_cmd_send.vy = 45.0f * (float)rc_data[TEMP].rc.rocker_r1; // degree/s
    
    // 左摇杆控制旋转(非小陀螺模式时)
    if (chassis_cmd_send.chassis_mode != CHASSIS_ROTATE)
    {
        int16_t rocker_l = rc_data[TEMP].rc.rocker_l_;
        if (rocker_l <= 10 && rocker_l >= -10) rocker_l = 0;
        chassis_cmd_send.wz = -20.0f * (float)rocker_l;
    }

    chassis_cmd_send.dash_mode = DASH_OFF;
    chassis_cmd_send.offset_angle = 0;  // 无云台时偏移角固定为0
    
#else
    // ===================== 完整模式 =====================
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],进入小陀螺模式
    {
#if IDENT_MODE_ENABLE == IDENT_MODE_YAW
        ApplyYawIdentificationMode();
#elif IDENT_MODE_ENABLE == IDENT_MODE_PITCH
        ApplyPitchIdentificationMode();
#elif IDENT_MODE_ENABLE == IDENT_MODE_SPIN
        ApplySpinIdentificationMode();
#elif IDENT_MODE_ENABLE == IDENT_MODE_YAW_SINE
        ApplyYawSineIdentificationMode();
#else
        ResetYawIdentificationMode();
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
#endif
        return;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        ResetYawIdentificationMode();
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; //全改成云台gyro反馈
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))  // 右侧开关状态[上],进入跟头模式
    {
        ResetYawIdentificationMode();
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; //全改成云台gyro反馈
    }
    

    // 云台参数,确定云台控制数据


    uint8_t no_target = 0;
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
    no_target = (vision_recv_cache.tracking == NO_TARGET);
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
    {
        float interp_yaw;
        float interp_pitch;
        uint8_t interp_pitch_valid;

        GetVisionInterpolatedAim(&interp_yaw, &interp_pitch, &interp_pitch_valid);
        yaw_diff = interp_yaw - gimbal_cmd_send.yaw;
        yaw_diff = loop_float_constrain(yaw_diff, -45.0f, 45.0f);
        gimbal_cmd_send.yaw += yaw_diff;
        if (interp_pitch_valid)
            gimbal_cmd_send.pitch = interp_pitch;
    }
        // 视觉模式下遥控器微调 (二段灵敏度, 方便快速更换目标)  
        //VisionRockerAdjust();
#elif defined(VISION_USE_SERIALPORT)
        // SerialPort协议 - 需要转换整数到浮点数
        // 上位机发送的是放大后的整数值,需要根据实际编码规则转换
        // 假设: yaw和pitch都是角度*100的整数形式
        if (vision_recv_data_sp->yaw != 0 || vision_recv_data_sp->pitch != 0) {
            gimbal_cmd_send.yaw = loop_float_constrain((float)vision_recv_data_sp->yaw / 100.0f, -180.0f, 180.0f);
            gimbal_cmd_send.pitch = (float)vision_recv_data_sp->pitch / 100.0f;
        }
        
        // 视觉模式下遥控器微调 (二段灵敏度, 方便快速更换目标)
        if (vision_recv_data_sp->shootStatus != 0) {
            VisionRockerAdjust();
        }
#elif defined(VISION_USE_SP)
        // SP协议, 视觉返回目标角 (前馈现在在gimbal中本地计算)
        if (vision_recv_sp->mode != 0) {
            gimbal_cmd_send.yaw = loop_float_constrain(vision_recv_sp->yaw, -180.0f, 180.0f);
            gimbal_cmd_send.pitch = vision_recv_sp->pitch;
            
            // 二段灵敏度微调
            VisionRockerAdjust();
        }
#endif
        
        // 软件限位
        if (gimbal_cmd_send.pitch <= PITCH_MIN_LIMIT)
         {
             gimbal_cmd_send.pitch = PITCH_MIN_LIMIT;
         }
         if (gimbal_cmd_send.pitch >= PITCH_MAX_LIMIT)
         {
             gimbal_cmd_send.pitch = PITCH_MAX_LIMIT;
         }
         
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
        gimbal_cmd_send.pitch += 0.0015f * (float)rc_data[TEMP].rc.rocker_l1;
        
        // 软件限位
         if (gimbal_cmd_send.pitch <= PITCH_MIN_LIMIT)
         {
             gimbal_cmd_send.pitch = PITCH_MIN_LIMIT;
         }
         if (gimbal_cmd_send.pitch >= PITCH_MAX_LIMIT)
         {
             gimbal_cmd_send.pitch = PITCH_MAX_LIMIT;
         }
    }


    // 单位: degree/s (电机转子角速度), 摇杆范围±660, 系数×660=满杆转速
    // M3508最高约54000 degree/s, 45×660≈29700, 留有余量
    chassis_cmd_send.vx = 80.0f * (float)rc_data[TEMP].rc.rocker_r_; // degree/s
    chassis_cmd_send.vy = 80.0f * (float)rc_data[TEMP].rc.rocker_r1; // degree/s
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
        // VCP/UART协议: 视觉fire=1且未手动拨弹时自动开火
    if (vision_fire_mode == FIRE_ON && vision_recv_cache.fire == 1 && shoot_cmd_send.load_mode == LOAD_STOP)
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
#elif defined(VISION_USE_SERIALPORT)
        // SerialPort协议: 使用shootStatus字段 (非0=跟踪且开火, 0=停火)
        if (vision_fire_mode == FIRE_ON && vision_recv_data_sp->shootStatus != 0)
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
        if (vision_fire_mode == FIRE_ON && vision_recv_sp->mode == 2)
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
    Limitshoot();
#endif // CHASSIS_ONLY
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
#ifdef CHASSIS_ONLY
    // ===================== CHASSIS_ONLY模式: 仅底盘急停处理 =====================
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || switch_is_mid(rc_data[TEMP].rc.switch_left))
    {
        if (robot_state == ROBOT_STOP || !RemoteControlIsOnline())
        {
            robot_state = ROBOT_STOP;
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        }
        if (switch_is_mid(rc_data[TEMP].rc.switch_right) && RemoteControlIsOnline())
        {
            robot_state = ROBOT_READY;
        }
    }
#else
    // ===================== 完整模式 =====================
    // 与参考实现一致: EmergencyHandler由RobotCMDTask中的丢控保护替代
#endif // CHASSIS_ONLY
    
}

/* ======================== 图传遥控器 → DJI遥控器 数据转写 ======================== */

/**
 * @brief 将图传遥控器数据转写到DJI遥控器数据结构, 实现统一控制路径
 * @note  switch_sw映射到switch_right
 *        switch_sw映射: LEFT(1)->UP(1), MID(2)->MID(3), RIGHT(3)->UP(1, 跟头模式)
 *        trigger覆盖dial为500
 */
static void image_rc_data_To_rc_data_t(void)
{
    rc_data[TEMP].rc.dial = image_rc_data[TEMP].rc.dial;
    rc_data[TEMP].rc.rocker_l_ = image_rc_data[TEMP].rc.rocker_l_x;
    rc_data[TEMP].rc.rocker_l1 = image_rc_data[TEMP].rc.rocker_l_y;
    rc_data[TEMP].rc.rocker_r_ = image_rc_data[TEMP].rc.rocker_r_x;
    rc_data[TEMP].rc.rocker_r1 = image_rc_data[TEMP].rc.rocker_r_y;
    switch (image_rc_data[TEMP].rc.switch_sw)
    {
    case IMAGE_SW_LEFT:  rc_data[TEMP].rc.switch_right = 1; break; // LEFT -> UP(跟头)
    case IMAGE_SW_MID:   rc_data[TEMP].rc.switch_right = 3; break; // MID  -> MID(云台底盘分离)
    case IMAGE_SW_RIGHT: rc_data[TEMP].rc.switch_right = 1; break; // RIGHT-> UP(跟头)
    }
    if (image_rc_data[TEMP].rc.trigger)
        rc_data[TEMP].rc.dial = 500;
}

/**
 * @brief 图传遥控器 - 键鼠控制模式 (switch_sw == LEFT)
 * @note  与原遥控器键鼠模式逻辑基本一致, 使用图传遥控器的键鼠数据
 */
static void ImageMouseKeySet()
{
    static uint8_t last_x_key_count;
    static uint8_t x_key_count_initialized;

    Limitshoot();
    chassis_speed_buff = CHASSIS_TRANSLATE_BASE_SPEED;
    chassis_cmd_send.fire_mode = vision_fire_mode;
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;

    // X键刷新UI
    uint8_t current_x_key_count = image_rc_data[TEMP].key_count[KEY_PRESS][Key_X];
    if (!x_key_count_initialized)
    {
        last_x_key_count = current_x_key_count;
        x_key_count_initialized = 1;
    }

    if (current_x_key_count != last_x_key_count)
    {
        chassis_cmd_send.ui_mode = UI_REFRESH;
        last_x_key_count = current_x_key_count;
    }
    else
    {
        chassis_cmd_send.ui_mode = UI_KEEP;
    }

    switch (image_rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 2)
    {
    case 0:  vision_fire_mode = FIRE_ON;  break;
    default: vision_fire_mode = FIRE_OFF; break;
    }
    chassis_cmd_send.fire_mode = vision_fire_mode;

    // 鼠标右键 + 视觉tracking=1 → 自瞄; 否则鼠标控制
    {
        uint8_t vision_tracking = 0;
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
    vision_tracking = (vision_recv_cache.tracking != 0);
#elif defined(VISION_USE_SERIALPORT)
        vision_tracking = (vision_recv_data_sp->shootStatus != 0);
#endif
        if (image_rc_data[TEMP].mouse.press_r && vision_tracking)
        {
            // 视觉自瞄
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
            {
                float interp_yaw;
                float interp_pitch;
                uint8_t interp_pitch_valid;

                GetVisionInterpolatedAim(&interp_yaw, &interp_pitch, &interp_pitch_valid);
                gimbal_cmd_send.yaw += loop_float_constrain(interp_yaw - gimbal_cmd_send.yaw, -45, 45);
                if (interp_pitch_valid)
                    gimbal_cmd_send.pitch = interp_pitch;
            }
#elif defined(VISION_USE_SERIALPORT)
            if (vision_recv_data_sp->yaw != 0 || vision_recv_data_sp->pitch != 0) {
                float aim_yaw = (float)vision_recv_data_sp->yaw / 100.0f;
                float aim_pitch = (float)vision_recv_data_sp->pitch / 100.0f;
                gimbal_cmd_send.yaw += loop_float_constrain(aim_yaw - gimbal_cmd_send.yaw, -45, 45);
                gimbal_cmd_send.pitch = aim_pitch;
            }
#endif
            if (gimbal_cmd_send.pitch <= PITCH_MIN_LIMIT)
                gimbal_cmd_send.pitch = PITCH_MIN_LIMIT;
            else if (gimbal_cmd_send.pitch >= PITCH_MAX_LIMIT)
                gimbal_cmd_send.pitch = PITCH_MAX_LIMIT;
            chassis_cmd_send.aim_mode = AIM_ON;
        }
        else
        {
            // 鼠标控制云台
            gimbal_cmd_send.yaw -= (float)image_rc_data[TEMP].mouse.x / 660 * 16;
            gimbal_cmd_send.pitch += (float)image_rc_data[TEMP].mouse.y / 660 * 16;
            if (gimbal_cmd_send.pitch <= PITCH_MIN_LIMIT)
                gimbal_cmd_send.pitch = PITCH_MIN_LIMIT;
            else if (gimbal_cmd_send.pitch >= PITCH_MAX_LIMIT)
                gimbal_cmd_send.pitch = PITCH_MAX_LIMIT;
            chassis_cmd_send.aim_mode = AIM_OFF;
        }
    }

    // E键发射模式
    switch (image_rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2)
    {
    case 0:  shoot_cmd_send.load_mode = LOAD_BURSTFIRE; break;
    default: shoot_cmd_send.load_mode = LOAD_3_BULLET; break;
    }
    // 鼠标左键射击
    switch (image_rc_data[TEMP].mouse.press_l)
    {
    case 0:  shoot_cmd_send.load_mode = LOAD_STOP; break;
    default: break;
    }

    // 视觉自瞄模式下自动开火
    if (image_rc_data[TEMP].mouse.press_r)
    {
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
        if (vision_fire_mode == FIRE_ON && vision_recv_cache.fire == 1)
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        else if (!image_rc_data[TEMP].mouse.press_l)
            shoot_cmd_send.load_mode = LOAD_STOP;
#elif defined(VISION_USE_SERIALPORT)
        if (vision_fire_mode == FIRE_ON && vision_recv_data_sp->shootStatus != 0)
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        else if (!image_rc_data[TEMP].mouse.press_l)
            shoot_cmd_send.load_mode = LOAD_STOP;
#endif
    }

    if (image_rc_data[TEMP].mouse.press_m)
    {
        shoot_cmd_send.load_mode = LOAD_REVERSE;
    }

    // F键摩擦轮
    switch (image_rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2)
    {
    case 0:  shoot_cmd_send.friction_mode = FRICTION_OFF; break;
    default: shoot_cmd_send.friction_mode = FRICTION_ON;  break;
    }
    // Q键底盘模式
    switch (image_rc_data[TEMP].key_count[KEY_PRESS][Key_Q] % 2)
    {
    case 0:  chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; break;
    default: chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;            break;
    }

    // Shift超级电容
    switch (image_rc_data[TEMP].key[KEY_PRESS].shift)
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

    if (chassis_cmd_send.dash_mode == DASH_ON)
    {
        chassis_speed_buff *= CHASSIS_TRANSLATE_DASH_RATIO;
    }

    chassis_cmd_send.vy = image_rc_data[TEMP].key[KEY_PRESS].w * chassis_speed_buff - image_rc_data[TEMP].key[KEY_PRESS].s * chassis_speed_buff;
    chassis_cmd_send.vx = image_rc_data[TEMP].key[KEY_PRESS].a * chassis_speed_buff - image_rc_data[TEMP].key[KEY_PRESS].d * chassis_speed_buff;

    // G键视觉目标
    switch (image_rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 3)
    {
    case 0:  vision_work_mode = VISION_MODE_AIM;        break;
    case 1:  vision_work_mode = VISION_MODE_SMALL_BUFF; break;
    default: vision_work_mode = VISION_MODE_BIG_BUFF;   break;
    }

    if (shoot_cmd_send.friction_mode == FRICTION_OFF && shoot_cmd_send.load_mode != LOAD_REVERSE)
        shoot_cmd_send.load_mode = LOAD_STOP;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
#ifdef CHASSIS_ONLY
    // ===================== CHASSIS_ONLY模式: 仅底盘控制任务 =====================
    // 获取底盘反馈
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    
    // 遥控器控制
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || switch_is_mid(rc_data[TEMP].rc.switch_left))
        RemoteControlSet();
    
    EmergencyHandler();
    
    // 发布底盘命令
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);

#else
    // ===================== 完整模式: 图传优先, 简化逻辑 =====================
    chassis_cmd_send.ui_mode = UI_KEEP;

    // 从其他应用获取回传数据
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    CalcOffsetAngle();
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
    RefreshVisionRecvCache();
    UpdateVisionInterpolation();
#endif

    // ======== 控制源选择: 图传优先, 两者都离线则失能 ========
    if (ImageRemoteIsOnline())
    {
        static uint8_t last_key_stop = 0;
        static uint8_t last_userkey_right = 0;
        uint8_t current_key_stop = image_rc_data[TEMP].rc.key_stop;
        uint8_t current_userkey_right = image_rc_data[TEMP].rc.userkey_right;

        // key_stop改为按键切换: 上升沿触发急停状态翻转
        if (current_key_stop && !last_key_stop)
        {
            stop_flag = !stop_flag;
        }
        last_key_stop = current_key_stop;

        // 右侧按键改为按键切换: 上升沿触发自旋状态翻转
        if (current_userkey_right && !last_userkey_right)
        {
            spin_flag = !spin_flag;
        }
        last_userkey_right = current_userkey_right;

        // 图传急停锁存开启时强制零力矩
        if (stop_flag)
        {
            rc_data[TEMP].rc.switch_right = 0;
        }
        else
        {
            // 转写图传数据到rc_data (摇杆 + switch_right)
            image_rc_data_To_rc_data_t();
            if (spin_flag)
            {
                rc_data[TEMP].rc.switch_right = 2;
            }
            // 模式选择: LEFT→键鼠(1), MID/RIGHT→摇杆(2)
            rc_data[TEMP].rc.switch_left =
                (image_rc_data[TEMP].rc.switch_sw == IMAGE_SW_LEFT) ? 1 : 2;
            // 拷贝鼠标键盘数据
            memcpy(&rc_data[TEMP].mouse, &image_rc_data[TEMP].mouse, sizeof(rc_data[TEMP].mouse));
            memcpy(&rc_data[TEMP].key, &image_rc_data[TEMP].key, sizeof(rc_data[TEMP].key));
        }
    }
    else if (!RemoteControlIsOnline())
    {
        // 两个遥控器都离线 → 失能
        rc_data[TEMP].rc.switch_right = 0;
    }
    // DJI在线且图传离线 → rc_data由sbus_to_rc自动填充, 无需处理

    // ======== 丢控保护: switch_right==0 → 全部零力矩 ========
    if (rc_data[TEMP].rc.switch_right == 0)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
    }
    else
    {
        // ======== 统一控制路径 ========
        if (switch_is_down(rc_data[TEMP].rc.switch_left) || switch_is_mid(rc_data[TEMP].rc.switch_left))
            RemoteControlSet();
        else if (switch_is_up(rc_data[TEMP].rc.switch_left))
            ImageMouseKeySet();
    }

    shoot_cmd_send.bullet_speed = chassis_fetch_data.bullet_speed;
    Detect_Color_e enemy_color = GetEnemyDetectColor(chassis_fetch_data.self_color);
    
    // ======================== 视觉发送数据准备 - 根据协议类型 ====== ==================
#if defined(VISION_USE_VCP) || defined(VISION_USE_UART)
    // VCP/UART协议 - 设置标志位和姿态角度
    const Vision_Send_s *vision_send_data = VisionGetSendData();
    VisionSetFlag(enemy_color, vision_work_mode, chassis_fetch_data.bullet_speed);
    VisionSetAltitude(loop_float_constrain(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle, -180.0f, 180.0f), 
                      gimbal_fetch_data.gimbal_imu_data.Pitch, 
                      gimbal_fetch_data.gimbal_imu_data.Roll);
    //VOFA(0, vision_recv_cache.aimYaw, vision_recv_cache.aimPitch,
    //vision_send_data->robotYaw, vision_send_data->robotPitch, vision_recv_cache.tracking);
#elif defined(VISION_USE_SERIALPORT)
    // SerialPort协议 - 准备发送数据
    vision_send_data_sp.startflag = '!';
    vision_send_data_sp.flag = 0x01;  // 状态标志
    
    // 转换角度为整数(放大100倍)
    vision_send_data_sp.yaw = (int32_t)(loop_float_constrain(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle, -180.0f, 180.0f) * 100.0f);
    vision_send_data_sp.pitch = (int16_t)(gimbal_fetch_data.gimbal_imu_data.Pitch * 100.0f);
    vision_send_data_sp.roll = (int16_t)(gimbal_fetch_data.gimbal_imu_data.Roll * 100.0f);
    
    // 颜色: 0=蓝色, 1=红色
    vision_send_data_sp.color = enemy_color;
    
    // 时间戳
    vision_send_data_sp.time_stamp = DWT_GetTimeline_ms();
    
    // 右击状态 (鼠标右键或其他准备状态, 转写模式下rc_data已包含图传数据)
    vision_send_data_sp.right_clicked = (rc_data[TEMP].mouse.press_r ? 1 : 0);
    
    // 用户时间偏差/弹速
    vision_send_data_sp.user_time_bias = (int8_t)chassis_fetch_data.bullet_speed;
    VOFA(0, loop_float_constrain((float)vision_recv_data_sp->yaw / 100.0f, -180.0f, 180.0f), (float)vision_recv_data_sp->pitch / 100.0f,
         (float)vision_send_data_sp.yaw / 100.0f, (float)vision_send_data_sp.pitch / 100.0f);
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
                         loop_float_constrain(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle, -180.0f, 180.0f), yaw_vel,
                         gimbal_fetch_data.gimbal_imu_data.Pitch, pitch_vel,
                         (float)chassis_fetch_data.bullet_speed, bullet_count);
           VOFA(0, loop_float_constrain(vision_recv_sp->yaw, -180.0f, 180.0f), vision_recv_sp->pitch,
               loop_float_constrain(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle, -180.0f, 180.0f),
               gimbal_fetch_data.gimbal_imu_data.Pitch);
    }0, 
#endif
    
    chassis_cmd_send.friction_mode = shoot_cmd_send.friction_mode;
    chassis_cmd_send.gimbal_mode = gimbal_cmd_send.gimbal_mode;
    chassis_cmd_send.load_mode = shoot_cmd_send.load_mode;
    chassis_cmd_send.pitch_angle = ((int)(gimbal_fetch_data.gimbal_imu_data.Pitch * 10 + 0.5)) / 10.0;

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
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
    
#endif // CHASSIS_ONLY
   
}
