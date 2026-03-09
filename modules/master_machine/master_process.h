/**
 * @file master_process.h
 * @brief 视觉通信模块头文件 - 定义与上位机视觉系统通信的数据结构和接口
 * @note 本模块支持多种通信方式:
 *       1. VISION_USE_UART: 串口通信(使用seasky协议)
 *       2. VISION_USE_VCP: USB虚拟串口通信
 *       3. VISION_USE_SP: SP协议(gimbal_aim_vision上位机)
 * @attention 使用#pragma pack(1)进行1字节对齐,确保通信数据包的正确性
 */

#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

// ======================== 枚举类型定义 ========================
// 使用1字节对齐,确保通信协议的数据包大小和结构正确
#pragma pack(1)

/**
 * @brief 开火模式枚举
 * @note 视觉返回的开火控制指令,用于控制发射机构
 */
typedef enum
{
	NO_FIRE = 0,      ///< 不开火 - 视觉未识别到目标或目标未锁定
	AUTO_FIRE = 1,    ///< 自动开火 - 视觉已锁定目标,允许自动击发
	AUTO_AIM = 2      ///< 自瞄模式 - 视觉辅助瞄准,但不自动开火
} Fire_Mode_e;

/**
 * @brief 目标状态枚举
 * @note 视觉识别目标的锁定状态,用于判断是否可以开火
 */
typedef enum
{
	NO_TARGET = 0,          ///< 未发现目标 - 视野中没有可识别的装甲板
	TARGET_CONVERGING = 1,  ///< 目标收敛中 - 正在跟踪目标,但尚未稳定锁定
	READY_TO_FIRE = 2       ///< 准备开火 - 目标已稳定锁定,可以击发
} Target_State_e;

/**
 * @brief 目标类型枚举
 * @note 识别到的目标机器人类型,对应RoboMaster比赛中的不同兵种
 */
typedef enum
{
	NO_TARGET_NUM = 0,  ///< 无目标编号
	HERO1 = 1,          ///< 英雄机器人(1号)
	ENGINEER2 = 2,      ///< 工程机器人(2号)
	INFANTRY3 = 3,      ///< 步兵机器人(3号)
	INFANTRY4 = 4,      ///< 步兵机器人(4号)
	INFANTRY5 = 5,      ///< 步兵机器人(5号)
	OUTPOST = 6,        ///< 前哨站
	SENTRY = 7,         ///< 哨兵机器人(7号)
	BASE = 8            ///< 基地
} Target_Type_e;

/**
 * @brief 识别颜色枚举
 * @note 设定要识别的敌方颜色(红色或蓝色装甲板灯条)
 * @attention 根据裁判系统的己方颜色自动设置
 */
typedef enum
{
	COLOR_NONE = -1,  ///< 未设置颜色(无效值)
	COLOR_RED = 1,    ///< 识别红色装甲板(己方为蓝色)
	COLOR_BLUE = 0,   ///< 识别蓝色装甲板(己方为红色)
} Detect_Color_e;

/**
 * @brief 视觉工作模式枚举
 * @note 不同的工作模式会影响视觉算法的识别策略和弹道解算
 */
typedef enum
{
	VISION_MODE_AIM = 0,       ///< 自瞄模式 - 识别装甲板,进行弹道解算
	VISION_MODE_SMALL_BUFF = 1,///< 小能量机关模式 - 识别扇叶,预测击打点
	VISION_MODE_BIG_BUFF = 2   ///< 大能量机关模式 - 识别R标,跟踪运动轨迹
} Work_Mode_e;

/**
 * @brief 弹丸速度枚举
 * @note 用于视觉进行弹道解算,不同的弹速对应不同的弹道模型
 * @attention 弹速必须与实际测量值一致,否则会影响自瞄精度
 */
typedef enum
{
	BULLET_SPEED_NONE = 0, ///< 未设置弹速(无效值)
	BIG_AMU_10 = 10,       ///< 大弹丸(42mm) 10m/s - 英雄机器人低速
	SMALL_AMU_15 = 15,     ///< 小弹丸(17mm) 15m/s - 步兵标准弹速
	BIG_AMU_16 = 16,       ///< 大弹丸(42mm) 16m/s - 英雄机器人标准弹速
	SMALL_AMU_18 = 18,     ///< 小弹丸(17mm) 18m/s - 步兵高射速弹速
	SMALL_AMU_30 = 30,     ///< 小弹丸(17mm) 30m/s - 哨兵/步兵高级弹速
} Bullet_Speed_e;

#pragma pack() // 恢复默认对齐方式

// ======================== 通信数据结构定义 ========================
// 1字节对齐,确保结构体大小和字段顺序符合通信协议
#pragma pack(1)

/**
 * @brief 发送给视觉的数据包结构体
 * @note 包含机器人当前状态信息,用于视觉进行坐标转换和弹道解算
 * 
 * @details 数据包内容:
 *          - 机器人姿态(Yaw/Pitch): 用于坐标系转换
 *          - 敌方颜色: 决定识别红色还是蓝色装甲板
 *          - 弹丸速度: 用于弹道模型计算
 *          - 工作模式: 自瞄/小符/大符
 * 
 * @attention 此结构体会通过串口或USB发送给上位机视觉程序
 * 
 * @note 协议修改 (2025-10-30):
 *       删除了 data 字段以匹配上位机 inf_robotMsg 结构 (16字节)
 */
typedef struct //发送视觉数据结构体
{
	uint16_t head;              ///< 包头标识(0xA500即0xA5 0x00) - 用于数据包校验
	uint8_t mode;               ///< 工作模式 - 自瞄/小符/大符(Work_Mode_e)
	Detect_Color_e foeColor;    ///< 敌方颜色 - 0:蓝色, 1:红色
	float robotYaw;             ///< 机器人Yaw轴角度(°) - 云台航向角
	float robotPitch;           ///< 机器人Pitch轴角度(°) - 云台俯仰角
	float muzzleSpeed;          ///< 枪口弹丸速度(m/s) - 用于弹道解算
} Vision_Send_s;

/**
 * @brief 从视觉接收的数据包结构体
 * @note 包含视觉识别结果和控制指令,用于调整云台姿态和控制开火
 * 
 * @details 数据包内容:
 *          - 开火标志: 是否允许开火
 *          - 跟踪标志: 是否正在跟踪目标
 *          - 目标角度: 云台应该转到的目标位置
 * 
 * @attention 接收到此数据后,云台会根据目标角度进行调整,
 *            并根据开火标志决定是否击发
 * 
 * @note 协议修改 (2025-10-30): 
 *       删除了 vx/vy/wz 字段以匹配上位机 inf_visionMsg 结构 (12字节)
 */
typedef struct	//接受视觉数据结构体
{
	uint16_t head;      ///< 包头标识(0xA5) - 用于数据包校验
	uint8_t fire;       ///< 开火标志 - 0:不开火, 1:允许开火
	uint8_t tracking;   ///< 跟踪标志 - 0:未跟踪, 1:正在跟踪目标
	float aimYaw;       ///< 目标Yaw角度(°) - 云台应转到的航向角
	float aimPitch;     ///< 目标Pitch角度(°) - 云台应转到的俯仰角
} Vision_Recv_s;
/**
 * @brief SP协议发送结构体,对应gimbal_aim_vision上位机的GimbalToVision
 */
typedef struct
{
	uint8_t head[2];        ///< 帧头 'S','P'
	uint8_t mode;           ///< 工作模式: 0空闲 1自瞄 2小符 3大符
	float q[4];             ///< 四元数(wxyz顺序)
	float yaw;              ///< 当前yaw角度(°)
	float yaw_vel;          ///< yaw角速度(°/s)
	float pitch;            ///< 当前pitch角度(°)
	float pitch_vel;        ///< pitch角速度(°/s)
	float bullet_speed;     ///< 弹速(m/s)
	uint16_t bullet_count;  ///< 子弹累计发射数
	uint16_t crc16;         ///< CRC16校验
} Vision_SP_Send_s;

/**
 * @brief SP协议接收结构体,对应gimbal_aim_vision上位机的VisionToGimbal
 */
typedef struct
{
	uint8_t head[2];        ///< 帧头 'S','P'
	uint8_t mode;           ///< 控制模式: 0不控制 1控制不开火 2控制+开火
	float yaw;              ///< 目标yaw角度(°)
	float yaw_vel;          ///< yaw速度前馈(°/s)
	float yaw_acc;          ///< yaw加速度前馈(°/s²)
	float pitch;            ///< 目标pitch角度(°)
	float pitch_vel;        ///< pitch速度前馈(°/s)
	float pitch_acc;        ///< pitch加速度前馈(°/s²)
	uint16_t crc16;         ///< CRC16校验
} Vision_SP_Recv_s;
// ======================== 通信参数宏定义 ========================
/**
 * @brief 接收数据包大小
 * @note 当前固定为Vision_Recv_s结构体大小
 *       包头(2) + fire(1) + tracking(1) + aimYaw(4) + aimPitch(4) = 12字节
 */
#define VISION_RECV_SIZE sizeof(Vision_Recv_s)

/**
 * @brief 发送数据包大小
 * @note 当前固定为Vision_Send_s结构体大小
 *       包头(2) + mode(1) + foeColor(1) + robotYaw(4) + robotPitch(4) + muzzleSpeed(4) + data(1) = 17字节
 */
#define VISION_SEND_SIZE sizeof(Vision_Send_s)
 

// ======================== 公共接口函数声明 ========================

/**
 * @brief 初始化视觉通信模块
 * @param _handle 用于视觉通信的串口句柄指针
 *                - C板: 一般使用USART1(丝印标注为USART2,4针接口)
 *                - 其他板: 根据实际硬件连接选择串口
 * @return Vision_Recv_s* 返回视觉接收数据结构体的指针
 * 
 * @note 使用说明:
 *       1. 根据robot_def.h中的宏定义选择通信方式:
 *          - VISION_USE_UART: 串口通信(需要传入有效的串口句柄)
 *          - VISION_USE_VCP: USB虚拟串口(串口句柄参数无效,可传NULL)
 *       2. 初始化会自动注册daemon看门狗,用于监测通信状态
 *       3. 返回的指针指向静态全局变量,无需手动释放
 * 
 * @example
 *       Vision_Recv_s *vision_data = VisionInit(&huart1);
 *       // 之后可通过vision_data访问视觉返回的数据
 *       if (vision_data->fire) {
 *           // 执行开火动作
 *       }
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 向视觉发送数据
 * @note 此函数会将Vision_Send_s结构体中的数据发送给上位机视觉程序
 * 
 * @details 发送内容包括:
 *          - 机器人当前姿态(Yaw/Pitch角度)
 *          - 敌方颜色(红/蓝)
 *          - 当前弹速
 *          - 工作模式(自瞄/能量机关)
 * 
 * @attention 使用注意事项:
 *            1. 发送前需先调用VisionSetFlag()和VisionSetAltitude()设置数据
 *            2. 建议在控制任务中周期性调用(如1kHz或500Hz)
 *            3. 串口模式使用DMA传输,函数会立即返回(非阻塞)
 *            4. USB模式使用库函数传输,传输可靠
 * 
 * @example
 *       // 在云台任务中周期性调用
 *       VisionSetAltitude(gimbal_yaw, gimbal_pitch, 0);
 *       VisionSetFlag(enemy_color, VISION_MODE_AIM, SMALL_AMU_15);
 *       VisionSend();
 */
void VisionSend();

/**
 * @brief 设置视觉识别标志位和参数
 * @param Detect_color 要识别的敌方颜色
 *                     - COLOR_RED: 识别红色装甲板(己方为蓝色)
 *                     - COLOR_BLUE: 识别蓝色装甲板(己方为红色)
 * @param work_mode 视觉工作模式
 *                  - VISION_MODE_AIM: 自瞄模式
 *                  - VISION_MODE_SMALL_BUFF: 小能量机关模式
 *                  - VISION_MODE_BIG_BUFF: 大能量机关模式
 * @param bullet_speed 当前弹丸速度(用于弹道解算)
 *                     - 步兵: SMALL_AMU_15 或 SMALL_AMU_18
 *                     - 英雄: BIG_AMU_16
 *                     - 哨兵: SMALL_AMU_30
 * 
 * @note 此函数会将参数保存到Vision_Send_s结构体中,
 *       在调用VisionSend()时一起发送给上位机
 * 
 * @attention 重要提示:
 *            1. 颜色应根据裁判系统返回的己方颜色自动设置
 *            2. 弹速必须与实际测量值一致,误差会影响自瞄精度
 *            3. 工作模式切换需要视觉程序支持
 * 
 * @example
 *       // 根据裁判系统设置敌方颜色
 *       Detect_Color_e enemy_color = (referee_data->self_color == COLOR_RED) 
 *                                     ? COLOR_BLUE : COLOR_RED;
 *       VisionSetFlag(enemy_color, VISION_MODE_AIM, SMALL_AMU_15);
 */
void VisionSetFlag(Detect_Color_e Detect_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置机器人当前姿态角度
 * @param yaw 机器人Yaw轴角度(航向角,单位:度)
 *            - 范围: -180° ~ +180° 或 0° ~ 360°(取决于陀螺仪输出)
 *            - 定义: 以正前方为0°,逆时针为正(符合右手系)
 * @param pitch 机器人Pitch轴角度(俯仰角,单位:度)
 *              - 范围: 通常 -30° ~ +30°(取决于机械限位)
 *              - 定义: 水平为0°,向上为正,向下为负
 * @param roll 机器人Roll轴角度(翻滚角,单位:度)
 *             - 当前未使用,传入0即可
 *             - 预留接口,用于未来可能的姿态补偿
 * 
 * @note 姿态角度用途:
 *       1. 坐标系转换: 将相机坐标系的目标位置转换到世界坐标系
 *       2. 运动预测: 结合目标运动状态预测未来位置
 *       3. 弹道补偿: 根据云台姿态调整弹道解算参数
 * 
 * @attention 重要提示:
 *            1. 姿态数据应来自陀螺仪或IMU,确保实时性和准确性
 *            2. 角度单位必须为度(°),不是弧度(rad)
 *            3. 坐标系定义必须与视觉程序约定一致
 *            4. 建议在云台控制任务中实时更新
 * 
 * @example
 *       // 在云台任务中实时更新姿态
 *       float gimbal_yaw = gimbal_imu_data->yaw;
 *       float gimbal_pitch = gimbal_imu_data->pitch;
 *       VisionSetAltitude(gimbal_yaw, gimbal_pitch, 0);
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

/**
 * @brief 获取当前实际发送给视觉的数据缓存(只读)
 * @return Vision_Send_s* 指向内部发送缓存的指针
 */
const Vision_Send_s *VisionGetSendData(void);

/**
 * @brief 设置导航状态数据
 * @param nav_data 导航状态值(0-255)
 *                 - 可用于传输地图状态、导航模式等信息
 *                 - 具体含义由上位机定义
 * 
 * @note 此字段用于与上位机交互导航相关信息,
 *       具体用途可根据实际需求定义
 * 
 * @example
 *       // 发送当前导航状态
 *       VisionSetNavData(current_nav_state);
 */
void VisionSetNavData(uint8_t nav_data);

/**
 * @brief SP协议初始化
 * @param _handle 串口句柄
 * @return Vision_SP_Recv_s* 接收数据指针
 */
Vision_SP_Recv_s *VisionSPInit(UART_HandleTypeDef *_handle);

/**
 * @brief SP协议发送数据
 */
void VisionSPSend(void);

/**
 * @brief 设置SP协议发送状态
 * @param mode 工作模式(0空闲/1自瞄/2小符/3大符)
 * @param q 四元数指针(wxyz,4个float)
 * @param yaw yaw角度
 * @param yaw_vel yaw角速度
 * @param pitch pitch角度
 * @param pitch_vel pitch角速度
 * @param bullet_speed 弹速
 * @param bullet_count 子弹计数
 */
void VisionSPSetState(uint8_t mode, float *q, float yaw, float yaw_vel, 
                      float pitch, float pitch_vel, float bullet_speed, uint16_t bullet_count);

#endif // !MASTER_PROCESS_
