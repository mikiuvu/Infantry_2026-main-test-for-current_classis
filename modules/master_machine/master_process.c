/**
 * @file master_process.c
 * @author neozng
 * @brief  视觉通信模块 - 负责与上位机(视觉处理程序)进行数据交互
 *         支持两种通信方式:
 *         1. VISION_USE_UART: 通过串口通信(使用seasky协议)
 *         2. VISION_USE_VCP: 通过USB虚拟串口通信
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
//#include "seasky_protocol.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "daemon.h"
#include "math.h"

#include "chassis.h"

#define VISION_AIM_YAW_ABS_MAX_DEG   720.0f
#define VISION_AIM_PITCH_ABS_MAX_DEG 180.0f

static uint8_t VisionAimValueValid(float aim_yaw, float aim_pitch)
{
    if (!isfinite(aim_yaw) || !isfinite(aim_pitch))
        return 0;

    if (fabsf(aim_yaw) > VISION_AIM_YAW_ABS_MAX_DEG)
        return 0;

    if (fabsf(aim_pitch) > VISION_AIM_PITCH_ABS_MAX_DEG)
        return 0;

    return 1;
}

// ======================== 静态全局变量 ========================
static Vision_Recv_s recv_data;              // 从视觉接收的数据(目标信息、控制指令等)
static Vision_Send_s send_data;              // 发送给视觉的数据(机器人姿态、颜色、弹速等)
static DaemonInstance *vision_daemon_instance; // 视觉通信看门狗实例,用于检测通信是否离线
// ======================== 公共接口函数 ========================

/**
 * @brief 设置视觉识别标志位
 * 
 * @param detect_color 要识别的颜色(红色/蓝色装甲板)
 * @param work_mode 工作模式(自瞄/小符/大符等)
 * @param bullet_speed 当前弹速(用于弹道解算)
 * @note 这些参数会被发送给视觉,用于调整识别算法和弹道计算
 */
void VisionSetFlag(Detect_Color_e detect_color, Work_Mode_e work_mode, float bullet_speed)
{
    // 旧版代码(已注释):
    // send_data.detect_color = detect_color;
    // send_data.mode = work_mode;
    send_data.muzzleSpeed = bullet_speed;

    // 使用memcpy将弹速转换为float类型后拷贝到发送数据结构体
    // 这样做是为了避免类型转换可能带来的精度损失
    //memcpy(&send_data.muzzleSpeed, &(float){bullet_speed}, sizeof(float));
    send_data.foeColor = detect_color; // 设置敌方颜色

}

/**
 * @brief 设置机器人姿态角度信息
 * @param yaw 机器人yaw轴角度(航向角)
 * @param pitch 机器人pitch轴角度(俯仰角)
 * @param roll 机器人roll轴角度(翻滚角,当前未使用)
 * @note 这些角度信息会发送给视觉,用于:
 *       1. 坐标系转换
 *       2. 预测目标运动
 *       3. 弹道补偿计算
 */
void VisionSetAltitude(float yaw, float pitch, float roll)
{
    // 使用memcpy拷贝浮点数,避免直接赋值可能的对齐问题
    memcpy(&send_data.robotYaw, &yaw, sizeof(float));
    memcpy(&send_data.robotPitch, &pitch, sizeof(float));
    
    // 旧版代码(已注释):
    // send_data.robotYaw = yaw;
    // send_data.robotPitch = pitch;
}

const Vision_Send_s *VisionGetSendData(void)
{
    return &send_data;
}

/**
 * @brief 设置导航状态数据
 * @param nav_data 导航状态值(0-255)
 * @note 此字段用于向上位机传输导航相关信息,
 *       如地图状态、导航模式等
 * @deprecated 协议修改 (2025-10-30): data字段已删除,此函数保留但不执行任何操作
 */
void VisionSetNavData(uint8_t nav_data)
{
    // send_data.data = nav_data; // 字段已删除
    UNUSED(nav_data); // 避免编译警告
}

// ======================== 串口通信模式 (UART) ========================
#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance; // 视觉通信串口实例

/**
 * @brief 接收解包回调函数 - 解析从视觉接收到的数据
 * @note  此函数会在bsp_usart.c的串口接收中断回调中被调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码的详细说明
 * 
 * @details 数据包格式(seasky协议):
 *          flag_register (16位):
 *          - [15:12] 4位: 开火模式 (fire_mode)
 *          - [11:8]  保留
 *          - [7:4]   4位: 目标状态 (target_state) - 是否发现目标/目标类型等
 *          - [3:0]   4位: 目标类型 (target_type) - 装甲板/基地/前哨站等
 */
static void DecodeVision()
{
    uint16_t flag_register; // 标志位寄存器,存储各种状态和模式
    
    DaemonReload(vision_daemon_instance); // 喂狗 - 
    
    // 从串口接收缓冲区解析数据
    // 参数: 接收缓冲区, 标志位指针, 数据起始地址(pitch角度)
    get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    
    recv_data.pitch *= (-1); // 
    
    // 解析标志位寄存器中的各个字段
    recv_data.fire_mode = ((flag_register & 0xf000) >> 12);  // 提取bit[15:12]: 开火模式
    recv_data.target_state = ((flag_register & 0x00f0) >> 4); // 提取bit[7:4]: 目标状态
    recv_data.target_type = ((flag_register & 0x000f));       // 提取bit[3:0]: 目标类型
    
    // TODO: 增加更详细的标志位解码逻辑
}

/**
 * @brief 视觉通信离线回调函数
 * @param id vision_usart_instance的地址(此处未使用)
 * 
 * @attention 解决HAL库串口DMA死锁问题:
 *            由于HAL库的设计缺陷,串口开启DMA接收后同时发送有概率出现
 *            __HAL_LOCK()导致的死锁,使得无法进入接收中断.
 *            解决方案: 通过daemon检测数据更新,若长时间未更新则判定为离线,
 *            重新调用串口初始化函数以恢复通信.
 * 
 * @note 此函数会在daemon.c的daemon task中被周期性检查并调用
 */
static void VisionOfflineCallback(void *id)
{
    USARTServiceInit(vision_usart_instance); // 重新初始化串口服务,恢复通信
}

/**
 * @brief 视觉通信模块初始化 (串口模式)
 * @param _handle 串口句柄(如 &huart1, &huart6 等)
 * @return Vision_Recv_s* 返回接收数据结构体指针,供其他模块读取视觉数据
 * 
 * @note 初始化流程:
 *       1. 配置串口参数和回调函数
 *       2. 注册串口实例
 *       3. 注册看门狗实例,用于监测通信状态
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
#ifdef VISION_USE_UART
    // 配置串口初始化参数
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;      // 设置接收回调函数
    conf.recv_buff_size = VISION_RECV_SIZE;   // 设置接收缓冲区大小
    conf.usart_handle = _handle;              // 设置串口句柄
    vision_usart_instance = USARTRegister(&conf); // 注册串口实例

    // 为视觉通信注册daemon看门狗,用于判断通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,    // 离线时调用的回调函数(重启串口)
        .owner_id = vision_usart_instance,    // 看门狗关联的实例ID
        .reload_count = 10,                   // 看门狗重载计数(超过此次数未喂狗则判定离线)
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);
#endif // VISION_USE_UART

    return &recv_data; // 返回接收数据指针
}

/**
 * @brief 向视觉发送数据 (串口模式)
 * 
 * @note  1. 使用seasky协议打包数据
 *        2. buff和txlen必须为static,才能保证在函数退出后不被释放,
 *           使得DMA能够正确完成发送(DMA是异步的)
 * 
 * @attention HAL库设计缺陷说明:
 *            - DMASTOP会同时停止发送和接收,导致再也无法进入接收中断
 *            - 可选方案1: 在发送完成中断中重新启动DMA接收(较复杂)
 *            - 当前方案: 使用IT(中断)方式发送,避免与DMA接收冲突
 *            - 若使用了daemon看门狗,也可以使用DMA发送(离线会自动重启)
 */
void VisionSend()
{
    // static变量 - 函数退出后仍保留,DMA传输过程中数据不会丢失
    static uint16_t flag_register;          // 标志位寄存器
    static uint8_t send_buff[VISION_SEND_SIZE]; // 发送缓冲区
    static uint16_t tx_len;                 // 实际发送长度
    
    // TODO: 增加设置flag_register的详细代码
    // 当前设置: [15:8]弹速, [7:0]颜色(1:蓝色, 0:红色)
    flag_register = bullet_speed << 8 | 0; 
    
    // 使用seasky协议将数据打包
    // 参数: 命令ID(0x02), 标志位, 数据起始地址(&yaw), 数据个数(3个float), 发送缓冲区, 长度指针
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    
    // 发送数据 - 使用DMA传输(如有daemon也可用DMA,否则建议用IT避免冲突)
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA);
}

#endif // VISION_USE_UART

// ======================== USB虚拟串口模式 (VCP) ========================
#ifdef VISION_USE_VCP

#include "bsp_usb.h"

static uint8_t *vis_recv_buff; // USB接收缓冲区指针

/**
 * @brief 视觉通信离线回调函数 (VCP模式)
 * @param id 未使用
 * 
 * @note VCP模式下的离线处理,清空接收数据并记录日志
 */
static void VisionOfflineCallback(void *id)
{
    // 离线时仅清空状态位,保留最后一帧角度用于诊断,控制层仍会依据tracking=0退出视觉控制
    recv_data.fire = 0;
    recv_data.tracking = NO_TARGET;
    LOGWARNING("[Vision] Vision communication offline!");
}

/**
 * @brief USB接收数据解码函数
 * @param recv_len 接收到的数据长度
 * 
 * @details 数据包格式验证:
 *          1. 检查数据长度是否匹配 Vision_Recv_s 结构体大小(26字节)
 *          2. 检查包头是否为 0xA5 (协议标识)
 *          3. 验证通过后拷贝数据到 recv_data
 *          4. 喂狗,重置看门狗计数器
 * 
 * @note 相比串口模式,VCP模式使用更简单的协议:
 *       - 固定包头: 0xA5
 *       - 数据结构: 直接传输 Vision_Recv_s 结构体
 *       - 无需复杂的协议解析
 */
static void DecodeVision(uint16_t recv_len)
{
    Vision_Recv_s temp_recv;
    // 数据包有效性检查:
    // 1. 长度必须等于 Vision_Recv_s 结构体大小(12字节)
    // 2. 包头必须是 0xA5 (单字节检查)
    if (recv_len != sizeof(Vision_Recv_s) || vis_recv_buff[0] != 0xA5)
    {
        return; // 数据包无效,直接返回
    }

    // 验证通过,拷贝数据到接收结构体
    memcpy(&temp_recv, vis_recv_buff, sizeof(Vision_Recv_s));

    if (!VisionAimValueValid(temp_recv.aimYaw, temp_recv.aimPitch))
    {
        return;
    }

    recv_data = temp_recv;
    
    // 根据实际坐标系需要,可以在这里对接收到的角度取反
    
    //recv_data.aimYaw *= -1.0f;      // Yaw角度取反
    //recv_data.aimPitch *= -1.0f;    // Pitch角度取反
    
    
    // 喂狗 - 重置看门狗计数,表明通信正常
    DaemonReload(vision_daemon_instance);
}

/**
 * @brief 视觉通信模块初始化 (USB虚拟串口模式)
 * @param _handle 串口句柄(此参数在VCP模式下未使用,仅为保持接口一致)
 * @return Vision_Recv_s* 返回接收数据结构体指针
 * 
 * @note 初始化流程:
 *       1. 配置USB接收回调函数
 *       2. 初始化USB模块,获取接收缓冲区指针
 *       3. 注册daemon看门狗实例
 *       4. 初始化发送数据的默认值
 * 
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 消除未使用参数警告 - VCP模式不需要串口句柄
    
    // 配置USB初始化参数
    USB_Init_Config_s conf = {0}; // 结构体清零初始化
    conf.rx_cbk = DecodeVision;   // 设置USB接收回调函数
    vis_recv_buff = USBInit(conf); // 初始化USB,返回接收缓冲区指针
    
    // 为视觉通信注册daemon看门狗,用于监测通信状态
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线回调
        .owner_id = NULL,                  // VCP模式暂不关联特定实例
        .reload_count = 10,                // 超过10个周期未收到数据则判定离线
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    // 初始化发送数据的默认值
    send_data.head = 0xA5;                // 包头为 0x00A5，小端序存储为 [A5 00]
    send_data.mode = VISION_MODE_AIM;     // 默认自瞄模式
    send_data.foeColor = COLOR_NONE;      // 颜色待设置
    // send_data.data = 0;                // [已删除] 导航状态字段

    return &recv_data; // 返回接收数据指针,供其他模块使用
}

/**
 * @brief 向视觉发送数据 (USB虚拟串口模式)
 * 
 * @details 发送数据包格式:
 *          - 包头 (head): 0xA5 0x00
 *          - 模式 (mode): 工作模式(自瞄/小符/大符)
 *          - 其余数据: Vision_Send_s 结构体的其他字段
 *                     (机器人姿态、弹速、颜色、导航状态等)
 * 
 * @note USB传输特点:
 *       1. 无需DMA,直接使用USB库函数发送
 *       2. 数据结构简单,直接传输整个结构体(17字节)
 *       3. 传输可靠,USB协议层自带错误检测和重传
 *       4. 无需担心发送接收冲突问题(全双工)
 * 
 * @attention 不要在此函数中硬编码 send_data.mode,
 *            应通过 VisionSetFlag() 函数设置
 */
void VisionSend()
{
    // 通过USB发送完整的数据结构体(17字节)
    // 包头已在VisionInit中初始化为0xA500
    // mode/foeColor/muzzleSpeed通过VisionSetFlag设置
    // robotYaw/robotPitch通过VisionSetAltitude设置
    // data字段可根据需要在应用层设置导航状态
    USBTransmit((void *)&send_data, sizeof(Vision_Send_s));
    
    // 注意: USB传输是异步的,但由于USB库内部有缓冲机制,
    //       不需要像串口DMA那样使用static变量保护数据
}

#endif // VISION_USE_VCP

// ======================== SP协议实现 ========================
#ifdef VISION_USE_SP

#include "bsp_usart.h"
#include "crc16.h"

static Vision_SP_Recv_s sp_recv_data;
static Vision_SP_Send_s sp_send_data;
static USARTInstance *sp_usart_instance;

/**
 * @brief CRC16计算
 */
static uint16_t SP_CRC16_Calculate(const uint8_t *data, uint16_t len)
{
    return crc_16(data, len);
}

/**
 * @brief CRC16校验
 * @return 1校验通过, 0失败
 */
static uint8_t SP_CRC16_Verify(const uint8_t *data, uint16_t len)
{
    if (len < 2) return 0;
    uint16_t crc_calc = SP_CRC16_Calculate(data, len - 2);
    uint16_t crc_recv = *(uint16_t *)(data + len - 2);
    return (crc_calc == crc_recv);
}

/**
 * @brief 串口接收回调,解析视觉数据
 */
static void DecodeSP(void)
{
    uint8_t *recv_buff = sp_usart_instance->recv_buff;
    
    // 检查帧头
    if (recv_buff[0] != 'S' || recv_buff[1] != 'P')
        return;
    
    // CRC校验
    if (!SP_CRC16_Verify(recv_buff, sizeof(Vision_SP_Recv_s)))
    {
        LOGWARNING("[SP] CRC fail");
        return;
    }
    
    memcpy(&sp_recv_data, recv_buff, sizeof(Vision_SP_Recv_s));
    DaemonReload(vision_daemon_instance);
}

/**
 * @brief 离线回调,重启串口
 */
static void VisionSPOfflineCallback(void *id)
{
    memset(&sp_recv_data, 0, sizeof(Vision_SP_Recv_s));
    LOGWARNING("[SP] offline");
    USARTServiceInit(sp_usart_instance);
}

/**
 * @brief SP协议初始化
 */
Vision_SP_Recv_s *VisionSPInit(UART_HandleTypeDef *_handle)
{
    init_crc16_tab();
    
    USART_Init_Config_s conf = {0};
    conf.module_callback = DecodeSP;
    conf.recv_buff_size = sizeof(Vision_SP_Recv_s);
    conf.usart_handle = _handle;
    sp_usart_instance = USARTRegister(&conf);
    
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionSPOfflineCallback,
        .owner_id = sp_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);
    
    sp_send_data.head[0] = 'S';
    sp_send_data.head[1] = 'P';
    
    return &sp_recv_data;
}

/**
 * @brief 设置发送数据
 */
void VisionSPSetState(uint8_t mode, float *q, float yaw, float yaw_vel, 
                      float pitch, float pitch_vel, float bullet_speed, uint16_t bullet_count)
{
    sp_send_data.mode = mode;
    if (q != NULL)
    {
        sp_send_data.q[0] = q[0];
        sp_send_data.q[1] = q[1];
        sp_send_data.q[2] = q[2];
        sp_send_data.q[3] = q[3];
    }
    sp_send_data.yaw = yaw;
    sp_send_data.yaw_vel = yaw_vel;
    sp_send_data.pitch = pitch;
    sp_send_data.pitch_vel = pitch_vel;
    sp_send_data.bullet_speed = bullet_speed;
    sp_send_data.bullet_count = bullet_count;
}

/**
 * @brief 发送数据,自动计算CRC
 */
void VisionSPSend(void)
{
    sp_send_data.crc16 = SP_CRC16_Calculate(
        (uint8_t *)&sp_send_data, 
        sizeof(Vision_SP_Send_s) - sizeof(sp_send_data.crc16)
    );
    USARTSend(sp_usart_instance, (uint8_t *)&sp_send_data, sizeof(Vision_SP_Send_s), USART_TRANSFER_DMA);
}

#endif // VISION_USE_SP
