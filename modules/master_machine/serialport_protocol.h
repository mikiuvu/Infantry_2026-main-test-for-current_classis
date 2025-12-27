/**
 * @file serialport_protocol.h
 * @author AI Assistant
 * @brief SerialPort通信协议头文件 - 兼容HUST上位机SerialPort.cpp协议
 * @version 1.0
 * @date 2025-10-28
 * 
 * @copyright Copyright (c) 2025
 * 
 * @note 本协议使用CRC8校验和大端序,与上位机SerialPort.cpp完全兼容
 */

#ifndef SERIALPORT_PROTOCOL_H
#define SERIALPORT_PROTOCOL_H

#include "stdint.h"
#include "bsp_usart.h"

// ======================== 数据结构定义 ========================
// 1字节对齐,确保数据包格式正确
#pragma pack(1)

/**
 * @brief 发送给上位机视觉的数据包 (18字节)
 * @note 对应上位机 SerialPortData 结构体
 * 
 * @details 数据包格式:
 *          [0]    startflag    : '!' (0x21) 起始标志
 *          [1]    flag         : 状态标志位
 *          [2-5]  yaw          : Yaw角度 (int32, 大端序)
 *          [6-7]  pitch        : Pitch角度 (int16, 大端序)
 *          [8]    color        : 己方颜色 (0:蓝, 1:红)
 *          [9-12] time_stamp   : 时间戳 (uint32, 大端序)
 *          [13-14]roll         : Roll角度 (int16, 大端序)
 *          [15]   right_clicked: 右击状态(操作手准备就绪)
 *          [16]   user_time_bias:用户时间偏差/弹速
 *          [17]   crc          : CRC8校验码
 */
typedef struct {
    uint8_t startflag;      // 起始位 '!' (0x21)
    uint8_t flag;           // 状态标志位
    int32_t yaw;            // Yaw角度 (大端序)
    int16_t pitch;          // Pitch角度 (大端序)
    uint8_t color;          // 己方颜色
    uint32_t time_stamp;    // 时间戳 (大端序)
    int16_t roll;           // Roll角度 (大端序)
    uint8_t right_clicked;  // 右击状态
    int8_t user_time_bias;  // 用户时间偏差/弹速
    uint8_t crc;            // CRC8校验
} SerialPort_Send_s;

/**
 * @brief 从上位机视觉接收的数据包 (16字节)
 * @note 对应上位机 SerialPortWriteData 结构体
 * 
 * @details 数据包格式:
 *          [0]    startflag   : '!' (0x21) 起始标志
 *          [1]    flag        : 固定0x05
 *          [2-3]  pitch       : 目标Pitch角度 (int16, 大端序)
 *          [4-7]  yaw         : 目标Yaw角度 (int32, 大端序)
 *          [8]    shootStatus : 射击状态
 *          [9-12] time_stamp  : 时间戳 (uint32, 大端序)
 *          [13]   state       : 当前状态
 *          [14]   num         : 目标编号ID
 *          [15]   crc         : CRC8校验码
 */
typedef struct {
    uint8_t startflag;      // 起始位 '!' (0x21)
    uint8_t flag;           // 固定0x05
    int16_t pitch;          // 目标Pitch (大端序)
    int32_t yaw;            // 目标Yaw (大端序)
    uint8_t shootStatus;    // 射击状态
    uint32_t time_stamp;    // 时间戳 (大端序)
    uint8_t state;          // 状态
    uint8_t num;            // 目标编号
    uint8_t crc;            // CRC8校验
} SerialPort_Recv_s;

#pragma pack()

// ======================== 函数声明 ========================

/**
 * @brief 初始化SerialPort通信模块
 * @param _handle 串口句柄指针 (如 &huart1)
 * @return SerialPort_Recv_s* 返回接收数据结构体指针
 * 
 * @note 使用说明:
 *       1. 在robot_cmd.c的初始化函数中调用
 *       2. 返回的指针指向静态全局变量,无需释放
 *       3. 自动注册daemon看门狗,监测通信状态
 * 
 * @example
 *       SerialPort_Recv_s *sp_data = SerialPortInit(&huart1);
 */
SerialPort_Recv_s *SerialPortInit(UART_HandleTypeDef *_handle);

/**
 * @brief 向上位机发送数据
 * @param send_data 待发送的数据结构体指针
 * 
 * @note 函数会自动:
 *       1. 转换为大端序
 *       2. 计算并添加CRC8校验
 *       3. 通过串口发送
 */
void SerialPortSend(SerialPort_Send_s *send_data);

/**
 * @brief 计算CRC8校验码
 * @param data 数据指针
 * @param length 数据长度(不含CRC字节)
 * @return uint8_t 计算得到的CRC8值
 * 
 * @note 使用标准CRC8查找表算法,初始值0xFF
 */
uint8_t SerialPort_GetCRC8(uint8_t *data, uint16_t length);

#endif // SERIALPORT_PROTOCOL_H
