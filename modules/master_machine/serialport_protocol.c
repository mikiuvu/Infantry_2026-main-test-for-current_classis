/**
 * @file serialport_protocol.c
 * @author AI Assistant
 * @brief SerialPort通信协议实现 - 兼容HUST上位机
 * @version 1.0
 * @date 2025-10-28
 */

#include "serialport_protocol.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include <string.h>

// ======================== CRC8查找表 ========================
// 标准CRC8查找表,初始值0xFF,多项式0x31
static const uint8_t CRC8_TAB[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

#define CRC8_INIT 0xFF
#define RECV_BUFF_SIZE 32  // 接收缓冲区大小

// ======================== 静态全局变量 ========================
static SerialPort_Recv_s recv_data;              // 接收数据
static USARTInstance *serialport_usart;          // 串口实例
static DaemonInstance *serialport_daemon;        // 看门狗实例
static uint8_t pingpong_buffer[RECV_BUFF_SIZE * 2]; // 乒乓缓冲区
static uint16_t pingpong_write_idx = 0;          // 写入索引

// ======================== 内部函数 ========================

/**
 * @brief CRC8计算函数
 * @param data 数据指针
 * @param length 数据长度
 * @return uint8_t CRC8校验值
 */
uint8_t SerialPort_GetCRC8(uint8_t *data, uint16_t length) {
    uint8_t ucCRC8 = CRC8_INIT;
    uint8_t ucIndex;
    while (length--) {
        ucIndex = ucCRC8 ^ (*data++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return ucCRC8;
}

/**
 * @brief 16位整数大端序转换
 * @param val 小端序值
 * @return uint16_t 大端序值
 */
static inline uint16_t swap_uint16(uint16_t val) {
    return (val << 8) | (val >> 8);
}

/**
 * @brief 16位有符号整数大端序转换
 */
static inline int16_t swap_int16(int16_t val) {
    return (int16_t)swap_uint16((uint16_t)val);
}

/**
 * @brief 32位整数大端序转换
 * @param val 小端序值
 * @return uint32_t 大端序值
 */
static inline uint32_t swap_uint32(uint32_t val) {
    return ((val << 24) & 0xFF000000) |
           ((val << 8)  & 0x00FF0000) |
           ((val >> 8)  & 0x0000FF00) |
           ((val >> 24) & 0x000000FF);
}

/**
 * @brief 32位有符号整数大端序转换
 */
static inline int32_t swap_int32(int32_t val) {
    return (int32_t)swap_uint32((uint32_t)val);
}

/**
 * @brief 串口接收解码回调函数
 * @note 此函数会在串口接收中断中被调用
 * 
 * @details 数据包搜索和校验流程:
 *          1. 将新接收数据放入乒乓缓冲区
 *          2. 搜索包头 '!'
 *          3. 验证CRC8校验
 *          4. 解析数据(大端序转小端序)
 *          5. 喂狗
 */
static void DecodeSerialPort()
{
    uint8_t *recv_buff = serialport_usart->recv_buff;
    uint16_t recv_len = serialport_usart->recv_buff_size;
    
    // 将接收数据复制到乒乓缓冲区
    if (pingpong_write_idx + recv_len < RECV_BUFF_SIZE * 2) {
        memcpy(&pingpong_buffer[pingpong_write_idx], recv_buff, recv_len);
        pingpong_write_idx += recv_len;
    } else {
        // 缓冲区满,重置
        memcpy(pingpong_buffer, recv_buff, recv_len);
        pingpong_write_idx = recv_len;
    }
    
    // 搜索包头并解析数据
    for (int i = 0; i < (int)pingpong_write_idx - 15; i++) {
        if (pingpong_buffer[i] == '!') {
            // 验证CRC (计算前15字节,校验第16字节)
            uint8_t calc_crc = SerialPort_GetCRC8(&pingpong_buffer[i], 15);
            if (calc_crc == pingpong_buffer[i + 15]) {
                // CRC校验通过,解析数据
                recv_data.startflag = pingpong_buffer[i];
                recv_data.flag = pingpong_buffer[i + 1];
                
                // 大端序转小端序
                recv_data.pitch = swap_int16(*(int16_t*)&pingpong_buffer[i + 2]);
                recv_data.yaw = swap_int32(*(int32_t*)&pingpong_buffer[i + 4]);
                recv_data.shootStatus = pingpong_buffer[i + 8];
                recv_data.time_stamp = swap_uint32(*(uint32_t*)&pingpong_buffer[i + 9]);
                recv_data.state = pingpong_buffer[i + 13];
                recv_data.num = pingpong_buffer[i + 14];
                recv_data.crc = pingpong_buffer[i + 15];
                
                // 喂狗
                DaemonReload(serialport_daemon);
                
                // 移除已处理的数据
                int remaining = pingpong_write_idx - (i + 16);
                if (remaining > 0) {
                    memmove(pingpong_buffer, &pingpong_buffer[i + 16], remaining);
                    pingpong_write_idx = remaining;
                } else {
                    pingpong_write_idx = 0;
                }
                
                LOGINFO("[SerialPort] Recv: yaw=%d, pitch=%d, shoot=%d, num=%d",
                        recv_data.yaw, recv_data.pitch, recv_data.shootStatus, recv_data.num);
                return;
            }
        }
    }
    
    // 如果缓冲区有效数据超过一半,移动到前面
    if (pingpong_write_idx > RECV_BUFF_SIZE) {
        int keep_size = 16; // 保留最后16字节(可能是不完整包)
        memmove(pingpong_buffer, &pingpong_buffer[pingpong_write_idx - keep_size], keep_size);
        pingpong_write_idx = keep_size;
    }
}

/**
 * @brief 视觉通信离线回调函数
 * @param id 串口实例ID
 * 
 * @note 当超过设定时间未收到数据时,daemon会调用此函数
 */
static void SerialPortOfflineCallback(void *id) {
    // 清空接收数据
    memset(&recv_data, 0, sizeof(SerialPort_Recv_s));
    LOGWARNING("[SerialPort] Vision communication offline!");
}

// ======================== 公共接口函数 ========================

/**
 * @brief 初始化SerialPort通信模块
 */
SerialPort_Recv_s *SerialPortInit(UART_HandleTypeDef *_handle) {
    // 配置串口
    USART_Init_Config_s conf;
    conf.module_callback = DecodeSerialPort;
    conf.recv_buff_size = 16;  // 单次接收16字节(一个完整包)
    conf.usart_handle = _handle;
    serialport_usart = USARTRegister(&conf);
    
    // 注册daemon看门狗
    Daemon_Init_Config_s daemon_conf = {
        .callback = SerialPortOfflineCallback,
        .owner_id = serialport_usart,
        .reload_count = 10,  // 10次未喂狗则判定离线
    };
    serialport_daemon = DaemonRegister(&daemon_conf);
    
    // 清空缓冲区
    memset(pingpong_buffer, 0, sizeof(pingpong_buffer));
    pingpong_write_idx = 0;
    memset(&recv_data, 0, sizeof(SerialPort_Recv_s));
    
    LOGINFO("[SerialPort] SerialPort protocol initialized on UART");
    
    return &recv_data;
}

/**
 * @brief 向上位机发送数据
 */
void SerialPortSend(SerialPort_Send_s *send_data) {
    static uint8_t send_buff[18];
    
    // 填充数据 - 注意:上位机读取的字节顺序与结构体定义顺序不同!
    // 参考 SerialPort.cpp readData() 函数的解析顺序
    send_buff[0] = '!';  // 起始位
    send_buff[1] = send_data->flag;
    
    // pitch (2字节, 大端序) - 上位机: _data_tmp[2-3]
    int16_t pitch_be = swap_int16(send_data->pitch);
    memcpy(&send_buff[2], &pitch_be, 2);
    
    // yaw (4字节, 大端序) - 上位机: _data_tmp[4-7]
    int32_t yaw_be = swap_int32(send_data->yaw);
    memcpy(&send_buff[4], &yaw_be, 4);
    
    // color (1字节) - 上位机: _data_tmp[8]
    send_buff[8] = send_data->color;
    
    // time_stamp (4字节, 大端序) - 上位机: _data_tmp[9-12]
    uint32_t timestamp_be = swap_uint32(send_data->time_stamp);
    memcpy(&send_buff[9], &timestamp_be, 4);
    
    // roll (2字节, 大端序) - 上位机: _data_tmp[13-14]
    int16_t roll_be = swap_int16(send_data->roll);
    memcpy(&send_buff[13], &roll_be, 2);
    
    // right_clicked (1字节) - 上位机: _data_tmp[15]
    send_buff[15] = send_data->right_clicked;
    
    // user_time_bias (1字节) - 上位机: _data_tmp[16]
    send_buff[16] = (uint8_t)send_data->user_time_bias;
    
    // 计算并添加CRC8 (前17字节)
    send_buff[17] = SerialPort_GetCRC8(send_buff, 17);
    
    // 发送数据
    USARTSend(serialport_usart, send_buff, 18, USART_TRANSFER_DMA);
    
    // 调试日志(每50次打印一次,避免刷屏)
    static uint32_t send_count = 0;
    if (++send_count % 50 == 0) {
        LOGINFO("[SerialPort] Send #%lu: yaw=%d, pitch=%d, color=%d, crc=0x%02X",
                send_count, send_data->yaw, send_data->pitch, send_data->color, send_buff[17]);
    }
}
