/**
 * @file transfer_image.h
 * @brief 图传遥控器驱动头文件, 适配2026年图传链路遥控器
 * @note  通过在 robot_def.h 中定义 USE_IMAGE_REMOTE 宏切换图传/原遥控器
 *        图传遥控器使用 UART6, 原遥控器使用 USART3(DBUS)
 */
#ifndef TRANSFER_IMAGE_H
#define TRANSFER_IMAGE_H

#include "main.h"
#include "usart.h"
#include "stdint.h"
#include "remote_control.h" // 复用 Key_t 和按键宏定义(KEY_PRESS, Key_W 等)

/* ======================== 图传模块宏定义 ======================== */
#define IMAGE_FRAME_SIZE         21u   // 26年图传接收端每14ms发送21字节数据
#define IMAGE_FIRST_SOF          0xA9
#define IMAGE_SECOND_SOF         0x53

/* 图传遥控器通道数值范围定义 */
#define IMAGE_REMOTE_VALUE_MIN    364
#define IMAGE_REMOTE_VALUE_OFFSET 1024
#define IMAGE_REMOTE_VALUE_MAX    1684

/* 开关档位定义 (协议解析中已+1, 存储值为1/2/3) */
#define IMAGE_SW_UP   1
#define IMAGE_SW_MID  2
#define IMAGE_SW_DOWN 3

/* 开关判断宏 */
#define img_switch_is_up(s)   ((s) == IMAGE_SW_UP)
#define img_switch_is_mid(s)  ((s) == IMAGE_SW_MID)
#define img_switch_is_down(s) ((s) == IMAGE_SW_DOWN)

/* ======================== 数据结构定义 ======================== */
typedef struct
{
    struct
    {
        int16_t rocker_r_x;    // 右摇杆X轴
        int16_t rocker_r_y;    // 右摇杆Y轴
        int16_t rocker_l_x;    // 左摇杆X轴
        int16_t rocker_l_y;    // 左摇杆Y轴
        uint8_t switch_sw;     // 三档开关 (IMAGE_SW_UP/MID/DOWN)
        uint8_t key_stop;      // 停止按键
        uint8_t userkey_right; // 右侧按键
        uint8_t userkey_left;  // 左侧按键
        int16_t dial;          // 侧边拨轮
        uint8_t trigger;       // 扳机键
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;       // 滚轮
        uint8_t press_l;
        uint8_t press_r;
        uint8_t press_m; // 中键
    } mouse;
    Key_t key[3];             // [KEY_PRESS], [KEY_PRESS_WITH_CTRL], [KEY_PRESS_WITH_SHIFT]
    uint8_t key_count[3][16]; // 按键边沿计数, 用于切换/长按检测
} Image_RC_ctrl_t;

/**
 * @brief 初始化图传遥控器, 注册到指定串口
 * @param transfer_image_usart_handle 串口句柄 (推荐 &huart6)
 * @return Image_RC_ctrl_t* 遥控器数据指针 (2元素数组, [TEMP]当前 / [LAST]上一次)
 */
Image_RC_ctrl_t *TransferImageInit(UART_HandleTypeDef *transfer_image_usart_handle);

/**
 * @brief 检查图传遥控器是否在线
 * @return uint8_t 1:在线 0:离线
 */
uint8_t ImageRemoteIsOnline(void);

#endif // TRANSFER_IMAGE_H
