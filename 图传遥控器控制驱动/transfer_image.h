#include "main.h"
#include "usart.h"
#include "stdint.h"
#include "remote_control.h"

/*按键按下判断*/
#define KEY_RELEASED 0
#define KEY_PRESSED 1

/* 图传模块宏定义 */
#define IMAGE_FRAME_SIZE 21u//26年图传接收端每14ms发送21字节数据
#define IMAGE_FIRST_SOF 0xA9
#define IMAGE_SECOND_SOF 0x53
/* 图传遥控器通道数值范围定义 */
#define IMAGE_REMOTE_VALUE_MIN 364
#define IMAGE_REMOTE_VALUE_OFFSET 1024
#define IMAGE_REMOTE_VALUE_MAX 1684

/*不同档位开关*/
#define IMAGE_SW_UP 0
#define IMAGE_SW_MID 1
#define IMAGE_SW_DOWN 2


typedef struct
{
    struct 
    {
        int16_t rocker_r_x; // 右摇杆X轴
        int16_t rocker_r_y; // 右摇杆Y轴
        int16_t rocker_l_x; // 左摇杆X轴
        int16_t rocker_l_y; // 左摇杆Y轴
        uint8_t switch_sw; // 右侧开关
        uint8_t key_stop;      // 停止按键
        uint8_t userkey_right; // 右侧按键
        uint8_t userkey_left;  // 左侧按键
        int16_t dial;    // 侧边拨轮
        uint8_t trigger; // 扳机键
    }rc;
    struct 
    {
        int16_t x;
        int16_t y;
        int16_t z;//滚轮
        uint8_t press_l;
        uint8_t press_r;
        uint8_t press_m;
    }mouse;
    Key_t key; 
    uint16_t crc16_check;
}Image_RC_ctrl_t;
    
Image_RC_ctrl_t *TransferImageInit(UART_HandleTypeDef *transfer_image_usart_handle);
