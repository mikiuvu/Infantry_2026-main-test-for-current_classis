#include "transfer_image.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "bsp_log.h"

static Image_RC_ctrl_t image_rc_ctrl; // 图传遥控数据

static DaemonInstance *image_daemon_instance;
static USARTInstance *image_usart_instance;

/*数据解析*/
uint8_t enable_flag = 0;
static void ana_image_rc(const uint8_t *image_buf)
{
    if(image_buf[0]!=IMAGE_FIRST_SOF||image_buf[1]!=IMAGE_SECOND_SOF)
        {
            return;
        }
    /*遥控器摇杆通道数据解析*/
    image_rc_ctrl.rc.rocker_r_x = ((image_buf[2] | (image_buf[3] << 8))&0x07FF)-IMAGE_REMOTE_VALUE_OFFSET;//!< Channel 0
    image_rc_ctrl.rc.rocker_r_y = (((image_buf[3] >> 3) | (image_buf[4] << 5))&0x07FF)-IMAGE_REMOTE_VALUE_OFFSET;//!< Channel 1
    image_rc_ctrl.rc.rocker_l_y = (((image_buf[4] >> 6) | (image_buf[5] << 2) | (image_buf[6] << 10))&0x07FF)-IMAGE_REMOTE_VALUE_OFFSET;//!< Channel 2
    image_rc_ctrl.rc.rocker_l_x = (((image_buf[6] >> 1) | (image_buf[7] << 7))&0x07FF)-IMAGE_REMOTE_VALUE_OFFSET;//!< Channel 3
    /*遥控器开关按键数据解析*/
    image_rc_ctrl.rc.switch_sw = ((image_buf[7] >> 4) & 0x0003)+1;     //!< Switch 
    image_rc_ctrl.rc.key_stop = (image_buf[7]>>6)&0x01;      //!< 停止按键
    image_rc_ctrl.rc.userkey_right = (image_buf[7]>>7)&0x01; //!< 右侧按键
    image_rc_ctrl.rc.userkey_left = image_buf[8]&0x01;  //!< 左侧按键
    image_rc_ctrl.rc.dial = ((image_buf[8] >> 1 | (image_buf[9] << 7))&0x07FF)-IMAGE_REMOTE_VALUE_OFFSET;    // 侧边拨轮
    image_rc_ctrl.rc.trigger = (image_buf[9]>>4)&0x01;
    /*鼠标解析*/
    image_rc_ctrl.mouse.x = (image_buf[10] | (image_buf[11] << 8)); //!< Mouse X axis
    image_rc_ctrl.mouse.y = (image_buf[12] | (image_buf[13] << 8)); //!< Mouse Y axis
    image_rc_ctrl.mouse.z = (image_buf[14] | (image_buf[15] << 8)); //!< Mouse Z axis
    image_rc_ctrl.mouse.press_l = image_buf[16]&0x01;                 //!< Mouse Left Is Press
    image_rc_ctrl.mouse.press_r = (image_buf[16]>>2)&0x01;                 //!< Mouse Right Is Press
    image_rc_ctrl.mouse.press_m = (image_buf[16]>>4)&0x01;                 //!< Mouse Middle Is Press
    /*按键解析*/
    *(uint16_t *)&image_rc_ctrl.key = image_buf[17] | (image_buf[18] << 8);
    /*CRC16校验*/
    if(Verify_CRC16_Check_Sum(image_buf, IMAGE_FRAME_SIZE)==FALSE)
    {
        memset(&image_rc_ctrl,0,sizeof(Image_RC_ctrl_t));//校验失败清零
    }
}   

/**
 * @brief 对ana_image_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void ImageControlRxCallback()
{
    DaemonReload(image_daemon_instance);         // 先喂狗
    ana_image_rc(image_usart_instance->recv_buff); // 进行协议解析
    enable_flag = 1;
}

static void ImageLostCallback(void *id)
{
    memset(&image_rc_ctrl, 0, sizeof(image_rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(image_usart_instance); // 尝试重新启动接收
    enable_flag = 0;
    LOGWARNING("[image] image lost");
}

Image_RC_ctrl_t *TransferImageInit(UART_HandleTypeDef *transfer_image_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = ImageControlRxCallback;
    conf.usart_handle = transfer_image_usart_handle;
    conf.recv_buff_size = IMAGE_FRAME_SIZE;
    image_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 200, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = ImageLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    image_daemon_instance = DaemonRegister(&daemon_conf);

    return &image_rc_ctrl;
}