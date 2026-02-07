/**
 * @file transfer_image.c
 * @brief 图传遥控器驱动, 适配2026年图传链路遥控器协议
 * @note  21字节帧: SOF(2) + 摇杆(5) + 开关/按键/拨轮(3) + 鼠标(7) + 键盘(2) + CRC16(2)
 */
#include "transfer_image.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "bsp_log.h"
#include "string.h"

static Image_RC_ctrl_t image_rc_ctrl[2]; // [TEMP]:当前数据, [LAST]:上一次数据
static uint8_t image_init_flag = 0;

static DaemonInstance *image_daemon_instance;
static USARTInstance *image_usart_instance;

/**
 * @brief 图传遥控器协议解析
 * @param image_buf 原始串口接收缓冲区
 */
static void ana_image_rc(const uint8_t *image_buf)
{
    if (image_buf[0] != IMAGE_FIRST_SOF || image_buf[1] != IMAGE_SECOND_SOF)
        return;

    /* CRC16校验 (先校验再解析, 避免使用错误数据) */
    if (Verify_CRC16_Check_Sum(image_buf, IMAGE_FRAME_SIZE) == FALSE)
        return;

    /* ---- 遥控器摇杆通道数据解析 ---- */
    image_rc_ctrl[TEMP].rc.rocker_r_x = ((image_buf[2] | (image_buf[3] << 8)) & 0x07FF) - IMAGE_REMOTE_VALUE_OFFSET;
    image_rc_ctrl[TEMP].rc.rocker_r_y = (((image_buf[3] >> 3) | (image_buf[4] << 5)) & 0x07FF) - IMAGE_REMOTE_VALUE_OFFSET;
    image_rc_ctrl[TEMP].rc.rocker_l_y = (((image_buf[4] >> 6) | (image_buf[5] << 2) | (image_buf[6] << 10)) & 0x07FF) - IMAGE_REMOTE_VALUE_OFFSET;
    image_rc_ctrl[TEMP].rc.rocker_l_x = (((image_buf[6] >> 1) | (image_buf[7] << 7)) & 0x07FF) - IMAGE_REMOTE_VALUE_OFFSET;

    /* ---- 开关/按键数据解析 ---- */
    image_rc_ctrl[TEMP].rc.switch_sw     = ((image_buf[7] >> 4) & 0x0003) + 1; // 存储值1/2/3对应UP/MID/DOWN
    image_rc_ctrl[TEMP].rc.key_stop      = (image_buf[7] >> 6) & 0x01;
    image_rc_ctrl[TEMP].rc.userkey_right = (image_buf[7] >> 7) & 0x01;
    image_rc_ctrl[TEMP].rc.userkey_left  = image_buf[8] & 0x01;
    image_rc_ctrl[TEMP].rc.dial          = ((image_buf[8] >> 1 | (image_buf[9] << 7)) & 0x07FF) - IMAGE_REMOTE_VALUE_OFFSET;
    image_rc_ctrl[TEMP].rc.trigger       = (image_buf[9] >> 4) & 0x01;

    /* ---- 鼠标解析 ---- */
    image_rc_ctrl[TEMP].mouse.x       = (int16_t)(image_buf[10] | (image_buf[11] << 8));
    image_rc_ctrl[TEMP].mouse.y       = (int16_t)(image_buf[12] | (image_buf[13] << 8));
    image_rc_ctrl[TEMP].mouse.z       = (int16_t)(image_buf[14] | (image_buf[15] << 8));
    image_rc_ctrl[TEMP].mouse.press_l = image_buf[16] & 0x01;
    image_rc_ctrl[TEMP].mouse.press_r = (image_buf[16] >> 2) & 0x01;
    image_rc_ctrl[TEMP].mouse.press_m = (image_buf[16] >> 4) & 0x01;

    /* ---- 键盘解析 (位域, 小端序) ---- */
    *(uint16_t *)&image_rc_ctrl[TEMP].key[KEY_PRESS] = image_buf[17] | (image_buf[18] << 8);

    /* ---- Ctrl/Shift 组合键处理 ---- */
    if (image_rc_ctrl[TEMP].key[KEY_PRESS].ctrl)
        image_rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = image_rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&image_rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));

    if (image_rc_ctrl[TEMP].key[KEY_PRESS].shift)
        image_rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = image_rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&image_rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    /* ---- 按键边沿计数 (上升沿检测, 与原遥控器逻辑一致) ---- */
    for (uint32_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        // 普通按键: 当前按下 && 上一次未按下 && 不是Ctrl/Shift组合
        if (((*(uint16_t *)&image_rc_ctrl[TEMP].key[KEY_PRESS] & j) == j) &&
            ((*(uint16_t *)&image_rc_ctrl[LAST].key[KEY_PRESS] & j) == 0) &&
            ((*(uint16_t *)&image_rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] & j) != j) &&
            ((*(uint16_t *)&image_rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] & j) != j))
        {
            image_rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
            if (image_rc_ctrl[TEMP].key_count[KEY_PRESS][i] >= 240)
                image_rc_ctrl[TEMP].key_count[KEY_PRESS][i] = 0;
        }
        // Ctrl组合键
        if (((*(uint16_t *)&image_rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] & j) == j) &&
            ((*(uint16_t *)&image_rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL] & j) == 0))
        {
            image_rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
            if (image_rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i] >= 240)
                image_rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i] = 0;
        }
        // Shift组合键
        if (((*(uint16_t *)&image_rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] & j) == j) &&
            ((*(uint16_t *)&image_rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT] & j) == 0))
        {
            image_rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
            if (image_rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i] >= 240)
                image_rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i] = 0;
        }
    }

    /* 保存上一次数据, 用于边沿检测 */
    memcpy(&image_rc_ctrl[LAST], &image_rc_ctrl[TEMP], sizeof(Image_RC_ctrl_t));
}

/**
 * @brief 串口接收回调
 */
static void ImageControlRxCallback()
{
    DaemonReload(image_daemon_instance);
    ana_image_rc(image_usart_instance->recv_buff);
}

/**
 * @brief 图传遥控器离线回调
 */
static void ImageLostCallback(void *id)
{
    memset(&image_rc_ctrl, 0, sizeof(image_rc_ctrl));
    USARTServiceInit(image_usart_instance); // 尝试重新启动接收
    LOGWARNING("[image] image remote lost");
}

Image_RC_ctrl_t *TransferImageInit(UART_HandleTypeDef *transfer_image_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = ImageControlRxCallback;
    conf.usart_handle = transfer_image_usart_handle;
    conf.recv_buff_size = IMAGE_FRAME_SIZE;
    image_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 200, // 200ms未收到数据视为离线, 图传接收频率约70Hz
        .callback = ImageLostCallback,
        .owner_id = NULL,
    };
    image_daemon_instance = DaemonRegister(&daemon_conf);

    image_init_flag = 1;
    return (Image_RC_ctrl_t *)&image_rc_ctrl;
}

uint8_t ImageRemoteIsOnline(void)
{
    if (image_init_flag)
        return DaemonIsOnline(image_daemon_instance);
    return 0;
}
