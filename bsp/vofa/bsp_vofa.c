/**
 * @file bsp_vofa.c
 * @brief VOFA+调试模块实现 - 极简版（收发一体）
 * @note 接收功能通过标准USARTRegister()注册，无需修改bsp_usart.c
 */

#include "bsp_vofa.h"
#include "bsp_usart.h"
#include "bsp_dwt.h"
#include <string.h>

/* ======================== 静态变量 - 发送 ======================== */
static uint8_t vofa_inited = 0;
static uint32_t vofa_last_time = 0;
static uint8_t vofa_tx_buf[VOFA_MAX_CH * 4 + 4];  // float数据 + 帧尾

/* ======================== 静态变量 - 接收 ======================== */
static USARTInstance *vofa_usart_instance = NULL; // 串口实例
static float vofa_rx_data[VOFA_MAX_CH];           // 解析后的float数组
static float **vofa_bind_ptrs = NULL;             // 绑定的指针数组
static uint8_t vofa_bind_count = 0;               // 绑定的指针个数
static uint8_t vofa_rx_count = 0;                 // 接收到的通道数
static volatile uint8_t vofa_updated = 0;         // 新数据标志

// JustFloat帧尾
static const uint8_t VOFA_TAIL[4] = {0x00, 0x00, 0x80, 0x7f};

/* ======================== 内部函数 ======================== */

static void VofaAutoInit(void)
{
    if (!vofa_inited) {
        vofa_inited = 1;
        vofa_last_time = 0;
    }
}

static void VofaTransmit(float *data, uint8_t len)
{
    if (len == 0 || len > VOFA_MAX_CH) return;
    if (VOFA_UART.gState != HAL_UART_STATE_READY) return;
    
    uint16_t idx = 0;
    memcpy(&vofa_tx_buf[idx], data, len * sizeof(float));
    idx += len * sizeof(float);
    memcpy(&vofa_tx_buf[idx], VOFA_TAIL, 4);
    idx += 4;
    
    HAL_UART_Transmit_DMA(&VOFA_UART, vofa_tx_buf, idx);
}

static uint8_t VofaParse(uint8_t *buf, uint16_t len)
{
    if (len < 8) return 0;
    if (memcmp(&buf[len - 4], VOFA_TAIL, 4) != 0) return 0;
    
    uint8_t count = (len - 4) / 4;
    if (count > VOFA_MAX_CH) count = VOFA_MAX_CH;
    
    memcpy(vofa_rx_data, buf, count * sizeof(float));
    return count;
}

/**
 * @brief 串口接收回调（通过USARTRegister注册，框架自动调用）
 */
static void VofaRxCallback(void)
{
    if (vofa_usart_instance == NULL) return;
    
    // 解析数据 - 需要计算实际接收长度
    // 由于框架不传递长度，我们需要搜索帧尾
    uint8_t *buf = vofa_usart_instance->recv_buff;
    uint16_t max_len = vofa_usart_instance->recv_buff_size;
    uint16_t len = 0;
    
    // 寻找帧尾位置
    for (uint16_t i = 4; i <= max_len; i++) {
        if (memcmp(&buf[i - 4], VOFA_TAIL, 4) == 0) {
            len = i;
            break;
        }
    }
    
    if (len == 0) return;  // 未找到有效帧
    
    uint8_t count = VofaParse(buf, len);
    if (count == 0) return;
    
    vofa_rx_count = count;
    vofa_updated = 1;
    
    // 自动更新绑定的变量
    if (vofa_bind_ptrs != NULL) {
        uint8_t update_count = (count < vofa_bind_count) ? count : vofa_bind_count;
        for (uint8_t i = 0; i < update_count; i++) {
            if (vofa_bind_ptrs[i] != NULL) {
                *vofa_bind_ptrs[i] = vofa_rx_data[i];
            }
        }
    }
}

static void VofaRxInit(void)
{
    if (vofa_usart_instance != NULL) return;
    
    USART_Init_Config_s vofa_usart_conf = {
        .usart_handle = &VOFA_UART,
        .recv_buff_size = VOFA_MAX_CH * 4 + 4,
        .module_callback = VofaRxCallback,
    };
    
    vofa_usart_instance = USARTRegister(&vofa_usart_conf);
}

/* ======================== 公开API ======================== */

void VofaSend(float *data, uint8_t len)
{
    VofaAutoInit();
    uint32_t now = DWT_GetTimeline_ms();
    if (now - vofa_last_time < VOFA_MIN_INTERVAL) return;
    vofa_last_time = now;
    VofaTransmit(data, len);
}

void VofaSendForce(float *data, uint8_t len)
{
    VofaAutoInit();
    vofa_last_time = DWT_GetTimeline_ms();
    VofaTransmit(data, len);
}

void VofaBind(float **ptrs, uint8_t len)
{
    vofa_bind_ptrs = ptrs;
    vofa_bind_count = len;
    VofaRxInit();
}

uint8_t VofaIsUpdated(void)
{
    if (vofa_updated) {
        vofa_updated = 0;
        return 1;
    }
    return 0;
}

float VofaGetChannel(uint8_t idx)
{
    if (idx < VOFA_MAX_CH) return vofa_rx_data[idx];
    return 0.0f;
}

uint8_t VofaGetCount(void)
{
    return vofa_rx_count;
}
