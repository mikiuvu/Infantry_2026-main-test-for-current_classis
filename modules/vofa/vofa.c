/**
 * @file vofa.c
 * @brief VOFA+调试模块 - 支持多绑定组
 * @note 接收格式: #ch=value! 例如 #0=3.14! #1=-1.5!
 * @note 上位机中VOFA+配置:
 *       比如：命令0：#1=%f! 代表通道1,%f为控件值占位符
 */

#include "vofa.h"
#include "bsp_usart.h"
#include <string.h>
#include <stdlib.h>

/* ======================== 绑定组结构体 ======================== */
typedef struct {
    float **ptrs;
    uint8_t start_ch;
    uint8_t count;
} VofaBindGroup_t;

/* ======================== 静态变量 ======================== */
static float vofa_tx_data[VOFA_MAX_CH];
static uint8_t vofa_tx_buf[VOFA_MAX_CH * 4 + 4];

static VofaBindGroup_t vofa_bind_groups[VOFA_MAX_BIND_GROUPS];
static uint8_t vofa_bind_group_count = 0;
static USARTInstance *vofa_uart_instance = NULL;

static const uint8_t VOFA_TAIL[4] = {0x00, 0x00, 0x80, 0x7f};

/* ======================== 解析函数 ======================== */
static void VofaParseAndUpdate(uint8_t *raw_buf, uint16_t max_len)
{
    char *buf = (char *)raw_buf;
    
    for (uint16_t i = 0; i < max_len && buf[i] != '\0'; i++) {
        if (buf[i] != '#') continue;
        
        uint16_t end = i + 1;
        while (end < max_len && buf[end] != '!' && buf[end] != '\0') end++;
        if (end >= max_len || buf[end] != '!') continue;
        
        char *p = &buf[i + 1];
        int ch = atoi(p);
        
        while (p < &buf[end] && *p != '=') p++;
        if (*p != '=') continue;
        p++;
        
        float value = (float)atof(p);
        
        // 遍历所有绑定组，找到匹配的通道
        for (uint8_t g = 0; g < vofa_bind_group_count; g++) {
            VofaBindGroup_t *grp = &vofa_bind_groups[g];
            int idx = ch - grp->start_ch;
            if (idx >= 0 && idx < grp->count && grp->ptrs[idx] != NULL) {
                *grp->ptrs[idx] = value;
                break;  // 一个通道只更新一次
            }
        }
        
        i = end;
    }
}

/* ======================== 回调函数 ======================== */
static void VofaRxCallback(void)
{
    if (vofa_uart_instance == NULL) return;
    VofaParseAndUpdate(vofa_uart_instance->recv_buff, 
                       vofa_uart_instance->recv_buff_size);
}

/* ======================== 公开API ======================== */

void VofaSend(UART_HandleTypeDef *uart, uint8_t start_ch, float *data, uint8_t len)
{
    if (uart == NULL) uart = &VOFA_UART;
    if (uart->gState != HAL_UART_STATE_READY) return;
    if (start_ch >= VOFA_MAX_CH || len == 0) return;
    if (start_ch + len > VOFA_MAX_CH) len = VOFA_MAX_CH - start_ch;
    
    // 填充数据到对应通道
    memcpy(&vofa_tx_data[start_ch], data, len * sizeof(float));
    
    // 打包发送
    uint8_t max_ch = start_ch + len;
    uint16_t tx_len = max_ch * sizeof(float);
    memcpy(vofa_tx_buf, vofa_tx_data, tx_len);
    memcpy(&vofa_tx_buf[tx_len], VOFA_TAIL, 4);
    
    HAL_UART_Transmit_DMA(uart, vofa_tx_buf, tx_len + 4);
}

void VofaBind(UART_HandleTypeDef *uart, uint8_t start_ch, float **ptrs, uint8_t len)
{
    if (vofa_bind_group_count >= VOFA_MAX_BIND_GROUPS) return;
    
    // 添加绑定组
    VofaBindGroup_t *grp = &vofa_bind_groups[vofa_bind_group_count++];
    grp->ptrs = ptrs;
    grp->start_ch = start_ch;
    grp->count = len;
    
    // 首次调用时注册串口
    if (vofa_uart_instance == NULL) {
        USART_Init_Config_s conf = {
            .usart_handle = (uart != NULL) ? uart : &VOFA_UART,
            .recv_buff_size = VOFA_MAX_CH * 4 + 4,
            .module_callback = VofaRxCallback,
        };
        vofa_uart_instance = USARTRegister(&conf);
    }
}
