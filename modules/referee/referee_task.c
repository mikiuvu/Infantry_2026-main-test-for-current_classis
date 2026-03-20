/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"

#include "rm_referee.h"
#include "ui_interface.h"
#include "ui_g.h"

static Referee_Interactive_info_t *Interactive_data;
static referee_info_t *referee_recv_info;
uint8_t UI_Seq;
static uint8_t ui_initialized;
static ui_mode_e last_ui_mode = UI_KEEP;
static uint8_t ui_refresh_repeat_count;

#define UI_REFRESH_REPEAT_TIMES 3u

referee_info_t *Referee_Interactive_init(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle);
    Interactive_data = UI_data;
    ui_bind_g(referee_recv_info, Interactive_data);
    return referee_recv_info;
}

void My_UI_init(void)
{
    if (Interactive_data == NULL || referee_recv_info == NULL)
    {
        return;
    }

    ui_bind_g(referee_recv_info, Interactive_data);
    ui_init_g();
    ui_initialized = 1;
}

void Referee_Interactive_task(void)
{
    if (Interactive_data == NULL || referee_recv_info == NULL)
    {
        return;
    }

    if (Interactive_data->ui_mode == UI_REFRESH && last_ui_mode != UI_REFRESH)
    {
        ui_refresh_repeat_count = UI_REFRESH_REPEAT_TIMES;
    }

    if (!ui_initialized || ui_refresh_repeat_count > 0)
    {
        My_UI_init();
        if (ui_refresh_repeat_count > 0)
        {
            ui_refresh_repeat_count--;
        }
    }
    else
    {
        ui_update_g();
    }

    last_ui_mode = Interactive_data->ui_mode;
}
