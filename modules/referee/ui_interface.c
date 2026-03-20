#include "ui_interface.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "crc_ref.h"

#define UI_CACHE_MAX_FIGURES 32
#define UI_CACHE_MAX_STRINGS 16
#define UI_DEFAULT_MAX_SEND_COUNT 3

#pragma pack(1)
typedef struct
{
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    Graph_Data_t data[1];
    uint16_t frametail;
} UI_graph_1_frame_t;

typedef struct
{
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    Graph_Data_t data[2];
    uint16_t frametail;
} UI_graph_2_frame_t;

typedef struct
{
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    Graph_Data_t data[5];
    uint16_t frametail;
} UI_graph_5_frame_t;

typedef struct
{
    xFrameHeader FrameHeader;
    uint16_t CmdID;
    ext_student_interactive_header_data_t datahead;
    Graph_Data_t data[7];
    uint16_t frametail;
} UI_graph_7_frame_t;
#pragma pack()

static referee_id_t *ui_active_id;

static Graph_Data_t ui_now_figures[UI_CACHE_MAX_FIGURES];
static uint8_t ui_dirty_figure[UI_CACHE_MAX_FIGURES];
static uint8_t ui_figure_used[UI_CACHE_MAX_FIGURES];
static String_Data_t ui_now_strings[UI_CACHE_MAX_STRINGS];
static uint8_t ui_dirty_string[UI_CACHE_MAX_STRINGS];
static uint8_t ui_string_used[UI_CACHE_MAX_STRINGS];

static uint8_t ui_max_send_count[UI_CACHE_MAX_FIGURES + UI_CACHE_MAX_STRINGS] = {
    [0 ... (UI_CACHE_MAX_FIGURES + UI_CACHE_MAX_STRINGS - 1)] = UI_DEFAULT_MAX_SEND_COUNT,
};

static ui_delete_frame_t ui_delete_frame;
static ui_string_frame_t ui_char_frame;
static UI_graph_1_frame_t ui_graph_1_frame;
static UI_graph_2_frame_t ui_graph_2_frame;
static UI_graph_5_frame_t ui_graph_5_frame;
static UI_graph_7_frame_t ui_graph_7_frame;

static void UI_FillFrameHeader(xFrameHeader *header, uint16_t data_length)
{
    header->SOF = REFEREE_SOF;
    header->DataLength = data_length;
    header->Seq = UI_Seq;
    header->CRC8 = Get_CRC8_Check_Sum((uint8_t *)header, LEN_CRC8, 0xFF);
}

static void UI_FillInteractiveDataHead(ext_student_interactive_header_data_t *datahead, uint16_t data_cmd_id)
{
    datahead->data_cmd_id = data_cmd_id;
    datahead->receiver_ID = ui_active_id->Cilent_ID;
    datahead->sender_ID = ui_active_id->Robot_ID;
}

static void UI_UpdateActiveID(referee_id_t *_id)
{
    if (_id != NULL)
    {
        ui_active_id = _id;
    }
}

static void UI_SetGraphicName(uint8_t graphic_name[3], const char graphname[3])
{
    memset(graphic_name, 0, 3);
    for (int i = 0; i < 3 && graphname[i] != '\0'; i++)
    {
        graphic_name[2 - i] = (uint8_t)graphname[i];
    }
}

static int UI_FindGraphSlot(const uint8_t graphic_name[3])
{
    for (int i = 0; i < UI_CACHE_MAX_FIGURES; i++)
    {
        if (ui_figure_used[i] && memcmp(ui_now_figures[i].graphic_name, graphic_name, 3) == 0)
        {
            return i;
        }
    }
    return -1;
}

static int UI_AllocGraphSlot(void)
{
    for (int i = 0; i < UI_CACHE_MAX_FIGURES; i++)
    {
        if (!ui_figure_used[i])
        {
            ui_figure_used[i] = 1;
            return i;
        }
    }
    return -1;
}

static int UI_FindStringSlot(const uint8_t graphic_name[3])
{
    for (int i = 0; i < UI_CACHE_MAX_STRINGS; i++)
    {
        if (ui_string_used[i] &&
            memcmp(ui_now_strings[i].Graph_Control.graphic_name, graphic_name, 3) == 0)
        {
            return i;
        }
    }
    return -1;
}

static int UI_AllocStringSlot(void)
{
    for (int i = 0; i < UI_CACHE_MAX_STRINGS; i++)
    {
        if (!ui_string_used[i])
        {
            ui_string_used[i] = 1;
            return i;
        }
    }
    return -1;
}

static void UI_FillGraphFrameHeader(xFrameHeader *header, uint16_t data_cmd_id, uint16_t data_length)
{
    UI_FillFrameHeader(header, data_length);
    (void)data_cmd_id;
}

void ui_proc_1_frame(void *msg)
{
    UI_graph_1_frame_t *frame = (UI_graph_1_frame_t *)msg;
    uint8_t payload_length = Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw;

    if (ui_active_id == NULL || frame == NULL)
    {
        return;
    }

    UI_FillFrameHeader(&frame->FrameHeader, payload_length);
    frame->CmdID = ID_student_interactive;
    UI_FillInteractiveDataHead(&frame->datahead, UI_Data_ID_Draw1);
    frame->frametail = Get_CRC16_Check_Sum((uint8_t *)frame, LEN_HEADER + LEN_CMDID + payload_length, 0xFFFF);
}

void ui_proc_2_frame(void *msg)
{
    UI_graph_2_frame_t *frame = (UI_graph_2_frame_t *)msg;
    uint8_t payload_length = Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * 2;

    if (ui_active_id == NULL || frame == NULL)
    {
        return;
    }

    UI_FillFrameHeader(&frame->FrameHeader, payload_length);
    frame->CmdID = ID_student_interactive;
    UI_FillInteractiveDataHead(&frame->datahead, UI_Data_ID_Draw2);
    frame->frametail = Get_CRC16_Check_Sum((uint8_t *)frame, LEN_HEADER + LEN_CMDID + payload_length, 0xFFFF);
}

void ui_proc_5_frame(void *msg)
{
    UI_graph_5_frame_t *frame = (UI_graph_5_frame_t *)msg;
    uint8_t payload_length = Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * 5;

    if (ui_active_id == NULL || frame == NULL)
    {
        return;
    }

    UI_FillFrameHeader(&frame->FrameHeader, payload_length);
    frame->CmdID = ID_student_interactive;
    UI_FillInteractiveDataHead(&frame->datahead, UI_Data_ID_Draw5);
    frame->frametail = Get_CRC16_Check_Sum((uint8_t *)frame, LEN_HEADER + LEN_CMDID + payload_length, 0xFFFF);
}

void ui_proc_7_frame(void *msg)
{
    UI_graph_7_frame_t *frame = (UI_graph_7_frame_t *)msg;
    uint8_t payload_length = Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * 7;

    if (ui_active_id == NULL || frame == NULL)
    {
        return;
    }

    UI_FillFrameHeader(&frame->FrameHeader, payload_length);
    frame->CmdID = ID_student_interactive;
    UI_FillInteractiveDataHead(&frame->datahead, UI_Data_ID_Draw7);
    frame->frametail = Get_CRC16_Check_Sum((uint8_t *)frame, LEN_HEADER + LEN_CMDID + payload_length, 0xFFFF);
}

void ui_proc_string_frame(void *msg)
{
    ui_string_frame_t *frame = (ui_string_frame_t *)msg;
    uint8_t payload_length = Interactive_Data_LEN_Head + UI_Operate_LEN_DrawChar;

    if (ui_active_id == NULL || frame == NULL)
    {
        return;
    }

    UI_FillFrameHeader(&frame->FrameHeader, payload_length);
    frame->CmdID = ID_student_interactive;
    UI_FillInteractiveDataHead(&frame->datahead, UI_Data_ID_DrawChar);
    frame->frametail = Get_CRC16_Check_Sum((uint8_t *)frame, LEN_HEADER + LEN_CMDID + payload_length, 0xFFFF);
}

void ui_proc_delete_frame(void *msg)
{
    ui_delete_frame_t *frame = (ui_delete_frame_t *)msg;
    uint8_t payload_length = Interactive_Data_LEN_Head + UI_Operate_LEN_Del;

    if (ui_active_id == NULL || frame == NULL)
    {
        return;
    }

    UI_FillFrameHeader(&frame->FrameHeader, payload_length);
    frame->CmdID = ID_student_interactive;
    UI_FillInteractiveDataHead(&frame->datahead, UI_Data_ID_Del);
    frame->frametail = Get_CRC16_Check_Sum((uint8_t *)frame, LEN_HEADER + LEN_CMDID + payload_length, 0xFFFF);
}

static void UI_SendGraphFrame(Graph_Data_t *graph_data, uint8_t graph_count, uint16_t data_cmd_id)
{
    uint8_t payload_length;

    if (ui_active_id == NULL || graph_count == 0)
    {
        return;
    }

    payload_length = Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * graph_count;
    switch (graph_count)
    {
    case 1:
        UI_FillGraphFrameHeader(&ui_graph_1_frame.FrameHeader, data_cmd_id, payload_length);
        ui_graph_1_frame.CmdID = ID_student_interactive;
        ui_graph_1_frame.datahead.data_cmd_id = data_cmd_id;
        ui_graph_1_frame.datahead.receiver_ID = ui_active_id->Cilent_ID;
        ui_graph_1_frame.datahead.sender_ID = ui_active_id->Robot_ID;
        memcpy(ui_graph_1_frame.data, graph_data, sizeof(Graph_Data_t));
        ui_graph_1_frame.frametail = Get_CRC16_Check_Sum(
            (uint8_t *)&ui_graph_1_frame,
            LEN_HEADER + LEN_CMDID + payload_length,
            0xFFFF);
        RefereeSend((uint8_t *)&ui_graph_1_frame,
                    LEN_HEADER + LEN_CMDID + payload_length + LEN_TAIL);
        break;
    case 2:
        UI_FillGraphFrameHeader(&ui_graph_2_frame.FrameHeader, data_cmd_id, payload_length);
        ui_graph_2_frame.CmdID = ID_student_interactive;
        ui_graph_2_frame.datahead.data_cmd_id = data_cmd_id;
        ui_graph_2_frame.datahead.receiver_ID = ui_active_id->Cilent_ID;
        ui_graph_2_frame.datahead.sender_ID = ui_active_id->Robot_ID;
        memcpy(ui_graph_2_frame.data, graph_data, sizeof(Graph_Data_t) * 2);
        ui_graph_2_frame.frametail = Get_CRC16_Check_Sum(
            (uint8_t *)&ui_graph_2_frame,
            LEN_HEADER + LEN_CMDID + payload_length,
            0xFFFF);
        RefereeSend((uint8_t *)&ui_graph_2_frame,
                    LEN_HEADER + LEN_CMDID + payload_length + LEN_TAIL);
        break;
    case 5:
        UI_FillGraphFrameHeader(&ui_graph_5_frame.FrameHeader, data_cmd_id, payload_length);
        ui_graph_5_frame.CmdID = ID_student_interactive;
        ui_graph_5_frame.datahead.data_cmd_id = data_cmd_id;
        ui_graph_5_frame.datahead.receiver_ID = ui_active_id->Cilent_ID;
        ui_graph_5_frame.datahead.sender_ID = ui_active_id->Robot_ID;
        memcpy(ui_graph_5_frame.data, graph_data, sizeof(Graph_Data_t) * 5);
        ui_graph_5_frame.frametail = Get_CRC16_Check_Sum(
            (uint8_t *)&ui_graph_5_frame,
            LEN_HEADER + LEN_CMDID + payload_length,
            0xFFFF);
        RefereeSend((uint8_t *)&ui_graph_5_frame,
                    LEN_HEADER + LEN_CMDID + payload_length + LEN_TAIL);
        break;
    case 7:
        UI_FillGraphFrameHeader(&ui_graph_7_frame.FrameHeader, data_cmd_id, payload_length);
        ui_graph_7_frame.CmdID = ID_student_interactive;
        ui_graph_7_frame.datahead.data_cmd_id = data_cmd_id;
        ui_graph_7_frame.datahead.receiver_ID = ui_active_id->Cilent_ID;
        ui_graph_7_frame.datahead.sender_ID = ui_active_id->Robot_ID;
        memcpy(ui_graph_7_frame.data, graph_data, sizeof(Graph_Data_t) * 7);
        ui_graph_7_frame.frametail = Get_CRC16_Check_Sum(
            (uint8_t *)&ui_graph_7_frame,
            LEN_HEADER + LEN_CMDID + payload_length,
            0xFFFF);
        RefereeSend((uint8_t *)&ui_graph_7_frame,
                    LEN_HEADER + LEN_CMDID + payload_length + LEN_TAIL);
        break;
    default:
        break;
    }

    UI_Seq++;
}

static void UI_SendStringFrame(const String_Data_t *string_data)
{
    uint8_t payload_length = Interactive_Data_LEN_Head + UI_Operate_LEN_DrawChar;

    if (ui_active_id == NULL || string_data == NULL)
    {
        return;
    }

    ui_char_frame.FrameHeader.SOF = REFEREE_SOF;
    ui_char_frame.FrameHeader.DataLength = payload_length;
    ui_char_frame.FrameHeader.Seq = UI_Seq;
    ui_char_frame.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&ui_char_frame.FrameHeader, LEN_CRC8, 0xFF);
    ui_char_frame.CmdID = ID_student_interactive;
    ui_char_frame.datahead.data_cmd_id = UI_Data_ID_DrawChar;
    ui_char_frame.datahead.receiver_ID = ui_active_id->Cilent_ID;
    ui_char_frame.datahead.sender_ID = ui_active_id->Robot_ID;
    ui_char_frame.option = *string_data;
    ui_char_frame.frametail = Get_CRC16_Check_Sum(
        (uint8_t *)&ui_char_frame,
        LEN_HEADER + LEN_CMDID + payload_length,
        0xFFFF);
    RefereeSend((uint8_t *)&ui_char_frame,
                LEN_HEADER + LEN_CMDID + payload_length + LEN_TAIL);
    UI_Seq++;
}

void ui_delete_layer(const uint8_t delete_type, const uint8_t layer)
{
    if (ui_active_id == NULL)
    {
        return;
    }

    ui_delete_frame.delete_type = delete_type;
    ui_delete_frame.layer = layer;
    ui_proc_delete_frame(&ui_delete_frame);
    RefereeSend((uint8_t *)&ui_delete_frame,
                LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_Del + LEN_TAIL);
    UI_Seq++;
}

void ui_scan_and_send(const ui_interface_figure_t *ui_now_figures, uint8_t *ui_dirty_figure,
                      const ui_interface_string_t *ui_now_strings, uint8_t *ui_dirty_string,
                      int total_figures, int total_strings)
{
    if (ui_active_id == NULL)
    {
        return;
    }

    if (total_figures > 0)
    {
        int total_figure = 0;
        int i;

        for (i = 0; i < total_figures; i++)
        {
            if (ui_dirty_figure[i] > 0)
            {
                total_figure++;
            }
        }

        for (i = 0; i < total_figures;)
        {
            int pack_size;
            int now_cap = 0;
            int remain_size;
            int j;

            while (i < total_figures && ui_dirty_figure[i] == 0)
            {
                i++;
            }
            if (i >= total_figures)
            {
                break;
            }

            remain_size = total_figure;
            if (remain_size > 5)
            {
                pack_size = 7;
            }
            else if (remain_size > 2)
            {
                pack_size = 5;
            }
            else if (remain_size > 1)
            {
                pack_size = 2;
            }
            else
            {
                pack_size = 1;
            }

            for (j = i; j < total_figures && now_cap < pack_size; j++)
            {
                if (ui_dirty_figure[j] > 0)
                {
                    if (pack_size == 7)
                    {
                        ui_graph_7_frame.data[now_cap] = ui_now_figures[j];
                    }
                    else if (pack_size == 5)
                    {
                        ui_graph_5_frame.data[now_cap] = ui_now_figures[j];
                    }
                    else if (pack_size == 2)
                    {
                        ui_graph_2_frame.data[now_cap] = ui_now_figures[j];
                    }
                    else
                    {
                        ui_graph_1_frame.data[now_cap] = ui_now_figures[j];
                    }
                    ui_dirty_figure[j]--;
                    now_cap++;
                    total_figure--;
                }
            }

            if (pack_size == 7)
            {
                ui_proc_7_frame(&ui_graph_7_frame);
                RefereeSend((uint8_t *)&ui_graph_7_frame,
                            LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * 7 + LEN_TAIL);
            }
            else if (pack_size == 5)
            {
                ui_proc_5_frame(&ui_graph_5_frame);
                RefereeSend((uint8_t *)&ui_graph_5_frame,
                            LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * 5 + LEN_TAIL);
            }
            else if (pack_size == 2)
            {
                ui_proc_2_frame(&ui_graph_2_frame);
                RefereeSend((uint8_t *)&ui_graph_2_frame,
                            LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw * 2 + LEN_TAIL);
            }
            else
            {
                ui_proc_1_frame(&ui_graph_1_frame);
                RefereeSend((uint8_t *)&ui_graph_1_frame,
                            LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_PerDraw + LEN_TAIL);
            }
            UI_Seq++;
            i = j;
        }
    }

    if (total_strings > 0)
    {
        int i;
        for (i = 0; i < total_strings; i++)
        {
            if (ui_dirty_string[i] > 0)
            {
                ui_char_frame.option = ui_now_strings[i];
                ui_proc_string_frame(&ui_char_frame);
                RefereeSend((uint8_t *)&ui_char_frame,
                            LEN_HEADER + LEN_CMDID + Interactive_Data_LEN_Head + UI_Operate_LEN_DrawChar + LEN_TAIL);
                ui_dirty_string[i]--;
                UI_Seq++;
            }
        }
    }
}

static void UI_ScanAndSend(void)
{
    Graph_Data_t graph_buffer[7];

    for (int sent_count = 0;;)
    {
        int total_dirty = 0;
        int pack_size = 0;

        for (int i = 0; i < UI_CACHE_MAX_FIGURES; i++)
        {
            if (ui_dirty_figure[i] > 0)
            {
                total_dirty++;
            }
        }

        if (total_dirty == 0)
        {
            break;
        }

        if (total_dirty > 5)
        {
            pack_size = 7;
        }
        else if (total_dirty > 2)
        {
            pack_size = 5;
        }
        else if (total_dirty > 1)
        {
            pack_size = 2;
        }
        else
        {
            pack_size = 1;
        }

        memset(graph_buffer, 0, sizeof(graph_buffer));
        int now_idx = 0;
        for (int i = 0; i < UI_CACHE_MAX_FIGURES && now_idx < pack_size; i++)
        {
            if (ui_dirty_figure[i] > 0)
            {
                graph_buffer[now_idx++] = ui_now_figures[i];
                ui_dirty_figure[i]--;
            }
        }

        if (now_idx > 0)
        {
            UI_SendGraphFrame(graph_buffer, (uint8_t)pack_size, UI_Data_ID_Draw1 + (pack_size == 1 ? 0 : pack_size == 2 ? 1 : pack_size == 5 ? 2 : 3));
            sent_count++;
        }
        else
        {
            break;
        }
    }

    for (int i = 0; i < UI_CACHE_MAX_STRINGS; i++)
    {
        if (ui_dirty_string[i] > 0)
        {
            UI_SendStringFrame(&ui_now_strings[i]);
            ui_dirty_string[i]--;
        }
    }
}

void UI_ResetManager(void)
{
    memset(ui_now_figures, 0, sizeof(ui_now_figures));
    memset(ui_dirty_figure, 0, sizeof(ui_dirty_figure));
    memset(ui_figure_used, 0, sizeof(ui_figure_used));
    memset(ui_now_strings, 0, sizeof(ui_now_strings));
    memset(ui_dirty_string, 0, sizeof(ui_dirty_string));
    memset(ui_string_used, 0, sizeof(ui_string_used));
}

void UI_ManualRefresh(void)
{
    UI_ScanAndSend();
}

void UI_PostInitSwitchToChange(void)
{
    for (int i = 0; i < UI_CACHE_MAX_FIGURES; i++)
    {
        if (ui_figure_used[i])
        {
            ui_now_figures[i].operate_tpye = UI_Graph_Change;
        }
    }

    for (int i = 0; i < UI_CACHE_MAX_STRINGS; i++)
    {
        if (ui_string_used[i])
        {
            ui_now_strings[i].Graph_Control.operate_tpye = UI_Graph_Change;
        }
    }
}

void UIDelete(referee_id_t *_id, uint8_t Del_Operate, uint8_t Del_Layer)
{
    uint8_t temp_datalength = Interactive_Data_LEN_Head + UI_Operate_LEN_Del;

    UI_UpdateActiveID(_id);
    if (ui_active_id == NULL)
    {
        return;
    }

    ui_delete_frame.FrameHeader.SOF = REFEREE_SOF;
    ui_delete_frame.FrameHeader.DataLength = temp_datalength;
    ui_delete_frame.FrameHeader.Seq = UI_Seq;
    ui_delete_frame.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&ui_delete_frame, LEN_CRC8, 0xFF);
    ui_delete_frame.CmdID = ID_student_interactive;
    ui_delete_frame.datahead.data_cmd_id = UI_Data_ID_Del;
    ui_delete_frame.datahead.receiver_ID = ui_active_id->Cilent_ID;
    ui_delete_frame.datahead.sender_ID = ui_active_id->Robot_ID;
    ui_delete_frame.delete_type = Del_Operate;
    ui_delete_frame.layer = Del_Layer;
    ui_delete_frame.frametail = Get_CRC16_Check_Sum(
        (uint8_t *)&ui_delete_frame,
        LEN_HEADER + LEN_CMDID + temp_datalength,
        0xFFFF);

    RefereeSend((uint8_t *)&ui_delete_frame,
                LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL);
    UI_Seq++;
}

void Line_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
               uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
    memset(graph, 0, sizeof(*graph));
    UI_SetGraphicName(graph->graphic_name, graphname);
    graph->operate_tpye = Graph_Operate;
    graph->graphic_tpye = UI_Graph_Line;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->end_x = End_x;
    graph->end_y = End_y;
}

void Rectangle_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                    uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
    memset(graph, 0, sizeof(*graph));
    UI_SetGraphicName(graph->graphic_name, graphname);
    graph->graphic_tpye = UI_Graph_Rectangle;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->end_x = End_x;
    graph->end_y = End_y;
}

void Circle_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                 uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
{
    memset(graph, 0, sizeof(*graph));
    UI_SetGraphicName(graph->graphic_name, graphname);
    graph->graphic_tpye = UI_Graph_Circle;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = Graph_Radius;
}

void Elliptical_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                     uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y)
{
    memset(graph, 0, sizeof(*graph));
    UI_SetGraphicName(graph->graphic_name, graphname);
    graph->graphic_tpye = UI_Graph_Ellipse;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

void Arc_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
              uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
              uint32_t end_x, uint32_t end_y)
{
    memset(graph, 0, sizeof(*graph));
    UI_SetGraphicName(graph->graphic_name, graphname);
    graph->graphic_tpye = UI_Graph_Arc;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;
    graph->start_angle = Graph_StartAngle;
    graph->end_angle = Graph_EndAngle;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->end_x = end_x;
    graph->end_y = end_y;
}

void Float_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Float)
{
    memset(graph, 0, sizeof(*graph));
    UI_SetGraphicName(graph->graphic_name, graphname);
    graph->graphic_tpye = UI_Graph_Float;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->start_angle = Graph_Size;
    graph->end_angle = Graph_Digit;
    graph->radius = Graph_Float & 0x3FF;
    graph->end_x = (Graph_Float >> 10) & 0x7FF;
    graph->end_y = (Graph_Float >> 21) & 0x7FF;
}

void Integer_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                  uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer)
{
    memset(graph, 0, sizeof(*graph));
    UI_SetGraphicName(graph->graphic_name, graphname);
    graph->graphic_tpye = UI_Graph_Int;
    graph->operate_tpye = Graph_Operate;
    graph->layer = Graph_Layer;
    graph->color = Graph_Color;
    graph->start_angle = Graph_Size;
    graph->width = Graph_Width;
    graph->start_x = Start_x;
    graph->start_y = Start_y;
    graph->radius = Graph_Integer & 0x3FF;
    graph->end_x = (Graph_Integer >> 10) & 0x7FF;
    graph->end_y = (Graph_Integer >> 21) & 0x7FF;
}

void Char_Draw(String_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
               uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *fmt, ...)
{
    va_list ap;

    memset(graph, 0, sizeof(*graph));
    UI_SetGraphicName(graph->Graph_Control.graphic_name, graphname);
    graph->Graph_Control.graphic_tpye = UI_Graph_Char;
    graph->Graph_Control.operate_tpye = Graph_Operate;
    graph->Graph_Control.layer = Graph_Layer;
    graph->Graph_Control.color = Graph_Color;
    graph->Graph_Control.width = Graph_Width;
    graph->Graph_Control.start_x = Start_x;
    graph->Graph_Control.start_y = Start_y;
    graph->Graph_Control.start_angle = Graph_Size;

    va_start(ap, fmt);
    vsnprintf((char *)graph->show_Data, sizeof(graph->show_Data), fmt, ap);
    va_end(ap);

    graph->Graph_Control.end_angle = strlen((const char *)graph->show_Data);
}

void UI_ReFresh(referee_id_t *_id, int cnt, ...)
{
    va_list ap;

    UI_UpdateActiveID(_id);
    va_start(ap, cnt);
    for (int i = 0; i < cnt; i++)
    {
        Graph_Data_t graph_data = va_arg(ap, Graph_Data_t);
        int slot = UI_FindGraphSlot(graph_data.graphic_name);
        if (slot < 0)
        {
            slot = UI_AllocGraphSlot();
        }
        if (slot >= 0)
        {
            ui_now_figures[slot] = graph_data;
            ui_dirty_figure[slot] = ui_max_send_count[slot];
        }
    }
    va_end(ap);
}

void Char_ReFresh(referee_id_t *_id, String_Data_t string_Data)
{
    int slot;

    UI_UpdateActiveID(_id);
    slot = UI_FindStringSlot(string_Data.Graph_Control.graphic_name);
    if (slot < 0)
    {
        slot = UI_AllocStringSlot();
    }
    if (slot >= 0)
    {
        ui_now_strings[slot] = string_Data;
        ui_dirty_string[slot] = ui_max_send_count[UI_CACHE_MAX_FIGURES + slot];
    }
}
