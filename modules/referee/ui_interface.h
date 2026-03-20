#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include "rm_referee.h"
#include "ui_types.h"

#define ui_set(obj, field, value) \
    do                            \
    {                             \
        (obj)->field = (value);   \
    } while (0)

void UIDelete(referee_id_t *_id, uint8_t Del_Operate, uint8_t Del_Layer);

void Line_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
               uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
void Rectangle_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                    uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
void Circle_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                 uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);
void Elliptical_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                     uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t end_x, uint32_t end_y);
void Arc_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
              uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y,
              uint32_t end_x, uint32_t end_y);
void Float_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Float);
void Integer_Draw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                  uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer);
void Char_Draw(String_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
               uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *fmt, ...);

void UI_ReFresh(referee_id_t *_id, int cnt, ...);
void Char_ReFresh(referee_id_t *_id, String_Data_t string_Data);
void UI_ResetManager(void);
void UI_ManualRefresh(void);
void UI_PostInitSwitchToChange(void);

void ui_proc_1_frame(void *msg);
void ui_proc_2_frame(void *msg);
void ui_proc_5_frame(void *msg);
void ui_proc_7_frame(void *msg);
void ui_proc_string_frame(void *msg);
void ui_proc_delete_frame(void *msg);
void ui_delete_layer(const uint8_t delete_type, const uint8_t layer);
void ui_scan_and_send(const ui_interface_figure_t *ui_now_figures, uint8_t *ui_dirty_figure,
                      const ui_interface_string_t *ui_now_strings, uint8_t *ui_dirty_string,
                      int total_figures, int total_strings);

#endif
