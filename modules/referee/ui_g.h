#ifndef UI_G_H
#define UI_G_H

#include "ui_interface.h"

#define UI_G_TOTAL_FIGURE 7
#define UI_G_TOTAL_STRING 12

extern Graph_Data_t ui_g_now_figures[UI_G_TOTAL_FIGURE];
extern uint8_t ui_g_dirty_figure[UI_G_TOTAL_FIGURE];
extern String_Data_t ui_g_now_strings[UI_G_TOTAL_STRING];
extern uint8_t ui_g_dirty_string[UI_G_TOTAL_STRING];
extern uint8_t ui_g_max_send_count[UI_G_TOTAL_FIGURE + UI_G_TOTAL_STRING];

#define ui_g_LINE_CrossHorizontal (&ui_g_now_figures[0])
#define ui_g_LINE_CrossVertical   (&ui_g_now_figures[1])
#define ui_g_RECT_ChassisBox      (&ui_g_now_figures[2])
#define ui_g_ARC_ChassisYaw       (&ui_g_now_figures[3])
#define ui_g_RECT_AimBox          (&ui_g_now_figures[4])
#define ui_g_RECT_EnergyFrame     (&ui_g_now_figures[5])
#define ui_g_LINE_EnergyBar       (&ui_g_now_figures[6])

#define ui_g_TEXT_AimLabel        (&ui_g_now_strings[0])
#define ui_g_TEXT_FrictionLabel   (&ui_g_now_strings[1])
#define ui_g_TEXT_FireLabel       (&ui_g_now_strings[2])
#define ui_g_TEXT_RotateLabel     (&ui_g_now_strings[3])
#define ui_g_TEXT_HeadLabel       (&ui_g_now_strings[4])
#define ui_g_TEXT_AimState        (&ui_g_now_strings[5])
#define ui_g_TEXT_FrictionState   (&ui_g_now_strings[6])
#define ui_g_TEXT_FireState       (&ui_g_now_strings[7])
#define ui_g_TEXT_RotateState     (&ui_g_now_strings[8])
#define ui_g_TEXT_HeadState       (&ui_g_now_strings[9])
#define ui_g_TEXT_JudgePower      (&ui_g_now_strings[10])
#define ui_g_TEXT_CapPower        (&ui_g_now_strings[11])

void ui_bind_g(referee_info_t *referee, Referee_Interactive_info_t *interactive);
void ui_init_g(void);
void ui_update_g(void);

#endif
