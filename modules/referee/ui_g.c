#include "ui_g.h"

#include <string.h>

#include "rm_referee.h"
#include "task.h"

Graph_Data_t ui_g_now_figures[UI_G_TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[UI_G_TOTAL_FIGURE];
String_Data_t ui_g_now_strings[UI_G_TOTAL_STRING];
uint8_t ui_g_dirty_string[UI_G_TOTAL_STRING];
uint8_t ui_g_max_send_count[UI_G_TOTAL_FIGURE + UI_G_TOTAL_STRING] = {
    3, 3, 3, 3, 3, 3, 3,
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
};

static referee_info_t *ui_g_referee_info;
static Referee_Interactive_info_t *ui_g_interactive_info;
static Graph_Data_t ui_g_last_figures[UI_G_TOTAL_FIGURE];
static String_Data_t ui_g_last_strings[UI_G_TOTAL_STRING];

static void ui_g_determine_robot_id(void)
{
    ui_g_referee_info->referee_id.Robot_Color =
        ui_g_referee_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    ui_g_referee_info->referee_id.Robot_ID = ui_g_referee_info->GameRobotState.robot_id;
    ui_g_referee_info->referee_id.Cilent_ID = 0x0100 + ui_g_referee_info->referee_id.Robot_ID;
    ui_g_referee_info->referee_id.Receiver_Robot_ID = 0;
}

static void ui_g_set_names_and_change_mode(void)
{
    static const char figure_names[UI_G_TOTAL_FIGURE][4] = {
        "sl0", "sl1", "sp0", "sd5", "bx0", "se0", "sd4"
    };
    static const char string_names[UI_G_TOTAL_STRING][4] = {
        "ss0", "ss1", "ss2", "ss3", "ss4", "sd0", "sd1", "sd2", "sd3", "sd6", "sd7", "sd8"
    };
    int i;

    for (i = 0; i < UI_G_TOTAL_FIGURE; i++)
    {
        ui_g_now_figures[i].operate_tpye = UI_Graph_Change;
        memcpy(ui_g_now_figures[i].graphic_name, ui_g_last_figures[i].graphic_name, 3);
        if (ui_g_now_figures[i].graphic_name[0] == 0 &&
            ui_g_now_figures[i].graphic_name[1] == 0 &&
            ui_g_now_figures[i].graphic_name[2] == 0)
        {
            ui_g_now_figures[i].graphic_name[2] = figure_names[i][0];
            ui_g_now_figures[i].graphic_name[1] = figure_names[i][1];
            ui_g_now_figures[i].graphic_name[0] = figure_names[i][2];
        }
    }

    for (i = 0; i < UI_G_TOTAL_STRING; i++)
    {
        ui_g_now_strings[i].Graph_Control.operate_tpye = UI_Graph_Change;
        memcpy(ui_g_now_strings[i].Graph_Control.graphic_name, ui_g_last_strings[i].Graph_Control.graphic_name, 3);
        if (ui_g_now_strings[i].Graph_Control.graphic_name[0] == 0 &&
            ui_g_now_strings[i].Graph_Control.graphic_name[1] == 0 &&
            ui_g_now_strings[i].Graph_Control.graphic_name[2] == 0)
        {
            ui_g_now_strings[i].Graph_Control.graphic_name[2] = string_names[i][0];
            ui_g_now_strings[i].Graph_Control.graphic_name[1] = string_names[i][1];
            ui_g_now_strings[i].Graph_Control.graphic_name[0] = string_names[i][2];
        }
    }
}

static void ui_g_build_static_objects(uint32_t graph_operate)
{
    // Line_Draw(ui_g_LINE_CrossHorizontal, "sl0", graph_operate, 0, UI_Color_Purplish_red, 3, 560, 540, 1360, 540);
    // Line_Draw(ui_g_LINE_CrossVertical, "sl1", graph_operate, 0, UI_Color_Purplish_red, 3, 960, 140, 960, 940);
    // Rectangle_Draw(ui_g_RECT_ChassisBox, "sp0", graph_operate, 0, UI_Color_White, 3, 1650, 400, 1740, 540);
    // Arc_Draw(ui_g_ARC_ChassisYaw, "sd5", graph_operate, 8, UI_Color_Main, 270, 90, 3, 1695, 470, 40, 40);
    Rectangle_Draw(ui_g_RECT_AimBox, "bx0", graph_operate, 0, UI_Color_Green, 3, 800, 400, 1120, 680);
    Rectangle_Draw(ui_g_RECT_EnergyFrame, "se0", graph_operate, 0, UI_Color_Pink, 7, 752, 81, 1165, 111);

    Char_Draw(ui_g_TEXT_AimLabel, "ss0", graph_operate, 8, UI_Color_Main, 15, 3, 50, 750, "AUTO_AIM:");
    Char_Draw(ui_g_TEXT_FrictionLabel, "ss1", graph_operate, 8, UI_Color_Main, 15, 3, 50, 700, "FRICTION:");
    Char_Draw(ui_g_TEXT_FireLabel, "ss2", graph_operate, 8, UI_Color_Main, 15, 3, 50, 650, "FIRE:");
    // Char_Draw(ui_g_TEXT_RotateLabel, "ss3", graph_operate, 8, UI_Color_Main, 15, 3, 50, 600, "ROTATE:");
    Char_Draw(ui_g_TEXT_HeadLabel, "ss4", graph_operate, 8, UI_Color_Main, 15, 3, 50, 600, "HEAD:");
}

static void ui_g_build_dynamic_objects(uint32_t graph_operate)
{

    switch (ui_g_interactive_info->aim_mode)
    {
    case AIM_ON:
        Char_Draw(ui_g_TEXT_AimState, "sd0", graph_operate, 8, UI_Color_Green, 15, 2, 185, 750, "ON ");
        break;
    case AIM_OFF:
    default:
        Char_Draw(ui_g_TEXT_AimState, "sd0", graph_operate, 8, UI_Color_Pink, 15, 2, 185, 750, "OFF");
        break;
    }

    switch (ui_g_interactive_info->friction_mode)
    {
    case FRICTION_ON:
        Char_Draw(ui_g_TEXT_FrictionState, "sd1", graph_operate, 8, UI_Color_Green, 15, 2, 185, 700, "ON ");
        break;
    case FRICTION_OFF:
    default:
        Char_Draw(ui_g_TEXT_FrictionState, "sd1", graph_operate, 8, UI_Color_Pink, 15, 2, 185, 700, "OFF");
        break;
    }

    switch (ui_g_interactive_info->fire_mode)
    {
    case FIRE_OFF:
        Char_Draw(ui_g_TEXT_FireState, "sd2", graph_operate, 8, UI_Color_Pink, 15, 2, 185, 650, "OFF");
        break;
    case FIRE_ON:
    default:
        Char_Draw(ui_g_TEXT_FireState, "sd2", graph_operate, 8, UI_Color_Green, 15, 2, 185, 650, "ON ");
        break;
    }

    // switch (ui_g_interactive_info->chassis_mode)
    // {
    // case CHASSIS_ROTATE:
    //     Char_Draw(ui_g_TEXT_RotateState, "sd3", graph_operate, 8, UI_Color_Green, 15, 2, 185, 600, "ON ");
    //     break;
    // default:
    //     Char_Draw(ui_g_TEXT_RotateState, "sd3", graph_operate, 8, UI_Color_Pink, 15, 2, 185, 600, "OFF");
    //     break;
    // }

    switch (ui_g_interactive_info->chassis_mode)
    {
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        Char_Draw(ui_g_TEXT_HeadState, "sd6", graph_operate, 8, UI_Color_Green, 15, 2, 185, 600, "FOLLOW");
        break;
    case CHASSIS_NO_FOLLOW:
        Char_Draw(ui_g_TEXT_HeadState, "sd6", graph_operate, 8, UI_Color_Cyan, 15, 2, 185, 600, "FREE  ");
        break;
    case CHASSIS_ROTATE:
        Char_Draw(ui_g_TEXT_HeadState, "sd6", graph_operate, 8, UI_Color_Orange, 15, 2, 185, 600, "SPIN  ");
        break;
    default:
        Char_Draw(ui_g_TEXT_HeadState, "sd6", graph_operate, 8, UI_Color_Pink, 15, 2, 185, 600, "STOP  ");
        break;
    }

    Line_Draw(ui_g_LINE_EnergyBar, "sd4", graph_operate, 8, UI_Color_Green, 30, 755, 97,
              (uint32_t)755 + (uint32_t)((float)ui_g_interactive_info->capEnergy / 255.0f * 410.0f), 97);

    Char_Draw(ui_g_TEXT_JudgePower, "sd7", graph_operate, 8, UI_Color_Purplish_red, 18, 3, 600, 88,
              "%dW", (int)(ui_g_referee_info->GameRobotState.chassis_power_limit + 0.5f));
    Char_Draw(ui_g_TEXT_CapPower, "sd8", graph_operate, 8, UI_Color_Green, 18, 3, 1285, 88,
              "%dW", (int)(ui_g_interactive_info->supercap_now_power +
              (ui_g_interactive_info->supercap_now_power >= 0.0f ? 0.5f : -0.5f)));

    // if (ui_g_interactive_info->chassis_mode == CHASSIS_ROTATE)
    // {
    //     Arc_Draw(ui_g_ARC_ChassisYaw, "sd5", graph_operate, 8, UI_Color_Main, 0, 360, 3, 1695, 470, 40, 40);
    // }
    // else
    // {
    //     Arc_Draw(ui_g_ARC_ChassisYaw, "sd5", graph_operate, 8, UI_Color_Main, (uint32_t)start_angle, (uint32_t)end_angle, 3, 1695, 470, 40, 40);
    // }
}

static void ui_g_submit_dirty(void)
{
    ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string,
                     UI_G_TOTAL_FIGURE, UI_G_TOTAL_STRING);
}

static void ui_g_sync_last_and_dirty_on_init(void)
{
    int i;

    for (i = 0; i < UI_G_TOTAL_FIGURE; i++)
    {
        ui_g_last_figures[i] = ui_g_now_figures[i];
        ui_g_dirty_figure[i] = 1;
    }

    for (i = 0; i < UI_G_TOTAL_STRING; i++)
    {
        ui_g_last_strings[i] = ui_g_now_strings[i];
        ui_g_dirty_string[i] = 1;
    }
}

void ui_bind_g(referee_info_t *referee, Referee_Interactive_info_t *interactive)
{
    ui_g_referee_info = referee;
    ui_g_interactive_info = interactive;
}

void ui_init_g(void)
{
    if (ui_g_referee_info == NULL || ui_g_interactive_info == NULL)
    {
        return;
    }

    while (ui_g_referee_info->GameRobotState.robot_id == 0)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ui_g_determine_robot_id();
    UI_ResetManager();
    UIDelete(&ui_g_referee_info->referee_id, UI_Data_Del_ALL, 0);

    memset(ui_g_now_figures, 0, sizeof(ui_g_now_figures));
    memset(ui_g_now_strings, 0, sizeof(ui_g_now_strings));
    memset(ui_g_last_figures, 0, sizeof(ui_g_last_figures));
    memset(ui_g_last_strings, 0, sizeof(ui_g_last_strings));
    memset(ui_g_dirty_figure, 0, sizeof(ui_g_dirty_figure));
    memset(ui_g_dirty_string, 0, sizeof(ui_g_dirty_string));

    ui_g_build_static_objects(UI_Graph_ADD);
    ui_g_build_dynamic_objects(UI_Graph_ADD);
    ui_g_sync_last_and_dirty_on_init();
    ui_g_submit_dirty();
    UI_PostInitSwitchToChange();
    ui_g_set_names_and_change_mode();
    memcpy(ui_g_last_figures, ui_g_now_figures, sizeof(ui_g_now_figures));
    memcpy(ui_g_last_strings, ui_g_now_strings, sizeof(ui_g_now_strings));
}

void ui_update_g(void)
{
    int i;

    if (ui_g_referee_info == NULL || ui_g_interactive_info == NULL)
    {
        return;
    }

    ui_g_build_dynamic_objects(UI_Graph_Change);

    for (i = 0; i < UI_G_TOTAL_FIGURE; i++)
    {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0)
        {
            ui_g_dirty_figure[i] = ui_g_max_send_count[i];
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }

    for (i = 0; i < UI_G_TOTAL_STRING; i++)
    {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0)
        {
            ui_g_dirty_string[i] = ui_g_max_send_count[UI_G_TOTAL_FIGURE + i];
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }

    ui_g_submit_dirty();
}
