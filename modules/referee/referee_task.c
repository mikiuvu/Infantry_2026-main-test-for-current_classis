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
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                               // 包序号，供整个referee文件使用
int StartAngle, EndAngle;                // 圆弧绘制用的起始角度和终止角度
static uint8_t ui_initialized = 0;
static ui_mode_e last_ui_mode = UI_KEEP;

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 +9* @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void My_UI_Refresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void Mode_Change_Check(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测

// syhtod 正式上车后需删除
// static void robot_mode_change(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t *Referee_Interactive_init(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    return referee_recv_info;
}

void Referee_Interactive_task()
{
    if (Interactive_data == NULL || referee_recv_info == NULL)
    {
        return;
    }

    if (!ui_initialized || (Interactive_data->ui_mode == UI_REFRESH && last_ui_mode != UI_REFRESH))
    {
        My_UI_init();
    }

    last_ui_mode = Interactive_data->ui_mode;
    My_UI_Refresh(referee_recv_info, Interactive_data);
}

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static uint32_t shoot_line_location[10] = {540, 960, 430, 450, 470, 495, 515, 560, 580};//射击基准线位置

static Graph_Data_t UI_position_line[10]; // 车身位姿 

static Graph_Data_t UI_Energy[3];      // 电容能量条

static String_Data_t UI_State_sta[6];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change
 
static Graph_Data_t UI_aim_circle[2]; //瞄准准圈

void My_UI_init()
{
    if (Interactive_data == NULL || referee_recv_info == NULL)
    {
        return;
    }

    while (referee_recv_info->GameRobotState.robot_id == 0)
    {
        RefereeSend(NULL,0);
    }
    DeterminRobotID();
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI
    
    // 绘制瞄准准圈
    Circle_Draw(&UI_aim_circle[0], "SCs",UI_Graph_ADD ,0,UI_Color_Yellow, 3,960,540,40); //小圈
    Circle_Draw(&UI_aim_circle[1], "SCm",UI_Graph_ADD ,0,UI_Color_Green, 3,960,540,360); //大圈
    UI_ReFresh(&referee_recv_info->referee_id, 2, UI_aim_circle[0], UI_aim_circle[1]);

    // 绘制基准线
    Line_Draw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 0, UI_Color_Purplish_red, 3, 560, 540, 1360, 540); // 横向基准线
    Line_Draw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 0, UI_Color_Purplish_red, 3, 960, 140, 960, 940); // 纵向基准线
    UI_ReFresh(&referee_recv_info->referee_id, 2, UI_shoot_line[0], UI_shoot_line[1]);

    // 绘制车身位姿
    Rectangle_Draw(&UI_position_line[0], "sp0", UI_Graph_ADD, 0, UI_Color_White, 3, 1650, 400, 1740, 540);
    Arc_Draw(&UI_position_line[1], "sd5", UI_Graph_ADD, 8, UI_Color_Main, 270, 90, 3, 1695, 470, 40, 40);
    UI_ReFresh(&referee_recv_info->referee_id, 2, UI_position_line[0], UI_position_line[1]);


    /**
    Line_Draw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Main, 2, 930, shoot_line_location[4], 960, shoot_line_location[4]);
    Line_Draw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Main, 2, 960, shoot_line_location[5], 990, shoot_line_location[5]);
    Line_Draw(&UI_shoot_line[6], "sl6", UI_Graph_ADD, 7, UI_Color_Main, 2, 930, shoot_line_location[6], 960, shoot_line_location[6]);
    
    //UI_ReFresh(&referee_recv_info->referee_id, 7, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4] ,UI_shoot_line[5],UI_shoot_line[6]);
    
    Line_Draw(&UI_shoot_line[7], "sl7", UI_Graph_ADD, 7, UI_Color_Main, 2, 960, shoot_line_location[7], 990, shoot_line_location[7]);
    Line_Draw(&UI_shoot_line[8], "sl8", UI_Graph_ADD, 7, UI_Color_Main, 2, 930, shoot_line_location[8], 960, shoot_line_location[8]);
    UI_ReFresh(&referee_recv_info->referee_id, 2, UI_shoot_line[7], UI_shoot_line[8]);
    **/


    // 绘制车辆状态标志指示
    Char_Draw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Main, 15, 3, 50, 750, "AUTO_AIM:");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    Char_Draw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Main, 15, 3, 50, 700, "FRICTION:");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    Char_Draw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Main, 15, 3, 50, 650, "COVER:");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    Char_Draw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Main, 15, 3, 50, 600, "ROTATE:");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_sta[3]);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    Char_Draw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 185, 750, "OFF");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    Char_Draw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 185, 700, "OFF");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    Char_Draw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 185, 650, "open ");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    Char_Draw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 185, 600, "OFF");
    Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[3]);


    // 能量条框
    Rectangle_Draw(&UI_Energy[0], "se0", UI_Graph_ADD, 0, UI_Color_Pink, 7, 752, 81, 1138, 111);
    UI_ReFresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);


    // 能量条初始状态
    Line_Draw(&UI_Energy[1], "sd4", UI_Graph_ADD, 8, UI_Color_Green, 30, 755, 97, 1165, 97);
    UI_ReFresh(&referee_recv_info->referee_id, 1, UI_Energy[1]);

    ui_initialized = 1;
}



static void My_UI_Refresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    
    Mode_Change_Check(_Interactive_data);
    // 自瞄
    if (_Interactive_data->Referee_Interactive_Flag.tracking_flag == 1)
    {
        switch (_Interactive_data->aim_mode)
        {
        case AIM_OFF:
            Char_Draw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 185, 750, "OFF");
            break;
        case AIM_ON:
            Char_Draw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 185, 750, "ON ");
            // 此处注意字数对齐问题，字数相同才能覆盖掉
            break;
        }
        Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.tracking_flag = 0;
    }
    // 摩擦轮
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        switch (_Interactive_data->friction_mode)
        {
        case FRICTION_OFF:
        {
            Char_Draw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 185, 700,"OFF");
            break;
        }
        case FRICTION_ON:
        {
            Char_Draw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 185, 700,"ON ");
            break;
        }
        }
        Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }

    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {   
        switch (_Interactive_data->chassis_mode)
        {
        case CHASSIS_ROTATE:
        {
            Char_Draw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 185, 600, "ON ");
            break;
        }
        default:
        {
            Char_Draw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 185, 600,  "OFF" );
            break;
        }
        }
        Char_ReFresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }

    // 超电能量条
    Line_Draw(&UI_Energy[1], "sd4", UI_Graph_Change, 8, UI_Color_Green, 30, 755, 97, (uint32_t)755 + (uint32_t)((float)_Interactive_data->capEnergy / 255 * 410), 97);

    // 车身位姿
    _Interactive_data->offset_angle = -_Interactive_data->offset_angle; 
    if (_Interactive_data->offset_angle < 0)
    {
        _Interactive_data->offset_angle += 360;
    }
    StartAngle = (int)(_Interactive_data->offset_angle - 90); //计算云台相对于车身的角度
    EndAngle = (int)(_Interactive_data->offset_angle + 90);
    if (StartAngle < 0)
    {
        StartAngle += 360;
    }

    if (EndAngle > 360)
    {
        EndAngle -= 360;
    }

    switch (_Interactive_data->chassis_mode) // 小陀螺判断
    {
    case CHASSIS_ROTATE:
    {
        Arc_Draw(&UI_position_line[1], "sd5", UI_Graph_Change, 8, UI_Color_Main, 0, 360, 3, 1695, 470, 40, 40); //小陀螺模式则无需找头
        break;
    }
    default:
    {
        Arc_Draw(&UI_position_line[1], "sd5", UI_Graph_Change, 8, UI_Color_Main, StartAngle, EndAngle, 3, 1695, 470, 40, 40);
        break;
    }
    }
    

    UI_ReFresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_position_line[1]);

}
/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void Mode_Change_Check(Referee_Interactive_info_t *_Interactive_data)
{
    // if (_Interactive_data->chassis_mode == CHASSIS_ROTATE) //判断是否小陀螺
    // {
    //     _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
    // }
    // else
    // {
    //     _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    // }

    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }

    if (_Interactive_data->aim_mode != _Interactive_data->last_aim_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.tracking_flag = 1;
        _Interactive_data->last_aim_mode = _Interactive_data->aim_mode;
    }
}
