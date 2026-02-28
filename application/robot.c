#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#warning check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
#include "chassis.h"
#endif

#ifdef GIMBAL_BOARD
#include "gimbal.h"
#include "shoot.h"
#endif

#if defined(GIMBAL_BOARD) || defined(CHASSIS_ONLY)
#include "robot_cmd.h"
#endif

#ifdef BALANCE_BAORD
#include "balance.h"
#endif // BALANCE_BOARD


void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();

#if defined(GIMBAL_BOARD) || defined(CHASSIS_ONLY)
    RobotCMDInit();
#endif

#ifdef GIMBAL_BOARD
    GimbalInit();
    ShootInit();
#endif

#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
    ChassisInit();
#endif

#ifdef BALANCE_BAORD
    BalanceInit();
#endif // BALANCE_BA

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
    
#if defined(GIMBAL_BOARD) || defined(CHASSIS_ONLY)
    RobotCMDTask();
#endif

#ifdef GIMBAL_BOARD
    GimbalTask();
    ShootTask();
#endif

#if defined(CHASSIS_BOARD) || defined(CHASSIS_ONLY)
    ChassisTask();
#endif

#ifdef BALANCE_BAORD
    BalanceTask();
#endif // BALANCE_BA
}
