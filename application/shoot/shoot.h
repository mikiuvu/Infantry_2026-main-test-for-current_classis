#ifndef SHOOT_H
#define SHOOT_H

#include "robot_def.h"

/**
 * @brief 发射初始化,会被RobotInit()调用
 *        根据USE_LASER_POINTER宏定义选择初始化激光笔或完整发射机构
 */
void ShootInit();

/**
 * @brief 发射任务
 *        根据USE_LASER_POINTER宏定义选择控制激光笔或完整发射机构
 */
void ShootTask();

#ifdef USE_LASER_POINTER
/**
 * @brief 激光笔控制函数
 * @param enable 1-打开激光笔, 0-关闭激光笔
 */
void LaserControl(uint8_t enable);
#endif

#endif // SHOOT_H