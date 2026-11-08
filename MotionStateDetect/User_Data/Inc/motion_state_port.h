/**
  ******************************************************************************
  * @file    motion_state_port.h
  * @brief   运动模式判断的 STM32H5_FT 平台适配层头文件。
  *          把 User_Motor_Data/User_IMU_Data/User_Barometer_Data 组装为
  *          核心算法的 Motion_Input_t，供控制任务与回传任务调用。
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 DRVEXO.
  * All rights reserved.
  *
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTION_STATE_PORT_H__
#define __MOTION_STATE_PORT_H__
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motion_state.h"

/* Exported functions prototypes ---------------------------------------------*/

/**
* @brief 初始化运动模式判断适配层（User_main中调用一次）
* @param 无
* @return 无
*/
void MotionState_Port_Init(void);

/**
* @brief 周期任务：读取平台传感器快照并更新状态机
* @param dT_ms 任务周期，单位ms，由控制任务传入
* @return 当前运动状态
* @note  在Awalk_Control_Task中调用（1ms中断上下文，禁止阻塞）。
*/
Motion_State_e MotionState_Port_Task(uint16_t dT_ms);

/**
* @brief 获取当前运动状态
* @param 无
* @return 当前运动状态
*/
Motion_State_e MotionState_Port_GetState(void);

/**
* @brief 获取最近一拍的算法输出快照（供回传/其它模块使用）
* @param 无
* @return 输出快照指针
*/
const Motion_Output_t *MotionState_Port_GetOutput(void);

#ifdef __cplusplus
}
#endif
#endif   /* __MOTION_STATE_PORT_H__ */
