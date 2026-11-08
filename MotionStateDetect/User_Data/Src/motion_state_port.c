/**
  ******************************************************************************
  * @file           : motion_state_port.c
  * @brief          : 运动模式判断的 STM32H5_FT 平台适配层实现。
  *                   平台输入：User_Motor_Data(左右髋角/角速度)、
  *                   User_IMU_Data(俯仰角)、User_Barometer_Data(高度)。
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
/* Includes ------------------------------------------------------------------*/
#include "motion_state_port.h"
#include "User_Data.h"

/* Define --------------------------------------------------------------------*/

/* Variable ------------------------------------------------------------------*/
static Motion_Handle_t s_motion;
static uint8_t s_inited = 0U;

/* Function ------------------------------------------------------------------*/

/**
* @brief 初始化运动模式判断适配层
* @param 无
* @return 无
*/
void MotionState_Port_Init(void)
{
    Motion_Config_t cfg;

    Motion_DefaultConfig(&cfg);
    cfg.dt_s = 0.001f;
    Motion_Init(&s_motion, &cfg);
    s_inited = 1U;
}

/**
* @brief 周期任务：读取平台传感器快照并更新状态机
* @param dT_ms 任务周期，单位ms
* @return 当前运动状态
* @note  多字段组合先一次性复制快照，避免后台任务半途改写。
*/
Motion_State_e MotionState_Port_Task(uint16_t dT_ms)
{
    const struct User_Motor_data_t motor = User_Motor_Data;
    const struct User_IMU_data_t imu = User_IMU_Data;
    const struct User_Barometer_data_t baro = User_Barometer_Data;
    Motion_Input_t in;

    if (s_inited == 0U)
    {
        MotionState_Port_Init();
    }

    /* 任务周期以实际传入为准，保证高度率等时间相关量正确 */
    s_motion.cfg.dt_s = (float)dT_ms * 0.001f;

    in.left_hip_angle = motor.left.output_position;
    in.left_hip_velocity = motor.left.output_velocity;
    in.right_hip_angle = motor.right.output_position;
    in.right_hip_velocity = motor.right.output_velocity;
    in.pitch_deg = imu.user_euler.pitch;
    in.altitude_m = baro.altitude_m;
    in.altitude_valid = baro.altitude_ready;

    return Motion_Process(&s_motion, &in);
}

/**
* @brief 获取当前运动状态
* @param 无
* @return 当前运动状态
*/
Motion_State_e MotionState_Port_GetState(void)
{
    if (s_inited == 0U)
    {
        return MOTION_STATE_INIT;
    }
    return Motion_GetState(&s_motion);
}

/**
* @brief 获取最近一拍的算法输出快照
* @param 无
* @return 输出快照指针
*/
const Motion_Output_t *MotionState_Port_GetOutput(void)
{
    return Motion_GetOutput(&s_motion);
}
