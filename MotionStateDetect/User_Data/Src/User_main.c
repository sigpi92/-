/**
  ******************************************************************************
  * @file           : User_main.c
  * @brief          : 用户二次开发主入口文件
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
#include "User_main.h"
#include "awalk_control.h"
#include "motion_state_port.h"
#include "transmission.h"
#include "DRVEXO_Scheduler.h"

/* Define --------------------------------------------------------------------*/
/* 控制任务周期：1ms（后台调度器触发，运行于TIM7中断上下文） */
#define USER_AWALK_TASK_PERIOD_MS   1U
/* 控制任务优先级：排在FDCAN(21)/LowPower(22)之后，取到本周期最新电机反馈 */
#define USER_AWALK_TASK_PRIO        23U
/* 数据回传任务周期：20ms（VOFA+观察，避免DMA忙） */
#define USER_DEBUG_TASK_PERIOD_MS   20U
/* 数据回传任务优先级：最低 */
#define USER_DEBUG_TASK_PRIO        31U
/* 数据回传通道数：原10通道 + 运动状态4通道 + 气压计原始高度1通道 */
#define USER_DEBUG_CHANNEL_NUM      15U

/* Variable ------------------------------------------------------------------*/
static TaskHandle_t AwalkTaskHandle = NULL;     // 控制任务句柄
static TaskHandle_t DebugTaskHandle = NULL;     // 数据回传任务句柄

/* Function ------------------------------------------------------------------*/

/**
* @brief 控制后台任务包装：调用控制计算主体（1ms周期）
* @param dT_ms 任务周期毫秒
* @return 无
*/
static void User_Awalk_Task(uint16_t dT_ms)
{
    Awalk_Control_Task(dT_ms);
}

/**
* @brief 数据回传后台任务：周期上传基本状态供VOFA+观察（20ms周期）
* @param dT_ms 任务周期毫秒
* @return 无
* @note  通道顺序即VOFA+显示顺序，团队按需增删；仅回传，不参与控制。
*/
static void User_Debug_Task(uint16_t dT_ms)
{
    float send_data[USER_DEBUG_CHANNEL_NUM];
    const Motion_Output_t *ms;
    (void)dT_ms;

    send_data[0] = User_Motor_Data.left.output_position;   // 左髋角(前踢正,0~pi)
    send_data[1] = User_Motor_Data.left.output_velocity;   // 左髋速度
    send_data[2] = User_Motor_Data.left.torque;            // 左反馈力矩
    send_data[3] = User_Motor_Data.right.output_position;  // 右髋角(前踢正,0~pi)
    send_data[4] = User_Motor_Data.right.output_velocity;  // 右髋速度
    send_data[5] = User_Motor_Data.right.torque;           // 右反馈力矩
    send_data[6] = User_IMU_Data.user_euler.pitch;         // IMU俯仰角(度)
    send_data[7] = User_IMU_Data.user_euler.roll;          // IMU横滚角(度)
    send_data[8] = (float)User_Motor_Data.left.fault_code; // 左故障码
    send_data[9] = (float)User_Battery_Level;              // 第10通道：电池电量，0~100%

    /* 运动状态回传：状态编码 + 调试量 */
    ms = MotionState_Port_GetOutput();
    send_data[10] = (float)MotionState_Port_GetState();    // 状态: 1静止 2行走 3上坡 4下坡
    send_data[11] = (ms != NULL) ? ms->gait_score : 0.0f;  // 步态活动评分0~1
    send_data[12] = (ms != NULL) ? ms->slope_deg : 0.0f;   // 去平地基线后俯仰坡度(度)
    send_data[13] = (ms != NULL) ? ms->alt_rate_ms : 0.0f; // 气压计高度变化率(m/s)
    send_data[14] = User_Barometer_Data.altitude_m;         // 气压计原始高度(m)，离线调参用

    (void)Transmission_Send(send_data, USER_DEBUG_CHANNEL_NUM);
}

/**
* @brief 用户二次开发主函数，系统就绪后进入并持续运行
* @param 无
* @return 无
* @note  流程：电机模式配置与使能 → 控制初始化 → 注册后台任务。
*        控制算法在awalk_control.c的Awalk_Control_Task中编写；
*        数据回传在User_Debug_Task中按需增删通道。
*/
void User_main(void)
{
    int32_t ret;

    /* 初始化模式与统一输出函数共用同一个开关，避免发送模式不匹配。 */
#if (AWALK_USE_MIT_OUTPUT == 1)
    ret = User_Motor_MIT_Mode_Set(USER_MOTOR_LEFT);
#else
    ret = User_Motor_Mode_Set(USER_MOTOR_LEFT, USER_MOTOR_CONTROL_MODE_TORQUE);
#endif
    if (ret == 0)
    {
        (void)User_Motor_Enable(USER_MOTOR_LEFT);
    }
#if (AWALK_USE_MIT_OUTPUT == 1)
    ret = User_Motor_MIT_Mode_Set(USER_MOTOR_RIGHT);
#else
    ret = User_Motor_Mode_Set(USER_MOTOR_RIGHT, USER_MOTOR_CONTROL_MODE_TORQUE);
#endif
    if (ret == 0)
    {
        (void)User_Motor_Enable(USER_MOTOR_RIGHT);
    }

    HAL_Delay(50U);

    /* 控制算法初始化 */
    Awalk_Control_Init();

    /* 运动模式判断初始化（读取默认阈值并做首帧自动标定） */
    MotionState_Port_Init();

    /* 注册1ms控制任务（优先级23，位于FDCAN数据刷新之后） */
    if (Scheduler_CreateStaticTask(User_Awalk_Task, "AWALK",
                                   USER_AWALK_TASK_PERIOD_MS,
                                   USER_AWALK_TASK_PRIO,
                                   &AwalkTaskHandle) != pdPASS)
    {
        /* 任务创建失败：控制不运行，电机保持零力矩 */
    }

    /* 注册20ms数据回传任务 */
    if (Scheduler_CreateStaticTask(User_Debug_Task, "ADEBUG",
                                   USER_DEBUG_TASK_PERIOD_MS,
                                   USER_DEBUG_TASK_PRIO,
                                   &DebugTaskHandle) != pdPASS)
    {
        /* 回传任务创建失败不影响控制任务运行 */
    }

    while (1)
    {
        /* 前台主循环空闲：控制与回传均由后台任务驱动，低频业务可放此处 */
    }
}
