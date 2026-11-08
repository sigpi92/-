/**
  ******************************************************************************
  * @file    awalk_control.h
  * @brief   助力/控制算法模板头文件。提供控制入口、数据快照句柄与统一常量，
  *          团队在此框架内编写自己的控制算法。
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
#ifndef __AWALK_CONTROL_H__
#define __AWALK_CONTROL_H__
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "User_Data.h"

/* Private defines -----------------------------------------------------------*/
/* 控制任务周期：1ms（后台调度器触发，运行于TIM7中断上下文，禁止阻塞/延时） */
#define AWALK_TASK_PERIOD_MS      1U
/* 助力力矩安全限幅（N·m）：电机协议上限15，建议≤14并从小值起步 */
#define AWALK_TORQUE_LIMIT        10.0f
/* 在原角度范围[0,pi]两端各内缩2度，允许范围[2,178]度，含边界。
   判断使用User_Motor_Data.left/right.output_position统一坐标，单位rad。 */
#define AWALK_ANGLE_MARGIN_DEG    2.0f
#define AWALK_ANGLE_MIN_RAD       (AWALK_ANGLE_MARGIN_DEG * 0.01745329252f)
#define AWALK_ANGLE_MAX_RAD       (3.14159265359f - AWALK_ANGLE_MIN_RAD)
/* 控制输出通道：1=MIT模式(kp=kd=0纯前馈力矩)，0=力矩模式 */
#ifndef AWALK_USE_MIT_OUTPUT
#define AWALK_USE_MIT_OUTPUT      1
#endif
#if (AWALK_USE_MIT_OUTPUT != 0) && (AWALK_USE_MIT_OUTPUT != 1)
#error "AWALK_USE_MIT_OUTPUT must be 0 or 1"
#endif
/* 右腿命令方向：用户已确认需要取反；算法输入/输出仍以前踢为正。
   与User_Data.c的反馈坐标换算独立，禁止在算法内再次取反。 */
#define AWALK_RIGHT_COMMAND_SIGN  (-1.0f)

/* Exported types ------------------------------------------------------------*/
/* 单侧髋关节控制句柄：保存控制所需的输入快照与输出。
   任务入口从User_Motor_Data/User_IMU_Data一次性复制，避免后台任务半途改写 */
typedef struct
{
    /* 输入快照 */
    float position;     // 关节位置 rad（统一坐标，本模板未做开机零位偏置）
    float velocity;     // 关节角速度 rad/s
    float torque_meas;  // 电机反馈力矩 N·m
    /* 输出 */
    float torque_des;   // 门控/限幅后的目标力矩 N·m（前踢正，非发送成功确认）
} Awalk_Side_Handle;

/* Exported extern variables -------------------------------------------------*/
extern Awalk_Side_Handle awalk_left;
extern Awalk_Side_Handle awalk_right;
extern float awalk_lpf_pitch_value;   // 俯仰角滤波结果 rad

/* Exported functions prototypes ---------------------------------------------*/
/**
* @brief 初始化控制句柄，用户可补充零位采样等初始化逻辑
* @param 无
* @return 无
* @note User_main中调用一次。
*/
void Awalk_Control_Init(void);
/**
* @brief 执行输入快照、用户算法与统一力矩输出
* @param dT_ms 任务周期，单位ms
* @return 无
* @note 1ms中断任务，禁止阻塞；算法只写统一坐标下的torque_des。
*/
void Awalk_Control_Task(uint16_t dT_ms);

/**
* @brief 统一下发双侧力矩，集中执行门控、限幅与右腿命令取反
* @param left_torque 左侧目标力矩，单位N·m，前踢为正
* @param right_torque 右侧目标力矩，单位N·m，前踢为正，调用者不要取反
* @return 0表示双侧发送启动成功；负值表示发送失败、输入无效或门控关闭
* @note 非阻塞，仅在控制任务中调用；双侧始终尝试发送。
*       目标力矩/反馈角度NaN或Inf、非法保护配置触发双侧零力矩并返回-LY_ERR_INVALID；
*       电机未就绪、故障、非舒适模式或任一角度超出[2,178]度时，
*       双侧发送零力矩并返回-LY_ERR_NOT_READY；回到范围内且其他门控通过即可恢复。
*       角度不做截断，以免掩盖越界；实际机械运动不由零力矩保证停止。
*       发送错误优先返回；底层累计发送错误，本函数不重复计数。
*       torque_des记录门控/限幅后的统一坐标目标，不代表电机已经执行。
*/
int32_t Awalk_Output_Torque(float left_torque, float right_torque);

#ifdef __cplusplus
}
#endif
#endif   /* __AWALK_CONTROL_H__ */
