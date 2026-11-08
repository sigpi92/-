/**
  ******************************************************************************
  * @file           : awalk_control.c
  * @brief          : 助力/控制算法模板实现。提供控制入口与数据快照更新框架，
  *                   内置统一力矩输出保护，团队在此框架内实现自己的算法。
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
#include "awalk_control.h"
#include "motion_state_port.h"
#include "DRVEXO_Scheduler.h"
#include "Sys.h"
#include <string.h>
#include <math.h>

/* Define --------------------------------------------------------------------*/

/* Variable ------------------------------------------------------------------*/
/* 助力控制两侧句柄 */
Awalk_Side_Handle awalk_left;
Awalk_Side_Handle awalk_right;

/* 俯仰角滤波结果（rad，供观察与算法使用） */
float awalk_lpf_pitch_value = 0.0f;

/* Function ------------------------------------------------------------------*/

/**
* @brief 控制算法初始化
* @param 无
* @return 无
* @note  User_main中调用一次。此处句柄清零，可在此添加开机零位采样、
*        滤波器初始化、查表/参数初始化等。
*/
void Awalk_Control_Init(void)
{
    memset(&awalk_left, 0, sizeof(awalk_left));
    memset(&awalk_right, 0, sizeof(awalk_right));

    /* TODO: 在此处添加你的初始化逻辑（如开机零位偏置、滤波器、参数表） */
}

/**
* @brief 统一下发双侧力矩，集中执行安全门控、数值检查、限幅与命令方向转换
* @param left_torque 左侧目标力矩，单位N·m，前踢为正
* @param right_torque 右侧目标力矩，单位N·m，前踢为正
* @return 0表示双侧发送启动成功；负值表示发送失败、输入无效或门控关闭
* @note 本函数不阻塞、不改变电机模式；始终尝试向双侧发送。
*       任一异常输入、故障、未就绪、非舒适模式或角度越界均发送双侧零力矩。
*       角度采用电机反馈统一坐标rad：[2,178]度含边界，返回范围内可恢复。
*       右腿只在本函数出口取反；torque_des保持统一坐标。
*       发送错误优先返回；底层已累计发送错误，不重复计数。
*/
int32_t Awalk_Output_Torque(float left_torque, float right_torque)
{
    const struct User_Motor_data_t motor = User_Motor_Data;
    const float torque_limit = AWALK_TORQUE_LIMIT;
    const float angle_min = AWALK_ANGLE_MIN_RAD;
    const float angle_max = AWALK_ANGLE_MAX_RAD;
    int32_t status = LY_OK;
    int32_t left_ret;
    int32_t right_ret;

    /* 非法配置与非有限输入不能进入协议；任一异常均清零双侧。 */
    if ((!isfinite(torque_limit)) || (torque_limit <= 0.0f) ||
        (torque_limit > 15.0f) || (!isfinite(left_torque)) ||
        (!isfinite(right_torque)) || (!isfinite(angle_min)) ||
        (!isfinite(angle_max)) || (angle_min < 0.0f) ||
        (angle_max > 3.14159265359f) || (angle_min >= angle_max) ||
        (!isfinite(motor.left.output_position)) ||
        (!isfinite(motor.right.output_position)))
    {
        left_torque = 0.0f;
        right_torque = 0.0f;
        sys_err.Motor_Control_err_cnt++;
        status = -LY_ERR_INVALID;
    }
    else if ((sys_ready.Left_Motor_Ready == 0U) ||
             (sys_ready.Right_Motor_Ready == 0U) ||
             (motor.left.fault_code != 0U) ||
             (motor.right.fault_code != 0U) ||
             (motor.left.system_mode != USER_MOTOR_SYSTEM_MODE_COMFORT) ||
             (motor.right.system_mode != USER_MOTOR_SYSTEM_MODE_COMFORT) ||
             (motor.left.output_position < angle_min) ||
             (motor.left.output_position > angle_max) ||
             (motor.right.output_position < angle_min) ||
             (motor.right.output_position > angle_max))
    {
        left_torque = 0.0f;
        right_torque = 0.0f;
        status = -LY_ERR_NOT_READY;
    }
    else
    {
        if (left_torque > torque_limit) { left_torque = torque_limit; }
        else if (left_torque < -torque_limit) { left_torque = -torque_limit; }
        if (right_torque > torque_limit) { right_torque = torque_limit; }
        else if (right_torque < -torque_limit) { right_torque = -torque_limit; }
    }

    /* 保留统一坐标目标，便于算法和回传观察；右腿协议命令在发送时取反。 */
    awalk_left.torque_des = left_torque;
    awalk_right.torque_des = right_torque;
#if (AWALK_USE_MIT_OUTPUT == 1)
    left_ret = User_Motor_MIT_Set(USER_MOTOR_LEFT,
                                  0.0f, 0.0f, 0.0f, 0.0f, left_torque);
    right_ret = User_Motor_MIT_Set(USER_MOTOR_RIGHT,
                                   0.0f, 0.0f, 0.0f, 0.0f,
                                   right_torque * AWALK_RIGHT_COMMAND_SIGN);
#else
    left_ret = User_Motor_Torque_Set(USER_MOTOR_LEFT, left_torque);
    right_ret = User_Motor_Torque_Set(USER_MOTOR_RIGHT,
                                      right_torque * AWALK_RIGHT_COMMAND_SIGN);
#endif
    if (left_ret < 0) { return left_ret; }
    if (right_ret < 0) { return right_ret; }
    return status;
}

/**
* @brief 控制周期任务：输入快照、用户算法与统一力矩输出
* @param dT_ms 任务周期，单位ms，由调度器传入
* @return 无
* @note 运行于TIM7 1ms中断上下文，禁止阻塞/延时/长循环。
*       用户算法只写torque_des；每拍末尾必须经过Awalk_Output_Torque。
*/
void Awalk_Control_Task(uint16_t dT_ms)
{
    const struct User_Motor_data_t motor = User_Motor_Data;
    const struct User_IMU_data_t imu = User_IMU_Data;
    (void)dT_ms;

    awalk_left.position = motor.left.output_position;
    awalk_left.velocity = motor.left.output_velocity;
    awalk_left.torque_meas = motor.left.torque;
    awalk_right.position = motor.right.output_position;
    awalk_right.velocity = motor.right.output_velocity;
    awalk_right.torque_meas = motor.right.torque;
    /* pitch单位为度；此变量当前仅做单位转换，未实现低通滤波。 */
    awalk_lpf_pitch_value = imu.user_euler.pitch * 0.01745f;

    /* 运动模式判断：读取快照并更新状态机（静止/行走/上坡/下坡） */
    (void)MotionState_Port_Task(dT_ms);

    /* 每拍先清目标，避免算法分支未赋值时残留上一拍力矩。 */
    awalk_left.torque_des = 0.0f;
    awalk_right.torque_des = 0.0f;
    if ((sys_ready.Left_Motor_Ready != 0U) &&
        (sys_ready.Right_Motor_Ready != 0U) &&
        (motor.left.fault_code == 0U) && (motor.right.fault_code == 0U) &&
        (motor.left.system_mode == USER_MOTOR_SYSTEM_MODE_COMFORT) &&
        (motor.right.system_mode == USER_MOTOR_SYSTEM_MODE_COMFORT) &&
        isfinite(motor.left.output_position) && isfinite(motor.right.output_position) &&
        (motor.left.output_position >= AWALK_ANGLE_MIN_RAD) &&
        (motor.left.output_position <= AWALK_ANGLE_MAX_RAD) &&
        (motor.right.output_position >= AWALK_ANGLE_MIN_RAD) &&
        (motor.right.output_position <= AWALK_ANGLE_MAX_RAD))
    {
        /* ==================== TODO: 在此处编写你的控制算法 ====================
         * 输入：awalk_left/right.position、velocity、torque_meas、俯仰角rad。
         * 输出：awalk_left/right.torque_des，单位N·m，左右均以前踢为正。
         * 只计算目标：不要自行取反右腿，不要直接调用底层电机发送接口。
         * 不要在此return跳过末尾统一输出；停止输出时将双侧目标置零。
         * ================================================================== */
    }

    /* 门控关闭也必须实际发零；发送错误由底层累计，异常输入由输出函数累计。 */
    (void)Awalk_Output_Torque(awalk_left.torque_des, awalk_right.torque_des);
}
