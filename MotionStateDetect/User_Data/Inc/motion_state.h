/**
  ******************************************************************************
  * @file    motion_state.h
  * @brief   人体运动模式判断模块头文件：静止/行走/上坡/下坡状态机。
  *          核心算法与硬件无关，通过 Motion_Input_t 喂入传感器数据；
  *          平台适配见 motion_state_port.h / motion_state_port.c。
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
#ifndef __MOTION_STATE_H__
#define __MOTION_STATE_H__
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/* 运动状态枚举：0初始化 1静止 2平地行走 3上坡 4下坡 5未知 */
typedef enum
{
    MOTION_STATE_INIT = 0,
    MOTION_STATE_STATIC = 1,
    MOTION_STATE_WALKING = 2,
    MOTION_STATE_UPHILL = 3,
    MOTION_STATE_DOWNHILL = 4,
    MOTION_STATE_UNKNOWN = 5
} Motion_State_e;

/* 单拍传感器输入快照（单位见注释） */
typedef struct
{
    float left_hip_angle;      /* 左髋输出角，rad，前踢为正（平台已统一坐标） */
    float left_hip_velocity;   /* 左髋输出角速度，rad/s，前踢为正 */
    float right_hip_angle;     /* 右髋输出角，rad，前踢为正 */
    float right_hip_velocity;  /* 右髋输出角速度，rad/s，前踢为正 */
    float pitch_deg;           /* IMU俯仰角，度 */
    float altitude_m;          /* 气压计高度，m（altitude_valid=1时有效） */
    uint8_t altitude_valid;    /* 高度数据有效标志：1有效，0无效 */
} Motion_Input_t;

/* 算法输出快照（含调试量，便于回传观察与调参） */
typedef struct
{
    Motion_State_e state;      /* 当前运动状态 */
    float activity_rms;        /* 双侧髋角速度均方根，rad/s */
    float swing_amp_rad;       /* 去偏置后髋摆动幅度，rad */
    float norm_swing;          /* 归一化摆动幅度（0~3，相对自适应参考） */
    float gait_score;          /* 步态活动评分（0~1，静止~行走映射） */
    float slope_deg;           /* 去平地基线后的俯仰坡度，度 */
    float alt_rate_ms;         /* 气压计高度变化率，m/s */
    float pitch_filtered_deg;  /* 俯仰低通滤波值，度 */
    float angle_baseline_rad;  /* 自适应髋角基线（双侧均值），rad */
    float pitch_baseline_deg;  /* 自适应俯仰平地基线，度 */
    uint8_t moving;            /* 当前是否判定为行走活动（1/0） */
    uint8_t slope_up;          /* 当前是否判定为上坡倾向（1/0） */
    uint8_t slope_down;        /* 当前是否判定为下坡倾向（1/0） */
} Motion_Output_t;

/* 算法配置：所有阈值集中于此，便于按人群/佩戴方式调整 */
typedef struct
{
    float dt_s;                 /* 采样周期，s；1ms任务填0.001 */
    /* 预处理系数（每拍EMA系数，数值需配合dt_s） */
    float vel_lpf_alpha;        /* 速度平方低通系数，默认0.02 */
    float baseline_alpha;       /* 髋角自适应基线系数，默认0.001 */
    float pitch_lpf_alpha;      /* 俯仰低通系数，默认0.02 */
    float pitch_base_alpha;     /* 俯仰平地基线系数，默认0.0002 */
    float envelope_decay;       /* 摆动包络泄漏衰减，默认0.999 */
    float amp_ref_alpha;        /* 摆动幅度自适应参考系数，默认0.0005 */
    float amp_ref_min_rad;      /* 摆动参考下限，防止除零，默认0.15 */
    float alt_lpf_alpha;        /* 高度变化率低通系数，默认0.002 */
    /* 静止/行走阈值 */
    float vel_static_rms;       /* 低于此速度RMS判静止，默认0.15 rad/s */
    float vel_walk_rms;         /* 高于此速度RMS判行走活动，默认0.35 rad/s */
    float swing_static_rad;     /* 低于此摆动幅度判静止，默认0.10 rad */
    /* 坡度阈值（气压计高度率有效时以其为权威，否则用俯仰） */
    float pitch_up_deg;         /* 俯仰坡度上坡阈值，默认4.0度 */
    float pitch_down_deg;       /* 俯仰坡度下坡阈值，默认-4.0度 */
    float pitch_level_deg;      /* 俯仰平地判定窗口，默认2.0度 */
    float alt_rate_up_ms;       /* 高度上升率上坡进入阈值，默认0.10 m/s */
    float alt_rate_down_ms;     /* 高度下降率下坡进入阈值，默认-0.10 m/s */
    float alt_rate_up_exit_ms;  /* 上坡退出迟滞阈值，默认0.04 m/s */
    float alt_rate_down_exit_ms;/* 下坡退出迟滞阈值，默认-0.04 m/s */
    /* 去抖保持拍数（1ms任务下：300=300ms，80=80ms，200=200ms） */
    uint32_t static_hold;       /* 静止去抖拍数，默认300 */
    uint32_t walk_hold;         /* 行走去抖拍数，默认80 */
    uint32_t slope_hold;        /* 坡度去抖拍数，默认400 */
    uint8_t use_altitude;       /* 是否使用气压计高度率，默认1 */
} Motion_Config_t;

/* 算法句柄：保存全部状态，支持多实例（左右单腿等） */
typedef struct
{
    Motion_Config_t cfg;
    Motion_Output_t out;
    /* 内部状态 */
    float vel_lpf;
    float angle_base_l;
    float angle_base_r;
    float pitch_lpf;
    float pitch_base;
    float env_max_l;
    float env_min_l;
    float env_max_r;
    float env_min_r;
    float amp_ref;
    float alt_prev;
    float alt_rate;
    uint32_t static_cnt;
    uint32_t walk_cnt;
    uint32_t up_cnt;
    uint32_t down_cnt;
    uint8_t inited;
    uint8_t alt_inited;
    Motion_State_e state;
} Motion_Handle_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
* @brief 填充默认配置
* @param cfg 配置指针，不能为NULL
* @return 无
*/
void Motion_DefaultConfig(Motion_Config_t *cfg);

/**
* @brief 初始化算法句柄
* @param h 句柄指针，不能为NULL
* @param cfg 配置指针，传NULL则使用默认配置
* @return 无
* @note  首次喂入数据时会自动标定角度/俯仰基线。
*/
void Motion_Init(Motion_Handle_t *h, const Motion_Config_t *cfg);

/**
* @brief 喂入一帧传感器数据并更新状态机
* @param h 句柄指针
* @param in 输入快照指针
* @return 当前运动状态；参数非法返回MOTION_STATE_UNKNOWN
* @note  非阻塞、无除法异常保护，适合1ms中断上下文调用。
*/
Motion_State_e Motion_Process(Motion_Handle_t *h, const Motion_Input_t *in);

/**
* @brief 获取当前运动状态
* @param h 句柄指针
* @return 当前运动状态；h为NULL返回MOTION_STATE_UNKNOWN
*/
Motion_State_e Motion_GetState(const Motion_Handle_t *h);

/**
* @brief 获取最近一拍的算法输出快照
* @param h 句柄指针
* @return 输出快照指针；h为NULL返回NULL
*/
const Motion_Output_t *Motion_GetOutput(const Motion_Handle_t *h);

/**
* @brief 将运动状态转为可读字符串（仅调试/日志使用）
* @param state 状态枚举
* @return 字符串常量指针
*/
const char *Motion_StateName(Motion_State_e state);

/**
* @brief 运动状态变化回调钩子（默认空实现，用户可覆盖）
* @param new_state 新的运动状态
* @return 无
* @note  状态每发生一次变化调用一次；任何 .c 文件里实现同名函数即可
*        自动覆盖此弱实现，用于联动语音/LED/控制模式等。
*/
void MotionState_OnStateChanged(Motion_State_e new_state);

#ifdef __cplusplus
}
#endif
#endif   /* __MOTION_STATE_H__ */
