/**
  ******************************************************************************
  * @file           : motion_state.c
  * @brief          : 人体运动模式判断核心实现：静止/行走/上坡/下坡状态机。
  *                   采用阈值判断、去抖迟滞与自适应归一化，适配多数人群。
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
#include "motion_state.h"
#include <math.h>
#include <string.h>

/* Define --------------------------------------------------------------------*/
#define MOTION_EPS          (1.0e-6f)
#define MOTION_DEFAULT_AMP  (0.40f)

/* Function ------------------------------------------------------------------*/

/**
* @brief 饱和到[lo,hi]区间
* @param x 输入值
* @param lo 下限
* @param hi 上限
* @return 饱和后的值
*/
static float motion_clamp(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

/**
* @brief 填充默认配置（按1ms控制周期）
* @param cfg 配置指针，不能为NULL
* @return 无
*/
void Motion_DefaultConfig(Motion_Config_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }
    memset(cfg, 0, sizeof(Motion_Config_t));
    cfg->dt_s = 0.001f;
    cfg->vel_lpf_alpha = 0.02f;
    cfg->baseline_alpha = 0.001f;
    cfg->pitch_lpf_alpha = 0.02f;
    cfg->pitch_base_alpha = 0.0002f;
    cfg->envelope_decay = 0.999f;
    cfg->amp_ref_alpha = 0.0005f;
    cfg->amp_ref_min_rad = 0.15f;
    cfg->alt_lpf_alpha = 0.00067f;
    cfg->vel_static_rms = 0.15f;
    cfg->vel_walk_rms = 0.35f;
    cfg->swing_static_rad = 0.10f;
    cfg->pitch_up_deg = 4.0f;
    cfg->pitch_down_deg = -4.0f;
    cfg->pitch_level_deg = 2.0f;
    cfg->alt_rate_up_ms = 0.10f;
    cfg->alt_rate_down_ms = -0.10f;
    cfg->alt_rate_up_exit_ms = 0.04f;
    cfg->alt_rate_down_exit_ms = -0.04f;
    cfg->static_hold = 300U;
    cfg->walk_hold = 80U;
    cfg->slope_hold = 400U;
    cfg->use_altitude = 1U;
}

/**
* @brief 初始化算法句柄
* @param h 句柄指针，不能为NULL
* @param cfg 配置指针，传NULL则使用默认配置
* @return 无
*/
void Motion_Init(Motion_Handle_t *h, const Motion_Config_t *cfg)
{
    if (h == NULL)
    {
        return;
    }
    memset(h, 0, sizeof(Motion_Handle_t));
    if (cfg == NULL)
    {
        Motion_DefaultConfig(&h->cfg);
    }
    else
    {
        h->cfg = *cfg;
    }
    h->amp_ref = MOTION_DEFAULT_AMP;
    h->state = MOTION_STATE_INIT;
    h->inited = 0U;
    h->alt_inited = 0U;
}

/**
* @brief 喂入一帧传感器数据并更新状态机
* @param h 句柄指针
* @param in 输入快照指针
* @return 当前运动状态；参数非法返回MOTION_STATE_UNKNOWN
* @note  算法流程：速度RMS判活动 → 自适应去偏置/包络估计摆动 →
*        俯仰坡度(气压计高度率优先) → 去抖迟滞 → 状态机转移。
*/
Motion_State_e Motion_Process(Motion_Handle_t *h, const Motion_Input_t *in)
{
    Motion_Config_t *c;
    float dts;
    float vsq;
    float act;
    float dl;
    float dr;
    float rng_l;
    float rng_r;
    float swing;
    float slope;
    float alt_inst = 0.0f;
    float denom;
    uint8_t moving;
    uint8_t still;
    uint8_t up;
    uint8_t down;
    uint8_t alt_valid;
    uint8_t still_ok;
    uint8_t walk_ok;
    uint8_t up_ok;
    uint8_t down_ok;
    Motion_State_e prev_state;

    if ((h == NULL) || (in == NULL))
    {
        return MOTION_STATE_UNKNOWN;
    }

    c = &h->cfg;
    dts = c->dt_s;
    if (dts <= MOTION_EPS)
    {
        dts = 0.001f;
    }

    /* 首次采样自动标定基线，省去人工零位标定 */
    if (h->inited == 0U)
    {
        h->vel_lpf = 0.0f;
        h->angle_base_l = in->left_hip_angle;
        h->angle_base_r = in->right_hip_angle;
        h->pitch_lpf = in->pitch_deg;
        h->pitch_base = in->pitch_deg;
        h->env_max_l = 0.0f;
        h->env_min_l = 0.0f;
        h->env_max_r = 0.0f;
        h->env_min_r = 0.0f;
        h->alt_prev = in->altitude_m;
        h->alt_rate = 0.0f;
        h->alt_inited = (in->altitude_valid != 0U) ? 1U : 0U;
        h->inited = 1U;
    }

    /* 活动量：双侧髋角速度均方根。左右摆腿反相，用平方均值避免抵消 */
    vsq = 0.5f * (in->left_hip_velocity * in->left_hip_velocity +
                  in->right_hip_velocity * in->right_hip_velocity);
    h->vel_lpf += c->vel_lpf_alpha * (vsq - h->vel_lpf);
    act = sqrtf(h->vel_lpf);

    /* 髋角自适应基线：缓慢跟踪站立零位，适配不同个体初始姿态 */
    h->angle_base_l += c->baseline_alpha * (in->left_hip_angle - h->angle_base_l);
    h->angle_base_r += c->baseline_alpha * (in->right_hip_angle - h->angle_base_r);
    dl = in->left_hip_angle - h->angle_base_l;
    dr = in->right_hip_angle - h->angle_base_r;

    /* 泄漏包络估计去偏置后的摆动幅度（O(1)，适合中断上下文） */
    {
        float leak;
        leak = h->env_max_l * c->envelope_decay + dl * (1.0f - c->envelope_decay);
        h->env_max_l = (dl > leak) ? dl : leak;
        leak = h->env_min_l * c->envelope_decay + dl * (1.0f - c->envelope_decay);
        h->env_min_l = (dl < leak) ? dl : leak;
        leak = h->env_max_r * c->envelope_decay + dr * (1.0f - c->envelope_decay);
        h->env_max_r = (dr > leak) ? dr : leak;
        leak = h->env_min_r * c->envelope_decay + dr * (1.0f - c->envelope_decay);
        h->env_min_r = (dr < leak) ? dr : leak;
    }
    rng_l = h->env_max_l - h->env_min_l;
    rng_r = h->env_max_r - h->env_min_r;
    swing = (rng_l > rng_r) ? rng_l : rng_r;

    moving = (act > c->vel_walk_rms) ? 1U : 0U;
    if (moving != 0U)
    {
        /* 自适应摆动参考：让归一化幅度适配多数人群 */
        h->amp_ref += c->amp_ref_alpha * (swing - h->amp_ref);
        if (h->amp_ref < c->amp_ref_min_rad)
        {
            h->amp_ref = c->amp_ref_min_rad;
        }
    }

    /* 俯仰坡度：先低通，再与自适应平地基线做差 */
    h->pitch_lpf += c->pitch_lpf_alpha * (in->pitch_deg - h->pitch_lpf);
    slope = h->pitch_lpf - h->pitch_base;
    if (fabsf(slope) < c->pitch_level_deg)
    {
        /* 近水平时才更新平地基线，坡道上冻结基线 */
        h->pitch_base += c->pitch_base_alpha * (h->pitch_lpf - h->pitch_base);
    }

    /* 气压计高度变化率：物理上更可靠的坡度信号 */
    if (c->use_altitude != 0U)
    {
        if (h->alt_inited == 0U)
        {
            if (in->altitude_valid != 0U)
            {
                h->alt_prev = in->altitude_m;
                h->alt_inited = 1U;
            }
        }
        else if (in->altitude_valid != 0U)
        {
            alt_inst = (in->altitude_m - h->alt_prev) / dts;
            h->alt_prev = in->altitude_m;
            h->alt_rate += c->alt_lpf_alpha * (alt_inst - h->alt_rate);
        }
    }

    alt_valid = (c->use_altitude != 0U) &&
                (h->alt_inited != 0U) &&
                (in->altitude_valid != 0U);

    /* 坡度方向：高度率有效时以其为权威（带进出迟滞），否则退化为俯仰阈值 */
    if (alt_valid != 0U)
    {
        if (h->state == MOTION_STATE_UPHILL)
        {
            up = (h->alt_rate > c->alt_rate_up_exit_ms) ? 1U : 0U;
        }
        else
        {
            up = (h->alt_rate > c->alt_rate_up_ms) ? 1U : 0U;
        }
        if (h->state == MOTION_STATE_DOWNHILL)
        {
            down = (h->alt_rate < c->alt_rate_down_exit_ms) ? 1U : 0U;
        }
        else
        {
            down = (h->alt_rate < c->alt_rate_down_ms) ? 1U : 0U;
        }
    }
    else
    {
        up = (slope > c->pitch_up_deg) ? 1U : 0U;
        down = (slope < c->pitch_down_deg) ? 1U : 0U;
    }

    still = ((act < c->vel_static_rms) && (swing < c->swing_static_rad)) ? 1U : 0U;

    /* 去抖计数：连续满足才翻转状态，避免瞬时抖动 */
    h->static_cnt = (still != 0U) ? (h->static_cnt + 1U) : 0U;
    h->walk_cnt = (moving != 0U) ? (h->walk_cnt + 1U) : 0U;
    h->up_cnt = (up != 0U) ? (h->up_cnt + 1U) : 0U;
    h->down_cnt = (down != 0U) ? (h->down_cnt + 1U) : 0U;

    still_ok = (h->static_cnt >= c->static_hold) ? 1U : 0U;
    walk_ok = (h->walk_cnt >= c->walk_hold) ? 1U : 0U;
    up_ok = (h->up_cnt >= c->slope_hold) ? 1U : 0U;
    down_ok = (h->down_cnt >= c->slope_hold) ? 1U : 0U;

    prev_state = h->state;

    /* 状态机转移：静止优先；行走时再按坡度细分；否则保持上一状态(迟滞) */
    if (still_ok != 0U)
    {
        h->state = MOTION_STATE_STATIC;
    }
    else if (walk_ok != 0U)
    {
        if (up_ok != 0U)
        {
            h->state = MOTION_STATE_UPHILL;
        }
        else if (down_ok != 0U)
        {
            h->state = MOTION_STATE_DOWNHILL;
        }
        else
        {
            h->state = MOTION_STATE_WALKING;
        }
    }

    /* 状态变化回调（预留输出接口，默认弱实现为空） */
    if (h->state != prev_state)
    {
        MotionState_OnStateChanged(h->state);
    }

    /* 填充输出快照（含调试量） */
    h->out.state = h->state;
    h->out.activity_rms = act;
    h->out.swing_amp_rad = swing;
    h->out.norm_swing = motion_clamp(swing / h->amp_ref, 0.0f, 3.0f);
    denom = c->vel_walk_rms - c->vel_static_rms;
    if (denom > MOTION_EPS)
    {
        h->out.gait_score = motion_clamp((act - c->vel_static_rms) / denom, 0.0f, 1.0f);
    }
    else
    {
        h->out.gait_score = 0.0f;
    }
    h->out.slope_deg = slope;
    h->out.alt_rate_ms = h->alt_rate;
    h->out.pitch_filtered_deg = h->pitch_lpf;
    h->out.angle_baseline_rad = 0.5f * (h->angle_base_l + h->angle_base_r);
    h->out.pitch_baseline_deg = h->pitch_base;
    h->out.moving = moving;
    h->out.slope_up = up;
    h->out.slope_down = down;

    return h->state;
}

/**
* @brief 获取当前运动状态
* @param h 句柄指针
* @return 当前运动状态；h为NULL返回MOTION_STATE_UNKNOWN
*/
Motion_State_e Motion_GetState(const Motion_Handle_t *h)
{
    if (h == NULL)
    {
        return MOTION_STATE_UNKNOWN;
    }
    return h->state;
}

/**
* @brief 获取最近一拍的算法输出快照
* @param h 句柄指针
* @return 输出快照指针；h为NULL返回NULL
*/
const Motion_Output_t *Motion_GetOutput(const Motion_Handle_t *h)
{
    if (h == NULL)
    {
        return NULL;
    }
    return &h->out;
}

/**
* @brief 将运动状态转为可读字符串（仅调试/日志使用）
* @param state 状态枚举
* @return 字符串常量指针
*/
const char *Motion_StateName(Motion_State_e state)
{
    switch (state)
    {
    case MOTION_STATE_INIT:
        return "INIT";
    case MOTION_STATE_STATIC:
        return "STATIC";
    case MOTION_STATE_WALKING:
        return "WALKING";
    case MOTION_STATE_UPHILL:
        return "UPHILL";
    case MOTION_STATE_DOWNHILL:
        return "DOWNHILL";
    default:
        return "UNKNOWN";
    }
}

/**
* @brief 运动状态变化回调钩子：默认空实现，用户可在任意 .c 中覆盖
* @param new_state 新的运动状态
* @return 无
*/
#if defined(__CC_ARM)
__weak void MotionState_OnStateChanged(Motion_State_e new_state)
#elif defined(__GNUC__)
__attribute__((weak)) void MotionState_OnStateChanged(Motion_State_e new_state)
#else
void MotionState_OnStateChanged(Motion_State_e new_state)
#endif
{
    (void)new_state;
}
