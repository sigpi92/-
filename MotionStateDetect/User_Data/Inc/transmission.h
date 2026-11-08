/**
  ******************************************************************************
  * @file    transmission.h
  * @brief   数据回传封装头文件：封装平台USART1 JustFloat发送接口，供用户层统一调用。
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
#ifndef __TRANSMISSION_H__
#define __TRANSMISSION_H__
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private defines -----------------------------------------------------------*/
/* 单帧最大浮点通道数（与平台User_UART_Send的JUSTFLOAT上限一致） */
#define TRANSMISSION_MAX_CHANNELS    255U
/* 调试/回传任务默认周期（毫秒），VOFA+观察建议≥20ms避免DMA忙 */
#define TRANSMISSION_DEFAULT_PERIOD_MS   20U

/* Exported types ------------------------------------------------------------*/

/* Exported extern variables -------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/* 发送一组浮点通道数据（VOFA+ JustFloat帧格式，帧尾00 00 80 7F由平台自动添加）
   @param data 浮点数组指针
   @param count 通道数量，1~255
   @return 0=发送启动成功；负值=参数无效/DMA忙/发送失败 */
int32_t Transmission_Send(const float *data, uint16_t count);

/* 便捷接口：发送单个浮点值（内部构造单元素数组） */
int32_t Transmission_SendSingle(float value);

#ifdef __cplusplus
}
#endif
#endif   /* __TRANSMISSION_H__ */
