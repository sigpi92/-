/**
  ******************************************************************************
  * @file           : transmission.c
  * @brief          : 数据回传封装实现：统一封装平台USART1 JustFloat发送。
  *                   团队开发时只需调用Transmission_Send即可回传数据，无需
  *                   关心底层DMA/帧格式细节。
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
#include "transmission.h"
#include "User_Data.h"
#include "Sys.h"

/* Define --------------------------------------------------------------------*/

/* Variable ------------------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

/**
* @brief 发送一组浮点通道数据（VOFA+ JustFloat帧格式）
* @param data 浮点数组指针
* @param count 通道数量，范围1~255
* @return 0=发送启动成功；负值=参数无效/DMA忙/发送失败
* @note  底层USART1为230400/8N1/3.3V TTL，DMA非阻塞发送；
*        帧尾00 00 80 7F由平台User_UART_Send自动添加，无需处理；
*        VOFA+侧选JustFloat协议、通道数=count即可。
*/
int32_t Transmission_Send(const float *data, uint16_t count)
{
    int32_t ret;

    /* 参数与就绪状态检查交给底层User_UART_Send统一处理 */
    ret = User_UART_Send(data, count);
    if (ret < 0)
    {
        /* 底层已累计User_UART_tx_err_cnt，此处仅做失败透传 */
    }
    return ret;
}

/**
* @brief 便捷接口：发送单个浮点值
* @param value 待发送的浮点值
* @return 0=发送启动成功；负值=参数无效/DMA忙/发送失败
*/
int32_t Transmission_SendSingle(float value)
{
    return Transmission_Send(&value, 1U);
}
