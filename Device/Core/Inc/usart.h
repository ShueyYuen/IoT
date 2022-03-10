/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void UART_Enable_Receive_IT(void);

void KE1_Clear_AT_Buf(void);

int KE1_Get_NB_Ch(void);
void KE1_Put_Console_Ch(uint8_t ch);
/*
* 函数说明: 向模组发送指定AT命令
* 输入参数
* pcAt : AT命令
* 返回�? 	0 成功 , 其它失败
* int
*/
int KE1_Send_AT(char *pcAt);


/*
* 函数说明: 接收并解析模组返回的AT响应数据
* 输入参数
* pcOut : 输入输出参数, 输入接收数据缓存地址, 输出已经接收的数�??
* puiSz : 输入输出参数, 输入接收缓存朿大�??, 输出已经接收的数据长�??
* uiTimeoutMs : 输入参数, 接收超时(ms)
* 返回�??
* -1 出错
* 0 AT命令响应超时
* 1 AT命令接收到OK响应
* 2 AT命令接收到ERROR响应
* 3 接收到网络已注册提示
* 4 AT命令接收到下发数�??
*/
int KE1_Recv_AT(char *pcOut, unsigned int *puiSz, unsigned int uiTimeoutMs);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
