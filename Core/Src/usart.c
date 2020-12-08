/**
  ******************************************************************************
  * File Name          : USART.c
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
#include <stdio.h>
#include <string.h>
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
PUTCHAR_PROTOTYPE
{
  // 注意下面第一个参数是&huart1，因为cubemx配置了串�???1自动生成�???
  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
#endif


uint8_t ucUart3Ch = 0;

uint8_t aucUsart3_Rev_Buf[256] = {0};
volatile uint8_t ucUsart3_In = 0, ucUsart3_Out = 0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  //printf("T\r\n");
  if(USART3 == UartHandle->Instance) {
		aucUsart3_Rev_Buf[ucUsart3_In++] = ucUart3Ch;

		HAL_UART_Receive_IT(&huart3, &ucUart3Ch, 1);
	}
}

void UART_Enable_Receive_IT(void)
{
  HAL_UART_Receive_IT(&huart3, &ucUart3Ch, 1);
}

void KE1_Put_Console_Ch(uint8_t ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFF);
}

int KE1_Get_NB_Ch(void)
{
	int ch = -1;
	if(ucUsart3_In != ucUsart3_Out){
		ch = aucUsart3_Rev_Buf[ucUsart3_Out++];
	}
	return ch;
}

void KE1_Clear_AT_Buf(void)
{
	int ch = 0;
	do{
		ch = KE1_Get_NB_Ch();
		if(-1 != ch) {
      KE1_Put_Console_Ch(ch);
    }
	}while(-1 != ch);
}

/*
* 函数说明: 向模组发送指定AT命令
* 输入参数
* pcAt : AT命令
* 返回 int 0 成功 , 其它失败
*/
int KE1_Send_AT(char *pcAt)
{
	//uart3_refresh_buf();  //清空模块返回数据的buff
	int ret = -1;
	HAL_StatusTypeDef sta;
	sta = HAL_UART_Transmit_IT(&huart3, (uint8_t *)pcAt, strlen(pcAt));
	if(HAL_OK != sta){
		printf("uart3 err:%d\n", sta);
	}else{
		ret = 0;
	}
	//printf("\n%s",pcAt); //回显到Console
	return ret;
}

/*
* 函数说明: 接收并解析模组返回的AT响应数据
* 输入参数
* pcOut : 输入输出参数, 输入接收数据缓存地址, 输出已经接收的数�???
* puiSz : 输入输出参数, 输入接收缓存朿大�???, 输出已经接收的数据长�???
* uiTimeoutMs : 输入参数, 接收超时(ms)
* 返回�???
* -1 出错
* 0 AT命令响应超时
* 1 AT命令接收到OK响应
* 2 AT命令接收到ERROR响应
* 3 接收到网络已注册提示
* 4 模组接收到云端下发数�??
*/
int KE1_Recv_AT(char *pcOut, unsigned int *puiSz, unsigned int uiTimeoutMs)
{
	unsigned char OK[] = "OK\r\n", okT = 0, ERROR[] = "ERROR\r\n", errT = 0;
	unsigned char NNMI[] = "+NNMI:", nnmiT = 0;
	//unsigned char QLWEVTIND[]="+QLWEVTIND:3\r\n", qlwev = 0;
	unsigned char CGATT[]="+CGATT:1\r\n", cgattT = 0;

	int ch = -1, len = 0;
	unsigned int uiSz = *puiSz;
	*puiSz = 0;
	if(NULL == pcOut || NULL == puiSz) return -1;
	do{
		ch = KE1_Get_NB_Ch();
		if(-1 != ch){
			if(0xFF == ch) return -2;

			KE1_Put_Console_Ch(ch);

			if(len != uiSz){
				pcOut[len++] = ch;
				*puiSz = len;
				pcOut[len] = 0;
			}

			if(4 != okT){
				if(OK[okT] == ch){
					okT++; if(4 == okT){return 1;}
				}else{
					okT = 0;
				}
			}

			if(7 != errT){
				if(ERROR[errT] == ch){
					errT++; if(7 == errT){return 2;}
				}else{
					errT = 0;
				}
			}

			if(10 != cgattT){
				if(CGATT[cgattT] == ch){
					cgattT++; if(10 == cgattT){return 3;}
				}else{
					cgattT = 0;
				}
			}

			/*if(14 != qlwev){
				if(QLWEVTIND[qlwev] == ch){
					qlwev++; if(14 == qlwev){return 3;}
				}else{
					qlwev = 0;
				}
			}*/


			if(6 != nnmiT){
				if(NNMI[nnmiT] == ch){
					nnmiT++;
				}else{
					nnmiT = 0;
				}
			}else{
				if(0x0A == ch){
					return 4;
				}
			}

		}else{
			if(0 != uiTimeoutMs) {
				uiTimeoutMs--;
				HAL_Delay(1);
				continue;
			}
		}
	}while(uiTimeoutMs);
	return 0;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
