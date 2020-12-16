/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "oled.h"
#include "nbiot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int KE1_Parse_NNMI(char *pcNNMI, char *pcOut)
{
	char ch = 0, lenFlag = 0, dataFlag = 0;
	char acLen[5] = {0};
	int i = 0, pos = 0, dLen = 0;
	//+NNMI:6,010009020000
	for(i=0; i<strlen(pcNNMI); i++){
		ch = pcNNMI[i];
		if(0x0D == ch && 1 == dataFlag) break;

		if(':' == ch){
			lenFlag = 1; pos = 0;
			continue;
		}
		if(',' == ch){
			lenFlag = 0;
			dataFlag = 1; pos = 0;
			dLen = atoi(acLen);
			continue;
		}
		if(1 == lenFlag){
			acLen[pos++] = ch;
		}
		if(1 == dataFlag){
			pcOut[pos++] = ch;
		}
	}
	return dLen;
}

void ascii2hex(char *pcAscii, char *pcHex)
{
	int iDlen = 0, i = 0, pos = 0;
	iDlen = strlen(pcAscii);
	if(128 < iDlen) return;
	for(i=0; i<iDlen; i++){
		sprintf(&pcHex[pos], "%02X", pcAscii[i]);
		pos += 2;
	}
}
extern unsigned char fingerprint, boxstate, autobeep;
// 是否开启自动报警，默认开。指纹认证后十分钟内关闭，或关上盖子关闭
unsigned char low_alarm = 1;
// 认证ing
unsigned char authing = 0;
// 认证过
unsigned char authed = 0;

char acDevInfo[128] = {0}, acHexBuf[256] = {0}, acAtBuf[512] = {0}, acUserCmd[64] = {0};
float tMax, tMin, hMax, hMin, iMax, iMin;

// 网络任务事件
uint32_t nettickstart;

// 控制灯光
unsigned char rstate = 0, gstate = 0, bstate = 0;
typedef enum{
	RED_LIGHT = 0x0020,
	GREEN_LIGHT = 0x0040,
	BLUE_LIGHT = 0x0080
}SELF_LIGHT;
void switch_light(SELF_LIGHT light, int st){
	unsigned char flag = 1;
	switch(light){
	case RED_LIGHT:
		if(rstate == st){
			flag = 0;
		}
		break;
	case GREEN_LIGHT:
		if(gstate == st){
			flag = 0;
		}
		break;
	case BLUE_LIGHT:
		if(bstate == st){
			flag = 0;
		}
		break;
	}
	if (flag == 1){
		HAL_GPIO_TogglePin(GPIOB, ((uint16_t)light));
	}
}


// 延时时间队列，延时记录为毫秒
typedef struct eventlist
{
	uint32_t time;
	void(*p)();
	struct eventlist *next;
} EventList;

EventList * DelayEvent;
// 添加延时行为，单位为毫秒
void addEvent(int microseconds, void(*ev)()){
    EventList* head = DelayEvent;
    
    uint32_t current = HAL_GetTick() + microseconds;
    EventList *node = (EventList *)malloc(sizeof(EventList));
    node->time = current;
    node->p = ev;
    node->next = NULL;

    if (NULL == head)
    {
        DelayEvent = node;
        printf("NULL is here\n");
    } else
    {
        if (head->time > current)
        {
            node->next = head;
            DelayEvent = node;
            return;
        }
        while(head->next != NULL){     //当头节点的指针域不为NULL
            if (head->next->time > current)
            {
                node->next = head->next;
                head->next = node;
                return;
            }
            head = head->next;             //pr指向下一个节点的地址
        }
        head->next = node;
        printf("NULL is not here\n");
    }
}
// 执行延时行为
void executEvent(){
    uint32_t current = HAL_GetTick();
	if (NULL == DelayEvent)
	{
		return;
	}
    if (current >= DelayEvent->time)
    {
        DelayEvent->p();
		EventList *node = DelayEvent;
        DelayEvent = DelayEvent->next;
		free(node);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	const char *pcVersion = "V0.1.7";
	float fTemp = 34.2, fHumi = 0.0;
	unsigned short usLight = 0, usSound = 0, usVoltage = 0;

	int iUserCase = 0, iRet = -1, tryCnt = 0, iSigVal = 0;

	char netFlag = 0, cmdLen = 0, acSigVal[3] = {0};

	unsigned int atLen = 0, dLen = 0, timeout = 1000, upDevFreq = 0, upNetFreq = 0;
	int nbSP = 0, nbCCID = 0, nbSNR = 0;
	unsigned char nbECL = 0;

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	nettickstart = HAL_GetTick();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//	Beep_Switch(1);
//	HAL_Delay(500);
//	Beep_Switch(0);

	UART_Enable_Receive_IT();// 使能串口接收中断，开始接收数???

	OLED_Init(); // 初始化OLDE


	OLED_DrawLogo(); // 显示bmp单色logo图片
	HAL_Delay(2000);

	OLED_ShowKE1(); // 显示 小熊座KE1
	OLED_ShowString(0, 3, (uint8_t *)pcVersion, 6);
	HAL_Delay(2000);
	printf("Hello! i am KE1 south demo %s\r\n",pcVersion);


	KE1_I2C_SHT31(&fTemp, &fHumi);
	printf("T:%0.2f,H:%0.2f\r\n", fTemp, fHumi);

	KE1_Send_AT("AT\r\n"); atLen = sizeof(acAtBuf);
	iRet = KE1_Recv_AT(acAtBuf, &atLen, 1000);

	if(0 == fHumi || 0 == iRet){
		printf("I2C or AT NVIC_SystemReset\n");
		NVIC_SystemReset();//无法读取I2C�??????AT没有响应则MCU 重启
	}

	OLED_Clear();

	tryCnt = 120;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(0 == netFlag){
			OLED_Show_Note(NULL, 1, tryCnt);
			if(0 != tryCnt) {
				tryCnt--;
			}else{
				OLED_Show_Note("reg err", 0, 0);
				if(NB_OK == nbiot_reboot(10)){
					iUserCase = NB_STEP_BUFF_CLEAR;tryCnt = 120;
				}else{
					NVIC_SystemReset();
				}
			}
		}else{
			if(0 < iSigVal && 32 > iSigVal){
				OLED_Show_Note(NULL, 2, iSigVal);
			}else if(99 == iSigVal){
				OLED_Show_Note("no net", 0, 0);
			}
		}

		// 判断温度湿度等是否正常，显示灯
		KE1_I2C_SHT31(&fTemp, &fHumi); // 采集温湿??
		KE1_ADC_Senser_Get(&usLight, &usSound, &usVoltage);//采集光强和噪??
		if(0 < fHumi){
			OLED_ShowT_H(fTemp, fHumi);
		}
		if(fTemp < tMin || fTemp > tMax)
		{
			switch_light(RED_LIGHT, 1);
		} else {
			switch_light(RED_LIGHT, 0);
		}
		if(fHumi < tMin || fHumi > tMax)
		{
			switch_light(BLUE_LIGHT, 1);
		} else {
			switch_light(BLUE_LIGHT, 0);
		}
		if(usLight < iMin || usLight > iMax)
		{
			switch_light(GREEN_LIGHT, 1);
		} else {
			switch_light(GREEN_LIGHT, 0);
		}
		// 判断结束

		// 判断是否需要报警
		if (1 == authing && 1 == fingerprint)
		{
			fingerprint = 0;
			authed = 1;
			authing = 0;
		}
		if (0 == authed)
		{
			if ((1 == autobeep || 1 == boxstate) && 1 == low_alarm)
			{
				if (1 == fingerprint)
				{
					autobeep = 0;
					fingerprint = 0;
					Beep_Switch(0);
				} else {
					Beep_Switch(1);
				}
			}
		} else
		{
			if (0 == boxstate)
			{
				authed = 0;
			}
		}
		if (1 == fingerprint)
		{
			fingerprint = 0;
		}
		// 结束报警检测

		// 执行延时任务
		executEvent();

		timeout = 500;
		switch(iUserCase){
			case NB_STEP_BUFF_CLEAR:
				KE1_Clear_AT_Buf();
				iUserCase = NB_STEP_CHECK_AT;
				continue;
			case NB_STEP_CHECK_AT:
				KE1_Send_AT("ATE1\r\n");// ??启模块AT命令回显功能
				break;
			case NB_STEP_CHECK_REG:
				if(NB_OK == nbiot_check_reg(3)){// ??查模块网络注册情??
					iUserCase = NB_STEP_UP_REG_INFO;
					netFlag = 1;
				}else{
					iUserCase = NB_STEP_STOP_MODULE;
				}
				break;
			case NB_STEP_STOP_MODULE:
				KE1_Send_AT("AT+CFUN=0\r\n"); // 关闭模组
				timeout = 10000;
				break;
			case NB_STEP_SET_COAP:
				KE1_Send_AT("AT+NCDP=180.101.147.115,5683\r\n"); // 设置电信物联网南向接口地??
				break;
			case NB_STEP_START_MODULE:
				KE1_Send_AT("AT+CFUN=1\r\n"); // 启动模块
				timeout = 10000;
				break;
			case NB_STEP_SET_PDP:
				KE1_Send_AT("AT+CGDCONT=1,\"IP\",\"CTNB\"\r\n"); // 设置PDP
				break;
			case NB_STEP_SIM_CHECK: //??查SIM是否存在
				KE1_Send_AT("AT+CIMI\r\n");
				break;
			case NB_STEP_START_REG:
				KE1_Send_AT("AT+CGATT=1\r\n"); // 启动网络附着
				break;
			case NB_STEP_SET_AUTO_REG:
				KE1_Send_AT("AT+QREGSWT=1\r\n"); // 网络自动注册
				break;
			case NB_STEP_WAITING_REG_OK:
				HAL_Delay(1000);
				KE1_Send_AT("AT+CGATT?\r\n"); // �???查网络注册状�???
				break;
			case NB_STEP_UP_REG_INFO:
				if(1 == netFlag && 0 == upNetFreq){
					nbiot_get_signl_val(acSigVal);iSigVal = atoi(acSigVal); printf("Signal:%d\r\n", iSigVal);

					tryCnt = 0;
					OLED_Show_UP_Flag(1);
					/*
					 * 	上报的无线参�??????必须在数据范围内才算有效数据，数据范围要�??????
						1. 信号强度，上报范围应???-140???-40之间
						2. 覆盖等级，上报范围应???0???2之间
						3. 信噪比，上报范围应在-20???30之间
						4. 小区ID，上报范围应???0???2147483647之间
					 * AT样例: AT+NMGS=11,03FFFFFFA608F651550E01
						平台JSON???: {"SignalPower":-90,"CellID":150360405,"SNR":14,"ECL":1}
					 * */
					memset(acAtBuf, 0, sizeof(acAtBuf));
					nbiot_get_nuestats(&nbSP, &nbSNR, &nbCCID, &nbECL);

					printf("Signal:%d, %d, %d, %d\r\n", nbSP, nbCCID, nbSNR, nbECL);

					snprintf(acAtBuf, sizeof(acAtBuf), "AT+NMGS=14,03%08X%08X%08X%02X\r\n", nbSP, nbCCID, nbSNR, nbECL);// 打包模组信号强度参数
					KE1_Send_AT(acAtBuf);// �??????�??????
					upNetFreq = (60*60)*2;
				}

				break;
			case NB_STEP_UP_DEV_INFO:
				tryCnt++;
				if(10 == tryCnt || (1 == netFlag && 0 == upDevFreq)){
					tryCnt = 0;
					nbiot_get_signl_val(acSigVal);iSigVal = atoi(acSigVal); printf("Signal:%d\r\n", iSigVal);
				}

				if(1 == netFlag && 0 == upDevFreq){
					memset(acDevInfo, 0, sizeof(acDevInfo));
					memset(acAtBuf, 0, sizeof(acAtBuf));

					dLen = snprintf(acDevInfo, sizeof(acDevInfo), "{\"T\":\"%0.2f\",\"H\":\"%0.2f\",\"L\":\"%d\",\"S\":\"%d\",\"V\":\"%d\",\"NB\":\"%d\"}", fTemp, fHumi, usLight,usSound,usVoltage,iSigVal);// 打包设备传感器数???
					printf("%s\r\n", acDevInfo);
					if(0 < fHumi){
						OLED_Show_UP_Flag(1);
						ascii2hex(acDevInfo, acHexBuf);
						snprintf(acAtBuf, sizeof(acAtBuf), "AT+NMGS=%d,00%04X%s\r\n", (dLen+3), dLen, acHexBuf);// 打包COAP数据包AT命令
						//printf("%s\r\n", acAtBuf);
						KE1_Send_AT(acAtBuf);
					}
					// 上传事件的时间
					upDevFreq = 5;
				}
				break;
		}

		atLen = sizeof(acAtBuf);
		iRet = KE1_Recv_AT(acAtBuf, &atLen, timeout);
		//printf("RAT:%d-%d\r\n",iRet, timeout);

		if(0 == iRet){//AT命令响应超时
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
		}else if(1 == iRet){//AT命令接收到OK响应
			if(NB_STEP_WAITING_REG_OK > iUserCase){
				iUserCase++;
				//HAL_Delay(1000);
			}
		}else if(2 == iRet){//AT命令接收到ERROR响应
			printf("AT error !\r\n");
			if(NB_STEP_START_MODULE == iUserCase || NB_STEP_SIM_CHECK == iUserCase){
				OLED_Show_Note("NO SIM", 0, 0);
			}
			if(NB_STEP_UP_REG_INFO == iUserCase){
				OLED_Show_Note("NO Reg", 0, 0);
			}
			do{ // 报警
				HAL_Delay(200);
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
			}while(1);

		}else if(3 == iRet){//接收到网络已注册提示
			printf("Net ready !\r\n");
			netFlag = 1;
			OLED_Show_Note("Net OK", 0, 0);
			HAL_Delay(3000);
			iUserCase = NB_STEP_UP_REG_INFO;
		}else if(4 == iRet){//AT命令接收到电信物联网平台下发数据
			printf("%s", acAtBuf);
			memset(acUserCmd, 0, sizeof(acUserCmd));
			cmdLen = KE1_Parse_NNMI(acAtBuf, acUserCmd);

			if(strstr(acUserCmd, "AAAA0000")){
				printf("device info upload successfully\r\n");
				OLED_Show_UP_Flag(0);
			}else if(strstr(acUserCmd, "CCCC0000")){
				printf("Module connectivity upload successfully\r\n");
				iUserCase = NB_STEP_UP_DEV_INFO;
				OLED_Show_UP_Flag(0);
			}else{
				printf("user data[%d]:%s\r\n", cmdLen, acUserCmd);
				/*
				 * 解析用户命令执行对应操作
				 * TO-DO
				 */

				int cmd_num = hex2dec(acUserCmd[6], acUserCmd[7]);
				// printf("````````````````````````````````````````\n%d\n", cmd_num);
				switch (cmd_num)
				{
				case 1: // 授权开始
					authing = 1;
					break;
				case 8: // 打开蜂鸣器
					Beep_Switch(0);
					printf("beep up!\n");
					break;
				case 9: // 关闭蜂鸣器
					Beep_Switch(1);
					printf("beep off!\n");
					break;
				case 10: // 打开自动报警
					autoBeep = 1;
					printf("auto up!\n");
					break;
				case 11: // 关闭自动报警
					autoBeep = 0;
					break;
				case 12: // 更新温度警报值
					float min_tem = (hex2dec(acUserCmd[12], acUserCmd[13]) - 48) * 10.0 + (hex2dec(acUserCmd[14], acUserCmd[15]) - 48) * 1.0;
					float max_tem = (hex2dec(acUserCmd[16], acUserCmd[17]) - 48) * 10.0 + (hex2dec(acUserCmd[18], acUserCmd[19]) - 48) * 1.0;
					tMax = max_tem;
					tMin = min_tem;
					printf("change limit!\n");
					printf("tMin:%.2f   tMax:%.2f\n", tMin, tMax);
					break;
				case 13: // 更新湿度警告
					float min_hum = (hex2dec(acUserCmd[12], acUserCmd[13]) - 48) * 10.0 + (hex2dec(acUserCmd[14], acUserCmd[15]) - 48) * 1.0;
					float max_hum = (hex2dec(acUserCmd[16], acUserCmd[17]) - 48) * 10.0 + (hex2dec(acUserCmd[18], acUserCmd[19]) - 48) * 1.0;
					hMax = max_hum;
					hMin = min_hum;
					printf("change limit!\n");
					printf("hMin:%.2f    hMax:%.2f", hMin, hMax);
					break;
				case 14: // 更新湿度警告
					float min_ins = (hex2dec(acUserCmd[12], acUserCmd[13]) - 48) * 10.0 + (hex2dec(acUserCmd[14], acUserCmd[15]) - 48) * 1.0;
					float max_ins = (hex2dec(acUserCmd[16], acUserCmd[17]) - 48) * 10.0 + (hex2dec(acUserCmd[18], acUserCmd[19]) - 48) * 1.0;
					iMax = max_ins;
					iMin = min_ins;
					printf("change limit!\n");
					printf("iMin:%.2f    iMax:%.2f", iMin, iMax);
					break;
				}

				if (1 == authed)
				{
					OLED_Show_Note("AUTHED", 0, 0);
				}
				if (1 == authing)
				{
					OLED_Show_Note("AUTHING", 0, 0);
				}

				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
				HAL_Delay(1000);
				if(0x33 == acUserCmd[7]){
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
				}else if(0x34 == acUserCmd[7]){
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
				}
				/* 向平台发送命令响????
				 * AT+NMGS=5,02000A000A
				 */
				acUserCmd[1] = '2';
				acUserCmd[6] = '0';acUserCmd[7] = '0';acUserCmd[8] = '0';acUserCmd[9] = '0';acUserCmd[10] = 0;
				printf("CmdResp:%s\r\n", acUserCmd);
				snprintf(acAtBuf, sizeof(acAtBuf), "AT+NMGS=%d,%s\r\n", 5, acUserCmd);// 打包COAP数据包AT命令
				KE1_Send_AT(acAtBuf);
			}

		}else{
			if(-2 == iRet){// UART error
				NVIC_SystemReset();
			}
		}

		if (HAL_GetTick() - nettickstart > 1000){
			nettickstart = HAL_GetTick();
			if(0 != upNetFreq) {
				upNetFreq--;
				if(0 == upNetFreq){iUserCase = NB_STEP_UP_REG_INFO;}
			}
			if(0 != upDevFreq) upDevFreq--;
		}
	}
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_Delay(1000);

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
