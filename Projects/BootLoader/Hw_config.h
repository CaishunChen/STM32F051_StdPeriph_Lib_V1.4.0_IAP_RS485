/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include <string.h>

#include "menu.h"
#include "common.h"

#include "RS485.h"

/* Define config -------------------------------------------------------------*/
#define TRUE			1
#define FALSE           0
#define ON				1
#define OFF				0
#define IN				1
#define OUT				0
typedef unsigned char   BOOL;

#define DEBUG

#if defined (DEBUG)
#include <stdio.h>

#if defined (DMX_ENABLE)
#define PRINTF(...)	\
{	\
	dmxEnableTx();	\
	DelayPrintf(1000);	\
	printf(__VA_ARGS__);	\
	DelayPrintf(1000);	\
	dmxEnableRx();	\
	DelayPrintf(1000);	\
}	\

#else

#define PRINTF(...) printf(__VA_ARGS__)
#endif	/*DMX_ENABLE*/

#else
#define PRINTF(...)
#endif	/*DEBUG*/

/* Macro ---------------------------------------------------------------------*/
/*
#define UartTxPutChar(x)		\
{	\
     UART1_SendData8(x);	\
     while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	\
}*/

/* Exported types ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
void DelayPrintf(__IO uint32_t nCount);
void RS485_Init(void)	;
void IAP_MCU_Check(void);

void IAP_IRQYmodemDataValid(void);

void IAP_BootFlow(void);
void IAP_UpgradeFlow(void);

void TIM6_Config(void);

//void USART2_Test(void);
void USART2_Config(void);     

void SYSTICKCONFIG(void);
void UART_SendByte(uint8_t Data);
void UART_SendString(uint8_t* Data,uint16_t len);
void SystemClkDelay(uint32_t u32Delay);
void Delay(__IO uint32_t uTime);
void TimingDelay_Decrement(void);
/* Exported constants --------------------------------------------------------*/

#endif  /* __HW_CONFIG_H */

