/* Includes ------------------------------------------------------------------*/
#include "Hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#if defined (DEBUG)
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small PRINTF (option LD Linker->Libraries->Small PRINTF
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART2, (uint8_t) ch);

	/* Loop until transmit data register is empty */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
	{}

	return ch;
}
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef 	RCC_ClockFreq;

//Flash size check
#define FlashSizeDataRegister			(uint32_t)0x1FFFF7CC

#define FlashSizeAmount_C6			(uint32_t)0xFFFF0020
#define FlashSizeAmount_C8			(uint32_t)0xFFFF0040

#define FlashSizeEndPage_C6			(uint32_t)0x08007C00
#define FlashSizeEndPage_C8			(uint32_t)0x0800FC00

//key data 
#define KEY1_Default					(uint32_t)0xFFFFFFFF
#define KEY1_Data					(uint32_t)0xFFFF600D					//to check application exist or not

#define KEY1_Address_C6				(uint32_t)((FlashSizeEndPage_C6|0x3FF) - 0x04 +1)	//0x8007FFC
#define KEY1_Address_C8				(uint32_t)((FlashSizeEndPage_C8|0x3FF) - 0x04 +1)	//0x800FFFC

uint32_t gFlashSizeEndPage = 0;
uint32_t gFlashSizeAmount = 0;
uint32_t gKEY1Address = 0;

//TIMR6 counter 
//extern uint16_t TIM6Counter_1s;
__IO uint16_t TIM6_1s = 0 ;

//Check RS485 tx ready flag
BOOL flag_RS485NoTransmit = FALSE;

//Check flash sign erase flag
BOOL flag_Flash_Sign_Erase = FALSE;

#if(defined ( __CC_ARM ))
  __IO uint32_t NotUse[48] __attribute__((at(0x20000000)));
  __IO uint32_t IAP_FLAG __attribute__((at(0x200000C0)));
//__attribute__((zero_init)) uint32_t IAP_FLAG __attribute__((at(0x200000C0)));
#elif (defined (__ICCARM__))
#pragma location = 0x200000C0
  __no_init __IO uint32_t IAP_FLAG;
#endif

extern pFunction Jump_To_Application;
extern uint32_t JumpAddress;

/* Private functions ---------------------------------------------------------*/
extern void SerialDownload(void);

void DelayPrintf(__IO uint32_t nCount)
{
	/* Decrement nCount value */
	while (nCount != 0)
	{
		nCount--;
	}
}


void RS485_Init(void)	//high : tx , low : rx
{
	#if defined (DMX_ENABLE)
	dmxPinInitial();	
//	dmxEnableTx();
	#endif
	
}

void IAP_MCUCheck(void)
{
	if (*(__IO uint32_t*)FlashSizeDataRegister == FlashSizeAmount_C6)	//32K
	{
		gFlashSizeEndPage = FlashSizeEndPage_C6;
		gFlashSizeAmount = FlashSizeAmount_C6;
		gKEY1Address = KEY1_Address_C6;
	}

	else if (*(__IO uint32_t*)FlashSizeDataRegister == FlashSizeAmount_C8)	//64K
	{
		gFlashSizeEndPage = FlashSizeEndPage_C8;
		gFlashSizeAmount = FlashSizeAmount_C8;
		gKEY1Address = KEY1_Address_C8;
	}
	
	#if 0	//debug
	PRINTF("gFlashSizeEndPage = 0x%8X\r\n",gFlashSizeEndPage);
	PRINTF("gFlashSizeAmount = 0x%8X\r\n",gFlashSizeAmount);
	PRINTF("gKEY1Address = 0x%8X\r\n",gKEY1Address);	
	#endif
}

void IAP_FlashDataDownload(void)
{
	uint8_t writeprotect = 0;
	
	if (FLASH_If_GetWriteProtectionStatus() != 0)  
	{
		/* Disable the write protection */
		writeprotect = FLASH_If_DisableWriteProtection();

		if ( writeprotect== 0)
		{
			/* Launch loading new option bytes */
			FLASH_OB_Launch();
		}
	}
	SerialDownload();	
}

void IAP_FlashSignErase(void)
{
//	uint8_t writeprotect = 0;

//	FLASH_If_Init();

//	if (FLASH_If_GetWriteProtectionStatus() != 0)  
//	{
//		/* Disable the write protection */
//		writeprotect = FLASH_If_DisableWriteProtection();

//		if ( writeprotect== 0)
//		{
//			/* Launch loading new option bytes */
//			FLASH_OB_Launch();
//		}
//	}

	FLASH_ErasePage(gFlashSizeEndPage);

//	FLASH_Lock();
}

void IAP_FlashSignWrite(void)
{
//	uint8_t writeprotect = 0;
	FLASH_Status status = FLASH_COMPLETE;

//	FLASH_If_Init();

//	if (FLASH_If_GetWriteProtectionStatus() != 0)  
//	{
//		/* Disable the write protection */
//		writeprotect = FLASH_If_DisableWriteProtection();

//		if ( writeprotect== 0)
//		{
//			/* Launch loading new option bytes */
//			FLASH_OB_Launch();
//		}
//	}
	
	status = FLASH_ProgramWord(gKEY1Address, KEY1_Data);
	PRINTF("B)FLASH_ProgramWord(%2d)\r\n",status);

//	FLASH_Lock();
}

BOOL IAP_SWResetFlag(void)
{
	return (IAP_FLAG == 0x12345678)?(TRUE):(FALSE);
}

void IAP_IRQYmodemDataValid(void)
{
	if (!flag_RS485NoTransmit)
	{
		TIM6_1s++;

		#if 1
		PRINTF("B)NO Ymodem data , JUMP in %2d sec",(10 - TIM6_1s));
		PRINTF("B)(DataReceive:%d,",flag_YmodemDataReceive);
		PRINTF("SWReset:%d)\r\n",IAP_SWResetFlag());		
		#endif

		if (flag_YmodemDataReceive == TRUE)		//if data coming , reset count flag
		{
			TIM6_1s = 0;

			if (IAP_SWResetFlag()&&
				(flag_Flash_Sign_Erase==FALSE)&&
				(flag_FlashUpateFinish == TRUE))//only erase sign one time
			{
				//must make sure upgrade finish before erase
				IAP_FlashSignErase();
				flag_Flash_Sign_Erase = TRUE;
			}
		}

		if (TIM6_1s == 10)
		{
			flag_RS485NoTransmit = TRUE;
			flag_YmodemDataTimeOut = TRUE;
		}
	}
}

void IAP_JumpToAP(void)
{
	if (((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
	{ 
	  /* Jump to user application */
	  JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
	  Jump_To_Application = (pFunction) JumpAddress;
	  
	  /* Initialize user application's Stack Pointer */
	  __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
	  
	  /* Jump to application */
	  Jump_To_Application();
	}
	else
	{
		IAP_UpgradeFlow();
		Delay(10);
	}
}

void IAP_UpgradeFlow(void)
{	
	FLASH_If_Init();

	#if defined (ERASE_EARLY)
	IAP_FlashSignErase();
	#endif
	
	IAP_FlashDataDownload();
//	PRINTF("B)flag_FlashUpateFinish = %d\r\n",flag_FlashUpateFinish);

	/*
		if flash already write finish 
		1. update KEY1 to (0x600D)
	*/
	
	if (flag_FlashUpateFinish == TRUE)
//	if (1)	//test
	{
		IAP_FlashSignWrite();
		#if 1	//after update finish , should jump to AP
		IAP_JumpToAP();
		#endif
	}
}

/**************
	if (SW reset)
	{
		if (under 10 sec , no Y modem data) ,exit
		if (Ymodem data pass through , erase key to default)
	}

	if (sign!=0x600D)
	{
		UPGRADE flow
		after upgrade , set key to 0x600D
	}
	else
	{
		jump to user code
	}

****************/

void IAP_BootFlow(void)
{
	uint32_t KEY_DataAddress = 0;

	#if 1
	IAP_MCUCheck();
	#endif

	#if 1	//boot from AP
//	PRINTF("Reset Flag: %d\r\n",IAP_SWResetFlag());
	if (IAP_SWResetFlag())
//	if (1)	//test
	{
		IAP_UpgradeFlow();
		Delay(10);		
	}
	#endif

	KEY_DataAddress = gKEY1Address;
	if (((*(__IO uint32_t*) KEY_DataAddress) != KEY1_Data)/*||IAP_SWResetFlag()*/)
//	if (1)	//test
	{
		PRINTF("B)IAP_UpgradeFlow (KEY_Data:0x%8X)\r\n",(*(__IO uint32_t*) KEY_DataAddress));	
		IAP_UpgradeFlow();
		Delay(10);		
	}
	else
	{	
		PRINTF("B)IAP_JumpToAP Flow(KEY data ok)\r\n");
		IAP_JumpToAP();	
	}
}

/*
	TIMxCLK = PCLK1 = HCLK = SystemCoreClock
	TIMx counter clock = TIMxCLK /((Prescaler + 1)*(Period + 1))
	                = 48 MHz / ((1-1 +1)*(48*10^3-1 +1))
	                = 1000 Hz 
     ==> TIMx counter period = 1 ms
*/
void TIM6_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/* TIMx clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	/* TIMx base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 10000);	
	TIM_TimeBaseStructure.TIM_Period = 10000 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	/* TIM Interrupts enable */
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	/* TIMx enable counter */
	TIM_Cmd(TIM6, ENABLE);

	/* Enable the TIMx gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//void USART2_Test(void)
//{
//	__IO uint8_t temp;
//	
//	if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
//	{
//			temp = USART_ReceiveData(USART2);
//			PRINTF("Press KEY : %c \n\r",temp);

//			switch (temp)
//			{

//				case '1' :
//	
//					break;
//					
//				default : 
//					PRINTF("INPUT CMD not support !\r\n");
//					break;
//			}

//	}
//}

void USART2_Config(void)	//TX : PA2 , RX : PA3
{
	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Clock configuration ---------------------------------------------------*/
	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* GPIO configuration ----------------------------------------------------*/
	GPIO_DeInit(GPIOA);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

	/* Configure USARTx_Tx,USARTx_Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART configuration ---------------------------------------------------*/
	USART_DeInit(USART2);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

//	/* Enable USARTy Receive and Transmit interrupts */
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
//	//USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

//	/* The software must wait until TC=1. The TC flag remains cleared during all data
//	transfers and it is set by hardware at the last frame’s end of transmission*/	
//	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
//	{}

//	/* NVIC configuration */
//	/* Enable the USARRx Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
//	NVIC_InitStructure.NVIC_IRQChannelPriority = 2; 
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
//	NVIC_Init(&NVIC_InitStructure); 

	/* Enable the USARRx */
	USART_Cmd(USART2, ENABLE);
}

void SYSTICKCONFIG(void)
{
	RCC_GetClocksFreq(&RCC_ClockFreq);
	
	#if 0 //debug
	PRINTF("B)===========================\r\n");
	PRINTF("B)SYSCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.SYSCLK_Frequency);
	PRINTF("B)HCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.HCLK_Frequency);
	PRINTF("B)PCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.PCLK_Frequency);
	PRINTF("B)ADCCLK_Frequency= %d \r\n" , 	RCC_ClockFreq.ADCCLK_Frequency);
	PRINTF("B)CECCLK_Frequency = %d \r\n" , 	RCC_ClockFreq.CECCLK_Frequency);
	PRINTF("B)I2C1CLK_Frequency = %d \r\n" , 	RCC_ClockFreq.I2C1CLK_Frequency);
	PRINTF("B)USART2CLK_Frequency = %d \r\n" , 	RCC_ClockFreq.USART2CLK_Frequency); 
	#endif /*debug*/
	
	/* Setup SysTick Timer for 1ms interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1);
	}
	
	/* Configure the SysTick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x01);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t uTime)
{ 
	uwTimingDelay = uTime;
	while(uwTimingDelay != 0);
}

//void Delay_s(__IO uint32_t mTime)
//{ 
//	uint32_t i;
//	for(i=0;i<mTime;i++)
//		Delay_ms(1000);
//}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

//currently not use
/*

void SystemClkDelay(void)
{
	uint32_t i;
	i = 0xffff;
	while(i--);
}

void wtPutChar(uint8_t ccc)
{
	UART1_SendData8(ccc);
	// Loop until the end of transmission 
	while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	
}

u16 GetAbsTime(u16 a,u16 b)
{
	u16 c;
	if(a>=b) c=(a-b);
	else c=65535-(b-a);	
	
	return c;
}
*/

void UART_SendByte(uint8_t Data)
{
	USART_SendData(USART2 , (unsigned char)Data);
	while (USART_GetFlagStatus(USART2 , USART_FLAG_TXE)==RESET);
	{
	}
}

void UART_SendString(uint8_t* Data,uint16_t len)
{
	uint16_t i=0;
	for(i=0;i<len;i++ )
	{
		UART_SendByte(Data[i]);
	}
}

void SystemClkDelay(uint32_t u32Delay)
{
	//uint32_t i;
	//i = 0xffff;
	while(u32Delay--);
}


