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

#if   (defined ( __CC_ARM ))
  __IO uint32_t VectorTable[48] __attribute__((at(0x20000000)));
  __IO uint32_t IAP_FLAG __attribute__((at(0x200000C0)));
#elif (defined (__ICCARM__))
#pragma location = 0x20000000
  __no_init __IO uint32_t VectorTable[48];
#pragma location = 0x200000C0
  __no_init __IO uint32_t IAP_FLAG;
#elif defined   (  __GNUC__  )
  __IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
#elif defined ( __TASKING__ )
  __IO uint32_t VectorTable[48] __at(0x20000000);
  __IO uint32_t IAP_FLAG __at(0x200000C0);
#endif

/*UART DATA variable*/
#define USART_RX_DATA_SIZE   10
uint8_t USART_Rx_ptr_in = 0;
uint8_t  USART_Rx_Buffer [USART_RX_DATA_SIZE] ={0}; 

/* Private functions ---------------------------------------------------------*/
void RS485_Init(void)	//high : tx , low : rx
{
	#if defined (DMX_ENABLE)
	dmxPinInitial();	
//	dmxEnableTx();
	#endif
	
}

void UARTReceiveStringCheck(void)
{

    USART_Rx_Buffer[USART_Rx_ptr_in++] = USART_ReceiveData(USART2);

	/* To avoid buffer overflow */
	if(USART_Rx_ptr_in == USART_RX_DATA_SIZE)
	{
		USART_Rx_ptr_in = 0;
	}

	#if 1
	if (USART_Rx_Buffer[0]== 'B' &&
		USART_Rx_Buffer[1]== 'O' &&
		USART_Rx_Buffer[2]== 'O' &&
		USART_Rx_Buffer[3]== 'T'	)
	#else
	if (USART_Rx_Buffer[0]== 'B' &&
		USART_Rx_Buffer[1]== 'O' &&
		USART_Rx_Buffer[2]== 'O' &&
		USART_Rx_Buffer[3]== 'T'	&&
		USART_Rx_Buffer[4]== '\0')
	#endif
	{
		PRINTF("A)JUMP to BOOT\r\n");

//		FLASH_If_Init();		
//		Flash_SignReverse();
//		JumpToBootFunction();	

		IAP_FLAG = 0x12345678;
		NVIC_SystemReset();				
	}

	#if 0//if wrong string , clear index since string incorrect
	PRINTF("String INPUT NG , reset INDEX start \r\n");
	memset( USART_Rx_Buffer,0,sizeof( USART_Rx_Buffer));			
	USART_Rx_ptr_in = 0;
	PRINTF("String INPUT NG , reset INDEX finish\r\n");

	#endif
}

void ReMapSRAM(void)
{
	uint32_t i = 0 ;
	
	for(i = 0; i < 48; i++)
	{
		VectorTable[i] = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}

	/* Enable the SYSCFG peripheral clock*/
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SYSCFG, ENABLE); 
	/* Remap SRAM at 0x00000000 */
	SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
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
	TIM_TimeBaseStructure.TIM_Prescaler = 1-1;	
	TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock / 1000)-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	/* TIM Interrupts enable */
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	/* TIMx enable counter */
	TIM_Cmd(TIM6, ENABLE);

	/* Enable the TIMx gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART2_Config(void)	//TX : PA2 , RX : PA3
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
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

	/* Enable USARTy Receive and Transmit interrupts */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
	//USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

	/* The software must wait until TC=1. The TC flag remains cleared during all data
	transfers and it is set by hardware at the last frame’s end of transmission*/	
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{}

	/* NVIC configuration */
	/* Enable the USARRx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 

	/* Enable the USARRx */
	USART_Cmd(USART2, ENABLE);
}

void SYSTICKCONFIG(void)
{
	RCC_GetClocksFreq(&RCC_ClockFreq);
	
	#if 1 //debug
	PRINTF("A)===========================\r\n");
	PRINTF("A)SYSCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.SYSCLK_Frequency);
	PRINTF("A)HCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.HCLK_Frequency);
	PRINTF("A)PCLK_Frequency = %d \r\n" , 		RCC_ClockFreq.PCLK_Frequency);
	PRINTF("A)ADCCLK_Frequency= %d \r\n" , 	RCC_ClockFreq.ADCCLK_Frequency);
	PRINTF("A)CECCLK_Frequency = %d \r\n" , 	RCC_ClockFreq.CECCLK_Frequency);
	PRINTF("A)I2C1CLK_Frequency = %d \r\n" , 	RCC_ClockFreq.I2C1CLK_Frequency);
	PRINTF("A)USART2CLK_Frequency = %d \r\n" , 	RCC_ClockFreq.USART2CLK_Frequency); 
	#endif /*debug*/
	
	/* Setup SysTick Timer for 1ms interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1);
	}
	
	/* Configure the SysTick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x00);
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


