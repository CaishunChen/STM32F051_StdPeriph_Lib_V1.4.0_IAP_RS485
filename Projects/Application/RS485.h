/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _RS485_H_
#define _RS485_H_
/*==========================================================
   Compiler Option
==========================================================*/
#define DMX_ENABLE

/*==========================================================
   Include
==========================================================*/


/*==========================================================
   Hardware Configuration
==========================================================*/
#define RS485_IO_PORT_CLK 	RCC_AHBPeriph_GPIOB
#define RS485_IO_PORT 		GPIOB
#define RS485_IO_PIN		GPIO_Pin_13 

#define DMX_IO_PORT_CLK 	RCC_AHBPeriph_GPIOF
#define DMX_IO_PORT 		GPIOF
#define DMX_IO_PIN			GPIO_Pin_7


/*==========================================================
   Defintion/Enum
==========================================================*/


/*==========================================================
   Typedef
==========================================================*/



/*==========================================================
   Varaibale
==========================================================*/

/*==========================================================
   Function Marco
==========================================================*/
#define rs485PinInitial() \
{\
	GPIO_InitTypeDef GPIO_InitStructure;\
	RCC_AHBPeriphClockCmd(RS485_IO_PORT_CLK, ENABLE);\
	GPIO_InitStructure.GPIO_Pin = RS485_IO_PIN;\
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;\
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;\
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;\
	GPIO_Init(RS485_IO_PORT, &GPIO_InitStructure);\
	rs485EnableRx();\
}

#define dmxPinInitial() \
{\
	GPIO_InitTypeDef GPIO_InitStructure;\
	RCC_AHBPeriphClockCmd(DMX_IO_PORT_CLK, ENABLE);\
	GPIO_InitStructure.GPIO_Pin = DMX_IO_PIN;\
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;\
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;\
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;\
	GPIO_Init(DMX_IO_PORT, &GPIO_InitStructure);\
	dmxEnableRx();\
}

#define rs485EnableTx()	RS485_IO_PORT->BSRR |= RS485_IO_PIN
#define rs485EnableRx()	RS485_IO_PORT->BRR |= RS485_IO_PIN
#define dmxEnableTx()	DMX_IO_PORT->BSRR |= DMX_IO_PIN
#define dmxEnableRx()	DMX_IO_PORT->BRR |= DMX_IO_PIN

/*==========================================================
   Function
==========================================================*/

#endif
