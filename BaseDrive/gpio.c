#include "gpio.h"

void GPIO_init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE,ENABLE);//GPIO_A,B,C,D

/******************************光电开关**************************************/
/*
	K1:PC12			
	K2:PC11 		C-10-1
	K3:PA15			C10  	
	K4:PC10			C11
	K5:PD3			
	K6:PD2			C-9
	K7:PD5			C-9-1
	K8:PD4			C-13-1
	K9:PD7			C-13
	K10:PD6			C-14-1
	K11:PB4			C-14
	K12:PB3			c12
	K13:PB6			
	K14:PB5			
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
	GPIO_Init(GPIOB, &GPIO_InitStructure);//B
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_12; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);//C
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);//D
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//A
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//C
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_7; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);//D

/*********************************步进电机*************************************/
/*
	dir
		1:PC7
		2:PC6
		3:PD11
		4:PD10
		5:PE13
		6:PE12
	Pul:
		1:PD15 T4-C4
		2:PD14 T4-C3
		3:PD13 T4-C2
		4:PD12 T4-C1
		5:PE11 T1-C2
		6:PE9  T1-C1
*/
//dir
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);//C
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);//D
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13; 
	GPIO_Init(GPIOE, &GPIO_InitStructure);//E
//pul
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);//复用
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_12|GPIO_Pin_13; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//复用
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_9; 
	GPIO_Init(GPIOE, &GPIO_InitStructure);

/*********************************继电器*************************************/
/*	
	PB7
	PB8
*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);//B
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//B
}