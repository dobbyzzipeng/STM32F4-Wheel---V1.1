#include "led.h"
#include "config.h"

void LED_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE,ENABLE);	//使能GPIOC的外设时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;			 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 		 
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			 	 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;			 		 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Warm_Led_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE, ENABLE);
	//24V port
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIO
	GPIO_ResetBits(GPIOE,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6);
	//5V
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO
	GPIO_ResetBits(GPIOD,GPIO_Pin_4|GPIO_Pin_5);
}


void Big_LED_Init(void)
{
	 GPIO_InitTypeDef  GPIO_InitStructure;
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	 //24V port
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	 GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIO
	 GPIO_ResetBits(GPIOE,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6);
}

