#include "anchor.h"
#include "config.h"
#include "bsp_delay.h"


void Anchor_Gpio_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//
	GPIO_Init(GPIOD, &GPIO_InitStructure);//
	GPIO_ResetBits(GPIOD,GPIO_Pin_8|GPIO_Pin_9);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//
	GPIO_ResetBits(GPIOC,GPIO_Pin_9);
	
}

void Exti_Gpio_Init(void)//�г̿��أ���翪��
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//
	GPIO_Init(GPIOD, &GPIO_InitStructure);//
}

int32_t anchor_pulse = 0;

void ANCHOR_PWR_ON(void)//ê����Դ��
{
	ANCHOR_PWR=1;
}

void ANCHOR_PWR_OFF(void)
{
	ANCHOR_PWR=0;
}

void ANCHOR_DIR_ON(void)//ê������1
{
	ANCHOR_DIR=1;
}

void ANCHOR_DIR_OFF(void)//ê������2
{
	ANCHOR_DIR=0;
}

void ANCHOR_RUN_CW(void)//ê����ת
{
	ANCHOR_DIR_OFF();
	ANCHOR_PWR_ON();	
}

void ANCHOR_RUN_CCW(void)//ê����ת
{
	ANCHOR_DIR_ON();
	ANCHOR_PWR_ON();
}

void ANCHOR_STOP(void)//ê��ֹͣ
{
	ANCHOR_PWR_OFF();
}

void ANCHOR_control(void){
	if(ANCHOR_flag==0){
		 delay_ms(5);
		 if(ANCHOR_flag==0){
			 anchor_pulse=0;
		 }
	 }
	
	 if(Control.ANCHOR_cmd==0){
		ANCHOR_STOP();
	 }
	 if(Control.ANCHOR_cmd==1){
		ANCHOR_RUN_CW();
	 }
	 if(Control.ANCHOR_cmd==2){
		ANCHOR_RUN_CCW();
	 }
}




