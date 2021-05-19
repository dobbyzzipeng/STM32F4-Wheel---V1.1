#include <stdlib.h>
#include <ctype.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "pwm.h"
#include "relay.h"
#include "can1.h"
#include "bsp_delay.h"
#include "ZL5SERVO.h"
#include "dr16.h"
#include "PickPlane.h"
#include "bms.h"

void TIM4_Encoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;    
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	TIM4->CNT = 0X7FFF;
	TIM_Cmd(TIM4, ENABLE); 
}

void Encoder4_Get_CNT(int32_t *pulse)
{
    *pulse += (TIM4->CNT)-0X7FFF;
    TIM4->CNT = 0X7FFF;
}


uint32_t s_screw_cnt = 0;
void EXTI3_IRQHandler(void)
{
	s_screw_cnt++;//脉冲计数
	EXTI_ClearITPendingBit(EXTI_Line3);
}


void EXTIX_Encoder_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//
	GPIO_Init(GPIOB, &GPIO_InitStructure);//

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource3);
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}


unsigned char Task_timer_flag = 0,timer_cnt = 0;
void TIM5_Init(uint16_t arr,uint16_t psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef  nvic;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

	TIM_DeInit(TIM5);
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(TIM5, ENABLE);  
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM5, ENABLE); 
	/* -------------- Configure NVIC ---------------------------------------*/
//{
	nvic.NVIC_IRQChannel = TIM5_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
//}
}

uint16_t speed = 8000;
volatile uint8_t g_cantxquene_index = 0;
void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update))
    {
		timer_cnt++;
		if(timer_cnt>25){
			timer_cnt = 0;
			Task_timer_flag = 1;
		}
		g_cantxquene_index++;
		if(g_cantxquene_index > 16){
			g_cantxquene_index = 1;
		}
		switch(g_cantxquene_index){
			case 1:
				Set_Driver_Vel(0x01,motor1_speed);
			Send_CatchMotor_Speed(CAMOTOR_ID,CAMotor0_Sp,CAMotor1_Sp);
				break;
			case 2:
				Set_Driver_Vel(0x02,motor2_speed);
			Send_PushPullMotor_Speed(PPMOTOR_ID,PPMotor0_Sp,PPMotor1_Sp);
				break;
			case 3:
				Set_Driver_Vel(0x03,motor3_speed);
			Send_UpDownMotor_Speed(UDMOTOR_ID,UDMotor0_Sp,UDMotor1_Sp);
				break;
			case 4:
				Set_Driver_Vel(0x04,motor4_speed);
				break;
	
			case 5:
				Set_Driver_Pos(0x05,omgset_pos1);
				Bms_Get_Soc();
				break;
			case 6:
				Set_Driver_Pos_Speed(0x05,speed);
				break;
			case 7:
				Start_Driver_Pos_Ctr(0x05);
				break;
			case 8:
				Set_Driver_Pos(0x06,omgset_pos2);
//			Send_CatchMotor_Speed(CAMOTOR_ID,CAMotor0_Sp,CAMotor1_Sp);
				break;
			case 9:
				Set_Driver_Pos_Speed(0x06,speed);
//			Send_PushPullMotor_Speed(PPMOTOR_ID,PPMotor0_Sp,PPMotor1_Sp);
				break;
			case 10:
				Start_Driver_Pos_Ctr(0x06);
			Send_UpDownMotor_Speed(UDMOTOR_ID,UDMotor0_Sp,UDMotor1_Sp);
				break;
			case 11:
				Set_Driver_Pos(0x07,omgset_pos3);
				break;
			case 12:
				Set_Driver_Pos_Speed(0x07,speed);
				break;
			case 13:
				Start_Driver_Pos_Ctr(0x07);
				break;
			case 14:
				Set_Driver_Pos(0x08,omgset_pos4);
				break;
			case 15:
				Set_Driver_Pos_Speed(0x08,speed);
				break;
			case 16:
				Start_Driver_Pos_Ctr(0x08);
				break;
			defalut:
				break;		
		}
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    }
} 

/*
	以PWM作为定时器时钟，进行计数，控制脉冲个数
*/
void TIM_Counter_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitTypeStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
    TIM_TimeBaseStructure.TIM_Prescaler=0;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period=0xFFFF;//只有16位
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;//不产生输出
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);//随时可通过软件修改通道的比较值

	TIM_ARRPreloadConfig(TIM2, ENABLE);//自动重装载
	TIM_ARRPreloadConfig(TIM3, ENABLE);//自动重装载

	TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
	TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
	TIM_ITConfig(TIM2,TIM_IT_CC2,ENABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	TIM_ITConfig(TIM3,TIM_IT_CC2,ENABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
	TIM_ITConfig(TIM3,TIM_IT_CC3,ENABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
	TIM_ITConfig(TIM3,TIM_IT_CC4,ENABLE);
	
//	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);//清掉标志位，避免上电进中断
//	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	
	NVIC_InitTypeStruct.NVIC_IRQChannel=TIM3_IRQn;
    NVIC_InitTypeStruct.NVIC_IRQChannelPreemptionPriority=1;
    NVIC_InitTypeStruct.NVIC_IRQChannelSubPriority=3;
    NVIC_InitTypeStruct.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_InitTypeStruct);
	
	NVIC_InitTypeStruct.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_Init(&NVIC_InitTypeStruct);

	TIM_SelectInputTrigger(TIM2,TIM_TS_ITR0);
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);
    TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(TIM3,TIM_TS_ITR3);
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_External1);
    TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);
	
	TIM_Cmd(TIM2,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
}

void TIM_PWM_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	TIM_DeInit(TIM1);
	TIM_DeInit(TIM4);
	
	TIM_TimeBaseStructure.TIM_Prescaler=83;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=1599;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //不分频
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler=83;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=3199;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //不分频
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM1, ENABLE); 
	TIM_Cmd(TIM4, ENABLE); 
	
	TIM_SelectOutputTrigger(TIM4,TIM_TRGOSource_Update); 
    TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);
	TIM_SelectOutputTrigger(TIM1,TIM_TRGOSource_Update); 
    TIM_SelectMasterSlaveMode(TIM1,TIM_MasterSlaveMode_Enable);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM4, ENABLE);
}


