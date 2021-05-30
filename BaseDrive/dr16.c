#include "config.h"
/* ------ ----------------- Internal Data ----------------------------------- */
volatile unsigned char sbus_rx_buffer[25] = {0};
volatile unsigned short Channel_0 = 1000,Channel_1 = 1000,Channel_2 = 1000,Channel_3 = 1000;
//volatile uint8_t Switch_left = 3,Switch_right = 0,last_swleft = 0,last_swright = 3;
volatile uint8_t rc_data_flag = 0,rc_data_count = 0;
/* ----------------------- Function Implements ---------------------------- */
/******************************************************************************
* @fn RC_Init
*
* @brief configure stm32 usart2 port
* - USART Parameters
* - 100Kbps
* - 8-E-2
* - DMA Mode
* - https://os.mbed.com/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/
* @return None. 
*
* @note This code is fully tested on STM32F405RGT6 Platform, You can port it
* to the other platform.
*/
void RC_Init(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2);
	/* -------------- Configure GPIO ---------------------------------------*/
	{
		GPIO_InitTypeDef gpio;
		USART_InitTypeDef usart2;
		
		gpio.GPIO_Pin = GPIO_Pin_3 ;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &gpio);
		USART_DeInit(USART2);
		usart2.USART_BaudRate = 100000;
		usart2.USART_WordLength = USART_WordLength_8b;
		#if USE_SBUS
		usart2.USART_StopBits = USART_StopBits_2;//sbus
		usart2.USART_Parity = USART_Parity_Even;//sbus
		#endif

		usart2.USART_Mode = USART_Mode_Rx;
		usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART2,&usart2);
		
		#if USE_SBUS
		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//使能串口接收中断
		#endif
		USART_Cmd(USART2,ENABLE);//使能串口
		
		#if USE_SBUS
		NVIC_InitTypeDef nvic;
		nvic.NVIC_IRQChannel = USART2_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 0;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
		#endif
		#if USE_DBUS
		USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
		#endif
	}
}

void DR16_Unlink_Check(void)
{
	rc_data_count++;
	if(rc_data_count>100){//100*20ms
		rc_data_count = 100;
		rc_data_flag = 0;
	}
}

volatile unsigned short left_Switch = 0,left_Wheel = 0,right_Switch = 0,right_Wheel = 0,last_right_Wheel=0;
unsigned char Rev = 0,revnum = 0,headflag = 0,tailflag = 0;
void USART2_IRQHandler(void)            //串口2中断服务程序
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
	{
		Rev = USART_ReceiveData(USART2);	//读取接收到的数据
		if(!headflag)
		{
			if(Rev == 0x0F)
			{
				headflag = 0xff;
			}
		}
		else
		{
			sbus_rx_buffer[revnum] = Rev;
			if(revnum == 23)
			{
			#if USE_10CH_FUTABA
				if((sbus_rx_buffer[23] == 0x02)||(sbus_rx_buffer[23] == 0x00))//0x02 14channel  0x08 0x00 0x04 0x14 0x24 0x34 10channel
			#elif USE_10CH_AT9S
				if((sbus_rx_buffer[23] == 0x80)||(sbus_rx_buffer[23] == 0x00))//0x02 14channel  0x08 0x00 0x04 0x14 0x24 0x34 10channel
			#elif USE_10CH_T8FB
				if((sbus_rx_buffer[23] == 0x02)||(sbus_rx_buffer[23] == 0x00))//0x02 14channel  0x08 0x00 0x04 0x14 0x24 0x34 10channel
			#endif
				{
					tailflag = 0xff;
				}
				else
				{
					headflag = 0;
					tailflag = 0;
					revnum = 0;
				}
			}
			revnum++;
			if(revnum>24)
			{
				revnum =0;
			}
		}
		if(tailflag)
		{
			Channel_0 = (sbus_rx_buffer[0]|((unsigned short)sbus_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
			Channel_1 = (uint16_t)((sbus_rx_buffer[2] << 5) | (sbus_rx_buffer[1] >> 3)) & 0x07ff; //!< Channel 1
			Channel_2 = (uint16_t)(((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2)) | (sbus_rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
			Channel_3 = (uint16_t)((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
			left_Switch =sbus_rx_buffer[9]& 0x0f; 
			right_Switch =(sbus_rx_buffer[6]& 0x0f)+1; 
			left_Wheel = sbus_rx_buffer[10];
			
			headflag = 0;
			tailflag = 0;
			revnum =0;//attention!!!
			if(sbus_rx_buffer[22]==0X0C){
				rc_data_flag = 0;
			}
			else{
				rc_data_count = 0;
				rc_data_flag = 1;
			}
			LED2_TOGGLE();
			#if DEBUG_PC
			u5_printf("Ch0:%d\tCh1:%d\tCh2:%d\tCh3:%d\tp1:%d\tf2:%d\r\n",Channel_0,Channel_1,Channel_2,Channel_3,Switch_left,Switch_right);
			#endif
		}
	}
}
