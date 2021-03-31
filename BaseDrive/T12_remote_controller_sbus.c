#include "T12_remote_controller_sbus.h"
#include "bsp_usart.h"

/********************USART2 CONFIG****************************/
#define USE_USART2_TX_DMA 1
#define USE_USART2_RX_DMA 1
#define USE_USART2_RX_IDLE 1//使用串口空闲中断接收不定长数据
//串口发送缓存区 	
__align(8) uint8_t USART2_TX_BUF[USART2_MAX_SEND_LEN] = {0};
void usart2_init(unsigned long int baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	#if USE_USART2_TX_DMA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	#endif
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	USART_DeInit(USART2);
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate=baudrate;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	#if USE_SBUS
	USART_InitStructure.USART_StopBits=USART_StopBits_2;
	USART_InitStructure.USART_Parity=USART_Parity_Even;
	#else
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	#endif
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(USART2,&USART_InitStructure);
/* -------------- Configure NVIC ---------------------------------------*/
	#if USE_USART2_RX_DMA
	nvic.NVIC_IRQChannel = DMA1_Stream5_IRQn;//USART2 rx
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
/* -------------- Configure DMA_RX -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA1_Stream5);
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART2_RX_BUF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = sizeof(USART2_RX_BUF);
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream5,&DMA_InitStructure);
		#if !USE_USART2_RX_IDLE
			DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);//DMA传输完成中断
		#endif
		USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
		DMA_Cmd(DMA1_Stream5,ENABLE);
	}
	#endif
		#if (!USE_USART2_RX_DMA)
		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//普通中断
		nvic.NVIC_IRQChannel = USART2_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 0;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
		#endif
		#if USE_USART2_RX_IDLE
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);//关闭接收中断
		USART_ITConfig(USART2, USART_IT_TC,DISABLE);    //关闭发送完成中断
		USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//空闲中断,用于不定长接收
		nvic.NVIC_IRQChannel = USART2_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 0;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
		#endif
		USART_Cmd(USART2,ENABLE);
	
	
		#if USE_USART2_TX_DMA
		nvic.NVIC_IRQChannel = DMA1_Stream6_IRQn;//USART2 tx
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
/* -------------- Configure DMA_TX -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA1_Stream6);
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART2_TX_BUF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_BufferSize = sizeof(USART2_TX_BUF);
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream6,&DMA_InitStructure);
		USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
		DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);//for rx dma run successfully,tx dma interrupt must be open
		DMA_Cmd(DMA1_Stream6,DISABLE);//初始化时要失能。。。不能！！！
	}
	#endif
}

uint8_t USART2_RX_BUF[USART2_MAX_RECV_LEN] = {0};
uint8_t USART2_dma_tx_flag = 0;
uint16_t USART2_dmarx_len = 0;

void USART2_IRQHandler(void)//串口2中断服务程序
{
	uint8_t Res = 0;
	uint8_t clear = clear;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
	{
		Res = USART_ReceiveData(USART2);	//读取接收到的数据
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
	if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)//空闲中断,
	{
		T12SBUS_remote_controller_prase(USART2_RX_BUF);
		
		USART2_dmarx_len = USART2_DMA_RX_LEN(DMA1_Stream5,USART2_MAX_RECV_LEN);//获取数据量
		DMA_Cmd(DMA1_Stream5,DISABLE);//DMA失能
		while(DMA_GetCmdStatus(DMA1_Stream5));//检测是否失能成功，DMA失能时需要等待少许时间才失能成功
		DMA_SetCurrDataCounter(DMA1_Stream5,USART2_dmarx_len);//重新设置DMA数据传输量
		DMA_Cmd(DMA1_Stream5,ENABLE);//DMA重新使能
		clear = USART2->SR;//这两步为清除空闲中断标志
		clear = USART2->DR;//这两步为清除空闲中断标志
	}
}
/**
 * [USART2_DMA_Send 通过DMA向串口发送不定长度的数据，看函数名就知道]
 * @param pbuffer [数据的传递指针]
 * @param size    [数据的长度]
 *  如： USART2_DMA_Send((uint8_t*)data_to_send,_cnt);
 */
void USART2_DMA_Send(uint8_t *pbuffer, uint32_t size)
{
	if(USART2_dma_tx_flag==0){
		DMA_Cmd (DMA1_Stream6,DISABLE);
		while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}
		DMA_MemoryTargetConfig(DMA1_Stream6,(uint32_t)pbuffer,DMA_Memory_0);
		DMA_SetCurrDataCounter(DMA1_Stream6,size);
		DMA_Cmd (DMA1_Stream6,ENABLE);//使能DMA,开始发送
		USART2_dma_tx_flag = 1;//发送中
	}
}

/******************************************************************************
* @fn DMA1_Stream5_IRQHandler
*
* @brief USART2 DMA ISR
*
* @return None.
*
* @note This code is fully tested on STM32F405RGT6 Platform, You can port it
* to the other platform.
*/
void DMA1_Stream5_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
	{
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
		
	}
}

/*
 * [USART2_DMA_RX_LEN 返回串口DMA不定长接收的接收长度]
 * @param  DMAy_Streamx [串口DMA]
 * @param  BufSize      [BUF的长度]
 * @return              [接收数据的长度]
 */
uint16_t USART2_DMA_RX_LEN(DMA_Stream_TypeDef* DMAy_Streamx,uint16_t BufSize)
{
	return (BufSize - DMAy_Streamx->NDTR);
}

void DMA1_Stream6_IRQHandler(void)//USART2 TX DMA
{
	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6))
	{
		USART2_dma_tx_flag = 0;//发送完成
		DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
	}
}

//volatile short left_horizontal = 0,left_vertical = 0,right_horizontal = 0,right_vertical = 0;
//volatile unsigned short left_Switch = 0,left_Wheel = 0,right_Switch = 0,right_Wheel = 0,last_right_Wheel=0;
T_T12SBUS_DATA T12SBUS_DATA = {0};
void T12SBUS_remote_controller_prase(uint8_t buf[100])
{
	if(buf[0]==0X0F)
	{
		T12SBUS_DATA.CH_X2 = (buf[1]|((unsigned short)buf[2] << 8)) & 0x07ff;
		T12SBUS_DATA.CH_Y2 = (uint16_t)((buf[2] >> 3) | (buf[3] << 5)) & 0x07ff;
		T12SBUS_DATA.CH_Y1 = (uint16_t)(((buf[3] >> 6) | (buf[4] << 2)) | (buf[5] << 10)) & 0x07ff;
		T12SBUS_DATA.CH_X1 = (uint16_t)((buf[5] >> 1) | (buf[6] << 7)) & 0x07ff; 
		
		T12SBUS_DATA.SWITCH_E = (uint16_t)(((buf[6] >> 4) | (buf[7] << 4))) & 0x07ff;
		T12SBUS_DATA.SWITCH_G = (uint16_t)((buf[7] >> 7) | (buf[8] << 1) | (buf[9] << 9)) & 0x07ff; 
		T12SBUS_DATA.SWITCH_H = (uint16_t)(((buf[9] >> 2) | (buf[10] << 6))) & 0x07ff;
		T12SBUS_DATA.SWITCH_F = (uint16_t)((buf[10] >> 5) | (buf[11] << 3)) & 0x07ff; //206  502 603
		T12SBUS_DATA.FUN_A_BUTTON = (uint16_t)((buf[12] | (buf[13] << 8))) & 0x07ff;
		T12SBUS_DATA.FUN_B_BUTTON = (uint16_t)((buf[13] >> 3) | (buf[14] << 5)) & 0x07ff;
		T12SBUS_DATA.FUN_C_BUTTON = (uint16_t)((buf[14] >> 6) | (buf[15] << 2) | (buf[16] << 10)) & 0x07ff;
		T12SBUS_DATA.FUN_D_BUTTON = (uint16_t)(((buf[16] >> 1) | (buf[17] << 7))) & 0x07ff;
	}
}
