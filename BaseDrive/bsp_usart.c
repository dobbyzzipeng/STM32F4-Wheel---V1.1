/*********************************************************************************************************
*
*	模块名称 : 串口
*	文件名称 : bsp_uart.h
*	版    本 : V1.0
*	说    明 : C文件
*	1)采用DMA不定长发送，DMA的不定长接收
*********************************************************************************************************
*/
#include "bsp_usart.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

#define USE_USART1_DMA_TX 1
#define USE_USART1_RX_IDLE 1//是否使用空闲中断

static uint8_t UsartDmaTxFlag = 0;//1:正在发送   0:发送完成
uint8_t USART1_RX_BUF[USART1_REC_LEN] = {0};/* 接收缓冲区 */
uint16_t USART1_RX_STA = 0;
uint16_t usart1_dma_tx_len = 0;
__align(8) uint8_t USART1_TX_BUF[UART1_TX_SIZE] = {0};
/*
*********************************************************************************************************
*	函 数 名: bsp_InitUart
*	功能说明: 初始化串口硬件，并对全局变量赋初值.
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitUart(int badu)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	/* 配置 USART Tx 为复用功能 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* 输出类型为推挽 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* 内部上拉电阻使能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* 配置 USART Rx 为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 第2步： 配置串口硬件参数 */
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = badu;	/* 波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	#if USE_USART1_RX_IDLE == 0
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* 普通接收中断 */
	#else
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//关闭接收中断
	USART_ITConfig(USART1, USART_IT_TC,DISABLE);    //关闭发送完成中断
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	/* 空闲接收中断 */
	#endif
	USART_Cmd(USART1, ENABLE);		/* 使能串口 */
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	#if USE_USART1_RX_IDLE == 1
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	#endif
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	/* -------------- Configure NVIC ---------------------------------------*/
	{
				/* 使能串口1中断 */
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority 		 = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		#if USE_USART1_RX_IDLE == 1
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		#endif
	}
	/* -------------- Configure DMA_RX -----------------------------------------*/
	#if USE_USART1_RX_IDLE == 1
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA2_Stream2);
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART1_RX_BUF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = USART1_REC_LEN;
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
		DMA_Init(DMA2_Stream2,&DMA_InitStructure);
		// DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);//用了该方法后不需要DMA中断
		DMA_Cmd(DMA2_Stream2,ENABLE);
	}
	#endif
	/* -------------- Configure DMA_TX -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA2_Stream7);
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART1_TX_BUF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_BufferSize = UART1_TX_SIZE;
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
		DMA_Init(DMA2_Stream7,&DMA_InitStructure);
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
		DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA2_Stream7,DISABLE);//初始化时要失能。。。不能！！！
	}
}

/*
*********************************************************************************************************
*	函 数 名: USART1_IRQHandler  
*	功能说明: USART中断服务程序
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
extern void DownLoad_prase(uint8_t buf[]);
void USART1_IRQHandler(void)
{
	uint8_t Res = 0,clear = 0;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		Res =USART_ReceiveData(USART1);  		 
	}
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		DownLoad_prase(USART1_RX_BUF);
		usart1_dma_tx_len = USART1_DMA_RX_LEN(DMA2_Stream2,USART1_REC_LEN);
		DMA_Cmd(DMA2_Stream2,DISABLE);//DMA失能
		while(DMA_GetCmdStatus(DMA2_Stream2));//检测是否失能成功，DMA失能时需要等待少许时间才失能成功
		DMA_SetCurrDataCounter(DMA2_Stream2,USART1_REC_LEN);//重新设置数据传输量
		DMA_Cmd(DMA2_Stream2,ENABLE);//DMA重新使能
		clear = USART1->SR;//这两步为清除空闲中断标志
		clear = USART1->DR;//这两步为清除空闲中断标志
	}
}

/*
*********************************************************************************************************
*	函 数 名: fgetc
*	功能说明: 重定义getc函数，这样可以使用getchar函数从串口1输入数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int fgetc(FILE *f)
{
	/* 等待串口1输入数据 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

	return (int)USART_ReceiveData(USART1);
}

/**
 * [USART1_DMA_Send 通过DMA向串口发送不定长度的数据，看函数名就知道]
 * @param pbuffer [数据的传递指针]
 * @param size    [数据的长度]
 *  如： USART1_DMA_Send((uint8_t*)data_to_send,_cnt);
 */
void USART1_DMA_Send(uint8_t *pbuffer, uint32_t size)
{
	if(UsartDmaTxFlag == 0)//发送数据完成后才能开始发送
	{
		DMA_Cmd (DMA2_Stream7,DISABLE);
		while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}
		DMA_MemoryTargetConfig(DMA2_Stream7,(uint32_t)pbuffer,DMA_Memory_0);
		DMA_SetCurrDataCounter(DMA2_Stream7,size);
	 	DMA_Cmd (DMA2_Stream7,ENABLE);//使能DMA,开始发送
		UsartDmaTxFlag=1; //数据发送中
	}
}

void send_data_dma_u1(uint8_t data[100],uint8_t num)
{
	uint8_t x = 0;
	for(x=0;x<num;x++)
	{
		USART1_TX_BUF[x] = data[x];
	}
	USART1_DMA_Send(USART1_TX_BUF,num);
}
/**
 * [u1_printf printf 函数]
 */
void u1_printf(char* fmt,...)
{
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART1_TX_BUF,fmt,ap);
	va_end(ap);
	USART1_DMA_Send(USART1_TX_BUF, strlen((const char*)USART1_TX_BUF));
}

/**
 * [USART1_DMA_RX_LEN 返回串口DMA不定长接收的接收长度]
 * @param  DMAy_Streamx [串口DMA]
 * @param  BufSize      [BUF的长度]
 * @return              [接收数据的长度]
 */
uint16_t USART1_DMA_RX_LEN(DMA_Stream_TypeDef* DMAy_Streamx,uint16_t BufSize)
{
	return (BufSize - DMAy_Streamx->NDTR);
}

void DMA2_Stream2_IRQHandler(void)//USART1  RX
{
	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
	{
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
	}
}


void DMA2_Stream7_IRQHandler(void)//USART1  TX
{
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7))
	{
		UsartDmaTxFlag = 0;//发送数据完成
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
	}
}

#ifdef __cplusplus
}
#endif

