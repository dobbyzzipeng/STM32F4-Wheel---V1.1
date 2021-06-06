#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include "usart.h"
#include "bsp_usart.h"
#include "gps.h"
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{
	int handle; 
};

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
}
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (uint8_t) ch;      
	return ch;
}
#endif

/********************USART3 CONFIG****************************/
#define USE_USART3_TX_DMA 1
#define USE_USART3_RX_DMA 1
#define USE_USART3_RX_IDLE 0
//���ڷ��ͻ�����
__align(8) uint8_t USART3_TX_BUF[USART3_MAX_SEND_LEN] = {0};
__align(8) uint8_t USART3_RX_BUF[USART3_MAX_RECV_LEN] = {0};
void usart3_init(unsigned long int baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	#if USE_USART3_TX_DMA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	#endif
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);

	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	USART_DeInit(USART3);
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate=baudrate;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(USART3,&USART_InitStructure);
	#if (!USE_USART3_RX_DMA)
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);//��ͨ�ж�
	nvic.NVIC_IRQChannel = USART3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	#endif
	#if USE_USART3_RX_IDLE
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//�����ж�,���ڲ���������
	nvic.NVIC_IRQChannel = USART3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	#endif
	USART_Cmd(USART3,ENABLE);
/* -------------- Configure NVIC ---------------------------------------*/
		#if USE_USART3_RX_DMA
		nvic.NVIC_IRQChannel = DMA1_Stream1_IRQn;//usart3 rx
		nvic.NVIC_IRQChannelPreemptionPriority = 2;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
/* -------------- Configure DMA_RX -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA1_Stream1);
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART3_RX_BUF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = sizeof(USART3_RX_BUF);
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
		DMA_Init(DMA1_Stream1,&DMA_InitStructure);
		#if !USE_USART3_RX_IDLE
		DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);//DMA��������ж�
		#endif
		USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
		DMA_Cmd(DMA1_Stream1,ENABLE);
	}
		#endif
		
		#if USE_USART3_TX_DMA
		nvic.NVIC_IRQChannel = DMA1_Stream3_IRQn;//usart3 tx
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
/* -------------- Configure DMA_TX -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA1_Stream3);
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART3_TX_BUF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_BufferSize = sizeof(USART3_TX_BUF);
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
		DMA_Init(DMA1_Stream3,&DMA_InitStructure);
		USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
		DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);//for rx dma run successfully,tx dma interrupt must be open
		DMA_Cmd(DMA1_Stream3,DISABLE);//��ʼ��ʱҪʧ�ܡ��������ܣ�����
	}
	#endif
}

uint8_t usart3_dma_tx_flag = 0;
uint16_t usart3_dmarx_len = 0;
//T_USART_REV U3_rev = {0};
void USART3_IRQHandler(void)//����3�жϷ������
{
	uint8_t Res = 0;
	uint8_t clear = clear;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{
		Res = USART_ReceiveData(USART3);	//��ȡ���յ�������
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);

	}
	
	
	if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)//�����ж�,
	{
		gps_data_prase(USART3_RX_BUF);
		usart3_dmarx_len = USART3_DMA_RX_LEN(DMA1_Stream1,USART3_MAX_RECV_LEN);//��ȡ������
		DMA_Cmd(DMA1_Stream1,DISABLE);//DMAʧ��
		while(DMA_GetCmdStatus(DMA1_Stream1));//����Ƿ�ʧ�ܳɹ���DMAʧ��ʱ��Ҫ�ȴ�����ʱ���ʧ�ܳɹ�
		DMA_SetCurrDataCounter(DMA1_Stream1,usart3_dmarx_len);//��������DMA���ݴ�����
		DMA_Cmd(DMA1_Stream1,ENABLE);//DMA����ʹ��
		clear = USART3->SR;//������Ϊ��������жϱ�־
		clear = USART3->DR;//������Ϊ��������жϱ�־
	}
}
/**
 * [USART3_DMA_Send ͨ��DMA�򴮿ڷ��Ͳ������ȵ����ݣ�����������֪��]
 * @param pbuffer [���ݵĴ���ָ��]
 * @param size    [���ݵĳ���]
 *  �磺 USART3_DMA_Send((uint8_t*)data_to_send,_cnt);
 */
void USART3_DMA_Send(uint8_t *pbuffer, uint32_t size)
{
	if(usart3_dma_tx_flag==0){
		DMA_Cmd (DMA1_Stream3,DISABLE);
		while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}
		DMA_MemoryTargetConfig(DMA1_Stream3,(uint32_t)pbuffer,DMA_Memory_0);
		DMA_SetCurrDataCounter(DMA1_Stream3,size);
		DMA_Cmd (DMA1_Stream3,ENABLE);//ʹ��DMA,��ʼ����
		usart3_dma_tx_flag = 1;//������
	}
}

void send_data_dma_u3(uint8_t data[100],uint8_t num)
{
	uint8_t x = 0;
	for(x=0;x<num;x++)
	{
		USART3_TX_BUF[x] = data[x];
	}
	USART3_DMA_Send(USART3_TX_BUF,num);
}
/*
 * [USART3_DMA_RX_LEN ���ش���DMA���������յĽ��ճ���]
 * @param  DMAy_Streamx [����DMA]
 * @param  BufSize      [BUF�ĳ���]
 * @return              [�������ݵĳ���]
 */
uint16_t USART3_DMA_RX_LEN(DMA_Stream_TypeDef* DMAy_Streamx,uint16_t BufSize)
{
	return (BufSize - DMAy_Streamx->NDTR);
}

void DMA1_Stream1_IRQHandler(void)//USART3 RX DMA
{
	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
	{
		gps_data_prase(USART3_RX_BUF);
		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
	}
}

/**
 * [u3_printf printf ����]
 */
void u3_printf(char* fmt,...)
{
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	USART3_DMA_Send(USART3_TX_BUF, strlen((const char*)USART3_TX_BUF));
}

void DMA1_Stream3_IRQHandler(void)//USART3 TX DMA
{
	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3))
	{
		usart3_dma_tx_flag = 0;//�������
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
	}
}

/**************************UART4 TX RX DMA Config !!********************************/
__align(8) uint8_t USART4_TX_BUF[USART4_MAX_SEND_LEN] = {0};
__align(8) uint8_t USART4_RX_BUF[USART4_MAX_RECV_LEN] = {0};
#define USE_USART4_TX_DMA 1
#define USE_USART4_RX_DMA 1
#define USE_USART4_RX_IDLE 1//ʹ�ܿ����ж�
void usart4_init(unsigned long int baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);

	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate=baudrate;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(UART4,&USART_InitStructure);

	#if USE_USART4_RX_DMA
	/* -------------- Configure NVIC ---------------------------------------*/
		nvic.NVIC_IRQChannel = DMA1_Stream2_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 2;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
	/* -------------- Configure DMA_RX -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA1_Stream2);
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART4_RX_BUF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = sizeof(USART4_RX_BUF);
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream2,&DMA_InitStructure);
		#if !USE_USART4_RX_IDLE
		DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);//ʹ��DMA��������жϿ����ж�Ҫʧ��
		#endif
		USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);//ʹ�ܽ���DMA����
		DMA_Cmd(DMA1_Stream2,ENABLE);
	}
		USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);//�رս����ж�
		USART_ITConfig(UART4, USART_IT_TC,DISABLE);    //�رշ�������ж�
		USART_ITConfig(UART4, USART_IT_IDLE, ENABLE); //�򿪴��ڿ����ж�
	#else
		USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);
	#endif
	nvic.NVIC_IRQChannel = UART4_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 3;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	USART_Cmd(UART4,ENABLE);
	
	#if USE_USART4_TX_DMA
	/* -------------- Configure DMA_TX -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA1_Stream4);
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART4_TX_BUF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_BufferSize = sizeof(USART4_TX_BUF);
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
		DMA_Init(DMA1_Stream4,&DMA_InitStructure);
		USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);
		DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);//ʹ�ܷ���DMA��������ж�
		DMA_Cmd(DMA1_Stream4,DISABLE);//��ʼ��ʱҪʧ�ܡ��������ܣ�����
	}
		/* -------------- Configure NVIC ---------------------------------------*/
//	{		UART4 TX
		nvic.NVIC_IRQChannel = DMA1_Stream4_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 2;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
//	}
	#endif
}
extern void NX_Data_prase(uint8_t buf[]);
uint8_t usart4_dma_len = 0;
uint8_t usart4_dma_tx_flag = 0;
//T_USART_REV U4_rev = {0};
void UART4_IRQHandler(void)//����4�жϷ������
{
	uint8_t res = 0,clear = 0;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) 
	{
		res = USART_ReceiveData(UART4);	//��ȡ���յ�������
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	}
	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
	{
		NX_Data_prase(USART3_RX_BUF);
		usart4_dma_len = USART4_DMA_RX_LEN(DMA1_Stream2,USART4_MAX_RECV_LEN);//��ȡ���ݴ�����
		DMA_Cmd(DMA1_Stream2,DISABLE);//DMAʧ��
		while(DMA_GetCmdStatus(DMA1_Stream2));//����Ƿ�ʧ�ܳɹ���DMAʧ��ʱ��Ҫ�ȴ�����ʱ���ʧ�ܳɹ�
		DMA_SetCurrDataCounter(DMA1_Stream2,USART4_MAX_RECV_LEN);//�����������ݴ�����
		DMA_Cmd(DMA1_Stream2,ENABLE);//DMA����ʹ��
		clear = UART4->SR;//������Ϊ��������жϱ�־
		clear = UART4->DR;//������Ϊ��������жϱ�־
	}
}

/*
 * [USART4_DMA_RX_LEN ���ش���DMA���������յĽ��ճ���]
 * @param  DMAy_Streamx [����DMA]
 * @param  BufSize      [BUF�ĳ���]
 * @return              [�������ݵĳ���]
 */
uint16_t USART4_DMA_RX_LEN(DMA_Stream_TypeDef* DMAy_Streamx,uint16_t BufSize)
{
	return (BufSize - DMAy_Streamx->NDTR);
}

/**
 * [USART4_DMA_Send ͨ��DMA�򴮿ڷ��Ͳ������ȵ����ݣ�����������֪��]
 * @param pbuffer [���ݵĴ���ָ��]
 * @param size    [���ݵĳ���]
 *  �磺 USART4_DMA_Send((uint8_t*)data_to_send,_cnt);
 */
void USART4_DMA_Send(uint8_t *pbuffer, uint32_t size)
{
	if(usart4_dma_tx_flag==0)//���ݷ�����ϲ��ܿ�ʼ��һ�η���
	{
		DMA_Cmd (DMA1_Stream4,DISABLE);
		while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}
		DMA_MemoryTargetConfig(DMA1_Stream4,(uint32_t)pbuffer,DMA_Memory_0);
		DMA_SetCurrDataCounter(DMA1_Stream4,size);
		DMA_Cmd (DMA1_Stream4,ENABLE);//ʹ��DMA,��ʼ����
		usart4_dma_tx_flag = 1;//���ݷ�����
	}
}

void send_data_dma_u4(uint8_t data[100],uint8_t num)
{
	uint8_t x = 0;
	for(x=0;x<num;x++)
	{
		USART4_TX_BUF[x] = data[x];
	}
	USART4_DMA_Send(USART4_TX_BUF,num);
}

void DMA1_Stream4_IRQHandler(void)//UART4 TX DMA IRQ
{
	if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4))
	{
		usart4_dma_tx_flag = 0;//���ݷ������
		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_FLAG_TCIF4);		
	}
}

void DMA1_Stream2_IRQHandler(void)//UART4 RX DMA IRQ
{
	if(DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_FLAG_TCIF2);
	}
}

#define USE_USART5_TX_DMA 1
#define USE_USART5_RX_DMA 1
#define USE_USART5_RX_IDLE 1//ʹ�ܿ����ж�
uint8_t USART5_RX_BUF[USART5_MAX_RECV_LEN] = {0};
uint8_t USART5_TX_BUF[USART5_MAX_SEND_LEN] = {0};
void usart5_init(unsigned long int baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef nvic;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);//ʹ��UART5ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); //GPIOA2����ΪUSART2
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5); //GPIOA3����ΪUSART2
	
	//USART5  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIOD2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOD,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 	

   //USART5 ��ʼ������
	USART_InitStructure.USART_BaudRate = baudrate;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(UART5, &USART_InitStructure); //��ʼ������5
	
	#if USE_USART5_RX_DMA
	/* -------------- Configure NVIC ---------------------------------------*/
		nvic.NVIC_IRQChannel = DMA1_Stream0_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 2;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
	/* -------------- Configure DMA_RX -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA1_Stream0);
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART5->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART5_RX_BUF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = sizeof(USART5_RX_BUF);
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream0,&DMA_InitStructure);
		#if !USE_USART5_RX_IDLE
		DMA_ITConfig(DMA1_Stream0,DMA_IT_TC,ENABLE);//ʹ��DMA��������жϣ�ʹ�ÿ����ж�Ҫʧ��
		#endif
		USART_DMACmd(UART5,USART_DMAReq_Rx,ENABLE);//ʹ�ܽ���DMA����
		DMA_Cmd(DMA1_Stream0,ENABLE);
	}
		USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);//�رս����ж�
		USART_ITConfig(UART5, USART_IT_TC,DISABLE);    //�رշ�������ж�
		USART_ITConfig(UART5, USART_IT_IDLE,ENABLE);//ʹ�ܿ����жϣ�����������
	#else
		USART_ClearFlag(UART5, USART_FLAG_TC);
		USART_ITConfig(UART5,USART_IT_RXNE,ENABLE);
	#endif
		nvic.NVIC_IRQChannel = UART5_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 0;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);	
		USART_Cmd(UART5, ENABLE);  //ʹ�ܴ��� 5
	
	#if USE_USART5_TX_DMA
		/* -------------- Configure NVIC ---------------------------------------*/
		nvic.NVIC_IRQChannel = DMA1_Stream7_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 2;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
	/* -------------- Configure DMA_TX -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA1_Stream7);
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART5->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART5_TX_BUF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_BufferSize = sizeof(USART5_TX_BUF);
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
		DMA_Init(DMA1_Stream7,&DMA_InitStructure);
		USART_DMACmd(UART5,USART_DMAReq_Tx,ENABLE);
		DMA_ITConfig(DMA1_Stream7,DMA_IT_TC,ENABLE);//ʹ�ܷ���DMA��������ж�
		DMA_Cmd(DMA1_Stream7,DISABLE);//��ʼ��ʱҪʧ�ܡ��������ܣ�����
	}
	#endif
}

//T_USART_REV U5_rev;
uint8_t usart5_dma_tx_flag = 0,usart5_dma_tx_len = 0;
void UART5_IRQHandler(void)
{
	uint8_t res = 0,clear = 0;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)//���յ�����
	{
		res =USART_ReceiveData(UART5);
	}
	if (USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)//�����ж�
	{
		
		usart5_dma_tx_len = USART5_DMA_RX_LEN(DMA1_Stream0,USART5_MAX_RECV_LEN);
		DMA_Cmd(DMA1_Stream0,DISABLE);//DMAʧ��
		while(DMA_GetCmdStatus(DMA1_Stream0));//����Ƿ�ʧ�ܳɹ���DMAʧ��ʱ��Ҫ�ȴ�����ʱ���ʧ�ܳɹ�
		DMA_SetCurrDataCounter(DMA1_Stream0,USART5_MAX_RECV_LEN);//�����������ݴ�����
		DMA_Cmd(DMA1_Stream0,ENABLE);//DMA����ʹ��
		clear = UART5->SR;//������Ϊ��������жϱ�־
		clear = UART5->DR;//������Ϊ��������жϱ�־
	}
}

void DMA1_Stream7_IRQHandler(void)//UART5 TX DMA IRQ
{
	if(DMA_GetITStatus(DMA1_Stream7, DMA_IT_TCIF7))
	{
		usart5_dma_tx_flag = 0;//���ݷ������
		DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7);
		DMA_ClearITPendingBit(DMA1_Stream7, DMA_FLAG_TCIF7);		
	}
}

/*
 * [USART5_DMA_RX_LEN ���ش���DMA���������յĽ��ճ���]
 * @param  DMAy_Streamx [����DMA]
 * @param  BufSize      [BUF�ĳ���]
 * @return              [�������ݵĳ���]
 */
uint16_t USART5_DMA_RX_LEN(DMA_Stream_TypeDef* DMAy_Streamx,uint16_t BufSize)
{
	return (BufSize - DMAy_Streamx->NDTR);
}

/**
 * [USART5_DMA_Send ͨ��DMA�򴮿ڷ��Ͳ������ȵ����ݣ�����������֪��]
 * @param pbuffer [���ݵĴ���ָ��]
 * @param size    [���ݵĳ���]
 *  �磺 USART5_DMA_Send((uint8_t*)data_to_send,_cnt);
 */
void USART5_DMA_Send(uint8_t *pbuffer, uint32_t size)
{
	if(usart5_dma_tx_flag==0)//���ݷ�����ϲ��ܿ�ʼ��һ�η���
	{
		DMA_Cmd (DMA1_Stream7,DISABLE);
		while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE){}
		DMA_MemoryTargetConfig(DMA1_Stream7,(uint32_t)pbuffer,DMA_Memory_0);
		DMA_SetCurrDataCounter(DMA1_Stream7,size);
		DMA_Cmd (DMA1_Stream7,ENABLE);//ʹ��DMA,��ʼ����
		usart5_dma_tx_flag = 1;//���ݷ�����
	}
}

void send_data_dma_u5(uint8_t data[100],uint8_t num)
{
	uint8_t x = 0;
	for(x=0;x<num;x++)
	{
		USART5_TX_BUF[x] = data[x];
	}
	USART5_DMA_Send(USART5_TX_BUF,num);
}
/**
 * [u5_printf printf ����]
 */
void u5_printf(char* fmt,...)
{
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART5_TX_BUF,fmt,ap);
	va_end(ap);
	USART5_DMA_Send(USART5_TX_BUF, strlen((const char*)USART5_TX_BUF));
}
/******************USART6 TX RX DMA Config !!*****************************/	
__align(8) uint8_t USART6_TX_BUF[USART6_MAX_SEND_LEN] = {0};	  	
__align(8) uint8_t USART6_RX_BUF[USART6_MAX_RECV_LEN] = {0};
#define USE_USART6_TX_DMA 1
#define USE_USART6_RX_DMA 1
#define USE_USART6_RX_IDLE 1
void usart6_init(unsigned long int baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  nvic;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);

	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	USART_DeInit(USART6);
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate=baudrate;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(USART6,&USART_InitStructure);
	#if USE_USART6_RX_DMA
	/* -------------- Configure NVIC ---------------------------------------*/
//	{
		nvic.NVIC_IRQChannel = DMA2_Stream1_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 2;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
//	}
	/* -------------- Configure DMA_RX -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA2_Stream1);
		DMA_InitStructure.DMA_Channel = DMA_Channel_5;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART6_RX_BUF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = sizeof(USART6_RX_BUF);
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA2_Stream1,&DMA_InitStructure);
		#if !USE_USART6_RX_IDLE
		DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,ENABLE);//ʹ��DMA��������ж�
		#endif
		USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);//ʹ�ܽ���DMA����
		DMA_Cmd(DMA2_Stream1,ENABLE);
	}
	#if USE_USART6_RX_IDLE
		USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);//�رս����ж�
		USART_ITConfig(USART6, USART_IT_TC,DISABLE);    //�رշ�������ж�
		USART_ITConfig(USART6, USART_IT_IDLE,ENABLE);//ʹ�ܿ����жϣ�����������
	#else
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//�رս����ж�
		USART_ITConfig(USART6, USART_IT_TC,ENABLE);    //�رշ�������ж�
		USART_ITConfig(USART6, USART_IT_IDLE,DISABLE);//ʹ�ܿ����жϣ�����������
	#endif
		nvic.NVIC_IRQChannel = USART6_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
	#endif
	USART_Cmd(USART6,ENABLE);
	
	#if USE_USART6_TX_DMA
		/* -------------- Configure NVIC ---------------------------------------*/
//	{
		nvic.NVIC_IRQChannel = DMA2_Stream6_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 2;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
//	}
	/* -------------- Configure DMA_TX -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA2_Stream6);
		DMA_InitStructure.DMA_Channel = DMA_Channel_5;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART6_TX_BUF;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_BufferSize = sizeof(USART6_TX_BUF);
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
		DMA_Init(DMA2_Stream6,&DMA_InitStructure);
		USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
		DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);//ʹ�ܷ���DMA��������ж�
		DMA_Cmd(DMA2_Stream6,DISABLE);//��ʼ��ʱҪʧ�ܡ��������ܣ�����
	}
	#endif
}


void reset2isp(void)  //���������λ!!
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

uint16_t usart6_dma_tx_len = 0;
uint8_t usart6_dma_tx_flag = 0;
void USART6_IRQHandler(void)
{
	uint8_t Res = 0,clear = 0;
	if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)//�����ж�
	{
		usart6_dma_tx_len = USART6_DMA_RX_LEN(DMA2_Stream1,USART6_MAX_RECV_LEN);
		//drgrtk
		drgrtk_prase(USART6_RX_BUF,128);
		DMA_Cmd(DMA2_Stream1,DISABLE);//DMAʧ��
		while(DMA_GetCmdStatus(DMA2_Stream1));//����Ƿ�ʧ�ܳɹ���DMAʧ��ʱ��Ҫ�ȴ�����ʱ���ʧ�ܳɹ�
		DMA_SetCurrDataCounter(DMA2_Stream1,USART6_MAX_RECV_LEN);//�����������ݴ�����
		DMA_Cmd(DMA2_Stream1,ENABLE);//DMA����ʹ��
		clear = USART6->SR;//������Ϊ��������жϱ�־
		clear = USART6->DR;//������Ϊ��������жϱ�־
	}
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
	}
}

void DMA2_Stream1_IRQHandler(void)//UART6 RX DMA IRQ
{
	if(DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1))
	{
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1);
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
	}
}

void DMA2_Stream6_IRQHandler(void)//UART6 TX DMA IRQ
{
	if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6))
	{
		usart6_dma_tx_flag = 0;//�������
		DMA_ClearFlag(DMA2_Stream6, DMA_IT_TCIF6);
		DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);
	}
}
/*
 * [USART6_DMA_RX_LEN ���ش���DMA���������յĽ��ճ���]
 * @param  DMAy_Streamx [����DMA]
 * @param  BufSize      [BUF�ĳ���]
 * @return              [�������ݵĳ���]
 */
uint16_t USART6_DMA_RX_LEN(DMA_Stream_TypeDef* DMAy_Streamx,uint16_t BufSize)
{
	return (BufSize - DMAy_Streamx->NDTR);
}
/**
 * [USART6_DMA_Send ͨ��DMA�򴮿ڷ��Ͳ������ȵ����ݣ�����������֪��]
 * @param pbuffer [���ݵĴ���ָ��]
 * @param size    [���ݵĳ���]
 *  �磺 USART6_DMA_Send((uint8_t*)data_to_send,_cnt);
 */
void USART6_DMA_Send(uint8_t *pbuffer, uint32_t size)
{
	if(usart6_dma_tx_flag==0){
		DMA_Cmd (DMA2_Stream6,DISABLE);
		while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE){}
		DMA_MemoryTargetConfig(DMA2_Stream6,(uint32_t)pbuffer,DMA_Memory_0);
		DMA_SetCurrDataCounter(DMA2_Stream6,size);
		DMA_Cmd (DMA2_Stream6,ENABLE);//ʹ��DMA,��ʼ����
		usart6_dma_tx_flag = 1;//������
	}
}

void send_data_dma_u6(uint8_t data[1000],uint8_t num)
{
	uint8_t x = 0;
	SERVER_485_TX();
	for(x=0;x<num;x++)
	{
		USART6_TX_BUF[x] = data[x];
	}
	USART6_DMA_Send(USART6_TX_BUF,num);
//	delay_ms(1);//��Ҫ��ʱ������ɾ��
	SERVER_485_RX();//�л�Ϊ����ģʽ
}


void server_485_init(void)
{
	usart6_init(115200);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	SERVER_485_RX();//Ĭ�Ͻ���ģʽ
}



