/*********************************************************************************************************
*
*	ģ������ : ����
*	�ļ����� : bsp_uart.h
*	��    �� : V1.0
*	˵    �� : C�ļ�
*	1)����DMA���������ͣ�DMA�Ĳ���������
*********************************************************************************************************
*/
#include "bsp_usart.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

#define USE_USART1_DMA_TX 1
#define USE_USART1_RX_IDLE 1//�Ƿ�ʹ�ÿ����ж�

static uint8_t UsartDmaTxFlag = 0;//1:���ڷ���   0:�������
uint8_t USART1_RX_BUF[USART1_REC_LEN] = {0};/* ���ջ����� */
uint16_t USART1_RX_STA = 0;
uint16_t usart1_dma_tx_len = 0;
__align(8) uint8_t USART1_TX_BUF[UART1_TX_SIZE] = {0};
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitUart
*	����˵��: ��ʼ������Ӳ��������ȫ�ֱ�������ֵ.
*	��    ��:  ��
*	�� �� ֵ: ��
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
	/* ���� USART Tx Ϊ���ù��� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* �������Ϊ���� */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* �ڲ���������ʹ�� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* ����ģʽ */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* ���� USART Rx Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ��2���� ���ô���Ӳ������ */
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = badu;	/* ������ */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	#if USE_USART1_RX_IDLE == 0
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* ��ͨ�����ж� */
	#else
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//�رս����ж�
	USART_ITConfig(USART1, USART_IT_TC,DISABLE);    //�رշ�������ж�
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	/* ���н����ж� */
	#endif
	USART_Cmd(USART1, ENABLE);		/* ʹ�ܴ��� */
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	#if USE_USART1_RX_IDLE == 1
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	#endif
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	/* -------------- Configure NVIC ---------------------------------------*/
	{
				/* ʹ�ܴ���1�ж� */
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
		// DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);//���˸÷�������ҪDMA�ж�
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
		DMA_Cmd(DMA2_Stream7,DISABLE);//��ʼ��ʱҪʧ�ܡ��������ܣ�����
	}
}

/*
*********************************************************************************************************
*	�� �� ��: USART1_IRQHandler  
*	����˵��: USART�жϷ������
*	��    ��: ��
*	�� �� ֵ: ��
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
		DMA_Cmd(DMA2_Stream2,DISABLE);//DMAʧ��
		while(DMA_GetCmdStatus(DMA2_Stream2));//����Ƿ�ʧ�ܳɹ���DMAʧ��ʱ��Ҫ�ȴ�����ʱ���ʧ�ܳɹ�
		DMA_SetCurrDataCounter(DMA2_Stream2,USART1_REC_LEN);//�����������ݴ�����
		DMA_Cmd(DMA2_Stream2,ENABLE);//DMA����ʹ��
		clear = USART1->SR;//������Ϊ��������жϱ�־
		clear = USART1->DR;//������Ϊ��������жϱ�־
	}
}

/*
*********************************************************************************************************
*	�� �� ��: fgetc
*	����˵��: �ض���getc��������������ʹ��getchar�����Ӵ���1��������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int fgetc(FILE *f)
{
	/* �ȴ�����1�������� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

	return (int)USART_ReceiveData(USART1);
}

/**
 * [USART1_DMA_Send ͨ��DMA�򴮿ڷ��Ͳ������ȵ����ݣ�����������֪��]
 * @param pbuffer [���ݵĴ���ָ��]
 * @param size    [���ݵĳ���]
 *  �磺 USART1_DMA_Send((uint8_t*)data_to_send,_cnt);
 */
void USART1_DMA_Send(uint8_t *pbuffer, uint32_t size)
{
	if(UsartDmaTxFlag == 0)//����������ɺ���ܿ�ʼ����
	{
		DMA_Cmd (DMA2_Stream7,DISABLE);
		while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}
		DMA_MemoryTargetConfig(DMA2_Stream7,(uint32_t)pbuffer,DMA_Memory_0);
		DMA_SetCurrDataCounter(DMA2_Stream7,size);
	 	DMA_Cmd (DMA2_Stream7,ENABLE);//ʹ��DMA,��ʼ����
		UsartDmaTxFlag=1; //���ݷ�����
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
 * [u1_printf printf ����]
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
 * [USART1_DMA_RX_LEN ���ش���DMA���������յĽ��ճ���]
 * @param  DMAy_Streamx [����DMA]
 * @param  BufSize      [BUF�ĳ���]
 * @return              [�������ݵĳ���]
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
		UsartDmaTxFlag = 0;//�����������
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
	}
}

#ifdef __cplusplus
}
#endif

