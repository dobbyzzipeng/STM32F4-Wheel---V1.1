#include "can1.h"
#include "bsp_usart.h"
#include "led.h"
#include "pwm.h"

/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/
/*************************************************************************
                          CAN1_Configuration
描述：初始化CAN1配置为1M波特率
*************************************************************************/

void CAN1_Configuration(uint16_t canid)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;//推挽输出
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &gpio);
    
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
	nvic.NVIC_IRQChannel = CAN1_RX1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);    
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 6;   //CAN BaudRate 42/(1+9+4)/6=500Kbps
    CAN_Init(CAN1, &can);
	
	can_filter.CAN_FilterNumber = 0;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = (canid+0)<<5;
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0XFFFF;
	can_filter.CAN_FilterMaskIdLow = 0XFFFE;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
//	
//	can_filter.CAN_FilterNumber = 4;
//	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
//	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
//	can_filter.CAN_FilterIdHigh = (canid+4)<<5;
//	can_filter.CAN_FilterIdLow = 0x0000;
//	can_filter.CAN_FilterMaskIdHigh = 0XFFFF;
//	can_filter.CAN_FilterMaskIdLow = 0XFFFE;
//	can_filter.CAN_FilterFIFOAssignment = 0;
//	can_filter.CAN_FilterActivation=ENABLE;
//	CAN_FilterInit(&can_filter);
//	
//	can_filter.CAN_FilterNumber = 5;
//	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
//	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
//	can_filter.CAN_FilterIdHigh = (canid+5)<<5;
//	can_filter.CAN_FilterIdLow = 0x0000;
//	can_filter.CAN_FilterMaskIdHigh = 0XFFFF;
//	can_filter.CAN_FilterMaskIdLow = 0XFFFE;
//	can_filter.CAN_FilterFIFOAssignment = 0;
//	can_filter.CAN_FilterActivation=ENABLE;
//	CAN_FilterInit(&can_filter);
//	
//	can_filter.CAN_FilterNumber = 6;
//	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
//	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
//	can_filter.CAN_FilterIdHigh = (canid+6)<<5;
//	can_filter.CAN_FilterIdLow = 0x0000;
//	can_filter.CAN_FilterMaskIdHigh = 0XFFFF;
//	can_filter.CAN_FilterMaskIdLow = 0XFFFE;
//	can_filter.CAN_FilterFIFOAssignment = 0;
//	can_filter.CAN_FilterActivation=ENABLE;
//	CAN_FilterInit(&can_filter);
//	
//	can_filter.CAN_FilterNumber = 7;
//	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
//	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
//	can_filter.CAN_FilterIdHigh = (canid+7)<<5;
//	can_filter.CAN_FilterIdLow = 0x0000;
//	can_filter.CAN_FilterMaskIdHigh = 0XFFFF;
//	can_filter.CAN_FilterMaskIdLow = 0XFFFE;
//	can_filter.CAN_FilterFIFOAssignment = 0;
//	can_filter.CAN_FilterActivation=ENABLE;
//	CAN_FilterInit(&can_filter);
//	
//	can_filter.CAN_FilterNumber = 8;                                //固件升级用的CAN中断
//	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
//	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
//	can_filter.CAN_FilterIdHigh = (canid+8)<<5;
//	can_filter.CAN_FilterIdLow = 0x0000;
//	can_filter.CAN_FilterMaskIdHigh = 0XFFFF;
//	can_filter.CAN_FilterMaskIdLow = 0XFFFE;
//	can_filter.CAN_FilterFIFOAssignment = 1;
//	can_filter.CAN_FilterActivation=ENABLE;
//	CAN_FilterInit(&can_filter);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_FMP1,ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}

/*************************************************************************
                          CAN1_TX_IRQHandler
描述：CAN1的发送中断函数
*************************************************************************/
static uint8_t can1_tx_flag = 0;
void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
		can1_tx_flag = 1;
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}
/*************************************************************************
                          CAN1_RX0_IRQHandler
描述：CAN1的接收中断函数
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);

	}
}
/*************************************************************************
                          CAN1_RX1_IRQHandler
描述：CAN1的接收中断函数
*************************************************************************/

void CAN1_RX1_IRQHandler(void)
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP1)!= RESET)
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP1);
        CAN_Receive(CAN1, CAN_FIFO1, &rx_message);
		

   }
}
/*************************************************************************
    void CAN1_TX_PACKET(unsigned char CAN_ID,unsigned char cantcbuf[])
描述：CAN1
*************************************************************************/
void CAN1_TX_PACKET(unsigned int CAN_ID,unsigned char cantxbuf[],unsigned char len)
{
    CanTxMsg tx_message;
    uint8_t i = 0;
	
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = len;          //帧长度
    tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID
    
	for(i=0;i<len;i++)
	{
		tx_message.Data[i] = cantxbuf[i];
	}
#if 1
	while(CAN_Transmit(CAN1,&tx_message)==CAN_TxStatus_NoMailBox && (i<0X7F)) i++;//超时发送失败
#else
	can1_tx_flag = 0;
	CAN_Transmit(CAN1,&tx_message);
	while(can1_tx_flag==0);//等待发送成功
#endif
}

//发送扩展帧
void CAN1_TX_EXTID(uint32_t CAN_ID,uint8_t cantxbuf[],uint8_t len)
{
    CanTxMsg tx_message;
    uint8_t i = 0,cnt = 0;
	
    tx_message.IDE = CAN_ID_EXT;    //扩展帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = len;          //帧长度
    tx_message.ExtId = CAN_ID;      //帧ID为传入参数的CAN_ID
    
	for(i=0;i<len;i++)
	{
		tx_message.Data[i] = cantxbuf[i];
	}

	while(CAN_Transmit(CAN1,&tx_message)==CAN_TxStatus_NoMailBox)
	{
		cnt++;
		if(cnt>5)
		{
			cnt = 0;
			break;
		}
	}
}


