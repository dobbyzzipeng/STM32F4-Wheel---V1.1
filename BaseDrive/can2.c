#include "can2.h"
#include "bms.h"
/*----CAN2_TX-----PB6----*/
/*----CAN2_RX-----PB5----*/

void CAN2_Configuration(uint16_t canid)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;//推挽输出
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpio);

    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = CAN2_RX1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
	nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
	
    CAN_DeInit(CAN2);
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
    can.CAN_Prescaler = 12;       //CAN BaudRate   42/(1+9+4)/12=250Kbps
    CAN_Init(CAN2, &can);
    
    can_filter.CAN_FilterNumber = 14;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = (canid+0)<<5;
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0x0000;
	can_filter.CAN_FilterMaskIdLow = 0x0000;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
	
	can_filter.CAN_FilterNumber = 15;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = (canid+1)<<5;
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0x0000;
	can_filter.CAN_FilterMaskIdLow = 0x0000;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
	
	can_filter.CAN_FilterNumber = 16;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = (canid+2)<<5;
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0x0000;
	can_filter.CAN_FilterMaskIdLow = 0x0000;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
	
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE); 
}

unsigned char can2_tx_success_flag=0;
void CAN2_TX_IRQHandler(void) //CAN TX
{
    if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
       can2_tx_success_flag=1;
    }
}
/*************************************************************************
                          CAN2_RX0_IRQHandler
*************************************************************************/

void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;  

    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
	{
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx_message);
		
		uint32_t GET_TOTAL_SOC_id = PROPOTY<<24|GET_TOTAL_SOC<<16|PC_ADDR<<8|BMS_ADDR;
		
		if(rx_message.ExtId == GET_TOTAL_SOC_id)
		{
			Battery_Msg.Voltage=((((uint16_t)rx_message.Data[0])<<8)|rx_message.Data[1])/10.0f;
			Battery_Msg.Current=((((uint16_t)rx_message.Data[4])<<8)|rx_message.Data[5])/10.0f-3000;
			Battery_Msg.Soc=((((uint16_t)rx_message.Data[6])<<8)|rx_message.Data[7])/10.0f;

			//u1_printf("Voltage:%f\t Current:%f\t Soc:%f\r\n",Battery_Msg.Voltage,Battery_Msg.Current,Battery_Msg.Soc);
			
		}
	}
}

/*************************************************************************
                          CAN2_RX0_IRQHandler
*************************************************************************/

void CAN2_RX1_IRQHandler(void)
{
    CanRxMsg rx_message;  

    if (CAN_GetITStatus(CAN2,CAN_IT_FMP1)!= RESET) 
	{
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        CAN_Receive(CAN2, CAN_FIFO1, &rx_message);
		
	}
}

/*************************************************************************
    void CAN2_TX_PACKET(unsigned char CAN_ID,unsigned char cantcbuf[])
描述：CAN2
*************************************************************************/
void CAN2_TX_PACKET(unsigned int CAN_ID,unsigned char cantxbuf[],unsigned char len)
{
    CanTxMsg tx_message;
    uint8_t i = 0,cnt = 0;
	
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = len;          //帧长度
    tx_message.StdId = CAN_ID;      //帧ID为传入参数的CAN_ID
    
	for(i=0;i<len;i++)
	{
		tx_message.Data[i] = cantxbuf[i];
	}

	while(CAN_Transmit(CAN2,&tx_message)==CAN_TxStatus_NoMailBox)
	{
		cnt++;
		if(cnt>5)
		{
			cnt = 0;
			break;
		}
	}
}


//发送扩展帧
void CAN2_TX_EXTID(uint32_t CAN_ID,uint8_t cantxbuf[],uint8_t len)
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

		while(CAN_Transmit(CAN2,&tx_message)==CAN_TxStatus_NoMailBox)
		{
			cnt++;
			if(cnt>5)
			{
				cnt = 0;
				break;
			}
		}
}


