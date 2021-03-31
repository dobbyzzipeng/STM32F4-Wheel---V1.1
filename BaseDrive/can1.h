#ifndef __CAN1_H__
#define __CAN1_H__
#include "stm32f4xx.h"

extern uint8_t Cabin_Index;
void CAN1_Configuration(unsigned short canid);
void CAN1_TX_PACKET(unsigned int CAN_ID,unsigned char cantxbuf[],unsigned char len);

#endif 
