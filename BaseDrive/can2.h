#ifndef __CAN2_H__
#define __CAN2_H__
#include "stm32f4xx.h"

void CAN2_Configuration(uint16_t canid);
void CAN2_TX_PACKET(unsigned int CAN_ID,unsigned char cantxbuf[],unsigned char len);
void CAN2_TX_EXTID(uint32_t CAN_ID,uint8_t cantxbuf[],uint8_t len);
#endif 
