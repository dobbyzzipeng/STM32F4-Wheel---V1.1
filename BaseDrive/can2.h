#ifndef __CAN2_H__
#define __CAN2_H__
#include "stm32f4xx.h"

typedef struct{
	uint8_t catch0;
	uint8_t catch1;
	uint8_t puspul0;
	uint8_t puspul1;
	uint8_t updown0;
	uint8_t updown1;
}T_EJECT;
extern T_EJECT Eject;

void CAN2_Configuration(uint16_t canid);
void CAN2_TX_PACKET(unsigned int CAN_ID,unsigned char cantxbuf[],unsigned char len);
void CAN2_TX_EXTID(uint32_t CAN_ID,uint8_t cantxbuf[],uint8_t len);
#endif 
