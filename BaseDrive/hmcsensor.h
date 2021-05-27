#ifndef _HMCSENSOR_H_
#define _HMCSENSOR_H_
#include "stdint.h"

typedef struct{
	uint8_t flag1_8;
	uint8_t flag9_16;
	uint16_t flag;
}T_HMC;
extern T_HMC T_hmcf,T_hmcb;

typedef struct{
	uint8_t tag;//��׼�м�
	uint8_t mid;//ʵ���߶�׼λ��
	int8_t err;//ƫ��
	uint8_t wlineflag;//����߱�־
	uint8_t last_wlineflag;
	uint8_t num;//̽�⵽�ŵ���ĸ���
	uint8_t twol;//�ֲ��־
	uint8_t reserve;
}T_LINE;
extern T_LINE Linef,Lineb;
void Line_Analysis(uint16_t flag,T_LINE *L);
#endif

