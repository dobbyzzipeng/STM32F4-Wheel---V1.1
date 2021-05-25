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
	uint8_t tag;
	uint8_t mid;
	int8_t err;
	uint8_t wlineflag;
}T_LINE;
extern T_LINE Linef,Lineb;
void Line_Analysis(uint16_t flag,T_LINE *L);
#endif

