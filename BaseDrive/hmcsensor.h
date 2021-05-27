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
	uint8_t tag;//对准中间
	uint8_t mid;//实际线对准位置
	int8_t err;//偏差
	uint8_t wlineflag;//横黑线标志
	uint8_t last_wlineflag;
	uint8_t num;//探测到磁导轨的个数
	uint8_t twol;//分叉标志
	uint8_t reserve;
}T_LINE;
extern T_LINE Linef,Lineb;
void Line_Analysis(uint16_t flag,T_LINE *L);
#endif

