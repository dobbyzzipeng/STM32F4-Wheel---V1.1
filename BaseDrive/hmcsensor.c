#include "hmcsensor.h"

T_HMC T_hmcf = {0},T_hmcb = {0};
T_LINE Linef = {8,0,0,0};
T_LINE Lineb = {9,0,0,0};
void Line_Analysis(uint16_t flag,T_LINE *L)
{
	uint8_t i = 0,pbuf[17] = {0};
	uint8_t f_mark= 0,b_mark = 0;
	if((flag&0xE187)==0XE187){
		L->wlineflag = 1;
	}
	for(i=1;i<17;i++)
	{
		if(flag&0x8000){
			pbuf[i] = 1;
			if(f_mark==0){
				f_mark = i;//mark down first point
			}
		}
		else{
			pbuf[i] = 0;
		}
		flag = flag<<1;
	}
	for(i=f_mark;i<17;i++)
	{
		if(pbuf[i]==0){
			b_mark=i;//mark down last point
			break;
		}
	}
	L->mid = (b_mark-f_mark)/2+f_mark;//find mid
	L->err = L->mid - L->tag;//find err
}

