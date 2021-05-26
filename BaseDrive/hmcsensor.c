#include "hmcsensor.h"

T_HMC T_hmcf = {0},T_hmcb = {0};
T_LINE Linef = {8,0,0,0};
T_LINE Lineb = {9,0,0,0};
void Line_Analysis(uint16_t flag,T_LINE *L)
{
	uint8_t i = 0,pbuf[17] = {0};
	uint8_t f_mark1= 0,b_mark1 = 0;
	uint8_t f_mark2= 0,b_mark2 = 0;
	uint8_t flag1 = 0,flag2 = 0;
	
	for(i=1;i<17;i++)
	{
		if(flag&0x8000){
			pbuf[i] = 1;
			if(f_mark1==0){
				f_mark1 = i;//mark down first point
			}
		}
		else{
			pbuf[i] = 0;
		}
		flag = flag<<1;
	}
	for(i=f_mark1;i<17;i++)
	{
		if(pbuf[i]==0){
			if(flag1==0){
				b_mark1=i;//mark down last point
				flag1 = 1;
			}
		}
		else{
			if(i==16&&b_mark1==0&&flag1==0){
				b_mark1 = 16;
				flag1 = 1;
				f_mark2 = 0,b_mark2 = 0;
			}
		}
	}
	for(i=b_mark1+1;i<17;i++)
	{
		if(pbuf[i]!=0){
			if(f_mark2==0){
				f_mark2 = i;
			}
		}
		else if(i==16&&f_mark2==0){
			f_mark2 = 0,b_mark2 = 0;
		}
	}
	for(i=f_mark2+1;i<17;i++)
	{
		if(pbuf[i]==0){
			if(b_mark2==0){
				b_mark2 = i;
				break;
			}
		}
		else if(i==16&&b_mark2==0){
			b_mark2 = 16;
		}
	}
	L->mid = (b_mark1-f_mark1)/2+f_mark1;//find mid
	L->num = (b_mark1-f_mark1+1);
	if(L->tag==8&&L->num>=6&&L->num<14){
		L->mid = 8;
	}
	if(f_mark2!=0&&b_mark2!=0){
		L->mid = 8;
		L->twol = 1;//探测开叉，不连续
	}
	else{
		L->twol = 0;
	}
	L->last_wlineflag = L->wlineflag;
	if(L->num>=12){
		L->mid = 8;
		L->wlineflag = 1;
	}
	else{
		L->wlineflag = 0;
	}
	L->err = L->mid - L->tag;//find err
}

