#ifndef __DAC_H
#define __DAC_H	 
#include "sys.h"	     			    

void Dac_Init(void);		//DAC通道初始化	 	 
void Dac_channel1_Set_Vol(u16 vol);	//设置通道1输出电压
void Dac_channel2_Set_Vol(u16 vol);
#endif

















