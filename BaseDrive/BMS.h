#ifndef _BMS_H_
#define _BMS_H_
#include "sys.h"
//优先级 + 数据ID + 上位机地址+ BMS地址
#define PROPOTY 0x18	//优先级

#define BMS_ADDR 0x01	//BMS地址
#define PC_ADDR	0x40//上位机地址

#define GET_TOTAL_SOC 0x90		//总电流
#define GET_VOL 0x91		//单体最高最低电压
#define GET_TEM 0x92		//单体最高最低温度
#define GET_MOS 0x93		//充放电、MOS状态
#define GET_MSG 0x94		//状态信息
#define GET_CELL_VOL 0x95		//单体电压
#define GET_CELL_TEM 0x96		//单体温度
#define GET_JUNHENG 0x97		//单体均衡状态
#define GET_ERROR 0x98		//电池故障状态

typedef struct{

	float Voltage;
	float Current;
	float Soc;
  uint8_t state;
}Battery;
extern Battery Battery_Msg;

void Bms_Get_Soc(void);
#endif

