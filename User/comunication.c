#include "comunication.h"
#include "bsp_usart.h"
#include "usart.h"
#include "bms.h"



extern uint8_t g_firegas_falg,g_lamp_state,g_elevator_state,g_sys_state;
void Data_Upload(void)
{
	uint8_t cnt=0;
	uint8_t Rbuf[128] = {0};
	U_Data udata = {0};
	
	Rbuf[cnt++] = 0XF1;
	Rbuf[cnt++] = 0X01;
	cnt++;
	udata.fdata = Battery_Msg.Voltage;
	Rbuf[cnt++] = udata.buf[0];
	Rbuf[cnt++] = udata.buf[1];
	Rbuf[cnt++] = udata.buf[2];
	Rbuf[cnt++] = udata.buf[3];

	udata.fdata = Battery_Msg.Current;
	Rbuf[cnt++] = udata.buf[0];
	Rbuf[cnt++] = udata.buf[1];
	Rbuf[cnt++] = udata.buf[2];
	Rbuf[cnt++] = udata.buf[3];

	udata.fdata = Battery_Msg.Soc;
	Rbuf[cnt++] = udata.buf[0];
	Rbuf[cnt++] = udata.buf[1];
	Rbuf[cnt++] = udata.buf[2];
	Rbuf[cnt++] = udata.buf[3];
	
//	Rbuf[cnt++]	= g_firegas_falg;
//	Rbuf[cnt++]	= g_lamp_state;
//	Rbuf[cnt++]	= g_elevator_state;
//	Rbuf[cnt++]	= g_sys_state;
	
	Rbuf[cnt++] = 0X08;
	Rbuf[cnt++] = 0X19;
	Rbuf[2] = cnt;
	send_data_dma_u1(Rbuf,cnt);
}

T_CMD AGV_CMD = {0};
void DownLoad_prase(uint8_t buf[])
{
	if(buf[0]==0XF1&buf[1]==0X02){
		AGV_CMD.agv_charge = buf[3];
		AGV_CMD.agv_start = buf[4];
		AGV_CMD.agv_back = buf[5];
	}
}


T_CV CV = {0};
void NX_Dta_prase(uint8_t buf[])
{
	if(buf[0]==0XF1&buf[1]==0X02){
//		g_lamp_onoff = buf[4];
//		g_elevator_onoff = buf[5];
	}
}

