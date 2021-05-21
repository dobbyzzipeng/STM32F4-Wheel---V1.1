#include "comunication.h"
#include "bsp_usart.h"
#include "usart.h"
#include "bms.h"
#include "crc8.h"
#include "can2.h"

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
void NX_Data_prase(uint8_t buf[])
{
	uint8_t len,crc;
	len = buf[2];
	if(len<2) return;
	crc = buf[len-2];
	if(buf[0]==0X71&buf[1]==0X08 /*&& CRC8_Table_Check(buf,len-2,crc)*/)
	{
		CV.cv_cmd = buf[3];
		CV.rgb_cmd = buf[4];
		CV.eject_cmd = buf[5];
		CV.agv_spx = buf[6]<<8|buf[7];
		CV.agv_spy = buf[8]<<8|buf[9];
		CV.agv_spw = buf[10]<<8|buf[11];
		CV.pick_spcatch = buf[12]<<8|buf[13];
		CV.pick_sppp = buf[14]<<8|buf[15];
		CV.pick_spupdown = buf[16]<<8|buf[17];
	}
}

extern uint8_t g_agv_work_mode;
void NX_Data_Return(void)
{
	static uint8_t runcnt = 0;
	uint8_t cnt = 0;
	uint8_t Rbuf[64] = {0};
	runcnt++;
	if(runcnt>10){
		runcnt = 0;
		Rbuf[cnt++] = 0X71;
		Rbuf[cnt++] = 0X07;
		cnt++;
		Rbuf[cnt++] = g_agv_work_mode;
		Rbuf[cnt++] = 0;
		Rbuf[cnt++] = 0;
		Rbuf[cnt++] = Eject.catch0;
		Rbuf[cnt++] = Eject.catch1;
		Rbuf[cnt++] = Eject.puspul0;
		Rbuf[cnt++] = Eject.puspul1;
		Rbuf[cnt++] = Eject.updown0;
		Rbuf[cnt++] = Eject.updown1;
		Rbuf[cnt++] = 0;
		
		Rbuf[cnt++] = 0X08;
		Rbuf[cnt++] = 0X19;
		Rbuf[2] = cnt;
		Rbuf[cnt-2] = CRC8_Table(Rbuf,cnt-2);//º∆À„–£—È
		send_data_dma_u1(Rbuf,cnt);
	}
}
