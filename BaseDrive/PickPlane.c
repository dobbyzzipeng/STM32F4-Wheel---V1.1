#include "PickPlane.h"
#include "can2.h"

int16_t UDMotor0_Sp = 0,UDMotor1_Sp = 0;
int16_t PPMotor0_Sp = 0,PPMotor1_Sp = 0;
int16_t CAMotor0_Sp = 0,CAMotor1_Sp = 0;
void Send_UpDownMotor_Speed(uint16_t id,int16_t sp0,int16_t sp1)
{
	uint8_t canbuf[8] = {0};
	canbuf[0] = sp0>>8;
	canbuf[1] = sp0;
	canbuf[2] = sp1>>8;
	canbuf[3] = sp1;
	CAN2_TX_PACKET(id,canbuf,8);
}

void Send_PushPullMotor_Speed(uint16_t id,int16_t sp0,int16_t sp1)
{
	uint8_t canbuf[8] = {0};
	canbuf[0] = sp0>>8;
	canbuf[1] = sp0;
	canbuf[2] = sp1>>8;
	canbuf[3] = sp1;
	
	CAN2_TX_PACKET(id,canbuf,8);
}

void Send_CatchMotor_Speed(uint16_t id,int16_t sp0,int16_t sp1)
{
	uint8_t canbuf[8] = {0};
	canbuf[0] = sp0>>8;
	canbuf[1] = sp0;
	canbuf[2] = sp1>>8;
	canbuf[3] = sp1;
	
	CAN2_TX_PACKET(id,canbuf,8);
}

void Stop_All_Bldcmotor(void)
{
	UDMotor0_Sp = 0;UDMotor1_Sp = 0;
	PPMotor0_Sp = 0;PPMotor1_Sp = 0;
	CAMotor0_Sp = 0;CAMotor1_Sp = 0;
}

