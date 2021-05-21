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

void Catch_Motor_Open(void)
{
	CAMotor0_Sp = 200;//打开
	CAMotor1_Sp = -200;
}

void Catch_Motor_Close(void)
{
	CAMotor0_Sp = -200;//收紧
	CAMotor1_Sp = 200;
}

void Catch_Motor_Stop(void)
{
	CAMotor0_Sp = 0;
	CAMotor1_Sp = 0;
}

void PP_Motor_Push(void)
{
	PPMotor0_Sp = -200;//推
	PPMotor1_Sp = 200;
}

void PP_Motor_Pull(void)
{
	PPMotor0_Sp = 200;//拉
	PPMotor1_Sp = -200;
}

void PP_Motor_Stop(void)
{
	PPMotor0_Sp = 0;//拉
	PPMotor1_Sp = 0;
}

void UD_Motor_Up(void)
{
	UDMotor0_Sp = 400;
	UDMotor1_Sp = 400;
}

void UD_Motor_Down(void)
{
	UDMotor0_Sp = -400;
	UDMotor1_Sp = -400;
}

void UD_Motor_Stop(void)
{
	UDMotor0_Sp = 0;
	UDMotor1_Sp = 0;
}
