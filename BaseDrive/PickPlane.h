#ifndef PICKPLANE_H_
#define PICKPLANE_H_
#include "stdint.h"

#define UDMOTOR_ID 0X101
#define PPMOTOR_ID 0X102
#define CAMOTOR_ID 0X103

extern int16_t UDMotor0_Sp,UDMotor1_Sp;
extern int16_t PPMotor0_Sp,PPMotor1_Sp;
extern int16_t CAMotor0_Sp,CAMotor1_Sp;
void Send_UpDownMotor_Speed(uint16_t id,int16_t sp0,int16_t sp1);
void Send_PushPullMotor_Speed(uint16_t id,int16_t sp0,int16_t sp1);
void Send_CatchMotor_Speed(uint16_t id,int16_t sp0,int16_t sp1);
void Stop_All_Bldcmotor(void);
void Catch_Motor_Open(void);
void Catch_Motor_Close(void);
void Catch_Motor_Stop(void);
void PP_Motor_Push(void);
void PP_Motor_Pull(void);
void PP_Motor_Stop(void);
void UD_Motor_Up(void);
void UD_Motor_Down(void);
void UD_Motor_Stop(void);
#endif

