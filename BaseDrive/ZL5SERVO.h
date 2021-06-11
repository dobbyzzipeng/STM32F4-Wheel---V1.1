#ifndef _ZL5SERVO_H_
#define _ZL5SERVO_H_
#include "stdint.h"

//主机地址
#define MAIN_ADDR	0XFF

//功能码
#define MAIN_READ	0X03//主机读，从机应答
#define MAIN_WRITE	0X06//主机写，从机应答
#define EMERGENCY_STOOP 0X10//紧急停止
//寄存器地址
#define MODBUS_EN	0X00
#define DRIVER_OUT_EN 0X01//驱动器输出使能
#define SPEED_ADDR	0X02
#define SYS_CUR		0X0F//系统电流
#define MOTOR_SPEED	0X10//电机当前速度
#define SYS_VOL		0X11//系统电压
#define SYS_TEM		0X12//系统温度
#define ABS_POS_L	0X16//绝对位置低十六位
#define ABS_POS_H	0X17//绝对位置高十六位
#define SPECIAL_FUN 0X19//特殊功能
#define ABS_POS_CAN_L	0X1A//绝对位置低十六位 CAN缓存
#define ABS_POS_CAN_H	0X1B//绝对位置高十六位 CAN缓存
#define OVER_CUR	0X1D//过流保护
#define CANOPEN_POS_ID	0X600

extern int16_t motor1_speed,motor2_speed,motor3_speed,motor4_speed;
extern int32_t omgset_pos1,omgset_pos2,omgset_pos3,omgset_pos4;

void Set_Motor_Speed(uint16_t motorid,int16_t speed);
void Set_Motor_Postion(uint16_t motorid,int32_t pos);
void Set_Motor_Stop(uint16_t motorid);
void Set_Motor_Modbus(uint16_t motorid,uint8_t en);
void Enable_All_Motor_Modbus(void);
void Send_chassic_speed(int16_t s1,int16_t s2,int16_t s3,int16_t s4);
void Send_chassic_omgset(int32_t w1,int32_t w2,int32_t w3,int32_t w4);

void Config_Pos_Mode(uint16_t id);
void Clear_Errcode(uint16_t id);
void Set_Driver_Ready(uint16_t id);
void Set_Driver_Pos(uint16_t id,int pos);
void Set_Driver_Pos_Speed(uint16_t id,int sp);
void Start_Driver_Pos_Ctr(uint16_t id);
void Read_Driver_Pos_Status(uint16_t id);

void Config_Vel_Mode(uint16_t id);
void Set_Driver_Ready_Vel(uint16_t id);
void Set_Driver_Vel(uint16_t id,int vel);
void Stop_All_Chassicmotor(void);
void Read_Motor_Status(uint8_t id);
#endif

