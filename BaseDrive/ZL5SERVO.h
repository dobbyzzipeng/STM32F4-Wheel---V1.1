#ifndef _ZL5SERVO_H_
#define _ZL5SERVO_H_
#include "stdint.h"

//������ַ
#define MAIN_ADDR	0XFF

//������
#define MAIN_READ	0X03//���������ӻ�Ӧ��
#define MAIN_WRITE	0X06//����д���ӻ�Ӧ��
#define EMERGENCY_STOOP 0X10//����ֹͣ
//�Ĵ�����ַ
#define MODBUS_EN	0X00
#define DRIVER_OUT_EN 0X01//���������ʹ��
#define SPEED_ADDR	0X02
#define SYS_CUR		0X0F//ϵͳ����
#define MOTOR_SPEED	0X10//�����ǰ�ٶ�
#define SYS_VOL		0X11//ϵͳ��ѹ
#define SYS_TEM		0X12//ϵͳ�¶�
#define ABS_POS_L	0X16//����λ�õ�ʮ��λ
#define ABS_POS_H	0X17//����λ�ø�ʮ��λ
#define SPECIAL_FUN 0X19//���⹦��
#define ABS_POS_CAN_L	0X1A//����λ�õ�ʮ��λ CAN����
#define ABS_POS_CAN_H	0X1B//����λ�ø�ʮ��λ CAN����
#define OVER_CUR	0X1D//��������
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

