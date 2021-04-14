#include "ZL5SERVO.h"
#include "can1.h"
#include "bsp_delay.h"

int16_t motor1_speed = 0,motor2_speed = 0,motor3_speed = 0,motor4_speed = 0;
int32_t omgea1 = 0,omgset_pos1 = 0,omgset_pos2 = 0,omgset_pos3 = 0,omgset_pos4 = 0;
//底盘轮子ID:1~4,舵轮ID：5~8.
//speed+:正看轮子逆时针转
//speed-:正看轮子顺时针转
void Set_Motor_Speed(uint16_t motorid,int16_t speed)
{
	uint8_t buf[8] = {MAIN_ADDR,MAIN_WRITE,SPEED_ADDR,01,0,0,0,0};
	
	buf[5] = speed>>8;
	buf[4] = speed;
	
	CAN1_TX_PACKET(motorid,buf,8);
}
//pos+:顶部看舵机顺时针转
//pos-:顶部看舵机逆时针转
void Set_Motor_Postion(uint16_t motorid,int32_t pos)
{
	uint8_t buf[8] = {MAIN_ADDR,MAIN_WRITE,ABS_POS_L,02,0,0,0,0};
	
	buf[4] = pos;
	buf[5] = pos>>8;
	buf[6] = pos>>16;
	buf[7] = pos>>24;
	
	CAN1_TX_PACKET(motorid,buf,8);
}

void Set_Motor_Modbus(uint16_t motorid,uint8_t en)
{
	uint8_t buf[5] = {MAIN_ADDR,MAIN_WRITE,MODBUS_EN,01,en};
	CAN1_TX_PACKET(motorid,buf,5);
}

void Set_Motor_Stop(uint16_t motorid)
{
	uint8_t buf[2] = {MAIN_ADDR,EMERGENCY_STOOP};
	CAN1_TX_PACKET(motorid,buf,2);
}

void Enable_All_Motor_Modbus(void)
{
	Config_Pos_Mode(0X05);
	delay_ms(10);
	Clear_Errcode(0X05);
	delay_ms(10);
	Set_Driver_Ready(0X05);
	delay_ms(10);
//	Set_Driver_Pos(0x05,0);
//	delay_ms(10);
//	Start_Driver_Pos_Ctr(0x05);
//	delay_ms(10);
//	Read_Driver_Pos_Status(0X05);
//	delay_ms(10);		
	
	Config_Pos_Mode(0X06);
	delay_ms(10);
	Clear_Errcode(0X06);
	delay_ms(10);
	Set_Driver_Ready(0X06);
	delay_ms(10);
//	Set_Driver_Pos(0x06,0);
//	delay_ms(10);
//	Start_Driver_Pos_Ctr(0x06);
//	delay_ms(10);
//	Read_Driver_Pos_Status(0X06);
//	delay_ms(10);
	
	Config_Pos_Mode(0X07);
	delay_ms(10);
	Clear_Errcode(0X07);
	delay_ms(10);
	Set_Driver_Ready(0X07);
	delay_ms(10);
//	Set_Driver_Pos(0x07,0);
//	delay_ms(10);
//	Start_Driver_Pos_Ctr(0x07);
//	delay_ms(10);
//	Read_Driver_Pos_Status(0X07);
//	delay_ms(10);
	
	Config_Pos_Mode(0X08);
	delay_ms(10);
	Clear_Errcode(0X08);
	delay_ms(10);
	Set_Driver_Ready(0X08);
	delay_ms(10);
//	Set_Driver_Pos(0x08,0);
//	delay_ms(10);
//	Start_Driver_Pos_Ctr(0x08);
//	delay_ms(10);
//	Read_Driver_Pos_Status(0X08);
//	delay_ms(10);
	
	Config_Vel_Mode(0X01);
	delay_ms(10);
	Clear_Errcode(0X01);
	delay_ms(10);
	Set_Driver_Ready_Vel(0X01);
	delay_ms(10);
	
	Config_Vel_Mode(0X02);
	delay_ms(10);
	Clear_Errcode(0X02);
	delay_ms(10);
	Set_Driver_Ready_Vel(0X02);
	delay_ms(10);
	
	Config_Vel_Mode(0X03);
	delay_ms(10);
	Clear_Errcode(0X03);
	delay_ms(10);
	Set_Driver_Ready_Vel(0X03);
	delay_ms(10);		
			
	Config_Vel_Mode(0X04);
	delay_ms(10);
	Clear_Errcode(0X04);
	delay_ms(10);
	Set_Driver_Ready_Vel(0X04);
	delay_ms(10);
}

void Send_chassic_speed(int16_t s1,int16_t s2,int16_t s3,int16_t s4)
{
	Set_Motor_Speed(0x01,s1);
	delay_ms(10);
	Set_Motor_Speed(0x02,s2);
	delay_ms(10);
	Set_Motor_Speed(0x03,s3);
	delay_ms(10);
	Set_Motor_Speed(0x04,s4);
	delay_ms(10);
}

void Send_chassic_omgset(int32_t w1,int32_t w2,int32_t w3,int32_t w4)
{
	Set_Motor_Postion(0x05,w1);
	delay_ms(10);
	Set_Motor_Postion(0x06,w2);
	delay_ms(10);
	Set_Motor_Postion(0x07,w3);
	delay_ms(10);
	Set_Motor_Postion(0x08,w4);
	delay_ms(10);
}

/************CANOPEN  POS****************/
void Config_Pos_Mode(uint16_t id)
{
	uint8_t buf[8] = {0X2F,0X60,0X60,0X01,0X01,0X00,0X00,0X00};
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf,8);
}

void Clear_Errcode(uint16_t id)
{
	uint8_t buf[8] = {0X2B,0X40,0X60,0X01,0X80,0X00,0X00,0X00};
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf,8);
}

void Set_Driver_Ready(uint16_t id)
{
	uint8_t buf[8] = {0X2B,0X40,0X60,0X01,0X06,0X00,0X00,0X00};
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf,8);
	delay_ms(10);
	uint8_t buf1[8] = {0X2B,0X40,0X60,0X01,0X07,0X00,0X00,0X00};
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf1,8);
	delay_ms(10);
}

void Set_Driver_Pos(uint16_t id,int pos)
{
	uint8_t buf[8] = {0X23,0X7A,0X60,0X01,0X00,0X00,0X00,0X00};
	buf[4] = pos&0xff;
	buf[5] = (pos>>8)&0xff;
	buf[6] = (pos>>16)&0xff;
	buf[7] = (pos>>24)&0xff;
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf,8);
}

void Set_Driver_Pos_Speed(uint16_t id,int sp)
{
	uint8_t buf[8] = {0X23,0X81,0X60,0X01,0X00,0X00,0X00,0X00};
	buf[4] = sp&0xff;
	buf[5] = (sp>>8)&0xff;
	buf[6] = (sp>>16)&0xff;
	buf[7] = (sp>>24)&0xff;
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf,8);
}

void Start_Driver_Pos_Ctr(uint16_t id)
{
//	uint8_t buf[8] = {0X2B,0X40,0X60,0X01,0X1F,0X00,0X00,0X00};
	uint8_t buf[8] = {0X2B,0X40,0X60,0X01,0X3F,0X00,0X00,0X00};
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf,8);
}

void Read_Driver_Pos_Status(uint16_t id)
{
	uint8_t buf[8] = {0X40,0X41,0X60,0X01,0X00,0X00,0X00,0X00};
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf,8);
}
/************CANOPEN  VEL****************/
void Config_Vel_Mode(uint16_t id)
{
	uint8_t buf[8] = {0X2F,0X60,0X60,0X01,0X03,0X00,0X00,0X00};
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf,8);
}

void Set_Driver_Ready_Vel(uint16_t id)
{
	uint8_t buf[8] = {0X2B,0X40,0X60,0X01,0X06,0X00,0X00,0X00};
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf,8);
	delay_ms(10);
	uint8_t buf1[8] = {0X2B,0X40,0X60,0X01,0X07,0X00,0X00,0X00};
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf1,8);
	delay_ms(10);
	uint8_t buf2[8] = {0X2B,0X40,0X60,0X01,0X0F,0X00,0X00,0X00};
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf2,8);
	delay_ms(10);
}

void Set_Driver_Vel(uint16_t id,int vel)
{
	uint8_t buf[8] = {0X23,0XFF,0X60,0X01,0X00,0X00,0X00,0X00};
	buf[4] = vel&0xff;
	buf[5] = (vel>>8)&0xff;
	buf[6] = (vel>>16)&0xff;
	buf[7] = (vel>>24)&0xff;
	CAN1_TX_PACKET(id+CANOPEN_POS_ID,buf,8);
}


