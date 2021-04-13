#include "config.h"
/***
修改更新记录见log.txt
参数配置详见config.h
//use 10CH_T8FB Remote controler
*/
//------------------------------------------------------------------------------
#define VERINFO_ADDR_BASE  (0x8009F00) // 版本信息在FLASH中的存放地址
const char Hardware_Ver[] __attribute__((at(VERINFO_ADDR_BASE + 0x00)))  = "Hardware: 1.0.0";
const char Firmware_Ver[] __attribute__((at(VERINFO_ADDR_BASE + 0x20)))  = "Firmware: 1.0.0";
const char Compiler_Date[] __attribute__((at(VERINFO_ADDR_BASE + 0x40))) = "Date: "__DATE__;
const char Compiler_Time[] __attribute__((at(VERINFO_ADDR_BASE + 0x60))) = "Time: "__TIME__;

void debug(void);
//------------------------------------------------------------------------------
void bsp_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
	LED_GPIO_Config();
	Relay_Init();
	bsp_InitUart(115200);//usart1 115200bps for debug
	RC_Init();
	Relay_ON(); 
	delay_ms(1000);
	CAN1_Configuration(0X01);
	CAN2_Configuration(0X501);
	delay_ms(3000);
	TIM5_Init(5000-1,84-1);//1ms  SYS TIMER
	u1_printf("sys init finish!\r\n");
}

#define DEAD_ZONE 150
#define LSB 2275.55f//外部电机轴转过1度，内部磁编码器要走的值
#define RAD_TO_DU 57.32f
#define RATIO	50
#define _90_ANGLE	204800

int Limit(int datain,int max)
{
	if(datain>max){
		datain = max;
	}
	else if(datain<-max){
		datain = -max;
	}
	return datain;
}
//this is for newtree
const int32_t step = 16384;//14位编码器	204800
uint8_t chassic_ctr_cnt = 0;
void chassic_control_task(void)
{
	int16_t speed = 0;
	int16_t ch1 = 0,ch0 = 0;
	int32_t omgea1 = 0,omgea2 = 0,omgea3 = 0,omgea4 = 0;
	float womg = 0;

	if(rc_data_flag!=0&&right_Switch==right_Switch_UP)//righ switch UP
	{
		ch1 = Channel_1-1000;	ch0 = Channel_0-1000;
		if(myabs(ch0)>DEAD_ZONE && myabs(ch1)<DEAD_ZONE){
			if(ch0>DEAD_ZONE){
				womg = ch0/8.0f;
				speed = ch0+ch1;
			}
			else if(ch0<-DEAD_ZONE){
				womg = ch0/8.0f;
				speed = -ch0-ch1;
			}
		}
		else if(myabs(ch0)<DEAD_ZONE && myabs(ch1)>DEAD_ZONE){
			womg = 0;
			speed = ch0+ch1;
		}
		
		if(myabs(Channel_3-1000)>DEAD_ZONE){
			speed = (Channel_3-1000);
			motor1_speed = -speed;
			motor2_speed = -speed;
			motor3_speed = speed;
			motor4_speed = speed;
			womg = 45;
			omgea1 = -womg*LSB;
			omgea2 = womg*LSB;
			omgea3 = -womg*LSB;
			omgea4 = womg*LSB;
		}
		else{
			motor1_speed = speed;
			motor2_speed = speed;
			motor3_speed = speed;
			motor4_speed = speed;
			omgea1 = womg*LSB;
			omgea2 = womg*LSB;
			omgea3 = womg*LSB;
			omgea4 = womg*LSB;
		}
		
		omgset_pos1 = Limit(omgea1,_90_ANGLE);
		omgset_pos2 = Limit(omgea2,_90_ANGLE);
		omgset_pos3 = Limit(omgea3,_90_ANGLE);
		omgset_pos4 = Limit(omgea4,_90_ANGLE);
		rc_data_flag = 0;
		debug();
	}
}

void debug(void)
{
	static uint8_t dbgcnt = 0;
	dbgcnt++;
	if(dbgcnt>10){
		u1_printf("ch0:%d ch1:%d sp:%d omg:%d\r\n",Channel_0,Channel_1,motor1_speed,omgset_pos1);
		dbgcnt = 0;
	}	
}

void Pick_Plane_Ctr_Task(void)
{
	int16_t s_ch0 = 0,s_ch1 = 0,s_ch2 = 0;
	if(rc_data_flag!=0&&right_Switch==right_Switch_DOWN)//righ switch Down
	{
		s_ch2 = Channel_2-1000,s_ch1 = Channel_1-1000; s_ch0 = Channel_0-1000;
		if(s_ch0>DEAD_ZONE)
		{
			CAMotor0_Sp = 100;
			CAMotor1_Sp = -100;
		}
		else if(s_ch0<-DEAD_ZONE)
		{
			CAMotor0_Sp = -100;
			CAMotor1_Sp = 100;
		}
		else 
		{
			CAMotor0_Sp = 0;
			CAMotor1_Sp = 0;
		}
		
		if(s_ch1>DEAD_ZONE)
		{
			PPMotor0_Sp = -100;
			PPMotor1_Sp = 100;
		}
		else if(s_ch1<-DEAD_ZONE)
		{
			PPMotor0_Sp = 100;
			PPMotor1_Sp = -100;
		}
		else 
		{
			PPMotor0_Sp = 0;
			PPMotor1_Sp = 0;
		}
		
		if(s_ch2>DEAD_ZONE)
		{
			UDMotor0_Sp = 100;
			UDMotor1_Sp = 100;
		}
		else if(s_ch2<-DEAD_ZONE)
		{
			UDMotor0_Sp = -100;
			UDMotor1_Sp = -100;
		}
		else 
		{
			UDMotor0_Sp = 0;
			UDMotor1_Sp = 0;
		}
		rc_data_flag = 0;
	}
}

int main(void)
{
	bsp_init();
	Enable_All_Motor_Modbus();
	delay_ms(100);
	while(1)
	{
		chassic_control_task();
		Pick_Plane_Ctr_Task();
	}
}



