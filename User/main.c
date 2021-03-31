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
	bsp_InitUart(115200);//usart1 115200bps for debug
	RC_Init();
	CAN1_Configuration(0X01);
	TIM5_Init(2000-1,84-1);//1ms  SYS TIMER
	delay_ms(3000);
	u1_printf("sys init finish!\r\n");
}

#define DEAD_ZONE 80
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

const int32_t step = 16384;//14位编码器	204800
uint8_t chassic_ctr_cnt = 0;
void chassic_control_task(void)
{
	int16_t speed = 0;
	int16_t ch1 = 0,ch0 = 0;
	int32_t omgea1 = 0;
	float womg = 0;

	if(rc_data_flag!=0)
	{
		ch1 = Channel_1-1000;	ch0 = Channel_0-1000;
		
		if(myabs(ch0)>DEAD_ZONE && myabs(ch1)>DEAD_ZONE){
			womg = RAD_TO_DU*atan(ch0/ch1);
			
			if(ch0 > DEAD_ZONE && ch1 < -DEAD_ZONE){
				womg = 180 + womg;
				speed = myabs(ch0)+myabs(ch1);
			}
			else if(ch0 < -DEAD_ZONE && ch1 < -DEAD_ZONE){
				womg = womg - 180;
				speed = myabs(ch0)+myabs(ch1);
			}
			else{
				speed = ch0+ch1;
			}
		}
		else if(myabs(ch0)>DEAD_ZONE && myabs(ch1)<DEAD_ZONE){
			if(ch0>DEAD_ZONE){
				womg = 90;
				speed = -ch0-ch1;
			}
			else if(ch0<DEAD_ZONE){
				womg = -90;
				speed = ch0+ch1;
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
			omgea1 = 0;
		}
		else{
			motor1_speed = speed;
			motor2_speed = speed;
			motor3_speed = speed;
			motor4_speed = speed;
			omgea1 = womg*LSB;
		}
		
		
		omgset_pos1 =  omgea1;
		omgset_pos1 = -Limit(omgset_pos1,_90_ANGLE);
		omgset_pos2 =  omgea1;
		omgset_pos2 = -Limit(omgset_pos2,_90_ANGLE);
		omgset_pos3 =  omgea1;
		omgset_pos3 = -Limit(omgset_pos3,_90_ANGLE);
		omgset_pos4 =  omgea1;
		omgset_pos4 = -Limit(omgset_pos4,_90_ANGLE);
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


int main(void)
{
	bsp_init();
	Enable_All_Motor_Modbus();
	delay_ms(100);
	while(1)
	{
		chassic_control_task();
	}
}



