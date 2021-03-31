#include "relay.h"
#include "bsp_delay.h"
#include "data_analysis.h"

uint8_t Step_enable_flag=0;

void Step_motor_control(void)
{
	if(Step_enable_flag)//打开步进电机
	{
		DIR_ON();//正转
		PULSE_TOGGLE();
	}
	else //关闭步进电机
	{
		PULSE_ON();//close pulse
	}
}
