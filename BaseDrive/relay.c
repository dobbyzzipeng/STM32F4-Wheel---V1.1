#include "relay.h"
#include "bsp_delay.h"
#include "data_analysis.h"

uint8_t Step_enable_flag=0;

void Step_motor_control(void)
{
	if(Step_enable_flag)//�򿪲������
	{
		DIR_ON();//��ת
		PULSE_TOGGLE();
	}
	else //�رղ������
	{
		PULSE_ON();//close pulse
	}
}
