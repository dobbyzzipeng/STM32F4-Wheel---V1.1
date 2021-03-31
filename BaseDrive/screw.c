#include "screw.h"
#include "pwm.h"

#define abs(x) ((x>0)? x :(-x))//ȡ����ֵ
uint16_t screw_rpm = 0;
uint8_t Screw_zreo_flag=0;
void Screw_gpio_config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11|GPIO_Pin_12;	//ѡ��Ҫ�õ�GPIO����		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��������Ϊ��ͨ���ģʽ		
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//��������Ϊ�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//���������ٶ�Ϊ100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//��������Ϊ���� 		 
	GPIO_Init(GPIOE, &GPIO_InitStructure);//���ÿ⺯������ʼ��GPIO
	GPIO_ResetBits(GPIOE,GPIO_Pin_11|GPIO_Pin_12);
}


void Screw_Speed_Set(uint16_t pwmval)
{
	Screw_DIR_OFF();
	if(pwmval>0){
		Screw_EN_ON();
	}
	else{
		Screw_EN_OFF();
	}
	if(pwmval>PWM_MAX){
		pwmval = PWM_MAX;
	}
	SCREW_PWM=pwmval;
}

