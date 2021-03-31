#ifndef _RELAY_H_
#define _RELAY_H_
#include "stdint.h"
#include "sys.h"
#include "pwm.h"
//24V

#define DIR_OFF()    	    GPIO_SetBits(GPIOD,GPIO_Pin_12)
#define DIR_ON() 	    GPIO_ResetBits(GPIOD,GPIO_Pin_12)
#define PULSE_ON() 		GPIO_SetBits(GPIOD,GPIO_Pin_11)
#define PULSE_OFF() 	GPIO_ResetBits(GPIOD,GPIO_Pin_11)
#define MOTOR_PWR_ON() 		GPIO_SetBits(GPIOA,GPIO_Pin_6)
#define MOTOR_PWR_OFF() 	GPIO_ResetBits(GPIOA,GPIO_Pin_6)

void MOTOR_PWRER_OFF(void);
void MOTOR_PWRER_ON(void);
void DIR_TOGGLE(void);
void PULSE_TOGGLE(void);
void step_motor_init(void);
void pwr_init(void);

#define abs(x) ((x>0)?(x):(-x))
#define READ_ENC()   PBin(6)



extern uint8_t Thow_feed_flag;
extern uint8_t Step_enable_flag;
extern uint8_t Soup_feed_flag;
extern uint32_t targt_Step_time;
extern uint32_t Step_time;

void Exti_Gpio_Init(void);
void Encoder_Exti_Init(void);
void Thow_feed_contorl(void);
void Step_motor_control(void);

void Soup_contorl(void);
void Set_feed_Weight(uint8_t waypiont_Weight);
void Feed_Weight__contorl(void);
#endif
