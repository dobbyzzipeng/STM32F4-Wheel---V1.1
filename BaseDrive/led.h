#ifndef _LED_H_
#define _LED_H_
#include "stm32f4xx.h"

/*407VET6*/
#define  LED2_ON()	  	GPIO_SetBits(GPIOE,GPIO_Pin_2)
#define	 LED2_OFF()	    GPIO_ResetBits(GPIOE,GPIO_Pin_2)
#define	 LED2_TOGGLE()	GPIO_ToggleBits(GPIOE,GPIO_Pin_2)

#define Relay1_ON()         		GPIO_SetBits(GPIOD,GPIO_Pin_6)
#define Relay1_OFF()        		GPIO_ResetBits(GPIOD,GPIO_Pin_6)
#define Relay2_ON()         		GPIO_SetBits(GPIOD,GPIO_Pin_7)
#define Relay2_OFF()        		GPIO_ResetBits(GPIOD,GPIO_Pin_7)

#define READ_EXTI1()	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_10)
#define READ_EXTI2()	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_12)

void LED_GPIO_Config(void);
void Relay_Init(void);
void Exti_Gpio_Init(void);

#endif
