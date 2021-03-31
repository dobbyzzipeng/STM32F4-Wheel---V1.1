#ifndef _LED_H_
#define _LED_H_
#include "stm32f4xx.h"

/*407VET6*/
#define  LED2_ON()	  	GPIO_SetBits(GPIOE,GPIO_Pin_2)
#define	 LED2_OFF()	    GPIO_ResetBits(GPIOE,GPIO_Pin_2)
#define	 LED2_TOGGLE()	GPIO_ToggleBits(GPIOE,GPIO_Pin_2)

#define RED_WARM_LED_ON()         	GPIO_SetBits(GPIOE,GPIO_Pin_6)
#define RED_WARM_LED_OFF()        	GPIO_ResetBits(GPIOE,GPIO_Pin_6)
#define RED_WARM_LED_TOGGLE()       GPIO_ToggleBits(GPIOE,GPIO_Pin_6)
#define FIRE_ON()         			GPIO_SetBits(GPIOD,GPIO_Pin_4)
#define FIRE_OFF()        			GPIO_ResetBits(GPIOD,GPIO_Pin_4)
#define FIRE_TOGGLE()      			GPIO_ToggleBits(GPIOD,GPIO_Pin_4)
#define Big_LED_ON()         		GPIO_SetBits(GPIOE,GPIO_Pin_4)
#define Big_LED_OFF()        		GPIO_ResetBits(GPIOE,GPIO_Pin_4)
#define Big_LED_TOGGLE()       		GPIO_ToggleBits(GPIOE,GPIO_Pin_4)

void Warm_Led_Init(void);
void LED_GPIO_Config(void);
void Big_LED_Init(void);
void LED_task(void);
#endif
