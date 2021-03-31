#ifndef __SCREW_H
#define __SCREW_H 			   
#include <sys.h>

#define  MOTOR_SPEED_MAX   1000
#define  Screw_EN_ON()	  	GPIO_SetBits(GPIOE,GPIO_Pin_11)
#define	 Screw_EN_OFF()		GPIO_ResetBits(GPIOE,GPIO_Pin_11)
#define  Screw_DIR_ON()	  	GPIO_SetBits(GPIOE,GPIO_Pin_12)
#define	 Screw_DIR_OFF()	GPIO_ResetBits(GPIOE,GPIO_Pin_12)

void Screw_gpio_config(void);
void Screw_Speed_Set(uint16_t pwmval);
extern uint16_t screw_rpm;
extern uint8_t Screw_zreo_flag;
#endif





























