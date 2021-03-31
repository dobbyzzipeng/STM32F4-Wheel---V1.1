#ifndef _ANCHOR_H_
#define _ANCHOR_H_
#include "stdint.h"
#include "sys.h"
#define ANCHOR_m  145000   //每米多少值

#define ANCHOR_PWR PCout(9)
//#define ANCHOR_PWR PDout(9)
#define ANCHOR_DIR PDout(8)
#define ANCHOR_flag   PDin(6)//行程开关
#define LIAO_flag   PDin(7)//光电开关

extern int32_t anchor_pulse;
void Anchor_Gpio_Init(void);
void ANCHOR_control(void);
void Exti_Gpio_Init(void);
void ANCHOR_PWR_ON(void);
void ANCHOR_PWR_OFF(void);
void ANCHOR_DIR_ON(void);
void ANCHOR_DIR_OFF(void);
void ANCHOR_RUN_CW(void);
void ANCHOR_RUN_CCW(void);
void ANCHOR_STOP(void);

#endif 

