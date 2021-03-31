#ifndef __pwm_h__
#define __pwm_h__
//----------------------- Include files ------------------------//
#include "stm32f4xx_rcc.h"
#define PWM_MAX 1000

extern unsigned char Task_timer_flag;
extern volatile uint8_t g_cantxquene_index;
//----------------------- Extern function ----------------------//
//----------------------- Extern variable ----------------------//
extern uint32_t s_screw_cnt;
void EXTIX_Encoder_Init(void);
void TIM4_Encoder_Init(void);
void Encoder4_Get_CNT(int32_t *pulse);
void TIM5_Init(uint16_t arr,uint16_t psc);
void TIM1_PWM_Init(uint16_t arr,uint16_t psc);
void TIM3_PWM_Init(uint16_t arr,uint16_t psc);
void TIM12_PWM_Init(uint16_t arr,uint16_t psc);
void TIM8_PWM_Init(uint16_t arr,uint16_t psc);

#define SCREW_PWM 		TIM12->CCR2//ÂÝ¸Ë  
#define ZhengDong_PWM 	TIM1->CCR1//Õñ¶¯Æ÷  100HZ  0~1000
#define PAOLIAO_PWM 	TIM1->CCR3//Å×ÁÏ
#define SOUP_PUMP_PWM   TIM1->CCR4
#define ANCHOR_PWM   	TIM8->CCR4

#endif
