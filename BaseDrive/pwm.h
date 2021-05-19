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
void TIM_Counter_init(void);
void TIM_PWM_init(void);

#endif
