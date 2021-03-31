#ifndef __BSP_DELAY_H
#define __BSP_DELAY_H 			   
#include <sys.h>	  
//********************************************************************************
//修改说明
//无
////////////////////////////////////////////////////////////////////////////////// 	 
void delay_init(uint8_t SYSCLK);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
void delay_solft_ms(unsigned int t);
#endif





























