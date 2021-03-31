#ifndef __SPI_H
#define __SPI_H
#include "sys.h"


void SPI1_Init(void);			 //初始化SPI1口
void SPI1_SetSpeed(uint8_t SpeedSet); //设置SPI1速度   
uint8_t SPI1_ReadWriteByte(uint8_t TxData);//SPI1总线读写一个字节
uint16_t SPI1_ReadWriteWorld(uint16_t TxData);

#endif

