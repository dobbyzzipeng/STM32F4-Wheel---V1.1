#ifndef __SPI_H
#define __SPI_H
#include "sys.h"


void SPI1_Init(void);			 //��ʼ��SPI1��
void SPI1_SetSpeed(uint8_t SpeedSet); //����SPI1�ٶ�   
uint8_t SPI1_ReadWriteByte(uint8_t TxData);//SPI1���߶�дһ���ֽ�
uint16_t SPI1_ReadWriteWorld(uint16_t TxData);

#endif

