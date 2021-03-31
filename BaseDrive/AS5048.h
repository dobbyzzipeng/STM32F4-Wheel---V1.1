#ifndef __AS5048A_H
#define __AS5048A_H
#include "stdint.h"
#include "spi.h"
////////////////////////// 	

#define AS5048A_CS() PAout(4)

//寄存器地址
//#define CMD_ANGLE  0x3FFF //((1<<15) | (1<<14) | 0x3FFF)//角度信息

/***********************/
// 定义好附加偶校验位的各个寄存器读取指令
#define CMD_ANGLE        0x3fff
#define CMD_AGC          0x3ffd
#define CMD_MAG          0x3ffe
#define CMD_CLAER        0x4001
#define CMD_NOP          0xc000

//寄存器地址
////#define CMD_ANGLE  0x3FFF //((1<<15) | (1<<14) | 0x3FFF)//角度信息
//#define CMD_READ_MAG  0x3FFE//磁信息
//#define CMD_READ_DIAG  0x3FFD //诊断信息
////#define CMD_NOP  0x0000 //No operation dummy information
//#define CMD_CLEAR_ERROR  0x0001 //错误状态寄存器
//#define CMD_ProgramControl 0x0003
//#define CMD_OTPHigh 0x0016
//#define CMD_OTPLow 0x0017

//uint8_t Write_As5048A_ZeroPosition(void);
//uint16_t Read_As5048A_Value(uint16_t cmd);
//void Write_As5048A_Reg(uint16_t cmd,uint16_t value);
//uint16_t Read_As5048A_Reg(uint16_t cmd);
//uint8_t parity_even(uint16_t v);

struct as5048_data 
{
    uint8_t iserror;
    uint16_t value;
    uint16_t mag;
    uint8_t agc;
    double angle;
};

extern struct as5048_data Mag5048a_data;
void AS5048A_Init(void);
uint16_t SPI_Read5048Data(uint16_t TxData);
uint16_t ClearAndNop(void);
struct as5048_data CollectData(void);

#endif
