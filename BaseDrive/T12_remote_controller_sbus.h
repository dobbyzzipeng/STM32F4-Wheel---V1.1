#ifndef _T12_REMOTE_CONTROLLER_SBUS_H
#define _T12_REMOTE_CONTROLLER_SBUS_H
#include "stm32f4xx.h"
/* ----------------------- RC Channel Definition---------------------------- */
/* ----------------------- Data Struct ------------------------------------- */
#define USE_SBUS  1//use futaba remote controller
#define DEBUG_PC  0

#define USART2_MAX_RECV_LEN		128//最大接收缓存字节数
#define USART2_MAX_SEND_LEN		128//最大发送缓存字节数

extern uint8_t  USART2_RX_BUF[USART2_MAX_RECV_LEN];
extern uint8_t  USART2_TX_BUF[USART2_MAX_SEND_LEN];
extern uint16_t usart2_rx_num;
extern uint8_t  usart2_rx_data_flag;

void usart2_init(unsigned long int baudrate);
void USART2_DMA_Send(uint8_t *pbuffer, uint32_t size);
void send_data_dma_u2(uint8_t data[100],uint8_t num);
uint16_t USART2_DMA_RX_LEN(DMA_Stream_TypeDef* DMAy_Streamx,uint16_t BufSize);

/***********SBUS版本*********/
#define	UP_STATE   0X116//278
#define	MID_STATE  0X3EA//1002
#define	DOWN_STATE 0X6B3//1715
#define	BUTTON_OFF 0X116
#define	BUTTON_ON  0X6B3

typedef struct{
	uint16_t CH_X2;//282~1002~1722
	uint16_t CH_Y2;
	uint16_t CH_Y1;
	uint16_t CH_X1;
	uint16_t CH_Y3;
	uint16_t CH_X3;
	
	uint16_t FUN_A_BUTTON;
	uint16_t FUN_B_BUTTON;
	uint16_t FUN_C_BUTTON;
	uint16_t FUN_D_BUTTON;
	
	uint16_t SWITCH_E;
	uint16_t SWITCH_F;
	uint16_t SWITCH_G;
	uint16_t SWITCH_H;
}T_T12SBUS_DATA;//美国手

extern T_T12SBUS_DATA T12SBUS_DATA;
void T12SBUS_remote_controller_prase(uint8_t buf[100]);

#endif
