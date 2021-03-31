/*********************************************************************************************************
*
*	模块名称 : 串口
*	文件名称 : bsp_uart.h
*	版    本 : V1.0
*	说    明 : C文件
*	1)采用DMA不定长发送，DMA的不定长接收
*********************************************************************************************************
*/

#ifndef _BSP_USART_H_
#define _BSP_USART_H_
#include "sys.h"
#ifdef __cplusplus
 extern "C" {
#endif

/************************************************************************************************/
/**************************************** 接口分割线 ********************************************/
/************************************************************************************************/
#define UART1_TX_SIZE  128
#define USART1_REC_LEN 128
extern uint8_t USART1_RX_BUF[USART1_REC_LEN];	/* 接收缓冲区 */
extern uint16_t USART1_RX_STA;
extern uint8_t USART1_TX_BUF[UART1_TX_SIZE];

void bsp_InitUart(int badu);
void UsartDMA_Init(void);
void USART1_DMA_Send(uint8_t *pbuffer, uint32_t size);
void u1_printf(char* fmt,...);
uint16_t USART1_DMA_RX_LEN(DMA_Stream_TypeDef* DMAy_Streamx,uint16_t BufSize);
void UsartDMAIRQ(void);
void send_data_dma_u1(uint8_t data[100],uint8_t num);
#ifdef __cplusplus
}
#endif
#endif

