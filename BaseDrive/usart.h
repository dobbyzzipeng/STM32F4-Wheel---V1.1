#ifndef __usart_h__
#define __usart_h__
#include "stm32f4xx.h"
#include <stdio.h>

#define USART4_MAX_RECV_LEN		128//�����ջ����ֽ���
#define USART4_MAX_SEND_LEN		128//����ͻ����ֽ���

#define USART3_MAX_RECV_LEN		256//�����ջ����ֽ���
#define USART3_MAX_SEND_LEN		128//����ͻ����ֽ���

#define USART5_MAX_RECV_LEN		128//�����ջ����ֽ���
#define USART5_MAX_SEND_LEN		256//����ͻ����ֽ���

extern uint8_t  USART4_RX_BUF[USART4_MAX_RECV_LEN];
extern uint8_t  USART4_TX_BUF[USART4_MAX_SEND_LEN];

extern uint8_t  USART3_RX_BUF[USART3_MAX_RECV_LEN];
extern uint8_t  USART3_TX_BUF[USART3_MAX_SEND_LEN];

#define USART6_MAX_SEND_LEN 128
#define USART6_MAX_RECV_LEN 128
extern __align(8) uint8_t USART6_TX_BUF[USART6_MAX_SEND_LEN];//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�  	  	
extern __align(8) uint8_t USART6_RX_BUF[USART6_MAX_RECV_LEN];//���ջ���,���USART3_MAX_RECV_LEN���ֽ�.

void usart6_init(unsigned long int baudrate);
void usart3_init(unsigned long int baudrate);
void usart4_init(unsigned long int baudrate);
void USART3_DMA_Send(uint8_t *pbuffer, uint32_t size);
void send_data_dma_u3(uint8_t data[100],uint8_t num);
uint16_t USART3_DMA_RX_LEN(DMA_Stream_TypeDef* DMAy_Streamx,uint16_t BufSize);
void USART6_DMA_Send(uint8_t *pbuffer, uint32_t size);
void send_data_dma_u6(uint8_t data[100],uint8_t num);
void send_data_dma_u4(uint8_t data[100],uint8_t num);
uint16_t USART4_DMA_RX_LEN(DMA_Stream_TypeDef* DMAy_Streamx,uint16_t BufSize);
void u3_printf(char* fmt,...);
uint16_t USART6_DMA_RX_LEN(DMA_Stream_TypeDef* DMAy_Streamx,uint16_t BufSize);

#define  SERVER_485_TX()	  	GPIO_SetBits(GPIOC,GPIO_Pin_8)
#define	 SERVER_485_RX()	    GPIO_ResetBits(GPIOC,GPIO_Pin_8)

typedef struct{
	uint8_t sof1_flag;
	uint8_t sof2_flag;
	uint8_t sof3_flag;
	uint8_t sof4_flag;
	
	uint8_t sof5_flag;
	uint8_t sof6_flag;
	uint8_t fun;
	uint8_t	crc;
	
	uint8_t eof;
	uint8_t rx_flag;
	uint8_t over_cnt;
	uint8_t unlink_flag;
	
	uint16_t len;
	uint16_t rx_cnt;
}T_USART_REV;
extern T_USART_REV U3_rev,U4_rev,U5_rev,U6_rev;

void reset2isp(void);
void server_485_init(void);
void usart5_init(unsigned long int baudrate);
void send_data_dma_u5(uint8_t data[100],uint8_t num);
void u5_printf(char* fmt,...);
uint16_t USART5_DMA_RX_LEN(DMA_Stream_TypeDef* DMAy_Streamx,uint16_t BufSize);

#endif // __usart_h__
